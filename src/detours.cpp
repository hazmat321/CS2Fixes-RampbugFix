/**
 * =============================================================================
 * CS2Fixes
 * Copyright (C) 2023-2024 Source2ZE
 * =============================================================================
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License, version 3.0, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "networkbasetypes.pb.h"
#include "usercmd.pb.h"
#include "cs_usercmd.pb.h"

#include "cdetour.h"
#include "module.h"
#include "addresses.h"
#include "detours.h"
#include "entity/ccsplayercontroller.h"
#include "entity/ccsplayerpawn.h"
#include "entity/cbasemodelentity.h"
#include "entity/cgamerules.h"
#include "entity/services.h"
#include "playermanager.h"
#include "igameevents.h"
#include "gameconfig.h"
#include "serversideclient.h"
#include "tracefilter.h"

#include <cmath>

#define VPROF_ENABLED
#include "tier0/vprof.h"

#include "tier0/memdbgon.h"

extern CGlobalVars *gpGlobals;
extern CGameEntitySystem *g_pEntitySystem;
extern CCSGameRules *g_pGameRules;

CUtlVector<CDetourBase *> g_vecDetours;

DECLARE_DETOUR(ProcessMovement, Detour_ProcessMovement);
DECLARE_DETOUR(TryPlayerMove, Detour_TryPlayerMove);
DECLARE_DETOUR(CategorizePosition, Detour_CategorizePosition);

// --------------------------------------------------------------------------------------
// Legacy acceleration integration (inline, uses surf settings + sv_maxvelocity clamp)
// --------------------------------------------------------------------------------------
struct LegacyAccelParams
{
    float sv_accelerate   = 10.0f;   // surf: snappy ground accel
    float sv_airaccelerate= 150.0f;  // surf: strong air control
    float sv_friction     = 5.2f;    // surf: a bit more braking (use 4.0f for more glide)
    float sv_stopspeed    = 80.0f;   // surf: slightly above classic 75
    float sv_maxvelocity  = 10000.0f;// surf: high ceiling for overall velocity
    bool  enable          = true;    // master toggle
};

static LegacyAccelParams g_LegacyAccelParams {};

static inline void Legacy_ApplyFriction(const LegacyAccelParams& p, float dt, Vector& vel)
{
    float speed = vel.Length2D();
    if (speed <= 0.0f) return;

    float control = speed < p.sv_stopspeed ? p.sv_stopspeed : speed;
    float drop = control * p.sv_friction * dt;

    float newSpeed = speed - drop;
    if (newSpeed < 0.0f) newSpeed = 0.0f;
    if (newSpeed != speed)
    {
        float scale = (speed > 0.0f) ? (newSpeed / speed) : 0.0f;
        vel.x *= scale;
        vel.y *= scale;
        // leave vel.z untouched here
    }
}

static inline void Legacy_AccelerateGround(const LegacyAccelParams& p, float dt, const Vector& wishdir, float wishspeed, Vector& velXY)
{
    float currentSpeed = DotProduct(velXY, wishdir);
    float addSpeed = wishspeed - currentSpeed;
    if (addSpeed <= 0.0f) return;

    float accelSpeed = p.sv_accelerate * wishspeed * dt;
    if (accelSpeed > addSpeed) accelSpeed = addSpeed;

    velXY.x += wishdir.x * accelSpeed;
    velXY.y += wishdir.y * accelSpeed;
}

static inline void Legacy_AccelerateAir(const LegacyAccelParams& p, float dt, const Vector& wishdir, float wishspeed, Vector& velXY)
{
    // NOTE: We do NOT apply the classic 30 wishspeed cap here.
    // Surf servers rely on strong air accel with high overall speed; we clamp with sv_maxvelocity later.
    float currentSpeed = DotProduct(velXY, wishdir);
    float addSpeed = wishspeed - currentSpeed;
    if (addSpeed <= 0.0f) return;

    float accelSpeed = p.sv_airaccelerate * wishspeed * dt;
    if (accelSpeed > addSpeed) accelSpeed = addSpeed;

    velXY.x += wishdir.x * accelSpeed;
    velXY.y += wishdir.y * accelSpeed;
}

// Computes a legacy acceleration/friction step using mv->m_outWishVel.
// - inOutVel: read current velocity; writes the legacy X/Y result (Z preserved by caller)
static inline void ComputeLegacyVelocityStep(CCSPlayer_MovementServices* ms, CMoveData* mv, const LegacyAccelParams& params, Vector& inOutVel)
{
    if (!params.enable || !gpGlobals || !mv) return;

    const float dt = gpGlobals->frametime > 0.f ? gpGlobals->frametime : (1.f / 64.f);

    // Use engine-provided wish vector for this subtick.
    Vector wish = mv->m_outWishVel;
    float wishspeed = wish.Length2D();

    Vector wishdir = wish;
    if (wishspeed > 0.0f)
        wishdir /= wishspeed;
    else
        wishdir = vec3_origin;

    // Work on XY only; preserve Z in the caller
    Vector velXY(inOutVel.x, inOutVel.y, 0.0f);

    if (mv->m_bOnGround)
    {
        Legacy_ApplyFriction(params, dt, velXY);
        Legacy_AccelerateGround(params, dt, wishdir, wishspeed, velXY);
    }
    else
    {
        Legacy_AccelerateAir(params, dt, wishdir, wishspeed, velXY);
    }

    inOutVel.x = velXY.x;
    inOutVel.y = velXY.y;
    // Z left as-is
}

static inline void ClampToMaxVelocity(Vector& v, float maxv)
{
    if (maxv > 0.0f)
    {
        float len = v.Length();
        if (len > maxv && len > 0.0f)
        {
            float scale = maxv / len;
            v.x *= scale;
            v.y *= scale;
            v.z *= scale;
        }
    }
}

// --------------------------------------------------------------------------------------

void FASTCALL Detour_ProcessMovement(CCSPlayer_MovementServices *pThis, void *pMove)
{
    CCSPlayerPawn *pPawn = pThis->GetPawn();

    ZEPlayer *player = g_playerManager->GetPlayer(pPawn->m_hController()->GetPlayerSlot());
    if(!player) return ProcessMovement(pThis, pMove);
    
    player->currentMoveData = static_cast<CMoveData*>(pMove);
    player->didTPM = false;
    player->processingMovement = true;

    // Compute legacy XY velocity candidate before engine runs
    Vector preVel;
    player->GetVelocity(&preVel);
    Vector legacyVelCandidate = preVel;

    if (player->currentMoveData && g_LegacyAccelParams.enable)
    {
        ComputeLegacyVelocityStep(pThis, player->currentMoveData, g_LegacyAccelParams, legacyVelCandidate);
        legacyVelCandidate.z = preVel.z; // preserve Z
    }

    // Run original engine movement (subtick, gravity, collision, etc.)
    ProcessMovement(pThis, pMove);

    // Selectively restore legacy XY accel if engine's XY diverges
    if (player->currentMoveData && g_LegacyAccelParams.enable)
    {
        Vector postVel;
        player->GetVelocity(&postVel);

        Vector legacyXY(legacyVelCandidate.x, legacyVelCandidate.y, 0.0f);
        Vector postXY(postVel.x, postVel.y, 0.0f);

        float lenLegacy = legacyXY.Length2D();
        float lenPost   = postXY.Length2D();

        bool diverged = false;
        if (lenLegacy > 1.0f && lenPost > 1.0f)
        {
            // Normalize XY to compare direction
            Vector nLegacy = legacyXY; if (lenLegacy > 0.0f) nLegacy /= lenLegacy;
            Vector nPost   = postXY;   if (lenPost   > 0.0f) nPost   /= lenPost;
            float dot = DotProduct(nLegacy, nPost);
            diverged = (dot < 0.999f);
        }

        // Merge and clamp to sv_maxvelocity
        if (diverged)
        {
            Vector merged = postVel;
            merged.x = legacyVelCandidate.x;
            merged.y = legacyVelCandidate.y;

            // Clamp overall speed if configured
            ClampToMaxVelocity(merged, g_LegacyAccelParams.sv_maxvelocity);

            player->SetVelocity(merged);
        }
        else
        {
            // Even if we didn't override, enforce max velocity ceiling if needed
            Vector clamped = postVel;
            ClampToMaxVelocity(clamped, g_LegacyAccelParams.sv_maxvelocity);
            if (clamped != postVel)
                player->SetVelocity(clamped);
        }
    }

    if(!player->didTPM)
        player->lastValidPlane = vec3_origin;
    
    player->processingMovement = false;
}

#define f32 float32
#define i32 int32_t
#define u32 uint32_t

void ClipVelocity(Vector &in, Vector &normal, Vector &out)
{
    // Determine how far along plane to slide based on incoming direction.
    f32 backoff = DotProduct(in, normal);

    for (i32 i = 0; i < 3; i++)
    {
        f32 change = normal[i] * backoff;
        out[i] = in[i] - change;
    }
    float adjust = DotProduct(out, normal);
    if (adjust < 0.0f)
    {
        adjust = MIN(adjust, -1 / 128);
        out -= (normal * adjust);
    }
}

bool IsValidMovementTrace(trace_t &tr, bbox_t bounds, CTraceFilterPlayerMovementCS *filter)
{
    trace_t stuck;
    // Maybe we don't need this one.
    // if (trm_flFraction < FLT_EPSILON)
    //{
    //  return false;
    //}

    if (tr.m_bStartInSolid)
    {
        return false;
    }

    // We hit something but no valid plane data?
    if (tr.m_flFraction < 1.0f && fabs(tr.m_vHitNormal.x) < FLT_EPSILON && fabs(tr.m_vHitNormal.y) < FLT_EPSILON && fabs(tr.m_vHitNormal.z) < FLT_EPSILON)
    {
        return false;
    }

    // Is the plane deformed?
    if (fabs(tr.m_vHitNormal.x) > 1.0f || fabs(tr.m_vHitNormal.y) > 1.0f || fabs(tr.m_vHitNormal.z) > 1.0f)
    {
        return false;
    }

    // Do an unswept trace and a backward trace just to be sure.
    addresses::TracePlayerBBox(tr.m_vEndPos, tr.m_vEndPos, bounds, filter, stuck);
    if (stuck.m_bStartInSolid || stuck.m_flFraction < 1.0f - FLT_EPSILON)
    {
        return false;
    }

    addresses::TracePlayerBBox(tr.m_vEndPos, tr.m_vStartPos, bounds, filter, stuck);
    // For whatever reason if you can hit something in only one direction and not the other way around.
    // Only happens since Call to Arms update, so this fraction check is commented out until it is fixed.
    if (stuck.m_bStartInSolid /*|| stuck.m_flFraction < 1.0f - FLT_EPSILON*/)
    {
        return false;
    }

    return true;
}

#define MAX_BUMPS 4
#define RAMP_PIERCE_DISTANCE 0.75f
#define RAMP_BUG_THRESHOLD 0.99f
#define RAMP_BUG_VELOCITY_THRESHOLD 0.95f 
#define NEW_RAMP_THRESHOLD 0.95f
void TryPlayerMovePre(CCSPlayer_MovementServices *ms, Vector *pFirstDest, trace_t *pFirstTrace, bool *bIsSurfing)
{
    CCSPlayerPawn *pawn = ms->GetPawn();
    ZEPlayer *player = g_playerManager->GetPlayer(pawn->m_hController()->GetPlayerSlot());
    player->overrideTPM = false;
    player->didTPM = true;

    f32 timeLeft = gpGlobals->frametime;

    Vector start, velocity, end;
    player->GetOrigin(&start);
    player->GetVelocity(&velocity);

    if (velocity.Length() == 0.0f)
    {
        // No move required.
        return;
    }
    Vector primalVelocity = velocity;
    bool validPlane {};

    f32 allFraction {};
    trace_t pm;
    u32 bumpCount {};
    Vector planes[5];
    u32 numPlanes {};
    trace_t pierce;

    bbox_t bounds;
    bounds.mins = {-16, -16, 0};
    bounds.maxs = {16, 16, 72};

    if (ms->m_bDucked())
    {
        bounds.maxs.z = 54;
    }

    CTraceFilterPlayerMovementCS filter(pawn);

    bool potentiallyStuck {};
    
    for (bumpCount = 0; bumpCount < MAX_BUMPS; bumpCount++)
    {
        // Assume we can move all the way from the current origin to the end point.
        VectorMA(start, timeLeft, velocity, end);
        // See if we can make it from origin to end point.
        // If their velocity Z is 0, then we can avoid an extra trace here during WalkMove.
        if (pFirstDest && end == *pFirstDest)
        {
            pm = *pFirstTrace;
        }
        else
        {
            addresses::TracePlayerBBox(start, end, bounds, &filter, pm);
            if (end == start)
            {
                continue;
            }
            if (IsValidMovementTrace(pm, bounds, &filter) && pm.m_flFraction == 1.0f)
            {
                // Player won't hit anything, nothing to do.
                break;
            }
            if (player->lastValidPlane.Length() > FLT_EPSILON
                && (!IsValidMovementTrace(pm, bounds, &filter) || pm.m_vHitNormal.Dot(player->lastValidPlane) < RAMP_BUG_THRESHOLD
                    || (potentiallyStuck && pm.m_flFraction == 0.0f)))
            {
                // We hit a plane that will significantly change our velocity. Make sure that this plane is significant
                // enough.
                Vector offsetDirection;
                f32 offsets[] = {0.0f, -1.0f, 1.0f};
                bool success {};
                for (u32 i = 0; i < 3 && !success; i++)
                {
                    for (u32 j = 0; j < 3 && !success; j++)
                    {
                        for (u32 k = 0; k < 3 && !success; k++)
                        {
                            if (i == 0 && j == 0 && k == 0)
                            {
                                offsetDirection = player->lastValidPlane;
                            }
                            else
                            {
                                offsetDirection = {offsets[i], offsets[j], offsets[k]};
                                // Check if this random offset is even valid.
                                if (player->lastValidPlane.Dot(offsetDirection) <= 0.0f)
                                {
                                    continue;
                                }
                                trace_t test;
                                addresses::TracePlayerBBox(start + offsetDirection * RAMP_PIERCE_DISTANCE, start, bounds, &filter, test);
                                if (!IsValidMovementTrace(test, bounds, &filter))
                                {
                                    continue;
                                }
                            }
                            bool goodTrace {};
                            f32 ratio {};
                            bool hitNewPlane {};
                            for (ratio = 0.1f; ratio <= 1.0f; ratio += 0.1f)
                            {
                                addresses::TracePlayerBBox(start + offsetDirection * RAMP_PIERCE_DISTANCE * ratio,
                                                            end + offsetDirection * RAMP_PIERCE_DISTANCE * ratio, bounds, &filter, pierce);
                                if (!IsValidMovementTrace(pierce, bounds, &filter))
                                {
                                    continue;
                                }
                                // Try until we hit a similar plane.
                                // clang-format off
                                validPlane = pierce.m_flFraction < 1.0f && pierce.m_flFraction > 0.1f 
                                             && pierce.m_vHitNormal.Dot(player->lastValidPlane) >= RAMP_BUG_THRESHOLD;

                                hitNewPlane = pm.m_vHitNormal.Dot(pierce.m_vHitNormal) < NEW_RAMP_THRESHOLD 
                                              && player->lastValidPlane.Dot(pierce.m_vHitNormal) > NEW_RAMP_THRESHOLD;
                                // clang-format on
                                goodTrace = CloseEnough(pierce.m_flFraction, 1.0f, FLT_EPSILON) || validPlane;
                                if (goodTrace)
                                {
                                    break;
                                }
                            }
                            if (goodTrace || hitNewPlane)
                            {
                                // Trace back to the original end point to find its normal.
                                trace_t test;
                                addresses::TracePlayerBBox(pierce.m_vEndPos, end, bounds, &filter, test);
                                pm = pierce;
                                pm.m_vStartPos = start;
                                pm.m_flFraction = Clamp((pierce.m_vEndPos - pierce.m_vStartPos).Length() / (end - start).Length(), 0.0f, 1.0f);
                                pm.m_vEndPos = test.m_vEndPos;
                                if (pierce.m_vHitNormal.Length() > 0.0f)
                                {
                                    pm.m_vHitNormal = pierce.m_vHitNormal;
                                    player->lastValidPlane = pierce.m_vHitNormal;
                                }
                                else
                                {
                                    pm.m_vHitNormal = test.m_vHitNormal;
                                    player->lastValidPlane = test.m_vHitNormal;
                                }
                                success = true;
                                player->overrideTPM = true;
                            }
                        }
                    }
                }
            }
            if (pm.m_vHitNormal.Length() > 0.99f)
            {
                player->lastValidPlane = pm.m_vHitNormal;
            }
            potentiallyStuck = pm.m_flFraction == 0.0f;
        }

        if (pm.m_flFraction * velocity.Length() > 0.03125f)
        {
            allFraction += pm.m_flFraction;
            start = pm.m_vEndPos;
            numPlanes = 0;
        }
        if (allFraction == 1.0f)
        {
            break;
        }
        timeLeft -= gpGlobals->frametime * pm.m_flFraction;
        planes[numPlanes] = pm.m_vHitNormal;
        numPlanes++;
        if (numPlanes == 1 && pawn->m_MoveType() == MOVETYPE_WALK && pawn->m_hGroundEntity().Get() == nullptr)
        {
            ClipVelocity(velocity, planes[0], velocity);
        }
        else
        {
            u32 i, j;
            for (i = 0; i < numPlanes; i++)
            {
                ClipVelocity(velocity, planes[i], velocity);
                for (j = 0; j < numPlanes; j++)
                {
                    if (j != i)
                    {
                        // Are we now moving against this plane?
                        if (velocity.Dot(planes[j]) < 0)
                        {
                            break; // not ok
                        }
                    }
                }

                if (j == numPlanes) // Didn't have to clip, so we're ok
                {
                    break;
                }
            }
            // Did we go all the way through plane set
            if (i != numPlanes)
            { // go along this plane
                // pmove.velocity is set in clipping call, no need to set again.
                ;
            }
            else
            { // go along the crease
                if (numPlanes != 2  || (pm.m_vHitNormal.z >= 0.7 && velocity.Length2D() < 1.0f))
                {
                    VectorCopy(vec3_origin, velocity);
                    break;
                }
                Vector dir;
                f32 d;
                CrossProduct(planes[0], planes[1], dir);
                dir.NormalizeInPlace();
                d = dir.Dot(velocity);
                VectorScale(dir, d, velocity);

                if (velocity.Dot(primalVelocity) <= 0)
                {
                    velocity = vec3_origin;
                    break;
                }
            }
        }
    }
    player->tpmOrigin = pm.m_vEndPos;
    player->tpmVelocity = velocity;
}

void TryPlayerMovePost(CCSPlayer_MovementServices *ms, bool *bIsSurfing)
{
    ZEPlayer *player = g_playerManager->GetPlayer(ms->GetPawn()->m_hController()->GetPlayerSlot());
    if(!player)
        return;
    Vector velocity;
    player->GetVelocity(&velocity);
    bool velocityHeavilyModified =
        player->tpmVelocity.Normalized().Dot(velocity.Normalized()) < RAMP_BUG_THRESHOLD
        || (player->tpmVelocity.Length() > 50.0f && velocity.Length() / player->tpmVelocity.Length() < RAMP_BUG_VELOCITY_THRESHOLD);

    if (player->overrideTPM && velocityHeavilyModified && player->tpmOrigin != vec3_invalid && player->tpmVelocity != vec3_invalid)
    {
        player->SetOrigin(player->tpmOrigin);
        player->SetVelocity(player->tpmVelocity);
    }
}

void FASTCALL Detour_TryPlayerMove(CCSPlayer_MovementServices *ms, CMoveData *mv, Vector *pFirstDest, trace_t *pFirstTrace, bool *bIsSurfing)
{
    TryPlayerMovePre(ms, pFirstDest, pFirstTrace, bIsSurfing);
    TryPlayerMove(ms, mv, pFirstDest, pFirstTrace, bIsSurfing);
    TryPlayerMovePost(ms, bIsSurfing);
}

void CategorizePositionPre(CCSPlayer_MovementServices *ms,bool bStayOnGround)
{
    CCSPlayerPawn *pawn = ms->GetPawn();
    ZEPlayer *player = g_playerManager->GetPlayer(pawn->m_hController()->GetPlayerSlot());
    // Already on the ground?
    // If we are already colliding on a standable valid plane, we don't want to do the check.
    if (bStayOnGround || player->lastValidPlane.Length() < 0.000001f|| player->lastValidPlane.z > 0.7f)
    {
        return;
    }
    Vector velocity;
    player->GetVelocity(&velocity);
    //Only attempt to fix rampbugs while going down significantly enough.
    if (velocity.z > -64.0f)
    {
        return;
    }
    bbox_t bounds;
    bounds.mins = {-16, -16, 0};
    bounds.maxs = {16, 16, 72};
                  
    if (ms->m_bDucked)
    {
        bounds.maxs.z = 54;
    }

    CTraceFilterPlayerMovementCS filter(pawn);

    trace_t trace;

    Vector origin, groundOrigin;
    player->GetOrigin(&origin);
    groundOrigin = origin;
    groundOrigin.z -= 2.0f;

    addresses::TracePlayerBBox(origin, groundOrigin, bounds, &filter, trace);

    if (trace.m_flFraction == 1.0f)
    {
        return;
    }
    // Is this something that you should be able to actually stand on?
    if (trace.m_flFraction < 0.95f && trace.m_vHitNormal.z > 0.7f && player->lastValidPlane.Dot(trace.m_vHitNormal) < RAMP_BUG_THRESHOLD)
    {
        origin += player->lastValidPlane * 0.0625f;
        groundOrigin = origin;
        groundOrigin.z -= 2.0f;
        addresses::TracePlayerBBox(origin, groundOrigin, bounds, &filter, trace);
        if (trace.m_bStartInSolid)
        {
            return;
        }
        if (trace.m_flFraction == 1.0f || player->lastValidPlane.Dot(trace.m_vHitNormal) >= RAMP_BUG_THRESHOLD)
        {
            player->SetOrigin(origin);
        }
    }
}

void FASTCALL Detour_CategorizePosition(CCSPlayer_MovementServices *ms, CMoveData *mv, bool bStayOnGround)
{
    CategorizePositionPre(ms, bStayOnGround);
    CategorizePosition(ms, mv, bStayOnGround);
}

bool InitDetours(CGameConfig *gameConfig)
{
    bool success = true;

    FOR_EACH_VEC(g_vecDetours, i)
    {
        if (!g_vecDetours[i]->CreateDetour(gameConfig))
            success = false;
        
        g_vecDetours[i]->EnableDetour();
    }

    return success;
}

void FlushAllDetours()
{
    g_vecDetours.Purge();
}
