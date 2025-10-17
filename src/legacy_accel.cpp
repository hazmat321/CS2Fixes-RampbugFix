#include "legacy_accel.h"
#include "entity/services.h"

extern CGlobalVars *gpGlobals;

static inline void ApplyFriction(const LegacyAccelParams& p, float dt, Vector& vel)
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

static inline void AccelerateGround(const LegacyAccelParams& p, float dt, const Vector& wishdir, float wishspeed, Vector& velXY)
{
    float currentSpeed = DotProduct(velXY, wishdir);
    float addSpeed = wishspeed - currentSpeed;
    if (addSpeed <= 0.0f) return;

    float accelSpeed = p.sv_accelerate * wishspeed * dt;
    if (accelSpeed > addSpeed) accelSpeed = addSpeed;

    velXY.x += wishdir.x * accelSpeed;
    velXY.y += wishdir.y * accelSpeed;
}

static inline void AccelerateAir(const LegacyAccelParams& p, float dt, const Vector& wishdir, float wishspeed, Vector& velXY)
{
    // Classic Source cap
    if (wishspeed > p.airspeed_cap)
        wishspeed = p.airspeed_cap;

    float currentSpeed = DotProduct(velXY, wishdir);
    float addSpeed = wishspeed - currentSpeed;
    if (addSpeed <= 0.0f) return;

    float accelSpeed = p.sv_airaccelerate * wishspeed * dt;
    if (accelSpeed > addSpeed) accelSpeed = addSpeed;

    velXY.x += wishdir.x * accelSpeed;
    velXY.y += wishdir.y * accelSpeed;
}

void ComputeLegacyVelocityStep(CCSPlayer_MovementServices* ms, CMoveData* mv, const LegacyAccelParams& params, Vector& inOutVel)
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
        ApplyFriction(params, dt, velXY);
        AccelerateGround(params, dt, wishdir, wishspeed, velXY);
    }
    else
    {
        AccelerateAir(params, dt, wishdir, wishspeed, velXY);
    }

    inOutVel.x = velXY.x;
    inOutVel.y = velXY.y;
    // Z left as-is
}
