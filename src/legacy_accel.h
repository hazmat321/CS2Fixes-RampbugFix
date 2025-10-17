#pragma once

#include "cs2_sdk/entity/services.h"
#include "mathlib/vector.h"

struct LegacyAccelParams
{
    float sv_accelerate = 5.5f;    // tune to pre-update values
    float sv_airaccelerate = 12.0f;
    float sv_friction = 4.0f;
    float sv_stopspeed = 75.0f;
    float airspeed_cap = 30.0f;    // classic Source airspeed cap
    bool  enable = true;           // master toggle
};

// Computes a legacy acceleration/friction step using mv->m_outWishVel.
// - inOutVel: read current velocity; writes the legacy X/Y result (Z preserved by caller)
void ComputeLegacyVelocityStep(CCSPlayer_MovementServices* ms, CMoveData* mv, const LegacyAccelParams& params, Vector& inOutVel);
