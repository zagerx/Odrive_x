
#include "MyProject.h"


/****************************************************************************/
TRAPTRAJ_Config_t  trapTraj_config;
TRAPTRAJ_t  trap_traj_;

/****************************************************************************/
void trapTraj_config_default(void)
{
	trapTraj_config.vel_limit = TRAPTraj_vel_limit;      //2.0f; // [turn/s]
	trapTraj_config.accel_limit = TRAPTraj_accel_limit;  //0.5f; // [turn/s^2]
	trapTraj_config.decel_limit = TRAPTraj_decel_limit;  //0.5f; // [turn/s^2]
}
/****************************************************************************/
// A sign function where input 0 has positive sign (not 0)
float sign_hard(float val)
{
	return signbit(val) ? -1.0f : 1.0f;
}
/****************************************************************************/
// Symbol                     Description
// Ta, Tv and Td              Duration of the stages of the AL profile
// Xi and Vi                  Adapted initial conditions for the AL profile
// Xf                         Position set-point
// s                          Direction (sign) of the trajectory
// Vmax, Amax, Dmax and jmax  Kinematic bounds
// Ar, Dr and Vr              Reached values of acceleration and velocity

uint8_t planTrapezoidal(float Xf, float Xi, float Vi,float Vmax, float Amax, float Dmax)
{
	float dX = Xf - Xi;  // Distance to travel
	float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance
	float dXstop = copysignf(stop_dist, Vi); // Minimum stopping displacement
	float s = sign_hard(dX - dXstop); // Sign of coast velocity (if any)
	TRAPTRAJ_t  *p;
	p = &trap_traj_;
	
	p->Ar_ = s * Amax;  // Maximum Acceleration (signed)
	p->Dr_ = -s * Dmax; // Maximum Deceleration (signed)
	p->Vr_ = s * Vmax;  // Maximum Velocity (signed)
	
	// If we start with a speed faster than cruising, then we need to decel instead of accel
	// aka "double deceleration move" in the paper
	if ((s * Vi) > (s * p->Vr_))p->Ar_ = -s * Amax;
	
	// Time to accel/decel to/from Vr (cruise speed)
	p->Ta_ = (p->Vr_ - Vi) / p->Ar_;
	p->Td_ = -p->Vr_ / p->Dr_;
	
	// Integral of velocity ramps over the full accel and decel times to get
	// minimum displacement required to reach cuising speed
	float dXmin = 0.5f * p->Ta_ * (p->Vr_ + Vi) + 0.5f * p->Td_ * p->Vr_;
	
	// Are we displacing enough to reach cruising speed?
	if (s*dX < s*dXmin)
	{
		// Short move (triangle profile)
		p->Vr_ = s * sqrtf(max((p->Dr_ * SQ(Vi) + 2 * p->Ar_ * p->Dr_ * dX) / (p->Dr_ - p->Ar_), 0.0f));
		p->Ta_ = max(0.0f, (p->Vr_ - Vi) / p->Ar_);
		p->Td_ = max(0.0f, -p->Vr_ / p->Dr_);
		p->Tv_ = 0.0f;
	}
	else
	{
		// Long move (trapezoidal profile)
		p->Tv_ = (dX - dXmin) / p->Vr_;
	}

	// Fill in the rest of the values used at evaluation-time
	p->Tf_ = p->Ta_ + p->Tv_ + p->Td_;
	p->Xi_ = Xi;
	p->Xf_ = Xf;
	p->Vi_ = Vi;
	p->yAccel_ = Xi + Vi * p->Ta_ + 0.5f * p->Ar_ * SQ(p->Ta_); // pos at end of accel phase
	
	return 1;
}
/****************************************************************************/
Step_t trap_traj_eval(float t)
{
	Step_t trajStep;
	
    if (t < 0.0f) {  // Initial Condition
        trajStep.Y   = trap_traj_.Xi_;
        trajStep.Yd  = trap_traj_.Vi_;
        trajStep.Ydd = 0.0f;
    } else if (t < trap_traj_.Ta_) {  // Accelerating
        trajStep.Y   = trap_traj_.Xi_ + trap_traj_.Vi_ * t + 0.5f * trap_traj_.Ar_ * SQ(t);
        trajStep.Yd  = trap_traj_.Vi_ + trap_traj_.Ar_*t;
        trajStep.Ydd = trap_traj_.Ar_;
    } else if (t < trap_traj_.Ta_ + trap_traj_.Tv_) {  // Coasting
        trajStep.Y   = trap_traj_.yAccel_ + trap_traj_.Vr_*(t - trap_traj_.Ta_);
        trajStep.Yd  = trap_traj_.Vr_;
        trajStep.Ydd = 0.0f;
    } else if (t < trap_traj_.Tf_) {  // Deceleration
        float td     = t - trap_traj_.Tf_;
        trajStep.Y   = trap_traj_.Xf_ + 0.5f * trap_traj_.Dr_ * SQ(td);
        trajStep.Yd  = trap_traj_.Dr_*td;
        trajStep.Ydd = trap_traj_.Dr_;
    } else if (t >= trap_traj_.Tf_) {  // Final Condition
        trajStep.Y   = trap_traj_.Xf_;
        trajStep.Yd  = 0.0f;
        trajStep.Ydd = 0.0f;
    } else {
        // TODO: report error here
    }

    return trajStep;
}
/****************************************************************************/


