
#include "MyProject.h"


/****************************************************************************/
OPENLOOP_struct openloop_controller_;
/****************************************************************************/
void openloop_controller_update(void)
{
	OPENLOOP_struct *p;
	p = &openloop_controller_;
	
	float prev_Id = p->Idq_setpoint_.d;
	//float prev_Iq = p->Idq_setpoint_.q;
	float prev_Vd = p->Vdq_setpoint_.d;
	//float prev_Vq = p->Vdq_setpoint_.q;
	float phase = p->phase_;
	float phase_vel = p->phase_vel_;
	
	float dt = 0.000125f; //(float)(timestamp - p->timestamp_) / (float)TIM_1_8_CLOCK_HZ;
	
	p->Idq_setpoint_.d = clamp(p->target_current_, prev_Id - p->max_current_ramp_ * dt, prev_Id + p->max_current_ramp_ * dt);
	p->Idq_setpoint_.q = 0;
	p->Vdq_setpoint_.d = clamp(p->target_voltage_, prev_Vd - p->max_voltage_ramp_ * dt, prev_Vd + p->max_voltage_ramp_ * dt);
	p->Vdq_setpoint_.q = 0;
	
	phase_vel = clamp(p->target_vel_, p->phase_vel_ - p->max_phase_vel_ramp_ * dt, p->phase_vel_ + p->max_phase_vel_ramp_ * dt);
	p->phase_vel_ = phase_vel;
	p->phase_ = wrap_pm_pi(phase + phase_vel * dt);
	p->total_distance_ = p->total_distance_ + phase_vel * dt;
	//p->timestamp_ = timestamp;
}
/****************************************************************************/



