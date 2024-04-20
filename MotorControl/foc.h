
#ifndef __FOC_LIB_H
#define __FOC_LIB_H

#include "MyProject.h"


/****************************************************************************/
extern  uint32_t  motor_error;
extern  float  Ialpha_beta[2];
extern  float2D  *Idq_setpoint_src_;
extern  float2D  *Vdq_setpoint_src_;
extern  float  *phase_src_;
extern  float  *phase_vel_src_;

extern  float  pi_gains_[2];
/****************************************************************************/
bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
void on_measurement(uint32_t input_timestamp, Iph_ABC_t *current);
void foc_update(void);
void pwm_update_cb(void);
void foc_reset(void);
/****************************************************************************/
static inline void set_error(uint32_t error)
{
	motor_error |= error;
}
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/


#endif

