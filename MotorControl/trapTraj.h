
#ifndef _TRAP_TRAJ_H
#define _TRAP_TRAJ_H

#include "MyProject.h"


/****************************************************************************/
typedef struct 
{
	float vel_limit;   // [turn/s]
	float accel_limit; // [turn/s^2]
	float decel_limit; // [turn/s^2]
} TRAPTRAJ_Config_t;

typedef struct 
{
	float Y;
	float Yd;
	float Ydd;
} Step_t;

typedef struct 
{
	float Xi_;
	float Xf_;
	float Vi_;
	
	float Ar_;
	float Vr_;
	float Dr_;
	
	float Ta_;
	float Tv_;
	float Td_;
	float Tf_;
	
	float yAccel_;
	
	float t_;
} TRAPTRAJ_t;

extern  TRAPTRAJ_Config_t  trapTraj_config;
extern  TRAPTRAJ_t  trap_traj_;
/****************************************************************************/
void trapTraj_config_default(void);
uint8_t planTrapezoidal(float Xf, float Xi, float Vi,float Vmax, float Amax, float Dmax);
Step_t trap_traj_eval(float t);
/****************************************************************************/

#endif

