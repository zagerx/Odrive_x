
#ifndef __ENCODER_LIB_H
#define __ENCODER_LIB_H

#include "MyProject.h"

/****************************************************************************/
#define hallA_port_  GPIOB
#define hallA_gpio_  (1<<4)
#define hallB_port_  GPIOB
#define hallB_gpio_  (1<<5)
#define hallC_port_  GPIOC
#define hallC_gpio_  (1<<9)
/****************************************************************************/
typedef enum
{
	MODE_INCREMENTAL = 0,
	MODE_SPI_AS5047P,
	MODE_SPI_MT6701,
	MODE_SPI_MA730,
	MODE_SPI_TLE5012B,
} ENCODER_enum;

typedef struct 
{
	ENCODER_enum  mode;
	float  calib_range;        // Accuracy required to pass encoder cpr check
	float  calib_scan_distance;// rad electrical
	float  calib_scan_omega;   // rad/s electrical
	float  bandwidth;          //编码器带宽，和cpr成正比
	int32_t  phase_offset;     //编码器和转子电角度之间的相位差，run_offset_calibration()校准这个参数
	float  phase_offset_float; //编码器和转子电角度之间的相位差浮点部分，run_offset_calibration()校准这个参数
	int32_t  cpr;              //每圈编码器的脉冲数
	float  index_offset;
	bool  use_index;           //是否使用编码器索引信号
	bool  pre_calibrated;      // If true, this means the offset stored in
	                           // configuration is valid and does not need
                             // be determined by run_offset_calibration.
                             // In this case the encoder will enter ready
                             // state as soon as the index is found.
	int32_t  direction;        //电机转动方向，run_offset_calibration()校准这个参数
	bool  use_index_offset;
	bool  enable_phase_interpolation; // Use velocity to interpolate inside the count state
	bool find_idx_on_lockin_only;     // Only be sensitive during lockin scan constant vel state
	float phase_;              //最终用于计算的当前电角度，范围-Pi~Pi。
	float phase_vel_;          //最终用于计算的当前电角速度，单位rad/s。
} ENCODER_CONFIG;

extern  ENCODER_CONFIG   encoder_config;

extern  float  pos_estimate_;
extern  float  vel_estimate_;
extern  float  pos_circular_;
/****************************************************************************/
void MagneticSensor_Init(void);
bool run_offset_calibration(void);
void sample_now(void);
bool encoder_update(void);
/****************************************************************************/

#endif

