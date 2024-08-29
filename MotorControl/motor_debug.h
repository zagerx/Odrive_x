#ifndef __MOTOR_DEBUG__H
#define __MOTOR_DEBUG__H

typedef struct motor_debug
{
    float ia_rawreal;
    float ib_rawreal;
    float ic_rawreal;
    float ia_real;
    float ib_real;
    float ic_real;    
    float iq_real;
    float id_real;
}m_debug_t;

extern m_debug_t motor_debug;

#endif
