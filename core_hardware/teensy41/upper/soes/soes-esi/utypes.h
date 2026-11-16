#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   float motor_state_pos[15];
   int16_t motor_state_vel[15];
   int16_t motor_state_torque[15];
   uint8_t system_state[2];
   uint8_t core_state[2];

   /* Outputs */

   float motor_ref[16];
   uint8_t system_ref[1];
   uint16_t LED_TAPE[3];

} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
