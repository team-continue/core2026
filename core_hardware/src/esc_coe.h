#ifndef CORE_HARDWARE_ESC_COE_H_
#define CORE_HARDWARE_ESC_COE_H_

#include "cc.h"

typedef struct
{
   uint16_t subindex;
   uint16_t datatype;
   uint16_t bitlength;
   uint16_t flags;
   const char *name;
   uint32_t value;
   void *data;
} _objd;

typedef struct
{
   uint16_t index;
   uint16_t objtype;
   uint8_t maxsub;
   uint8_t pad1;
   const char *name;
   const _objd *objdesc;
} _objectlist;

#define OTYPE_VAR 0x0007
#define OTYPE_ARRAY 0x0008
#define OTYPE_RECORD 0x0009

#define DTYPE_BOOLEAN 0x0001
#define DTYPE_INTEGER16 0x0003
#define DTYPE_UNSIGNED8 0x0005
#define DTYPE_UNSIGNED16 0x0006
#define DTYPE_UNSIGNED32 0x0007
#define DTYPE_REAL32 0x0008
#define DTYPE_VISIBLE_STRING 0x0009

#define ATYPE_Rpre 0x01
#define ATYPE_Rsafe 0x02
#define ATYPE_Rop 0x04
#define ATYPE_Wpre 0x08
#define ATYPE_Wsafe 0x10
#define ATYPE_Wop 0x20
#define ATYPE_RXPDO 0x40
#define ATYPE_TXPDO 0x80

#define ATYPE_RO (ATYPE_Rpre | ATYPE_Rsafe | ATYPE_Rop)

#endif
