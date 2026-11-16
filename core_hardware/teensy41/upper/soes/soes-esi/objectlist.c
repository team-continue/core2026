#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>


static const char acName1000[] = "Device Type";
static const char acName1008[] = "Device Name";
static const char acName1009[] = "Hardware Version";
static const char acName100A[] = "Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1600[] = "motor_ref";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "m0_vel";
static const char acName1600_02[] = "m1_vel";
static const char acName1600_03[] = "m2_vel";
static const char acName1600_04[] = "m3_vel";
static const char acName1600_05[] = "m4_pos";
static const char acName1600_06[] = "m5_pos";
static const char acName1600_07[] = "m6_pos";
static const char acName1600_08[] = "m7_pos";
static const char acName1600_09[] = "m8_pos";
static const char acName1600_10[] = "m9_pos";
static const char acName1600_11[] = "m10_pos";
static const char acName1600_12[] = "m11_pos";
static const char acName1600_13[] = "m12_pos";
static const char acName1600_14[] = "m13_pos";
static const char acName1600_15[] = "m14_pos";
static const char acName1600_16[] = "m15_vel";
static const char acName1601[] = "system_ref";
static const char acName1601_00[] = "Max SubIndex";
static const char acName1601_01[] = "bottom";
static const char acName1602[] = "LED_TAPE";
static const char acName1602_00[] = "Max SubIndex";
static const char acName1602_01[] = "led0_upper";
static const char acName1602_02[] = "led1_bottom";
static const char acName1602_03[] = "led2_bottom";
static const char acName1A00[] = "motor_state_pos";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "m0";
static const char acName1A00_02[] = "m1";
static const char acName1A00_03[] = "m2";
static const char acName1A00_04[] = "m3";
static const char acName1A00_05[] = "m4";
static const char acName1A00_06[] = "m5";
static const char acName1A00_07[] = "m6";
static const char acName1A00_08[] = "m7";
static const char acName1A00_09[] = "m8";
static const char acName1A00_10[] = "m9";
static const char acName1A00_11[] = "m10";
static const char acName1A00_12[] = "m11";
static const char acName1A00_13[] = "m12";
static const char acName1A00_14[] = "m13";
static const char acName1A00_15[] = "m14";
static const char acName1A01[] = "motor_state_vel";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "m0";
static const char acName1A01_02[] = "m1";
static const char acName1A01_03[] = "m2";
static const char acName1A01_04[] = "m3";
static const char acName1A01_05[] = "m4";
static const char acName1A01_06[] = "m5";
static const char acName1A01_07[] = "m6";
static const char acName1A01_08[] = "m7";
static const char acName1A01_09[] = "m8";
static const char acName1A01_10[] = "m9";
static const char acName1A01_11[] = "m10";
static const char acName1A01_12[] = "m11";
static const char acName1A01_13[] = "m12";
static const char acName1A01_14[] = "m13";
static const char acName1A01_15[] = "m14";
static const char acName1A02[] = "motor_state_torque";
static const char acName1A02_00[] = "Max SubIndex";
static const char acName1A02_01[] = "m0";
static const char acName1A02_02[] = "m1";
static const char acName1A02_03[] = "m2";
static const char acName1A02_04[] = "m3";
static const char acName1A02_05[] = "m4";
static const char acName1A02_06[] = "m5";
static const char acName1A02_07[] = "m6";
static const char acName1A02_08[] = "m7";
static const char acName1A02_09[] = "m8";
static const char acName1A02_10[] = "m9";
static const char acName1A02_11[] = "m10";
static const char acName1A02_12[] = "m11";
static const char acName1A02_13[] = "m12";
static const char acName1A02_14[] = "m13";
static const char acName1A02_15[] = "m14";
static const char acName1A03[] = "system_state";
static const char acName1A03_00[] = "Max SubIndex";
static const char acName1A03_01[] = "upper";
static const char acName1A03_02[] = "bottom";
static const char acName1A04[] = "core_state";
static const char acName1A04_00[] = "Max SubIndex";
static const char acName1A04_01[] = "damage";
static const char acName1A04_02[] = "destory";
static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communications Type SM0";
static const char acName1C00_02[] = "Communications Type SM1";
static const char acName1C00_03[] = "Communications Type SM2";
static const char acName1C00_04[] = "Communications Type SM3";
static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";
static const char acName1C12_02[] = "PDO Mapping";
static const char acName1C12_03[] = "PDO Mapping";
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName1C13_02[] = "PDO Mapping";
static const char acName1C13_03[] = "PDO Mapping";
static const char acName1C13_04[] = "PDO Mapping";
static const char acName1C13_05[] = "PDO Mapping";
static const char acName6000[] = "motor_state_pos";
static const char acName6000_00[] = "Max SubIndex";
static const char acName6000_01[] = "m0";
static const char acName6000_02[] = "m1";
static const char acName6000_03[] = "m2";
static const char acName6000_04[] = "m3";
static const char acName6000_05[] = "m4";
static const char acName6000_06[] = "m5";
static const char acName6000_07[] = "m6";
static const char acName6000_08[] = "m7";
static const char acName6000_09[] = "m8";
static const char acName6000_10[] = "m9";
static const char acName6000_11[] = "m10";
static const char acName6000_12[] = "m11";
static const char acName6000_13[] = "m12";
static const char acName6000_14[] = "m13";
static const char acName6000_15[] = "m14";
static const char acName6001[] = "motor_state_vel";
static const char acName6001_00[] = "Max SubIndex";
static const char acName6001_01[] = "m0";
static const char acName6001_02[] = "m1";
static const char acName6001_03[] = "m2";
static const char acName6001_04[] = "m3";
static const char acName6001_05[] = "m4";
static const char acName6001_06[] = "m5";
static const char acName6001_07[] = "m6";
static const char acName6001_08[] = "m7";
static const char acName6001_09[] = "m8";
static const char acName6001_10[] = "m9";
static const char acName6001_11[] = "m10";
static const char acName6001_12[] = "m11";
static const char acName6001_13[] = "m12";
static const char acName6001_14[] = "m13";
static const char acName6001_15[] = "m14";
static const char acName6002[] = "motor_state_torque";
static const char acName6002_00[] = "Max SubIndex";
static const char acName6002_01[] = "m0";
static const char acName6002_02[] = "m1";
static const char acName6002_03[] = "m2";
static const char acName6002_04[] = "m3";
static const char acName6002_05[] = "m4";
static const char acName6002_06[] = "m5";
static const char acName6002_07[] = "m6";
static const char acName6002_08[] = "m7";
static const char acName6002_09[] = "m8";
static const char acName6002_10[] = "m9";
static const char acName6002_11[] = "m10";
static const char acName6002_12[] = "m11";
static const char acName6002_13[] = "m12";
static const char acName6002_14[] = "m13";
static const char acName6002_15[] = "m14";
static const char acName6003[] = "system_state";
static const char acName6003_00[] = "Max SubIndex";
static const char acName6003_01[] = "upper";
static const char acName6003_02[] = "bottom";
static const char acName6004[] = "core_state";
static const char acName6004_00[] = "Max SubIndex";
static const char acName6004_01[] = "damage";
static const char acName6004_02[] = "destory";
static const char acName7000[] = "motor_ref";
static const char acName7000_00[] = "Max SubIndex";
static const char acName7000_01[] = "m0_vel";
static const char acName7000_02[] = "m1_vel";
static const char acName7000_03[] = "m2_vel";
static const char acName7000_04[] = "m3_vel";
static const char acName7000_05[] = "m4_pos";
static const char acName7000_06[] = "m5_pos";
static const char acName7000_07[] = "m6_pos";
static const char acName7000_08[] = "m7_pos";
static const char acName7000_09[] = "m8_pos";
static const char acName7000_10[] = "m9_pos";
static const char acName7000_11[] = "m10_pos";
static const char acName7000_12[] = "m11_pos";
static const char acName7000_13[] = "m12_pos";
static const char acName7000_14[] = "m13_pos";
static const char acName7000_15[] = "m14_pos";
static const char acName7000_16[] = "m15_vel";
static const char acName7001[] = "system_ref";
static const char acName7001_00[] = "Max SubIndex";
static const char acName7001_01[] = "bottom";
static const char acName7002[] = "LED_TAPE";
static const char acName7002_00[] = "Max SubIndex";
static const char acName7002_01[] = "led0_upper";
static const char acName7002_02[] = "led1_bottom";
static const char acName7002_03[] = "led2_bottom";

const _objd SDO1000[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 5001, NULL},
};
const _objd SDO1008[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 112, ATYPE_RO, acName1008, 0, "core2026_upper"},
};
const _objd SDO1009[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName1009, 0, "0.0.1"},
};
const _objd SDO100A[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName100A, 0, "0.0.1"},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 4384, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 4384, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 2, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 1, &Obj.serial},
};
const _objd SDO1600[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 16, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x70000120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_02, 0x70000220, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_03, 0x70000320, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_04, 0x70000420, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_05, 0x70000520, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_06, 0x70000620, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_07, 0x70000720, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_08, 0x70000820, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_09, 0x70000920, NULL},
  {0x0a, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_10, 0x70000a20, NULL},
  {0x0b, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_11, 0x70000b20, NULL},
  {0x0c, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_12, 0x70000c20, NULL},
  {0x0d, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_13, 0x70000d20, NULL},
  {0x0e, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_14, 0x70000e20, NULL},
  {0x0f, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_15, 0x70000f20, NULL},
  {0x10, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_16, 0x70001020, NULL},
};
const _objd SDO1601[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1601_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_01, 0x70010101, NULL},
};
const _objd SDO1602[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1602_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_01, 0x70020110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_02, 0x70020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_03, 0x70020310, NULL},
};
const _objd SDO1A00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 15, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60000120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_02, 0x60000220, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_03, 0x60000320, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_04, 0x60000420, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_05, 0x60000520, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_06, 0x60000620, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_07, 0x60000720, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_08, 0x60000820, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_09, 0x60000920, NULL},
  {0x0a, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_10, 0x60000a20, NULL},
  {0x0b, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_11, 0x60000b20, NULL},
  {0x0c, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_12, 0x60000c20, NULL},
  {0x0d, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_13, 0x60000d20, NULL},
  {0x0e, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_14, 0x60000e20, NULL},
  {0x0f, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_15, 0x60000f20, NULL},
};
const _objd SDO1A01[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 15, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x60010110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_02, 0x60010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_03, 0x60010310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_04, 0x60010410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_05, 0x60010510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_06, 0x60010610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_07, 0x60010710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_08, 0x60010810, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_09, 0x60010910, NULL},
  {0x0a, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_10, 0x60010a10, NULL},
  {0x0b, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_11, 0x60010b10, NULL},
  {0x0c, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_12, 0x60010c10, NULL},
  {0x0d, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_13, 0x60010d10, NULL},
  {0x0e, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_14, 0x60010e10, NULL},
  {0x0f, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_15, 0x60010f10, NULL},
};
const _objd SDO1A02[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A02_00, 15, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x60020110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_02, 0x60020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_03, 0x60020310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_04, 0x60020410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_05, 0x60020510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_06, 0x60020610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_07, 0x60020710, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_08, 0x60020810, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_09, 0x60020910, NULL},
  {0x0a, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_10, 0x60020a10, NULL},
  {0x0b, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_11, 0x60020b10, NULL},
  {0x0c, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_12, 0x60020c10, NULL},
  {0x0d, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_13, 0x60020d10, NULL},
  {0x0e, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_14, 0x60020e10, NULL},
  {0x0f, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_15, 0x60020f10, NULL},
};
const _objd SDO1A03[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A03_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x60030101, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_02, 0x60030201, NULL},
};
const _objd SDO1A04[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A04_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x60040108, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_02, 0x60040208, NULL},
};
const _objd SDO1C00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_02, 0x1601, NULL},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_03, 0x1602, NULL},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 5, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_02, 0x1A01, NULL},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_03, 0x1A02, NULL},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_04, 0x1A03, NULL},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_05, 0x1A04, NULL},
};
const _objd SDO6000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6000_00, 15, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_01, 0x00000000, &Obj.motor_state_pos[0]},
  {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_02, 0x00000000, &Obj.motor_state_pos[1]},
  {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_03, 0x00000000, &Obj.motor_state_pos[2]},
  {0x04, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_04, 0x00000000, &Obj.motor_state_pos[3]},
  {0x05, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_05, 0x00000000, &Obj.motor_state_pos[4]},
  {0x06, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_06, 0x00000000, &Obj.motor_state_pos[5]},
  {0x07, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_07, 0x00000000, &Obj.motor_state_pos[6]},
  {0x08, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_08, 0x00000000, &Obj.motor_state_pos[7]},
  {0x09, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_09, 0x00000000, &Obj.motor_state_pos[8]},
  {0x0a, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_10, 0x00000000, &Obj.motor_state_pos[9]},
  {0x0b, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_11, 0x00000000, &Obj.motor_state_pos[10]},
  {0x0c, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_12, 0x00000000, &Obj.motor_state_pos[11]},
  {0x0d, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_13, 0x00000000, &Obj.motor_state_pos[12]},
  {0x0e, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_14, 0x00000000, &Obj.motor_state_pos[13]},
  {0x0f, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_TXPDO, acName6000_15, 0x00000000, &Obj.motor_state_pos[14]},
};
const _objd SDO6001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6001_00, 15, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_01, 0, &Obj.motor_state_vel[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_02, 0, &Obj.motor_state_vel[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_03, 0, &Obj.motor_state_vel[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_04, 0, &Obj.motor_state_vel[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_05, 0, &Obj.motor_state_vel[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_06, 0, &Obj.motor_state_vel[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_07, 0, &Obj.motor_state_vel[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_08, 0, &Obj.motor_state_vel[7]},
  {0x09, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_09, 0, &Obj.motor_state_vel[8]},
  {0x0a, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_10, 0, &Obj.motor_state_vel[9]},
  {0x0b, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_11, 0, &Obj.motor_state_vel[10]},
  {0x0c, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_12, 0, &Obj.motor_state_vel[11]},
  {0x0d, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_13, 0, &Obj.motor_state_vel[12]},
  {0x0e, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_14, 0, &Obj.motor_state_vel[13]},
  {0x0f, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6001_15, 0, &Obj.motor_state_vel[14]},
};
const _objd SDO6002[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6002_00, 15, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_01, 0, &Obj.motor_state_torque[0]},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_02, 0, &Obj.motor_state_torque[1]},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_03, 0, &Obj.motor_state_torque[2]},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_04, 0, &Obj.motor_state_torque[3]},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_05, 0, &Obj.motor_state_torque[4]},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_06, 0, &Obj.motor_state_torque[5]},
  {0x07, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_07, 0, &Obj.motor_state_torque[6]},
  {0x08, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_08, 0, &Obj.motor_state_torque[7]},
  {0x09, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_09, 0, &Obj.motor_state_torque[8]},
  {0x0a, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_10, 0, &Obj.motor_state_torque[9]},
  {0x0b, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_11, 0, &Obj.motor_state_torque[10]},
  {0x0c, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_12, 0, &Obj.motor_state_torque[11]},
  {0x0d, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_13, 0, &Obj.motor_state_torque[12]},
  {0x0e, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_14, 0, &Obj.motor_state_torque[13]},
  {0x0f, DTYPE_INTEGER16, 16, ATYPE_RO | ATYPE_TXPDO, acName6002_15, 0, &Obj.motor_state_torque[14]},
};
const _objd SDO6003[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6003_00, 2, NULL},
  {0x01, DTYPE_BOOLEAN, 1, ATYPE_RO | ATYPE_TXPDO, acName6003_01, 0, &Obj.system_state[0]},
  {0x02, DTYPE_BOOLEAN, 1, ATYPE_RO | ATYPE_TXPDO, acName6003_02, 0, &Obj.system_state[1]},
};
const _objd SDO6004[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6004_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName6004_01, 0, &Obj.core_state[0]},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName6004_02, 0, &Obj.core_state[1]},
};
const _objd SDO7000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7000_00, 16, NULL},
  {0x01, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_01, 0x00000000, &Obj.motor_ref[0]},
  {0x02, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_02, 0x00000000, &Obj.motor_ref[1]},
  {0x03, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_03, 0x00000000, &Obj.motor_ref[2]},
  {0x04, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_04, 0x00000000, &Obj.motor_ref[3]},
  {0x05, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_05, 0x00000000, &Obj.motor_ref[4]},
  {0x06, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_06, 0x00000000, &Obj.motor_ref[5]},
  {0x07, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_07, 0x00000000, &Obj.motor_ref[6]},
  {0x08, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_08, 0x00000000, &Obj.motor_ref[7]},
  {0x09, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_09, 0x00000000, &Obj.motor_ref[8]},
  {0x0a, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_10, 0x00000000, &Obj.motor_ref[9]},
  {0x0b, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_11, 0x00000000, &Obj.motor_ref[10]},
  {0x0c, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_12, 0x00000000, &Obj.motor_ref[11]},
  {0x0d, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_13, 0x00000000, &Obj.motor_ref[12]},
  {0x0e, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_14, 0x00000000, &Obj.motor_ref[13]},
  {0x0f, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_15, 0x00000000, &Obj.motor_ref[14]},
  {0x10, DTYPE_REAL32, 32, ATYPE_RO | ATYPE_RXPDO, acName7000_16, 0x00000000, &Obj.motor_ref[15]},
};
const _objd SDO7001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7001_00, 1, NULL},
  {0x01, DTYPE_BOOLEAN, 1, ATYPE_RO | ATYPE_RXPDO, acName7001_01, 0, &Obj.system_ref[0]},
};
const _objd SDO7002[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7002_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_RXPDO, acName7002_01, 0, &Obj.LED_TAPE[0]},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_RXPDO, acName7002_02, 0, &Obj.LED_TAPE[1]},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO | ATYPE_RXPDO, acName7002_03, 0, &Obj.LED_TAPE[2]},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 16, 0, acName1600, SDO1600},
  {0x1601, OTYPE_RECORD, 1, 0, acName1601, SDO1601},
  {0x1602, OTYPE_RECORD, 3, 0, acName1602, SDO1602},
  {0x1A00, OTYPE_RECORD, 15, 0, acName1A00, SDO1A00},
  {0x1A01, OTYPE_RECORD, 15, 0, acName1A01, SDO1A01},
  {0x1A02, OTYPE_RECORD, 15, 0, acName1A02, SDO1A02},
  {0x1A03, OTYPE_RECORD, 2, 0, acName1A03, SDO1A03},
  {0x1A04, OTYPE_RECORD, 2, 0, acName1A04, SDO1A04},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 3, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 5, 0, acName1C13, SDO1C13},
  {0x6000, OTYPE_ARRAY, 15, 0, acName6000, SDO6000},
  {0x6001, OTYPE_ARRAY, 15, 0, acName6001, SDO6001},
  {0x6002, OTYPE_ARRAY, 15, 0, acName6002, SDO6002},
  {0x6003, OTYPE_ARRAY, 2, 0, acName6003, SDO6003},
  {0x6004, OTYPE_ARRAY, 2, 0, acName6004, SDO6004},
  {0x7000, OTYPE_ARRAY, 16, 0, acName7000, SDO7000},
  {0x7001, OTYPE_ARRAY, 1, 0, acName7001, SDO7001},
  {0x7002, OTYPE_ARRAY, 3, 0, acName7002, SDO7002},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
