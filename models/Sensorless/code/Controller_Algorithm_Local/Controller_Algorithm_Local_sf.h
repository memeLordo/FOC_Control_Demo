#ifndef Controller_Algorithm_Local_sf_h_
#define Controller_Algorithm_Local_sf_h_
#include <cmath>
#include <stdlib.h>
#define S_FUNCTION_NAME                Controller_Algorithm_Local_sf
#define S_FUNCTION_LEVEL               2
#ifndef RTW_GENERATED_S_FUNCTION
#define RTW_GENERATED_S_FUNCTION
#endif

#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_nonfinite.h"
#if !defined(MATLAB_MEX_FILE)
#include "rt_matrx.h"
#endif

#if !defined(RTW_SFUNCTION_DEFINES)
#define RTW_SFUNCTION_DEFINES

struct LocalS {
  void *blockIO;
  void *defaultParam;
  void *nonContDerivSig;
};

#define ssSetLocalBlockIO(S, io)       ((LocalS *)ssGetUserData(S))->blockIO = ((void *)(io))
#define ssGetLocalBlockIO(S)           ((LocalS *)ssGetUserData(S))->blockIO
#define ssSetLocalDefaultParam(S, paramVector) ((LocalS *)ssGetUserData(S))->defaultParam = (paramVector)
#define ssGetLocalDefaultParam(S)      ((LocalS *)ssGetUserData(S))->defaultParam
#define ssSetLocalNonContDerivSig(S, pSig) ((LocalS *)ssGetUserData(S))->nonContDerivSig = (pSig)
#define ssGetLocalNonContDerivSig(S)   ((LocalS *)ssGetUserData(S))->nonContDerivSig
#endif

#include "Controller_Algorithm_Local_sf_types.h"

extern "C"
{

#include "rtGetNaN.h"

}

#include <cstring>
#include "rt_defines.h"
#include "simstruc_types.h"

struct B_Controller_Algorithm_Local_T {
  uint32_T Product;
  uint32_T Delay;
  uint32_T Sum[4];
  real32_T Merge[3];
  real32_T Switch2;
  real32_T Product1;
  real32_T Switch;
  real32_T Switch2_j;
  real32_T Sum3;
  real32_T Sum2;
  real32_T algDD_o1;
  real32_T Unary_Minus;
  real32_T ADCOffsetPhA;
  real32_T ADCOffsetPhB;
  real32_T Lookup[4];
  real32_T Convert_back;
  real32_T Convert_back_c;
  real32_T DataTypeConversion2;
  real32_T velocityControlError;
  real32_T Add;
  real32_T Sum1;
  real32_T Switch1;
  real32_T sum_alpha;
  real32_T Switch_n[2];
  real32_T Atan2;
  real32_T preSatCurrent;
  real32_T Min;
  real32_T Sum_i;
  real32_T Add1;
  real32_T Add2;
  real32_T Add3;
  int32_T Sum1_b;
  int32_T Sum1_d;
  int32_T Sum_b;
  int32_T Sum_b3;
  int32_T Sum_g;
  int32_T Sum_p;
  int32_T DataTypeConversion;
  int32_T Gain;
  int32_T Switch_p;
  int32_T Bias;
  int32_T Delay_f;
  int32_T Delay_c;
  int32_T Add1_f;
  int32_T Add1_l;
  int32_T SpeedGain;
  int32_T Sum1_l;
  int32_T Switch_a;
  int32_T Saturation;
  int32_T Sum3_e;
  int32_T Sum3_h;
  int32_T Delay1;
  int32_T Delay1_p;
  int32_T Integrator;
  int32_T Integrator_b;
  int32_T DataTypeConversion_g[4];
  int32_T Sum1_g;
  int32_T elect2mech;
  int32_T Switch_c;
  int32_T Slope[3];
  int32_T Saturation_c[3];
  modeSchedulerEnum mode;
  int32_T Switch1_g[2];
  uint16_T Register10;
  uint16_T Register20;
  uint16_T pwmCompare[3];
  uint16_T Register18;
  uint16_T Register19;
  uint16_T Register24;
  uint16_T Register4;
  uint16_T Current_Fixed_Point1;
  uint16_T Current_Fixed_Point2;
  uint16_T QuadHandle1;
  uint16_T CastU16En16;
  uint16_T Get_Integer;
  uint16_T p50mA;
  uint16_T Amp50;
  uint16_T CastU16En16_o;
  uint16_T Register12;
  int16_T Register15;
  int16_T Register22;
  int16_T SignCorrected;
  int16_T LookUpTable;
  int16_T Negate;
  int16_T Negate_f;
  int16_T Register23;
  int16_T LookUpTable_a;
  int16_T Kalpha;
  int16_T Kbeta;
  int16_T Current_Fixed_Point;
  int16_T Voltage_Fixed_Point[3];
  int16_T Switch_px[3];
  int16_T TmpSignalConversionAtGainInport[3];
  int16_T Register2;
  int16_T Min_p;
  int16_T Kb;
  int16_T Add1_n;
  int16_T one_by_two;
  int16_T Add3_p;
  int16_T Sat[3];
  int16_T DataTypeConversion2_a;
  int16_T Add2_l;
  int16_T Convert_uint16;
  int16_T Convert_uint16_e;
  int16_T Sat_j[2];
  uint8_T ustor4thQuad;
  int8_T Switch2_o;
  int8_T Switch1_k;
  boolean_T GTEp75;
  boolean_T Register8;
  boolean_T enableInverter;
  boolean_T Open_Loop;
  boolean_T closedLoop;
  boolean_T adcCalibration;
  boolean_T Flag;
  boolean_T error;
  boolean_T Register1;
  boolean_T UpperRelop;
  boolean_T LTEp25;
  boolean_T LogicalOperator;
  boolean_T Compare;
  boolean_T Register7;
  boolean_T enableInverter_d;
};

struct PrevZCX_Controller_Algorithm_Local_T {
  ZCSigState Delay1_Reset_ZCE;
  ZCSigState Delay_Reset_ZCE;
  ZCSigState Delay1_Reset_ZCE_e;
  ZCSigState Delay_Reset_ZCE_m;
};

struct ConstP_Controller_Algorithm_Local_T {
  real32_T sine_table_values_Value[1002];
  int16_T pooled11[33];
};

struct ExternalUPtrs_Controller_Algorithm_Local_T {
  boolean_T *Motor_On;
  commandTypeEnum *Command_Type;
  real32_T *Velocity_Command;
  uint16_T *ADC_Count[2];
  boolean_T *Encoder_Index_Found;
  uint16_T *Encoder_Count;
};

struct ExtY_Controller_Algorithm_Local_T {
  modeSchedulerEnum *Controller_Mode;
  boolean_T *Inverter_Enable;
  uint16_T *PWM_Compare[3];
};

extern const ConstP_Controller_Algorithm_Local_T
  Controller_Algorithm_Local_ConstP;

#endif
