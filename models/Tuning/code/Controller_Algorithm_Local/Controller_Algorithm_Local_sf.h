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
#include <cstring>
#include "rt_defines.h"

struct B_Controller_Algorithm_Local_T {
  uint32_T DataTypeConversion4;
  uint32_T Delay;
  uint32_T PositionToCount;
  real32_T Position_Single;
  real32_T Merge[3];
  real32_T Switch2;
  real32_T Product1;
  real32_T Switch;
  real32_T Switch_o;
  real32_T accleration;
  real32_T Unary_Minus;
  real32_T Switch1;
  real32_T Add2;
  real32_T Add1;
  real32_T Difference;
  real32_T Delay_i;
  real32_T Switch_d;
  real32_T Switch_i;
  real32_T Switch1_l;
  real32_T Switch2_m;
  real32_T wrapPositionNegPii;
  real32_T wrapPositionPi;
  real32_T negativeIn;
  real32_T DataTypeConversion[2];
  real32_T ADCOffsetPhA;
  real32_T ADCOffsetPhB;
  real32_T velocityControlError;
  real32_T preSatCurrent;
  real32_T Delay_d;
  real32_T Add;
  real32_T Add1_l;
  real32_T Add_c;
  real32_T Add_d;
  real32_T Gain1;
  real32_T Add_n;
  real32_T Sum;
  real32_T Add1_g;
  int32_T Integrator;
  int32_T Integrator_n;
  int32_T currentControlError;
  int32_T Switch_h;
  int32_T Slope[3];
  int32_T Saturation[3];
  int32_T p50mA;
  int32_T Saturation_i;
  int32_T QuadHandle1;
  int32_T Amp50;
  int32_T QuadHandle1b;
  int32_T p75mA;
  int32_T Amp75;
  int32_T QuadHandle1a;
  int32_T Amp25;
  int32_T p25mA;
  modeSchedulerEnum mode;
  int32_T Data_Type_Convert[2];
  int32_T Add_m[2];
  int32_T Product[2];
  int32_T Switch_j;
  uint16_T Register10;
  uint16_T Register20;
  uint16_T pwmCompare[3];
  uint16_T Register12;
  uint16_T Register18;
  uint16_T Register19;
  uint16_T Register24;
  uint16_T Register4;
  uint16_T Current_Fixed_Point1;
  uint16_T Current_Fixed_Point2;
  uint16_T CastU16En16;
  uint16_T CastU16En16_i;
  int16_T Register15;
  int16_T Delay1;
  int16_T LookUpTable;
  int16_T Negate;
  int16_T Register23;
  int16_T Register22;
  int16_T Data_Type_Convert_k;
  int16_T Register9;
  int16_T Register13;
  int16_T Offset_Fixed_Point;
  int16_T Switch_m;
  int16_T LookUpTable_f;
  int16_T rads;
  int16_T SpeedGain;
  int16_T Sat[2];
  int16_T Current_Fixed_Point;
  int16_T Voltage_Fixed_Point[3];
  int16_T Switch_f[3];
  int16_T TmpSignalConversionAtGainInport[3];
  int16_T Product_j;
  int16_T Min;
  int16_T Kb;
  int16_T Add1_p;
  int16_T one_by_two;
  int16_T Add3;
  int16_T Sat_e[3];
  int16_T Register2;
  int16_T Add2_e;
  uint8_T ustor4thQuad;
  int8_T Switch2_k;
  int8_T Switch1_j;
  boolean_T Register5;
  boolean_T LogicalOperator;
  boolean_T Register21;
  boolean_T Register8;
  boolean_T enableInverter;
  boolean_T openLoop;
  boolean_T calibrateEncoder;
  boolean_T closedLoop;
  boolean_T adcCalibration;
  boolean_T Delay1_j;
  boolean_T Greater_Or_Equal;
  boolean_T latchPosition;
  boolean_T Flag;
  boolean_T error;
  boolean_T UpperRelop;
  boolean_T Less_Than;
  boolean_T positionLessThanPi;
  boolean_T positiveDirection;
  boolean_T LTEp25;
  boolean_T stopAccelerating;
  boolean_T GTEp75;
  boolean_T Register1;
  boolean_T stopNegativeAcceleration;
  boolean_T negativeDirection;
  boolean_T Register7;
  boolean_T enableInverter_j;
};

struct ConstB_Controller_Algorithm_Local_T {
  real32_T Unary_Minus;
  real32_T Product;
};

#define Controller_Algorithm_Local_rtC(rts) ((ConstB_Controller_Algorithm_Local_T *) _ssGetConstBlockIO(rts))

struct ConstP_Controller_Algorithm_Local_T {
  int16_T pooled12[33];
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

extern const ConstB_Controller_Algorithm_Local_T
  Controller_Algorithm_Local_Invariant;
extern const ConstP_Controller_Algorithm_Local_T
  Controller_Algorithm_Local_ConstP;


#endif
