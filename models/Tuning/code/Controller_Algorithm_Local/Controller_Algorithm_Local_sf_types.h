#ifndef Controller_Algorithm_Local_sf_types_h_
#define Controller_Algorithm_Local_sf_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_commandTypeEnum_
#define DEFINED_TYPEDEF_FOR_commandTypeEnum_

enum class commandTypeEnum
  : double {
  Error = 0,
  OpenLoopVelocity,
  CalibrateEncoder,
  Velocity
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_modeSchedulerEnum_
#define DEFINED_TYPEDEF_FOR_modeSchedulerEnum_

enum class modeSchedulerEnum
  : double {
  None = 0,
  Stand_By,
  Open_Loop_Command,
  Find_Encoder_Index,
  Calibrate_Encoder_Offset,
  Closed_Loop,
  Error
};

#endif
#endif
