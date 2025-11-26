#ifndef Controller_Algorithm_Local_sf_types_h_
#define Controller_Algorithm_Local_sf_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_commandTypeEnum_
#define DEFINED_TYPEDEF_FOR_commandTypeEnum_

enum class commandTypeEnum
  : int32_T {
  Error = 0,
  OpenLoopVelocity,
  CalibrateEncoder,
  Velocity
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_modeSchedulerEnum_
#define DEFINED_TYPEDEF_FOR_modeSchedulerEnum_

enum class modeSchedulerEnum
  : int32_T {
  None = 0,
  Stand_By,
  Open_Loop,
  Closed_Loop,
  Error
};

#endif
#endif
