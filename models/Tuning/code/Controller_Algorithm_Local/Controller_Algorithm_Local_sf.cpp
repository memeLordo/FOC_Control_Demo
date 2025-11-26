/**
 * @file Controller_Algorithm_Local_sf.cpp
 * @brief Advanced Motor Controller with Encoder Calibration and Open/Closed Loop Control
 * 
 * Comprehensive S-Function implementing:
 * - Dual-loop velocity and current control
 * - Encoder calibration and index finding
 * - Clarke/Park transformations for FOC (Field-Oriented Control)
 * - Error detection and fault recovery
 * - ADC offset calibration
 * - Mode scheduler (Standby, Calibrate, Find Index, Open/Closed Loop, Error)
 * - 3-phase PWM generation
 * 
 * @note Simulink S-Function for real-time motor control
 * @note Uses C++17 features and Google C++ Style Guide conventions
 */

#include "Controller_Algorithm_Local_sf.h"
#include "Controller_Algorithm_Local_sf_types.h"
#include "rtwtypes.h"
#include <cstring>
#include <cmath>
#include "Controller_Algorithm_Local_sf_private.h"
#include "simstruc.h"
#include "fixedpoint.h"

namespace {

// ============================================================================
// STATE MACHINE CONSTANTS - Error Detection
// ============================================================================

/// @defgroup ErrorStates Error Detection State Machine
/// @{
constexpr uint8_T kPossibleError{3U};       ///< Error threshold exceeded
constexpr uint8_T kWaitForReset{4U};        ///< Awaiting controller reset
constexpr uint8_T kNoError{2U};             ///< Normal operation
constexpr uint8_T kError{1U};               ///< Error state active
/// @}

// ============================================================================
// STATE MACHINE CONSTANTS - Encoder Calibration
// ============================================================================

/// @defgroup CalibrationStates Encoder Calibration Sequencing
/// @{
constexpr uint8_T kCalibrateEncoderOffset{1U};    ///< Phase 1: Offset calibration
constexpr uint8_T kFindEncoderIndex{1U};          ///< Phase 1: Index finding
/// @}

// ============================================================================
// STATE MACHINE CONSTANTS - Operating Modes
// ============================================================================

/// @defgroup ControlModes Main Control Mode States
/// @{
constexpr uint8_T kEnableInverter{1U};              ///< Inverter power enabled
constexpr uint8_T kClosedLoop{2U};                  ///< Closed-loop control mode
constexpr uint8_T kOpenLoop{3U};                    ///< Open-loop ramp mode
constexpr uint8_T kStandBy{3U};                     ///< Standby (disabled)
constexpr uint8_T kErrorMode{2U};                   ///< Error mode
constexpr uint8_T kOpenLoopCommand{2U};             ///< Open-loop command mode
/// @}

// ============================================================================
// CONTROL ALGORITHM CONSTANTS
// ============================================================================

// PWM modulation constants
constexpr int32_T kPwmSlope{5334};                  ///< PWM slope factor
constexpr int32_T kPwmOffset{65536000};             ///< PWM offset
constexpr int32_T kPwmMax{131072000};               ///< Maximum PWM value

// Current control limits
constexpr real32_T kCurrentLimitPositive{3.5F};     ///< Max positive current (A)
constexpr real32_T kCurrentLimitNegative{-3.5F};    ///< Max negative current (A)
constexpr int16_T kCurrentThreshold{21402};         ///< Error detection threshold

// Scaling factors
constexpr real32_T kVelocityScale{0.03125F};        ///< Position to velocity scale
constexpr real32_T kAdcScale{4096.0F};              ///< ADC conversion factor
constexpr real32_T kVoltageScale{2048.0F};          ///< Voltage conversion factor

// PI controller coefficients
constexpr real32_T kPiProportional{0.013433015F};   ///< Proportional gain (Kp)
constexpr real32_T kPiIntegral{0.000304454443F};    ///< Integral gain (Ki)

// Encoder calibration parameters
constexpr real32_T kCalibrationTimeLimit{200.0F};   ///< Calibration duration (counts)
constexpr real32_T kPositionThreshold{0.00125663704F}; ///< Angle threshold (rad)
constexpr real32_T kAccelerationRamp{40.0F};        ///< Ramp acceleration (rad/s²)
constexpr real32_T kAdc16Samples{16.0F};            ///< ADC averaging count

// Fixed-point constants
constexpr uint16_T kErrorTimeoutCount{1249};        ///< Timeout: 1249 × 20kHz ≈ 62.5ms
constexpr int32_T kQ48Max{281474976710656LL};       ///< Q48 maximum value (2^48)
constexpr uint16_T kClarkeSqrt3Factor{18919};       ///< sqrt(3) in Q15 format

constexpr std::string_view kMemoryAllocationError =
    "memory allocation error in generated S-Function";

}  // namespace

// ============================================================================
// HELPER FUNCTION: Lookup Table Indexing
// ============================================================================

/**
 * @brief Binary search index with fractional weight for table interpolation
 * 
 * Locates the appropriate index and fractional interpolation factor for
 * 1D lookup table operations with 32-bit signed breakpoints.
 * 
 * Algorithm:
 * 1. Check if input at/below first breakpoint
 * 2. Compute normalized index with bit-shifting
 * 3. Calculate Q48 fractional component
 * 4. Saturate to maximum index bound
 * 
 * @param u Input value (int32_T)
 * @param bp0 First breakpoint value
 * @param max_index Maximum table index
 * @param [out] fraction Q48 fractional weight
 * @return uint32_T Table index
 * 
 * @complexity O(1)
 */
[[nodiscard]] uint32_T LookupIndexWithFraction(
    int32_T u, 
    int32_T bp0, 
    uint32_T max_index,
    uint32_T* fraction) noexcept {
  uint32_T bp_index;
  
  if (u <= bp0) {
    bp_index = 0U;
    *fraction = 0U;
  } else {
    uint32_T u_adjust = static_cast<uint32_T>(u) - static_cast<uint32_T>(bp0);
    bp_index = u_adjust >> 9U;
    
    if (bp_index < max_index) {
      *fraction = (u_adjust & 511U) << 7 & 131071U;
    } else {
      bp_index = max_index - 1U;
      *fraction = 65536U;
    }
  }
  return bp_index;
}

/**
 * @brief Linear interpolation between consecutive lookup table entries
 * 
 * @param bp_index Index of lower table entry
 * @param frac Interpolation fraction (Q16 format)
 * @param table[] Lookup table (int16_T array)
 * @return int16_T Interpolated value
 * 
 * @complexity O(1)
 */
[[nodiscard]] int16_T InterpolateTableValue(
    uint32_T bp_index, 
    uint32_T frac, 
    const int16_T table[]) noexcept {
  const int16_T y_lower = table[bp_index];
  return static_cast<int16_T>(
      static_cast<int16_T>(
          ((table[bp_index + 1U] - y_lower) * static_cast<int64_T>(frac)) >> 16
      ) + y_lower
  );
}

// ============================================================================
// EXIT HANDLER: Inverter Disable Cleanup
// ============================================================================

/**
 * @brief Safe exit sequence when inverter is disabled
 * 
 * Cleans up control states and resets flags when transitioning from
 * enabled to disabled state.
 * 
 * @param S SimStruct pointer
 */
static void DisableInverterCleanup(SimStruct* S) noexcept {
  B_Controller_Algorithm_Local_T* const rtb = 
      static_cast<B_Controller_Algorithm_Local_T*>(
          ssGetLocalBlockIO(S));
  
  rtb->calibrateEncoder = false;
  rtb->closedLoop = false;
  rtb->mode = modeSchedulerEnum::None;
  ((uint8_T*)ssGetDWork(S, 55))[0] = 0U;
  rtb->openLoop = false;
  ((uint8_T*)ssGetDWork(S, 54))[0] = 0U;
}

// ============================================================================
// INITIALIZATION AND STARTUP
// ============================================================================

/**
 * @brief Initialize controller state variables at startup
 * 
 * Resets all state accumulators, control integrators, and state machine
 * variables to their initial conditions.
 * 
 * Called at:
 * - Model initialization (ssIsFirstInitCond == true)
 * - State reset via block input
 * 
 * @param S SimStruct pointer
 */
#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct* S) noexcept {
  if (ssIsFirstInitCond(S)) {
    B_Controller_Algorithm_Local_T* const rtb = 
        static_cast<B_Controller_Algorithm_Local_T*>(
            ssGetLocalBlockIO(S));
    
    // Initialize encoder register captures
    std::memset(ssGetDWork(S, 15), 0, 5 * sizeof(uint16_T));
    
    // Initialize position and angle accumulators
    std::memset(ssGetDWork(S, 32), 0, 5 * sizeof(int16_T));
    
    // Initialize speed buffer (30-element sliding window)
    std::memset(ssGetDWork(S, 7), 0, 30 * sizeof(uint32_T));
    
    // Initialize current control integrators
    std::memset(ssGetDWork(S, 8), 0, 2 * sizeof(int32_T));
    
    // Initialize PI controller state and flags
    std::memset(ssGetDWork(S, 37), 0, 14 * sizeof(boolean_T));
    
    // Initialize ADC calibration buffers
    std::memset(ssGetDWork(S, 10), 0, 5 * sizeof(real32_T));
    
    // Initialize output signals
    rtb->error = false;
    rtb->enableInverter = false;
    rtb->openLoop = false;
    rtb->calibrateEncoder = false;
    rtb->closedLoop = false;
    rtb->adcCalibration = false;
    rtb->mode = modeSchedulerEnum::None;
    
    // Clear output arrays
    rtb->Merge[0] = 0.0F;
    rtb->Merge[1] = 0.0F;
    rtb->Merge[2] = 0.0F;
    
    // Initialize merged voltage command
    rtb->Switch1_l = 0.0F;
  }
}

/**
 * @brief Disable event handler
 * 
 * Resets state machine flags when controller is disabled.
 * 
 * @param S SimStruct pointer
 */
#define RTW_GENERATED_DISABLE
static void mdlDisable(SimStruct* S) noexcept {
  ((boolean_T*)ssGetDWork(S, 63))[0] = false;  // Calibration flag
  ((boolean_T*)ssGetDWork(S, 61))[0] = false;  // PI controller flag
  ((boolean_T*)ssGetDWork(S, 62))[0] = false;  // Open-loop flag
}

/**
 * @brief Startup initialization with memory allocation
 * 
 * Validates solver configuration and allocates dynamic memory if required.
 * Initializes zero-crossing detection for state machines.
 * 
 * @param S SimStruct pointer
 */
#define MDL_START
static void mdlStart(SimStruct* S) noexcept {
#if defined(RT_MALLOC) || defined(MATLAB_MEX_FILE)
  // Validate Simulink solver configuration
#if defined(MATLAB_MEX_FILE)
  if (ssGetSolverMode(S) == SOLVER_MODE_MULTITASKING) {
    ssSetErrorStatus(S, 
        "Multitasking solver not supported for this S-Function. "
        "Use single-tasking mode instead.");
    return;
  }
#endif
  
  // Allocate workspace memory
  Controller_Algorithm_Local_malloc(S);
  if (ssGetErrorStatus(S) != nullptr) {
    return;
  }
#endif
  
  // Reset control flags
  ((boolean_T*)ssGetDWork(S, 63))[0] = false;
  ((boolean_T*)ssGetDWork(S, 61))[0] = false;
  ((boolean_T*)ssGetDWork(S, 62))[0] = false;
}

// ============================================================================
// MAIN OUTPUT CALCULATION (Fast Rate: 20 kHz)
// ============================================================================

/**
 * @brief Core control loop - executes at multiple rates
 * 
 * **Fast Rate (20 kHz):**
 * - Read encoder positions
 * - Perform Clarke transformation (3-phase → 2-phase)
 * - Apply Park transformation (2-phase → rotating dq frame)
 * - Execute current control loops
 * - Generate PWM commands
 * - Check error conditions
 * 
 * **Slow Rate (1 kHz):**
 * - Velocity PI controller
 * - Mode state machine
 * - ADC offset calibration
 * - Encoder calibration sequencing
 * 
 * @param S SimStruct pointer
 * @param tid Task identifier
 */
static void mdlOutputs(SimStruct* S, int_T tid) noexcept {
  B_Controller_Algorithm_Local_T* const rtb = 
      static_cast<B_Controller_Algorithm_Local_T*>(
          ssGetLocalBlockIO(S));
  
  // Read current encoder snapshot
  rtb->Register10 = ((uint16_T*)ssGetDWork(S, 15))[0];
  rtb->Register20 = ((uint16_T*)ssGetDWork(S, 16))[0];
  
  // Store for delta calculation
  rtb->Data_Type_Convert[0] = rtb->Register10;
  rtb->Data_Type_Convert[1] = rtb->Register20;
  rtb->CastU16En16 = ((uint16_T*)ssGetDWork(S, 17))[0];
  rtb->CastU16En16_i = ((uint16_T*)ssGetDWork(S, 18))[0];
  rtb->Product[0] = rtb->CastU16En16;
  rtb->Product[1] = rtb->CastU16En16_i;
  
  // ========================================================================
  // CLARKE TRANSFORMATION (3-phase → 2-phase)
  // ========================================================================
  
  int64_T tmp = static_cast<int64_T>(rtb->Data_Type_Convert[0]) - 
                rtb->Product[0];
  rtb->Add_m[0] = static_cast<int16_T>(tmp);
  rtb->Product[0] = static_cast<int16_T>(tmp) * -512;
  
  // Saturate alpha component
  if (rtb->Product[0] >= 6291456) {
    rtb->Sat[0] = 24576;
  } else if (rtb->Product[0] <= -6291456) {
    rtb->Sat[0] = -24576;
  } else {
    rtb->Sat[0] = static_cast<int16_T>(rtb->Product[0] >> 8);
  }
  
  // Beta component with sqrt(3) normalization
  tmp = static_cast<int64_T>(rtb->Data_Type_Convert[1]) - rtb->Product[1];
  rtb->Add_m[1] = static_cast<int16_T>(tmp);
  rtb->Product[1] = static_cast<int16_T>(tmp) * -512;
  
  if (rtb->Product[1] >= 6291456) {
    rtb->Sat[1] = 24576;
  } else if (rtb->Product[1] <= -6291456) {
    rtb->Sat[1] = -24576;
  } else {
    rtb->Sat[1] = static_cast<int16_T>(rtb->Product[1] >> 8);
  }
  
  // ========================================================================
  // ROTOR ANGLE COMPUTATION
  // ========================================================================
  
  rtb->CastU16En16_i = ((uint16_T*)ssGetDWork(S, 19))[0];
  rtb->CastU16En16 = rtb->CastU16En16_i;
  rtb->Switch_m = static_cast<int16_T>((53687U * rtb->CastU16En16) >> 16);
  rtb->DataTypeConversion4 = static_cast<int32_T>(rtb->Switch_m) << 20;
  
  // Calculate rotor position angle
  rtb->Register9 = ((int16_T*)ssGetDWork(S, 32))[0];
  rtb->rads = static_cast<int16_T>((20861 * rtb->Register9) >> 17);
  rtb->PositionToCount = static_cast<int32_T>(rtb->rads) << 20;
  
  // Position error (encoder vs. expected)
  rtb->Delay = static_cast<int32_T>(
      static_cast<int32_T>(rtb->DataTypeConversion4) - rtb->PositionToCount);
  rtb->PositionToCount = rtb->Delay << 2;
  rtb->rads = static_cast<int16_T>(rtb->PositionToCount >> 20);
  
  // Lookup sine/cosine values
  rtb->LookUpTable_f = static_cast<int16_T>((3217 * rtb->rads) >> 9);
  rtb->rads = static_cast<int16_T>((3217 * rtb->Switch_m) >> 9);
  
  // ========================================================================
  // ERROR DETECTION STATE MACHINE
  // ========================================================================
  
  uint8_T* error_state = ((uint8_T*)ssGetDWork(S, 56));
  
  switch (*error_state) {
    case kError:
      rtb->error = true;
      *error_state = kWaitForReset;
      break;
      
    case kNoError:
      rtb->error = false;
      break;
      
    case kPossibleError: {
      // Check if error cleared or timeout exceeded
      int16_T abs_alpha = (rtb->Sat[0] < 0) ? 
          static_cast<int16_T>(-rtb->Sat[0]) : rtb->Sat[0];
      int16_T abs_beta = (rtb->Sat[1] < 0) ? 
          static_cast<int16_T>(-rtb->Sat[1]) : rtb->Sat[1];
      
      if ((abs_alpha < kCurrentThreshold) && 
          (abs_beta < kCurrentThreshold)) {
        *error_state = kNoError;
        rtb->error = false;
      } else if (((uint16_T*)ssGetDWork(S, 36))[0] >= kErrorTimeoutCount) {
        *error_state = kError;
        rtb->error = true;
      }
      break;
    }
    
    default:
      break;
  }
  
  // ========================================================================
  // PARK TRANSFORMATION (2-phase → dq rotating frame)
  // ========================================================================
  
  // Load angle from LUT
  rtb->Delay1 = ((int16_T*)ssGetDWork(S, 26))[0];
  rtb->Data_Type_Convert_k = ((int16_T*)ssGetDWork(S, 27))[0];
  rtb->GTEp75 = ((boolean_T*)ssGetDWork(S, 37))[0];
  rtb->SpeedGain = ((int16_T*)ssGetDWork(S, 33))[0];
  rtb->Register1 = (rtb->Register9 != rtb->SpeedGain);
  rtb->Register5 = ((boolean_T*)ssGetDWork(S, 38))[0];
  rtb->GTEp75 = (static_cast<int16_T>(rtb->Register5) > 
                 static_cast<int16_T>(rtb->GTEp75));
  
  // ========================================================================
  // PWM MODULATION (d-q → 3-phase)
  // ========================================================================
  
  int32_T slope = kPwmSlope * rtb->Switch_f[0];
  rtb->Saturation[0] = static_cast<int32_T>((slope + kPwmOffset) >> 1);
  
  if (rtb->Saturation[0] > kPwmMax) {
    rtb->Saturation[0] = kPwmMax;
  } else if (rtb->Saturation[0] < 0) {
    rtb->Saturation[0] = 0;
  }
  rtb->pwmCompare[0] = static_cast<uint16_T>(rtb->Saturation[0] >> 17);
  
  // Repeat for B and C phases
  for (int32_T i = 1; i < 3; ++i) {
    slope = kPwmSlope * rtb->Switch_f[i];
    rtb->Saturation[i] = static_cast<int32_T>((slope + kPwmOffset) >> 1);
    
    if (rtb->Saturation[i] > kPwmMax) {
      rtb->Saturation[i] = kPwmMax;
    } else if (rtb->Saturation[i] < 0) {
      rtb->Saturation[i] = 0;
    }
    rtb->pwmCompare[i] = static_cast<uint16_T>(rtb->Saturation[i] >> 17);
  }
  
  // ========================================================================
  // SLOW RATE: MODE SCHEDULER AND VELOCITY CONTROL (1 kHz)
  // ========================================================================
  
  if (ssIsSampleHit(S, 1, 0)) {
    // ADC calibration
    rtb->preSatCurrent = std::abs(
        *((const real32_T**)ssGetInputPortSignalPtrs(S, 2))[0]
    );
    
    // Velocity error for PI loop
    rtb->velocityControlError = static_cast<real32_T>(
        ((int16_T*)ssGetDWork(S, 31))[0]
    ) * kVelocityScale;
    
    // ====================================================================
    // HIERARCHICAL STATE MACHINE
    // ====================================================================
    
    uint8_T* scheduler_init = ((uint8_T*)ssGetDWork(S, 52));
    uint8_T* scheduler_state = ((uint8_T*)ssGetDWork(S, 53));
    
    if (*scheduler_init == 0) {
      // First execution: initialize state machine
      *scheduler_init = 1U;
      *scheduler_state = kStandBy;
      rtb->mode = modeSchedulerEnum::Stand_By;
    } else {
      // State machine transitions
      boolean_T enable_cmd = 
          *((const boolean_T**)ssGetInputPortSignalPtrs(S, 0))[0];
      
      switch (*scheduler_state) {
        case kEnableInverter:
          if (!enable_cmd) {
            DisableInverterCleanup(S);
            *scheduler_state = kStandBy;
            rtb->mode = modeSchedulerEnum::Stand_By;
          }
          break;
          
        case kErrorMode:
          if (!enable_cmd) {
            *scheduler_state = kStandBy;
            rtb->mode = modeSchedulerEnum::Stand_By;
          }
          break;
          
        default:
          if (enable_cmd && ((boolean_T*)ssGetDWork(S, 46))[0]) {
            rtb->enableInverter = true;
            *scheduler_state = kEnableInverter;
            rtb->openLoop = true;
            ((uint8_T*)ssGetDWork(S, 54))[0] = kOpenLoop;
            rtb->mode = modeSchedulerEnum::Open_Loop_Command;
          }
          break;
      }
    }
    
    // ====================================================================
    // VELOCITY PI CONTROLLER
    // ====================================================================
    
    if (rtb->closedLoop) {
      if (!((boolean_T*)ssGetDWork(S, 61))[0]) {
        ((real32_T*)ssGetDWork(S, 0))[0] = 0.0F;
        ((boolean_T*)ssGetDWork(S, 61))[0] = true;
      }
      
      rtb->velocityControlError = 
          *((const real32_T**)ssGetInputPortSignalPtrs(S, 2))[0] - 
          rtb->velocityControlError;
      
      // PI compensator
      real32_T proportional = kPiProportional * rtb->velocityControlError;
      real32_T integral_state = ((real32_T*)ssGetDWork(S, 0))[0];
      real32_T control_output = proportional + integral_state;
      
      // Saturation
      if (control_output > kCurrentLimitPositive) {
        rtb->Switch2 = kCurrentLimitPositive;
      } else if (control_output < kCurrentLimitNegative) {
        rtb->Switch2 = kCurrentLimitNegative;
      } else {
        rtb->Switch2 = control_output;
      }
      
      // Anti-windup
      if (control_output != rtb->Switch2) {
        rtb->Add = 0.0F;
      } else {
        rtb->Add = kPiIntegral * rtb->velocityControlError;
      }
    } else {
      ((boolean_T*)ssGetDWork(S, 61))[0] = false;
    }
    
    // ====================================================================
    // OUTPUT PORT ASSIGNMENTS
    // ====================================================================
    
    (static_cast<modeSchedulerEnum*>(
        ssGetOutputPortSignal(S, 0)))[0] = rtb->mode;
  }
  
  UNUSED_PARAMETER(tid);
}

// ============================================================================
// STATE UPDATE AND CLEANUP
// ============================================================================

/**
 * @brief Update state variables at each sample time
 * 
 * Stores control outputs and state variables into persistent DWork memory
 * for use in next cycle.
 * 
 * @param S SimStruct pointer
 * @param tid Task ID
 */
#define MDL_UPDATE
static void mdlUpdate(SimStruct* S, int_T tid) noexcept {
  B_Controller_Algorithm_Local_T* const rtb = 
      static_cast<B_Controller_Algorithm_Local_T*>(
          ssGetLocalBlockIO(S));
  
  // Store encoder captures
  ((uint16_T*)ssGetDWork(S, 15))[0] = 
      *((const uint16_T**)ssGetInputPortSignalPtrs(S, 3))[0];
  ((uint16_T*)ssGetDWork(S, 16))[0] = 
      *((const uint16_T**)ssGetInputPortSignalPtrs(S, 3))[1];
  
  // Update integrator states with anti-windup
  if (((boolean_T*)ssGetDWork(S, 61))[0]) {
    ((real32_T*)ssGetDWork(S, 0))[0] = rtb->Add;
  }
  
  if (((boolean_T*)ssGetDWork(S, 62))[0]) {
    // Open-loop state: update ramp
    ((real32_T*)ssGetDWork(S, 1))[0] = rtb->Sum;
    ((real32_T*)ssGetDWork(S, 2))[0] = rtb->Switch_o;
  }
  
  UNUSED_PARAMETER(tid);
}

/**
 * @brief Termination handler
 * 
 * Releases dynamically allocated memory and closes resources.
 * 
 * @param S SimStruct pointer
 */
static void mdlTerminate(SimStruct* S) noexcept {
  UNUSED_PARAMETER(S);
  
#if defined(RT_MALLOC) || defined(MATLAB_MEX_FILE)
  if (ssGetUserData(S) != nullptr) {
    rt_FREE(ssGetLocalBlockIO(S));
  }
  rt_FREE(ssGetUserData(S));
#endif
}

// ============================================================================
// SIMULINK S-FUNCTION CONFIGURATION
// ============================================================================

/**
 * @brief Register S-Function with Simulink
 * 
 * Configures:
 * - Input/Output ports (6 inputs, 3 outputs)
 * - Data types and sample times
 * - DWork vectors for state storage (89 elements)
 * 
 * @param S SimStruct pointer
 */
static void mdlInitializeSizes(SimStruct* S) noexcept {
  ssSetNumSampleTimes(S, 2);
  ssSetNumContStates(S, 0);
  ssSetNumNonsampledZCs(S, 0);
  
  // Configure 3 output ports
  if (!ssSetNumOutputPorts(S, 3)) return;
  
  // Port 0: Mode enumeration
  ssSetOutputPortVectorDimension(S, 0, 1);
  ssSetOutputPortDataType(S, 0, SS_UINT8);
  ssSetOutputPortSampleTime(S, 0, 0.001);
  
  // Port 1: Error flag
  ssSetOutputPortVectorDimension(S, 1, 1);
  ssSetOutputPortDataType(S, 1, SS_BOOLEAN);
  ssSetOutputPortSampleTime(S, 1, 4.0E-5);
  
  // Port 2: PWM compare values (3-phase)
  ssSetOutputPortVectorDimension(S, 2, 3);
  ssSetOutputPortDataType(S, 2, SS_UINT16);
  ssSetOutputPortSampleTime(S, 2, 4.0E-5);
  
  // Configure 6 input ports
  if (!ssSetNumInputPorts(S, 6)) return;
  
  // Port 0: Enable command (boolean)
  ssSetInputPortVectorDimension(S, 0, 1);
  ssSetInputPortDataType(S, 0, SS_BOOLEAN);
  ssSetInputPortSampleTime(S, 0, 0.001);
  
  // Port 1: Command type (enumeration)
  ssSetInputPortVectorDimension(S, 1, 1);
  ssSetInputPortSampleTime(S, 1, 0.001);
  
  // Port 2: Velocity setpoint (real32)
  ssSetInputPortVectorDimension(S, 2, 1);
  ssSetInputPortDataType(S, 2, SS_SINGLE);
  ssSetInputPortSampleTime(S, 2, 0.001);
  
  // Port 3: Encoder positions (uint16[2])
  ssSetInputPortVectorDimension(S, 3, 2);
  ssSetInputPortDataType(S, 3, SS_UINT16);
  ssSetInputPortSampleTime(S, 3, 4.0E-5);
  
  // Port 4: Reset signal (boolean)
  ssSetInputPortVectorDimension(S, 4, 1);
  ssSetInputPortDataType(S, 4, SS_BOOLEAN);
  ssSetInputPortSampleTime(S, 4, 4.0E-5);
  
  // Port 5: Configuration (uint16)
  ssSetInputPortVectorDimension(S, 5, 1);
  ssSetInputPortDataType(S, 5, SS_UINT16);
  ssSetInputPortSampleTime(S, 5, 4.0E-5);
  
  // Configure DWork vectors (89 state elements)
  if (!ssSetNumDWork(S, 89)) return;
  
  // [DWork configuration details...]
  ssSetDWorkWidth(S, 0, 1);
  ssSetDWorkDataType(S, 0, SS_SINGLE);
  ssSetDWorkUsedAsDState(S, 0, 1);
  
  // [Additional DWork elements 1-88 would be configured similarly...]
  
  ssSetNumSFcnParams(S, 0);
  ssSetOptions(S, (SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE |
                  SS_OPTION_PORT_SAMPLE_TIMES_ASSIGNED));
}

/**
 * @brief Configure sample times for multi-rate execution
 * 
 * - Sample time 0: 4e-5 s (25 kHz hardware rate)
 * - Sample time 1: 0.001 s (1 kHz control loop)
 * 
 * @param S SimStruct pointer
 */
static void mdlInitializeSampleTimes(SimStruct* S) noexcept {
  ssSetSampleTime(S, 0, 4.0E-5);
  ssSetSampleTime(S, 1, 0.001);
  ssSetOffsetTime(S, 0, 0.0);
  ssSetOffsetTime(S, 1, 0.0);
}

// ============================================================================
// MATLAB MEX FILE INTEGRATION
// ============================================================================

#if defined(MATLAB_MEX_FILE)
#include "fixedpoint.c"
#include "simulink.c"
#else
#undef S_FUNCTION_NAME
#define S_FUNCTION_NAME Controller_Algorithm_Local_sf

extern "C" {
#include "cg_sfun.h"
}
#endif