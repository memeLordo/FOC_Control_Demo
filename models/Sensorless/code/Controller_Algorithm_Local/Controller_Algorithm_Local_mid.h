#if defined(MATLAB_MEX_FILE) || defined(RT_MALLOC)

static int_T RegNumInputPorts(SimStruct *S, int_T nInputPorts)
{
  _ssSetNumInputPorts(S,nInputPorts);
  return true;
}

static int_T RegNumOutputPorts(SimStruct *S, int_T nOutputPorts)
{
  _ssSetNumOutputPorts(S,nOutputPorts);
  return true;
}

static int_T FcnSetErrorStatus(const SimStruct *S, DTypeId arg2)
{
  static char_T msg[256];
  if (strlen(ssGetModelName(S)) < 128) {
    snprintf(msg, sizeof(msg),
             "S-function %s does not have a tlc file. It cannot use macros that access regDataType field in simstruct.",
             ssGetModelName(S));
  } else {
    snprintf(msg, sizeof(msg),
             "A S-function does not have a tlc file. It cannot use macros that access regDataType field in simstruct.");
  }

  ssSetErrorStatus(S, msg);
  UNUSED_PARAMETER(arg2);
  return 0;
}

static void * FcnSetErrorStatusWithReturnPtr(const SimStruct *S, DTypeId arg2)
{
  FcnSetErrorStatus(S,0);
  UNUSED_PARAMETER(arg2);
  return 0;
}

static int_T FcnSetErrorStatusWithArgPtr(const SimStruct *S, const void* arg2)
{
  FcnSetErrorStatus(S,0);
  UNUSED_PARAMETER(arg2);
  return 0;
}

#endif

void *Controller_Algorithm_Local_malloc(SimStruct *rts)
{
  LocalS *lS = (LocalS *) malloc(sizeof(LocalS));
  ss_VALIDATE_MEMORY(rts,lS);
  (void) std::memset(static_cast<void *>(lS), 0,
                     sizeof(LocalS));
  ssSetUserData(rts, lS);
  ssSetLocalBlockIO(rts, 0);
  ssSetLocalDefaultParam(rts, 0);
  ssSetLocalNonContDerivSig(rts, 0);

  {
    void *b = malloc(sizeof(B_Controller_Algorithm_Local_T));
    ss_VALIDATE_MEMORY(rts,b);
    ssSetLocalBlockIO(rts, b);
    (void) std::memset(b, 0,
                       sizeof(B_Controller_Algorithm_Local_T));

    {
      (static_cast<B_Controller_Algorithm_Local_T *>(ssGetLocalBlockIO(rts)))
        ->mode = modeSchedulerEnum::None;
    }
  }

  {
    int_T i;
    ZCSigState *zc = (ZCSigState *) malloc(sizeof
      (PrevZCX_Controller_Algorithm_Local_T));
    ss_VALIDATE_MEMORY(rts,zc);
    _ssSetPrevZCSigState(rts, zc);
    for (i = 0; i < 4; i++) {
      zc[i] = UNINITIALIZED_ZCSIG;
    }
  }

  ssSetChecksumVal(rts, 0, 4251703258U);
  ssSetChecksumVal(rts, 1, 3309979479U);
  ssSetChecksumVal(rts, 2, 1769019106U);
  ssSetChecksumVal(rts, 3, 2318490014U);
  return (nullptr);
}
