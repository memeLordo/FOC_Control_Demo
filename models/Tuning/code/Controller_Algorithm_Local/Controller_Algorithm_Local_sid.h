{
  extern const ConstB_Controller_Algorithm_Local_T
    Controller_Algorithm_Local_Invariant;

  {
    static LocalS slS;
    LocalS *lS = &slS;
    ssSetUserData(rts, lS);

    {
      static B_Controller_Algorithm_Local_T sfcnB;
      void *b = (real_T *) &sfcnB;
      ssSetLocalBlockIO(rts, b);
      (void) std::memset(b, 0,
                         sizeof(B_Controller_Algorithm_Local_T));

      {
        (static_cast<B_Controller_Algorithm_Local_T *>(ssGetLocalBlockIO(rts))
          )->mode = modeSchedulerEnum::None;
      }
    }

    _ssSetConstBlockIO(rts, &Controller_Algorithm_Local_Invariant);
    ssSetChecksumVal(rts, 0, 2523124112U);
    ssSetChecksumVal(rts, 1, 461042283U);
    ssSetChecksumVal(rts, 2, 1854142380U);
    ssSetChecksumVal(rts, 3, 3846498744U);
  }
}
