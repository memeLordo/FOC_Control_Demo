{
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

    {
      int_T i;
      static PrevZCX_Controller_Algorithm_Local_T
        Controller_Algorithm_Local_PrevZCX;
      ZCSigState *zc = (ZCSigState *) &Controller_Algorithm_Local_PrevZCX;
      _ssSetPrevZCSigState(rts, zc);
      for (i = 0; i < 4; i++) {
        zc[i] = UNINITIALIZED_ZCSIG;
      }
    }

    ssSetChecksumVal(rts, 0, 4251703258U);
    ssSetChecksumVal(rts, 1, 3309979479U);
    ssSetChecksumVal(rts, 2, 1769019106U);
    ssSetChecksumVal(rts, 3, 2318490014U);
  }
}
