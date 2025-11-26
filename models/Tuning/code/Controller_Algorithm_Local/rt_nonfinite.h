

        /*
    * rt_nonfinite.h
    *
        * Code generation for model "Controller_Algorithm_Local_sf".
    *
    * Model version              : 11.26
    * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
        * C++ source code generated on : Tue Nov 25 19:45:31 2025
 * 
 * Target selection: rtwsfcn.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ASIC/FPGA->ASIC/FPGA
 * Emulation hardware selection: 
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
    */


    #ifndef rt_nonfinite_h_
    #define rt_nonfinite_h_



    
#include "rtwtypes.h"


    

    

    

    

    

    

    

    

    

    

    

    

    

    

    
                #ifdef __cplusplus
            extern "C" {
                #endif

                extern real_T rtInf;



                extern real_T rtMinusInf;



                extern real_T rtNaN;



                extern real32_T rtInfF;



                extern real32_T rtMinusInfF;



                extern real32_T rtNaNF;




        extern boolean_T rtIsInf(real_T value);
            extern boolean_T rtIsInfF(real32_T value);
            extern boolean_T rtIsNaN(real_T value);
            extern boolean_T rtIsNaNF(real32_T value);
    
                #ifdef __cplusplus
        } /* extern "C" */
        #endif



    

    

    

    

    

    #endif /* rt_nonfinite_h_ */
