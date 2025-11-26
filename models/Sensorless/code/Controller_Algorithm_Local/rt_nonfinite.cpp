

        /*
    * rt_nonfinite.cpp
    *
        * Code generation for model "Controller_Algorithm_Local_sf".
    *
    * Model version              : 11.23
    * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
        * C++ source code generated on : Tue Nov 25 05:23:56 2025
 * 
 * Target selection: rtwsfcn.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM 10
 * Emulation hardware selection: 
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
    */




    
#include "rtwtypes.h"

extern "C" {

#include "rt_nonfinite.h"
}

#include "limits"

#include "cmath"


    

    

    

    

    

    

    

    

    

    

    

    

    

    

    
        extern "C" {

                real_T rtNaN { -std::numeric_limits<real_T>::quiet_NaN() };



                real_T rtInf { std::numeric_limits<real_T>::infinity() };



                real_T rtMinusInf { -std::numeric_limits<real_T>::infinity() };



                real32_T rtNaNF { -std::numeric_limits<real32_T>::quiet_NaN() };



                real32_T rtInfF { std::numeric_limits<real32_T>::infinity() };



                real32_T rtMinusInfF { -std::numeric_limits<real32_T>::infinity() };




    }


            extern "C" {
    
         boolean_T rtIsInf(real_T value) {
        return std::isinf(value);
    }
    
    
         boolean_T rtIsInfF(real32_T value) {
        return std::isinf(value);
    }
    
    
         boolean_T rtIsNaN(real_T value) {
        return std::isnan(value);
    }
    
    
         boolean_T rtIsNaNF(real32_T value) {
        return std::isnan(value);
    }
        }



    

    

    

    
