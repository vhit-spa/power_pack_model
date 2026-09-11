/**  @file
  *  MSERef Emachines Simulink Library 
  *  protection class C-SC1
  *
  *  Copyright Robert Bosch GmbH 2021
  *
  *  All rights reserved, also regarding any disposal, exploitation,
  *  reproduction, editing, distribution, as well as in the event of
  *  applications for industrial property rights.
  *
  */

#define S_FUNCTION_NAME EMPSMDQ301MSEREF_sfn
#define S_FUNCTION_LEVEL 2
#define MAX_STRING_PARAM_LENGTH 1024


#include "simstruc.h"

#define _USE_MATH_DEFINES
#include <math.h>

/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%PREHEADER) */
// S-function parameter related functions
#if defined(MATLAB_MEX_FILE) || defined(RT) ||defined(NRT)

real_T empsmdq301mseref_GetRealParam(SimStruct *S, int_T idx)
{
    return mxGetScalar( ssGetSFcnParam(S,idx));
}

int_T empsmdq301mseref_GetIntParam(SimStruct *S, int_T idx)
{
    return (int_T)mxGetScalar( ssGetSFcnParam(S,idx) );
}

int_T empsmdq301mseref_ParamIsDouble(SimStruct *S, int_T paridx)
{
    return mxIsDouble( ssGetSFcnParam(S, paridx) );
}

size_t empsmdq301mseref_ParamGetNumElements(SimStruct *S, int_T paridx)
{
    return mxGetNumberOfElements( ssGetSFcnParam(S, paridx) );
}

#endif /* MATLAB_MEX_FILE */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%PREHEADER) */

/*
    Enum type for use with inline functions
        empsmdq301mseref_GetRealParam(),
        empsmdq301mseref_GetIntParam(),
        empsmdq301mseref_ParamIsDouble(),
        empsmdq301mseref_ParamGetNumElements()
*/
enum EMPSMDQ301MSEREF_enum_params
{
    en_xIdinit,    /* Initial Stator current in d direction [A] */
    en_xIqinit,    /* Initial Stator current in q direction [A] */
    en_N,        /* Number of pole pairs */
    en_R20,        /* Winding resistance at 20 °C [Ohm] */
    en_Ld,        /* d axis inductance [H] */
    en_Lq,        /* q axis inductance [H] */
    en_Km20,    /* Fundamental of flux at 20 °C [V s] */
    en_Acog,    /* Amplitude of cogging torque [N m] */
    en_Ncog,    /* Number of cogging positions [-] */
    en_alpha,    /* Thermal coefficient of resistance [1/K] */
    en_alphamag    /* Thermal coefficient of remanence [1/K] */
};

#define DECLARE_XIDINIT     real_T xIdinit = empsmdq301mseref_GetRealParam(S, en_xIdinit);    // Initial Stator current in d direction [A]
#define DECLARE_XIQINIT     real_T xIqinit = empsmdq301mseref_GetRealParam(S, en_xIqinit);    // Initial Stator current in q direction [A]
#define DECLARE_N             int_T N = empsmdq301mseref_GetIntParam(S, en_N);                     // Number of pole pairs
#define DECLARE_R20         real_T R20 = empsmdq301mseref_GetRealParam(S, en_R20);            // Winding resistance at 20 °C [Ohm]
#define DECLARE_LD             real_T Ld = empsmdq301mseref_GetRealParam(S, en_Ld);                // d axis inductance [H]
#define DECLARE_LQ             real_T Lq = empsmdq301mseref_GetRealParam(S, en_Lq);                // q axis inductance [H]
#define DECLARE_KM20         real_T Km20 = empsmdq301mseref_GetRealParam(S, en_Km20);            // Fundamental of flux at 20 °C [V s]
#define DECLARE_ACOG         real_T Acog = empsmdq301mseref_GetRealParam(S, en_Acog);            // Amplitude of cogging torque [N m]
#define DECLARE_NCOG         real_T Ncog = empsmdq301mseref_GetRealParam(S, en_Ncog);            // Number of cogging positions [-]
#define DECLARE_ALPHA         real_T alpha = empsmdq301mseref_GetRealParam(S, en_alpha);        // Thermal coefficient of resistance [1/K]
#define DECLARE_ALPHAMAG    real_T alphamag = empsmdq301mseref_GetRealParam(S, en_alphamag);    // Thermal coefficient of remanence [1/K]


/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlCheckParameters =============================================
 * Abstract:
 *    Validate our parameters to verify they are okay.
 *
 *    This method is called from mdlInitializeSizes() at the start of
 *    the simulation or by Simulink if any S-Function parameters are changed
 *    while the simulation runs.
 */
#define MDL_CHECK_PARAMETERS
#if defined (MDL_CHECK_PARAMETERS)
/* PARAMETERS */
static void mdlCheckParameters(SimStruct *S)
{
    /* declaring parameters as local variables */
    real_T xIdinit = empsmdq301mseref_GetRealParam(S, en_xIdinit);    // Initial Stator current in d direction [A]
    real_T xIqinit = empsmdq301mseref_GetRealParam(S, en_xIqinit);    // Initial Stator current in q direction [A]
    int_T N = empsmdq301mseref_GetIntParam(S, en_N);                     // Number of pole pairs
    real_T R20 = empsmdq301mseref_GetRealParam(S, en_R20);            // Winding resistance at 20 °C [Ohm]
    real_T Ld = empsmdq301mseref_GetRealParam(S, en_Ld);                // d axis inductance [H]
    real_T Lq = empsmdq301mseref_GetRealParam(S, en_Lq);                // q axis inductance [H]
    real_T Km20 = empsmdq301mseref_GetRealParam(S, en_Km20);            // Fundamental of flux at 20 °C [V s]
    real_T Acog = empsmdq301mseref_GetRealParam(S, en_Acog);            // Amplitude of cogging torque [N m]
    real_T Ncog = empsmdq301mseref_GetRealParam(S, en_Ncog);            // Number of cogging positions [-]
    real_T alpha = empsmdq301mseref_GetRealParam(S, en_alpha);        // Thermal coefficient of resistance [1/K]
    real_T alphamag = empsmdq301mseref_GetRealParam(S, en_alphamag);    // Thermal coefficient of remanence [1/K]

    /* checking parameter xIdinit [-1e9; 1e9] -- Initial Stator current in d direction */
    if (!((xIdinit >= -1e9) && (xIdinit <= 1e9))) {
        ssPrintf("ERROR: Parameter 'xIdinit'=%f outside boundaries [-1e9;1e9]", xIdinit);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'xIdinit', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_xIdinit) || empsmdq301mseref_ParamGetNumElements(S, en_xIdinit) != 1) {
        ssPrintf("ERROR: Parameter 'xIdinit' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'xIdinit', see command window for more information");
    }

    /* checking parameter xIqinit [-1e9; 1e9] -- Initial Stator current in q direction */
    if (!((xIqinit >= -1e9) && (xIqinit <= 1e9))) {
        ssPrintf("ERROR: Parameter 'xIqinit'=%f outside boundaries [-1e9;1e9]", xIqinit);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'xIqinit', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_xIqinit) || empsmdq301mseref_ParamGetNumElements(S, en_xIqinit) != 1) {
        ssPrintf("ERROR: Parameter 'xIqinit' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'xIqinit', see command window for more information");
    }

    /* checking parameter N [1; 1000] -- Number of pole pairs    N */
    if (!((N >= 1) && (N <= 1000))) {
        ssPrintf("ERROR: Parameter 'N'=%d outside boundaries [1;1000]", N);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'N', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_N) || empsmdq301mseref_ParamGetNumElements(S, en_N) != 1|| (empsmdq301mseref_GetRealParam(S, en_N) != empsmdq301mseref_GetIntParam(S, en_N))) {
        ssPrintf("ERROR: Parameter 'N' must be a scalar integer");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'N', see command window for more information");
    }

    /* checking parameter R20 [0; 1e9] -- Winding resistance at 20 °C    R */
    if (!((R20 >= 0) && (R20 <= 1e9))) {
        ssPrintf("ERROR: Parameter 'R20'=%f outside boundaries [0;1e9]", R20);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'R20', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_R20) || empsmdq301mseref_ParamGetNumElements(S, en_R20) != 1) {
        ssPrintf("ERROR: Parameter 'R20' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'R20', see command window for more information");
    }

    /* checking parameter Ld ]0.0; 1.0] -- d axis inductance     Ld */
    if (!((Ld > 0.0) && (Ld <= 1.0))) {
        ssPrintf("ERROR: Parameter 'Ld'=%f outside boundaries ]0.0;1.0]", Ld);
		ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'Ld', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_Ld) || empsmdq301mseref_ParamGetNumElements(S, en_Ld) != 1) {
        ssPrintf("ERROR: Parameter 'Ld' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'Ld', see command window for more information");
    }

    /* checking parameter Lq ]0.0; 1.0] -- q axis inductance    Lq */
    if (!((Lq > 0.0) && (Lq <= 1.0))) {
        ssPrintf("ERROR: Parameter 'Lq'=%f outside boundaries ]0.0;1.0]", Lq);
		ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'Lq', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_Lq) || empsmdq301mseref_ParamGetNumElements(S, en_Lq) != 1) {
        ssPrintf("ERROR: Parameter 'Lq' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'Lq', see command window for more information");
    }

    /* checking parameter Km20 [0; 1e9] -- Fundamental of flux at 20 °C    Km */
    if (!((Km20 >= 0) && (Km20 <= 1e9))) {
        ssPrintf("ERROR: Parameter 'Km20'=%f outside boundaries [0;1e9]", Km20);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'Km20', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_Km20) || empsmdq301mseref_ParamGetNumElements(S, en_Km20) != 1) {
        ssPrintf("ERROR: Parameter 'Km20' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'Km20', see command window for more information");
    }

    /* checking parameter Acog [0; 1e9] -- Amplitude of cogging torque     Acog */
    if (!((Acog >= 0) && (Acog <= 1e9))) {
        ssPrintf("ERROR: Parameter 'Acog'=%f outside boundaries [0;1e9]", Acog);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'Acog', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_Acog) || empsmdq301mseref_ParamGetNumElements(S, en_Acog) != 1) {
        ssPrintf("ERROR: Parameter 'Acog' must be a scalar double");
         ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'Acog', see command window for more information");
    }

    /* checking parameter Ncog [0; 1e3] -- Number of cogging positions    Ncog */
    if (!((Ncog >= 0) && (Ncog <= 1e3))) {
        ssPrintf("ERROR: Parameter 'Ncog'=%f outside boundaries [0;1e3]", Ncog);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'Ncog', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_Ncog) || empsmdq301mseref_ParamGetNumElements(S, en_Ncog) != 1) {
        ssPrintf("ERROR: Parameter 'Ncog' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'Ncog', see command window for more information");
    }

    /* checking parameter alpha [-1; 1] -- Thermal coefficient of resistance */
    if (!((alpha >= -1) && (alpha <= 1))) {
        ssPrintf("ERROR: Parameter 'alpha'=%f outside boundaries [-1;1]", alpha);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'alpha', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_alpha) || empsmdq301mseref_ParamGetNumElements(S, en_alpha) != 1) {
        ssPrintf("ERROR: Parameter 'alpha' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'alpha', see command window for more information");
    }

    /* checking parameter alphamag [-1; 1] -- Thermal coefficient of remanence */
    if (!((alphamag >= -1) && (alphamag <= 1))) {
        ssPrintf("ERROR: Parameter 'alphamag'=%f outside boundaries [-1;1]", alphamag);
        ssSetErrorStatus(S,"ERROR: Wrong parameter value for 'alphamag', see command window for more information");
    }
    if (!empsmdq301mseref_ParamIsDouble(S, en_alphamag) || empsmdq301mseref_ParamGetNumElements(S, en_alphamag) != 1) {
        ssPrintf("ERROR: Parameter 'alphamag' must be a scalar double");
        ssSetErrorStatus(S,"ERROR: Wrong parameter definition for 'alphamag', see command window for more information");
    }

/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%CHECKPARAMETERS) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%CHECKPARAMETERS) */
}
#endif /* MDL_CHECK_PARAMETERS */

/* Function: mdlProcessParameters =============================================
 * Abstract:
 *    This function is for pre-processing parameters, e.g. when you have
 *    to convert units of mask parameters for use in your S-Function or
 *    to do some computation with the parameters once they change.
 *
 *    However we also use this function for validating parameters
 *    when the validation depends on the value of other parameters,
 *    e.g. when one parameter must be larger than another parameter.
 *
 *    This method is called in mdlStart() at the beginning of the simulation
 *    and every time when S-Function parameters have changed.
 *
 */
#define MDL_PROCESS_PARAMETERS
#if defined(MDL_PROCESS_PARAMETERS)
static void mdlProcessParameters(SimStruct *S)
{
/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%PROCESSPARAMETERS) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%PROCESSPARAMETERS) */
}
#endif /* MDL_PROCESS_PARAMETERS */

/* Function: mdlStart =======================================================
 * Abstract:
 *    Do startup tasks.
 */
#define MDL_START
#if defined (MDL_START)
static void mdlStart(SimStruct *S)
{
/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%START) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%START) */
}
#endif /* MDL_START */

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* set number of parameters */
    /* parameters: xIdinit, xIqinit, N, R20, Ld, Lq, Km20, Acog, Ncog, alpha, alphamag */
    ssSetNumSFcnParams(S, 11);
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) return;
    } else {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    /* Continuous states: xId, xIq */
    ssSetNumContStates(S, 2);

    /* Time Discrete States:  */
    ssSetNumDiscStates(S, 0);

    /* Event Discrete States:  */
    ssSetNumModes(S, 0);

    /* Inputs ------------------ */
    if (!ssSetNumInputPorts(S, 6)) return;

    /* Input: Vd */
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 0, 0);

    /* Input: Vq */
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 1, 0);

    /* Input: phi */
    ssSetInputPortWidth(S, 2, 1);
    ssSetInputPortDataType(S, 2, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 2, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 2, 1);

    /* Input: nRPM */
    ssSetInputPortWidth(S, 3, 1);
    ssSetInputPortDataType(S, 3, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 3, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 3, 0);

    /* Input: theta_s */
    ssSetInputPortWidth(S, 4, 1);
    ssSetInputPortDataType(S, 4, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 4, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 4, 1);

    /* Input: theta_r */
    ssSetInputPortWidth(S, 5, 1);
    ssSetInputPortDataType(S, 5, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 5, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 5, 1);   
    
    /* Outputs ------------------- */
    if (!ssSetNumOutputPorts(S, 5)) return;

    /* Output: Id */
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);

    /* Output: Iq */
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType(S, 1, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 1, COMPLEX_NO);

    /* Output: T */
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortDataType(S, 2, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 2, COMPLEX_NO);

    /* Output: Ploss */
    ssSetOutputPortWidth(S, 3, 1);
    ssSetOutputPortDataType(S, 3, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 3, COMPLEX_NO);

    /* Output: Internal variables */
    ssSetOutputPortWidth(S, 4, 3);
    ssSetOutputPortDataType(S, 4, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 4, COMPLEX_NO);

    ssSetNumRWork(S, 0); /*  */
    ssSetNumIWork(S, 0); /*  */
    ssSetNumSampleTimes(S, 1); /* multiple sample times are not supported by the s-fn generator */

    /* If you want to use pointer work vectors, use the following statement */
    /* ssSetNumPWork(S, [number of work vectors]); */

    /* Take care when specifying exception free code - see sfuntmpl_doc.c */
    /* ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE | SS_OPTION_WORKS_WITH_CODE_REUSE)); */

/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%INITSIZES) */
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%INITSIZES) */
}

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
/* Function: mdlSetWorkWidths ===============================================
* Abstract:
*      Set up run-time parameters.
*/
static void mdlSetWorkWidths(SimStruct *S)
{
    ssSetNumRunTimeParams(S, 0);
    if (ssGetErrorStatus(S) != NULL) return;

/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%SETWORKWIDTHS) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%SETWORKWIDTHS) */
}
#endif /* MDL_SET_WORK_WIDTHS */


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specify the sample time.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);

/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%INITSAMPLETIMES) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%INITSAMPLETIMES) */
}


#define MDL_INITIALIZE_CONDITIONS
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    Initialize the states
 */
static void mdlInitializeConditions(SimStruct *S)
{
    /* declaring parameters as local variables */
    real_T xIdinit = empsmdq301mseref_GetRealParam(S, en_xIdinit);    // Initial Stator current in d direction [A]
    real_T xIqinit = empsmdq301mseref_GetRealParam(S, en_xIqinit);    // Initial Stator current in q direction [A]

    /* declaring state variables as local variables */
    real_T *adContStates = ssGetContStates(S);
    real_T& xId = adContStates[0];    // continuous state: Stator current in d direction [A]
    real_T& xIq = adContStates[1];    // continuous state: Stator current in q direction [A]

    xId = xIdinit;    /* initialize Stator current in d direction */
    xIq = xIqinit;    /* initialize Stator current in q direction */

/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%INITIALIZECONDITIONS) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%INITIALIZECONDITIONS) */
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    Calculate model outputs here.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* declaring parameters as local variables */
    int_T N = empsmdq301mseref_GetIntParam(S, en_N);                     // Number of pole pairs
    real_T R20 = empsmdq301mseref_GetRealParam(S, en_R20);            // Winding resistance at 20 °C [Ohm]
    real_T Ld = empsmdq301mseref_GetRealParam(S, en_Ld);    // d axis inductance     Ld [H]
    real_T Lq = empsmdq301mseref_GetRealParam(S, en_Lq);    // q axis inductance    Lq [H]
    real_T Km20 = empsmdq301mseref_GetRealParam(S, en_Km20);            // Fundamental of flux at 20 °C [V s]
    real_T Acog = empsmdq301mseref_GetRealParam(S, en_Acog);            // Amplitude of cogging torque [N m]
    real_T Ncog = empsmdq301mseref_GetRealParam(S, en_Ncog);            // Number of cogging positions [-]
    real_T alpha = empsmdq301mseref_GetRealParam(S, en_alpha);        // Thermal coefficient of resistance [1/K]
    real_T alphamag = empsmdq301mseref_GetRealParam(S, en_alphamag);    // Thermal coefficient of remanence [1/K]

    /* declaring state variables as local variables */
    real_T *adContStates = ssGetContStates(S);
    real_T& xId = adContStates[0];    // continuous state: Stator current in d direction [A]
    real_T& xIq = adContStates[1];    // continuous state: Stator current in q direction [A]

    /* declaring inputs as local variables */
    real_T phi = *(ssGetInputPortRealSignalPtrs(S, 2))[0];        // Rotor position [degree]
    real_T theta_s = *(ssGetInputPortRealSignalPtrs(S, 4))[0];    // Stator Temperature [degC]
    real_T theta_r = *(ssGetInputPortRealSignalPtrs(S, 5))[0];    // Rotor Temperature [degC]

    /* declaring output variables */
    real_T& Id = ((real_T *)(ssGetOutputPortSignal(S, 0)))[0];        // Stator current in d direction [A]
    real_T& Iq = ((real_T *)(ssGetOutputPortSignal(S, 1)))[0];        // Stator current in q direction [A]
    real_T& T = ((real_T *)(ssGetOutputPortSignal(S, 2)))[0];        // Motor torque [N m]
    real_T& Ploss = ((real_T *)(ssGetOutputPortSignal(S, 3)))[0];    // Power loss [W]

    /* output for internal variables */
    real_T *internalsOutput = (real_T *)ssGetOutputPortSignal(S, 4);

/*
    Id = ???
    Iq = ???
    T = ???
    Ploss = ???
*/
/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%OUTPUTS) */
    real_T Km;      // Magnetic remanence [V s]
    real_T Tcog; // Cogging torque [N m]
    real_T Rm;      // Winding resistance [Ohm]
    
    // Temperature effect on remanence and resistance
    Km = Km20* (1.0 + alphamag*(theta_r - 20.0));
    Rm = R20* (1.0 + alpha*(theta_s - 20.0));

    // Compute cogging torque
    Tcog = Acog * sin(phi*Ncog*M_PI/180.0);  //"PI/180" since Input phi is in degree

    // Output equations
    Id  = xId;
    Iq  = xIq;
    T = N * (Km * xIq + (Ld-Lq) * xId * xIq) * 1.5 + Tcog;
    Ploss = 3.0/2.0*Rm*(xId*xId + xIq*xIq);

/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%OUTPUTS) */
    internalsOutput[0] = Km;
    internalsOutput[1] = Tcog;
    internalsOutput[2] = Rm;
}

#define MDL_DERIVATIVES
#ifdef MDL_DERIVATIVES
/* Function: mdlDerivatives =================================================
 * Abstract:
 *    In this function, you compute the S-function block's derivatives.
 *    The derivatives are placed in the derivative vector, ssGetdX(S).
 */
static void mdlDerivatives(SimStruct *S)
{
    /* declaring parameters as local variables */
    int_T N = empsmdq301mseref_GetIntParam(S, en_N); // Number of pole pairs    N
    real_T R20 = empsmdq301mseref_GetRealParam(S, en_R20);    // Winding resistance at 20 °C    R [Ohm]
    real_T Ld = empsmdq301mseref_GetRealParam(S, en_Ld);    // d axis inductance     Ld [H]
    real_T Lq = empsmdq301mseref_GetRealParam(S, en_Lq);    // q axis inductance    Lq [H]
    real_T Km20 = empsmdq301mseref_GetRealParam(S, en_Km20);    // Fundamental of flux at 20 °C    Km [V s]
    real_T alpha = empsmdq301mseref_GetRealParam(S, en_alpha);    // Thermal coefficient of resistance [1/K]
    real_T alphamag = empsmdq301mseref_GetRealParam(S, en_alphamag);    // Thermal coefficient of remanence [1/K]

    /* declaring state variables as local variables */
    real_T *adContStates = ssGetContStates(S);
    const real_T& xId = adContStates[0];    // continuous state: Stator current in d direction [A]
    const real_T& xIq = adContStates[1];    // continuous state: Stator current in q direction [A]

    /* declaring state derivatives as local variables */
    real_T *adContStateDerivs = ssGetdX(S);
    real_T& dxId = adContStateDerivs[0];
    real_T& dxIq = adContStateDerivs[1];

    /* declaring inputs as local variables */
    real_T Vd = *(ssGetInputPortRealSignalPtrs(S, 0))[0];    // Stator voltage in d direction [V]
    real_T Vq = *(ssGetInputPortRealSignalPtrs(S, 1))[0];    // Stator voltage in q direction [V]
    real_T phi = *(ssGetInputPortRealSignalPtrs(S, 2))[0];    // Rotor position [degree]
    real_T nRPM = *(ssGetInputPortRealSignalPtrs(S, 3))[0];    // Rotor speed [rev/min]
    real_T theta_s = *(ssGetInputPortRealSignalPtrs(S, 4))[0];    // Stator Temperature [degC]
    real_T theta_r = *(ssGetInputPortRealSignalPtrs(S, 5))[0];    // Rotor Temperature [degC]

    /*
        dxId = ???
        dxIq = ???
    */
/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%DERIVATIVES) */
    real_T Km;      // Magnetic remanence [V s]
    real_T Rm;      // Winding resistance [Ohm]
    
    // Unit conversion
    real_T omega;
    omega = nRPM*M_PI/30;  // rev/min -> rad/s

    // Temperature effect on remanence and resistance
    Km = Km20* (1.0 + alphamag*(theta_r - 20.0));
    Rm = R20* (1.0 + alpha*(theta_s - 20.0));

    // Derivations of currents
    dxId = (Vd - Rm * xId + N * Lq * omega * xIq)/Ld;
    dxIq = (Vq - Rm * xIq - N * Ld * omega * xId - Km * N * omega)/Lq;

/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%DERIVATIVES) */
}
#endif

/* Function: mdlUpdate ========================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Time-discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per integration
 *    step.
 */
#undef MDL_UPDATE
/* mdlUpdate() is disabled because EMPSMDQ301MSEREF has no time-discrete states */
#if defined (MDL_UPDATE)
static void mdlUpdate(SimStruct *S, int_T tid)
{
/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%UPDATE) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%UPDATE) */
}
#endif /* MDL_UPDATE */

#undef MDL_ZERO_CROSSINGS
/* mdlZeroCrossings() is disabled because EMPSMDQ301MSEREF has no "hasDiscontinuities" flag. */
#ifdef MDL_ZERO_CROSSINGS
/* Function: mdlZeroCrossings ===============================================
 * Abstract:
 *    If your S-function has registered CONTINUOUS_SAMPLE_TIME and there
 *    are signals entering the S-function or internally generated signals
 *    which have discontinuities, you can use this method to locate the
 *    discontinuities. When called, this method must update the
 *    ssGetNonsampleZCs(S) vector.
 */
static void mdlZeroCrossings(SimStruct *S)
{
/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%ZEROCROSSINGS) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%ZEROCROSSINGS) */
}
#endif

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
/* {{{{{{{{{{{{{{{ YOUR CODE HERE (%%TERMINATE) */
/* }}}}}}}}}}}}}}} END OF YOUR CODE (%%TERMINATE) */
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
    #include "simulink.c"      /* MEX-file interface mechanism */
#else
    #include "cg_sfun.h"       /* Code generation registration function */
#endif
