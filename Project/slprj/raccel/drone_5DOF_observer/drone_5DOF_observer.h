#ifndef RTW_HEADER_drone_5DOF_observer_h_
#define RTW_HEADER_drone_5DOF_observer_h_
#ifndef drone_5DOF_observer_COMMON_INCLUDES_
#define drone_5DOF_observer_COMMON_INCLUDES_
#include <stdlib.h>
#include "sl_AsyncioQueue/AsyncioQueueCAPI.h"
#include "rtwtypes.h"
#include "sigstream_rtw.h"
#include "simtarget/slSimTgtSigstreamRTW.h"
#include "simtarget/slSimTgtSlioCoreRTW.h"
#include "simtarget/slSimTgtSlioClientsRTW.h"
#include "simtarget/slSimTgtSlioSdiRTW.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "raccel.h"
#include "slsv_diagnostic_codegen_c_api.h"
#include "rt_logging_simtarget.h"
#include "dt_info.h"
#include "ext_work.h"
#include "stdlib.h"
#endif
#include "drone_5DOF_observer_types.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#include "rtGetInf.h"
#define MODEL_NAME drone_5DOF_observer
#define NSAMPLE_TIMES (4) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (13) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (27)   
#elif NCSTATES != 27
#error Invalid specification of NCSTATES defined in compiler command
#endif
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm) (*rt_dataMapInfoPtr)
#endif
#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val) (rt_dataMapInfoPtr = &val)
#endif
#ifndef IN_RACCEL_MAIN
#endif
typedef struct { real_T ccu4dpkpbc [ 10 ] ; real_T ahei004b0s [ 10 ] ; real_T
dw5qib4r4x [ 7 ] ; real_T h43ikrnm2w [ 7 ] ; real_T afd4wlreda [ 10 ] ;
real_T aqub0r2ekx [ 7 ] ; real_T m3dx5xndf2 ; real_T mqnbs0uqzk [ 7 ] ;
real_T c30cf505b4 [ 10 ] ; real_T lvo3gfwy02 [ 10 ] ; real_T g01h5zrukl ;
real_T a21stht3j2 ; real_T f2t000eoch ; } B ; typedef struct { struct { void
* LoggedData [ 2 ] ; } jmq3tsxawc ; struct { void * LoggedData ; } l5wjhqfbqc
; struct { void * LoggedData ; } pegw3rnsv1 ; struct { void * AQHandles ; }
l0qhxzdoan ; struct { void * AQHandles ; } jmxv5akigp ; struct { void *
AQHandles ; } dgtxfbydlw ; struct { void * AQHandles ; } preosqjjml ; struct
{ void * AQHandles ; } om1g1jtddz ; struct { void * AQHandles ; } kg0cpsbr4n
; struct { void * AQHandles ; } gnkcthcwfm ; struct { void * AQHandles ; }
otdgdsww4h ; int32_T jkne1n5eru ; int32_T ou0nyrelia ; uint32_T myhykfgvxr ;
uint32_T mapbnxxsya [ 2 ] ; int_T crw5jrqanh ; int_T dubitlkqqo ; uint8_T
e2f2zvadqk ; uint8_T lvor0bqhcv ; boolean_T lw2jdaqrz3 ; boolean_T a2bz2wiwxl
; } DW ; typedef struct { real_T eent0niwjp [ 10 ] ; real_T d3azydidno [ 10 ]
; real_T f0bfxnhi2t [ 7 ] ; } X ; typedef struct { real_T eent0niwjp [ 10 ] ;
real_T d3azydidno [ 10 ] ; real_T f0bfxnhi2t [ 7 ] ; } XDot ; typedef struct
{ boolean_T eent0niwjp [ 10 ] ; boolean_T d3azydidno [ 10 ] ; boolean_T
f0bfxnhi2t [ 7 ] ; } XDis ; typedef struct { real_T eent0niwjp [ 10 ] ;
real_T d3azydidno [ 10 ] ; real_T f0bfxnhi2t [ 7 ] ; } CStateAbsTol ; typedef
struct { real_T eent0niwjp [ 10 ] ; real_T d3azydidno [ 10 ] ; real_T
f0bfxnhi2t [ 7 ] ; } CXPtMin ; typedef struct { real_T eent0niwjp [ 10 ] ;
real_T d3azydidno [ 10 ] ; real_T f0bfxnhi2t [ 7 ] ; } CXPtMax ; typedef
struct { real_T bl4x1v1bkh ; real_T daddiwwtso ; } ZCV ; typedef struct {
rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct P_ { real_T A [ 100 ] ;
real_T B [ 30 ] ; real_T Cob [ 70 ] ; real_T DisturbanceVariance [ 10 ] ;
real_T K [ 30 ] ; real_T Ki [ 21 ] ; real_T Kp [ 21 ] ; real_T L [ 70 ] ;
real_T ObservedVariance [ 7 ] ; real_T TrackingEnabled ; real_T x0 [ 10 ] ;
real_T RandomSource_MeanVal ; real_T Step_Time ; real_T Step_Y0 ; real_T
Step_YFinal ; real_T Step1_Time ; real_T Step1_Y0 ; real_T Step1_YFinal ;
real_T RandomSource_VarianceRTP ; real_T Integrator2_IC ; real_T
Integrator1_IC ; real_T Constant1_Value [ 8 ] ; real_T Constant2_Value [ 3 ]
; real_T Constant3_Value [ 2 ] ; real_T Constant4_Value [ 4 ] ; } ; extern
const char_T * RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ;
extern DW rtDW ; extern P rtP ; extern mxArray *
mr_drone_5DOF_observer_GetDWork ( ) ; extern void
mr_drone_5DOF_observer_SetDWork ( const mxArray * ssDW ) ; extern mxArray *
mr_drone_5DOF_observer_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * drone_5DOF_observer_GetCAPIStaticMap ( void
) ; extern SimStruct * const rtS ; extern DataMapInfo * rt_dataMapInfoPtr ;
extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs (
int_T tid ) ; void MdlOutputsParameterSampleTime ( int_T tid ) ; void
MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void
MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ;
SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#endif
