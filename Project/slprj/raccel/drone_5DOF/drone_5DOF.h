#ifndef RTW_HEADER_drone_5DOF_h_
#define RTW_HEADER_drone_5DOF_h_
#ifndef drone_5DOF_COMMON_INCLUDES_
#define drone_5DOF_COMMON_INCLUDES_
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
#endif
#include "drone_5DOF_types.h"
#include <stddef.h>
#include "rtw_modelmap_simtarget.h"
#include "rt_defines.h"
#include <string.h>
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#define MODEL_NAME drone_5DOF
#define NSAMPLE_TIMES (4) 
#define NINPUTS (0)       
#define NOUTPUTS (0)     
#define NBLOCKIO (8) 
#define NUM_ZC_EVENTS (0) 
#ifndef NCSTATES
#define NCSTATES (20)   
#elif NCSTATES != 20
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
typedef struct { real_T kmwcbilhqq [ 10 ] ; real_T ku2bcekbpk [ 10 ] ; real_T
kgojaz3q5t [ 3 ] ; real_T i3et0lx2gv [ 3 ] ; real_T fx25dj0plx [ 3 ] ; real_T
b3g5ksvk55 [ 10 ] ; real_T gp3f1tfouj ; real_T h1cjs4ulia [ 10 ] ; } B ;
typedef struct { struct { void * LoggedData [ 2 ] ; } pgelvgrvt1 ; struct {
void * AQHandles ; } bildbdyfzk ; struct { void * AQHandles ; } h3mcukzweq ;
struct { void * AQHandles ; } fy4dfotmyf ; struct { void * AQHandles ; }
o4yowjfnvg ; struct { void * AQHandles ; } aw1sjpengb ; int32_T o3aryxf3r0 ;
int_T prcj0bragy [ 3 ] ; uint8_T e3nps1yxro ; boolean_T hwccjcyk2m [ 10 ] ;
boolean_T maae0yta5q ; } DW ; typedef struct { real_T epou2i2aqe [ 10 ] ;
real_T b1mchtjukh [ 10 ] ; } X ; typedef struct { real_T epou2i2aqe [ 10 ] ;
real_T b1mchtjukh [ 10 ] ; } XDot ; typedef struct { boolean_T epou2i2aqe [
10 ] ; boolean_T b1mchtjukh [ 10 ] ; } XDis ; typedef struct { real_T
epou2i2aqe [ 10 ] ; real_T b1mchtjukh [ 10 ] ; } CStateAbsTol ; typedef
struct { real_T epou2i2aqe [ 10 ] ; real_T b1mchtjukh [ 10 ] ; } CXPtMin ;
typedef struct { real_T epou2i2aqe [ 10 ] ; real_T b1mchtjukh [ 10 ] ; }
CXPtMax ; typedef struct { real_T j23xiiu4aj [ 10 ] ; real_T ehx5ayseso ; }
ZCV ; typedef struct { rtwCAPI_ModelMappingInfo mmi ; } DataMapInfo ; struct
P_ { real_T A [ 100 ] ; real_T B [ 30 ] ; real_T K [ 30 ] ; real_T stepVal [
3 ] ; real_T x0 [ 10 ] ; real_T CheckStaticUpperBound_max ; real_T Step_Time
; real_T Step_Y0 ; real_T Constant_Value [ 3 ] ; } ; extern const char_T *
RT_MEMORY_ALLOCATION_ERROR ; extern B rtB ; extern X rtX ; extern DW rtDW ;
extern P rtP ; extern mxArray * mr_drone_5DOF_GetDWork ( ) ; extern void
mr_drone_5DOF_SetDWork ( const mxArray * ssDW ) ; extern mxArray *
mr_drone_5DOF_GetSimStateDisallowedBlocks ( ) ; extern const
rtwCAPI_ModelMappingStaticInfo * drone_5DOF_GetCAPIStaticMap ( void ) ;
extern SimStruct * const rtS ; extern DataMapInfo * rt_dataMapInfoPtr ;
extern rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr ; void MdlOutputs (
int_T tid ) ; void MdlOutputsParameterSampleTime ( int_T tid ) ; void
MdlUpdate ( int_T tid ) ; void MdlTerminate ( void ) ; void
MdlInitializeSizes ( void ) ; void MdlInitializeSampleTimes ( void ) ;
SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) ;
#endif
