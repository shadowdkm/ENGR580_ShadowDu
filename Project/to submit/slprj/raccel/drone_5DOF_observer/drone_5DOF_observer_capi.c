#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "drone_5DOF_observer_capi_host.h"
#define sizeof(s) ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el) ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s) (s)
#ifndef SS_UINT64
#define SS_UINT64 17
#endif
#ifndef SS_INT64
#define SS_INT64 18
#endif
#else
#include "builtin_typeid_types.h"
#include "drone_5DOF_observer.h"
#include "drone_5DOF_observer_capi.h"
#include "drone_5DOF_observer_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 1 , TARGET_STRING (
"drone_5DOF_observer/MATLAB Function" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 ,
0 , 0 } , { 1 , 1 , TARGET_STRING ( "drone_5DOF_observer/MATLAB Function" ) ,
TARGET_STRING ( "" ) , 1 , 0 , 0 , 0 , 0 } , { 2 , 1 , TARGET_STRING (
"drone_5DOF_observer/MATLAB Function" ) , TARGET_STRING ( "" ) , 2 , 0 , 0 ,
0 , 0 } , { 3 , 0 , TARGET_STRING (
"drone_5DOF_observer/MATLAB Function/is_active_c2_drone_5DOF_observer" ) ,
TARGET_STRING ( "is_active_c2_drone_5DOF_observer" ) , 0 , 1 , 0 , 0 , 0 } ,
{ 4 , 2 , TARGET_STRING ( "drone_5DOF_observer/MATLAB Function1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 5 , 0 , TARGET_STRING (
"drone_5DOF_observer/MATLAB Function1/is_active_c1_drone_5DOF_observer" ) ,
TARGET_STRING ( "is_active_c1_drone_5DOF_observer" ) , 0 , 1 , 0 , 0 , 0 } ,
{ 6 , 0 , TARGET_STRING ( "drone_5DOF_observer/Clock" ) , TARGET_STRING ( ""
) , 0 , 0 , 0 , 0 , 0 } , { 7 , 0 , TARGET_STRING (
"drone_5DOF_observer/Gain11" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 , 1 } ,
{ 8 , 0 , TARGET_STRING ( "drone_5DOF_observer/Gain5" ) , TARGET_STRING ( ""
) , 0 , 0 , 3 , 0 , 0 } , { 9 , 0 , TARGET_STRING (
"drone_5DOF_observer/Gain9" ) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } ,
{ 10 , 0 , TARGET_STRING ( "drone_5DOF_observer/Integrator2" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 11 , 0 , TARGET_STRING (
"drone_5DOF_observer/Sum5" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , {
12 , 0 , TARGET_STRING ( "drone_5DOF_observer/Sum7" ) , TARGET_STRING ( "" )
, 0 , 0 , 3 , 0 , 0 } , { 13 , 0 , TARGET_STRING ( "drone_5DOF_observer/Sum8"
) , TARGET_STRING ( "" ) , 0 , 0 , 3 , 0 , 0 } , { 14 , 0 , TARGET_STRING (
"drone_5DOF_observer/Sum9" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , {
0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const
rtwCAPI_BlockParameters rtBlockParameters [ ] = { { 15 , TARGET_STRING (
"drone_5DOF_observer/Constant1" ) , TARGET_STRING ( "Value" ) , 0 , 4 , 0 } ,
{ 16 , TARGET_STRING ( "drone_5DOF_observer/Constant2" ) , TARGET_STRING (
"Value" ) , 0 , 5 , 0 } , { 17 , TARGET_STRING (
"drone_5DOF_observer/Constant3" ) , TARGET_STRING ( "Value" ) , 0 , 6 , 0 } ,
{ 18 , TARGET_STRING ( "drone_5DOF_observer/Constant4" ) , TARGET_STRING (
"Value" ) , 0 , 7 , 0 } , { 19 , TARGET_STRING (
"drone_5DOF_observer/Integrator1" ) , TARGET_STRING ( "InitialCondition" ) ,
0 , 0 , 0 } , { 20 , TARGET_STRING ( "drone_5DOF_observer/Integrator2" ) ,
TARGET_STRING ( "InitialCondition" ) , 0 , 0 , 0 } , { 21 , TARGET_STRING (
"drone_5DOF_observer/Random Source" ) , TARGET_STRING ( "MeanVal" ) , 0 , 0 ,
0 } , { 22 , TARGET_STRING ( "drone_5DOF_observer/Random Source" ) ,
TARGET_STRING ( "VarianceRTP" ) , 0 , 0 , 0 } , { 23 , TARGET_STRING (
"drone_5DOF_observer/Step" ) , TARGET_STRING ( "Time" ) , 0 , 0 , 0 } , { 24
, TARGET_STRING ( "drone_5DOF_observer/Step" ) , TARGET_STRING ( "Before" ) ,
0 , 0 , 0 } , { 25 , TARGET_STRING ( "drone_5DOF_observer/Step" ) ,
TARGET_STRING ( "After" ) , 0 , 0 , 0 } , { 26 , TARGET_STRING (
"drone_5DOF_observer/Step1" ) , TARGET_STRING ( "Time" ) , 0 , 0 , 0 } , { 27
, TARGET_STRING ( "drone_5DOF_observer/Step1" ) , TARGET_STRING ( "Before" )
, 0 , 0 , 0 } , { 28 , TARGET_STRING ( "drone_5DOF_observer/Step1" ) ,
TARGET_STRING ( "After" ) , 0 , 0 , 0 } , { 0 , ( NULL ) , ( NULL ) , 0 , 0 ,
0 } } ; static int_T rt_LoggedStateIdxList [ ] = { - 1 } ; static const
rtwCAPI_Signals rtRootInputs [ ] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 ,
0 , 0 , 0 } } ; static const rtwCAPI_Signals rtRootOutputs [ ] = { { 0 , 0 ,
( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const
rtwCAPI_ModelParameters rtModelParameters [ ] = { { 29 , TARGET_STRING ( "A"
) , 0 , 8 , 0 } , { 30 , TARGET_STRING ( "B" ) , 0 , 9 , 0 } , { 31 ,
TARGET_STRING ( "Cob" ) , 0 , 10 , 0 } , { 32 , TARGET_STRING (
"DisturbanceVariance" ) , 0 , 2 , 0 } , { 33 , TARGET_STRING ( "K" ) , 0 , 11
, 0 } , { 34 , TARGET_STRING ( "Ki" ) , 0 , 12 , 0 } , { 35 , TARGET_STRING (
"Kp" ) , 0 , 12 , 0 } , { 36 , TARGET_STRING ( "L" ) , 0 , 13 , 0 } , { 37 ,
TARGET_STRING ( "ObservedVariance" ) , 0 , 14 , 0 } , { 38 , TARGET_STRING (
"TrackingEnabled" ) , 0 , 0 , 0 } , { 39 , TARGET_STRING ( "x0" ) , 0 , 2 , 0
} , { 0 , ( NULL ) , 0 , 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . g01h5zrukl , & rtB . a21stht3j2 ,
& rtB . f2t000eoch , & rtDW . lvor0bqhcv , & rtB . lvo3gfwy02 [ 0 ] , & rtDW
. e2f2zvadqk , & rtB . m3dx5xndf2 , & rtB . ccu4dpkpbc [ 0 ] , & rtB .
dw5qib4r4x [ 0 ] , & rtB . aqub0r2ekx [ 0 ] , & rtB . afd4wlreda [ 0 ] , &
rtB . c30cf505b4 [ 0 ] , & rtB . mqnbs0uqzk [ 0 ] , & rtB . h43ikrnm2w [ 0 ]
, & rtB . ahei004b0s [ 0 ] , & rtP . Constant1_Value [ 0 ] , & rtP .
Constant2_Value [ 0 ] , & rtP . Constant3_Value [ 0 ] , & rtP .
Constant4_Value [ 0 ] , & rtP . Integrator1_IC , & rtP . Integrator2_IC , &
rtP . RandomSource_MeanVal , & rtP . RandomSource_VarianceRTP , & rtP .
Step_Time , & rtP . Step_Y0 , & rtP . Step_YFinal , & rtP . Step1_Time , &
rtP . Step1_Y0 , & rtP . Step1_YFinal , & rtP . A [ 0 ] , & rtP . B [ 0 ] , &
rtP . Cob [ 0 ] , & rtP . DisturbanceVariance [ 0 ] , & rtP . K [ 0 ] , & rtP
. Ki [ 0 ] , & rtP . Kp [ 0 ] , & rtP . L [ 0 ] , & rtP . ObservedVariance [
0 ] , & rtP . TrackingEnabled , & rtP . x0 [ 0 ] , } ; static int32_T *
rtVarDimsAddrMap [ ] = { ( NULL ) } ;
#endif
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap [ ] = { { "double" ,
"real_T" , 0 , 0 , sizeof ( real_T ) , ( uint8_T ) SS_DOUBLE , 0 , 0 , 0 } ,
{ "unsigned char" , "uint8_T" , 0 , 0 , sizeof ( uint8_T ) , ( uint8_T )
SS_UINT8 , 0 , 0 , 0 } } ;
#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif
static TARGET_CONST rtwCAPI_ElementMap rtElementMap [ ] = { { ( NULL ) , 0 ,
0 , 0 , 0 } , } ; static const rtwCAPI_DimensionMap rtDimensionMap [ ] = { {
rtwCAPI_SCALAR , 0 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 2 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 4 , 2 , 0 } , {
rtwCAPI_VECTOR , 6 , 2 , 0 } , { rtwCAPI_VECTOR , 8 , 2 , 0 } , {
rtwCAPI_VECTOR , 10 , 2 , 0 } , { rtwCAPI_VECTOR , 12 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 14 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 16 , 2
, 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 18 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 20 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 22 , 2
, 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 24 , 2 , 0 } , { rtwCAPI_VECTOR , 4 , 2
, 0 } } ; static const uint_T rtDimensionArray [ ] = { 1 , 1 , 10 , 1 , 7 , 1
, 8 , 1 , 3 , 1 , 2 , 1 , 4 , 1 , 10 , 10 , 10 , 3 , 7 , 10 , 3 , 10 , 3 , 7
, 10 , 7 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 , 1.0 } ;
static const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static const
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , (
int8_T ) 0 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 0 ]
, ( const void * ) & rtcapiStoredFloats [ 1 ] , ( int8_T ) 1 , ( uint8_T ) 0
} } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals ,
15 , rtRootInputs , 0 , rtRootOutputs , 0 } , { rtBlockParameters , 14 ,
rtModelParameters , 11 } , { ( NULL ) , 0 } , { rtDataTypeMap ,
rtDimensionMap , rtFixPtMap , rtElementMap , rtSampleTimeMap ,
rtDimensionArray } , "float" , { 3091924031U , 3399062413U , 1667400322U ,
798752003U } , ( NULL ) , 0 , ( boolean_T ) 0 , rt_LoggedStateIdxList } ;
const rtwCAPI_ModelMappingStaticInfo * drone_5DOF_observer_GetCAPIStaticMap (
void ) { return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void drone_5DOF_observer_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion
( ( * rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , & mmiStatic ) ; rtwCAPI_SetLoggingStaticMap ( ( *
rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ; rtwCAPI_SetDataAddressMap ( ( *
rt_dataMapInfoPtr ) . mmi , rtDataAddrMap ) ; rtwCAPI_SetVarDimsAddressMap (
( * rt_dataMapInfoPtr ) . mmi , rtVarDimsAddrMap ) ;
rtwCAPI_SetInstanceLoggingInfo ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArray ( ( * rt_dataMapInfoPtr ) . mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( ( * rt_dataMapInfoPtr ) . mmi , 0 ) ; }
#else
#ifdef __cplusplus
extern "C" {
#endif
void drone_5DOF_observer_host_InitializeDataMapInfo (
drone_5DOF_observer_host_DataMapInfo_T * dataMap , const char * path ) {
rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ; rtwCAPI_SetStaticMap ( dataMap ->
mmi , & mmiStatic ) ; rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetPath ( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap ->
mmi , ( NULL ) ) ; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
