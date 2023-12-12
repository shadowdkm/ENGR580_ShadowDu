#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "drone_5DOF_capi_host.h"
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
#include "drone_5DOF.h"
#include "drone_5DOF_capi.h"
#include "drone_5DOF_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif
static const rtwCAPI_Signals rtBlockSignals [ ] = { { 0 , 1 , TARGET_STRING (
"drone_5DOF/MATLAB Function" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } ,
{ 1 , 0 , TARGET_STRING (
"drone_5DOF/MATLAB Function/is_active_c2_drone_5DOF" ) , TARGET_STRING (
"is_active_c2_drone_5DOF" ) , 0 , 1 , 1 , 0 , 0 } , { 2 , 0 , TARGET_STRING (
"drone_5DOF/Clock" ) , TARGET_STRING ( "" ) , 0 , 0 , 1 , 0 , 0 } , { 3 , 0 ,
TARGET_STRING ( "drone_5DOF/Gain1" ) , TARGET_STRING ( "" ) , 0 , 0 , 2 , 0 ,
0 } , { 4 , 0 , TARGET_STRING ( "drone_5DOF/Integrator" ) , TARGET_STRING (
"" ) , 0 , 0 , 3 , 0 , 0 } , { 5 , 0 , TARGET_STRING (
"drone_5DOF/Integrator1" ) , TARGET_STRING ( "" ) , 0 , 0 , 0 , 0 , 0 } , { 6
, 0 , TARGET_STRING ( "drone_5DOF/Step" ) , TARGET_STRING ( "" ) , 0 , 0 , 2
, 0 , 1 } , { 7 , 0 , TARGET_STRING ( "drone_5DOF/Sum" ) , TARGET_STRING ( ""
) , 0 , 0 , 3 , 0 , 0 } , { 8 , 0 , TARGET_STRING ( "drone_5DOF/Sum1" ) ,
TARGET_STRING ( "" ) , 0 , 0 , 4 , 0 , 0 } , { 0 , 0 , ( NULL ) , ( NULL ) ,
0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_BlockParameters
rtBlockParameters [ ] = { { 9 , TARGET_STRING (
"drone_5DOF/Check Static  Upper Bound" ) , TARGET_STRING ( "max" ) , 0 , 1 ,
0 } , { 10 , TARGET_STRING ( "drone_5DOF/Constant" ) , TARGET_STRING (
"Value" ) , 0 , 2 , 0 } , { 11 , TARGET_STRING ( "drone_5DOF/Step" ) ,
TARGET_STRING ( "Time" ) , 0 , 1 , 0 } , { 12 , TARGET_STRING (
"drone_5DOF/Step" ) , TARGET_STRING ( "Before" ) , 0 , 1 , 0 } , { 0 , ( NULL
) , ( NULL ) , 0 , 0 , 0 } } ; static int_T rt_LoggedStateIdxList [ ] = { - 1
} ; static const rtwCAPI_Signals rtRootInputs [ ] = { { 0 , 0 , ( NULL ) , (
NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const rtwCAPI_Signals rtRootOutputs [
] = { { 0 , 0 , ( NULL ) , ( NULL ) , 0 , 0 , 0 , 0 , 0 } } ; static const
rtwCAPI_ModelParameters rtModelParameters [ ] = { { 13 , TARGET_STRING ( "A"
) , 0 , 5 , 0 } , { 14 , TARGET_STRING ( "B" ) , 0 , 6 , 0 } , { 15 ,
TARGET_STRING ( "K" ) , 0 , 7 , 0 } , { 16 , TARGET_STRING ( "stepVal" ) , 0
, 8 , 0 } , { 17 , TARGET_STRING ( "x0" ) , 0 , 3 , 0 } , { 0 , ( NULL ) , 0
, 0 , 0 } } ;
#ifndef HOST_CAPI_BUILD
static void * rtDataAddrMap [ ] = { & rtB . h1cjs4ulia [ 0 ] , & rtDW .
e3nps1yxro , & rtB . gp3f1tfouj , & rtB . kgojaz3q5t [ 0 ] , & rtB .
ku2bcekbpk [ 0 ] , & rtB . kmwcbilhqq [ 0 ] , & rtB . fx25dj0plx [ 0 ] , &
rtB . b3g5ksvk55 [ 0 ] , & rtB . i3et0lx2gv [ 0 ] , & rtP .
CheckStaticUpperBound_max , & rtP . Constant_Value [ 0 ] , & rtP . Step_Time
, & rtP . Step_Y0 , & rtP . A [ 0 ] , & rtP . B [ 0 ] , & rtP . K [ 0 ] , &
rtP . stepVal [ 0 ] , & rtP . x0 [ 0 ] , } ; static int32_T *
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
rtwCAPI_MATRIX_COL_MAJOR , 0 , 2 , 0 } , { rtwCAPI_SCALAR , 2 , 2 , 0 } , {
rtwCAPI_VECTOR , 4 , 2 , 0 } , { rtwCAPI_VECTOR , 0 , 2 , 0 } , {
rtwCAPI_MATRIX_COL_MAJOR , 4 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR , 6 , 2 ,
0 } , { rtwCAPI_MATRIX_COL_MAJOR , 8 , 2 , 0 } , { rtwCAPI_MATRIX_COL_MAJOR ,
10 , 2 , 0 } , { rtwCAPI_VECTOR , 12 , 2 , 0 } } ; static const uint_T
rtDimensionArray [ ] = { 10 , 1 , 1 , 1 , 3 , 1 , 10 , 10 , 10 , 3 , 3 , 10 ,
1 , 3 } ; static const real_T rtcapiStoredFloats [ ] = { 0.0 , 1.0 } ; static
const rtwCAPI_FixPtMap rtFixPtMap [ ] = { { ( NULL ) , ( NULL ) ,
rtwCAPI_FIX_RESERVED , 0 , 0 , ( boolean_T ) 0 } , } ; static const
rtwCAPI_SampleTimeMap rtSampleTimeMap [ ] = { { ( const void * ) &
rtcapiStoredFloats [ 0 ] , ( const void * ) & rtcapiStoredFloats [ 0 ] , (
int8_T ) 0 , ( uint8_T ) 0 } , { ( const void * ) & rtcapiStoredFloats [ 0 ]
, ( const void * ) & rtcapiStoredFloats [ 1 ] , ( int8_T ) 1 , ( uint8_T ) 0
} } ; static rtwCAPI_ModelMappingStaticInfo mmiStatic = { { rtBlockSignals ,
9 , rtRootInputs , 0 , rtRootOutputs , 0 } , { rtBlockParameters , 4 ,
rtModelParameters , 5 } , { ( NULL ) , 0 } , { rtDataTypeMap , rtDimensionMap
, rtFixPtMap , rtElementMap , rtSampleTimeMap , rtDimensionArray } , "float"
, { 3831711950U , 1178921623U , 2545239216U , 3475595613U } , ( NULL ) , 0 ,
( boolean_T ) 0 , rt_LoggedStateIdxList } ; const
rtwCAPI_ModelMappingStaticInfo * drone_5DOF_GetCAPIStaticMap ( void ) {
return & mmiStatic ; }
#ifndef HOST_CAPI_BUILD
void drone_5DOF_InitializeDataMapInfo ( void ) { rtwCAPI_SetVersion ( ( *
rt_dataMapInfoPtr ) . mmi , 1 ) ; rtwCAPI_SetStaticMap ( ( *
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
void drone_5DOF_host_InitializeDataMapInfo ( drone_5DOF_host_DataMapInfo_T *
dataMap , const char * path ) { rtwCAPI_SetVersion ( dataMap -> mmi , 1 ) ;
rtwCAPI_SetStaticMap ( dataMap -> mmi , & mmiStatic ) ;
rtwCAPI_SetDataAddressMap ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetVarDimsAddressMap ( dataMap -> mmi , ( NULL ) ) ; rtwCAPI_SetPath
( dataMap -> mmi , path ) ; rtwCAPI_SetFullPath ( dataMap -> mmi , ( NULL ) )
; rtwCAPI_SetChildMMIArray ( dataMap -> mmi , ( NULL ) ) ;
rtwCAPI_SetChildMMIArrayLen ( dataMap -> mmi , 0 ) ; }
#ifdef __cplusplus
}
#endif
#endif
