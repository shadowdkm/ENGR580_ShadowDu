#include "drone_5DOF.h"
#include "rtwtypes.h"
#include <stdlib.h>
#include "drone_5DOF_private.h"
#include <string.h>
#include <emmintrin.h>
#include "mwmathutil.h"
#include "drone_5DOF_types.h"
#include "rt_logging_mmi.h"
#include "drone_5DOF_capi.h"
#include <math.h>
#include "drone_5DOF_dt.h"
extern void * CreateDiagnosticAsVoidPtr_wrapper ( const char * id , int nargs
, ... ) ; extern ssExecutionInfo gblExecutionInfo ; RTWExtModeInfo *
gblRTWExtModeInfo = NULL ; void raccelForceExtModeShutdown ( boolean_T
extModeStartPktReceived ) { if ( ! extModeStartPktReceived ) { boolean_T
stopRequested = false ; rtExtModeWaitForStartPkt ( gblRTWExtModeInfo , 3 , &
stopRequested ) ; } rtExtModeShutdown ( 3 ) ; }
#include "slsv_diagnostic_codegen_c_api.h"
#include "slsa_sim_engine.h"
#ifdef RSIM_WITH_SOLVER_MULTITASKING
boolean_T gbl_raccel_isMultitasking = 1 ;
#else
boolean_T gbl_raccel_isMultitasking = 0 ;
#endif
boolean_T gbl_raccel_tid01eq = 0 ; int_T gbl_raccel_NumST = 4 ; const char_T
* gbl_raccel_Version = "23.2 (R2023b) 01-Aug-2023" ; void
raccel_setup_MMIStateLog ( SimStruct * S ) {
#ifdef UseMMIDataLogging
rt_FillStateSigInfoFromMMI ( ssGetRTWLogInfo ( S ) , & ssGetErrorStatus ( S )
) ;
#else
UNUSED_PARAMETER ( S ) ;
#endif
} static DataMapInfo rt_dataMapInfo ; DataMapInfo * rt_dataMapInfoPtr = &
rt_dataMapInfo ; rtwCAPI_ModelMappingInfo * rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; int_T enableFcnCallFlag [ ] = { 1 , 1 , 1 , 1 } ;
const char * raccelLoadInputsAndAperiodicHitTimes ( SimStruct * S , const
char * inportFileName , int * matFileFormat ) { return
rt_RAccelReadInportsMatFile ( S , inportFileName , matFileFormat ) ; }
#include "simstruc.h"
#include "fixedpoint.h"
#include "slsa_sim_engine.h"
#include "simtarget/slSimTgtSLExecSimBridge.h"
#define fxt5zp2mzq (-1)
B rtB ; X rtX ; DW rtDW ; static SimStruct model_S ; SimStruct * const rtS =
& model_S ; void RandSrcInitState_U_64 ( const uint32_T seed [ ] , real_T
state [ ] , int32_T nChans ) { real_T d ; int32_T i ; int32_T k ; int32_T n ;
uint32_T j ; for ( i = 0 ; i < nChans ; i ++ ) { if ( seed [ i ] != 0U ) { j
= seed [ i ] ; } else { j = 2147483648U ; } state [ 35 * i + 34 ] = j ; for (
k = 0 ; k < 32 ; k ++ ) { d = 0.0 ; for ( n = 0 ; n < 53 ; n ++ ) { j ^= j <<
13 ; j ^= j >> 17 ; j ^= j << 5 ; d = ( real_T ) ( ( int32_T ) ( j >> 19 ) &
1 ) + ( d + d ) ; } state [ 35 * i + k ] = ldexp ( d , - 53 ) ; } state [ 35
* i + 32 ] = 0.0 ; state [ 35 * i + 33 ] = 0.0 ; } } void RandSrc_U_D (
real_T y [ ] , const real_T minVec [ ] , int32_T minLen , const real_T maxVec
[ ] , int32_T maxLen , real_T state [ ] , int32_T nChans , int32_T nSamps ) {
real_T d ; real_T min ; real_T scale ; int32_T ii [ 2 ] ; int32_T chan ;
int32_T i ; int32_T one ; int32_T samps ; uint32_T j ; int8_T * onePtr ; one
= 1 ; onePtr = ( int8_T * ) & one ; one = ( onePtr [ 0U ] == 0 ) ; for ( chan
= 0 ; chan < nChans ; chan ++ ) { if ( minLen > 1 ) { i = chan ; } else { i =
0 ; } min = minVec [ i ] ; if ( maxLen > 1 ) { i = chan ; } else { i = 0 ; }
scale = maxVec [ i ] - min ; i = ( int32_T ) ( ( uint32_T ) state [ chan * 35
+ 33 ] & 31U ) ; j = ( uint32_T ) state [ chan * 35 + 34 ] ; for ( samps = 0
; samps < nSamps ; samps ++ ) { d = ( state [ ( ( i + 20 ) & 31 ) + chan * 35
] - state [ ( ( i + 5 ) & 31 ) + chan * 35 ] ) - state [ chan * 35 + 32 ] ;
if ( d >= 0.0 ) { state [ chan * 35 + 32 ] = 0.0 ; } else { d ++ ; state [
chan * 35 + 32 ] = 1.1102230246251565E-16 ; } state [ chan * 35 + i ] = d ; i
= ( i + 1 ) & 31 ; memcpy ( & ii [ 0U ] , & d , sizeof ( real_T ) ) ; ii [
one ] = ( int32_T ) ( ( uint32_T ) ii [ one ] ^ j ) ; j ^= j << 13 ; j ^= j
>> 17 ; j ^= j << 5 ; ii [ one ^ 1 ] = ( int32_T ) ( ( uint32_T ) ii [ one ^
1 ] ^ ( j & 1048575U ) ) ; memcpy ( & d , & ii [ 0U ] , sizeof ( real_T ) ) ;
y [ chan * nSamps + samps ] = scale * d + min ; } state [ chan * 35 + 33 ] =
i ; state [ chan * 35 + 34 ] = j ; } } void RandSrcCreateSeeds_64 ( uint32_T
initSeed , uint32_T seedArray [ ] , int32_T numSeeds ) { real_T state [ 35 ]
; real_T max ; real_T min ; real_T tmp ; int32_T i ; min = 0.0 ; max = 1.0 ;
RandSrcInitState_U_64 ( & initSeed , & state [ 0 ] , 1 ) ; for ( i = 0 ; i <
numSeeds ; i ++ ) { RandSrc_U_D ( & tmp , & min , 1 , & max , 1 , state , 1 ,
1 ) ; seedArray [ i ] = ( uint32_T ) ( tmp * 2.147483648E+9 ) ; } } void
RandSrcInitState_GZ ( const uint32_T seed [ ] , uint32_T state [ ] , int32_T
nChans ) { int32_T i ; for ( i = 0 ; i < nChans ; i ++ ) { state [ i << 1 ] =
362436069U ; if ( seed [ i ] == 0U ) { state [ ( i << 1 ) + 1 ] = 521288629U
; } else { state [ ( i << 1 ) + 1 ] = seed [ i ] ; } } } void RandSrc_GZ_D (
real_T y [ ] , const real_T mean [ ] , int32_T meanLen , const real_T xstd [
] , int32_T xstdLen , uint32_T state [ ] , int32_T nChans , int32_T nSamps )
{ real_T r ; real_T s ; real_T std ; real_T x ; real_T y_p ; int32_T chan ;
int32_T i ; int32_T j ; int32_T samp ; uint32_T icng ; uint32_T jsr ; static
const real_T vt [ 65 ] = { 0.340945 , 0.4573146 , 0.5397793 , 0.6062427 ,
0.6631691 , 0.7136975 , 0.7596125 , 0.8020356 , 0.8417227 , 0.8792102 ,
0.9148948 , 0.9490791 , 0.9820005 , 1.0138492 , 1.044781 , 1.0749254 ,
1.1043917 , 1.1332738 , 1.161653 , 1.189601 , 1.2171815 , 1.2444516 ,
1.2714635 , 1.298265 , 1.3249008 , 1.3514125 , 1.3778399 , 1.4042211 ,
1.4305929 , 1.4569915 , 1.4834527 , 1.5100122 , 1.5367061 , 1.5635712 ,
1.5906454 , 1.617968 , 1.6455802 , 1.6735255 , 1.7018503 , 1.7306045 ,
1.7598422 , 1.7896223 , 1.8200099 , 1.851077 , 1.8829044 , 1.9155831 ,
1.9492166 , 1.9839239 , 2.0198431 , 2.0571356 , 2.095993 , 2.136645 ,
2.1793713 , 2.2245175 , 2.2725186 , 2.3239338 , 2.3795008 , 2.4402218 ,
2.5075117 , 2.5834658 , 2.6713916 , 2.7769942 , 2.7769942 , 2.7769942 ,
2.7769942 } ; for ( chan = 0 ; chan < nChans ; chan ++ ) { if ( xstdLen > 1 )
{ i = chan ; } else { i = 0 ; } std = xstd [ i ] ; icng = state [ chan << 1 ]
; jsr = state [ ( chan << 1 ) + 1 ] ; for ( samp = 0 ; samp < nSamps ; samp
++ ) { icng = 69069U * icng + 1234567U ; jsr ^= jsr << 13 ; jsr ^= jsr >> 17
; jsr ^= jsr << 5 ; i = ( int32_T ) ( icng + jsr ) ; j = ( i & 63 ) + 1 ; r =
( real_T ) i * 4.6566128730773926E-10 * vt [ j ] ; if ( ! ( muDoubleScalarAbs
( r ) <= vt [ j - 1 ] ) ) { x = ( muDoubleScalarAbs ( r ) - vt [ j - 1 ] ) /
( vt [ j ] - vt [ j - 1 ] ) ; icng = 69069U * icng + 1234567U ; jsr ^= jsr <<
13 ; jsr ^= jsr >> 17 ; jsr ^= jsr << 5 ; y_p = ( real_T ) ( int32_T ) ( icng
+ jsr ) * 2.328306436538696E-10 + 0.5 ; s = x + y_p ; if ( s > 1.301198 ) {
if ( r < 0.0 ) { r = 0.4878992 * x - 0.4878992 ; } else { r = 0.4878992 -
0.4878992 * x ; } } else if ( ! ( s <= 0.9689279 ) ) { x = 0.4878992 -
0.4878992 * x ; if ( y_p > 12.67706 - muDoubleScalarExp ( - 0.5 * x * x ) *
12.37586 ) { if ( r < 0.0 ) { r = - x ; } else { r = x ; } } else if ( ! (
muDoubleScalarExp ( - 0.5 * vt [ j ] * vt [ j ] ) + y_p * 0.01958303 / vt [ j
] <= muDoubleScalarExp ( - 0.5 * r * r ) ) ) { do { icng = 69069U * icng +
1234567U ; jsr ^= jsr << 13 ; jsr ^= jsr >> 17 ; jsr ^= jsr << 5 ; x =
muDoubleScalarLog ( ( real_T ) ( int32_T ) ( icng + jsr ) *
2.328306436538696E-10 + 0.5 ) / 2.776994 ; icng = 69069U * icng + 1234567U ;
jsr ^= jsr << 13 ; jsr ^= jsr >> 17 ; jsr ^= jsr << 5 ; } while (
muDoubleScalarLog ( ( real_T ) ( int32_T ) ( icng + jsr ) *
2.328306436538696E-10 + 0.5 ) * - 2.0 <= x * x ) ; if ( r < 0.0 ) { r = x -
2.776994 ; } else { r = 2.776994 - x ; } } } } if ( meanLen > 1 ) { i = chan
; } else { i = 0 ; } y [ chan * nSamps + samp ] = std * r + mean [ i ] ; }
state [ chan << 1 ] = icng ; state [ ( chan << 1 ) + 1 ] = jsr ; } } void
MdlInitialize ( void ) { memcpy ( & rtX . epou2i2aqe [ 0 ] , & rtP . x0 [ 0 ]
, 10U * sizeof ( real_T ) ) ; memcpy ( & rtX . b1mchtjukh [ 0 ] , & rtP . x0
[ 0 ] , 10U * sizeof ( real_T ) ) ; rtDW . maae0yta5q = false ; rtDW .
o3aryxf3r0 = fxt5zp2mzq ; } void MdlStart ( void ) { uint32_T initSeed ; {
bool externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( rtS ) ;
rtwISigstreamManagerGetInputIsInDatasetFormat ( pISigstreamManager , &
externalInputIsInDatasetFormat ) ; if ( externalInputIsInDatasetFormat ) { }
} { { { bool isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU
srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars ( "Integrator" ) ;
sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Integrator" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF/To Workspace2" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Integrator" )
; sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 1 ] = { 10 } ; sigDims . nDims = 1 ; sigDims . dimensions = sigDimsArray ;
srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU
) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ; srcInfo .
subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo . signalName =
sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . bildbdyfzk . AQHandles =
sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo . mmi .
InstanceMap . fullPath , "2d769af7-7cef-45a3-be82-d6a8afd144aa" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ;
sdiCompleteAsyncioQueueCreation ( rtDW . bildbdyfzk . AQHandles , hDT , &
srcInfo ) ; if ( rtDW . bildbdyfzk . AQHandles ) {
sdiSetSignalSampleTimeString ( rtDW . bildbdyfzk . AQHandles , "0.01" , 0.01
, ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW . bildbdyfzk . AQHandles
, 0.0 ) ; sdiSetRunStartTime ( rtDW . bildbdyfzk . AQHandles , ssGetTaskTime
( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings ( rtDW . bildbdyfzk .
AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName ( rtDW . bildbdyfzk .
AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . bildbdyfzk . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"simuLinearStates" ) ; sdiRegisterWksVariable ( rtDW . bildbdyfzk . AQHandles
, varName , "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } { { { bool
isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU srcInfo ;
sdiLabelU loggedName = sdiGetLabelFromChars ( "Integrator1" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Integrator1" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF/To Workspace3" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Integrator1"
) ; sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 2 ] = { 10 , 1 } ; sigDims . nDims = 2 ; sigDims . dimensions =
sigDimsArray ; srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = (
sdiFullBlkPathU ) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ;
srcInfo . subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo .
signalName = sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . h3mcukzweq .
AQHandles = sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo .
mmi . InstanceMap . fullPath , "332ef2be-35ba-4a39-97cd-bc58f0c317c4" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ; if ( rtDW
. h3mcukzweq . AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . h3mcukzweq
. AQHandles , "0.01" , 0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate (
rtDW . h3mcukzweq . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW .
h3mcukzweq . AQHandles , ssGetTaskTime ( rtS , 2 ) ) ;
sdiAsyncRepoSetSignalExportSettings ( rtDW . h3mcukzweq . AQHandles , 1 , 0 )
; sdiAsyncRepoSetSignalExportName ( rtDW . h3mcukzweq . AQHandles ,
loggedName , origSigName , propName ) ; sdiAsyncRepoSetBlockPathDomain ( rtDW
. h3mcukzweq . AQHandles ) ; sdiSetSignalIsFrameBased ( rtDW . h3mcukzweq .
AQHandles , true ) ; sdiCompleteAsyncioQueueCreation ( rtDW . h3mcukzweq .
AQHandles , hDT , & srcInfo ) ; } sdiFreeLabel ( sigName ) ; sdiFreeLabel (
loggedName ) ; sdiFreeLabel ( origSigName ) ; sdiFreeLabel ( propName ) ;
sdiFreeLabel ( blockPath ) ; sdiFreeLabel ( blockSID ) ; sdiFreeLabel (
subPath ) ; } } if ( ! isStreamoutAlreadyRegistered ) { { sdiLabelU varName =
sdiGetLabelFromChars ( "simuRealStates" ) ; sdiRegisterWksVariable ( rtDW .
h3mcukzweq . AQHandles , varName , "array2D" ) ; sdiFreeLabel ( varName ) ; }
} } } } { { { bool isStreamoutAlreadyRegistered = false ; {
sdiSignalSourceInfoU srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars (
"Gain1" ) ; sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU
propName = sdiGetLabelFromChars ( "Gain1" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF/To Workspace4" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Gain1" ) ;
sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 1 ] = { 3 } ; sigDims . nDims = 1 ; sigDims . dimensions = sigDimsArray ;
srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU
) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ; srcInfo .
subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo . signalName =
sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . fy4dfotmyf . AQHandles =
sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo . mmi .
InstanceMap . fullPath , "d8036e6f-bdfd-4b16-9176-b1bf7dea093d" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ;
sdiCompleteAsyncioQueueCreation ( rtDW . fy4dfotmyf . AQHandles , hDT , &
srcInfo ) ; if ( rtDW . fy4dfotmyf . AQHandles ) {
sdiSetSignalSampleTimeString ( rtDW . fy4dfotmyf . AQHandles , "0.01" , 0.01
, ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW . fy4dfotmyf . AQHandles
, 0.0 ) ; sdiSetRunStartTime ( rtDW . fy4dfotmyf . AQHandles , ssGetTaskTime
( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings ( rtDW . fy4dfotmyf .
AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName ( rtDW . fy4dfotmyf .
AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . fy4dfotmyf . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"simuLinearU" ) ; sdiRegisterWksVariable ( rtDW . fy4dfotmyf . AQHandles ,
varName , "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } { { { bool
isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU srcInfo ;
sdiLabelU loggedName = sdiGetLabelFromChars ( "Sum1" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Sum1" ) ; sdiLabelU blockPath = sdiGetLabelFromChars
( "drone_5DOF/To Workspace5" ) ; sdiLabelU blockSID = sdiGetLabelFromChars (
"" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ;
sdiLabelU sigName = sdiGetLabelFromChars ( "Sum1" ) ;
sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 2 ] = { 3 , 1 } ; sigDims . nDims = 2 ; sigDims . dimensions = sigDimsArray
; srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = (
sdiFullBlkPathU ) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ;
srcInfo . subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo .
signalName = sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . o4yowjfnvg .
AQHandles = sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo .
mmi . InstanceMap . fullPath , "e85ac045-0197-4021-9e0c-4bf8123d2736" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ; if ( rtDW
. o4yowjfnvg . AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . o4yowjfnvg
. AQHandles , "0.01" , 0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate (
rtDW . o4yowjfnvg . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW .
o4yowjfnvg . AQHandles , ssGetTaskTime ( rtS , 2 ) ) ;
sdiAsyncRepoSetSignalExportSettings ( rtDW . o4yowjfnvg . AQHandles , 1 , 0 )
; sdiAsyncRepoSetSignalExportName ( rtDW . o4yowjfnvg . AQHandles ,
loggedName , origSigName , propName ) ; sdiAsyncRepoSetBlockPathDomain ( rtDW
. o4yowjfnvg . AQHandles ) ; sdiSetSignalIsFrameBased ( rtDW . o4yowjfnvg .
AQHandles , true ) ; sdiCompleteAsyncioQueueCreation ( rtDW . o4yowjfnvg .
AQHandles , hDT , & srcInfo ) ; } sdiFreeLabel ( sigName ) ; sdiFreeLabel (
loggedName ) ; sdiFreeLabel ( origSigName ) ; sdiFreeLabel ( propName ) ;
sdiFreeLabel ( blockPath ) ; sdiFreeLabel ( blockSID ) ; sdiFreeLabel (
subPath ) ; } } if ( ! isStreamoutAlreadyRegistered ) { { sdiLabelU varName =
sdiGetLabelFromChars ( "simuRealU" ) ; sdiRegisterWksVariable ( rtDW .
o4yowjfnvg . AQHandles , varName , "array2D" ) ; sdiFreeLabel ( varName ) ; }
} } } } { { { bool isStreamoutAlreadyRegistered = false ; {
sdiSignalSourceInfoU srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars (
"Clock" ) ; sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU
propName = sdiGetLabelFromChars ( "Clock" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF/To Workspace1" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Clock" ) ;
sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 1 ] = { 1 } ; sigDims . nDims = 1 ; sigDims . dimensions = sigDimsArray ;
srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU
) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ; srcInfo .
subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo . signalName =
sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . aw1sjpengb . AQHandles =
sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo . mmi .
InstanceMap . fullPath , "b98078cf-4ccd-4146-b29a-d41992bf7cab" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ;
sdiCompleteAsyncioQueueCreation ( rtDW . aw1sjpengb . AQHandles , hDT , &
srcInfo ) ; if ( rtDW . aw1sjpengb . AQHandles ) {
sdiSetSignalSampleTimeString ( rtDW . aw1sjpengb . AQHandles , "0.01" , 0.01
, ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW . aw1sjpengb . AQHandles
, 0.0 ) ; sdiSetRunStartTime ( rtDW . aw1sjpengb . AQHandles , ssGetTaskTime
( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings ( rtDW . aw1sjpengb .
AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName ( rtDW . aw1sjpengb .
AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . aw1sjpengb . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"simuT" ) ; sdiRegisterWksVariable ( rtDW . aw1sjpengb . AQHandles , varName
, "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } initSeed = ( uint32_T )
( 100000 * rand ( ) ) ; RandSrcCreateSeeds_64 ( initSeed , & rtDW .
atzgtyu2uw , 1 ) ; RandSrcInitState_GZ ( & rtDW . atzgtyu2uw , & rtDW .
bgwp0luil4 [ 0 ] , 1 ) ; MdlInitialize ( ) ; } void MdlOutputs ( int_T tid )
{ __m128d tmp_g ; __m128d tmp_m ; real_T tmp [ 30 ] ; real_T tmp_p [ 10 ] ;
real_T kgojaz3q5t ; real_T knukvxsfml ; real_T tmp_e ; real_T tmp_i ; int32_T
i ; int32_T i_p ; boolean_T h5agxojluo [ 10 ] ; SimStruct * S ; void * diag ;
memcpy ( & rtB . kmwcbilhqq [ 0 ] , & rtX . epou2i2aqe [ 0 ] , 10U * sizeof (
real_T ) ) ; if ( ssIsSampleHit ( rtS , 1 , 0 ) ) { if (
ssIsModeUpdateTimeStep ( rtS ) ) { for ( i = 0 ; i < 10 ; i ++ ) { rtDW .
hwccjcyk2m [ i ] = ( rtB . kmwcbilhqq [ i ] <= rtP .
CheckStaticUpperBound_max ) ; } } for ( i = 0 ; i < 10 ; i ++ ) { h5agxojluo
[ i ] = rtDW . hwccjcyk2m [ i ] ; } for ( i = 0 ; i < 10 ; i ++ ) { if ( !
h5agxojluo [ i ] ) { S = rtS ; diag = CreateDiagnosticAsVoidPtr (
"Simulink:blocks:AssertionAssert" , 2 , 5 ,
"drone_5DOF/Check Static  Upper Bound/Assertion" , 2 , ssGetT ( rtS ) ) ;
rt_ssSet_slErrMsg ( S , diag ) ; ssSetStopRequested ( rtS , ( int ) ssGetT (
rtS ) ) ; } } } memcpy ( & rtB . ku2bcekbpk [ 0 ] , & rtX . b1mchtjukh [ 0 ]
, 10U * sizeof ( real_T ) ) ; if ( ssIsSampleHit ( rtS , 2 , 0 ) ) { { if (
rtDW . bildbdyfzk . AQHandles && ssGetLogOutput ( rtS ) ) { sdiWriteSignal (
rtDW . bildbdyfzk . AQHandles , ssGetTaskTime ( rtS , 2 ) , ( char * ) & rtB
. ku2bcekbpk [ 0 ] + 0 ) ; } } { if ( rtDW . h3mcukzweq . AQHandles &&
ssGetLogOutput ( rtS ) ) { sdiWriteSignal ( rtDW . h3mcukzweq . AQHandles ,
ssGetTaskTime ( rtS , 2 ) , ( char * ) & rtB . kmwcbilhqq [ 0 ] + 0 ) ; } } }
for ( i = 0 ; i <= 28 ; i += 2 ) { tmp_g = _mm_loadu_pd ( & rtP . K [ i ] ) ;
_mm_storeu_pd ( & tmp [ i ] , _mm_mul_pd ( tmp_g , _mm_set1_pd ( - 1.0 ) ) )
; } for ( i = 0 ; i < 3 ; i ++ ) { kgojaz3q5t = 0.0 ; for ( i_p = 0 ; i_p <
10 ; i_p ++ ) { kgojaz3q5t += tmp [ 3 * i_p + i ] * rtB . ku2bcekbpk [ i_p ]
; } rtB . kgojaz3q5t [ i ] = kgojaz3q5t ; } if ( ssIsSampleHit ( rtS , 2 , 0
) ) { { if ( rtDW . fy4dfotmyf . AQHandles && ssGetLogOutput ( rtS ) ) {
sdiWriteSignal ( rtDW . fy4dfotmyf . AQHandles , ssGetTaskTime ( rtS , 2 ) ,
( char * ) & rtB . kgojaz3q5t [ 0 ] + 0 ) ; } } } RandSrc_GZ_D ( & knukvxsfml
, & rtP . RandomSource1_MeanVal , 1 , & rtP . RandomSource1_VarianceRTP , 1 ,
rtDW . bgwp0luil4 , 1 , 1 ) ; for ( i = 0 ; i <= 8 ; i += 2 ) { tmp_g =
_mm_loadu_pd ( & rtP . stateNoiseGain [ i ] ) ; tmp_m = _mm_loadu_pd ( & rtB
. kmwcbilhqq [ i ] ) ; _mm_storeu_pd ( & tmp_p [ i ] , _mm_add_pd (
_mm_mul_pd ( tmp_g , _mm_set1_pd ( knukvxsfml ) ) , tmp_m ) ) ; } for ( i = 0
; i < 3 ; i ++ ) { kgojaz3q5t = 0.0 ; for ( i_p = 0 ; i_p < 10 ; i_p ++ ) {
kgojaz3q5t += rtP . K [ 3 * i_p + i ] * tmp_p [ i_p ] ; } rtB . i3et0lx2gv [
i ] = rtP . Constant_Value [ i ] - kgojaz3q5t ; } if ( ssIsSampleHit ( rtS ,
2 , 0 ) ) { { if ( rtDW . o4yowjfnvg . AQHandles && ssGetLogOutput ( rtS ) )
{ sdiWriteSignal ( rtDW . o4yowjfnvg . AQHandles , ssGetTaskTime ( rtS , 2 )
, ( char * ) & rtB . i3et0lx2gv [ 0 ] + 0 ) ; } } } if ( ssIsSampleHit ( rtS
, 1 , 0 ) ) { knukvxsfml = ssGetTaskTime ( rtS , 1 ) ; i = ( knukvxsfml >=
rtP . Step_Time ) ; rtDW . prcj0bragy [ 0 ] = i ; if ( i == 1 ) { rtB .
fx25dj0plx [ 0 ] = rtP . stepVal [ 0 ] ; } else { rtB . fx25dj0plx [ 0 ] =
rtP . Step_Y0 ; } i = ( knukvxsfml >= rtP . Step_Time ) ; rtDW . prcj0bragy [
1 ] = i ; if ( i == 1 ) { rtB . fx25dj0plx [ 1 ] = rtP . stepVal [ 1 ] ; }
else { rtB . fx25dj0plx [ 1 ] = rtP . Step_Y0 ; } i = ( knukvxsfml >= rtP .
Step_Time ) ; rtDW . prcj0bragy [ 2 ] = i ; if ( i == 1 ) { rtB . fx25dj0plx
[ 2 ] = rtP . stepVal [ 2 ] ; } else { rtB . fx25dj0plx [ 2 ] = rtP . Step_Y0
; } } rtDW . o3aryxf3r0 = fxt5zp2mzq ; rtB . h1cjs4ulia [ 0 ] = rtB .
kmwcbilhqq [ 3 ] ; rtB . h1cjs4ulia [ 1 ] = rtB . kmwcbilhqq [ 4 ] ; rtB .
h1cjs4ulia [ 2 ] = rtB . kmwcbilhqq [ 5 ] ; rtB . h1cjs4ulia [ 3 ] = - rtB .
kmwcbilhqq [ 9 ] * rtB . kmwcbilhqq [ 5 ] - 9.81 * muDoubleScalarSin ( rtB .
kmwcbilhqq [ 7 ] ) ; rtB . h1cjs4ulia [ 4 ] = 9.81 * muDoubleScalarCos ( rtB
. kmwcbilhqq [ 7 ] ) * muDoubleScalarSin ( rtB . kmwcbilhqq [ 6 ] ) + rtB .
kmwcbilhqq [ 5 ] * rtB . kmwcbilhqq [ 8 ] ; rtB . h1cjs4ulia [ 5 ] = ( ( rtB
. kmwcbilhqq [ 3 ] * rtB . kmwcbilhqq [ 9 ] - rtB . kmwcbilhqq [ 4 ] * rtB .
kmwcbilhqq [ 8 ] ) + 9.81 * muDoubleScalarCos ( rtB . kmwcbilhqq [ 7 ] ) *
muDoubleScalarCos ( rtB . kmwcbilhqq [ 6 ] ) ) - ( rtB . fx25dj0plx [ 0 ] +
rtB . i3et0lx2gv [ 0 ] ) / 0.1 ; rtB . h1cjs4ulia [ 6 ] = rtB . kmwcbilhqq [
8 ] ; rtB . h1cjs4ulia [ 7 ] = rtB . kmwcbilhqq [ 9 ] ; rtB . h1cjs4ulia [ 8
] = ( rtB . fx25dj0plx [ 1 ] + rtB . i3et0lx2gv [ 1 ] ) / 0.0001 ; rtB .
h1cjs4ulia [ 9 ] = ( rtB . fx25dj0plx [ 2 ] + rtB . i3et0lx2gv [ 2 ] ) /
0.0001 ; knukvxsfml = rtB . fx25dj0plx [ 0 ] + rtB . kgojaz3q5t [ 0 ] ; tmp_e
= rtB . fx25dj0plx [ 1 ] + rtB . kgojaz3q5t [ 1 ] ; tmp_i = rtB . fx25dj0plx
[ 2 ] + rtB . kgojaz3q5t [ 2 ] ; for ( i = 0 ; i < 10 ; i ++ ) { kgojaz3q5t =
0.0 ; for ( i_p = 0 ; i_p < 10 ; i_p ++ ) { kgojaz3q5t += rtP . A [ 10 * i_p
+ i ] * rtB . ku2bcekbpk [ i_p ] ; } rtB . b3g5ksvk55 [ i ] = ( ( rtP . B [ i
+ 10 ] * tmp_e + rtP . B [ i ] * knukvxsfml ) + rtP . B [ i + 20 ] * tmp_i )
+ kgojaz3q5t ; } rtB . gp3f1tfouj = ssGetT ( rtS ) ; if ( ssIsSampleHit ( rtS
, 2 , 0 ) ) { { if ( rtDW . aw1sjpengb . AQHandles && ssGetLogOutput ( rtS )
) { sdiWriteSignal ( rtDW . aw1sjpengb . AQHandles , ssGetTaskTime ( rtS , 2
) , ( char * ) & rtB . gp3f1tfouj + 0 ) ; } } } UNUSED_PARAMETER ( tid ) ; }
void MdlOutputsTID3 ( int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void
MdlUpdate ( int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID3 (
int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) {
XDot * _rtXdot ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; memcpy ( &
_rtXdot -> epou2i2aqe [ 0 ] , & rtB . h1cjs4ulia [ 0 ] , 10U * sizeof (
real_T ) ) ; memcpy ( & _rtXdot -> b1mchtjukh [ 0 ] , & rtB . b3g5ksvk55 [ 0
] , 10U * sizeof ( real_T ) ) ; } void MdlProjection ( void ) { } void
MdlZeroCrossings ( void ) { __m128d tmp ; ZCV * _rtZCSV ; int32_T i ; _rtZCSV
= ( ( ZCV * ) ssGetSolverZcSignalVector ( rtS ) ) ; for ( i = 0 ; i <= 8 ; i
+= 2 ) { tmp = _mm_loadu_pd ( & rtB . kmwcbilhqq [ i ] ) ; _mm_storeu_pd ( &
_rtZCSV -> j23xiiu4aj [ i ] , _mm_sub_pd ( tmp , _mm_set1_pd ( rtP .
CheckStaticUpperBound_max ) ) ) ; } _rtZCSV -> ehx5ayseso = ssGetT ( rtS ) -
rtP . Step_Time ; } void MdlTerminate ( void ) { { if ( rtDW . bildbdyfzk .
AQHandles ) { sdiTerminateStreaming ( & rtDW . bildbdyfzk . AQHandles ) ; } }
{ if ( rtDW . h3mcukzweq . AQHandles ) { sdiTerminateStreaming ( & rtDW .
h3mcukzweq . AQHandles ) ; } } { if ( rtDW . fy4dfotmyf . AQHandles ) {
sdiTerminateStreaming ( & rtDW . fy4dfotmyf . AQHandles ) ; } } { if ( rtDW .
o4yowjfnvg . AQHandles ) { sdiTerminateStreaming ( & rtDW . o4yowjfnvg .
AQHandles ) ; } } { if ( rtDW . aw1sjpengb . AQHandles ) {
sdiTerminateStreaming ( & rtDW . aw1sjpengb . AQHandles ) ; } } } static void
mr_drone_5DOF_cacheDataAsMxArray ( mxArray * destArray , mwIndex i , int j ,
const void * srcData , size_t numBytes ) ; static void
mr_drone_5DOF_cacheDataAsMxArray ( mxArray * destArray , mwIndex i , int j ,
const void * srcData , size_t numBytes ) { mxArray * newArray =
mxCreateUninitNumericMatrix ( ( size_t ) 1 , numBytes , mxUINT8_CLASS ,
mxREAL ) ; memcpy ( ( uint8_T * ) mxGetData ( newArray ) , ( const uint8_T *
) srcData , numBytes ) ; mxSetFieldByNumber ( destArray , i , j , newArray )
; } static void mr_drone_5DOF_restoreDataFromMxArray ( void * destData ,
const mxArray * srcArray , mwIndex i , int j , size_t numBytes ) ; static
void mr_drone_5DOF_restoreDataFromMxArray ( void * destData , const mxArray *
srcArray , mwIndex i , int j , size_t numBytes ) { memcpy ( ( uint8_T * )
destData , ( const uint8_T * ) mxGetData ( mxGetFieldByNumber ( srcArray , i
, j ) ) , numBytes ) ; } static void mr_drone_5DOF_cacheBitFieldToMxArray (
mxArray * destArray , mwIndex i , int j , uint_T bitVal ) ; static void
mr_drone_5DOF_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex i , int
j , uint_T bitVal ) { mxSetFieldByNumber ( destArray , i , j ,
mxCreateDoubleScalar ( ( real_T ) bitVal ) ) ; } static uint_T
mr_drone_5DOF_extractBitFieldFromMxArray ( const mxArray * srcArray , mwIndex
i , int j , uint_T numBits ) ; static uint_T
mr_drone_5DOF_extractBitFieldFromMxArray ( const mxArray * srcArray , mwIndex
i , int j , uint_T numBits ) { const uint_T varVal = ( uint_T ) mxGetScalar (
mxGetFieldByNumber ( srcArray , i , j ) ) ; return varVal & ( ( 1u << numBits
) - 1u ) ; } static void mr_drone_5DOF_cacheDataToMxArrayWithOffset ( mxArray
* destArray , mwIndex i , int j , mwIndex offset , const void * srcData ,
size_t numBytes ) ; static void mr_drone_5DOF_cacheDataToMxArrayWithOffset (
mxArray * destArray , mwIndex i , int j , mwIndex offset , const void *
srcData , size_t numBytes ) { uint8_T * varData = ( uint8_T * ) mxGetData (
mxGetFieldByNumber ( destArray , i , j ) ) ; memcpy ( ( uint8_T * ) & varData
[ offset * numBytes ] , ( const uint8_T * ) srcData , numBytes ) ; } static
void mr_drone_5DOF_restoreDataFromMxArrayWithOffset ( void * destData , const
mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t numBytes ) ;
static void mr_drone_5DOF_restoreDataFromMxArrayWithOffset ( void * destData
, const mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t
numBytes ) { const uint8_T * varData = ( const uint8_T * ) mxGetData (
mxGetFieldByNumber ( srcArray , i , j ) ) ; memcpy ( ( uint8_T * ) destData ,
( const uint8_T * ) & varData [ offset * numBytes ] , numBytes ) ; } static
void mr_drone_5DOF_cacheBitFieldToCellArrayWithOffset ( mxArray * destArray ,
mwIndex i , int j , mwIndex offset , uint_T fieldVal ) ; static void
mr_drone_5DOF_cacheBitFieldToCellArrayWithOffset ( mxArray * destArray ,
mwIndex i , int j , mwIndex offset , uint_T fieldVal ) { mxSetCell (
mxGetFieldByNumber ( destArray , i , j ) , offset , mxCreateDoubleScalar ( (
real_T ) fieldVal ) ) ; } static uint_T
mr_drone_5DOF_extractBitFieldFromCellArrayWithOffset ( const mxArray *
srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) ; static
uint_T mr_drone_5DOF_extractBitFieldFromCellArrayWithOffset ( const mxArray *
srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) { const
uint_T fieldVal = ( uint_T ) mxGetScalar ( mxGetCell ( mxGetFieldByNumber (
srcArray , i , j ) , offset ) ) ; return fieldVal & ( ( 1u << numBits ) - 1u
) ; } mxArray * mr_drone_5DOF_GetDWork ( ) { static const char_T *
ssDWFieldNames [ 3 ] = { "rtB" , "rtDW" , "NULL_PrevZCX" , } ; mxArray * ssDW
= mxCreateStructMatrix ( 1 , 1 , 3 , ssDWFieldNames ) ;
mr_drone_5DOF_cacheDataAsMxArray ( ssDW , 0 , 0 , ( const void * ) & ( rtB )
, sizeof ( rtB ) ) ; { static const char_T * rtdwDataFieldNames [ 7 ] = {
"rtDW.o3aryxf3r0" , "rtDW.atzgtyu2uw" , "rtDW.bgwp0luil4" , "rtDW.prcj0bragy"
, "rtDW.e3nps1yxro" , "rtDW.hwccjcyk2m" , "rtDW.maae0yta5q" , } ; mxArray *
rtdwData = mxCreateStructMatrix ( 1 , 1 , 7 , rtdwDataFieldNames ) ;
mr_drone_5DOF_cacheDataAsMxArray ( rtdwData , 0 , 0 , ( const void * ) & (
rtDW . o3aryxf3r0 ) , sizeof ( rtDW . o3aryxf3r0 ) ) ;
mr_drone_5DOF_cacheDataAsMxArray ( rtdwData , 0 , 1 , ( const void * ) & (
rtDW . atzgtyu2uw ) , sizeof ( rtDW . atzgtyu2uw ) ) ;
mr_drone_5DOF_cacheDataAsMxArray ( rtdwData , 0 , 2 , ( const void * ) & (
rtDW . bgwp0luil4 ) , sizeof ( rtDW . bgwp0luil4 ) ) ;
mr_drone_5DOF_cacheDataAsMxArray ( rtdwData , 0 , 3 , ( const void * ) & (
rtDW . prcj0bragy ) , sizeof ( rtDW . prcj0bragy ) ) ;
mr_drone_5DOF_cacheDataAsMxArray ( rtdwData , 0 , 4 , ( const void * ) & (
rtDW . e3nps1yxro ) , sizeof ( rtDW . e3nps1yxro ) ) ;
mr_drone_5DOF_cacheDataAsMxArray ( rtdwData , 0 , 5 , ( const void * ) & (
rtDW . hwccjcyk2m ) , sizeof ( rtDW . hwccjcyk2m ) ) ;
mr_drone_5DOF_cacheDataAsMxArray ( rtdwData , 0 , 6 , ( const void * ) & (
rtDW . maae0yta5q ) , sizeof ( rtDW . maae0yta5q ) ) ; mxSetFieldByNumber (
ssDW , 0 , 1 , rtdwData ) ; } return ssDW ; } void mr_drone_5DOF_SetDWork (
const mxArray * ssDW ) { ( void ) ssDW ; mr_drone_5DOF_restoreDataFromMxArray
( ( void * ) & ( rtB ) , ssDW , 0 , 0 , sizeof ( rtB ) ) ; { const mxArray *
rtdwData = mxGetFieldByNumber ( ssDW , 0 , 1 ) ;
mr_drone_5DOF_restoreDataFromMxArray ( ( void * ) & ( rtDW . o3aryxf3r0 ) ,
rtdwData , 0 , 0 , sizeof ( rtDW . o3aryxf3r0 ) ) ;
mr_drone_5DOF_restoreDataFromMxArray ( ( void * ) & ( rtDW . atzgtyu2uw ) ,
rtdwData , 0 , 1 , sizeof ( rtDW . atzgtyu2uw ) ) ;
mr_drone_5DOF_restoreDataFromMxArray ( ( void * ) & ( rtDW . bgwp0luil4 ) ,
rtdwData , 0 , 2 , sizeof ( rtDW . bgwp0luil4 ) ) ;
mr_drone_5DOF_restoreDataFromMxArray ( ( void * ) & ( rtDW . prcj0bragy ) ,
rtdwData , 0 , 3 , sizeof ( rtDW . prcj0bragy ) ) ;
mr_drone_5DOF_restoreDataFromMxArray ( ( void * ) & ( rtDW . e3nps1yxro ) ,
rtdwData , 0 , 4 , sizeof ( rtDW . e3nps1yxro ) ) ;
mr_drone_5DOF_restoreDataFromMxArray ( ( void * ) & ( rtDW . hwccjcyk2m ) ,
rtdwData , 0 , 5 , sizeof ( rtDW . hwccjcyk2m ) ) ;
mr_drone_5DOF_restoreDataFromMxArray ( ( void * ) & ( rtDW . maae0yta5q ) ,
rtdwData , 0 , 6 , sizeof ( rtDW . maae0yta5q ) ) ; } } mxArray *
mr_drone_5DOF_GetSimStateDisallowedBlocks ( ) { mxArray * data =
mxCreateCellMatrix ( 1 , 3 ) ; mwIndex subs [ 2 ] , offset ; { static const
char_T * blockType [ 1 ] = { "Scope" , } ; static const char_T * blockPath [
1 ] = { "drone_5DOF/Scope" , } ; static const int reason [ 1 ] = { 0 , } ;
for ( subs [ 0 ] = 0 ; subs [ 0 ] < 1 ; ++ ( subs [ 0 ] ) ) { subs [ 1 ] = 0
; offset = mxCalcSingleSubscript ( data , 2 , subs ) ; mxSetCell ( data ,
offset , mxCreateString ( blockType [ subs [ 0 ] ] ) ) ; subs [ 1 ] = 1 ;
offset = mxCalcSingleSubscript ( data , 2 , subs ) ; mxSetCell ( data ,
offset , mxCreateString ( blockPath [ subs [ 0 ] ] ) ) ; subs [ 1 ] = 2 ;
offset = mxCalcSingleSubscript ( data , 2 , subs ) ; mxSetCell ( data ,
offset , mxCreateDoubleScalar ( ( real_T ) reason [ subs [ 0 ] ] ) ) ; } }
return data ; } void MdlInitializeSizes ( void ) { ssSetNumContStates ( rtS ,
20 ) ; ssSetNumPeriodicContStates ( rtS , 0 ) ; ssSetNumY ( rtS , 0 ) ;
ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS , 0 ) ;
ssSetNumSampleTimes ( rtS , 3 ) ; ssSetNumBlocks ( rtS , 27 ) ;
ssSetNumBlockIO ( rtS , 8 ) ; ssSetNumBlockParams ( rtS , 191 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.0 ) ; ssSetSampleTime ( rtS , 2 , 0.01 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 1.0 ) ;
ssSetOffsetTime ( rtS , 2 , 0.0 ) ; } void raccel_set_checksum ( ) {
ssSetChecksumVal ( rtS , 0 , 1876633778U ) ; ssSetChecksumVal ( rtS , 1 ,
710715817U ) ; ssSetChecksumVal ( rtS , 2 , 2342842469U ) ; ssSetChecksumVal
( rtS , 3 , 1004280244U ) ; }
#if defined(_MSC_VER)
#pragma optimize( "", off )
#endif
SimStruct * raccel_register_model ( ssExecutionInfo * executionInfo ) {
static struct _ssMdlInfo mdlInfo ; static struct _ssBlkInfo2 blkInfo2 ;
static struct _ssBlkInfoSLSize blkInfoSLSize ; rt_modelMapInfoPtr = & (
rt_dataMapInfo . mmi ) ; executionInfo -> gblObjects_ . numToFiles = 0 ;
executionInfo -> gblObjects_ . numFrFiles = 0 ; executionInfo -> gblObjects_
. numFrWksBlocks = 0 ; executionInfo -> gblObjects_ . numModelInputs = 0 ;
executionInfo -> gblObjects_ . numRootInportBlks = 0 ; executionInfo ->
gblObjects_ . inportDataTypeIdx = NULL ; executionInfo -> gblObjects_ .
inportDims = NULL ; executionInfo -> gblObjects_ . inportComplex = NULL ;
executionInfo -> gblObjects_ . inportInterpoFlag = NULL ; executionInfo ->
gblObjects_ . inportContinuous = NULL ; ( void ) memset ( ( char_T * ) rtS ,
0 , sizeof ( SimStruct ) ) ; ( void ) memset ( ( char_T * ) & mdlInfo , 0 ,
sizeof ( struct _ssMdlInfo ) ) ; ( void ) memset ( ( char_T * ) & blkInfo2 ,
0 , sizeof ( struct _ssBlkInfo2 ) ) ; ( void ) memset ( ( char_T * ) &
blkInfoSLSize , 0 , sizeof ( struct _ssBlkInfoSLSize ) ) ; ssSetBlkInfo2Ptr (
rtS , & blkInfo2 ) ; ssSetBlkInfoSLSizePtr ( rtS , & blkInfoSLSize ) ;
ssSetMdlInfoPtr ( rtS , & mdlInfo ) ; ssSetExecutionInfo ( rtS ,
executionInfo ) ; slsaAllocOPModelData ( rtS ) ; { static time_T mdlPeriod [
NSAMPLE_TIMES ] ; static time_T mdlOffset [ NSAMPLE_TIMES ] ; static time_T
mdlTaskTimes [ NSAMPLE_TIMES ] ; static int_T mdlTsMap [ NSAMPLE_TIMES ] ;
static int_T mdlSampleHits [ NSAMPLE_TIMES ] ; static boolean_T
mdlTNextWasAdjustedPtr [ NSAMPLE_TIMES ] ; static int_T mdlPerTaskSampleHits
[ NSAMPLE_TIMES * NSAMPLE_TIMES ] ; static time_T mdlTimeOfNextSampleHit [
NSAMPLE_TIMES ] ; { int_T i ; for ( i = 0 ; i < NSAMPLE_TIMES ; i ++ ) {
mdlPeriod [ i ] = 0.0 ; mdlOffset [ i ] = 0.0 ; mdlTaskTimes [ i ] = 0.0 ;
mdlTsMap [ i ] = i ; mdlSampleHits [ i ] = 1 ; } } ssSetSampleTimePtr ( rtS ,
& mdlPeriod [ 0 ] ) ; ssSetOffsetTimePtr ( rtS , & mdlOffset [ 0 ] ) ;
ssSetSampleTimeTaskIDPtr ( rtS , & mdlTsMap [ 0 ] ) ; ssSetTPtr ( rtS , &
mdlTaskTimes [ 0 ] ) ; ssSetSampleHitPtr ( rtS , & mdlSampleHits [ 0 ] ) ;
ssSetTNextWasAdjustedPtr ( rtS , & mdlTNextWasAdjustedPtr [ 0 ] ) ;
ssSetPerTaskSampleHitsPtr ( rtS , & mdlPerTaskSampleHits [ 0 ] ) ;
ssSetTimeOfNextSampleHitPtr ( rtS , & mdlTimeOfNextSampleHit [ 0 ] ) ; }
ssSetSolverMode ( rtS , SOLVER_MODE_SINGLETASKING ) ; { ssSetBlockIO ( rtS ,
( ( void * ) & rtB ) ) ; ( void ) memset ( ( ( void * ) & rtB ) , 0 , sizeof
( B ) ) ; } { real_T * x = ( real_T * ) & rtX ; ssSetContStates ( rtS , x ) ;
( void ) memset ( ( void * ) x , 0 , sizeof ( X ) ) ; } { void * dwork = (
void * ) & rtDW ; ssSetRootDWork ( rtS , dwork ) ; ( void ) memset ( dwork ,
0 , sizeof ( DW ) ) ; } { static DataTypeTransInfo dtInfo ; ( void ) memset (
( char_T * ) & dtInfo , 0 , sizeof ( dtInfo ) ) ; ssSetModelMappingInfo ( rtS
, & dtInfo ) ; dtInfo . numDataTypes = 23 ; dtInfo . dataTypeSizes = &
rtDataTypeSizes [ 0 ] ; dtInfo . dataTypeNames = & rtDataTypeNames [ 0 ] ;
dtInfo . BTransTable = & rtBTransTable ; dtInfo . PTransTable = &
rtPTransTable ; dtInfo . dataTypeInfoTable = rtDataTypeInfoTable ; }
drone_5DOF_InitializeDataMapInfo ( ) ; ssSetIsRapidAcceleratorActive ( rtS ,
true ) ; ssSetRootSS ( rtS , rtS ) ; ssSetVersion ( rtS ,
SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS , "drone_5DOF" ) ;
ssSetPath ( rtS , "drone_5DOF" ) ; ssSetTStart ( rtS , 0.0 ) ; ssSetTFinal (
rtS , 10.0 ) ; { static RTWLogInfo rt_DataLoggingInfo ; rt_DataLoggingInfo .
loggingInterval = ( NULL ) ; ssSetRTWLogInfo ( rtS , & rt_DataLoggingInfo ) ;
} { { static int_T rt_LoggedStateWidths [ ] = { 10 , 10 } ; static int_T
rt_LoggedStateNumDimensions [ ] = { 1 , 1 } ; static int_T
rt_LoggedStateDimensions [ ] = { 10 , 10 } ; static boolean_T
rt_LoggedStateIsVarDims [ ] = { 0 , 0 } ; static BuiltInDTypeId
rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE , SS_DOUBLE } ; static int_T
rt_LoggedStateComplexSignals [ ] = { 0 , 0 } ; static RTWPreprocessingFcnPtr
rt_LoggingStatePreprocessingFcnPtrs [ ] = { ( NULL ) , ( NULL ) } ; static
const char_T * rt_LoggedStateLabels [ ] = { "CSTATE" , "CSTATE" } ; static
const char_T * rt_LoggedStateBlockNames [ ] = { "drone_5DOF/Integrator1" ,
"drone_5DOF/Integrator" } ; static const char_T * rt_LoggedStateNames [ ] = {
"" , "" } ; static boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 } ;
static RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE
, SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 ,
0 , 0 , 1.0 , 0 , 0.0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 0 , 1
} ; static RTWLogSignalInfo rt_LoggedStateSignalInfo = { 2 ,
rt_LoggedStateWidths , rt_LoggedStateNumDimensions , rt_LoggedStateDimensions
, rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , rt_LoggingStatePreprocessingFcnPtrs
, { rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert , rt_LoggedStateIdxList
} ; static void * rt_LoggedStateSignalPtrs [ 2 ] ; rtliSetLogXSignalPtrs (
ssGetRTWLogInfo ( rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . epou2i2aqe [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . b1mchtjukh [ 0 ] ; }
rtliSetLogT ( ssGetRTWLogInfo ( rtS ) , "tout" ) ; rtliSetLogX (
ssGetRTWLogInfo ( rtS ) , "" ) ; rtliSetLogXFinal ( ssGetRTWLogInfo ( rtS ) ,
"xFinal" ) ; rtliSetLogVarNameModifier ( ssGetRTWLogInfo ( rtS ) , "none" ) ;
rtliSetLogFormat ( ssGetRTWLogInfo ( rtS ) , 4 ) ; rtliSetLogMaxRows (
ssGetRTWLogInfo ( rtS ) , 0 ) ; rtliSetLogDecimation ( ssGetRTWLogInfo ( rtS
) , 1 ) ; rtliSetLogY ( ssGetRTWLogInfo ( rtS ) , "" ) ;
rtliSetLogYSignalInfo ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ;
rtliSetLogYSignalPtrs ( ssGetRTWLogInfo ( rtS ) , ( NULL ) ) ; } { static
struct _ssStatesInfo2 statesInfo2 ; ssSetStatesInfo2 ( rtS , & statesInfo2 )
; } { static ssPeriodicStatesInfo periodicStatesInfo ;
ssSetPeriodicStatesInfo ( rtS , & periodicStatesInfo ) ; } { static
ssJacobianPerturbationBounds jacobianPerturbationBounds ;
ssSetJacobianPerturbationBounds ( rtS , & jacobianPerturbationBounds ) ; } {
static ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 20 ] ;
static real_T absTol [ 20 ] = { 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 } ; static
uint8_T absTolControl [ 20 ] = { 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U ,
0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U } ; static real_T
contStateJacPerturbBoundMinVec [ 20 ] ; static real_T
contStateJacPerturbBoundMaxVec [ 20 ] ; static uint8_T zcAttributes [ 11 ] =
{ ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) ,
( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , (
ZC_EVENT_ALL ) , ( ZC_EVENT_ALL ) , ( ZC_EVENT_ALL_UP ) } ; static
ssNonContDerivSigInfo nonContDerivSigInfo [ 1 ] = { { 3 * sizeof ( real_T ) ,
( char * ) ( & rtB . fx25dj0plx [ 0 ] ) , ( NULL ) } } ; { int i ; for ( i =
0 ; i < 20 ; ++ i ) { contStateJacPerturbBoundMinVec [ i ] = 0 ;
contStateJacPerturbBoundMaxVec [ i ] = rtGetInf ( ) ; } } ssSetSolverRelTol (
rtS , 0.001 ) ; ssSetStepSize ( rtS , 0.0 ) ; ssSetMinStepSize ( rtS , 0.0 )
; ssSetMaxNumMinSteps ( rtS , - 1 ) ; ssSetMinStepViolatedError ( rtS , 0 ) ;
ssSetMaxStepSize ( rtS , 0.001 ) ; ssSetSolverMaxOrder ( rtS , - 1 ) ;
ssSetSolverRefineFactor ( rtS , 1 ) ; ssSetOutputTimes ( rtS , ( NULL ) ) ;
ssSetNumOutputTimes ( rtS , 0 ) ; ssSetOutputTimesOnly ( rtS , 0 ) ;
ssSetOutputTimesIndex ( rtS , 0 ) ; ssSetZCCacheNeedsReset ( rtS , 0 ) ;
ssSetDerivCacheNeedsReset ( rtS , 0 ) ; ssSetNumNonContDerivSigInfos ( rtS ,
1 ) ; ssSetNonContDerivSigInfos ( rtS , nonContDerivSigInfo ) ;
ssSetSolverInfo ( rtS , & slvrInfo ) ; ssSetSolverName ( rtS , "ode45" ) ;
ssSetVariableStepSolver ( rtS , 1 ) ; ssSetSolverConsistencyChecking ( rtS ,
0 ) ; ssSetSolverAdaptiveZcDetection ( rtS , 0 ) ;
ssSetSolverRobustResetMethod ( rtS , 0 ) ; ssSetAbsTolVector ( rtS , absTol )
; ssSetAbsTolControlVector ( rtS , absTolControl ) ;
ssSetSolverAbsTol_Obsolete ( rtS , absTol ) ;
ssSetSolverAbsTolControl_Obsolete ( rtS , absTolControl ) ;
ssSetJacobianPerturbationBoundsMinVec ( rtS , contStateJacPerturbBoundMinVec
) ; ssSetJacobianPerturbationBoundsMaxVec ( rtS ,
contStateJacPerturbBoundMaxVec ) ; ssSetSolverStateProjection ( rtS , 0 ) ;
ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetModelDerivatives ( rtS ,
MdlDerivatives ) ; ssSetSolverZcSignalAttrib ( rtS , zcAttributes ) ;
ssSetSolverNumZcSignals ( rtS , 11 ) ; ssSetModelZeroCrossings ( rtS ,
MdlZeroCrossings ) ; ssSetSolverConsecutiveZCsStepRelTol ( rtS ,
2.8421709430404007E-13 ) ; ssSetSolverMaxConsecutiveZCs ( rtS , 1000 ) ;
ssSetSolverConsecutiveZCsError ( rtS , 2 ) ; ssSetSolverMaskedZcDiagnostic (
rtS , 1 ) ; ssSetSolverIgnoredZcDiagnostic ( rtS , 1 ) ;
ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ;
ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid ( rtS , INT_MIN )
; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 11 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; }
ssSetChecksumVal ( rtS , 0 , 1876633778U ) ; ssSetChecksumVal ( rtS , 1 ,
710715817U ) ; ssSetChecksumVal ( rtS , 2 , 2342842469U ) ; ssSetChecksumVal
( rtS , 3 , 1004280244U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 2 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
rteiSetModelMappingInfoPtr ( ssGetRTWExtModeInfo ( rtS ) , &
ssGetModelMappingInfo ( rtS ) ) ; rteiSetChecksumsPtr ( ssGetRTWExtModeInfo (
rtS ) , ssGetChecksums ( rtS ) ) ; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS )
, ssGetTPtr ( rtS ) ) ; } slsaDisallowedBlocksForSimTargetOP ( rtS ,
mr_drone_5DOF_GetSimStateDisallowedBlocks ) ; slsaGetWorkFcnForSimTargetOP (
rtS , mr_drone_5DOF_GetDWork ) ; slsaSetWorkFcnForSimTargetOP ( rtS ,
mr_drone_5DOF_SetDWork ) ; rt_RapidReadMatFileAndUpdateParams ( rtS ) ; if (
ssGetErrorStatus ( rtS ) ) { return rtS ; } return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
void MdlOutputsParameterSampleTime ( int_T tid ) { MdlOutputsTID3 ( tid ) ; }
