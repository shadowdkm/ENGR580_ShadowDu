#include "drone_5DOF_observer.h"
#include "rtwtypes.h"
#include <stdlib.h>
#include "drone_5DOF_observer_private.h"
#include <emmintrin.h>
#include <string.h>
#include "mwmathutil.h"
#include "rt_logging_mmi.h"
#include "drone_5DOF_observer_capi.h"
#include <math.h>
#include "drone_5DOF_observer_dt.h"
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
#define mjenhlctum (-1)
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
MdlInitialize ( void ) { int32_T i ; for ( i = 0 ; i < 10 ; i ++ ) { rtX .
eent0niwjp [ i ] = rtP . x0 [ i ] ; rtX . d3azydidno [ i ] = rtP .
Integrator2_IC ; } for ( i = 0 ; i < 7 ; i ++ ) { rtX . f0bfxnhi2t [ i ] =
rtP . Integrator1_IC ; } rtDW . a2bz2wiwxl = false ; rtDW . ou0nyrelia =
mjenhlctum ; rtDW . lw2jdaqrz3 = false ; rtDW . jkne1n5eru = mjenhlctum ; }
void MdlStart ( void ) { uint32_T initSeed ; { bool
externalInputIsInDatasetFormat = false ; void * pISigstreamManager =
rt_GetISigstreamManager ( rtS ) ;
rtwISigstreamManagerGetInputIsInDatasetFormat ( pISigstreamManager , &
externalInputIsInDatasetFormat ) ; if ( externalInputIsInDatasetFormat ) { }
} { { { bool isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU
srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars ( "Clock" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Clock" ) ; sdiLabelU blockPath = sdiGetLabelFromChars
( "drone_5DOF_observer/To Workspace1" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Clock" ) ;
sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 1 ] = { 1 } ; sigDims . nDims = 1 ; sigDims . dimensions = sigDimsArray ;
srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU
) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ; srcInfo .
subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo . signalName =
sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . l0qhxzdoan . AQHandles =
sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo . mmi .
InstanceMap . fullPath , "978d51b5-edbf-4ca2-a1c3-e680a4c8a34a" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ;
sdiCompleteAsyncioQueueCreation ( rtDW . l0qhxzdoan . AQHandles , hDT , &
srcInfo ) ; if ( rtDW . l0qhxzdoan . AQHandles ) {
sdiSetSignalSampleTimeString ( rtDW . l0qhxzdoan . AQHandles , "0.01" , 0.01
, ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW . l0qhxzdoan . AQHandles
, 0.0 ) ; sdiSetRunStartTime ( rtDW . l0qhxzdoan . AQHandles , ssGetTaskTime
( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings ( rtDW . l0qhxzdoan .
AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName ( rtDW . l0qhxzdoan .
AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . l0qhxzdoan . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"simuT" ) ; sdiRegisterWksVariable ( rtDW . l0qhxzdoan . AQHandles , varName
, "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } { { { bool
isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU srcInfo ;
sdiLabelU loggedName = sdiGetLabelFromChars ( "Gain5" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Gain5" ) ; sdiLabelU blockPath = sdiGetLabelFromChars
( "drone_5DOF_observer/To Workspace2" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Gain5" ) ;
sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 2 ] = { 7 , 1 } ; sigDims . nDims = 2 ; sigDims . dimensions = sigDimsArray
; srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = (
sdiFullBlkPathU ) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ;
srcInfo . subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo .
signalName = sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . jmxv5akigp .
AQHandles = sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo .
mmi . InstanceMap . fullPath , "66847e0b-2e78-45b2-9314-4cd65e4a11f3" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ; if ( rtDW
. jmxv5akigp . AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . jmxv5akigp
. AQHandles , "0.01" , 0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate (
rtDW . jmxv5akigp . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW .
jmxv5akigp . AQHandles , ssGetTaskTime ( rtS , 2 ) ) ;
sdiAsyncRepoSetSignalExportSettings ( rtDW . jmxv5akigp . AQHandles , 1 , 0 )
; sdiAsyncRepoSetSignalExportName ( rtDW . jmxv5akigp . AQHandles ,
loggedName , origSigName , propName ) ; sdiAsyncRepoSetBlockPathDomain ( rtDW
. jmxv5akigp . AQHandles ) ; sdiSetSignalIsFrameBased ( rtDW . jmxv5akigp .
AQHandles , true ) ; sdiCompleteAsyncioQueueCreation ( rtDW . jmxv5akigp .
AQHandles , hDT , & srcInfo ) ; } sdiFreeLabel ( sigName ) ; sdiFreeLabel (
loggedName ) ; sdiFreeLabel ( origSigName ) ; sdiFreeLabel ( propName ) ;
sdiFreeLabel ( blockPath ) ; sdiFreeLabel ( blockSID ) ; sdiFreeLabel (
subPath ) ; } } if ( ! isStreamoutAlreadyRegistered ) { { sdiLabelU varName =
sdiGetLabelFromChars ( "simuReal2bEstOutput" ) ; sdiRegisterWksVariable (
rtDW . jmxv5akigp . AQHandles , varName , "array2D" ) ; sdiFreeLabel (
varName ) ; } } } } } { { { bool isStreamoutAlreadyRegistered = false ; {
sdiSignalSourceInfoU srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars (
"Gain9" ) ; sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU
propName = sdiGetLabelFromChars ( "Gain9" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF_observer/To Workspace3" ) ; sdiLabelU
blockSID = sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath =
sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ; sdiLabelU sigName =
sdiGetLabelFromChars ( "Gain9" ) ; sdiAsyncRepoDataTypeHandle hDT =
sdiAsyncRepoGetBuiltInDataTypeHandle ( DATA_TYPE_DOUBLE ) ; { sdiComplexity
sigComplexity = REAL ; sdiSampleTimeContinuity stCont =
SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray [ 2 ] = { 7 , 1 } ; sigDims .
nDims = 2 ; sigDims . dimensions = sigDimsArray ; srcInfo . numBlockPathElems
= 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU ) & blockPath ; srcInfo .
SID = ( sdiSignalIDU ) & blockSID ; srcInfo . subPath = subPath ; srcInfo .
portIndex = 0 + 1 ; srcInfo . signalName = sigName ; srcInfo . sigSourceUUID
= 0 ; rtDW . dgtxfbydlw . AQHandles = sdiStartAsyncioQueueCreation ( hDT , &
srcInfo , rt_dataMapInfo . mmi . InstanceMap . fullPath ,
"df01889e-16cd-4a1c-b871-7cdc74c0b47e" , sigComplexity , & sigDims ,
DIMENSIONS_MODE_FIXED , stCont , "" ) ; if ( rtDW . dgtxfbydlw . AQHandles )
{ sdiSetSignalSampleTimeString ( rtDW . dgtxfbydlw . AQHandles , "0.01" ,
0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW . dgtxfbydlw .
AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW . dgtxfbydlw . AQHandles ,
ssGetTaskTime ( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings ( rtDW .
dgtxfbydlw . AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName ( rtDW .
dgtxfbydlw . AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . dgtxfbydlw . AQHandles ) ;
sdiSetSignalIsFrameBased ( rtDW . dgtxfbydlw . AQHandles , true ) ;
sdiCompleteAsyncioQueueCreation ( rtDW . dgtxfbydlw . AQHandles , hDT , &
srcInfo ) ; } sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ;
sdiFreeLabel ( origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel (
blockPath ) ; sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if (
! isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars
( "simuEstOutput" ) ; sdiRegisterWksVariable ( rtDW . dgtxfbydlw . AQHandles
, varName , "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } { { { bool
isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU srcInfo ;
sdiLabelU loggedName = sdiGetLabelFromChars ( "MATLAB Function:1" ) ;
sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "MATLAB Function:1" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF_observer/To Workspace4" ) ; sdiLabelU
blockSID = sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath =
sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ; sdiLabelU sigName =
sdiGetLabelFromChars ( "MATLAB Function:1" ) ; sdiAsyncRepoDataTypeHandle hDT
= sdiAsyncRepoGetBuiltInDataTypeHandle ( DATA_TYPE_DOUBLE ) ; { sdiComplexity
sigComplexity = REAL ; sdiSampleTimeContinuity stCont =
SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray [ 1 ] = { 1 } ; sigDims . nDims =
1 ; sigDims . dimensions = sigDimsArray ; srcInfo . numBlockPathElems = 1 ;
srcInfo . fullBlockPath = ( sdiFullBlkPathU ) & blockPath ; srcInfo . SID = (
sdiSignalIDU ) & blockSID ; srcInfo . subPath = subPath ; srcInfo . portIndex
= 0 + 1 ; srcInfo . signalName = sigName ; srcInfo . sigSourceUUID = 0 ; rtDW
. preosqjjml . AQHandles = sdiStartAsyncioQueueCreation ( hDT , & srcInfo ,
rt_dataMapInfo . mmi . InstanceMap . fullPath ,
"3900f238-8d22-4375-906d-7d939d3b8918" , sigComplexity , & sigDims ,
DIMENSIONS_MODE_FIXED , stCont , "" ) ; sdiCompleteAsyncioQueueCreation (
rtDW . preosqjjml . AQHandles , hDT , & srcInfo ) ; if ( rtDW . preosqjjml .
AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . preosqjjml . AQHandles ,
"0.01" , 0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW .
preosqjjml . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW . preosqjjml .
AQHandles , ssGetTaskTime ( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings
( rtDW . preosqjjml . AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName (
rtDW . preosqjjml . AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . preosqjjml . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"simuTargetX" ) ; sdiRegisterWksVariable ( rtDW . preosqjjml . AQHandles ,
varName , "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } { { { bool
isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU srcInfo ;
sdiLabelU loggedName = sdiGetLabelFromChars ( "MATLAB Function:2" ) ;
sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "MATLAB Function:2" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF_observer/To Workspace5" ) ; sdiLabelU
blockSID = sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath =
sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ; sdiLabelU sigName =
sdiGetLabelFromChars ( "MATLAB Function:2" ) ; sdiAsyncRepoDataTypeHandle hDT
= sdiAsyncRepoGetBuiltInDataTypeHandle ( DATA_TYPE_DOUBLE ) ; { sdiComplexity
sigComplexity = REAL ; sdiSampleTimeContinuity stCont =
SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray [ 1 ] = { 1 } ; sigDims . nDims =
1 ; sigDims . dimensions = sigDimsArray ; srcInfo . numBlockPathElems = 1 ;
srcInfo . fullBlockPath = ( sdiFullBlkPathU ) & blockPath ; srcInfo . SID = (
sdiSignalIDU ) & blockSID ; srcInfo . subPath = subPath ; srcInfo . portIndex
= 0 + 1 ; srcInfo . signalName = sigName ; srcInfo . sigSourceUUID = 0 ; rtDW
. om1g1jtddz . AQHandles = sdiStartAsyncioQueueCreation ( hDT , & srcInfo ,
rt_dataMapInfo . mmi . InstanceMap . fullPath ,
"70cde016-b9cd-43f1-8da4-0e1d199eb0ab" , sigComplexity , & sigDims ,
DIMENSIONS_MODE_FIXED , stCont , "" ) ; sdiCompleteAsyncioQueueCreation (
rtDW . om1g1jtddz . AQHandles , hDT , & srcInfo ) ; if ( rtDW . om1g1jtddz .
AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . om1g1jtddz . AQHandles ,
"0.01" , 0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW .
om1g1jtddz . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW . om1g1jtddz .
AQHandles , ssGetTaskTime ( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings
( rtDW . om1g1jtddz . AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName (
rtDW . om1g1jtddz . AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . om1g1jtddz . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"simuTargetY" ) ; sdiRegisterWksVariable ( rtDW . om1g1jtddz . AQHandles ,
varName , "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } { { { bool
isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU srcInfo ;
sdiLabelU loggedName = sdiGetLabelFromChars ( "Integrator2" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Integrator2" ) ; sdiLabelU blockPath =
sdiGetLabelFromChars ( "drone_5DOF_observer/To Workspace6" ) ; sdiLabelU
blockSID = sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath =
sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ; sdiLabelU sigName =
sdiGetLabelFromChars ( "Integrator2" ) ; sdiAsyncRepoDataTypeHandle hDT =
sdiAsyncRepoGetBuiltInDataTypeHandle ( DATA_TYPE_DOUBLE ) ; { sdiComplexity
sigComplexity = REAL ; sdiSampleTimeContinuity stCont =
SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray [ 2 ] = { 10 , 1 } ; sigDims .
nDims = 2 ; sigDims . dimensions = sigDimsArray ; srcInfo . numBlockPathElems
= 1 ; srcInfo . fullBlockPath = ( sdiFullBlkPathU ) & blockPath ; srcInfo .
SID = ( sdiSignalIDU ) & blockSID ; srcInfo . subPath = subPath ; srcInfo .
portIndex = 0 + 1 ; srcInfo . signalName = sigName ; srcInfo . sigSourceUUID
= 0 ; rtDW . kg0cpsbr4n . AQHandles = sdiStartAsyncioQueueCreation ( hDT , &
srcInfo , rt_dataMapInfo . mmi . InstanceMap . fullPath ,
"d0c1b260-0e72-4e4b-9b62-0b335221e757" , sigComplexity , & sigDims ,
DIMENSIONS_MODE_FIXED , stCont , "" ) ; if ( rtDW . kg0cpsbr4n . AQHandles )
{ sdiSetSignalSampleTimeString ( rtDW . kg0cpsbr4n . AQHandles , "0.01" ,
0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW . kg0cpsbr4n .
AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW . kg0cpsbr4n . AQHandles ,
ssGetTaskTime ( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings ( rtDW .
kg0cpsbr4n . AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName ( rtDW .
kg0cpsbr4n . AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . kg0cpsbr4n . AQHandles ) ;
sdiSetSignalIsFrameBased ( rtDW . kg0cpsbr4n . AQHandles , true ) ;
sdiCompleteAsyncioQueueCreation ( rtDW . kg0cpsbr4n . AQHandles , hDT , &
srcInfo ) ; } sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ;
sdiFreeLabel ( origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel (
blockPath ) ; sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if (
! isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars
( "simuEstStates" ) ; sdiRegisterWksVariable ( rtDW . kg0cpsbr4n . AQHandles
, varName , "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } { { { bool
isStreamoutAlreadyRegistered = false ; { sdiSignalSourceInfoU srcInfo ;
sdiLabelU loggedName = sdiGetLabelFromChars ( "Sum9" ) ; sdiLabelU
origSigName = sdiGetLabelFromChars ( "" ) ; sdiLabelU propName =
sdiGetLabelFromChars ( "Sum9" ) ; sdiLabelU blockPath = sdiGetLabelFromChars
( "drone_5DOF_observer/To Workspace7" ) ; sdiLabelU blockSID =
sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath = sdiGetLabelFromChars ( "" )
; sdiDims sigDims ; sdiLabelU sigName = sdiGetLabelFromChars ( "Sum9" ) ;
sdiAsyncRepoDataTypeHandle hDT = sdiAsyncRepoGetBuiltInDataTypeHandle (
DATA_TYPE_DOUBLE ) ; { sdiComplexity sigComplexity = REAL ;
sdiSampleTimeContinuity stCont = SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray
[ 2 ] = { 10 , 1 } ; sigDims . nDims = 2 ; sigDims . dimensions =
sigDimsArray ; srcInfo . numBlockPathElems = 1 ; srcInfo . fullBlockPath = (
sdiFullBlkPathU ) & blockPath ; srcInfo . SID = ( sdiSignalIDU ) & blockSID ;
srcInfo . subPath = subPath ; srcInfo . portIndex = 0 + 1 ; srcInfo .
signalName = sigName ; srcInfo . sigSourceUUID = 0 ; rtDW . gnkcthcwfm .
AQHandles = sdiStartAsyncioQueueCreation ( hDT , & srcInfo , rt_dataMapInfo .
mmi . InstanceMap . fullPath , "374cf9a4-2e00-4b44-bc9d-b8bac6407bdf" ,
sigComplexity , & sigDims , DIMENSIONS_MODE_FIXED , stCont , "" ) ; if ( rtDW
. gnkcthcwfm . AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . gnkcthcwfm
. AQHandles , "0.01" , 0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate (
rtDW . gnkcthcwfm . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW .
gnkcthcwfm . AQHandles , ssGetTaskTime ( rtS , 2 ) ) ;
sdiAsyncRepoSetSignalExportSettings ( rtDW . gnkcthcwfm . AQHandles , 1 , 0 )
; sdiAsyncRepoSetSignalExportName ( rtDW . gnkcthcwfm . AQHandles ,
loggedName , origSigName , propName ) ; sdiAsyncRepoSetBlockPathDomain ( rtDW
. gnkcthcwfm . AQHandles ) ; sdiSetSignalIsFrameBased ( rtDW . gnkcthcwfm .
AQHandles , true ) ; sdiCompleteAsyncioQueueCreation ( rtDW . gnkcthcwfm .
AQHandles , hDT , & srcInfo ) ; } sdiFreeLabel ( sigName ) ; sdiFreeLabel (
loggedName ) ; sdiFreeLabel ( origSigName ) ; sdiFreeLabel ( propName ) ;
sdiFreeLabel ( blockPath ) ; sdiFreeLabel ( blockSID ) ; sdiFreeLabel (
subPath ) ; } } if ( ! isStreamoutAlreadyRegistered ) { { sdiLabelU varName =
sdiGetLabelFromChars ( "simuReal2bEstStates" ) ; sdiRegisterWksVariable (
rtDW . gnkcthcwfm . AQHandles , varName , "array2D" ) ; sdiFreeLabel (
varName ) ; } } } } } { { { bool isStreamoutAlreadyRegistered = false ; {
sdiSignalSourceInfoU srcInfo ; sdiLabelU loggedName = sdiGetLabelFromChars (
"MATLAB Function:3" ) ; sdiLabelU origSigName = sdiGetLabelFromChars ( "" ) ;
sdiLabelU propName = sdiGetLabelFromChars ( "MATLAB Function:3" ) ; sdiLabelU
blockPath = sdiGetLabelFromChars ( "drone_5DOF_observer/To Workspace8" ) ;
sdiLabelU blockSID = sdiGetLabelFromChars ( "" ) ; sdiLabelU subPath =
sdiGetLabelFromChars ( "" ) ; sdiDims sigDims ; sdiLabelU sigName =
sdiGetLabelFromChars ( "MATLAB Function:3" ) ; sdiAsyncRepoDataTypeHandle hDT
= sdiAsyncRepoGetBuiltInDataTypeHandle ( DATA_TYPE_DOUBLE ) ; { sdiComplexity
sigComplexity = REAL ; sdiSampleTimeContinuity stCont =
SAMPLE_TIME_CONTINUOUS ; int_T sigDimsArray [ 1 ] = { 1 } ; sigDims . nDims =
1 ; sigDims . dimensions = sigDimsArray ; srcInfo . numBlockPathElems = 1 ;
srcInfo . fullBlockPath = ( sdiFullBlkPathU ) & blockPath ; srcInfo . SID = (
sdiSignalIDU ) & blockSID ; srcInfo . subPath = subPath ; srcInfo . portIndex
= 0 + 1 ; srcInfo . signalName = sigName ; srcInfo . sigSourceUUID = 0 ; rtDW
. otdgdsww4h . AQHandles = sdiStartAsyncioQueueCreation ( hDT , & srcInfo ,
rt_dataMapInfo . mmi . InstanceMap . fullPath ,
"e2655de2-d527-491a-9ef1-235e5d4b2d84" , sigComplexity , & sigDims ,
DIMENSIONS_MODE_FIXED , stCont , "" ) ; sdiCompleteAsyncioQueueCreation (
rtDW . otdgdsww4h . AQHandles , hDT , & srcInfo ) ; if ( rtDW . otdgdsww4h .
AQHandles ) { sdiSetSignalSampleTimeString ( rtDW . otdgdsww4h . AQHandles ,
"0.01" , 0.01 , ssGetTFinal ( rtS ) ) ; sdiSetSignalRefRate ( rtDW .
otdgdsww4h . AQHandles , 0.0 ) ; sdiSetRunStartTime ( rtDW . otdgdsww4h .
AQHandles , ssGetTaskTime ( rtS , 2 ) ) ; sdiAsyncRepoSetSignalExportSettings
( rtDW . otdgdsww4h . AQHandles , 1 , 0 ) ; sdiAsyncRepoSetSignalExportName (
rtDW . otdgdsww4h . AQHandles , loggedName , origSigName , propName ) ;
sdiAsyncRepoSetBlockPathDomain ( rtDW . otdgdsww4h . AQHandles ) ; }
sdiFreeLabel ( sigName ) ; sdiFreeLabel ( loggedName ) ; sdiFreeLabel (
origSigName ) ; sdiFreeLabel ( propName ) ; sdiFreeLabel ( blockPath ) ;
sdiFreeLabel ( blockSID ) ; sdiFreeLabel ( subPath ) ; } } if ( !
isStreamoutAlreadyRegistered ) { { sdiLabelU varName = sdiGetLabelFromChars (
"simuTargetZ" ) ; sdiRegisterWksVariable ( rtDW . otdgdsww4h . AQHandles ,
varName , "array2D" ) ; sdiFreeLabel ( varName ) ; } } } } } initSeed = (
uint32_T ) ( 100000 * rand ( ) ) ; RandSrcCreateSeeds_64 ( initSeed , & rtDW
. myhykfgvxr , 1 ) ; RandSrcInitState_GZ ( & rtDW . myhykfgvxr , & rtDW .
mapbnxxsya [ 0 ] , 1 ) ; MdlInitialize ( ) ; } void MdlOutputs ( int_T tid )
{ __m128d tmp_e ; __m128d tmp_i ; real_T tmp [ 30 ] ; real_T blz51iaeyj [ 10
] ; real_T tmp_p [ 10 ] ; real_T oue3h5sy2u [ 7 ] ; real_T eku0karrxa [ 3 ] ;
real_T jmgmm3pcgj [ 3 ] ; real_T lfvkfa1w3y [ 3 ] ; real_T absx ; real_T
b_absx ; real_T ejxvev4ywd ; real_T kpomojrxkq ; int32_T i ; int32_T i_p ; if
( ssIsSampleHit ( rtS , 1 , 0 ) ) { rtDW . crw5jrqanh = ( ssGetTaskTime ( rtS
, 1 ) >= rtP . Step_Time ) ; rtDW . dubitlkqqo = ( ssGetTaskTime ( rtS , 1 )
>= rtP . Step1_Time ) ; if ( rtDW . crw5jrqanh == 1 ) { ejxvev4ywd = rtP .
Step_YFinal ; } else { ejxvev4ywd = rtP . Step_Y0 ; } if ( rtDW . dubitlkqqo
== 1 ) { b_absx = rtP . Step1_YFinal ; } else { b_absx = rtP . Step1_Y0 ; }
ejxvev4ywd += b_absx ; for ( i = 0 ; i <= 8 ; i += 2 ) { tmp_i = _mm_loadu_pd
( & rtP . DisturbanceVariance [ i ] ) ; _mm_storeu_pd ( & rtB . ccu4dpkpbc [
i ] , _mm_mul_pd ( tmp_i , _mm_set1_pd ( ejxvev4ywd ) ) ) ; } } for ( i = 0 ;
i <= 8 ; i += 2 ) { tmp_i = _mm_loadu_pd ( & rtX . eent0niwjp [ i ] ) ; tmp_e
= _mm_loadu_pd ( & rtB . ccu4dpkpbc [ i ] ) ; _mm_storeu_pd ( & rtB .
ahei004b0s [ i ] , _mm_add_pd ( tmp_i , tmp_e ) ) ; } for ( i_p = 0 ; i_p < 7
; i_p ++ ) { ejxvev4ywd = 0.0 ; for ( i = 0 ; i < 10 ; i ++ ) { ejxvev4ywd +=
rtP . Cob [ 7 * i + i_p ] * rtB . ahei004b0s [ i ] ; } rtB . dw5qib4r4x [ i_p
] = ejxvev4ywd ; } RandSrc_GZ_D ( & kpomojrxkq , & rtP . RandomSource_MeanVal
, 1 , & rtP . RandomSource_VarianceRTP , 1 , rtDW . mapbnxxsya , 1 , 1 ) ;
for ( i = 0 ; i < 7 ; i ++ ) { rtB . h43ikrnm2w [ i ] = rtP .
ObservedVariance [ i ] * kpomojrxkq + rtB . dw5qib4r4x [ i ] ; } memcpy ( &
rtB . afd4wlreda [ 0 ] , & rtX . d3azydidno [ 0 ] , 10U * sizeof ( real_T ) )
; for ( i_p = 0 ; i_p < 7 ; i_p ++ ) { kpomojrxkq = 0.0 ; for ( i = 0 ; i <
10 ; i ++ ) { kpomojrxkq += rtP . Cob [ 7 * i + i_p ] * rtB . afd4wlreda [ i
] ; } rtB . aqub0r2ekx [ i_p ] = kpomojrxkq ; } rtB . m3dx5xndf2 = ssGetT (
rtS ) ; kpomojrxkq = rtP . TrackingEnabled * rtB . m3dx5xndf2 ; rtDW .
ou0nyrelia = mjenhlctum ; ejxvev4ywd = muDoubleScalarTanh ( kpomojrxkq / 5.0
) ; b_absx = kpomojrxkq * 36.0 ; if ( muDoubleScalarIsInf ( b_absx ) ||
muDoubleScalarIsNaN ( b_absx ) ) { b_absx = ( rtNaN ) ; } else { b_absx =
muDoubleScalarRem ( b_absx , 360.0 ) ; absx = muDoubleScalarAbs ( b_absx ) ;
if ( absx > 180.0 ) { if ( b_absx > 0.0 ) { b_absx -= 360.0 ; } else { b_absx
+= 360.0 ; } absx = muDoubleScalarAbs ( b_absx ) ; } if ( absx <= 45.0 ) {
b_absx *= 0.017453292519943295 ; b_absx = muDoubleScalarSin ( b_absx ) ; }
else if ( absx <= 135.0 ) { if ( b_absx > 0.0 ) { b_absx = ( b_absx - 90.0 )
* 0.017453292519943295 ; b_absx = muDoubleScalarCos ( b_absx ) ; } else {
b_absx = ( b_absx + 90.0 ) * 0.017453292519943295 ; b_absx = -
muDoubleScalarCos ( b_absx ) ; } } else { if ( b_absx > 0.0 ) { b_absx = (
b_absx - 180.0 ) * 0.017453292519943295 ; } else { b_absx = ( b_absx + 180.0
) * 0.017453292519943295 ; } b_absx = - muDoubleScalarSin ( b_absx ) ; } }
rtB . g01h5zrukl = - ejxvev4ywd * b_absx ; kpomojrxkq *= 36.0 ; if (
muDoubleScalarIsInf ( kpomojrxkq ) || muDoubleScalarIsNaN ( kpomojrxkq ) ) {
kpomojrxkq = ( rtNaN ) ; } else { kpomojrxkq = muDoubleScalarRem ( kpomojrxkq
, 360.0 ) ; b_absx = muDoubleScalarAbs ( kpomojrxkq ) ; if ( b_absx > 180.0 )
{ if ( kpomojrxkq > 0.0 ) { kpomojrxkq -= 360.0 ; } else { kpomojrxkq +=
360.0 ; } b_absx = muDoubleScalarAbs ( kpomojrxkq ) ; } if ( b_absx <= 45.0 )
{ kpomojrxkq *= 0.017453292519943295 ; kpomojrxkq = muDoubleScalarCos (
kpomojrxkq ) ; } else if ( b_absx <= 135.0 ) { if ( kpomojrxkq > 0.0 ) {
kpomojrxkq = ( kpomojrxkq - 90.0 ) * 0.017453292519943295 ; kpomojrxkq = -
muDoubleScalarSin ( kpomojrxkq ) ; } else { kpomojrxkq = ( kpomojrxkq + 90.0
) * 0.017453292519943295 ; kpomojrxkq = muDoubleScalarSin ( kpomojrxkq ) ; }
} else { if ( kpomojrxkq > 0.0 ) { kpomojrxkq = ( kpomojrxkq - 180.0 ) *
0.017453292519943295 ; } else { kpomojrxkq = ( kpomojrxkq + 180.0 ) *
0.017453292519943295 ; } kpomojrxkq = - muDoubleScalarCos ( kpomojrxkq ) ; }
} rtB . a21stht3j2 = - ejxvev4ywd * kpomojrxkq ; rtB . f2t000eoch =
ejxvev4ywd ; if ( ssIsSampleHit ( rtS , 2 , 0 ) ) { { if ( rtDW . l0qhxzdoan
. AQHandles && ssGetLogOutput ( rtS ) ) { sdiWriteSignal ( rtDW . l0qhxzdoan
. AQHandles , ssGetTaskTime ( rtS , 2 ) , ( char * ) & rtB . m3dx5xndf2 + 0 )
; } } { if ( rtDW . jmxv5akigp . AQHandles && ssGetLogOutput ( rtS ) ) {
sdiWriteSignal ( rtDW . jmxv5akigp . AQHandles , ssGetTaskTime ( rtS , 2 ) ,
( char * ) & rtB . dw5qib4r4x [ 0 ] + 0 ) ; } } { if ( rtDW . dgtxfbydlw .
AQHandles && ssGetLogOutput ( rtS ) ) { sdiWriteSignal ( rtDW . dgtxfbydlw .
AQHandles , ssGetTaskTime ( rtS , 2 ) , ( char * ) & rtB . aqub0r2ekx [ 0 ] +
0 ) ; } } { if ( rtDW . preosqjjml . AQHandles && ssGetLogOutput ( rtS ) ) {
sdiWriteSignal ( rtDW . preosqjjml . AQHandles , ssGetTaskTime ( rtS , 2 ) ,
( char * ) & rtB . g01h5zrukl + 0 ) ; } } { if ( rtDW . om1g1jtddz .
AQHandles && ssGetLogOutput ( rtS ) ) { sdiWriteSignal ( rtDW . om1g1jtddz .
AQHandles , ssGetTaskTime ( rtS , 2 ) , ( char * ) & rtB . a21stht3j2 + 0 ) ;
} } { if ( rtDW . kg0cpsbr4n . AQHandles && ssGetLogOutput ( rtS ) ) {
sdiWriteSignal ( rtDW . kg0cpsbr4n . AQHandles , ssGetTaskTime ( rtS , 2 ) ,
( char * ) & rtB . afd4wlreda [ 0 ] + 0 ) ; } } { if ( rtDW . gnkcthcwfm .
AQHandles && ssGetLogOutput ( rtS ) ) { sdiWriteSignal ( rtDW . gnkcthcwfm .
AQHandles , ssGetTaskTime ( rtS , 2 ) , ( char * ) & rtB . ahei004b0s [ 0 ] +
0 ) ; } } { if ( rtDW . otdgdsww4h . AQHandles && ssGetLogOutput ( rtS ) ) {
sdiWriteSignal ( rtDW . otdgdsww4h . AQHandles , ssGetTaskTime ( rtS , 2 ) ,
( char * ) & rtB . f2t000eoch + 0 ) ; } } } blz51iaeyj [ 0 ] = rtB .
g01h5zrukl ; blz51iaeyj [ 1 ] = rtB . a21stht3j2 ; memcpy ( & blz51iaeyj [ 2
] , & rtP . Constant1_Value [ 0 ] , sizeof ( real_T ) << 3U ) ; oue3h5sy2u [
0 ] = rtP . Constant3_Value [ 0 ] ; oue3h5sy2u [ 1 ] = rtP . Constant3_Value
[ 1 ] ; oue3h5sy2u [ 2 ] = rtB . f2t000eoch ; oue3h5sy2u [ 3 ] = rtP .
Constant4_Value [ 0 ] ; oue3h5sy2u [ 4 ] = rtP . Constant4_Value [ 1 ] ;
oue3h5sy2u [ 5 ] = rtP . Constant4_Value [ 2 ] ; oue3h5sy2u [ 6 ] = rtP .
Constant4_Value [ 3 ] ; for ( i = 0 ; i < 7 ; i ++ ) { rtB . mqnbs0uqzk [ i ]
= oue3h5sy2u [ i ] - rtB . h43ikrnm2w [ i ] ; } for ( i_p = 0 ; i_p < 3 ; i_p
++ ) { ejxvev4ywd = 0.0 ; b_absx = 0.0 ; for ( i = 0 ; i < 7 ; i ++ ) {
ejxvev4ywd += rtP . Ki [ 3 * i + i_p ] * rtX . f0bfxnhi2t [ i ] ; b_absx +=
rtP . Kp [ 3 * i + i_p ] * rtB . mqnbs0uqzk [ i ] ; } lfvkfa1w3y [ i_p ] =
ejxvev4ywd + b_absx ; } for ( i_p = 0 ; i_p <= 28 ; i_p += 2 ) { tmp_i =
_mm_loadu_pd ( & rtP . K [ i_p ] ) ; _mm_storeu_pd ( & tmp [ i_p ] ,
_mm_mul_pd ( tmp_i , _mm_set1_pd ( - 1.0 ) ) ) ; } for ( i_p = 0 ; i_p <= 8 ;
i_p += 2 ) { tmp_i = _mm_loadu_pd ( & rtB . afd4wlreda [ i_p ] ) ; tmp_e =
_mm_loadu_pd ( & blz51iaeyj [ i_p ] ) ; _mm_storeu_pd ( & tmp_p [ i_p ] ,
_mm_add_pd ( tmp_i , tmp_e ) ) ; } rtDW . jkne1n5eru = mjenhlctum ; rtB .
lvo3gfwy02 [ 0 ] = rtB . ahei004b0s [ 3 ] ; rtB . lvo3gfwy02 [ 1 ] = rtB .
ahei004b0s [ 4 ] ; rtB . lvo3gfwy02 [ 2 ] = rtB . ahei004b0s [ 5 ] ; rtB .
lvo3gfwy02 [ 3 ] = - rtB . ahei004b0s [ 9 ] * rtB . ahei004b0s [ 5 ] - 9.81 *
muDoubleScalarSin ( rtB . ahei004b0s [ 7 ] ) ; rtB . lvo3gfwy02 [ 4 ] = 9.81
* muDoubleScalarCos ( rtB . ahei004b0s [ 7 ] ) * muDoubleScalarSin ( rtB .
ahei004b0s [ 6 ] ) + rtB . ahei004b0s [ 5 ] * rtB . ahei004b0s [ 8 ] ; rtB .
lvo3gfwy02 [ 6 ] = rtB . ahei004b0s [ 8 ] ; rtB . lvo3gfwy02 [ 7 ] = rtB .
ahei004b0s [ 9 ] ; for ( i = 0 ; i < 3 ; i ++ ) { kpomojrxkq = 0.0 ; for (
i_p = 0 ; i_p < 10 ; i_p ++ ) { kpomojrxkq += tmp [ 3 * i_p + i ] * tmp_p [
i_p ] ; } ejxvev4ywd = lfvkfa1w3y [ i ] ; jmgmm3pcgj [ i ] = ( rtP .
Constant2_Value [ i ] + ejxvev4ywd ) + kpomojrxkq ; eku0karrxa [ i ] =
ejxvev4ywd + kpomojrxkq ; } rtB . lvo3gfwy02 [ 5 ] = ( ( rtB . ahei004b0s [ 3
] * rtB . ahei004b0s [ 9 ] - rtB . ahei004b0s [ 4 ] * rtB . ahei004b0s [ 8 ]
) + 9.81 * muDoubleScalarCos ( rtB . ahei004b0s [ 7 ] ) * muDoubleScalarCos (
rtB . ahei004b0s [ 6 ] ) ) - jmgmm3pcgj [ 0 ] / 0.1 ; rtB . lvo3gfwy02 [ 8 ]
= jmgmm3pcgj [ 1 ] / 0.0001 ; rtB . lvo3gfwy02 [ 9 ] = jmgmm3pcgj [ 2 ] /
0.0001 ; for ( i_p = 0 ; i_p < 7 ; i_p ++ ) { oue3h5sy2u [ i_p ] = rtB .
aqub0r2ekx [ i_p ] - rtB . h43ikrnm2w [ i_p ] ; } for ( i_p = 0 ; i_p < 10 ;
i_p ++ ) { ejxvev4ywd = 0.0 ; for ( i = 0 ; i < 7 ; i ++ ) { ejxvev4ywd +=
rtP . L [ 10 * i + i_p ] * oue3h5sy2u [ i ] ; } b_absx = 0.0 ; for ( i = 0 ;
i < 10 ; i ++ ) { b_absx += rtP . A [ 10 * i + i_p ] * rtB . afd4wlreda [ i ]
; } rtB . c30cf505b4 [ i_p ] = ( ( ( rtP . B [ i_p + 10 ] * eku0karrxa [ 1 ]
+ rtP . B [ i_p ] * eku0karrxa [ 0 ] ) + rtP . B [ i_p + 20 ] * eku0karrxa [
2 ] ) - ejxvev4ywd ) + b_absx ; } UNUSED_PARAMETER ( tid ) ; } void
MdlOutputsTID3 ( int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlUpdate (
int_T tid ) { UNUSED_PARAMETER ( tid ) ; } void MdlUpdateTID3 ( int_T tid ) {
UNUSED_PARAMETER ( tid ) ; } void MdlDerivatives ( void ) { XDot * _rtXdot ;
int32_T i ; _rtXdot = ( ( XDot * ) ssGetdX ( rtS ) ) ; memcpy ( & _rtXdot ->
eent0niwjp [ 0 ] , & rtB . lvo3gfwy02 [ 0 ] , 10U * sizeof ( real_T ) ) ;
memcpy ( & _rtXdot -> d3azydidno [ 0 ] , & rtB . c30cf505b4 [ 0 ] , 10U *
sizeof ( real_T ) ) ; for ( i = 0 ; i < 7 ; i ++ ) { _rtXdot -> f0bfxnhi2t [
i ] = rtB . mqnbs0uqzk [ i ] ; } } void MdlProjection ( void ) { } void
MdlZeroCrossings ( void ) { ZCV * _rtZCSV ; _rtZCSV = ( ( ZCV * )
ssGetSolverZcSignalVector ( rtS ) ) ; _rtZCSV -> bl4x1v1bkh = ssGetT ( rtS )
- rtP . Step_Time ; _rtZCSV -> daddiwwtso = ssGetT ( rtS ) - rtP . Step1_Time
; } void MdlTerminate ( void ) { { if ( rtDW . l0qhxzdoan . AQHandles ) {
sdiTerminateStreaming ( & rtDW . l0qhxzdoan . AQHandles ) ; } } { if ( rtDW .
jmxv5akigp . AQHandles ) { sdiTerminateStreaming ( & rtDW . jmxv5akigp .
AQHandles ) ; } } { if ( rtDW . dgtxfbydlw . AQHandles ) {
sdiTerminateStreaming ( & rtDW . dgtxfbydlw . AQHandles ) ; } } { if ( rtDW .
preosqjjml . AQHandles ) { sdiTerminateStreaming ( & rtDW . preosqjjml .
AQHandles ) ; } } { if ( rtDW . om1g1jtddz . AQHandles ) {
sdiTerminateStreaming ( & rtDW . om1g1jtddz . AQHandles ) ; } } { if ( rtDW .
kg0cpsbr4n . AQHandles ) { sdiTerminateStreaming ( & rtDW . kg0cpsbr4n .
AQHandles ) ; } } { if ( rtDW . gnkcthcwfm . AQHandles ) {
sdiTerminateStreaming ( & rtDW . gnkcthcwfm . AQHandles ) ; } } { if ( rtDW .
otdgdsww4h . AQHandles ) { sdiTerminateStreaming ( & rtDW . otdgdsww4h .
AQHandles ) ; } } } static void mr_drone_5DOF_observer_cacheDataAsMxArray (
mxArray * destArray , mwIndex i , int j , const void * srcData , size_t
numBytes ) ; static void mr_drone_5DOF_observer_cacheDataAsMxArray ( mxArray
* destArray , mwIndex i , int j , const void * srcData , size_t numBytes ) {
mxArray * newArray = mxCreateUninitNumericMatrix ( ( size_t ) 1 , numBytes ,
mxUINT8_CLASS , mxREAL ) ; memcpy ( ( uint8_T * ) mxGetData ( newArray ) , (
const uint8_T * ) srcData , numBytes ) ; mxSetFieldByNumber ( destArray , i ,
j , newArray ) ; } static void mr_drone_5DOF_observer_restoreDataFromMxArray
( void * destData , const mxArray * srcArray , mwIndex i , int j , size_t
numBytes ) ; static void mr_drone_5DOF_observer_restoreDataFromMxArray ( void
* destData , const mxArray * srcArray , mwIndex i , int j , size_t numBytes )
{ memcpy ( ( uint8_T * ) destData , ( const uint8_T * ) mxGetData (
mxGetFieldByNumber ( srcArray , i , j ) ) , numBytes ) ; } static void
mr_drone_5DOF_observer_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex
i , int j , uint_T bitVal ) ; static void
mr_drone_5DOF_observer_cacheBitFieldToMxArray ( mxArray * destArray , mwIndex
i , int j , uint_T bitVal ) { mxSetFieldByNumber ( destArray , i , j ,
mxCreateDoubleScalar ( ( real_T ) bitVal ) ) ; } static uint_T
mr_drone_5DOF_observer_extractBitFieldFromMxArray ( const mxArray * srcArray
, mwIndex i , int j , uint_T numBits ) ; static uint_T
mr_drone_5DOF_observer_extractBitFieldFromMxArray ( const mxArray * srcArray
, mwIndex i , int j , uint_T numBits ) { const uint_T varVal = ( uint_T )
mxGetScalar ( mxGetFieldByNumber ( srcArray , i , j ) ) ; return varVal & ( (
1u << numBits ) - 1u ) ; } static void
mr_drone_5DOF_observer_cacheDataToMxArrayWithOffset ( mxArray * destArray ,
mwIndex i , int j , mwIndex offset , const void * srcData , size_t numBytes )
; static void mr_drone_5DOF_observer_cacheDataToMxArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , const void * srcData ,
size_t numBytes ) { uint8_T * varData = ( uint8_T * ) mxGetData (
mxGetFieldByNumber ( destArray , i , j ) ) ; memcpy ( ( uint8_T * ) & varData
[ offset * numBytes ] , ( const uint8_T * ) srcData , numBytes ) ; } static
void mr_drone_5DOF_observer_restoreDataFromMxArrayWithOffset ( void *
destData , const mxArray * srcArray , mwIndex i , int j , mwIndex offset ,
size_t numBytes ) ; static void
mr_drone_5DOF_observer_restoreDataFromMxArrayWithOffset ( void * destData ,
const mxArray * srcArray , mwIndex i , int j , mwIndex offset , size_t
numBytes ) { const uint8_T * varData = ( const uint8_T * ) mxGetData (
mxGetFieldByNumber ( srcArray , i , j ) ) ; memcpy ( ( uint8_T * ) destData ,
( const uint8_T * ) & varData [ offset * numBytes ] , numBytes ) ; } static
void mr_drone_5DOF_observer_cacheBitFieldToCellArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal ) ; static
void mr_drone_5DOF_observer_cacheBitFieldToCellArrayWithOffset ( mxArray *
destArray , mwIndex i , int j , mwIndex offset , uint_T fieldVal ) {
mxSetCell ( mxGetFieldByNumber ( destArray , i , j ) , offset ,
mxCreateDoubleScalar ( ( real_T ) fieldVal ) ) ; } static uint_T
mr_drone_5DOF_observer_extractBitFieldFromCellArrayWithOffset ( const mxArray
* srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) ; static
uint_T mr_drone_5DOF_observer_extractBitFieldFromCellArrayWithOffset ( const
mxArray * srcArray , mwIndex i , int j , mwIndex offset , uint_T numBits ) {
const uint_T fieldVal = ( uint_T ) mxGetScalar ( mxGetCell (
mxGetFieldByNumber ( srcArray , i , j ) , offset ) ) ; return fieldVal & ( (
1u << numBits ) - 1u ) ; } mxArray * mr_drone_5DOF_observer_GetDWork ( ) {
static const char_T * ssDWFieldNames [ 3 ] = { "rtB" , "rtDW" ,
"NULL_PrevZCX" , } ; mxArray * ssDW = mxCreateStructMatrix ( 1 , 1 , 3 ,
ssDWFieldNames ) ; mr_drone_5DOF_observer_cacheDataAsMxArray ( ssDW , 0 , 0 ,
( const void * ) & ( rtB ) , sizeof ( rtB ) ) ; { static const char_T *
rtdwDataFieldNames [ 10 ] = { "rtDW.jkne1n5eru" , "rtDW.ou0nyrelia" ,
"rtDW.myhykfgvxr" , "rtDW.mapbnxxsya" , "rtDW.crw5jrqanh" , "rtDW.dubitlkqqo"
, "rtDW.e2f2zvadqk" , "rtDW.lvor0bqhcv" , "rtDW.lw2jdaqrz3" ,
"rtDW.a2bz2wiwxl" , } ; mxArray * rtdwData = mxCreateStructMatrix ( 1 , 1 ,
10 , rtdwDataFieldNames ) ; mr_drone_5DOF_observer_cacheDataAsMxArray (
rtdwData , 0 , 0 , ( const void * ) & ( rtDW . jkne1n5eru ) , sizeof ( rtDW .
jkne1n5eru ) ) ; mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 1
, ( const void * ) & ( rtDW . ou0nyrelia ) , sizeof ( rtDW . ou0nyrelia ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 2 , ( const void *
) & ( rtDW . myhykfgvxr ) , sizeof ( rtDW . myhykfgvxr ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 3 , ( const void *
) & ( rtDW . mapbnxxsya ) , sizeof ( rtDW . mapbnxxsya ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 4 , ( const void *
) & ( rtDW . crw5jrqanh ) , sizeof ( rtDW . crw5jrqanh ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 5 , ( const void *
) & ( rtDW . dubitlkqqo ) , sizeof ( rtDW . dubitlkqqo ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 6 , ( const void *
) & ( rtDW . e2f2zvadqk ) , sizeof ( rtDW . e2f2zvadqk ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 7 , ( const void *
) & ( rtDW . lvor0bqhcv ) , sizeof ( rtDW . lvor0bqhcv ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 8 , ( const void *
) & ( rtDW . lw2jdaqrz3 ) , sizeof ( rtDW . lw2jdaqrz3 ) ) ;
mr_drone_5DOF_observer_cacheDataAsMxArray ( rtdwData , 0 , 9 , ( const void *
) & ( rtDW . a2bz2wiwxl ) , sizeof ( rtDW . a2bz2wiwxl ) ) ;
mxSetFieldByNumber ( ssDW , 0 , 1 , rtdwData ) ; } return ssDW ; } void
mr_drone_5DOF_observer_SetDWork ( const mxArray * ssDW ) { ( void ) ssDW ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtB ) , ssDW ,
0 , 0 , sizeof ( rtB ) ) ; { const mxArray * rtdwData = mxGetFieldByNumber (
ssDW , 0 , 1 ) ; mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) &
( rtDW . jkne1n5eru ) , rtdwData , 0 , 0 , sizeof ( rtDW . jkne1n5eru ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
ou0nyrelia ) , rtdwData , 0 , 1 , sizeof ( rtDW . ou0nyrelia ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
myhykfgvxr ) , rtdwData , 0 , 2 , sizeof ( rtDW . myhykfgvxr ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
mapbnxxsya ) , rtdwData , 0 , 3 , sizeof ( rtDW . mapbnxxsya ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
crw5jrqanh ) , rtdwData , 0 , 4 , sizeof ( rtDW . crw5jrqanh ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
dubitlkqqo ) , rtdwData , 0 , 5 , sizeof ( rtDW . dubitlkqqo ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
e2f2zvadqk ) , rtdwData , 0 , 6 , sizeof ( rtDW . e2f2zvadqk ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
lvor0bqhcv ) , rtdwData , 0 , 7 , sizeof ( rtDW . lvor0bqhcv ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
lw2jdaqrz3 ) , rtdwData , 0 , 8 , sizeof ( rtDW . lw2jdaqrz3 ) ) ;
mr_drone_5DOF_observer_restoreDataFromMxArray ( ( void * ) & ( rtDW .
a2bz2wiwxl ) , rtdwData , 0 , 9 , sizeof ( rtDW . a2bz2wiwxl ) ) ; } }
mxArray * mr_drone_5DOF_observer_GetSimStateDisallowedBlocks ( ) { mxArray *
data = mxCreateCellMatrix ( 3 , 3 ) ; mwIndex subs [ 2 ] , offset ; { static
const char_T * blockType [ 3 ] = { "Scope" , "Scope" , "Scope" , } ; static
const char_T * blockPath [ 3 ] = { "drone_5DOF_observer/Scope" ,
"drone_5DOF_observer/Scope1" , "drone_5DOF_observer/Scope2" , } ; static
const int reason [ 3 ] = { 0 , 0 , 0 , } ; for ( subs [ 0 ] = 0 ; subs [ 0 ]
< 3 ; ++ ( subs [ 0 ] ) ) { subs [ 1 ] = 0 ; offset = mxCalcSingleSubscript (
data , 2 , subs ) ; mxSetCell ( data , offset , mxCreateString ( blockType [
subs [ 0 ] ] ) ) ; subs [ 1 ] = 1 ; offset = mxCalcSingleSubscript ( data , 2
, subs ) ; mxSetCell ( data , offset , mxCreateString ( blockPath [ subs [ 0
] ] ) ) ; subs [ 1 ] = 2 ; offset = mxCalcSingleSubscript ( data , 2 , subs )
; mxSetCell ( data , offset , mxCreateDoubleScalar ( ( real_T ) reason [ subs
[ 0 ] ] ) ) ; } } return data ; } void MdlInitializeSizes ( void ) {
ssSetNumContStates ( rtS , 27 ) ; ssSetNumPeriodicContStates ( rtS , 0 ) ;
ssSetNumY ( rtS , 0 ) ; ssSetNumU ( rtS , 0 ) ; ssSetDirectFeedThrough ( rtS
, 0 ) ; ssSetNumSampleTimes ( rtS , 3 ) ; ssSetNumBlocks ( rtS , 55 ) ;
ssSetNumBlockIO ( rtS , 13 ) ; ssSetNumBlockParams ( rtS , 397 ) ; } void
MdlInitializeSampleTimes ( void ) { ssSetSampleTime ( rtS , 0 , 0.0 ) ;
ssSetSampleTime ( rtS , 1 , 0.0 ) ; ssSetSampleTime ( rtS , 2 , 0.01 ) ;
ssSetOffsetTime ( rtS , 0 , 0.0 ) ; ssSetOffsetTime ( rtS , 1 , 1.0 ) ;
ssSetOffsetTime ( rtS , 2 , 0.0 ) ; } void raccel_set_checksum ( ) {
ssSetChecksumVal ( rtS , 0 , 3091924031U ) ; ssSetChecksumVal ( rtS , 1 ,
3399062413U ) ; ssSetChecksumVal ( rtS , 2 , 1667400322U ) ; ssSetChecksumVal
( rtS , 3 , 798752003U ) ; }
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
drone_5DOF_observer_InitializeDataMapInfo ( ) ; ssSetIsRapidAcceleratorActive
( rtS , true ) ; ssSetRootSS ( rtS , rtS ) ; ssSetVersion ( rtS ,
SIMSTRUCT_VERSION_LEVEL2 ) ; ssSetModelName ( rtS , "drone_5DOF_observer" ) ;
ssSetPath ( rtS , "drone_5DOF_observer" ) ; ssSetTStart ( rtS , 0.0 ) ;
ssSetTFinal ( rtS , 10.0 ) ; { static RTWLogInfo rt_DataLoggingInfo ;
rt_DataLoggingInfo . loggingInterval = ( NULL ) ; ssSetRTWLogInfo ( rtS , &
rt_DataLoggingInfo ) ; } { { static int_T rt_LoggedStateWidths [ ] = { 10 ,
10 , 7 } ; static int_T rt_LoggedStateNumDimensions [ ] = { 1 , 1 , 1 } ;
static int_T rt_LoggedStateDimensions [ ] = { 10 , 10 , 7 } ; static
boolean_T rt_LoggedStateIsVarDims [ ] = { 0 , 0 , 0 } ; static BuiltInDTypeId
rt_LoggedStateDataTypeIds [ ] = { SS_DOUBLE , SS_DOUBLE , SS_DOUBLE } ;
static int_T rt_LoggedStateComplexSignals [ ] = { 0 , 0 , 0 } ; static
RTWPreprocessingFcnPtr rt_LoggingStatePreprocessingFcnPtrs [ ] = { ( NULL ) ,
( NULL ) , ( NULL ) } ; static const char_T * rt_LoggedStateLabels [ ] = {
"CSTATE" , "CSTATE" , "CSTATE" } ; static const char_T *
rt_LoggedStateBlockNames [ ] = { "drone_5DOF_observer/Integrator4" ,
"drone_5DOF_observer/Integrator2" , "drone_5DOF_observer/Integrator1" } ;
static const char_T * rt_LoggedStateNames [ ] = { "" , "" , "" } ; static
boolean_T rt_LoggedStateCrossMdlRef [ ] = { 0 , 0 , 0 } ; static
RTWLogDataTypeConvert rt_RTWLogDataTypeConvert [ ] = { { 0 , SS_DOUBLE ,
SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0
, 0 , 1.0 , 0 , 0.0 } , { 0 , SS_DOUBLE , SS_DOUBLE , 0 , 0 , 0 , 1.0 , 0 ,
0.0 } } ; static int_T rt_LoggedStateIdxList [ ] = { 0 , 1 , 2 } ; static
RTWLogSignalInfo rt_LoggedStateSignalInfo = { 3 , rt_LoggedStateWidths ,
rt_LoggedStateNumDimensions , rt_LoggedStateDimensions ,
rt_LoggedStateIsVarDims , ( NULL ) , ( NULL ) , rt_LoggedStateDataTypeIds ,
rt_LoggedStateComplexSignals , ( NULL ) , rt_LoggingStatePreprocessingFcnPtrs
, { rt_LoggedStateLabels } , ( NULL ) , ( NULL ) , ( NULL ) , {
rt_LoggedStateBlockNames } , { rt_LoggedStateNames } ,
rt_LoggedStateCrossMdlRef , rt_RTWLogDataTypeConvert , rt_LoggedStateIdxList
} ; static void * rt_LoggedStateSignalPtrs [ 3 ] ; rtliSetLogXSignalPtrs (
ssGetRTWLogInfo ( rtS ) , ( LogSignalPtrsType ) rt_LoggedStateSignalPtrs ) ;
rtliSetLogXSignalInfo ( ssGetRTWLogInfo ( rtS ) , & rt_LoggedStateSignalInfo
) ; rt_LoggedStateSignalPtrs [ 0 ] = ( void * ) & rtX . eent0niwjp [ 0 ] ;
rt_LoggedStateSignalPtrs [ 1 ] = ( void * ) & rtX . d3azydidno [ 0 ] ;
rt_LoggedStateSignalPtrs [ 2 ] = ( void * ) & rtX . f0bfxnhi2t [ 0 ] ; }
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
static ssSolverInfo slvrInfo ; static boolean_T contStatesDisabled [ 27 ] ;
static real_T absTol [ 27 ] = { 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 ,
1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 , 1.0E-6 } ; static uint8_T
absTolControl [ 27 ] = { 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U
, 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U , 0U ,
0U } ; static real_T contStateJacPerturbBoundMinVec [ 27 ] ; static real_T
contStateJacPerturbBoundMaxVec [ 27 ] ; static uint8_T zcAttributes [ 2 ] = {
( ZC_EVENT_ALL_UP ) , ( ZC_EVENT_ALL_UP ) } ; static ssNonContDerivSigInfo
nonContDerivSigInfo [ 1 ] = { { 10 * sizeof ( real_T ) , ( char * ) ( & rtB .
ccu4dpkpbc [ 0 ] ) , ( NULL ) } } ; { int i ; for ( i = 0 ; i < 27 ; ++ i ) {
contStateJacPerturbBoundMinVec [ i ] = 0 ; contStateJacPerturbBoundMaxVec [ i
] = rtGetInf ( ) ; } } ssSetSolverRelTol ( rtS , 0.001 ) ; ssSetStepSize (
rtS , 0.0 ) ; ssSetMinStepSize ( rtS , 0.0 ) ; ssSetMaxNumMinSteps ( rtS , -
1 ) ; ssSetMinStepViolatedError ( rtS , 0 ) ; ssSetMaxStepSize ( rtS , 0.001
) ; ssSetSolverMaxOrder ( rtS , - 1 ) ; ssSetSolverRefineFactor ( rtS , 1 ) ;
ssSetOutputTimes ( rtS , ( NULL ) ) ; ssSetNumOutputTimes ( rtS , 0 ) ;
ssSetOutputTimesOnly ( rtS , 0 ) ; ssSetOutputTimesIndex ( rtS , 0 ) ;
ssSetZCCacheNeedsReset ( rtS , 0 ) ; ssSetDerivCacheNeedsReset ( rtS , 0 ) ;
ssSetNumNonContDerivSigInfos ( rtS , 1 ) ; ssSetNonContDerivSigInfos ( rtS ,
nonContDerivSigInfo ) ; ssSetSolverInfo ( rtS , & slvrInfo ) ;
ssSetSolverName ( rtS , "ode45" ) ; ssSetVariableStepSolver ( rtS , 1 ) ;
ssSetSolverConsistencyChecking ( rtS , 0 ) ; ssSetSolverAdaptiveZcDetection (
rtS , 0 ) ; ssSetSolverRobustResetMethod ( rtS , 0 ) ; ssSetAbsTolVector (
rtS , absTol ) ; ssSetAbsTolControlVector ( rtS , absTolControl ) ;
ssSetSolverAbsTol_Obsolete ( rtS , absTol ) ;
ssSetSolverAbsTolControl_Obsolete ( rtS , absTolControl ) ;
ssSetJacobianPerturbationBoundsMinVec ( rtS , contStateJacPerturbBoundMinVec
) ; ssSetJacobianPerturbationBoundsMaxVec ( rtS ,
contStateJacPerturbBoundMaxVec ) ; ssSetSolverStateProjection ( rtS , 0 ) ;
ssSetSolverMassMatrixType ( rtS , ( ssMatrixType ) 0 ) ;
ssSetSolverMassMatrixNzMax ( rtS , 0 ) ; ssSetModelOutputs ( rtS , MdlOutputs
) ; ssSetModelUpdate ( rtS , MdlUpdate ) ; ssSetModelDerivatives ( rtS ,
MdlDerivatives ) ; ssSetSolverZcSignalAttrib ( rtS , zcAttributes ) ;
ssSetSolverNumZcSignals ( rtS , 2 ) ; ssSetModelZeroCrossings ( rtS ,
MdlZeroCrossings ) ; ssSetSolverConsecutiveZCsStepRelTol ( rtS ,
2.8421709430404007E-13 ) ; ssSetSolverMaxConsecutiveZCs ( rtS , 1000 ) ;
ssSetSolverConsecutiveZCsError ( rtS , 2 ) ; ssSetSolverMaskedZcDiagnostic (
rtS , 1 ) ; ssSetSolverIgnoredZcDiagnostic ( rtS , 1 ) ;
ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ;
ssSetSolverShapePreserveControl ( rtS , 2 ) ; ssSetTNextTid ( rtS , INT_MIN )
; ssSetTNext ( rtS , rtMinusInf ) ; ssSetSolverNeedsReset ( rtS ) ;
ssSetNumNonsampledZCs ( rtS , 2 ) ; ssSetContStateDisabled ( rtS ,
contStatesDisabled ) ; ssSetSolverMaxConsecutiveMinStep ( rtS , 1 ) ; }
ssSetChecksumVal ( rtS , 0 , 3091924031U ) ; ssSetChecksumVal ( rtS , 1 ,
3399062413U ) ; ssSetChecksumVal ( rtS , 2 , 1667400322U ) ; ssSetChecksumVal
( rtS , 3 , 798752003U ) ; { static const sysRanDType rtAlwaysEnabled =
SUBSYS_RAN_BC_ENABLE ; static RTWExtModeInfo rt_ExtModeInfo ; static const
sysRanDType * systemRan [ 3 ] ; gblRTWExtModeInfo = & rt_ExtModeInfo ;
ssSetRTWExtModeInfo ( rtS , & rt_ExtModeInfo ) ;
rteiSetSubSystemActiveVectorAddresses ( & rt_ExtModeInfo , systemRan ) ;
systemRan [ 0 ] = & rtAlwaysEnabled ; systemRan [ 1 ] = & rtAlwaysEnabled ;
systemRan [ 2 ] = & rtAlwaysEnabled ; rteiSetModelMappingInfoPtr (
ssGetRTWExtModeInfo ( rtS ) , & ssGetModelMappingInfo ( rtS ) ) ;
rteiSetChecksumsPtr ( ssGetRTWExtModeInfo ( rtS ) , ssGetChecksums ( rtS ) )
; rteiSetTPtr ( ssGetRTWExtModeInfo ( rtS ) , ssGetTPtr ( rtS ) ) ; }
slsaDisallowedBlocksForSimTargetOP ( rtS ,
mr_drone_5DOF_observer_GetSimStateDisallowedBlocks ) ;
slsaGetWorkFcnForSimTargetOP ( rtS , mr_drone_5DOF_observer_GetDWork ) ;
slsaSetWorkFcnForSimTargetOP ( rtS , mr_drone_5DOF_observer_SetDWork ) ;
rt_RapidReadMatFileAndUpdateParams ( rtS ) ; if ( ssGetErrorStatus ( rtS ) )
{ return rtS ; } return rtS ; }
#if defined(_MSC_VER)
#pragma optimize( "", on )
#endif
void MdlOutputsParameterSampleTime ( int_T tid ) { MdlOutputsTID3 ( tid ) ; }
