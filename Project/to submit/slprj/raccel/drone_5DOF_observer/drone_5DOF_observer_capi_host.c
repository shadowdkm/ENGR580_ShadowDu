#include "drone_5DOF_observer_capi_host.h"
static drone_5DOF_observer_host_DataMapInfo_T root;
static int initialized = 0;
__declspec( dllexport ) rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        drone_5DOF_observer_host_InitializeDataMapInfo(&(root), "drone_5DOF_observer");
    }
    return &root.mmi;
}

rtwCAPI_ModelMappingInfo *mexFunction(){return(getRootMappingInfo());}
