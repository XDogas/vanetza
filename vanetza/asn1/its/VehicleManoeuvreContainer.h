
#ifndef	_VehicleManoeuvreContainer_H_
#define	_VehicleManoeuvreContainer_H_


#include "asn_application.h"

/* Including external dependencies */
#include "McmStartPoint.h"
#include "McmTrajectory.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct LowFrequencyContainer;
struct SpecialVehicleContainer;

/* VehicleManoeuvreContainer */
typedef struct VehicleManoeuvreContainer {
	McmStartPoint_t    currentPoint;
	McmTrajectory_t    mcmTrajectories;
	struct McmAutomationState    *automationState;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VehicleManoeuvreContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VehicleManoeuvreContainer;
extern asn_SEQUENCE_specifics_t asn_SPC_VehicleManoeuvreContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_VehicleManoeuvreContainer_1[3];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "McmAutomationState.h"

#endif	/* _VehicleManoeuvreContainer_H_ */
#include "asn_internal.h"
