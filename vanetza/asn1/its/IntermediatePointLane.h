
#ifndef	_IntermediatePointLane_H_
#define	_IntermediatePointLane_H_


#include "asn_application.h"

/* Including external dependencies */
#include "Lane.h"
#include "Reason.h"
#include "TimeOfPos.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IntermediatePointLane */
typedef struct IntermediatePointLane {
    Lane_t  lane;
    Reason_t reason;
    TimeOfPos_t timeOfPos;

	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IntermediatePointLane_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IntermediatePointLane;
extern asn_SEQUENCE_specifics_t asn_SPC_IntermediatePointLane_specs_1;
extern asn_TYPE_member_t asn_MBR_IntermediatePointLane_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _IntermediatePointLane_H_ */
#include "asn_internal.h"
