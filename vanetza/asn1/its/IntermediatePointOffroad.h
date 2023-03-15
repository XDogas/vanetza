
#ifndef	_IntermediatePointOffroad_H_
#define	_IntermediatePointOffroad_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ReferencePosition.h"
#include "Heading.h"
#include "TimeOfPos.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IntermediatePointOffroad */
typedef struct IntermediatePointOffroad {
    ReferencePosition_t referencePosition;
    Heading_t referenceHeading;
    TimeOfPos_t timeOfPos;

	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} IntermediatePointOffroad_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_IntermediatePointOffroad;
extern asn_SEQUENCE_specifics_t asn_SPC_IntermediatePointOffroad_specs_1;
extern asn_TYPE_member_t asn_MBR_IntermediatePointOffroad_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _IntermediatePointOffroad_H_ */
#include "asn_internal.h"
