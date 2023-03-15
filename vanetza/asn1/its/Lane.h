
#ifndef	_Lane_H_
#define	_Lane_H_


#include "asn_application.h"

/* Including external dependencies */
#include "LanePosition.h"
#include "LaneCount.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Lane */
typedef struct Lane {
    LanePosition_t lanePosition;
    LaneCount_t laneCount;

	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Lane_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Lane;
extern asn_SEQUENCE_specifics_t asn_SPC_Lane_specs_1;
extern asn_TYPE_member_t asn_MBR_Lane_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Lane_H_ */
#include "asn_internal.h"
