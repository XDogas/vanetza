
#ifndef	_ManueverCoordination_H_
#define	_ManueverCoordination_H_


#include "asn_application.h"

/* Including external dependencies */
#include "GenerationDeltaTime.h"
#include "McmContainer.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ManueverCoordination */
typedef struct ManueverCoordination {
	GenerationDeltaTime_t	generationDeltaTime;
	McmContainer_t	mcmContainer;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ManueverCoordination_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ManueverCoordination;
extern asn_SEQUENCE_specifics_t asn_SPC_ManueverCoordination_specs_1;
extern asn_TYPE_member_t asn_MBR_ManueverCoordination_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _ManueverCoordination_H_ */
#include "asn_internal.h"
