
#ifndef	_McmStartPoint_H_
#define	_McmStartPoint_H_


#include "asn_application.h"

/* Including external dependencies */
#include "IntermediatePointReference.h"
#include "IntermediatePointOffroad.h"
#include "constr_CHOICE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum McmStartPoint_PR {
	McmStartPoint_PR_NOTHING,	/* No components present */
	McmStartPoint_PR_IntermediatePointReference,
	McmStartPoint_PR_IntermediatePointOffroad
	/* Extensions may appear below */
	
} McmStartPoint_PR;

/* McmStartPoint */
typedef struct McmStartPoint {
	McmStartPoint_PR present;
	union McmStartPoint_u {
		IntermediatePointReference_t    IntermediatePointReference;
		IntermediatePointOffroad_t      IntermediatePointOffroad;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} McmStartPoint_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_McmStartPoint;
extern asn_CHOICE_specifics_t asn_SPC_McmStartPoint_specs_1;
extern asn_TYPE_member_t asn_MBR_McmStartPoint_1[2];
extern asn_per_constraints_t asn_PER_type_McmStartPoint_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _McmStartPoint_H_ */
#include "asn_internal.h"
