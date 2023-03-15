
#ifndef	_McmContainer_H_
#define	_McmContainer_H_


#include "asn_application.h"

/* Including external dependencies */
#include "VehicleManoeuvreContainer.h"
#include "ManoeuvreAdviceContainer.h"
#include "constr_CHOICE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum McmContainer_PR {
	McmContainer_PR_NOTHING,	/* No components present */
	McmContainer_PR_VehicleManoeuvreContainer,
	McmContainer_PR_ManoeuvreAdviceContainer
	/* Extensions may appear below */
	
} McmContainer_PR;

/* McmContainer */
typedef struct McmContainer {
	McmContainer_PR present;
	union McmContainer_u {
		VehicleManoeuvreContainer_t	 VehicleManoeuvreContainer;
		ManoeuvreAdviceContainer_t	 ManoeuvreAdviceContainer;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} McmContainer_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_McmContainer;
extern asn_CHOICE_specifics_t asn_SPC_McmContainer_specs_1;
extern asn_TYPE_member_t asn_MBR_McmContainer_1[2];
extern asn_per_constraints_t asn_PER_type_McmContainer_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _McmContainer_H_ */
#include "asn_internal.h"
