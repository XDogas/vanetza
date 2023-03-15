
#ifndef	_MCM_H_
#define	_MCM_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ItsPduHeader.h"
#include "ManueverCoordination.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MCM */
typedef struct MCM {
	ItsPduHeader_t	        header;
	ManueverCoordination_t	mcm;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MCM_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_MCM;

#ifdef __cplusplus
}
#endif

#endif	/* _MCM_H_ */
#include "asn_internal.h"
