/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "build.asn1/iso/ISO19321.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_TrainCharacteristics_H_
#define	_TrainCharacteristics_H_


#include "asn_application.h"

/* Including external dependencies */
#include "TractorCharacteristics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TrainCharacteristics */
typedef TractorCharacteristics_t	 TrainCharacteristics_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_TrainCharacteristics;
asn_struct_free_f TrainCharacteristics_free;
asn_struct_print_f TrainCharacteristics_print;
asn_constr_check_f TrainCharacteristics_constraint;
ber_type_decoder_f TrainCharacteristics_decode_ber;
der_type_encoder_f TrainCharacteristics_encode_der;
xer_type_decoder_f TrainCharacteristics_decode_xer;
xer_type_encoder_f TrainCharacteristics_encode_xer;
jer_type_encoder_f TrainCharacteristics_encode_jer;
oer_type_decoder_f TrainCharacteristics_decode_oer;
oer_type_encoder_f TrainCharacteristics_encode_oer;
per_type_decoder_f TrainCharacteristics_decode_uper;
per_type_encoder_f TrainCharacteristics_encode_uper;
per_type_decoder_f TrainCharacteristics_decode_aper;
per_type_encoder_f TrainCharacteristics_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _TrainCharacteristics_H_ */
#include "asn_internal.h"
