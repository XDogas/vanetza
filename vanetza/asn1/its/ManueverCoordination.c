
#include "ManueverCoordination.h"

asn_TYPE_member_t asn_MBR_ManueverCoordination_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ManueverCoordination, generationDeltaTime),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_GenerationDeltaTime,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"generationDeltaTime"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ManueverCoordination, mcmContainer),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_McmContainer,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"mcmContainer"
		},
};
static const ber_tlv_tag_t asn_DEF_ManueverCoordination_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ManueverCoordination_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* generationDeltaTime */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* mcmContainer */
};
asn_SEQUENCE_specifics_t asn_SPC_ManueverCoordination_specs_1 = {
	sizeof(struct ManueverCoordination),
	offsetof(struct ManueverCoordination, _asn_ctx),
	asn_MAP_ManueverCoordination_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ManueverCoordination = {
	"ManueverCoordination",
	"ManueverCoordination",
	&asn_OP_SEQUENCE,
	asn_DEF_ManueverCoordination_tags_1,
	sizeof(asn_DEF_ManueverCoordination_tags_1)
		/sizeof(asn_DEF_ManueverCoordination_tags_1[0]), /* 1 */
	asn_DEF_ManueverCoordination_tags_1,	/* Same as above */
	sizeof(asn_DEF_ManueverCoordination_tags_1)
		/sizeof(asn_DEF_ManueverCoordination_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_ManueverCoordination_1,
	2,	/* Elements count */
	&asn_SPC_ManueverCoordination_specs_1	/* Additional specs */
};

