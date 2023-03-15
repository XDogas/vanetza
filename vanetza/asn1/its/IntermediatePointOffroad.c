
#include "IntermediatePointOffroad.h"

asn_TYPE_member_t asn_MBR_IntermediatePointOffroad_1[] = {
    { ATF_NOFLAGS, 0, offsetof(struct IntermediatePointOffroad, referencePosition),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ReferencePosition,
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
		"referencePosition"
		},
    { ATF_NOFLAGS, 0, offsetof(struct IntermediatePointOffroad, referenceHeading),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Heading,
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
		"referenceHeading"
		},
    { ATF_NOFLAGS, 0, offsetof(struct IntermediatePointOffroad, timeOfPos),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_TimeOfPos,
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
		"timeOfPos"
		},
};
static const ber_tlv_tag_t asn_DEF_IntermediatePointOffroad_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_IntermediatePointOffroad_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* referencePosition */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* referenceHeading */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* timeOfPos */
};
asn_SEQUENCE_specifics_t asn_SPC_IntermediatePointOffroad_specs_1 = {
	sizeof(struct IntermediatePointOffroad),
	offsetof(struct IntermediatePointOffroad, _asn_ctx),
	asn_MAP_IntermediatePointOffroad_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_IntermediatePointOffroad = {
	"IntermediatePointOffroad",
	"IntermediatePointOffroad",
	&asn_OP_SEQUENCE,
	asn_DEF_IntermediatePointOffroad_tags_1,
	sizeof(asn_DEF_IntermediatePointOffroad_tags_1)
		/sizeof(asn_DEF_IntermediatePointOffroad_tags_1[0]), /* 1 */
	asn_DEF_IntermediatePointOffroad_tags_1,	/* Same as above */
	sizeof(asn_DEF_IntermediatePointOffroad_tags_1)
		/sizeof(asn_DEF_IntermediatePointOffroad_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_IntermediatePointOffroad_1,
	3,	/* Elements count */
	&asn_SPC_IntermediatePointOffroad_specs_1	/* Additional specs */
};

