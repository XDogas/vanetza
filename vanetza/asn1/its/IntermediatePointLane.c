
#include "IntermediatePointLane.h"

asn_TYPE_member_t asn_MBR_IntermediatePointLane_1[] = {
    { ATF_NOFLAGS, 0, offsetof(struct IntermediatePointLane, lane),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_Lane,
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
		"lane"
		},
    { ATF_NOFLAGS, 0, offsetof(struct IntermediatePointLane, reason),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_Reason,
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
		"reason"
		},
    { ATF_NOFLAGS, 0, offsetof(struct IntermediatePointLane, timeOfPos),
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
static const ber_tlv_tag_t asn_DEF_IntermediatePointLane_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_IntermediatePointLane_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* lane */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* reason */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* timeOfPos */
};
asn_SEQUENCE_specifics_t asn_SPC_IntermediatePointLane_specs_1 = {
	sizeof(struct IntermediatePointLane),
	offsetof(struct IntermediatePointLane, _asn_ctx),
	asn_MAP_IntermediatePointLane_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_IntermediatePointLane = {
	"IntermediatePointLane",
	"IntermediatePointLane",
	&asn_OP_SEQUENCE,
	asn_DEF_IntermediatePointLane_tags_1,
	sizeof(asn_DEF_IntermediatePointLane_tags_1)
		/sizeof(asn_DEF_IntermediatePointLane_tags_1[0]), /* 1 */
	asn_DEF_IntermediatePointLane_tags_1,	/* Same as above */
	sizeof(asn_DEF_IntermediatePointLane_tags_1)
		/sizeof(asn_DEF_IntermediatePointLane_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_IntermediatePointLane_1,
	3,	/* Elements count */
	&asn_SPC_IntermediatePointLane_specs_1	/* Additional specs */
};

