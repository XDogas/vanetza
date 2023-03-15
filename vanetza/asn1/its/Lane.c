
#include "Lane.h"

asn_TYPE_member_t asn_MBR_Lane_1[] = {
    { ATF_NOFLAGS, 0, offsetof(struct Lane, lanePosition),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_LanePosition,
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
		"lanePosition"
		},
    { ATF_NOFLAGS, 0, offsetof(struct Lane, laneCount),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_LaneCount,
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
		"laneCount"
		},
};
static const ber_tlv_tag_t asn_DEF_Lane_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_Lane_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* lanePosition */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* laneCount */
};
asn_SEQUENCE_specifics_t asn_SPC_Lane_specs_1 = {
	sizeof(struct Lane),
	offsetof(struct Lane, _asn_ctx),
	asn_MAP_Lane_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_Lane = {
	"Lane",
	"Lane",
	&asn_OP_SEQUENCE,
	asn_DEF_Lane_tags_1,
	sizeof(asn_DEF_Lane_tags_1)
		/sizeof(asn_DEF_Lane_tags_1[0]), /* 1 */
	asn_DEF_Lane_tags_1,	/* Same as above */
	sizeof(asn_DEF_Lane_tags_1)
		/sizeof(asn_DEF_Lane_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_Lane_1,
	2,	/* Elements count */
	&asn_SPC_Lane_specs_1	/* Additional specs */
};

