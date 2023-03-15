
#include "VehicleManoeuvreContainer.h"

asn_TYPE_member_t asn_MBR_VehicleManoeuvreContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleManoeuvreContainer, currentPoint),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_McmStartPoint,
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
		"currentPoint"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VehicleManoeuvreContainer, mcmTrajectories),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_McmTrajectory,
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
		"mcmTrajectories"
		},
	{ ATF_POINTER, 1, offsetof(struct VehicleManoeuvreContainer, automationState),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_McmAutomationState,
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
		"automationState"
		},
};
static const int asn_MAP_VehicleManoeuvreContainer_oms_1[] = { 2 };
static const ber_tlv_tag_t asn_DEF_VehicleManoeuvreContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_VehicleManoeuvreContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* currentPoint */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* mcmTrajectories */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* automationState */
};
asn_SEQUENCE_specifics_t asn_SPC_VehicleManoeuvreContainer_specs_1 = {
	sizeof(struct VehicleManoeuvreContainer),
	offsetof(struct VehicleManoeuvreContainer, _asn_ctx),
	asn_MAP_VehicleManoeuvreContainer_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_VehicleManoeuvreContainer_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_VehicleManoeuvreContainer = {
	"VehicleManoeuvreContainer",
	"VehicleManoeuvreContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_VehicleManoeuvreContainer_tags_1,
	sizeof(asn_DEF_VehicleManoeuvreContainer_tags_1)
		/sizeof(asn_DEF_VehicleManoeuvreContainer_tags_1[0]), /* 1 */
	asn_DEF_VehicleManoeuvreContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_VehicleManoeuvreContainer_tags_1)
		/sizeof(asn_DEF_VehicleManoeuvreContainer_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_VehicleManoeuvreContainer_1,
	3,	/* Elements count */
	&asn_SPC_VehicleManoeuvreContainer_specs_1	/* Additional specs */
};

