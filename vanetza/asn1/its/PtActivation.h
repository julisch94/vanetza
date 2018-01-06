/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "CDD_TS102894-2/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_PtActivation_H_
#define	_PtActivation_H_


#include "asn_application.h"

/* Including external dependencies */
#include "PtActivationType.h"
#include "PtActivationData.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PtActivation */
typedef struct PtActivation {
	PtActivationType_t	 ptActivationType;
	PtActivationData_t	 ptActivationData;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PtActivation_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PtActivation;
extern asn_SEQUENCE_specifics_t asn_SPC_PtActivation_specs_1;
extern asn_TYPE_member_t asn_MBR_PtActivation_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _PtActivation_H_ */
#include "asn_internal.h"