/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* Copyright (C) 2017 Arm Ltd. */
#ifndef _UAPI_LINUX_ARM_SDEI_H
#define _UAPI_LINUX_ARM_SDEI_H

#define SDEI_1_0_FN_BASE			0xC4000020
#define SDEI_1_0_MASK				0xFFFFFFE0
#define SDEI_1_0_FN(n)				(SDEI_1_0_FN_BASE + (n))

#define SDEI_1_0_FN_SDEI_VERSION			SDEI_1_0_FN(0x00)
#define SDEI_1_0_FN_SDEI_EVENT_REGISTER			SDEI_1_0_FN(0x01)
#define SDEI_1_0_FN_SDEI_EVENT_ENABLE			SDEI_1_0_FN(0x02)
#define SDEI_1_0_FN_SDEI_EVENT_DISABLE			SDEI_1_0_FN(0x03)
#define SDEI_1_0_FN_SDEI_EVENT_CONTEXT			SDEI_1_0_FN(0x04)
#define SDEI_1_0_FN_SDEI_EVENT_COMPLETE			SDEI_1_0_FN(0x05)
#define SDEI_1_0_FN_SDEI_EVENT_COMPLETE_AND_RESUME	SDEI_1_0_FN(0x06)
#define SDEI_1_0_FN_SDEI_EVENT_UNREGISTER		SDEI_1_0_FN(0x07)
#define SDEI_1_0_FN_SDEI_EVENT_STATUS			SDEI_1_0_FN(0x08)
#define SDEI_1_0_FN_SDEI_EVENT_GET_INFO			SDEI_1_0_FN(0x09)
#define SDEI_1_0_FN_SDEI_EVENT_ROUTING_SET		SDEI_1_0_FN(0x0A)
#define SDEI_1_0_FN_SDEI_PE_MASK			SDEI_1_0_FN(0x0B)
#define SDEI_1_0_FN_SDEI_PE_UNMASK			SDEI_1_0_FN(0x0C)
#define SDEI_1_0_FN_SDEI_INTERRUPT_BIND			SDEI_1_0_FN(0x0D)
#define SDEI_1_0_FN_SDEI_INTERRUPT_RELEASE		SDEI_1_0_FN(0x0E)
#define SDEI_1_0_FN_SDEI_PRIVATE_RESET			SDEI_1_0_FN(0x11)
#define SDEI_1_0_FN_SDEI_SHARED_RESET			SDEI_1_0_FN(0x12)

#define SDEI_1_0_FN_SDEI_EVENT_SET_INFO			SDEI_1_0_FN(0x1F)

#define SDEI_VERSION_MAJOR_SHIFT			48
#define SDEI_VERSION_MAJOR_MASK				0x7fff
#define SDEI_VERSION_MINOR_SHIFT			32
#define SDEI_VERSION_MINOR_MASK				0xffff
#define SDEI_VERSION_VENDOR_SHIFT			0
#define SDEI_VERSION_VENDOR_MASK			0xffffffff

#define SDEI_VERSION_MAJOR(x)	(x>>SDEI_VERSION_MAJOR_SHIFT & SDEI_VERSION_MAJOR_MASK)
#define SDEI_VERSION_MINOR(x)	(x>>SDEI_VERSION_MINOR_SHIFT & SDEI_VERSION_MINOR_MASK)
#define SDEI_VERSION_VENDOR(x)	(x>>SDEI_VERSION_VENDOR_SHIFT & SDEI_VERSION_VENDOR_MASK)

/* SDEI return values */
#define SDEI_SUCCESS		0
#define SDEI_NOT_SUPPORTED	-1
#define SDEI_INVALID_PARAMETERS	-2
#define SDEI_DENIED		-3
#define SDEI_PENDING		-5
#define SDEI_OUT_OF_RESOURCE	-10

/* EVENT_REGISTER flags */
#define SDEI_EVENT_REGISTER_RM_ANY	0
#define SDEI_EVENT_REGISTER_RM_PE	1

/* EVENT_STATUS return value bits */
#define SDEI_EVENT_STATUS_RUNNING	2
#define SDEI_EVENT_STATUS_ENABLED	1
#define SDEI_EVENT_STATUS_REGISTERED	0

/* EVENT_COMPLETE status values */
#define SDEI_EV_HANDLED	0
#define SDEI_EV_FAILED	1

/* GET_INFO values */
#define SDEI_EVENT_INFO_EV_TYPE			0
#define SDEI_EVENT_INFO_EV_SIGNALED		1
#define SDEI_EVENT_INFO_EV_PRIORITY		2
#define SDEI_EVENT_INFO_EV_ROUTING_MODE		3
#define SDEI_EVENT_INFO_EV_ROUTING_AFF		4

/* SET values */
#define SDEI_EVENT_SET_IDM_MEMORY		1
#define SDEI_EVENT_SET_IDM_RESET		2
#define SDEI_EVENT_SET_TEE_MEMORY		3
#define SDEI_EVENT_SET_TEE_ENABLE		4
#define SDEI_EVENT_SET_TZC400_ENABLE	(0x10)
#define SDEI_EVENT_SET_TZC400_MEMORY	(0x11)
#define SDEI_EVENT_SET_OS_MEM_SIZE		(0x12)
#define SDEI_EVENT_SET_TFA_TRACE_MEMORY	(0x13)

#define SDEI_EVENT_SET_REBOOT_REASON    (0x21)
#define SDEI_EVENT_EXCEPTION_DEBUG		(0xff)

/* and their results */
#define SDEI_EVENT_TYPE_PRIVATE			0
#define SDEI_EVENT_TYPE_SHARED			1
#define SDEI_EVENT_PRIORITY_NORMAL		0
#define SDEI_EVENT_PRIORITY_CRITICAL		1

#endif /* _UAPI_LINUX_ARM_SDEI_H */
