/*
 * Minimal bare-metal start for the C6000 FPU tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

	.text
	.global _start
_start:
	mvkl	.s2	__stack_top, b15
	mvkh	.s2	__stack_top, b15
	mvkl	.s2	__c6xabi_DSBT_BASE, b14
	mvkh	.s2	__c6xabi_DSBT_BASE, b14
	callp	.s2	__start_c, b3
1:	b	.s2	1b
	nop	5
