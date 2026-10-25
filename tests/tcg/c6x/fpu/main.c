/*
 * Check results of the generated FPU tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "fpu.h"

extern int printf(const char *, ...);
extern int putchar(int);

static void hex(unsigned long long v, int digits)
{
	while (digits--)
		putchar("0123456789abcdef"[(v >> (digits * 4)) & 15]);
}

/*
 * An approximation must have the expected sign and exponent and lie
 * within 2^-8 of the reference mantissa (SPRUFE8B: "mantissa error is
 * less than 2^-8").
 */
static int approx_ok(unsigned long long got, unsigned long long ref, int mant)
{
	unsigned long long top = got >> mant, rtop = ref >> mant;
	long long gm = (long long)(got & ((1ull << mant) - 1)) | (1ll << mant);
	long long rm = (long long)(ref & ((1ull << mant) - 1)) | (1ll << mant);
	long long d = gm - rm;

	if (top != rtop) {
		/* mantissa may cross into the next binade */
		if (top + 1 == rtop && (got & ((1ull << mant) - 1)) > ((1ull << mant) - (1ull << (mant - 8))))
			return 1;
		if (top == rtop + 1 && (got & ((1ull << mant) - 1)) < (1ull << (mant - 8)))
			return 1;
		return 0;
	}
	if (d < 0)
		d = -d;
	return d < (1ll << (mant - 8 + 1));
}

int check(const char *name, const struct vec *v, unsigned long long got)
{
	int ok;

	switch (v->kind) {
	case K_EITHER:
		ok = got == v->exp || got == v->exp2;
		break;
	case K_APPROX_SP:
		ok = approx_ok(got & 0xffffffffull, v->exp, 23);
		break;
	case K_APPROX_DP:
		ok = approx_ok(got, v->exp, 52);
		break;
	default:
		ok = got == v->exp;
	}
	if (ok)
		return 0;
	printf("FAIL %s a=", name);
	hex(v->a, 16);
	printf(" b=");
	hex(v->b, 16);
	printf(" got=");
	hex(got, 16);
	printf(" want=");
	hex(v->exp, 16);
	printf("\n");
	return 1;
}

int main(void)
{
	int f = run_all();

	printf("fails %d\n", f);
	return f ? 1 : 0;
}
