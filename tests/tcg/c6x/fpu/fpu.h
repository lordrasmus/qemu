/*
 * Shared declarations of the C6000 FPU tests
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

enum { K_EXACT, K_EITHER, K_APPROX_SP, K_APPROX_DP };

struct vec {
	unsigned long long a, b, exp, exp2;
	int kind;
};

int check(const char *name, const struct vec *v, unsigned long long got);
int run_all(void);
