/*
 * Console and exit for the bare-metal FPU tests: a 16550 UART with
 * 32-bit register stride, and qemu's dsk6455 exit port.
 *
 * UART_BASE and EXIT_PORT can be overridden with -D; without an exit
 * port (real hardware) the program spins after printing its result.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <stdarg.h>

#ifndef UART_BASE
#define UART_BASE 0x02b80000
#endif

#define UART ((volatile unsigned int *)UART_BASE)
#define UART_THR 0
#define UART_LSR 5
#define LSR_THRE 0x20

extern char __bss_start[], __bss_end[];
int main(void);

int putchar(int c)
{
    while (!(UART[UART_LSR] & LSR_THRE)) {
    }
    UART[UART_THR] = c;
    return c;
}

static void exit_(int code)
{
#ifdef EXIT_PORT
    *(volatile unsigned int *)EXIT_PORT = code;
#endif
    for (;;) {
    }
}

static void pnum(unsigned long v)
{
    char b[12];
    int i = 0;

    do {
        b[i++] = '0' + v % 10;
        v /= 10;
    } while (v);
    while (i) {
        putchar(b[--i]);
    }
}

/* Just what main.c needs: %s and %d */
int printf(const char *f, ...)
{
    va_list ap;

    va_start(ap, f);
    for (; *f; f++) {
        if (*f != '%') {
            putchar(*f);
            continue;
        }
        switch (*++f) {
        case 's': {
            const char *s = va_arg(ap, const char *);
            while (*s) {
                putchar(*s++);
            }
            break;
        }
        case 'd': {
            int v = va_arg(ap, int);
            if (v < 0) {
                putchar('-');
                v = -v;
            }
            pnum(v);
            break;
        }
        default:
            putchar(*f);
        }
    }
    va_end(ap);
    return 0;
}

void __start_c(void)
{
    for (char *p = __bss_start; p < __bss_end; p++) {
        *p = 0;
    }
    exit_(main());
}

/* referenced by the unwind tables gcc emits; never called */
void __c6xabi_unwind_cpp_pr0(void) { exit_(134); }
void __c6xabi_unwind_cpp_pr1(void) { exit_(134); }
void __c6xabi_unwind_cpp_pr2(void) { exit_(134); }
void __c6xabi_unwind_cpp_pr3(void) { exit_(134); }
void __c6xabi_unwind_cpp_pr4(void) { exit_(134); }
