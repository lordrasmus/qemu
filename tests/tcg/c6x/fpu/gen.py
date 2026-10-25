#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""
Generate tests for the C674x floating-point instructions.

Expected values come from the special-case rules in SPRUFE8B (C674x
Instruction Set Reference, July 2010), round to nearest only.  The model
below implements those rules on exact rationals; it does not use any
host floating point for results.
"""
import sys
from fractions import Fraction as F

SP = dict(bits=32, mant=23, ebits=8, bias=127)
DP = dict(bits=64, mant=52, ebits=11, bias=1023)

NAN_OUT = {32: 0x7fffffff, 64: 0x7fffffffffffffff}


def fields(fmt, v):
    s = v >> (fmt['bits'] - 1)
    e = (v >> fmt['mant']) & ((1 << fmt['ebits']) - 1)
    f = v & ((1 << fmt['mant']) - 1)
    return s, e, f


def classify(fmt, v):
    s, e, f = fields(fmt, v)
    emax = (1 << fmt['ebits']) - 1
    if e == emax:
        if f == 0:
            return 'inf', s
        return ('qnan' if f >> (fmt['mant'] - 1) else 'snan'), s
    if e == 0:
        return ('zero' if f == 0 else 'den'), s
    return 'norm', s


def value(fmt, v):
    """Exact value of a normal number (denormals read as signed zero)."""
    s, e, f = fields(fmt, v)
    if e == 0:
        return F(0)
    m = F((1 << fmt['mant']) | f, 1 << fmt['mant'])
    x = m * (F(2) ** (e - fmt['bias']))
    return -x if s else x


def signed_zero(fmt, s):
    return s << (fmt['bits'] - 1)


def signed_inf(fmt, s):
    emax = (1 << fmt['ebits']) - 1
    return (s << (fmt['bits'] - 1)) | (emax << fmt['mant'])


def signed_nan_out(fmt, s):
    return NAN_OUT[fmt['bits']] | (s << (fmt['bits'] - 1))


def round_ne(fmt, x, zero_sign=0):
    """Round exact x to fmt, nearest even; tiny results become signed 0
    (C67x: underflow gives 0 in round-to-nearest), overflow gives inf."""
    if x == 0:
        return signed_zero(fmt, zero_sign)
    s = 1 if x < 0 else 0
    a = -x if s else x
    # find exponent with 1 <= a / 2^e < 2
    e = a.numerator.bit_length() - a.denominator.bit_length()
    if F(2) ** e > a:
        e -= 1
    if F(2) ** (e + 1) <= a:
        e += 1
    scaled = a / (F(2) ** e) * (1 << fmt['mant'])
    q, r = divmod(scaled.numerator, scaled.denominator)
    half = F(scaled.denominator, 2)
    rem = F(r)
    if rem > half or (rem == half and q & 1):
        q += 1
    if q == 1 << (fmt['mant'] + 1):
        q >>= 1
        e += 1
    emin = 1 - fmt['bias']
    emax = (1 << fmt['ebits']) - 2 - fmt['bias']
    if e > emax:
        return signed_inf(fmt, s)
    if e < emin:
        return signed_zero(fmt, s)
    be = e + fmt['bias']
    r = (s << (fmt['bits'] - 1)) | (be << fmt['mant']) | (q & ((1 << fmt['mant']) - 1))
    if a < F(2) ** emin:
        # tiny before rounding, SFPN after: SPRUFE8B does not say whether
        # underflow is detected before or after rounding
        return ('either', r, signed_zero(fmt, s))
    return r


# ---- the instruction models ---------------------------------------------

def m_add(fmt, a, b, sub=False):
    ca, sa = classify(fmt, a)
    cb, sb = classify(fmt, b)
    if ca in ('qnan', 'snan') or cb in ('qnan', 'snan'):
        return NAN_OUT[fmt['bits']]
    if sub:
        sb ^= 1
    if ca == 'inf' and cb == 'inf':
        return signed_inf(fmt, sa) if sa == sb else NAN_OUT[fmt['bits']]
    if ca == 'inf':
        return signed_inf(fmt, sa)
    if cb == 'inf':
        return signed_inf(fmt, sb)
    za = ca in ('zero', 'den')
    zb = cb in ('zero', 'den')
    if za and zb:
        return signed_zero(fmt, sa if sa == sb else 0)
    va = value(fmt, a)
    vb = value(fmt, b)
    if sub:
        vb = -vb
    if za:
        return round_ne(fmt, vb)
    if zb:
        return round_ne(fmt, va)
    return round_ne(fmt, va + vb, 0)


def m_mpy(fa, fb, fr, a, b):
    ca, sa = classify(fa, a)
    cb, sb = classify(fb, b)
    s = sa ^ sb
    if ca in ('qnan', 'snan') or cb in ('qnan', 'snan'):
        return signed_nan_out(fr, s)
    za = ca in ('zero', 'den')
    zb = cb in ('zero', 'den')
    if (ca == 'inf' and zb) or (cb == 'inf' and za):
        return signed_nan_out(fr, s)
    if ca == 'inf' or cb == 'inf':
        return signed_inf(fr, s)
    if za or zb:
        return signed_zero(fr, s)
    return round_ne(fr, value(fa, a) * value(fb, b))


def m_cmp(fmt, a, b, op):
    ca, _ = classify(fmt, a)
    cb, _ = classify(fmt, b)
    if ca in ('qnan', 'snan') or cb in ('qnan', 'snan'):
        return 0
    def num(v, c):
        s = fields(fmt, v)[0]
        if c == 'inf':
            return F(10) ** 400 * (-1 if s else 1)
        return value(fmt, v)
    x, y = num(a, ca), num(b, cb)
    return int({'eq': x == y, 'gt': x > y, 'lt': x < y}[op])


def m_abs(fmt, a):
    c, s = classify(fmt, a)
    if c in ('qnan', 'snan'):
        return NAN_OUT[fmt['bits']]
    if c in ('zero', 'den'):
        return 0
    return a & ~(1 << (fmt['bits'] - 1))


def m_toint(fmt, a, trunc):
    c, s = classify(fmt, a)
    if c in ('qnan', 'snan'):
        return ('either', 0x7fffffff, 0x80000000)
    if c == 'inf':
        return 0x80000000 if s else 0x7fffffff
    if c in ('zero', 'den'):
        return 0
    x = value(fmt, a)
    if trunc:
        q = int(x)          # Fraction int() truncates toward zero
    else:
        fl = x.numerator // x.denominator
        r = x - fl
        q = fl + (1 if r > F(1, 2) or (r == F(1, 2) and fl & 1) else 0)
    if q > 0x7fffffff:
        return 0x7fffffff
    if q < -0x80000000:
        return 0x80000000
    return q & 0xffffffff


def m_fromint(fmt, a, unsigned):
    x = a if unsigned or a < 0x80000000 else a - (1 << 32)
    return round_ne(fmt, F(x))


def m_cvt(ff, ft, a):
    c, s = classify(ff, a)
    if c in ('qnan', 'snan'):
        return NAN_OUT[ft['bits']]
    if c == 'inf':
        return signed_inf(ft, s)
    if c in ('zero', 'den'):
        return signed_zero(ft, s)
    return round_ne(ft, value(ff, a))


def m_rcp(fmt, a, rsqr):
    c, s = classify(fmt, a)
    if c in ('qnan', 'snan'):
        return NAN_OUT[fmt['bits']]
    if rsqr and s and c in ('norm', 'inf'):
        return NAN_OUT[fmt['bits']]
    if c in ('zero', 'den'):
        return signed_inf(fmt, s)
    if c == 'inf':
        return signed_zero(fmt, s)
    x = value(fmt, a)
    if rsqr:
        # exact 1/sqrt only matters as a reference for the 2^-8 check
        import math
        ref = 1 / math.sqrt(float(x))
        return ('approx', round_ne(fmt, F(ref)))
    r = round_ne(fmt, 1 / x)
    if isinstance(r, tuple) or classify(fmt, r)[0] == 'zero':
        return r
    return ('approx', r)


# ---- operand sets --------------------------------------------------------

SPV = [0x00000000, 0x80000000, 0x00000001, 0x807fffff, 0x3f800000,
       0xbf800000, 0x3fc00000, 0xc0200000, 0x40400000, 0x3eaaaaab,
       0x7f7fffff, 0xff7fffff, 0x00800000, 0x80800000, 0x00800001,
       0x7f800000, 0xff800000, 0x7fc00000, 0xffc00001, 0x7f800001,
       0x7f000000, 0x01000000, 0x501502f9]

DPV = [0x0000000000000000, 0x8000000000000000, 0x0000000000000001,
       0x800fffffffffffff, 0x3ff0000000000000, 0xbff0000000000000,
       0x3ff8000000000000, 0xc004000000000000, 0x4008000000000000,
       0x3fd5555555555555, 0x7fefffffffffffff, 0xffefffffffffffff,
       0x0010000000000000, 0x8010000000000000, 0x0010000000000001,
       0x7ff0000000000000, 0xfff0000000000000, 0x7ff8000000000000,
       0xfff8000000000001, 0x7ff0000000000001, 0x7fe0000000000000,
       0x0020000000000000, 0x4202a05f20000000, 0x47efffffe0000000,
       0x3690000000000000, 0x380fffffffffffff]

INTV = [0, 1, 0xffffffff, 0x7fffffff, 0x80000000, 16777217, 16777219,
        0x01000003, 0xfefffffd, 12345, 0xffffcfc7, 0x40000001]

TOINT_SP = SPV + [0x40200000, 0x40600000, 0xc0200000, 0xc0600000,
                  0x4f000000, 0xcf000000, 0x4effffff, 0xcf000001,
                  0x3f000000, 0xbf000000, 0x3f400000]
TOINT_DP = DPV + [0x4004000000000000, 0x400c000000000000,
                  0xc004000000000000, 0x41dfffffffc00000,
                  0x41dfffffffe00000, 0xc1e0000000000000,
                  0xc1e0000000100000, 0x41e0000000000000,
                  0x3fe0000000000000, 0xbfe0000000000000]

# ---- instruction table: name, asm template, operand kinds, model ---------
# kinds: s = 32-bit register, d = register pair
INSNS = [
    ('addsp',   'addsp .l1 %1,%2,%0',   's', 'ss', lambda a, b: m_add(SP, a, b)),
    ('subsp',   'subsp .l1 %1,%2,%0',   's', 'ss', lambda a, b: m_add(SP, a, b, True)),
    ('adddp',   'adddp .l1 %1,%2,%0',   'd', 'dd', lambda a, b: m_add(DP, a, b)),
    ('subdp',   'subdp .l1 %1,%2,%0',   'd', 'dd', lambda a, b: m_add(DP, a, b, True)),
    ('mpysp',   'mpysp .m1 %1,%2,%0',   's', 'ss', lambda a, b: m_mpy(SP, SP, SP, a, b)),
    ('mpydp',   'mpydp .m1 %1,%2,%0',   'd', 'dd', lambda a, b: m_mpy(DP, DP, DP, a, b)),
    ('mpyspdp', 'mpyspdp .m1 %1,%2,%0', 'd', 'sd', lambda a, b: m_mpy(SP, DP, DP, a, b)),
    ('mpysp2dp', 'mpysp2dp .m1 %1,%2,%0', 'd', 'ss', lambda a, b: m_mpy(SP, SP, DP, a, b)),
    ('cmpeqsp', 'cmpeqsp .s1 %1,%2,%0', 's', 'ss', lambda a, b: m_cmp(SP, a, b, 'eq')),
    ('cmpgtsp', 'cmpgtsp .s1 %1,%2,%0', 's', 'ss', lambda a, b: m_cmp(SP, a, b, 'gt')),
    ('cmpltsp', 'cmpltsp .s1 %1,%2,%0', 's', 'ss', lambda a, b: m_cmp(SP, a, b, 'lt')),
    ('cmpeqdp', 'cmpeqdp .s1 %1,%2,%0', 's', 'dd', lambda a, b: m_cmp(DP, a, b, 'eq')),
    ('cmpgtdp', 'cmpgtdp .s1 %1,%2,%0', 's', 'dd', lambda a, b: m_cmp(DP, a, b, 'gt')),
    ('cmpltdp', 'cmpltdp .s1 %1,%2,%0', 's', 'dd', lambda a, b: m_cmp(DP, a, b, 'lt')),
    ('abssp',   'abssp .s1 %1,%0',      's', 's',  lambda a: m_abs(SP, a)),
    ('absdp',   'absdp .s1 %1,%0',      'd', 'd',  lambda a: m_abs(DP, a)),
    ('spint',   'spint .l1 %1,%0',      's', 's',  lambda a: m_toint(SP, a, False)),
    ('sptrunc', 'sptrunc .l1 %1,%0',    's', 's',  lambda a: m_toint(SP, a, True)),
    ('dpint',   'dpint .l1 %1,%0',      's', 'd',  lambda a: m_toint(DP, a, False)),
    ('dptrunc', 'dptrunc .l1 %1,%0',    's', 'd',  lambda a: m_toint(DP, a, True)),
    ('intsp',   'intsp .l1 %1,%0',      's', 'i',  lambda a: m_fromint(SP, a, False)),
    ('intspu',  'intspu .l1 %1,%0',     's', 'i',  lambda a: m_fromint(SP, a, True)),
    ('intdp',   'intdp .l1 %1,%0',      'd', 'i',  lambda a: m_fromint(DP, a, False)),
    ('intdpu',  'intdpu .l1 %1,%0',     'd', 'i',  lambda a: m_fromint(DP, a, True)),
    ('spdp',    'spdp .s1 %1,%0',       'd', 's',  lambda a: m_cvt(SP, DP, a)),
    ('dpsp',    'dpsp .l1 %1,%0',       's', 'd',  lambda a: m_cvt(DP, SP, a)),
    ('rcpsp',   'rcpsp .s1 %1,%0',      's', 's',  lambda a: m_rcp(SP, a, False)),
    ('rsqrsp',  'rsqrsp .s1 %1,%0',     's', 's',  lambda a: m_rcp(SP, a, True)),
    ('rcpdp',   'rcpdp .s1 %1,%0',      'd', 'd',  lambda a: m_rcp(DP, a, False)),
    ('rsqrdp',  'rsqrdp .s1 %1,%0',     'd', 'd',  lambda a: m_rcp(DP, a, True)),
]


def operands(kinds, name):
    if kinds == 'i':
        return [(a,) for a in INTV]
    if kinds == 's':
        return [(a,) for a in (TOINT_SP if name in ('spint', 'sptrunc') else SPV)]
    if kinds == 'd':
        return [(a,) for a in (TOINT_DP if name in ('dpint', 'dptrunc') else DPV)]
    sets = [SPV if k == 's' else DPV for k in kinds]
    return [(a, b) for a in sets[0] for b in sets[1]]


def main():
    out = sys.stdout
    out.write('/* generated by gen.py from SPRUFE8B special-case rules */\n')
    out.write('#include "fpu.h"\n\n')
    for i, (name, tmpl, rk, kinds, model) in enumerate(INSNS):
        ct = {'s': 'unsigned', 'd': 'unsigned long long', 'i': 'unsigned'}
        args = ', '.join(f'{ct[k]} a{j}' for j, k in enumerate(kinds))
        ins = ', '.join(f'"a"(a{j})' for j in range(len(kinds)))
        out.write(f'static unsigned long long x_{name}({args})\n{{\n')
        out.write(f'\t{ct[rk]} r;\n')
        out.write(f'\t__asm__ volatile("{tmpl}\\n\\tnop 9\\n\\tnop 9" : "=a"(r) : {ins});\n')
        out.write('\treturn r;\n}\n\n')
    for i, (name, tmpl, rk, kinds, model) in enumerate(INSNS):
        out.write(f'static const struct vec v_{name}[] = {{\n')
        for ops in operands(kinds, name):
            exp = model(*ops)
            kind, e2 = 'K_EXACT', 0
            if isinstance(exp, tuple):
                if exp[0] == 'either':
                    kind, exp, e2 = 'K_EITHER', exp[1], exp[2]
                else:
                    kind, exp = ('K_APPROX_SP' if rk == 's' else 'K_APPROX_DP'), exp[1]
            a = ops[0]
            b = ops[1] if len(ops) > 1 else 0
            out.write(f'\t{{ {a:#x}ull, {b:#x}ull, {exp:#x}ull, {e2:#x}ull, {kind} }},\n')
        out.write('};\n\n')
    out.write('int run_all(void)\n{\n\tint fails = 0;\n\tunsigned i;\n')
    for name, tmpl, rk, kinds, model in INSNS:
        call = ', '.join(['v[i].a', 'v[i].b'][:len(kinds)])
        out.write(f'\t{{ const struct vec *v = v_{name};\n')
        out.write(f'\t  for (i = 0; i < sizeof(v_{name}) / sizeof(*v); i++)\n')
        out.write(f'\t\tfails += check("{name}", &v[i], x_{name}({call}));\n\t}}\n')
    out.write('\treturn fails;\n}\n')


if __name__ == '__main__':
    main()
