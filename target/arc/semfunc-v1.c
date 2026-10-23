/*
 * QEMU ARC CPU -- semantic functions that exist only on ARCompact (ARCv1).
 *
 * The ARCv1 instruction set shares most of its encodings, and therefore its
 * semantics, with ARCv2 (see semfunc-v2.c).  What lives here are the few
 * instructions that ARCv2 dropped, so that the ARC700 decode tree has a
 * semantic function to map to.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "translate.h"
#include "target/arc/semfunc.h"

/*
 * EXTW: dest = zero_extend16(src).  Same shape as EXTB in semfunc-v2.c,
 * only the width differs.
 */
int
arc_gen_EXTW(DisasCtxt *ctx, TCGv dest, TCGv src)
{
    int ret = DISAS_NEXT;
    TCGv temp_3 = tcg_temp_new();
    TCGv cc_flag = tcg_temp_new();
    TCGv temp_1 = tcg_temp_new();
    TCGv temp_2 = tcg_temp_new();
    int f_flag;
    getCCFlag(temp_3);
    tcg_gen_mov_tl(cc_flag, temp_3);
    TCGLabel *done_1 = gen_new_label();
    tcg_gen_setcond_tl(TCG_COND_EQ, temp_1, cc_flag, arc_true);
    tcg_gen_xori_tl(temp_2, temp_1, 1);
    tcg_gen_andi_tl(temp_2, temp_2, 1);
    tcg_gen_brcond_tl(TCG_COND_EQ, temp_2, arc_true, done_1);
    tcg_gen_andi_tl(dest, src, 65535);
    f_flag = getFFlag ();
    if ((f_flag == true)) {
        setZFlag(dest);
        setNFlag(dest);
    }
    gen_set_label(done_1);

    return ret;
}

/*
 * SEXW: dest = sign_extend16(src).  Same shape as SEXB.
 */
int
arc_gen_SEXW(DisasCtxt *ctx, TCGv dest, TCGv src)
{
    int ret = DISAS_NEXT;
    TCGv temp_3 = tcg_temp_new();
    TCGv cc_flag = tcg_temp_new();
    TCGv temp_1 = tcg_temp_new();
    TCGv temp_2 = tcg_temp_new();
    TCGv temp_6 = tcg_temp_new();
    TCGv temp_5 = tcg_temp_new();
    TCGv temp_4 = tcg_temp_new();
    int f_flag;
    getCCFlag(temp_3);
    tcg_gen_mov_tl(cc_flag, temp_3);
    TCGLabel *done_1 = gen_new_label();
    tcg_gen_setcond_tl(TCG_COND_EQ, temp_1, cc_flag, arc_true);
    tcg_gen_xori_tl(temp_2, temp_1, 1);
    tcg_gen_andi_tl(temp_2, temp_2, 1);
    tcg_gen_brcond_tl(TCG_COND_EQ, temp_2, arc_true, done_1);
    tcg_gen_movi_tl(temp_6, 16);
    tcg_gen_shli_tl(temp_5, src, 16);
    arithmeticShiftRight(temp_4, temp_5, temp_6);
    tcg_gen_mov_tl(dest, temp_4);
    f_flag = getFFlag ();
    if ((f_flag == true)) {
        setZFlag(dest);
        setNFlag(dest);
    }
    gen_set_label(done_1);

    return ret;
}
