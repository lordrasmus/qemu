/*
 * FR-V helper declarations
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

DEF_HELPER_FLAGS_2(raise, TCG_CALL_NO_WG, noreturn, env, i32)
DEF_HELPER_FLAGS_2(trap, TCG_CALL_NO_WG, noreturn, env, i32)
DEF_HELPER_3(sdiv, i32, env, i32, i32)
DEF_HELPER_3(udiv, i32, env, i32, i32)
DEF_HELPER_2(movsg, i32, env, i32)
DEF_HELPER_3(movgs, void, env, i32, i32)
DEF_HELPER_2(rett, i32, env, i32)
DEF_HELPER_FLAGS_3(crlogic, TCG_CALL_NO_RWG_SE, i32, i32, i32, i32)
DEF_HELPER_FLAGS_2(addss, TCG_CALL_NO_RWG_SE, i32, i32, i32)
DEF_HELPER_FLAGS_2(subss, TCG_CALL_NO_RWG_SE, i32, i32, i32)
DEF_HELPER_FLAGS_2(slass, TCG_CALL_NO_RWG_SE, i32, i32, i32)
DEF_HELPER_4(iacc_mul, void, env, i32, i32, i32)
DEF_HELPER_2(scutss, i32, env, i32)
