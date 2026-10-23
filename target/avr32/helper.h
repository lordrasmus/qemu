/*
 * QEMU AVR CPU
 *
 * Copyright (c) 2022-2023 Florian Göhler
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */
DEF_HELPER_2(raise_exception, noreturn, env, i32)
DEF_HELPER_2(scall, void, env, i32)
DEF_HELPER_1(rets, void, env)
DEF_HELPER_1(rete, void, env)
DEF_HELPER_1(retd, void, env)
DEF_HELPER_3(sleep, noreturn, env, i32, i32)
DEF_HELPER_2(mfsr, i32, env, i32)
DEF_HELPER_3(mtsr, void, env, i32, i32)
DEF_HELPER_2(mfdr, i32, env, i32)
DEF_HELPER_1(tlbw, void, env)
DEF_HELPER_1(tlbr, void, env)
DEF_HELPER_1(tlbs, void, env)
DEF_HELPER_4(macsathhw, void, env, i32, i32, i32)
