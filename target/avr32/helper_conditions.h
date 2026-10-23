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
#ifndef QEMU_AVR32_HELPER_CP_INST_H
#define QEMU_AVR32_HELPER_CP_INST_H
#include "qemu/osdep.h"
#include "cpu.h"
#include "tcg/tcg-op.h"

#define sflagC  SR_C
#define sflagZ  SR_Z
#define sflagN  SR_N
#define sflagV  SR_V
#define sflagQ  SR_Q
#define sflagL  SR_L
#define sflagT  SR_T
#define sflagR  SR_R
#define sflagGM SR_GM
#define sflagEM SR_EM

int checkCondition(int condition, TCGv returnReg, TCGv cpu_r[], TCGv cpu_sflags[]);

void set_v_flag_add(TCGv op1, TCGv op2, TCGv result, TCGv cpu_sflags[]);
void set_c_flag_add(TCGv op1, TCGv op2, TCGv result, TCGv cpu_sflags[]);

void set_v_flag_cp(TCGv op1, TCGv op2, TCGv result, TCGv cpu_sflags[]);
void set_c_flag_cp(TCGv op1, TCGv op2, TCGv result, TCGv cpu_sflags[]);

void set_flags_cpc(TCGv op1, TCGv op2, TCGv result, TCGv cpu_sflags[]);

void cpw_instruction(TCGv Rd, TCGv Rs, TCGv cpu_sflags[]);

#endif //QEMU_AVR32_HELPER_CP_INST_H
