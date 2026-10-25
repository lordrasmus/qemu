/*
 * META Core Registers.
 *
 * Copyright (C) 2011 Imagination Technologies Ltd.
 */

#ifndef meta_core_registers_h
# define meta_core_registers_h "core_registers.h"

#include "cpu.h"

void meta_reset_core_registers(MetaCore *core);
void meta_init_core_registers(MetaCore *core, unsigned int extirqs);

#endif
