/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation, or (at your option) any
 * later version. See the COPYING file in the top-level directory.
 */

#ifndef V850_TARGET_FCNTL_H
#define V850_TARGET_FCNTL_H

/* include/asm-v850/fcntl.h */
#define TARGET_O_DIRECTORY      040000 /* must be a directory */
#define TARGET_O_NOFOLLOW      0100000 /* don't follow links */
#define TARGET_O_DIRECT        0200000 /* direct disk access hint */
#define TARGET_O_LARGEFILE     0400000

#include "../generic/fcntl.h"
#endif
