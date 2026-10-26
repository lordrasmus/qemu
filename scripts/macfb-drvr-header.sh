#!/bin/sh
# Turn the assembled macfb Mac OS video driver into a C header
# usage: macfb-drvr-header.sh macfb-drvr.o macfb-drvr.bin > macfb-drvr.h
# SPDX-License-Identifier: GPL-2.0-or-later

NM=${NM:-m68k-linux-gnu-nm}

params=$($NM "$1" | awk '$3 == "params" { print $1 }')
[ -n "$params" ] || { echo "no params symbol in $1" >&2; exit 1; }

cat <<HDR
/* Generated from macfb-drvr.S by scripts/macfb-drvr-header.sh, do not edit */

#define MACFB_DRVR_PARAMS 0x$params

static const uint8_t macfb_drvr[] = {
HDR
od -An -v -tx1 "$2" | sed -e 's/ *\([0-9a-f][0-9a-f]\)/ 0x\1,/g' -e 's/^/   /'
echo "};"
