/*
 *  Meta register units header
 *
 *  Copyright (c) 2010 Imagination Technologies
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#ifndef INC_METAG_UNITS_H_
#define INC_METAG_UNITS_H_

/* unit codes */
typedef enum {
    META_UNIT_CT,       /* 0x0 */
    META_UNIT_D0,
    META_UNIT_D1,
    META_UNIT_A0,
    META_UNIT_A1,       /* 0x4 */
    META_UNIT_PC,
    META_UNIT_RA,
    META_UNIT_TR,
    META_UNIT_TT,       /* 0x8 */
    META_UNIT_FX,
    META_UNIT_MAX,
    META_UNIT_INVALID = -1,
} MetaUnit;

static inline MetaUnit meta_unit_partner(MetaUnit unit)
{
    switch (unit) {
    case META_UNIT_D0:  return META_UNIT_D1;
    case META_UNIT_D1:  return META_UNIT_D0;
    case META_UNIT_A0:  return META_UNIT_A1;
    case META_UNIT_A1:  return META_UNIT_A0;
    case META_UNIT_FX:  return META_UNIT_FX;
    case META_UNIT_RA:  return META_UNIT_RA;
    default:            return META_UNIT_INVALID;
    }
}

#endif /* INC_METAG_UNITS_H_ */
