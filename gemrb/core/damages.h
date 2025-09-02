/* GemRB - Infinity Engine Emulator
 * Copyright (C) 2003 The GemRB Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *
 */

#ifndef DAMAGE_H
#define DAMAGE_H

namespace GemRB {

//damage types
#define DAMAGE_CRUSHING 0
#define DAMAGE_ACID     1 << 16
#define DAMAGE_COLD     2 << 16
#define DAMAGE_ELECTRICITY 4 << 16
#define DAMAGE_FIRE     8 << 16
#define DAMAGE_PIERCING 0x10 << 16
#define DAMAGE_POISON 0x20 << 16
#define DAMAGE_MAGIC 0x40 << 16
#define DAMAGE_MISSILE 0x80 << 16
#define DAMAGE_SLASHING 0x100
#define DAMAGE_MAGICFIRE 0x200
#define DAMAGE_PIERCINGMISSILE 0x200 //iwd2
#define DAMAGE_MAGICCOLD 0x400
#define DAMAGE_CRUSHINGMISSILE 0x400 //iwd2
#define DAMAGE_STUNNING 0x800
#define DAMAGE_SOULEATER 0x1000  //iwd2
#define DAMAGE_BLEEDING 0x2000   //iwd2
#define DAMAGE_DISEASE 0x4000    //iwd2

#define DAMAGE_CHUNKING 0x8000
#define DAMAGE_DISINTEGRATE 0x10000 // internal gemrb marker
//damage levels
#define DL_CRITICAL 0
#define DL_BLOOD  1
#define DL_FIRE   4
#define DL_ELECTRICITY  7
#define DL_COLD   10
#define DL_ACID   13
#define DL_DISINTEGRATE 16

#define WDAMAGE_NONE              0
#define WDAMAGE_PIERCING          1
#define WDAMAGE_CRUSHING          2
#define WDAMAGE_SLASHING          3
#define WDAMAGE_MISSILE           4
#define WDAMAGE_FIST              5
#define WDAMAGE_PIERCING_CRUSHING 6
#define WDAMAGE_PIERCING_SLASHING 7
#define WDAMAGE_CRUSHING_SLASHING 8

}

#endif
