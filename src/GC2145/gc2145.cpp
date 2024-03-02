/*
 * Copyright 2021 Arduino SA
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * GC2145 driver.
 */
#include "Wire.h"
#include "gc2145.h"

#define GC_MAX_WIN_W                    (1600)
#define GC_MAX_WIN_H                    (1200)

#define REG_AMODE1                      (0x17)
#define REG_AMODE1_DEF                  (0x14)
#define REG_AMODE1_SET_HMIRROR(r, x)    ((r&0xFE)|((x&1)<<0))
#define REG_AMODE1_SET_VMIRROR(r, x)    ((r&0xFD)|((x&1)<<1))

#define REG_OUTPUT_FMT                  (0x84)
#define REG_OUTPUT_FMT_RGB565           (0x06)
#define REG_OUTPUT_FMT_YCBYCR           (0x02)
#define REG_OUTPUT_FMT_BAYER            (0x17)
#define REG_OUTPUT_SET_FMT(r, x)        ((r&0xE0)|(x))

#define REG_SYNC_MODE                   (0x86)
#define REG_SYNC_MODE_DEF               (0x23)
#define REG_SYNC_MODE_COL_SWITCH        (0x10)
#define REG_SYNC_MODE_ROW_SWITCH        (0x20)


#define DEBUG_CAMERA

static const uint8_t default_regs[][2] = {
    {0xfe, 0xf0},
    {0xfe, 0xf0},
    {0xfe, 0xf0},
    {0xfc, 0x06},
    {0xf6, 0x00},
    {0xf7, 0x1d},
    {0xf8, 0x85},
    {0xfa, 0x00},
    {0xf9, 0xfe},
    {0xf2, 0x00},
    /////////////////////////////////////////////////
    //////////////////ISP reg//////////////////////
    ////////////////////////////////////////////////////
    {0xfe, 0x00},
    {0x03, 0x04},
    {0x04, 0xe2},

    {0x09, 0x00},   // row start
    {0x0a, 0x00},

    {0x0b, 0x00},   // col start
    {0x0c, 0x00},

    {0x0d, 0x04},   // Window height
    {0x0e, 0xc0},

    {0x0f, 0x06},   // Window width
    {0x10, 0x52},

    {0x99, 0x11},   // Subsample
    {0x9a, 0x0E},   // Subsample mode

    {0x12, 0x2e},   //
    {0x17, 0x14},   // Analog Mode 1 (vflip/mirror[1:0])
    {0x18, 0x22},   // Analog Mode 2
    {0x19, 0x0e},
    {0x1a, 0x01},
    {0x1b, 0x4b},
    {0x1c, 0x07},
    {0x1d, 0x10},
    {0x1e, 0x88},
    {0x1f, 0x78},
    {0x20, 0x03},
    {0x21, 0x40},
    {0x22, 0xa0},
    {0x24, 0x16},
    {0x25, 0x01},
    {0x26, 0x10},
    {0x2d, 0x60},
    {0x30, 0x01},
    {0x31, 0x90},
    {0x33, 0x06},
    {0x34, 0x01},
    {0x80, 0x7f},
    {0x81, 0x26},
    {0x82, 0xfa},
    {0x83, 0x00},
    {0x84, 0x06},   //RGB565
    {0x86, 0x23},
    {0x88, 0x03},
    {0x89, 0x03},
    {0x85, 0x08},
    {0x8a, 0x00},
    {0x8b, 0x00},
    {0xb0, 0x55},
    {0xc3, 0x00},
    {0xc4, 0x80},
    {0xc5, 0x90},
    {0xc6, 0x3b},
    {0xc7, 0x46},
    {0xec, 0x06},
    {0xed, 0x04},
    {0xee, 0x60},
    {0xef, 0x90},
    {0xb6, 0x01},

    {0x90, 0x01},   // Enable crop
    {0x91, 0x00},   // Y offset
    {0x92, 0x00},
    {0x93, 0x00},   // X offset
    {0x94, 0x00},
    {0x95, 0x02},   // Window height
    {0x96, 0x58},
    {0x97, 0x03},   // Window width
    {0x98, 0x20},
    {0x99, 0x22},   // Subsample
    {0x9a, 0x0E},   // Subsample mode

    {0x9b, 0x00},
    {0x9c, 0x00},
    {0x9d, 0x00},
    {0x9e, 0x00},
    {0x9f, 0x00},
    {0xa0, 0x00},
    {0xa1, 0x00},
    {0xa2, 0x00},
    /////////////////////////////////////////
    /////////// BLK ////////////////////////
    /////////////////////////////////////////
    {0xfe, 0x00},
    {0x40, 0x42},
    {0x41, 0x00},
    {0x43, 0x5b},
    {0x5e, 0x00},
    {0x5f, 0x00},
    {0x60, 0x00},
    {0x61, 0x00},
    {0x62, 0x00},
    {0x63, 0x00},
    {0x64, 0x00},
    {0x65, 0x00},
    {0x66, 0x20},
    {0x67, 0x20},
    {0x68, 0x20},
    {0x69, 0x20},
    {0x76, 0x00},
    {0x6a, 0x08},
    {0x6b, 0x08},
    {0x6c, 0x08},
    {0x6d, 0x08},
    {0x6e, 0x08},
    {0x6f, 0x08},
    {0x70, 0x08},
    {0x71, 0x08},
    {0x76, 0x00},
    {0x72, 0xf0},
    {0x7e, 0x3c},
    {0x7f, 0x00},
    {0xfe, 0x02},
    {0x48, 0x15},
    {0x49, 0x00},
    {0x4b, 0x0b},
    {0xfe, 0x00},
    ////////////////////////////////////////
    /////////// AEC ////////////////////////
    ////////////////////////////////////////
    {0xfe, 0x01},
    {0x01, 0x04},
    {0x02, 0xc0},
    {0x03, 0x04},
    {0x04, 0x90},
    {0x05, 0x30},
    {0x06, 0x90},
    {0x07, 0x30},
    {0x08, 0x80},
    {0x09, 0x00},
    {0x0a, 0x82},
    {0x0b, 0x11},
    {0x0c, 0x10},
    {0x11, 0x10},
    {0x13, 0x68}, //7b->68 bob
    {0x17, 0x00},
    {0x1c, 0x11},
    {0x1e, 0x61},
    {0x1f, 0x35},
    {0x20, 0x40},
    {0x22, 0x40},
    {0x23, 0x20},
    {0xfe, 0x02},
    {0x0f, 0x04},
    {0xfe, 0x01},
    {0x12, 0x30}, //35
    {0x15, 0xb0},
    {0x10, 0x31},
    {0x3e, 0x28},
    {0x3f, 0xb0},
    {0x40, 0x90},
    {0x41, 0x0f},
    /////////////////////////////
    //////// INTPEE /////////////
    /////////////////////////////
    {0xfe, 0x02},
    {0x90, 0x6c},
    {0x91, 0x03},
    {0x92, 0xcb},
    {0x94, 0x33},
    {0x95, 0x84},
    {0x97, 0x65}, // 54->65 bob
    {0xa2, 0x11},
    {0xfe, 0x00},
    /////////////////////////////
    //////// DNDD///////////////
    /////////////////////////////
    {0xfe, 0x02},
    {0x80, 0xc1},
    {0x81, 0x08},
    {0x82, 0x05}, //05
    {0x83, 0x08}, //08
    {0x84, 0x0a},
    {0x86, 0xf0},
    {0x87, 0x50},
    {0x88, 0x15},
    {0x89, 0xb0},
    {0x8a, 0x30},
    {0x8b, 0x10},
    /////////////////////////////////////////
    /////////// ASDE ////////////////////////
    /////////////////////////////////////////
    {0xfe, 0x01},
    {0x21, 0x04},
    {0xfe, 0x02},
    {0xa3, 0x50},
    {0xa4, 0x20},
    {0xa5, 0x40},
    {0xa6, 0x80},
    {0xab, 0x40},
    {0xae, 0x0c},
    {0xb3, 0x46},
    {0xb4, 0x64},
    {0xb6, 0x38},
    {0xb7, 0x01}, //01
    {0xb9, 0x2b}, //2b
    {0x3c, 0x04}, //04
    {0x3d, 0x15}, //15
    {0x4b, 0x06}, //06
    {0x4c, 0x20},
    {0xfe, 0x00},
    /////////////////////////////////////////
    /////////// GAMMA   ////////////////////////
    /////////////////////////////////////////
    ///////////////////gamma1////////////////////
    {0xfe, 0x02},
    {0x10, 0x09},
    {0x11, 0x0d},
    {0x12, 0x13},
    {0x13, 0x19},
    {0x14, 0x27},
    {0x15, 0x37},
    {0x16, 0x45},
    {0x17, 0x53},
    {0x18, 0x69},
    {0x19, 0x7d},
    {0x1a, 0x8f},
    {0x1b, 0x9d},
    {0x1c, 0xa9},
    {0x1d, 0xbd},
    {0x1e, 0xcd},
    {0x1f, 0xd9},
    {0x20, 0xe3},
    {0x21, 0xea},
    {0x22, 0xef},
    {0x23, 0xf5},
    {0x24, 0xf9},
    {0x25, 0xff},
    {0xfe, 0x00},
    {0xc6, 0x20},
    {0xc7, 0x2b},
    ///////////////////gamma2////////////////////
    {0xfe, 0x02},
    {0x26, 0x0f},
    {0x27, 0x14},
    {0x28, 0x19},
    {0x29, 0x1e},
    {0x2a, 0x27},
    {0x2b, 0x33},
    {0x2c, 0x3b},
    {0x2d, 0x45},
    {0x2e, 0x59},
    {0x2f, 0x69},
    {0x30, 0x7c},
    {0x31, 0x89},
    {0x32, 0x98},
    {0x33, 0xae},
    {0x34, 0xc0},
    {0x35, 0xcf},
    {0x36, 0xda},
    {0x37, 0xe2},
    {0x38, 0xe9},
    {0x39, 0xf3},
    {0x3a, 0xf9},
    {0x3b, 0xff},
    ///////////////////////////////////////////////
    ///////////YCP ///////////////////////
    ///////////////////////////////////////////////
    {0xfe, 0x02},
    {0xd1, 0x32}, //32->2d
    {0xd2, 0x32}, //32->2d bob
    {0xd3, 0x40},
    {0xd6, 0xf0},
    {0xd7, 0x10},
    {0xd8, 0xda},
    {0xdd, 0x14},
    {0xde, 0x86},
    {0xed, 0x80}, //80
    {0xee, 0x00}, //00
    {0xef, 0x3f},
    {0xd8, 0xd8},
    ///////////////////abs/////////////////
    {0xfe, 0x01},
    {0x9f, 0x40},
    /////////////////////////////////////////////
    //////////////////////// LSC ///////////////
    //////////////////////////////////////////
    {0xfe, 0x01},
    {0xc2, 0x14},
    {0xc3, 0x0d},
    {0xc4, 0x0c},
    {0xc8, 0x15},
    {0xc9, 0x0d},
    {0xca, 0x0a},
    {0xbc, 0x24},
    {0xbd, 0x10},
    {0xbe, 0x0b},
    {0xb6, 0x25},
    {0xb7, 0x16},
    {0xb8, 0x15},
    {0xc5, 0x00},
    {0xc6, 0x00},
    {0xc7, 0x00},
    {0xcb, 0x00},
    {0xcc, 0x00},
    {0xcd, 0x00},
    {0xbf, 0x07},
    {0xc0, 0x00},
    {0xc1, 0x00},
    {0xb9, 0x00},
    {0xba, 0x00},
    {0xbb, 0x00},
    {0xaa, 0x01},
    {0xab, 0x01},
    {0xac, 0x00},
    {0xad, 0x05},
    {0xae, 0x06},
    {0xaf, 0x0e},
    {0xb0, 0x0b},
    {0xb1, 0x07},
    {0xb2, 0x06},
    {0xb3, 0x17},
    {0xb4, 0x0e},
    {0xb5, 0x0e},
    {0xd0, 0x09},
    {0xd1, 0x00},
    {0xd2, 0x00},
    {0xd6, 0x08},
    {0xd7, 0x00},
    {0xd8, 0x00},
    {0xd9, 0x00},
    {0xda, 0x00},
    {0xdb, 0x00},
    {0xd3, 0x0a},
    {0xd4, 0x00},
    {0xd5, 0x00},
    {0xa4, 0x00},
    {0xa5, 0x00},
    {0xa6, 0x77},
    {0xa7, 0x77},
    {0xa8, 0x77},
    {0xa9, 0x77},
    {0xa1, 0x80},
    {0xa2, 0x80},

    {0xfe, 0x01},
    {0xdf, 0x0d},
    {0xdc, 0x25},
    {0xdd, 0x30},
    {0xe0, 0x77},
    {0xe1, 0x80},
    {0xe2, 0x77},
    {0xe3, 0x90},
    {0xe6, 0x90},
    {0xe7, 0xa0},
    {0xe8, 0x90},
    {0xe9, 0xa0},
    {0xfe, 0x00},
    ///////////////////////////////////////////////
    /////////// AWB////////////////////////
    ///////////////////////////////////////////////
    {0xfe, 0x01},
    {0x4f, 0x00},
    {0x4f, 0x00},
    {0x4b, 0x01},
    {0x4f, 0x00},

    {0x4c, 0x01}, // D75
    {0x4d, 0x71},
    {0x4e, 0x01},
    {0x4c, 0x01},
    {0x4d, 0x91},
    {0x4e, 0x01},
    {0x4c, 0x01},
    {0x4d, 0x70},
    {0x4e, 0x01},
    {0x4c, 0x01}, // D65
    {0x4d, 0x90},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xb0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x8f},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0x6f},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xaf},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xd0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xf0},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xcf},
    {0x4e, 0x02},
    {0x4c, 0x01},
    {0x4d, 0xef},
    {0x4e, 0x02},
    {0x4c, 0x01}, //D50
    {0x4d, 0x6e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8e},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xae},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xce},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8d},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xad},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcd},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8c},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xac},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcc},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xcb},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x4b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x6b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0x8b},
    {0x4e, 0x03},
    {0x4c, 0x01},
    {0x4d, 0xab},
    {0x4e, 0x03},
    {0x4c, 0x01}, //CWF
    {0x4d, 0x8a},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xaa},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xca},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xc9},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0x8a},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0x89},
    {0x4e, 0x04},
    {0x4c, 0x01},
    {0x4d, 0xa9},
    {0x4e, 0x04},
    {0x4c, 0x02}, //tl84
    {0x4d, 0x0b},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x0a},
    {0x4e, 0x05},
    {0x4c, 0x01},
    {0x4d, 0xeb},
    {0x4e, 0x05},
    {0x4c, 0x01},
    {0x4d, 0xea},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x09},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x29},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x2a},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x4a},
    {0x4e, 0x05},
    {0x4c, 0x02},
    {0x4d, 0x8a},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x49},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x89},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0xa9},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x48},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x68},
    {0x4e, 0x06},
    {0x4c, 0x02},
    {0x4d, 0x69},
    {0x4e, 0x06},
    {0x4c, 0x02}, //H
    {0x4d, 0xca},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc9},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe9},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x09},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc8},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe8},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xa7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xc7},
    {0x4e, 0x07},
    {0x4c, 0x02},
    {0x4d, 0xe7},
    {0x4e, 0x07},
    {0x4c, 0x03},
    {0x4d, 0x07},
    {0x4e, 0x07},

    {0x4f, 0x01},
    {0x50, 0x80},
    {0x51, 0xa8},
    {0x52, 0x47},
    {0x53, 0x38},
    {0x54, 0xc7},
    {0x56, 0x0e},
    {0x58, 0x08},
    {0x5b, 0x00},
    {0x5c, 0x74},
    {0x5d, 0x8b},
    {0x61, 0xdb},
    {0x62, 0xb8},
    {0x63, 0x86},
    {0x64, 0xc0},
    {0x65, 0x04},
    {0x67, 0xa8},
    {0x68, 0xb0},
    {0x69, 0x00},
    {0x6a, 0xa8},
    {0x6b, 0xb0},
    {0x6c, 0xaf},
    {0x6d, 0x8b},
    {0x6e, 0x50},
    {0x6f, 0x18},
    {0x73, 0xf0},
    {0x70, 0x0d},
    {0x71, 0x60},
    {0x72, 0x80},
    {0x74, 0x01},
    {0x75, 0x01},
    {0x7f, 0x0c},
    {0x76, 0x70},
    {0x77, 0x58},
    {0x78, 0xa0},
    {0x79, 0x5e},
    {0x7a, 0x54},
    {0x7b, 0x58},
    {0xfe, 0x00},
    //////////////////////////////////////////
    ///////////CC////////////////////////
    //////////////////////////////////////////
    {0xfe, 0x02},
    {0xc0, 0x01},
    {0xc1, 0x44},
    {0xc2, 0xfd},
    {0xc3, 0x04},
    {0xc4, 0xF0},
    {0xc5, 0x48},
    {0xc6, 0xfd},
    {0xc7, 0x46},
    {0xc8, 0xfd},
    {0xc9, 0x02},
    {0xca, 0xe0},
    {0xcb, 0x45},
    {0xcc, 0xec},
    {0xcd, 0x48},
    {0xce, 0xf0},
    {0xcf, 0xf0},
    {0xe3, 0x0c},
    {0xe4, 0x4b},
    {0xe5, 0xe0},
    //////////////////////////////////////////
    ///////////ABS ////////////////////
    //////////////////////////////////////////
    {0xfe, 0x01},
    {0x9f, 0x40},
    {0xfe, 0x00},

    //////////////////////////////////////
    ///////////  OUTPUT   ////////////////
    //////////////////////////////////////
    {0xfe, 0x00},
    {0xf2, 0x0f},

    ///////////////dark sun////////////////////
    {0xfe, 0x02},
    {0x40, 0xbf},
    {0x46, 0xcf},
    {0xfe, 0x00},

    //////////////frame rate control/////////
    {0xfe, 0x00},
    {0x05, 0x01},   // HBLANK
    {0x06, 0x1C},
    {0x07, 0x00},   // VBLANK
    {0x08, 0x32},
    {0x11, 0x00},   // SH Delay
    {0x12, 0x1D},
    {0x13, 0x00},   // St
    {0x14, 0x00},   // Et

    {0xfe, 0x01},
    {0x3c, 0x00},
    {0x3d, 0x04},
    {0xfe, 0x00},
    {0x00, 0x00},
};

GC2145::GC2145(WIRECLASS &i2c) : 
    _i2c(&i2c)
{
}

int GC2145::init()
{
    _i2c->begin();
    _i2c->setClock(100000);

    int ret = 0;

    // Write default regsiters
    for (int i = 0; default_regs[i][0]; i++) {
        ret |= regWrite(GC2145_I2C_ADDR, default_regs[i][0], default_regs[i][1]);
    }

    return ret;
}

int GC2145::setWindow(uint16_t reg, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    int ret = 0;

    // P0 regs
    ret |= regWrite(GC2145_I2C_ADDR, 0xFE, 0x00);

    // Y/row offset
    ret |= regWrite(GC2145_I2C_ADDR, reg++, y >> 8);
    ret |= regWrite(GC2145_I2C_ADDR, reg++, y & 0xff);

    // X/col offset
    ret |= regWrite(GC2145_I2C_ADDR, reg++, x >> 8);
    ret |= regWrite(GC2145_I2C_ADDR, reg++, x & 0xff);

    // Window height
    ret |= regWrite(GC2145_I2C_ADDR, reg++, h >> 8);
    ret |= regWrite(GC2145_I2C_ADDR, reg++, h & 0xff);

    // Window width
    ret |= regWrite(GC2145_I2C_ADDR, reg++, w >> 8);
    ret |= regWrite(GC2145_I2C_ADDR, reg++, w & 0xff);

    return ret;
}

int GC2145::reset()
{
    return 0;
}

int GC2145::setFrameRate(int32_t framerate)
{
    return 0;
}

int GC2145::setVerticalFlip(bool flip_enable)
{
    // The GC2145 doesn't return this value when reading the Analog mode 1 register
    // so we have to save it for setHorizontalMirror()
    vertical_flip_state = flip_enable;
    // Using the Analog mode 1 register (0x17)
    uint8_t old_value = regRead(GC2145_I2C_ADDR, 0x17);
    int retVal = regWrite(GC2145_I2C_ADDR, 0x17, (old_value & 0b11111100) | (flip_enable << 1) | horizontal_mirror_state);
    // Notice that the error codes from regWrite() are positive ones passed on from Wire, not -1
    return ((0 == retVal) ? 0 : -1);
}

int GC2145::setHorizontalMirror(bool mirror_enable)
{
    // The GC2145 doesn't return this value when reading the Analog mode 1 register
    // so we have to save it for setVerticalFlip()
    horizontal_mirror_state = mirror_enable;
    // Using the Analog mode 1 register (0x17)
    uint8_t old_value = regRead(GC2145_I2C_ADDR, 0x17);
    int retVal = regWrite(GC2145_I2C_ADDR, 0x17, (old_value & 0b11111100) | mirror_enable | (vertical_flip_state << 1));
    // Notice that the error codes from regWrite() are positive ones passed on from Wire, not -1
    return ((0 == retVal) ? 0 : -1);
}

int GC2145::setResolution(int32_t resolution)
{
    return setResolutionWithZoom(resolution, resolution, 0, 0);
}

int GC2145::setResolutionWithZoom(int32_t resolution, int32_t zoom_resolution, uint32_t zoom_x, uint32_t zoom_y)
{
    int ret = 0;

    uint16_t win_w;
    uint16_t win_h;

    uint16_t w = restab[resolution][0];
    uint16_t h = restab[resolution][1];

    switch (resolution) {
        case CAMERA_R160x120:
            win_w = w * 4;
            win_h = h * 4;
            break;
        case CAMERA_R320x240:
        case CAMERA_R320x320:
            win_w = w * 3;
            win_h = h * 3;
            break;
        case CAMERA_R640x480:
            win_w = w * 2;
            win_h = h * 2;
            break;
        case CAMERA_R800x600:
        case CAMERA_R1600x1200:
            // For frames bigger than subsample using full UXGA window.
            win_w = 1600;
            win_h = 1200;
            break;
        default:
            return -1;
    }

    uint16_t c_ratio = win_w / w;
    uint16_t r_ratio = win_h / h;

    uint16_t x = (((win_w / c_ratio) - w) / 2);
    uint16_t y = (((win_h / r_ratio) - h) / 2);

    uint16_t win_x = ((GC_MAX_WIN_W - win_w) / 2);
    uint16_t win_y = ((GC_MAX_WIN_H - win_h) / 2);

    // Set readout window first.
    ret |= setWindow(0x09, win_x, win_y, win_w + 16, win_h + 8);

    // Zoom mode active
    if (resolution != zoom_resolution)
    {
        // Can't zoom into a larger window than the original
        if (zoom_resolution > resolution)
        {
            return -1;
        }
        
        // The zoom resolution constant is outside of the allowed range
        if ((zoom_resolution < 0) || (zoom_resolution >= CAMERA_RMAX))
        {
            return -1;
        }

        uint32_t zoom_w = restab[zoom_resolution][0];
        uint32_t zoom_h = restab[zoom_resolution][1];

        // Check if the zoom window goes outside the frame on the x axis
        // Notice that this form prevents uint32_t wraparound, so don't change it
        if (zoom_x >= (w - zoom_w))
        {
            return -1;
        }
        // Check of the zoom window goes outside the frame on the y axis
        // Notice that this form prevents uint32_t wraparound, so don't change it
        if (zoom_y >= (h - zoom_h))
        {
            return -1;
        }

        // Set the cropping window parameters to the zoom window parameters
        x = zoom_x;
        y = zoom_y;
        w = zoom_w;
        h = zoom_h;
    }

    // Set cropping window next.
    ret |= setWindow(0x91, x, y, w, h);

    // Enable crop
    ret |= regWrite(GC2145_I2C_ADDR, 0x90, 0x01);

    // Set Sub-sampling ratio and mode
    ret |= regWrite(GC2145_I2C_ADDR, 0x99, ((r_ratio << 4) | c_ratio));
    ret |= regWrite(GC2145_I2C_ADDR, 0x9A, 0x0E);

    return ret;
}

int GC2145::setPixelFormat(int32_t pixformat)
{
    int ret = 0;
    uint8_t reg;

    // P0 regs
    ret |= regWrite(GC2145_I2C_ADDR, 0xFE, 0x00);

    // Read current output format reg
    reg = regRead(GC2145_I2C_ADDR, REG_OUTPUT_FMT);

    switch (pixformat) {
        case CAMERA_RGB565:
            ret |= regWrite(GC2145_I2C_ADDR,
                    REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_RGB565));
            break;
        case CAMERA_GRAYSCALE:
            // TODO: There's no support for extracting GS from YUV so we use Bayer for 1BPP for now.
            //ret |= regWrite(GC2145_I2C_ADDR,
            //        REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_YCBYCR));
            //break;
        case CAMERA_BAYER:
            // There's no BAYER support so it will just look off.
            // Make sure odd/even row are switched to work with our bayer conversion.
            ret |= regWrite(GC2145_I2C_ADDR,
                    REG_SYNC_MODE, REG_SYNC_MODE_DEF | REG_SYNC_MODE_ROW_SWITCH);
            ret |= regWrite(GC2145_I2C_ADDR,
                    REG_OUTPUT_FMT, REG_OUTPUT_SET_FMT(reg, REG_OUTPUT_FMT_BAYER));
            break;
        default:
            return -1;
    }

    return ret;
}
typedef struct {
  uint16_t reg;
  const __FlashStringHelper *reg_name;
} GC2145_TO_NAME_t;

static const GC2145_TO_NAME_t GC2145_reg_name_table[] PROGMEM {
    {0x00f0, F(" chip_ID[15:8]")},
    {0x00f1, F(" chip_ID[7:0]")},
    {0x00f2, F(" pad_vb_hiz_mode data_pad_io sync_pad_io")},
    {0x00f3, F(" I2C_open_en")},
    {0x00f6, F(" Up_dn Pwd_dn")},
    {0x00f7, F(" PLL_mode1")},
    {0x00f8, F(" PLL_mode2")},
    {0x00f9, F(" cm_mode")},
    {0x00fa, F(" clk_div_mode")},
    {0x00fb, F(" I2C_device_ID")},
    {0x00fc, F(" analog_pwc")},
    {0x00fd, F(" Scalar mode")},
    {0x00fe, F(" Reset related")},
    {0x0003, F("Exposure[12:8]")},
    {0x0004, F("Exposure[7:0]")},
    {0x0005, F("buf_CISCTL_capt_hb[11:8]")},
    {0x0006, F("buf_CISCTL_capt_hb[7:0]")},
    {0x0007, F("buf_CISCTL_capt_vb[12:8]")},
    {0x0008, F("buf_CISCTL_capt_vb[7:0]")},
    {0x0009, F("buf_CISCTL_capt_row_start[10:8]")},
    {0x000a, F("buf_CISCTL_capt_row_start[7:0]")},
    {0x000b, F("buf_CISCTL_capt_col_start[10:8 ]")},
    {0x000c, F("buf_CISCTL_capt_col_start[7:1]")},
    {0x000d, F("buf_CISCTL_capt_win_height[10:8]")},
    {0x000e, F("buf_CISCTL_capt_win_height[7:0]")},
    {0x000f, F("buf_CISCTL_capt_win_width[10:8]")},
    {0x0010, F("buf_CISCTL_capt_win_width[7:1]")},
    {0x0017, F("Analog mode1")},
    {0x0018, F("Analog mode2")},
    {0x0020, F("Analog mode3")},
    {0x0024, F("Driver mode")},
    {0x003f, F("dark_current_st able_th")},
    {0x0040, F("Blk_mode1")},
    {0x0042, F("BLK_limit_value")},
    {0x0043, F("BLK_fame_cnt_TH")},
    {0x005c, F("Exp_rate_darkc")},
    {0x005e, F("current_G1_offset_odd_ratio")},
    {0x005f, F("current_G1_offset_even_ratio")},
    {0x0060, F("current_R1_offset_odd_ratio")},
    {0x0061, F("current_R1_offset_even_ratio")},
    {0x0062, F("current_B1_offset_odd_ratio")},
    {0x0063, F("current_B1_offset_even_ratio")},
    {0x0064, F("current_G2_offset_odd_ratio")},
    {0x0065, F("current_G2_offset_even_ratio")},
    {0x0066, F("Dark_current_G1_ratio")},
    {0x0067, F("Dark_current_R_ratio")},
    {0x0068, F("Dark_current_B_ratio")},
    {0x0069, F("Dark_current_G2_ratio")},
    {0x006a, F("manual_G1_odd_offset")},
    {0x006b, F("manual_G1_even_offset")},
    {0x006c, F("manual_R1_odd_offset")},
    {0x006d, F("manual_R1_even_offset")},
    {0x006e, F("manual_B2_odd_offset")},
    {0x006f, F("manual_B2_even_offset")},
    {0x0070, F("manual_G2_odd_offset")},
    {0x0071, F("manual_G2_even_offset")},
    {0x0072, F("BLK_DD_thBLK_various_th")},
    {0x0080, F("Block_enable1")},
    {0x0081, F("Block_enable2")},
    {0x0082, F("Block enable")},
    {0x0083, F("Special effect")},
    {0x0084, F("Output format")},
    {0x0085, F("Frame start")},
    {0x0086, F("Sync mode")},
    {0x0087, F("block_enable3_buf")},
    {0x0088, F("module_gating")},
    {0x0089, F("bypass_mode")},
    {0x008c, F("debug_mode2")},
    {0x008d, F("Debug_mode3")},
    {0x0090, F("Crop enable")},
    {0x0091, F("out_win_y1[10:8]")},
    {0x0092, F("out_win_y1 [7:0]")},
    {0x0093, F("out_win_x1[10:8]")},
    {0x0094, F("out_win_x1[7:0]")},
    {0x0095, F("out_win_height[10:8]")},
    {0x0096, F("out_win_height[7:0]")},
    {0x0097, F("out_win_width[10:8]")},
    {0x0098, F("out_win_width[7:0]")},
    {0x0099, F("subsample")},
    {0x009a, F("Subsample mode")},
    {0x009b, F("Sub_row_N1")},
    {0x009c, F("Sub_row_N2")},
    {0x009d, F("Sub_row_N3")},
    {0x009e, F("Sub_row_N4")},
    {0x009f, F("Sub_col_N1")},
    {0x00a0, F("Sub_col_N2")},
    {0x00a1, F("Sub_col_N3")},
    {0x00a2, F("Sub_col_N4")},
    {0x00a3, F("channel_gain_G1_odd")},
    {0x00a4, F("channel_gain_G1_even")},
    {0x00a5, F("channel_gain_R1_odd")},
    {0x00a6, F("channel_gain_R1_even")},
    {0x00a7, F("channel_gain_B2_odd")},
    {0x00a8, F("channel_gain_")},
    {0x00a9, F("channel_gain_G2_odd")},
    {0x00aa, F("channel_gain_G2_even")},
    {0x00ad, F("R_ratio")},
    {0x00ae, F("G_ratio")},
    {0x00af, F("B_ratio")},
    {0x00b0, F("Global_gain")},
    {0x00b1, F("Auto_pregain")},
    {0x00b2, F("Auto_postgain")},
    {0x00b3, F("AWB_R_gain")},
    {0x00b4, F("AWB_G_gain")},
    {0x00b5, F("AWB_B_gain")},
    {0x00b6, F("AEC_enable")},
    {0x00c2, F("output_buf_enable_buf")},
    {0x0101, F("AEC_x1")},
    {0x0102, F("AEC_x2")},
    {0x0103, F("AEC_y1")},
    {0x0104, F("AEC_y2")},
    {0x0105, F("AEC_center_x1")},
    {0x0106, F("AEC_center_x2")},
    {0x0107, F("AEC_center_y1")},
    {0x0108, F("AEC_center_y2")},
    {0x010a, F("AEC_mode1")},
    {0x010b, F("AEC_mode2")},
    {0x010c, F("AEC_mode3")},
    {0x010d, F("AEC_mode4")},
    {0x010e, F("AEC_high_range")},
    {0x010f, F("AEC_low_range")},
    {0x0113, F("AEC_target_Y")},
    {0x0114, F("Y_average")},
    {0x0115, F("target_Y_limit_from_histogram")},
    {0x0116, F("AEC_number_limit_high_range")},
    {0x0118, F("AEC mode5")},
    {0x0119, F("AEC mode 6")},
    {0x011a, F("AEC gainmode")},
    {0x011f, F("AEC_max_pre_dg_gain")},
    {0x0120, F("AEC_max_post_dg_gain")},
    {0x0125, F("AEC_anti_flicker_step[12:8]")},
    {0x0126, F("AEC_anti_flicker_step[7:0]")},
    {0x0127, F("AEC_exp_level_1[12:8]")},
    {0x0128, F("AEC_exp_level_1[7:0]")},
    {0x0129, F("AEC_exp_level_2[12:8]")},
    {0x012a, F("AEC_exp_level_2[7:0]")},
    {0x012b, F("AEC_exp_level_3[12:8]")},
    {0x012c, F("AEC_exp_level_3[7:0]")},
    {0x012d, F("AEC_exp_level_4[12:8]")},
    {0x012e, F("AEC_exp_level_4[7:0]")},
    {0x012f, F("AEC_exp_level_5[12:8]")},
    {0x0130, F("AEC_exp_level_5[7:0]")},
    {0x0131, F("AEC_exp_level_6[12:8]")},
    {0x0132, F("AEC_exp_level_6[7:0]")},
    {0x0133, F("AEC_exp_level_7[12:8]")},
    {0x0134, F("AEC_exp_level_7[7:0]")},
    {0x0135, F("AEC_max_dg_gain1")},
    {0x0136, F("AEC_max_dg_gain2")},
    {0x0137, F("AEC_max_dg_gain3")},
    {0x0138, F("AEC_max_dg_gain4")},
    {0x0139, F("AEC_max_dg_gain5")},
    {0x013a, F("AEC_max_dg_gain6")},
    {0x013b, F("AEC_max_dg_gain7")},
    {0x013c, F("AEC_max_exp_level")},
    {0x013d, F("AEC_exp_min_l[7:0]")},
    {0x0150, F("AWB mode 1")},
    {0x0151, F("AWBparameter")},
    {0x0152, F("AWBparameter")},
    {0x0153, F("AWBparameter")},
    {0x0154, F("AWBparameter")},
    {0x0155, F("AWBparameter")},
    {0x0156, F("AWBparameter")},
    {0x0157, F("AWBparameter")},
    {0x0158, F("AWBparameter")},
    {0x0159, F("AWB_PRE_RGB_low")},
    {0x015a, F("AWB_PRE_RGB_high")},
    {0x015b, F("AWBparameter")},
    {0x0175, F("AWB_every_N")},
    {0x0176, F("AWB_R_gain_limit")},
    {0x0177, F("AWB_G_gain_limit")},
    {0x0178, F("AWB_B_gain_limit")},
    {0x0179, F("AWB_R_gain_out_h_limit")},
    {0x017a, F("AWB_G_gain_out_h_limit")},
    {0x017b, F("AWB_B_gain_out_h_limit")},
    {0x017c, F("AWB_R_gain_out_l_limit")},
    {0x017d, F("AWB_G_gain_out_l_limit")},
    {0x017e, F("AWB_B_gain_out_l_limit")},
    {0x019a, F("ABS_range_compesateABS_skip_frame")},
    {0x019b, F("ABS_stop_margin")},
    {0x019c, F("Y_S_compesateABS_manual_K")},
    {0x019d, F("Y_stretch_limit")},
    {0x01a0, F("LSC_row_x2LSC_col_x2LSC_pixel_array_select")},
    {0x01a1, F("LSC_row_center")},
    {0x01a2, F("LSC_col_cente")},
    {0x01a4, F("LSC_Q12_RGB_sign")},
    {0x01a5, F("LSC_Q34_RGB_SIGN")},
    {0x01a6, F("LSC_right_left_rgb_b4_sign")},
    {0x01a7, F("LSC_up_down_rgb_b4_sign")},
    {0x01a8, F("LSC_right_up_down_rgb_b22_sign")},
    {0x01a9, F("LSC_left_up_down_rgb_b22_sign")},
    {0x01aa, F("LSC_Q1_red_b1")},
    {0x01ab, F("LSC_Q1_green_b1")},
    {0x01ac, F("LSC_Q1_blue_b1")},
    {0x01ad, F("LSC_Q2_red_b1")},
    {0x01ae, F("LSC_Q2_green_b1")},
    {0x01af, F("LSC_Q2_blue_b1")},
    {0x01b0, F("LSC_Q3_red_b1")},
    {0x01b1, F("LSC_Q3_green_b1")},
    {0x01b2, F("LSC_Q3_blue_b1")},
    {0x01b3, F("LSC_Q4_red_b1")},
    {0x01b4, F("LSC_Q4_green_b1")},
    {0x01b5, F("LSC_Q4_blue_b1")},
    {0x01b6, F("LSC_right_red_b2")},
    {0x01b7, F("LSC_right_green_b2")},
    {0x01b8, F("LSC_right_blue_b2")},
    {0x01b9, F("LSC_right_red_b4")},
    {0x01ba, F("LSC_right_green_b4")},
    {0x01bb, F("LSC_right_blue_b4")},
    {0x01bc, F("LSC_left_red_b2")},
    {0x01bd, F("LSC_left_green_b2")},
    {0x01be, F("LSC_left_blue_b2")},
    {0x01bf, F("LSC_left_red_b4")},
    {0x01c0, F("LSC_left_green_b4")},
    {0x01c1, F("LSC_left_blue_")},
    {0x01c2, F("LSC_up_red_b2")},
    {0x01c3, F("LSC_up_green_b2")},
    {0x01c4, F("LSC_up_blue_b2")},
    {0x01c5, F("LSC_up_red_b4")},
    {0x01c6, F("LSC_up_green_b4")},
    {0x01c7, F("LSC_up_blue_b4")},
    {0x01c8, F("LSC_down_red_b2")},
    {0x01c9, F("LSC_down_green_b2")},
    {0x01ca, F("LSC_down_blue_b2")},
    {0x01cb, F("LSC_down_red_b4")},
    {0x01cc, F("LSC_down_green_b4")},
    {0x01cd, F("LSC_down_blue_b4")},
    {0x01d0, F("LSC_right_up_red_b22")},
    {0x01d1, F("LSC_right_up_green_b22")},
    {0x01d2, F("LSC_right_up_blue_b22")},
    {0x01d3, F("LSC_right_down_red_b22")},
    {0x01d4, F("LSC_right_down_green_b22")},
    {0x01d5, F("LSC_right_down_blue_b22")},
    {0x01d6, F("LSC_left_up_red_b22")},
    {0x01d7, F("LSC_left_up_green_b22")},
    {0x01d8, F("LSC_left_up_blue_b22")},
    {0x01d9, F("LSC_left_down_red_b22")},
    {0x01da, F("LSC_left_down_green_b22")},
    {0x01db, F("LSC_left_down_blue_b22")},
    {0x01dc, F("LSC_Y_dark_th")},
    {0x01dd, F("LSC_Y_dark_slope")},
    {0x01df, F("LSC_U_B2G_stand_plus")},
    {0x0210, F("Gamma_out1")},
    {0x0211, F("Gamma_out2")},
    {0x0212, F("Gamma_out3")},
    {0x0213, F("Gamma_out4")},
    {0x0214, F("Gamma_out5")},
    {0x0215, F("Gamma_out6")},
    {0x0216, F("Gamma_out7")},
    {0x0217, F("Gamma_out8")},
    {0x0218, F("Gamma_out9")},
    {0x0219, F("Gamma_out10")},
    {0x021a, F("Gamma_out11")},
    {0x021b, F("Gamma_out12")},
    {0x021c, F("Gamma_out13")},
    {0x021d, F("Gamma_out14")},
    {0x021e, F("Gamma_out15")},
    {0x021f, F("Gamma_out16")},
    {0x0220, F("Gamma_out17")},
    {0x0221, F("Gamma_out18")},
    {0x0222, F("Gamma_out19")},
    {0x0223, F("Gamma_out20")},
    {0x0224, F("Gamma_out21")},
    {0x0225, F("Gamma_out22")},
    {0x0284, F("DD_dark_th")},
    {0x0285, F("ASDE_DN_B_slope")},
    {0x0289, F("ASDE_low_luma_value_DD_th2")},
    {0x028a, F("ASDE_low_luma_value_DD_th3")},
    {0x028b, F("ASDE_low_luma_value_DD_th4")},
    {0x0290, F("EEINTP")},
    {0x0291, F("EEINTP")},
    {0x0292, F("direction_TH1")},
    {0x0293, F("Direction_TH2")},
    {0x0294, F("diff_HV_mode")},
    {0x0295, F("direction_diff_")},
    {0x0296, F("edge level")},
    {0x0297, F("Edge1 effect")},
    {0x0298, F("Edge_pos_ratio")},
    {0x0299, F("Edge1_max")},
    {0x029a, F("Edge2_max")},
    {0x029b, F("Edge1_th")},
    {0x029c, F("Edge_pos_max")},
    {0x029d, F("Edge_effect_sc")},
    {0x02c0, F("CC_mode")},
    {0x02c1, F("CC_CT1_11")},
    {0x02c2, F("CC_CT1_12")},
    {0x02c3, F("CC_CT1_13")},
    {0x02c4, F("CC_CT1_21")},
    {0x02c5, F("CC_CT1_22")},
    {0x02c6, F("CC_CT1_23")},
    {0x02d0, F("Global")},
    {0x02d1, F("saturation_Cb")},
    {0x02d2, F("saturation_Cr")},
    {0x02d3, F("luma_contrast")},
    {0x02d4, F("Contrast center")},
    {0x02d5, F("Luma_offset")},
    {0x02d6, F("skin_Cb_center")},
    {0x02d7, F("skin_Cr_center")},
    {0x02d9, F("Skin brightnessmode")},
    {0x02da, F("Fixed_Cb")},
    {0x02db, F("Fixed_Cr")},
    {0x02e6, F("CC_R_offset")},
    {0x02e7, F("CC_G_offset")},
    {0x02e8, F("CC_B_offset")},
    {0x0301, F("DPHY_analog_mode1")},
    {0x0302, F("DPHY_analog_mode2")},
    {0x0303, F("DPHY_analog_mode3")},
    {0x0304, F("FIFO_prog_full_level[7:0]")},
    {0x0305, F("FIFO_prog_full_level[11:8]")},
    {0x0306, F("FIFO_mode")},
    {0x0310, F("BUF_CSI2_mode")},
    {0x0311, F("LDI_set")},
    {0x0312, F("LWC_set[7:0]")},
    {0x0313, F("LWC_set[15:8]")},
    {0x0314, F("SYNC_set")},
    {0x0315, F("DPHY_mode")},
    {0x0316, F("LP_set")},
    {0x0317, F("fifo_gate_modeMIPI_wdiv_set")},
    {0x0320, F("T_init_set")},
    {0x0321, F("T_LPX_set")},
    {0x0322, F("T_CLK_HS_PREPARE_set")},
    {0x0323, F("T_CLK_zero_set")},
    {0x0324, F("T_CLK_PRE_set")},
    {0x0325, F("T_CLK_POST_set")},
    {0x0326, F("T_CLK_TRAIL_set")},
    {0x0327, F("T_HS_exit_set")},
    {0x0328, F("T_wakeup_set")},
    {0x0329, F("T_HS_PREPARE_set")},
    {0x032a, F("T_HS_Zero_set")},
    {0x032b, F("T_HS_TRAIL_set")},
    {0x0330, F("MIPI_Test")},
    {0x0331, F("MIPI_Test_data0")},
    {0x0332, F("MIPI_Test_data1")},
    {0x0333, F("MIPI_Test_data2")},
    {0x0334, F("MIPI_Test_data3")},
    {0x033f, F("FIFO_error log")},
    {0x0340, F("output_buf_mode1")},
    {0x0341, F("output_buf_mode2")},
    {0x0342, F("buf_win_width[7:0]")},
    {0x0343, F("buf_win_width[11:8]")},
};

int GC2145::regWrite(uint8_t dev_addr, uint16_t reg_addr, uint8_t reg_data, bool wide_addr)
{
#ifdef DEBUG_CAMERA
    static uint16_t reg_page = 0;
    if (_debug != nullptr) {
        if (reg_addr == 0xfe) reg_page = (reg_data & 0x7) << 8; 

        _debug->print("Write Register ("); 
        _debug->print(reg_page >> 8);
        _debug->print(":0x");
        _debug->print(reg_addr, HEX);
        _debug->print("): (0x"); 
        _debug->print(reg_data, HEX);
        _debug->print(" - ");
        _debug->print(reg_data, DEC);
        _debug->print(")");
        uint16_t reg_lookup = reg_addr;
        if (reg_data < 0xf0) reg_lookup += reg_page;
        for (uint16_t jj=0; jj < (sizeof(GC2145_reg_name_table)/sizeof(GC2145_reg_name_table[0])); jj++) {
            if (reg_lookup == GC2145_reg_name_table[jj].reg) {
                _debug->print("\t: ");
                _debug->print(GC2145_reg_name_table[jj].reg_name);
                break;
            }
        }
        _debug->println();        
    }

#endif

    _i2c->beginTransmission(dev_addr);
    uint8_t buf[3] = {(uint8_t) (reg_addr >> 8), (uint8_t) (reg_addr & 0xFF), reg_data};
    if (wide_addr == true) {
        _i2c->write(buf, 1);
    }
    _i2c->write(&buf[1], 2);
    return _i2c->endTransmission();
}

uint8_t GC2145::regRead(uint8_t dev_addr, uint16_t reg_addr, bool wide_addr)
{
    uint8_t reg_data = 0;
    uint8_t buf[2] = {(uint8_t) (reg_addr >> 8), (uint8_t) (reg_addr & 0xFF)};
    _i2c->beginTransmission(dev_addr);
    if (wide_addr) {
        _i2c->write(buf, 2);
    } else {
        _i2c->write(&buf[1], 1);
    }
    _i2c->endTransmission(false);
    _i2c->requestFrom(dev_addr, 1);
    if (_i2c->available()) {
        reg_data = _i2c->read();
    }
    while (_i2c->available()) {
        _i2c->read();
    }
    return reg_data;
}

void GC2145::debug(Stream &stream)
{
  _debug = &stream;
}


uint8_t GC2145::printRegs(void)
{
    if (_debug == nullptr) return 0;
    uint8_t reg;
    _debug->println("\n*** Camera Registers ***");
    for (uint16_t ii = 3; ii < 182; ii++) {
        reg = regRead(GC2145_I2C_ADDR, ii);

        _debug->print("(0x"); 
        _debug->print(ii, HEX);
        _debug->print("): (0x"); 
        _debug->print(reg, HEX);
        _debug->print(" - ");
        _debug->print(reg, DEC);
        _debug->print(")");

        for (uint16_t jj=0; jj < (sizeof(GC2145_reg_name_table)/sizeof(GC2145_reg_name_table[0])); jj++) {
            if (ii == GC2145_reg_name_table[jj].reg) {
                _debug->print("\t: ");
                _debug->print(GC2145_reg_name_table[jj].reg_name);
                break;
            }
        }
        _debug->println();
    }
    return 1;
}
