/*
 *
 ****************************************************************************
 * Copyright (C) 2021 QST Corporation. <www.qstcorp.com>
 * All rights reserved.
 *
 * File : type_support.h
 *
 * Date : 2021/06/05
 *
 * Revision : 1.0.0
 *
 * Usage: Define various support type
 *
 ****************************************************************************
 *
 */
 
#ifndef __TYPE_SUPPORT_H__
#define __TYPE_SUPPORT_H__

/*signed integer types*/
typedef	signed char  s8;
typedef	signed short int s16;
typedef	signed int s32;
typedef	signed long long s64;

/*unsigned integer types*/
typedef	unsigned char u8;
typedef	unsigned short int u16;
typedef	unsigned int u32;
typedef	unsigned long long u64;

typedef union {
   struct{
	s32 x;
	s32 y;
	s32 z;
	s32 t;
   } u;
	s32 v[4];
} raw_data_xyzt_t;

#endif //__TYPE_SUPPORT_H__
