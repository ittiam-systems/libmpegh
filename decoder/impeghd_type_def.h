/* 	Copyright (c) [2020]-[2021] Ittiam Systems Pvt. Ltd.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted (subject to the limitations in the
   disclaimer below) provided that the following conditions are met:
   •	Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   •	Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   •	Neither the names of Dolby Laboratories, Inc. (or its affiliates),
   Ittiam Systems Pvt. Ltd. nor the names of its contributors may be used
   to endorse or promote products derived from this software without
   specific prior written permission.

   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
   BY THIS LICENSE. YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT
   LICENSE(S) BY THIRD PARTIES, INCLUDING, WITHOUT LIMITATION, DOLBY
   LABORATORIES, INC. OR ANY OF ITS AFFILIATES. THIS SOFTWARE IS PROVIDED
   BY ITTIAM SYSTEMS LTD. AND ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
   IN NO EVENT SHALL ITTIAM SYSTEMS LTD OR ITS CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------
*/

#ifndef IMPEGHD_TYPE_DEF_H
#define IMPEGHD_TYPE_DEF_H

/****************************************************************************/
/*     types               type define    prefix    examples
 * bytes */
/************************  ***********    ******    ****************  ***** */

#define SIZE_T size_t

// Definitions of common types
#ifdef _WIN64
typedef unsigned __int64 size_t;
typedef __int64 ptrdiff_t;
typedef __int64 intptr_t;
#else
#ifndef _X86_
// typedef unsigned int size_t;
#endif
typedef int ptrdiff_t;
typedef int intptr_t;
#endif

typedef char CHAR8;   /* c       CHAR8    c_name     1   */
typedef char *pCHAR8; /* pc      pCHAR8   pc_nmae    1   */

typedef signed char WORD8;   /* b       WORD8    b_name     1   */
typedef signed char *pWORD8; /* pb      pWORD8   pb_nmae    1   */

typedef unsigned char UWORD8;   /*  ub    UWORD8   ub_count  1  */
typedef unsigned char *pUWORD8; /*  pub    pUWORD8   pub_count  1  */

typedef signed short WORD16;      /* s       WORD16   s_count    2   */
typedef signed short *pWORD16;    /* ps      pWORD16  ps_count   2   */
typedef unsigned short UWORD16;   /*  us    UWORD16   us_count  2  */
typedef unsigned short *pUWORD16; /*  pus    pUWORD16 pus_count  2  */

typedef signed int WORD24;      /* k       WORD24   k_count    3   */
typedef signed int *pWORD24;    /* pk      pWORD24  pk_count   3   */
typedef unsigned int UWORD24;   /*  uk    UWORD24   uk_count  3  */
typedef unsigned int *pUWORD24; /*  puk    pUWORD24 puk_count  3  */

typedef signed int WORD32;      /* i       WORD32   i_count    4   */
typedef signed int *pWORD32;    /* pi      pWORD32  pi_count   4   */
typedef unsigned int UWORD32;   /*  ui    UWORD32   ui_count  4  */
typedef unsigned int *pUWORD32; /*  pui    pUWORD32 pui_count  4  */
#ifdef WIN32
typedef signed __int64 WORD40;      /*  m    WORD40   m_count  5  */
typedef signed __int64 *pWORD40;    /*  pm    pWORD40   pm_count  5  */
typedef unsigned __int64 UWORD40;   /*  um    UWORD40   um_count  5  */
typedef unsigned __int64 *pUWORD40; /*  pum    pUWORD40 pum_count  5  */

typedef signed __int64 WORD64;      /*  h    WORD64   h_count  8  */
typedef signed __int64 *pWORD64;    /*  ph    pWORD64   ph_count  8  */
typedef unsigned __int64 UWORD64;   /*  uh    UWORD64   uh_count  8  */
typedef unsigned __int64 *pUWORD64; /*  puh    pUWORD64 puh_count  8  */
#else
typedef signed long long int WORD40;      /*  m    WORD40   m_count  5  */
typedef signed long long int *pWORD40;    /*  pm    pWORD40   pm_count  5  */
typedef unsigned long long int UWORD40;   /*  um    UWORD40   um_count  5  */
typedef unsigned long long int *pUWORD40; /*  pum    pUWORD40 pum_count  5  */

typedef signed long long int WORD64;    /*  h    WORD64   h_count  8  */
typedef signed long long int *pWORD64;  /*  ph    pWORD64   ph_count  8  */
typedef unsigned long long int UWORD64; /*  uh    UWORD64   uh_count  8  */
typedef unsigned long long int *pUWORD64;
#endif
typedef float FLOAT32;    /*  f    FLOAT32   f_count  4
                             */
typedef float *pFLOAT32;  /* pf      pFLOAT32 pf_count   4   */
typedef double FLOAT64;   /*  d    UFLOAT64 d_count  8
                             */
typedef double *pFlOAT64; /* pd      pFLOAT64 pd_count   8   */

typedef void VOID;   /*  v    VOID   v_flag    4  */
typedef void *pVOID; /*  pv    pVOID   pv_flag  4  */

/* variable size types: platform optimized implementation */
typedef signed int BOOL;       /* bool    BOOL     bool_true      */
typedef unsigned int UBOOL;    /*  ubool  BOOL   ubool_true    */
typedef signed int FLAG;       /* flag    FLAG     flag_false     */
typedef unsigned int UFLAG;    /* uflag  FLAG   uflag_false  */
typedef signed int LOOPIDX;    /* lp      LOOPIDX  lp_index       */
typedef unsigned int ULOOPIDX; /*  ulp    SLOOPIDX ulp_index    */
typedef signed int WORD;       /* lp      LOOPIDX  lp_index       */
typedef unsigned int UWORD;    /*  ulp    SLOOPIDX ulp_index    */

typedef LOOPIDX LOOPINDEX;   /* lp    LOOPIDX  lp_index       */
typedef ULOOPIDX ULOOPINDEX; /* ulp   SLOOPIDX ulp_index      */

typedef WORD32 IA_ERRORCODE;

#ifndef NULL
#define NULL ((void *)0)
#endif

#define PLATFORM_INLINE __inline
#endif /* IMPEGHD_TYPE_DEF_H */
