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

#ifndef IMPEGHD_MP4_UTILS_H
#define IMPEGHD_MP4_UTILS_H

typedef struct
{
  UWORD32 lsw;
  UWORD32 msw;
} it_uint64;

#define IT_EXIT 1
#define IT_ERROR -1
#define IT_OK 0
#define IT_DASH 2
#define IT_STR_LEN 1024
#define IT_BUF_LEN 1024
#define IT_LITTLE_ENDIAN 1
#if IT_LITTLE_ENDIAN
#define RTP_LITTLE_ENDIAN 1
#define RTP_BIG_ENDIAN 0
#else
#define RTP_LITTLE_ENDIAN 0
#define RTP_BIG_ENDIAN 1
#endif

#ifdef _WIN32_WCE
#define IT_CONSOLE 0
#else
#define IT_CONSOLE 1
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

typedef enum { W_OK, W_STRSTOP, W_FRMSTOP } buf_status;

typedef struct
{
  WORD32 read;
  WORD32 write;
  WORD32 wrap;
  buf_status status;
  pWORD8 buffer;
  WORD32 buf_len;
  FILE *fp, *fp2;
} ia_mp4_buf;

typedef struct mem
{
  pVOID ptr;
  struct mem *next;
} ia_mp4_mem_node;

VOID impeghd_mp4_error_hdl(FILE *dfp, pWORD8 format, ...);

UWORD32 impeghd_mp4_rev32(UWORD32 l);
UWORD32 impeghd_mp4_rev24(UWORD32 l);
UWORD16 impeghd_mp4_rev16(UWORD16 l);
UWORD32 impeghd_mp4_rev64(UWORD32 l);
#define bit_pack(temp1, data, bits) (temp1 << bits) | data
pVOID impeghd_mp4_mem_node_malloc(ia_mp4_mem_node **m, WORD32 size);
pVOID impeghd_mp4_mem_node_calloc(ia_mp4_mem_node **m, WORD32 count, WORD32 size);
VOID impeghd_mp4_free_mem_node(pVOID ptr, ia_mp4_mem_node **m);
VOID impeghd_mp4_free_all_nodes(ia_mp4_mem_node **m);
WORD32 impeghd_mp4_find_str_file(VOID **pfp, pWORD8 buf);
VOID impeghd_mp4_add64(it_uint64 *ans, it_uint64 a, UWORD32 b);
UWORD32 impeghd_mp4_div64(UWORD32 *ans, it_uint64 divident, UWORD32 divisor);
VOID impeghd_mp4_multi_1k(it_uint64 *a, it_uint64 b);

pVOID impeghd_mp4_malloc_wrapper(WORD32 size);
pVOID impeghd_mp4_calloc_wrapper(WORD32 count, WORD32 size);
VOID impeghd_mp4_free_wrapper(pVOID ptr);

#endif /*IMPEGHD_MP4_UTILS_H */
