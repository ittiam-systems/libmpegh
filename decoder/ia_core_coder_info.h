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

#ifndef IA_CORE_CODER_INFO_H
#define IA_CORE_CODER_INFO_H

#define chans 51

#define EXT_SBR_DATA 13

typedef struct
{
  WORD32 samp_rate;
  WORD32 num_sfb_1024;
  const WORD16 *ptr_sfb_1024;
  WORD32 num_sfb_128;
  const WORD16 *ptr_sfb_128;
  WORD32 short_fss_width;
  WORD32 long_fss_groups;
  WORD32 num_sfb_512;
  const WORD16 *ptr_sfb_512;
} ia_usac_samp_rate_info;

typedef struct
{
  WORD32 index;
  WORD32 len;
  UWORD32 code_word;
} ia_huff_code_word_struct;

typedef struct
{
  WORD32 num;
  WORD32 dim;
  WORD32 lav;
  WORD32 lav_incr_esc;
  WORD32 huff_mode;
  WORD32 off;
  WORD32 sign_code_book;
  UWORD16 max_code_word_len;
  const ia_huff_code_word_struct *pstr_huff_code_word;
  const WORD16 *code_book_tbl;
  const WORD32 *idx_tbl;
} ia_huff_code_book_struct;

typedef struct
{
  WORD32 num_ele;
  WORD32 ele_is_cpe[(1 << LEN_TAG)];
  WORD32 ele_tag[(1 << LEN_TAG)];
} ia_ele_list_struct;

typedef struct
{
  WORD32 present;
  WORD32 ele_tag;
  WORD32 pseudo_enab;
} ia_mix_dwn_struct;

typedef struct
{
  WORD32 tag;
  WORD32 profile;
  WORD32 sampling_rate_idx;
  ia_ele_list_struct front;
  ia_ele_list_struct side;
  ia_ele_list_struct back;
  ia_ele_list_struct lfe;
  ia_ele_list_struct data;
  ia_ele_list_struct coupling;
  ia_mix_dwn_struct mono_mix;
  ia_mix_dwn_struct stereo_mix;
  ia_mix_dwn_struct matrix_mix;
  WORD8 comments[(1 << LEN_PC_COMM) + 1];
  WORD32 buffer_fullness;
} ia_prog_config_struct;

extern ia_huff_code_book_struct ia_core_coder_book;

VOID ia_core_coder_hufftab(ia_huff_code_book_struct *ptr_huff_code_book,
                           const ia_huff_code_word_struct *ptr_huff_code_word,
                           const WORD16 *code_book_tbl, const WORD32 *index, WORD32 dim,
                           WORD32 lav, WORD32 lav_incr_esc, WORD32 sign_code_book,
                           UWORD8 max_code_word_len);

WORD32 ia_core_coder_huff_codeword(const ia_huff_code_word_struct *h,
                                   ia_bit_buf_struct *it_bit_buff);

#endif /* IA_CORE_CODER_INFO_H */
