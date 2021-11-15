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

#ifndef __IMPEGH_HOA_MATRIX_STRUCT_H__
#define __IMPEGH_HOA_MATRIX_STRUCT_H__

typedef struct ia_hoa_matrix_struct_t
{
  UWORD32 is_full_matrix;
  UWORD32 is_hoa_coef_sparse[128];
  UWORD32 first_sparse_order;
  UWORD32 value_symmetric_pairs[128];
  UWORD32 sign_symmetric_pairs[128];
  UWORD32 zeroth_order_always_positive;
  UWORD32 has_vertical_coef;
  UWORD32 is_value_symmetric[128];
  UWORD32 is_sign_symmetric[128];
  UWORD32 num_of_hoa_rendering_matrices;
  UWORD32 hoa_rendering_matrix_id[32];
  UWORD32 cicp_speaker_layout_idx;
  UWORD32 hoa_matrix_len_bits;
  UWORD32 has_lfe_rendering;
  UWORD32 precision_level;
  UWORD32 is_normalized;
  UWORD32 gain_limit_per_hoa_order;
  UWORD32 is_all_value_symmetric;
  UWORD32 is_any_value_symmetric;
  UWORD32 is_any_sign_symmetric;
  UWORD32 is_all_sign_symmetric;

  WORD32 max_gain[MAXIMUM_HOA_ORDER + 1];
  WORD32 min_gain[MAXIMUM_HOA_ORDER + 1];
  WORD32 sign_matrix[MAXIMUM_NUM_HOA_COEFF * HOA_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 sym_signs[MAXIMUM_NUM_HOA_COEFF];
  WORD32 vert_bitmask[MAXIMUM_NUM_HOA_COEFF];
} ia_hoa_matrix_struct;

#endif // __IMPEGH_HOA_MATRIX_STRUCT_H__
