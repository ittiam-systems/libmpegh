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

#ifndef __IA_MPEGH_HOA_SIMPLE_MTRX_H__
#define __IA_MPEGH_HOA_SIMPLE_MTRX_H__

typedef struct
{
  WORD32 cols;
  WORD32 rows;
  FLOAT32 mtrx[MAXIMUM_NUM_HOA_COEFF * HOA_N_PSI_POINTS];
} ia_render_hoa_simple_mtrx_str;

typedef struct
{
  WORD32 cols;
  WORD32 rows;
  FLOAT32 mtrx[HOA_MAXIMUM_OUTPUT_LS * HOA_MAXIMUM_OUTPUT_LS];
} ia_render_hoa_pan_mtrx_str;

typedef struct
{
  FLOAT32 w_s[HOA_MAX_MATRIX_SIZE];
  WORD32 w_s_sz;
  ia_render_hoa_pan_mtrx_str u;
  ia_render_hoa_pan_mtrx_str s;
  ia_render_hoa_pan_mtrx_str v;
} ia_render_hoa_simple_mtrx_svd_str;

typedef struct
{
  ia_render_hoa_space_positions_str source_pos_sp;
  ia_render_hoa_space_positions_str source_pos_ca;
  ia_render_hoa_pan_mtrx_str bb;
  ia_render_hoa_simple_mtrx_str hml_t_r;
  ia_render_hoa_simple_mtrx_str hml_t_i;
  ia_render_hoa_pan_mtrx_str a;
  ia_render_hoa_pan_mtrx_str c_mat;
  ia_render_hoa_pan_mtrx_str t;
  ia_render_hoa_pan_mtrx_str y00;
  ia_render_hoa_pan_mtrx_str y01;
  ia_render_hoa_pan_mtrx_str a1;
  ia_render_hoa_pan_mtrx_str a2;
  ia_render_hoa_pan_mtrx_str r0temp;
  ia_render_hoa_pan_mtrx_str r0;
  ia_render_hoa_pan_mtrx_str t_temp;
  ia_render_hoa_simple_mtrx_svd_str svd_a;
  ia_render_hoa_simple_mtrx_svd_str svd_t;
  FLOAT32 b[HOA_MAX_VECTOR_SIZE];
  FLOAT32 b1[HOA_MAX_VECTOR_SIZE];
  FLOAT32 b2[HOA_MAX_VECTOR_SIZE];
  FLOAT32 d[HOA_MAX_VECTOR_SIZE];
  FLOAT32 d1[HOA_MAX_VECTOR_SIZE];
  FLOAT32 d2[HOA_MAX_VECTOR_SIZE];
  FLOAT32 g[HOA_MAX_VECTOR_SIZE];
  FLOAT32 g1[HOA_MAX_VECTOR_SIZE];
  FLOAT32 g2[HOA_MAX_VECTOR_SIZE];
} ia_render_hoa_pan_calculate_gains_pre_scratch;

VOID impeghd_hoa_ren_simple_mtrx_diag_mult(pVOID handle, pVOID arr, WORD32 diag_len);

IA_ERRORCODE impeghd_hoa_ren_simple_mtrx_svd_init(ia_render_hoa_simple_mtrx_svd_str *svd_handle,
                                                  ia_render_hoa_simple_mtrx_str *sm_handle_og,
                                                  pVOID scratch);

IA_ERRORCODE impeghd_hoa_ren_simple_mtrx_mult(pVOID handle_1, pVOID handle_2, pVOID handle_mult,
                                              pVOID scratch);

FLOAT32 impeghd_hoa_ren_simple_mtrx_norm_fro(pVOID handle);

VOID impeghd_hoa_ren_simple_mtrx_svd_pinv(pVOID handle_svd, pVOID handle_sm, FLOAT32 threshold,
                                          pVOID scratch);

FLOAT32 impeghd_hoa_ren_simple_mtrx_norm_fro(pVOID handle);

VOID impeghd_hoa_ren_simple_mtrx_svd_pinv(pVOID handle_svd, pVOID handle_sm, FLOAT32 threshold,
                                          pVOID scratch);

VOID impeghd_hoa_ren_simple_mtrx_transpose_keep(pVOID handle_og, pVOID handle);

VOID impeghd_hoa_ren_simple_mtrx_transpose(pVOID handle, pVOID scratch);

VOID impeghd_hoa_ren_simple_mtrx_init_with_size(pVOID handle, WORD32 rows, WORD32 cols);

VOID impeghd_hoa_ren_simple_mtrx_init_with_matrix(pVOID handle_og, pVOID handle);

#endif // __IA_MPEGH_HOA_SIMPLE_MTRX_H__
