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

#ifndef __IA_MPEGHD_HOA_ROM_H__
#define __IA_MPEGHD_HOA_ROM_H__

typedef struct
{
  UWORD16 symbol;
  WORD32 len;
  WORD32 code_word;
} impeghd_word_struct;

extern const FLOAT32 ia_hoa_beta_value_table[5];
extern const FLOAT32 *ia_hoa_3d_mix_ren_win_tbl[7];
extern const FLOAT32 *ia_hoa_kaiser_win_tbl[7];
extern const FLOAT32 ia_hoa_pow_2_table[64];
extern const FLOAT32 ia_hoa_pow_2_inv_table[64];
extern const FLOAT32 ia_hoa_spk_incl_m1_theta[256];
extern const FLOAT32 ia_hoa_spk_azimuth_m1_phi[256];

extern const impeghd_word_struct ia_hoa_frame_huffman_table[10][5][15];

extern const FLOAT32 ia_hoa_spk_incl_s1_theta[324];
extern const FLOAT32 ia_spk_azimuth_s1_phi[324];

extern const WORD32 ia_hoa_curr_order[MAXIMUM_NUM_HOA_COEFF];
extern const FLOAT32 ia_hoa_curr_scalar[MAXIMUM_HOA_ORDER + 1];

extern const WORD32 ia_hoa_spat_interp_time_code_table[8];

extern const FLOAT32 ia_hoa_nf_roots_re_abs[];
extern const FLOAT32 ia_hoa_product_n_to_m[17][17];

extern const FLOAT32 *ia_hoa_search_grid_azimuths[3];
extern const FLOAT32 *ia_hoa_cos_search_grid_inclinations[3];
extern const FLOAT32 *ia_hoa_sin_search_grid_inclinations[3];

extern const FLOAT32 *ia_hoa_cos_inclinations[MAXIMUM_HOA_ORDER];
extern const FLOAT32 *ia_hoa_sin_inclinations[MAXIMUM_HOA_ORDER];

extern const FLOAT32 ia_hoa_psi_n6[49 * HOA_N_PSI_POINTS];

extern const FLOAT32 ia_hoa_weighting_cdbk_256x8[256 * 8];

extern const FLOAT32 ia_hoa_cos_cicp_speaker_inclinations[51];
extern const FLOAT32 ia_hoa_sin_cicp_speaker_inclinations[51];
extern const FLOAT32 *ia_hoa_azimuths[MAXIMUM_HOA_ORDER];

extern const FLOAT32 *ia_hoa_coded_vec_q_mat_elems[7];

extern pFLOAT32 ia_hoa_p_phi_grid_sa[];
extern pFLOAT32 ia_hoa_p_theta_grid_sa[];

extern const UWORD32 ia_hoa_cicp_speakerd_azimuths[51];
extern const FLOAT32 ia_hoa_cos_table_compute_fade_win_for_dir_based_syn[2048];
extern const FLOAT32 ia_hoa_cos_table_inv_dyn_correction_compute_win_func[1024];
extern const FLOAT32 ia_hoa_cos_table_fade_win_for_vec_based_syn[2][8][2048];
extern const FLOAT32 ia_hoa_inv_dyn_win_pow_table[7][1024];
extern const WORD32 ia_hoa_frame_length_table[3];
extern const FLOAT32 ia_hoa_coded_vec_q_mat_elems_0[1];
extern const FLOAT32 ia_hoa_coded_vec_q_mat_elems_1[16];
extern const FLOAT32 ia_hoa_coded_vec_q_mat_elems_2[81];
extern const FLOAT32 ia_hoa_coded_vec_q_mat_elems_3[256];
extern const FLOAT32 ia_hoa_coded_vec_q_mat_elems_4[800];
extern const FLOAT32 ia_hoa_coded_vec_q_mat_elems_5[1296];
extern const FLOAT32 ia_hoa_coded_vec_q_mat_elems_6[2401];

// extern const FLOAT32 ia_hoa_nf_roots_re_abs[];

// extern const WORD32 ia_hoa_square[64];

#endif //__IA_MPEGHD_HOA_ROM_H__
