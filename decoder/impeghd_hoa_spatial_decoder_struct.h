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

#ifndef __IA_MPEGH_HOA_SPATIAL_DECODER_STRUCT_H__
#define __IA_MPEGH_HOA_SPATIAL_DECODER_STRUCT_H__

typedef struct
{
  UWORD32 low_order_mat_sz;
  FLOAT32 order_matrix[HOA_MAXIMUM_MODE_MATRIX_DIMENSION * HOA_MAXIMUM_MODE_MATRIX_DIMENSION];
} ia_spatial_dec_amb_syn_str;

typedef struct
{
  UWORD32 prev_ps_sig_indices_set[HOA_MAXIMUM_SIG_INDICES];
  UWORD32 num_prev_vec;
  ia_hoa_vec_sig_str map_prev_vec[MAX_HOA_CHANNELS];
} ia_spatial_dec_vector_based_predom_sound_syn_str;

typedef struct
{
  FLOAT32 mode_mt_all_coarse_grid_points[MAXIMUM_NUM_HOA_COEFF * MAXIMUM_NUM_HOA_COEFF];
  FLOAT32 *transpose_mode_all_fine_grid_points;
  FLOAT32 last_prediction_factors_mat[HOA_MAX_PRED_DIR_SIGNALS * MAXIMUM_NUM_HOA_COEFF];
  FLOAT32 prediction_factors_mat[HOA_MAX_PRED_DIR_SIGNALS * MAXIMUM_NUM_HOA_COEFF];
  FLOAT32 pred_grid_dir_sigs_last[MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE];
  FLOAT32 pred_grid_dir_sigs_smthed[MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE];
  UWORD32 filter_delay;
  UWORD32 set_of_old_ps_sig_indices[HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED];
  UWORD32 last_prediction_type[MAXIMUM_NUM_HOA_COEFF];
  UWORD32 last_prediction_indices_mat[HOA_MAX_PRED_DIR_SIGNALS * MAXIMUM_NUM_HOA_COEFF];
  UWORD32 num_prev_dir_predom_sounds;
  UWORD32 num_prev_ps_indices;
  ia_hoa_dir_id_str last_active_dir_and_grid_dir_indices[HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED];
} ia_spatial_dec_dir_based_pre_dom_sound_syn_str;

typedef struct
{
  FLOAT32 fade_in_win_for_vec_sigs[MAXIMUM_FRAME_SIZE];
  FLOAT32 fade_out_win_for_vec_sigs[MAXIMUM_FRAME_SIZE];
  FLOAT32 fade_in_win_for_dir_sigs[MAXIMUM_FRAME_SIZE];
  FLOAT32 fade_out_win_for_dir_sigs[MAXIMUM_FRAME_SIZE];
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str dir_based_pre_dom_sound_syn_handle;
  ia_spatial_dec_vector_based_predom_sound_syn_str vec_based_pre_dom_sound_syn_handle;
} ia_spatial_dec_pre_dom_sound_syn_str;

typedef struct
{
  UWORD32 min_num_coeffs_for_amb_hoa;
  UWORD32 num_vec_elems;
  UWORD32 vec_start;
  UWORD32 num_bits_for_vec_elem_quant;
  UWORD32 max_num_transmitted_hoa_coeffs;
  UWORD32 num_subband_groups;
  UWORD32 app_dyn_corr;
  WORD32 use_phase_shift_decorr;
  FLOAT32 *ptr_inv_mode_mat;
  ia_hoa_config_struct *ia_hoa_config;
  ia_hoa_dec_frame_param_str frame_params_handle;
  ia_spatial_dec_pre_dom_sound_syn_str pre_dom_sound_syn_handle;
  ia_spatial_dec_amb_syn_str amb_syn_handle;

  FLOAT32 transpose_mod_mtx_grid_pts[900 * HOA_MAXIMUM_MATRIX_SIZE];
  FLOAT32 dict_cicp_speaker_points[34 * HOA_MAXIMUM_MATRIX_SIZE];
  FLOAT32 dict_2d_points[64 * HOA_MAXIMUM_MATRIX_SIZE];

  FLOAT32 dir_sigs_hoa_frame_buf[MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE];
  FLOAT32 amb_hoa_coeffs_frame_buf[MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE];
  FLOAT32 *inv_dyn_corr_win_func;
  FLOAT32 inv_dyn_corr_prev_gain[HOA_MAXIMUM_NUM_PERC_CODERS];
  FLOAT32 dyn_corr_sample_buf[MAX_HOA_CHANNELS * MAXIMUM_FRAME_SIZE];
  FLOAT32 ones_long_vec[2 * MAXIMUM_FRAME_SIZE];
  pVOID scratch;
  ia_hoa_vec_sig_str old_nominators[MAX_HOA_CHANNELS];
} ia_spatial_dec_str;
#endif //__IA_MPEGH_HOA_SPATIAL_DECODER_STRUCT_H__
