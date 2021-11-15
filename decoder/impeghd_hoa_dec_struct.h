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

#ifndef __IMPEGH_HOA_DEC_STRUCT_H__
#define __IMPEGH_HOA_DEC_STRUCT_H__

typedef struct
{
  ia_hoa_config_struct *ptr_config_data;

  UWORD8 additional_info[MAX_HOA_CHANNELS][HOA_MAX_ADDITIONAL_CODERS];

  UWORD32 non_transitional_add_hoa_channels[HOA_MAX_ADDITIONAL_CODERS + 1];
  UWORD32 fade_in_add_hoa_channels[HOA_MAX_ADDITIONAL_CODERS];
  UWORD32 new_vec_channels[HOA_MAX_ADDITIONAL_CODERS];

  UWORD32 additional_value[MAX_HOA_CHANNELS][HOA_MAX_ADDITIONAL_CODERS];
  UWORD32 element_bitmask[MAX_HOA_CHANNELS][HOA_MAX_ADDITIONAL_CODERS];
  UWORD16 decoded_huffmann_word[MAX_HOA_CHANNELS][HOA_MAX_ADDITIONAL_CODERS];

  UWORD32 vvec_idx[128];
  UWORD8 sgn_val[MAX_HOA_CHANNELS][128];
  WORD32 sgn_val_size[128];
  WORD32 prev_sgn_val_size[128];
  WORD32 weight_idx[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 c_8bit_quantizer_word[MAX_HOA_CHANNELS][HOA_MAX_ADDITIONAL_CODERS];
  WORD32 v_vec_length_used;
  WORD32 prev_channel_type[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 prev_n_bits_q[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 prev_p_flag[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 prev_cb_flag[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 prev_codebk_idx[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 prev_num_vvec_indices[HOA_MAX_ADDITIONAL_CODERS];

  WORD32 active_pred[MAXIMUM_NUM_HOA_COEFF];
  WORD32 pred_dir_sig_ids[128 * 4];
  WORD32 pred_gains[128 * 4];
  WORD32 num_active_pred;
  WORD32 num_of_gains;

  UWORD32 amb_coeff_transition_state[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 amb_coeff_idx[HOA_MAX_ADDITIONAL_CODERS];

  WORD32 gain_corr_prev_amp_exp[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 coded_gain_correction_exp[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 gain_correction_exception[HOA_MAX_ADDITIONAL_CODERS];

  WORD32 ps_prediction_active;
  WORD32 kind_of_coded_pred_ids;

  WORD32 channel_type[HOA_MAX_ADDITIONAL_CODERS];
  UWORD32 active_dirs_ids[HOA_MAX_ADDITIONAL_CODERS];
  UWORD32 n_bits_q[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 codebk_idx[HOA_MAX_ADDITIONAL_CODERS];
  UWORD32 num_vvec_indices[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 p_flag[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 cb_flag[HOA_MAX_ADDITIONAL_CODERS];

  WORD32 num_of_dir_sigs;
  UWORD32 num_of_vec_sigs;
  WORD32 hoa_independency_flag;
  WORD32 dir_sig_channel_ids[HOA_MAX_ADDITIONAL_CODERS];
  WORD32 vec_sig_channel_ids[HOA_MAX_ADDITIONAL_CODERS];
} ia_hoa_frame_struct;

#endif // __IMPEGH_HOA_DEC_STRUCT_H__
