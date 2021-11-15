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

#ifndef __IA_MPEGH_HOA_FRAME_PARAMS_H__
#define __IA_MPEGH_HOA_FRAME_PARAMS_H__

typedef struct
{
  ia_hoa_dir_id_str active_and_grid_dir_indices[MAXIMUM_NUM_HOA_COEFF];

  ia_hoa_vec_sig_str vectors[MAX_HOA_CHANNELS];

  WORD32 *ptr_is_exception;
  WORD32 exponents[HOA_MAXIMUM_NUM_PERC_CODERS];
  WORD32 quant_pred_fact_mat[HOA_MAX_PRED_DIR_SIGNALS * MAXIMUM_NUM_HOA_COEFF];

  UWORD32 amb_coeff_indices_to_enable[HOA_MAXIMUM_SET_SIZE];
  UWORD32 amb_coeff_indices_to_disable[HOA_MAXIMUM_SET_SIZE];
  UWORD32 non_en_dis_able_act_hoa_coeff_indices[HOA_MAXIMUM_SET_SIZE];
  UWORD32 num_dir_predom_sounds;
  UWORD32 num_vec_predom_sounds;
  UWORD32 num_enable_coeff;
  UWORD32 num_disable_coeff;
  UWORD32 num_en_dis_able_coeff;

  UWORD32 amb_hoa_assign[HOA_MAXIMUM_NUM_PERC_CODERS];

  UWORD32 prediction_type_vec[MAXIMUM_NUM_HOA_COEFF];
  UWORD32 prediction_indices_mat[HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED * MAXIMUM_NUM_HOA_COEFF];

} ia_hoa_dec_frame_param_str;

#endif //__IA_MPEGH_HOA_FRAME_PARAMS_H__