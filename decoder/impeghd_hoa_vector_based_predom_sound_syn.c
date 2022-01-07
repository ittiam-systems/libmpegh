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

#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_frame_params.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_vector_based_predom_sound_syn.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_tbe_dec.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_vector_based_predom_sound_syn_process
 *
 *  \brief Vector based predominant sound synthesis
 *
 *  \param [in,out]  dec_handle          Spatial decoder handle.
 *  \param [in]    set_of_ps_sig_indices  Signal indices set.
 *  \param [out]  ptr_predom_sound_out  Vector based predominant sound synthesis array.
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
IA_ERRORCODE
impeghd_hoa_vector_based_predom_sound_syn_process(ia_spatial_dec_str *dec_handle,
                                                  const pUWORD32 set_of_ps_sig_indices,
                                                  pFLOAT32 ptr_predom_sound_out)
{
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  ia_hoa_dec_frame_param_str *frame_param = &(dec_handle->frame_params_handle);
  ia_spatial_dec_pre_dom_sound_syn_str *sound_syn_handle =
      &(dec_handle->pre_dom_sound_syn_handle);
  ia_spatial_dec_vector_based_predom_sound_syn_str *syn_handle =
      &(sound_syn_handle->vec_based_pre_dom_sound_syn_handle);

  pFLOAT32 curr_vec;
  FLOAT32 intrpl_vec, curr_hoa_contri;
  pFLOAT32 ptr_out, ptr_in_dir_sig_frame, curr_interp_win;
  UWORD32 intrpl_samples = 0, i, vec_sig_idx, it, map_it, coeff_idx, set_it, itr, sample_idx;
  const pFLOAT32 in_dir_sig_frame = dec_handle->dyn_corr_sample_buf;
  const pUWORD32 amb_coef_indices_to_disable = frame_param->amb_coeff_indices_to_disable;
  const pUWORD32 amb_coef_indices_to_enable = frame_param->amb_coeff_indices_to_enable;
  const ia_hoa_vec_sig_str *vectors = frame_param->vectors;

  ia_core_coder_memset(ptr_predom_sound_out,
                       pstr_hoa_config->frame_length * pstr_hoa_config->num_coeffs);

  for (i = 0; i < syn_handle->num_prev_vec; i++)
  {
    curr_vec = syn_handle->map_prev_vec[i].sig_id;
    vec_sig_idx = syn_handle->map_prev_vec[i].index;

    for (it = 0; it < HOA_MAXIMUM_SIG_INDICES; it++)
    {
      if (set_of_ps_sig_indices[it] == vec_sig_idx)
        break;
    }

    if (it != HOA_MAXIMUM_SIG_INDICES)
    {
      intrpl_samples = pstr_hoa_config->frame_length;
      curr_interp_win = &(sound_syn_handle->fade_out_win_for_dir_sigs[0]);
      for (map_it = 0; map_it < HOA_MAXIMUM_VECTOR_SIG_INDICES; map_it++)
      {
        if (vectors[map_it].index == vec_sig_idx)
          break;
      }
      if (map_it != HOA_MAXIMUM_VECTOR_SIG_INDICES)
      {
        intrpl_samples = pstr_hoa_config->spat_interpolation_time;
        curr_interp_win = &(sound_syn_handle->fade_out_win_for_vec_sigs[0]);
      }
      ptr_in_dir_sig_frame =
          (pFLOAT32)(in_dir_sig_frame + (vec_sig_idx - 1) * pstr_hoa_config->frame_length);
      for (coeff_idx = dec_handle->vec_start; coeff_idx < pstr_hoa_config->num_coeffs;
           coeff_idx++)
      {
        ptr_out = ptr_predom_sound_out + (coeff_idx) * (pstr_hoa_config->frame_length);
        for (sample_idx = 0; sample_idx < intrpl_samples; sample_idx++)
        {
          intrpl_vec = curr_vec[coeff_idx - dec_handle->vec_start] * curr_interp_win[sample_idx];
          curr_hoa_contri = intrpl_vec * (ptr_in_dir_sig_frame[sample_idx]);
          ptr_out[sample_idx] += curr_hoa_contri;
        }
      }
    }
  }

  for (map_it = 0; map_it < frame_param->num_vec_predom_sounds; map_it++)
  {
    curr_interp_win = &(dec_handle->ones_long_vec[0]);
    curr_vec = (pFLOAT32) & (vectors[map_it].sig_id[0]);
    vec_sig_idx = vectors[map_it].index;

    for (set_it = 0; set_it < HOA_MAXIMUM_SIG_INDICES; set_it++)
    {
      if (syn_handle->prev_ps_sig_indices_set[set_it] == vec_sig_idx)
        break;
    }
    if (set_it != HOA_MAXIMUM_SIG_INDICES)
    {
      curr_interp_win = &(sound_syn_handle->fade_in_win_for_vec_sigs[0]);
    }

    ptr_in_dir_sig_frame =
        (pFLOAT32)(in_dir_sig_frame + (vec_sig_idx - 1) * pstr_hoa_config->frame_length);
    for (coeff_idx = dec_handle->vec_start; coeff_idx < pstr_hoa_config->num_coeffs; coeff_idx++)
    {
      ptr_out = ptr_predom_sound_out + (coeff_idx) * (pstr_hoa_config->frame_length);
      for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
      {
        intrpl_vec = curr_vec[coeff_idx - dec_handle->vec_start] * curr_interp_win[sample_idx];
        curr_hoa_contri = intrpl_vec * (ptr_in_dir_sig_frame[sample_idx]);
        ptr_out[sample_idx] += curr_hoa_contri;
      }
    }
  }

  if (1 != pstr_hoa_config->coded_v_vec_length)
  {
    for (map_it = 0; map_it < frame_param->num_vec_predom_sounds; map_it++)
    {
      syn_handle->map_prev_vec[map_it].index = vectors[map_it].index;
      ia_core_coder_mem_cpy(&vectors[map_it].sig_id[0],
                            &syn_handle->map_prev_vec[map_it].sig_id[0], MAXIMUM_NUM_HOA_COEFF);
    }
    syn_handle->num_prev_vec = frame_param->num_vec_predom_sounds;
  }
  else
  {
    for (set_it = 0; set_it < frame_param->num_enable_coeff; set_it++)
    {
      coeff_idx = amb_coef_indices_to_enable[set_it] - 1;
      ptr_out = ptr_predom_sound_out + coeff_idx * (pstr_hoa_config->frame_length);
      const pFLOAT32 intrpl_fade_out_win_for_dir_sigs =
          sound_syn_handle->fade_out_win_for_dir_sigs;
      for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
      {
        ptr_out[sample_idx] =
            ia_mul_flt(ptr_out[sample_idx], intrpl_fade_out_win_for_dir_sigs[sample_idx]);
      }
    }
    for (set_it = 0; set_it < frame_param->num_disable_coeff; set_it++)
    {
      coeff_idx = amb_coef_indices_to_disable[set_it] - 1;
      ptr_out = ptr_predom_sound_out + coeff_idx * (pstr_hoa_config->frame_length);
      const pFLOAT32 intrpl_fade_in_win_for_dir_sigs = sound_syn_handle->fade_in_win_for_dir_sigs;
      for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
      {
        ptr_out[sample_idx] =
            ia_mul_flt(ptr_out[sample_idx], intrpl_fade_in_win_for_dir_sigs[sample_idx]);
      }
    }

    for (map_it = 0; map_it < frame_param->num_vec_predom_sounds; map_it++)
    {
      vec_sig_idx = vectors[map_it].index;

      syn_handle->map_prev_vec[map_it].index = vec_sig_idx;
      ia_core_coder_mem_cpy(&vectors[map_it].sig_id[0],
                            &syn_handle->map_prev_vec[map_it].sig_id[0], MAXIMUM_NUM_HOA_COEFF);

      for (itr = 0; itr < frame_param->num_enable_coeff; itr++)
      {
        coeff_idx = amb_coef_indices_to_enable[itr] - 1;
        syn_handle->map_prev_vec[map_it].sig_id[coeff_idx - dec_handle->vec_start] = 0;
      }
    }
    syn_handle->num_prev_vec = frame_param->num_vec_predom_sounds;
  }

  memcpy(&syn_handle->prev_ps_sig_indices_set[0], &set_of_ps_sig_indices[0],
         sizeof(syn_handle->prev_ps_sig_indices_set[0]) * HOA_MAXIMUM_SIG_INDICES);

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAProc */