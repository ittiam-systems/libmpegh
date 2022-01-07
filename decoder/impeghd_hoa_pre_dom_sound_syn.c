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
#include "impeghd_hoa_dir_based_pre_dom_sound_syn.h"
#include "impeghd_hoa_pre_dom_sound_syn.h"
#include "impeghd_hoa_vector_based_predom_sound_syn.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_pre_dom_sound_syn_process
 *
 *  \brief Predominant sound synthesis
 *
 *  \param [in,out]  dec_handle      Spatial decoder handle
 *  \param [out]  out_pre_dom_sounds  Pre dominant sound array
 *
 *  \return IA_ERRORCODE  Error
 *
 */

IA_ERRORCODE impeghd_hoa_pre_dom_sound_syn_process(ia_spatial_dec_str *dec_handle,
                                                   pFLOAT32 out_pre_dom_sounds)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  ia_hoa_dec_frame_param_str *frame_param = &(dec_handle->frame_params_handle);
  const ia_hoa_vec_sig_str *vectors = frame_param->vectors;
  const ia_hoa_dir_id_str *active_and_grid_dir_indices = frame_param->active_and_grid_dir_indices;

  UWORD32 sig_indices_set[MAX_HOA_CHANNELS] = {0};
  UWORD32 num_sounds = 0;

  for (num_sounds = 0; num_sounds < frame_param->num_dir_predom_sounds; num_sounds++)
  {
    sig_indices_set[num_sounds] = active_and_grid_dir_indices[num_sounds].ch_idx;
  }

  for (num_sounds = 0; num_sounds < frame_param->num_vec_predom_sounds; num_sounds++)
  {
    sig_indices_set[num_sounds] = vectors[num_sounds].index;
  }

  err_code = impeghd_hoa_vector_based_predom_sound_syn_process(dec_handle, sig_indices_set,
                                                               out_pre_dom_sounds);
  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_hoa_dir_based_pre_dom_sound_syn_process(dec_handle, sig_indices_set,
                                                             out_pre_dom_sounds);
  if (err_code)
  {
    return err_code;
  }
  return err_code;
}
/** @} */ /* End of HOAProc */