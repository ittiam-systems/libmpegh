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
#include "impeghd_hoa_ch_reassignment.h"
#include "impeghd_tbe_dec.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_ch_reassignment_process
 *
 *  \brief HOA channel reassignment processing
 *
 *  \param [in,out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE              Error
 *
 */
IA_ERRORCODE impeghd_hoa_ch_reassignment_process(ia_spatial_dec_str *dec_handle)
{
  ia_hoa_dec_frame_param_str *ptr_frame_param = &(dec_handle->frame_params_handle);
  const ia_hoa_vec_sig_str *vectors = ptr_frame_param->vectors;
  const ia_hoa_dir_id_str *ptr_active_and_grid_dir_indices =
      ptr_frame_param->active_and_grid_dir_indices;
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  pFLOAT32 ptr_in_all_decoded_sigs = dec_handle->dyn_corr_sample_buf;
  pFLOAT32 ptr_in_all_decoded_sigs_tmp;
  UWORD32 loop_cnt, curr_ch_sig;
  WORD32 idx_to_clear[HOA_MAXIMUM_NUM_PERC_CODERS];
  memset(idx_to_clear, -1, sizeof(idx_to_clear));

  for (loop_cnt = 0; loop_cnt < ptr_frame_param->num_vec_predom_sounds; loop_cnt++)
  {
    curr_ch_sig = vectors[loop_cnt].index;
    idx_to_clear[curr_ch_sig - 1] = 0;
  }

  for (loop_cnt = 0; loop_cnt < ptr_frame_param->num_dir_predom_sounds; loop_cnt++)
  {
    curr_ch_sig = ptr_active_and_grid_dir_indices[loop_cnt].ch_idx;
    idx_to_clear[curr_ch_sig - 1] = 0;
  }

  for (loop_cnt = 0; loop_cnt < pstr_hoa_config->num_addnl_coders; loop_cnt++)
  {
    if (idx_to_clear[loop_cnt] != 0)
    {
      ptr_in_all_decoded_sigs_tmp =
          (ptr_in_all_decoded_sigs + (loop_cnt * pstr_hoa_config->frame_length));
      ia_core_coder_memset(ptr_in_all_decoded_sigs_tmp, pstr_hoa_config->frame_length);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAProc */