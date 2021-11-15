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

#include <math.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_hoa_rom.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_amb_syn.h"
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
 *  impeghd_hoa_ambience_synthesis_process
 *
 *  \brief HOA ambience synthesis processing
 *
 *  \param [in/out] dec_handle      Pointer to Spatial decoder handle
 *  \param [in]     reassign_vec    Reassign vector pointer
 *
 *  \return IA_ERRORCODE                  Error
 *
 */
IA_ERRORCODE impeghd_hoa_ambience_synthesis_process(ia_spatial_dec_str *dec_handle,
                                                    pWORD32 reassign_vec)
{
  ia_spatial_dec_amb_syn_str *syn_handle = &(dec_handle->amb_syn_handle);
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  UWORD32 num_coeffs = pstr_hoa_config->num_coeffs, frame_length = pstr_hoa_config->frame_length;
  UWORD32 coeff;
  UWORD32 low_order_mat_sz = syn_handle->low_order_mat_sz;
  FLOAT32 **ptr_amb_in;
  FLOAT32 *ptr_zeros_array;
  FLOAT32 *ptr_amb_syn_out = &dec_handle->amb_hoa_coeffs_frame_buf[0];
  ptr_amb_syn_out += (low_order_mat_sz * frame_length);

  if (dec_handle->scratch == NULL)
  {
    return IA_MPEGH_HOA_EXE_FATAL_SCRATCH_NULL;
  }

  ptr_amb_in = (FLOAT32 **)(dec_handle->scratch);
  ptr_zeros_array =
      (FLOAT32 *)((WORD8 *)dec_handle->scratch + sizeof(ptr_amb_in[0]) * num_coeffs);
  ia_core_coder_memset(ptr_zeros_array, frame_length);
  for (coeff = 0; coeff < num_coeffs; coeff++)
  {
    if (reassign_vec[coeff] < 0)
    {
      ptr_amb_in[coeff] = ptr_zeros_array;
    }
    else
    {
      ptr_amb_in[coeff] =
          (FLOAT32 *)&dec_handle->dyn_corr_sample_buf[reassign_vec[coeff] * frame_length];
    }
  }

  for (coeff = low_order_mat_sz; coeff < num_coeffs; coeff++)
  {
    if (-1 == reassign_vec[coeff])
    {
      ia_core_coder_memset(ptr_amb_syn_out, frame_length);
    }
    else
    {
      ia_core_coder_mem_cpy(ptr_amb_in[coeff], ptr_amb_syn_out, frame_length);
    }
    ptr_amb_syn_out += frame_length;
  }

  if (!(dec_handle->use_phase_shift_decorr))
  {
    FLOAT32 *ptr_order_mtrx = (FLOAT32 *)(&syn_handle->order_matrix[0]);
    UWORD32 mat_sz, length;

    ptr_amb_syn_out = (FLOAT32 *)(&dec_handle->amb_hoa_coeffs_frame_buf[0]);
    for (coeff = 0; coeff < low_order_mat_sz; coeff++)
    {
      for (length = 0; length < frame_length; length++)
      {
        FLOAT32 acc = 0.0f;
        mat_sz = 0;
        while (mat_sz < low_order_mat_sz)
        {
          acc = ia_mac_flt(acc, ptr_order_mtrx[mat_sz], ptr_amb_in[mat_sz][length]);
          mat_sz++;
        }
        ptr_amb_syn_out[coeff * frame_length + length] = acc;
      }
      ptr_order_mtrx += (low_order_mat_sz);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAProc */