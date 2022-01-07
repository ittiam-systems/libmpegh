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
#include <stdio.h>
#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_nfc_filtering.h"

#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_hoa_config.h"
#include "impeghd_hoa_renderer.h"

#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_frame.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_spatial_decoder.h"
#include "impeghd_hoa_decoder.h"

#include "impeghd_tbe_dec.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_dec_init
 *
 *  \brief HOA decoder initialization
 *
 *  \param [in,out] dec_handle          Pointer to HOA decoder handle
 *  \param [in]     spk_idx             Speaker index
 *  \param [in]     samp_freq           Sampling frequency
 *  \param [in]     scratch             Pointer to scratch buffer
 *
 *  \return IA_ERRORCODE                Error code if any
 *
 */
IA_ERRORCODE impeghd_hoa_dec_init(impeghd_hoa_dec_struct *dec_handle, WORD32 spk_idx,
                                  ia_speaker_config_3d *ref_spk_layout, UWORD32 samp_freq,
                                  pVOID scratch, UWORD32 mpegh_profile_lvl)
{
  ia_hoa_config_struct *ia_hoa_config = dec_handle->ia_hoa_config;
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  dec_handle->ptr_scratch = scratch;
  impeghd_hoa_spatial_init(&dec_handle->spat_dec_handle, ia_hoa_config, dec_handle->ptr_scratch);
  {
    err = impeghd_hoa_ren_renderer_init(&(dec_handle->hoa_renderer), spk_idx, ref_spk_layout,
                                        &ia_hoa_config->order, MAX_NUM_HOA_ORDERS, 1,
                                        dec_handle->ptr_scratch);
    if (err)
    {
      return err;
    }
    err = impeghd_hoa_ren_input_init(
        &(dec_handle->hoa_renderer), ref_spk_layout, ia_hoa_config->order, (pVOID)ia_hoa_config,
        ia_hoa_config->matrix_present, (FLOAT32)ia_hoa_config->nfc_ref_distance, samp_freq,
        mpegh_profile_lvl);
    if (err)
    {
      return err;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_dec_decode
 *
 *  \brief HOA decode processing
 *
 *  \param [in,out] dec_handle      Pointer to HOA decoder handle
 *  \param [out]    ptr_out_buf     Pointer to output buffer
 *  \param [in]     ptr_in_buf      Pointer to input buffer
 *  \param [in]     ptr_prev_in_buf Pointer to previous input buffer
 *  \param [in]     num_out_ch      Number of output channels
 *  \param [in]     delay_flag      Delay flag
 *
 *  \return IA_ERRORCODE                  Error
 *
 */
IA_ERRORCODE impeghd_hoa_dec_decode(impeghd_hoa_dec_struct *dec_handle, pFLOAT32 ptr_out_buf,
                                    pFLOAT32 ptr_in_buf, pFLOAT32 ptr_prev_in_buf,
                                    pWORD32 num_out_ch, WORD32 delay_flag)
{
  ia_bit_buf_struct *ptr_bit_buf = dec_handle->ia_bit_buf;
  FLOAT32 *ptr_hoa_array = NULL;
  UWORD32 ch, rem, scratch_idx = 0;
  UWORD32 hoa_coeffs = dec_handle->ia_hoa_config->num_coeffs;
  UWORD32 frame_length = dec_handle->frame_length;
  pWORD8 buf = (pWORD8)dec_handle->ptr_scratch;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;

  if (dec_handle->is_brir_rendering)
  {
    *num_out_ch = dec_handle->ia_hoa_config->num_coeffs;
  }
  else
  {
    *num_out_ch = dec_handle->hoa_renderer.num_out_channels;
  }

  /* Memory for ptr_out_buf is same as scratch, so keeping aside the space required for
   * ptr_out_buf */
  scratch_idx += sizeof(FLOAT32) * (*num_out_ch) * (frame_length);

  rem = (ptr_bit_buf->size - ptr_bit_buf->cnt_bits);
  if (rem % 8)
  {
    UWORD32 fill_bits = (8 - (rem % 8));
    ia_core_coder_skip_bits_buf(ptr_bit_buf, fill_bits);
  }
  if (ptr_bit_buf->size == 0)
  {
    *num_out_ch = 0;
  }
  else
  {
    err_code = impeghd_hoa_frame(ptr_bit_buf, dec_handle->ia_hoa_frame);
    if (err_code)
    {
      return err_code;
    }
  }

  if (dec_handle->is_brir_rendering)
  {
    /* If BRIR rendering is active, HOA rendering will be skipped */
    /* So, HOA signals will be copied to out buffer               */
    ptr_hoa_array = ptr_out_buf;
  }
  else
  {
    ptr_hoa_array = (pFLOAT32)(buf + scratch_idx);
    scratch_idx += (sizeof(*ptr_hoa_array) * hoa_coeffs * frame_length);
  }

  if (1 != delay_flag)
  {
    if (impeghd_hoa_spatial_process(&dec_handle->spat_dec_handle, ptr_in_buf, ptr_hoa_array,
                                    (pVOID)(buf + scratch_idx), dec_handle->ia_hoa_frame))
    {
      return IA_MPEGH_HOA_EXE_FATAL_SPATIAL_PROCESS_FAILED;
    }
  }
  else
  {
    for (ch = 0; ch < dec_handle->ia_hoa_config->num_transport_ch; ch++)
    {
      ia_core_coder_mem_cpy(&ptr_in_buf[ch * frame_length],
                            &ptr_prev_in_buf[ch * frame_length + 768], 256);
    }
    if (impeghd_hoa_spatial_process(&dec_handle->spat_dec_handle, ptr_prev_in_buf, ptr_hoa_array,
                                    (pVOID)(buf + scratch_idx), dec_handle->ia_hoa_frame))
    {
      return IA_MPEGH_HOA_EXE_FATAL_SPATIAL_PROCESS_FAILED;
    }
    for (ch = 0; ch < dec_handle->ia_hoa_config->num_transport_ch; ch++)
    {
      ia_core_coder_mem_cpy(&ptr_in_buf[ch * frame_length + 256],
                            &ptr_prev_in_buf[ch * frame_length], (frame_length - 256));
    }
  }

  if (dec_handle->spat_delay_frames)
  {
    dec_handle->spat_delay_frames--;
  }
  else if (0 == dec_handle->is_brir_rendering)
  {
    IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
    WORD32 num_rows = hoa_coeffs;
    err = impeghd_hoa_ren_block_process(&(dec_handle->hoa_renderer), ptr_hoa_array, num_rows,
                                        frame_length, 1, (pVOID)(buf + scratch_idx), ptr_out_buf);
    if (err)
    {
      return err;
    }
  }

  return err_code;
}
/** @} */ /* End of HOAProc */