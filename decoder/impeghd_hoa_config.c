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
#include <string.h>
#include <stdio.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_hoa_rom.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_config.h"

/**
 * @defgroup HOAConfParse HOA config data parsing
 * @ingroup  HOAConfParse
 * @brief HOA config data parsing
 *
 * @{
 */

/**
 *  impeghd_hoa_v_vec_len_and_coeff_calc
 *
 *  \brief Calculation of HOA vector length and coefficients
 *
 *  \param [in/out] ptr_config_data     Pointer to HOA config structure
 *
 *  \return VOID
 *
 */
static VOID impeghd_hoa_v_vec_len_and_coeff_calc(ia_hoa_config_struct *ptr_config_data)
{
  if (ptr_config_data->coded_v_vec_length == 1 || ptr_config_data->coded_v_vec_length == 2)
  {
    ptr_config_data->idx_offset =
        (ptr_config_data->min_amb_order + 1) * (ptr_config_data->min_amb_order + 1) + 1;
  }
  else
  {
    ptr_config_data->idx_offset = 1;
  }
  ptr_config_data->v_vec_length_used =
      ptr_config_data->num_coeffs - ptr_config_data->idx_offset + 1;
  ptr_config_data->index_length = impeghd_hoa_get_ceil_log2(ptr_config_data->num_coeffs);
}

/**
 *  impeghd_fill_config
 *
 *  \brief Fill HOA config details
 *
 *  \param [in/out] ptr_bit_buf     Pointer to bit buffer structure
 *  \param [out]    ptr_config_data Pointer to HOA config structure
 *
 *  \return IA_ERRORCODE                  Error code if any
 *
 */
IA_ERRORCODE impeghd_fill_config(ia_bit_buf_struct *ptr_bit_buf,
                                 ia_hoa_config_struct *ptr_config_data)
{
  WORD32 max, val, i = 1, offset;
  val = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
  if (HOA_ESC_VAL_BITS_3 == val)
  {
    val += ia_core_coder_read_bits_buf(ptr_bit_buf, 5);
  }

  ptr_config_data->order = val;
  if (val > MAXIMUM_HOA_ORDER)
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_HOA_ORDER;
  }
  ptr_config_data->num_coeffs = (ptr_config_data->order + 1) * (ptr_config_data->order + 1);

  ptr_config_data->is_screen_relative = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  ptr_config_data->uses_nfc = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (!(ptr_config_data->uses_nfc))
  {
    ptr_config_data->nfc_ref_distance = 0;
  }
  else
  {
    FLOAT32 ref;
    pWORD8 temp = (pWORD8)&ref;

    temp[0] = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    temp[1] = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    temp[2] = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    temp[3] = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    ptr_config_data->nfc_ref_distance = ref;
  }

  val = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
  if (HOA_ESC_VAL_BITS_3 == val)
  {
    val += ia_core_coder_read_bits_buf(ptr_bit_buf, 5);
  }
  ptr_config_data->min_amb_order = val - 1;

  /*shall be smaller than or equal to min( floor(sqrt(numHOATransportChannels)−1), HoaOrder)*/

  if (ptr_config_data->min_amb_order >
      ia_min_int(floor(sqrt(ptr_config_data->num_transport_ch) - 1), ptr_config_data->order))
  {
    return IA_MPEGH_HOA_INIT_FATAL_INVALID_MIN_AMB_ORDER;
  }

  ptr_config_data->min_coeffs_for_amb =
      (ptr_config_data->min_amb_order + 1) * (ptr_config_data->min_amb_order + 1);

  offset = ia_max_int(
      0, (WORD32)ptr_config_data->num_transport_ch - ptr_config_data->min_coeffs_for_amb);
  ptr_config_data->num_layers = 1;
  ptr_config_data->num_addnl_coders = offset;

  ptr_config_data->is_single_layer = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  ptr_config_data->coded_spat_interpolation_time = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
  ptr_config_data->spat_interpolation_method = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  ptr_config_data->coded_v_vec_length = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
  /*shall NOT be 3*/
  if (ptr_config_data->coded_v_vec_length >= 3)
  {
    return IA_MPEGH_HOA_INIT_FATAL_INVALID_VEC_LENGTH;
  }

  ptr_config_data->max_gain_corr_amp_exp = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
  ptr_config_data->frame_length_indicator = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);

  if (ptr_config_data->frame_length_indicator >= 3)
  {
    return IA_MPEGH_HOA_INIT_FATAL_INVALID_FRAME_LENGTH_INDICATOR;
  }
  else
  {
    if (ptr_config_data->core_coder_frame_length == 1024)
    {
      ptr_config_data->frame_length = MAXIMUM_FRAME_SIZE;
    }
    else
    {
      return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_FRAME_SIZE;
    }
  }

  if (ptr_config_data->frame_length == 1024)
  {
    ptr_config_data->spat_interpolation_time =
        ia_hoa_spat_interp_time_code_table[ptr_config_data->coded_spat_interpolation_time];
  }
  else
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_FRAME_SIZE;
  }

  if (ptr_config_data->min_amb_order >= ptr_config_data->order)
  {
    ptr_config_data->max_order_to_be_transmitted = ptr_config_data->order;
  }
  else
  {
    ptr_config_data->diff_order_bits =
        impeghd_hoa_get_ceil_log2((ptr_config_data->order - ptr_config_data->min_amb_order + 1));
    ptr_config_data->diff_order =
        ia_core_coder_read_bits_buf(ptr_bit_buf, ptr_config_data->diff_order_bits);

    ptr_config_data->max_order_to_be_transmitted =
        ptr_config_data->diff_order + ptr_config_data->min_amb_order;
  }

  ptr_config_data->max_num_of_coeffs_to_be_transmitted =
      (ptr_config_data->max_order_to_be_transmitted + 1) *
      (ptr_config_data->max_order_to_be_transmitted + 1);
  ptr_config_data->max_num_add_active_amb_coeffs =
      (ptr_config_data->max_num_of_coeffs_to_be_transmitted -
       ptr_config_data->min_coeffs_for_amb);
  if (ptr_config_data->max_num_add_active_amb_coeffs < 0)
    return IA_MPEGH_HOA_INIT_FATAL_RENDER_MATRIX_INIT_FAILED;

  ptr_config_data->vq_conf_bits =
      impeghd_hoa_get_ceil_log2(impeghd_hoa_get_ceil_log2((ptr_config_data->num_coeffs + 1)));
  ptr_config_data->num_v_vec_vq_elements_bits =
      ia_core_coder_read_bits_buf(ptr_bit_buf, ptr_config_data->vq_conf_bits);

  /*restricted to values [0..7]*/
  if (ptr_config_data->num_v_vec_vq_elements_bits > 7)
  {
    return IA_MPEGH_HOA_INIT_FATAL_INVALID_NUM_VQ_ELE_BITS;
  }

  if (1 == ptr_config_data->min_amb_order)
  {
    ptr_config_data->use_phase_shift_decorr = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }

  val = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
  ptr_config_data->pred_dir_sigs = val + 1;

  val = ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
  ptr_config_data->num_bits_per_sf = val + 1;

  ptr_config_data->pred_subbands_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);

  if (ptr_config_data->pred_subbands_idx > 0)
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_SUBBAND_CFG;
  }

  ptr_config_data->par_subband_table_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
  if (ptr_config_data->par_subband_table_idx > 0)
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_SUBBAND_CFG;
  }

  ptr_config_data->amb_asign_m_bits =
      impeghd_hoa_get_ceil_log2(ptr_config_data->max_num_add_active_amb_coeffs);
  ptr_config_data->active_pred_ids_bits = impeghd_hoa_get_ceil_log2(ptr_config_data->num_coeffs);

  while ((WORD32)(i * ptr_config_data->active_pred_ids_bits + impeghd_hoa_get_ceil_log2(i)) <
         (WORD32)ptr_config_data->num_coeffs)
  {
    i++;
  }
  max = (1 > (i - 1)) ? 1 : (i - 1);
  ptr_config_data->num_active_pred_ids_bits = impeghd_hoa_get_ceil_log2(max);

  ptr_config_data->gain_corr_prev_amp_exp_bits = impeghd_hoa_get_ceil_log2(
      impeghd_hoa_get_ceil_log2((WORD32)(1.5f * ptr_config_data->num_coeffs)) +
      ptr_config_data->max_gain_corr_amp_exp + 1);

  impeghd_hoa_v_vec_len_and_coeff_calc(ptr_config_data);
  return IA_MPEGH_DEC_NO_ERROR;
}

/** @} */ /* End of HOAConfParse */