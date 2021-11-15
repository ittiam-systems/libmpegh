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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <impeghd_type_def.h>

#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_dec.h"
#include "ia_core_coder_api_defs.h"
#include "ia_core_coder_apicmd_standards.h"

#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_definitions.h"
#include "ia_core_coder_env_extr.h"

#include "impeghd_memory_standards.h"

#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"

#include "impeghd_binaural.h"
#include "impeghd_binaural_filter_design.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_ren_interface_utils.h"

/**
 * @defgroup BinauralParsing Binaural render payload parsing
 * @ingroup  BinauralParsing
 * @brief Binaural render payload parsing
 *
 * @{
 */

/**
*  impeghd_binaural_read_fir_data
*
*  \brief Read FIR data from bitstream
*
*  \param [in]  buf_handle               Pointer to input configuration structure
*  \param [in]  ia_binaural_fir_data_str Pointer to brir fir data structure
*  \param [in]  brir_pairs               brir pairs
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_binaural_read_fir_data(
    pVOID buf_handle, ia_binaural_fir_data_str *ia_binaural_fir_data_str, WORD16 brir_pairs)
{
  WORD32 pair, tap;
  WORD32 data;
  FLOAT32 *ptr_data;
  ia_bit_buf_struct *ia_bit_buf = (ia_bit_buf_struct *)buf_handle;
  ia_binaural_fir_data_str->n_taps = ia_core_coder_read_bits_buf(ia_bit_buf, 24);

  if (ia_binaural_fir_data_str->n_taps > MAX_BRIR_SIZE)
  {
    return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_TAPS;
  }
  if (brir_pairs > MAX_NUM_BRIR_PAIRS)
  {
    return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_CHANNELS;
  }

  for (pair = 0; pair < brir_pairs; pair++)
  {
    for (tap = 0; tap < ia_binaural_fir_data_str->n_taps; tap++)
    {
      data = ia_core_coder_read_bits_buf_32(ia_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      ia_binaural_fir_data_str->taps[0][pair][tap] = *(ptr_data);
      data = ia_core_coder_read_bits_buf_32(ia_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      ia_binaural_fir_data_str->taps[1][pair][tap] = *(ptr_data);
    }
  }

  ia_binaural_fir_data_str->all_cut_freq = ia_core_coder_read_bits_buf_32(ia_bit_buf);

  if (ia_binaural_fir_data_str->all_cut_freq != 0)
  {
    for (pair = 0; pair < brir_pairs; pair++)
    {
      ia_binaural_fir_data_str->cut_freq[0][pair] =
          (FLOAT32)ia_binaural_fir_data_str->all_cut_freq;
      ia_binaural_fir_data_str->cut_freq[1][pair] =
          (FLOAT32)ia_binaural_fir_data_str->all_cut_freq;
    }
  }
  else
  {
    for (pair = 0; pair < brir_pairs; pair++)
    {
      data = ia_core_coder_read_bits_buf_32(ia_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      ia_binaural_fir_data_str->cut_freq[0][pair] = *(ptr_data);

      data = ia_core_coder_read_bits_buf_32(ia_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      ia_binaural_fir_data_str->cut_freq[1][pair] = *(ptr_data);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_binaural_read_td_data
*
*  \brief Read td binaural renderer params from bit stream
*
*  \param [in]  ptr_bit_buf				  Pointer to bit buffer
*  \param [in]  pstr_binaural_rep   Pointer to binaural representation structure
* description
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE
impeghd_binaural_read_td_data(ia_bit_buf_struct *ptr_bit_buf,
                              ia_binaural_representation_str *pstr_binaural_rep)
{
  WORD32 pair, blk, tap, data;
  FLOAT32 *ptr_data;
  ia_td_binaural_ren_param_str *pstr_binaural_td_data = &pstr_binaural_rep->td_binaural_ren_param;
  pstr_binaural_td_data->begin_delay = ia_core_coder_read_bits_buf(ptr_bit_buf, 16);
  pstr_binaural_td_data->len_direct = ia_core_coder_read_bits_buf(ptr_bit_buf, 16);
  pstr_binaural_td_data->num_diffuse_block = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
  pstr_binaural_td_data->num_channel = pstr_binaural_rep->brir_pairs;

  if (pstr_binaural_rep->brir_pairs > MAX_NUM_BRIR_PAIRS)
  {
    return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_CHANNELS;
  }

  for (pair = 0; pair < pstr_binaural_rep->brir_pairs; pair++)
  {
    data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
    ptr_data = (FLOAT32 *)&data;
    pstr_binaural_td_data->ptr_direct_fc[0][pair] = *(ptr_data);
    data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
    ptr_data = (FLOAT32 *)&data;
    pstr_binaural_td_data->ptr_direct_fc[1][pair] = *(ptr_data);
  }

  for (blk = 0; blk < pstr_binaural_td_data->num_diffuse_block; blk++)
  {
    data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
    ptr_data = (FLOAT32 *)&data;
    pstr_binaural_td_data->ptr_diffuse_fc[0][blk] = *(ptr_data);
    data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
    ptr_data = (FLOAT32 *)&data;
    pstr_binaural_td_data->ptr_diffuse_fc[1][blk] = *(ptr_data);
  }
  for (pair = 0; pair < pstr_binaural_rep->brir_pairs; pair++)
  {
    data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
    ptr_data = (FLOAT32 *)&data;
    pstr_binaural_td_data->ptr_inv_diffuse_weight[pair] = *(ptr_data);
  }

  for (pair = 0; pair < pstr_binaural_rep->brir_pairs; pair++)
  {
    for (tap = 0; tap < pstr_binaural_td_data->len_direct; tap++)
    {
      data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      pstr_binaural_td_data->ptr_taps_direct[0][pair][tap] = *(ptr_data);
      data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      pstr_binaural_td_data->ptr_taps_direct[1][pair][tap] = *(ptr_data);
    }
  }

  for (blk = 0; blk < pstr_binaural_td_data->num_diffuse_block; blk++)
  {
    for (tap = 0; tap < pstr_binaural_td_data->len_direct; tap++)
    {
      data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      pstr_binaural_td_data->ptr_taps_diffuse[blk][0][tap] = *(ptr_data);
      data = ia_core_coder_read_bits_buf_32(ptr_bit_buf);
      ptr_data = (FLOAT32 *)&data;
      pstr_binaural_td_data->ptr_taps_diffuse[blk][1][tap] = *(ptr_data);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_parse_brir_info
*
*  \brief Parse BRIR info from input bitstream configuration and updates in brir renderer
* structure
*
*  \param [in]  buf_handle      Pointer to input configuration  structure
*  \param [in]  binaural_ren_info  Pointer to brir renderer structure
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_parse_brir_info(VOID *buf_handle,
                                            ia_binaural_renderer *binaural_ren_info)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 hoa_order_binaural = 0, brir_pairs = 0;
  UWORD32 fill_bits, rem;
  WORD8 filename_length = 0, i = 0, brir_sampling_freq_index = 0;
  ia_bit_buf_struct *ia_bit_buf = (ia_bit_buf_struct *)buf_handle;
  ia_binaural_representation_str *ptr_binaural_rep = &binaural_ren_info->binaural_rep;

  ia_core_coder_read_bits_buf_32(ia_bit_buf);
  ia_core_coder_read_bits_buf(ia_bit_buf, 8);
  filename_length = (WORD8)ia_core_coder_read_bits_buf(ia_bit_buf, 8);

  for (i = 0; i < filename_length; i++)
  {
    ia_core_coder_read_bits_buf(ia_bit_buf, 8);
  }

  ia_core_coder_read_bits_buf(ia_bit_buf, 1);
  binaural_ren_info->num_binaural_representation =
      (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 4);

  ptr_binaural_rep->brir_sampling_frequency = 0;
  ptr_binaural_rep->is_hoa_data = 0;
  ptr_binaural_rep->hoa_order_binaural = 0;
  ptr_binaural_rep->brir_pairs = 0;
  ptr_binaural_rep->binaural_data_format_id = 0;

  for (i = 0; i < binaural_ren_info->num_binaural_representation; i++)
  {
    brir_sampling_freq_index = (WORD8)ia_core_coder_read_bits_buf(ia_bit_buf, 5);
    if (brir_sampling_freq_index != 0x1F)
    {
      ptr_binaural_rep->brir_sampling_frequency =
          brir_sampling_frequency_table[brir_sampling_freq_index];
    }
    else
    {
      ptr_binaural_rep->brir_sampling_frequency = ia_core_coder_read_bits_buf(ia_bit_buf, 24);
    }
    ptr_binaural_rep->is_hoa_data = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 1);
    if (!ptr_binaural_rep->is_hoa_data)
    {
      err_code = impeghd_speaker_config_3d(buf_handle, &ptr_binaural_rep->setup_spk_config_3d);
      if (err_code)
      {
        return err_code;
      }

      if (ptr_binaural_rep->setup_spk_config_3d.spk_layout_type != 0)
      {
        if (ptr_binaural_rep->setup_spk_config_3d.num_speakers > BINURAL_MAX_3D_SPEAKER)
        {
          return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_SPEAKERS;
        }
        ptr_binaural_rep->brir_pairs = (WORD16)ptr_binaural_rep->setup_spk_config_3d.num_speakers;
      }
      else
      {
        ia_core_coder_read_escape_value(ia_bit_buf, (UWORD32 *)(&brir_pairs), 5, 8, 0);
        ptr_binaural_rep->brir_pairs = (WORD16)(brir_pairs + 1);
      }
    }
    else
    {
      ia_core_coder_read_escape_value(ia_bit_buf, (UWORD32 *)(&hoa_order_binaural), 3, 5, 0);
      ptr_binaural_rep->hoa_order_binaural = (WORD16)hoa_order_binaural;
      ptr_binaural_rep->brir_pairs =
          (ptr_binaural_rep->hoa_order_binaural + 1) * (ptr_binaural_rep->hoa_order_binaural + 1);
    }
    ptr_binaural_rep->binaural_data_format_id =
        (WORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 2);

    if ((ptr_binaural_rep->is_hoa_data == 0) &&
        (ptr_binaural_rep->brir_pairs > CICP2GEOMETRY_MAX_LOUDSPEAKERS_BRIR))
    {
      return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_SPEAKERS;
    }
    if (ptr_binaural_rep->brir_pairs > MAX_NUM_BRIR_PAIRS)
    {
      return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_CHANNELS;
    }
    rem = (ia_bit_buf->size - ia_bit_buf->cnt_bits);
    if (rem % 8)
    {
      fill_bits = (8 - ((ia_bit_buf->size - ia_bit_buf->cnt_bits) % 8));
      ia_core_coder_skip_bits_buf(ia_bit_buf, fill_bits);
    }

    switch (ptr_binaural_rep->binaural_data_format_id)
    {
    case 2:
      err_code = impeghd_binaural_read_td_data(ia_bit_buf, ptr_binaural_rep);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }
      break;
    case 0:
      err_code = impeghd_binaural_read_fir_data(
          buf_handle, &ptr_binaural_rep->ia_binaural_fir_data_str, ptr_binaural_rep->brir_pairs);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }
      break;
    default:
      return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_DATA_FORMAT;
    }
    binaural_ren_info->ptr_binaural_rep[i] = ptr_binaural_rep;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_read_brir_info
*
*  \brief Reads BRIR info from input configuration and updates in brir structure
*
*  \param [out]  pstr_brir_info  Pointer to brir renderer structure
*  \param [in]   pstr_bit_buf    Pointer to bit buffer structure
*
*  \return error IA_ERRORCODE if any
*
*/
IA_ERRORCODE impeghd_read_brir_info(ia_binaural_renderer *pstr_brir_info,
                                    ia_bit_buf_struct *pstr_bit_buf)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;

  err_code = impeghd_parse_brir_info((VOID *)pstr_bit_buf, pstr_brir_info);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }

  for (i = 0; i < pstr_brir_info->num_binaural_representation; i++)
  {
    if (pstr_brir_info->ptr_binaural_rep[i]->binaural_data_format_id == 0)
    {
      err_code = impeghd_compute_filter_params(
          &pstr_brir_info->ptr_binaural_rep[i]->td_binaural_ren_param,
          &pstr_brir_info->ptr_binaural_rep[i]->ia_binaural_fir_data_str,
          pstr_brir_info->ptr_binaural_rep[i]->brir_pairs, pstr_brir_info->ptr_scratch);
      if (err_code == IA_MPEGH_DEC_NO_ERROR)
      {
        pstr_brir_info->ptr_binaural_rep[i]->binaural_data_format_id = 2;
      }
      else
      {
        return err_code;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/** @} */ /* End of BinauralParsing */