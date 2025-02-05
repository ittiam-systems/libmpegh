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
#include <string.h>

#include <impeghd_type_def.h>
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_defines.h"

#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"

#include "ia_core_coder_acelp_info.h"

#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_env_extr.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include <ia_core_coder_rom.h>
#include "ia_core_coder_dec.h"
#include "impeghd_memory_standards.h"
#include "ia_core_coder_tns_usac.h"

#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_struct_def.h"
#include "impeghd_binaural.h"
#include "impeghd_error_codes.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_tbe_dec.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_nfc_filtering.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_hoa_renderer.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_spatial_decoder.h"
#include "impeghd_hoa_decoder.h"
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_obj_ren_dec.h"
#include "impeghd_uni_drc_struct.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_func_def.h"
#include "ia_core_coder_igf_dec.h"
#include "ia_core_coder_ltpf.h"
#include "ia_core_coder_struct.h"
#include "impeghd_peak_limiter_struct_def.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_resampler.h"
#include "ia_core_coder_create.h"
#include "ia_core_coder_bit_extract.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
*  ia_core_coder_info_init
*
*  \brief Initialization of scale factor band information
*
*  \param [out] pstr_sfb_info_long  Pointer to Scale factor band info - long window.
*  \param [out] pstr_sfb_info_short Pointer to Scale factor band info - short window.
*  \param [in]  ptr_samp_info       Pointer to sampling frequency info structure.
*  \param [in]  block_size_samples  Block size in samples.
*  \param [in]  sfb_width_short     Pointer to scale factor band width data - short.
*  \param [in]  sfb_width_long      Pointer to scale factor band width data - long.
*
*
*
*/
static VOID ia_core_coder_info_init(ia_sfb_info_struct *pstr_sfb_info_long,
                                    ia_sfb_info_struct *pstr_sfb_info_short,
                                    const ia_usac_samp_rate_info *ptr_samp_info,
                                    WORD32 block_size_samples, WORD16 *sfb_width_short,
                                    WORD16 *sfb_width_long)
{
  ia_sfb_info_struct *pstr_sfb_info_ip;
  const WORD16 *sfbands;
  WORD32 n, ws, sfb, win, k;

  pstr_sfb_info_long->samp_per_bk = block_size_samples;
  pstr_sfb_info_long->max_win_len = 1;
  pstr_sfb_info_long->islong = 1;

  switch (block_size_samples)
  {
  case 512:
    pstr_sfb_info_long->ptr_sfb_tbl = ptr_samp_info->ptr_sfb_512;
    pstr_sfb_info_long->sfb_per_sbk = ptr_samp_info->num_sfb_512;
    break;
  case 1024:
    pstr_sfb_info_long->sfb_per_sbk = ptr_samp_info->num_sfb_1024;
    pstr_sfb_info_long->ptr_sfb_tbl = ptr_samp_info->ptr_sfb_1024;
    break;
  default:
    break;
  }

  pstr_sfb_info_long->group_len[0] = 1;
  pstr_sfb_info_long->num_groups = 1;
  pstr_sfb_info_long->sfb_width = sfb_width_long;

  if (pstr_sfb_info_long->sfb_per_sbk)
  {
    pstr_sfb_info_long->sfb_width[0] = pstr_sfb_info_long->ptr_sfb_tbl[0];
    for (sfb = 1; sfb < pstr_sfb_info_long->sfb_per_sbk; sfb++)
    {
      pstr_sfb_info_long->sfb_width[sfb] =
          (pstr_sfb_info_long->ptr_sfb_tbl[sfb] - pstr_sfb_info_long->ptr_sfb_tbl[sfb - 1]);
    }
  }

  pstr_sfb_info_short->samp_per_bk = block_size_samples;
  pstr_sfb_info_short->max_win_len = NSHORT;
  pstr_sfb_info_short->islong = 0;

  if (pstr_sfb_info_short->max_win_len > 0)
  {
    switch (block_size_samples)
    {
    case 1024:
      pstr_sfb_info_short->ptr_sfb_tbl = ptr_samp_info->ptr_sfb_128;
      pstr_sfb_info_short->sfb_per_sbk = ptr_samp_info->num_sfb_128;
      break;
    default:
      break;
    }
  }

  pstr_sfb_info_short->sfb_width = sfb_width_short;
  if (pstr_sfb_info_short->sfb_per_sbk != 0)
  {
    pstr_sfb_info_short->sfb_width[0] = pstr_sfb_info_short->ptr_sfb_tbl[0];
    for (sfb = 1; sfb < pstr_sfb_info_short->sfb_per_sbk; sfb++)
    {
      pstr_sfb_info_short->sfb_width[sfb] =
          (pstr_sfb_info_short->ptr_sfb_tbl[sfb] - pstr_sfb_info_short->ptr_sfb_tbl[sfb - 1]);
    }
  }

  pstr_sfb_info_ip = pstr_sfb_info_long;
  for (ws = 0; ws < 2; ws++)
  {
    n = 0;
    k = 0;
    pstr_sfb_info_ip->sfb_per_bk = 0;

    for (win = 0; win < pstr_sfb_info_ip->max_win_len; win++)
    {
      pstr_sfb_info_ip->sfb_per_bk += pstr_sfb_info_ip->sfb_per_sbk;

      pstr_sfb_info_ip->bins_per_sbk =
          pstr_sfb_info_ip->samp_per_bk / pstr_sfb_info_ip->max_win_len;

      sfbands = pstr_sfb_info_ip->ptr_sfb_tbl;
      for (sfb = pstr_sfb_info_ip->sfb_per_sbk - 1; sfb >= 0; sfb--)
      {
        pstr_sfb_info_ip->sfb_idx_tbl[k + sfb] = sfbands[sfb] + (WORD16)n;
      }

      k += pstr_sfb_info_ip->sfb_per_sbk;
      n += pstr_sfb_info_ip->bins_per_sbk;
    }
    pstr_sfb_info_ip = pstr_sfb_info_short;
  }
}

/**
*  ia_core_coder_decode_init
*
*  \brief Decoder initialization
*
*  \param [in,out] handle             Pointer to decoder handle.
*  \param [in,out] usac_data          Pointer to USAC data structure.
*  \param [in,out] pstr_stream_config Pointer to audio specific configuration structure.
*  \param [in]  sample_rate        Sampling rate of the input stream.
*
*  \return IA_ERRORCODE Error code if any.
*
*/
static IA_ERRORCODE ia_core_coder_decode_init(VOID *handle, ia_usac_data_struct *usac_data,
                                              ia_audio_specific_config_struct *pstr_stream_config,
                                              WORD32 sample_rate)
{
  ia_mpegh_dec_api_struct *codec_handle = (ia_mpegh_dec_api_struct *)handle;
  ia_mpegh_dec_state_struct *mpegh_dec_handle = codec_handle->p_state_mpeghd;
  ia_usac_config_struct *ptr_usac_config = &(pstr_stream_config->str_usac_config);
  ia_usac_decoder_config_struct *ptr_usac_dec_config =
      &(pstr_stream_config->str_usac_config.str_usac_dec_config);
  ia_signals_3d *pstr_signals = &ptr_usac_config->signals_3d;

  WORD32 sample, ch, element_id = 0, slpd_index = 0, chan = 0, tot_chan = 0,
                     num_elements = ptr_usac_dec_config->num_elements, typ;
  UWORD32 element_type;

  usac_data->tns_max_bands_tbl_usac =
      &mpegh_dec_handle->pstr_mpeghd_tables->pstr_block_tables->tns_max_bands_tbl_usac;

  usac_data->huffman_code_book_scl_index = mpegh_dec_handle->huffman_code_book_scl_index;
  usac_data->huffman_code_book_scl = mpegh_dec_handle->huffman_code_book_scl;

  for (sample = 0; sample < SAMPLE_BOUNDARY; sample++)
  {
    if (ia_core_coder_sampling_boundaries[sample] <= sample_rate)
      break;
  }

  if (sample == (1 << LEN_SAMP_IDX))
    return IA_MPEGH_DEC_INIT_FATAL_SAMP_FREQ_NOT_SUPPORTED;
  usac_data->sampling_rate_idx = sample;

  memset(usac_data->window_shape, 0, MAX_NUM_CHANNELS * sizeof(WORD32));
  memset(usac_data->window_shape_prev, 0, MAX_NUM_CHANNELS * sizeof(WORD32));

  ia_core_coder_hufftab(&ia_core_coder_book, ia_core_coder_book_scl_huff_code_word,
                        ia_core_coder_book_scl_code_book, ia_core_coder_book_scl_index, 1, 60, 60,
                        1, 19);

  usac_data->pstr_usac_winmap[4] = &usac_data->str_only_long_info;
  usac_data->pstr_usac_winmap[3] = &usac_data->str_only_long_info;
  usac_data->pstr_usac_winmap[2] = &usac_data->str_eight_short_info;
  usac_data->pstr_usac_winmap[1] = &usac_data->str_only_long_info;
  usac_data->pstr_usac_winmap[0] = &usac_data->str_only_long_info;

  if (usac_data->ccfl != 1024)
    return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_FRAME_LENGTH;
  ia_core_coder_info_init(usac_data->pstr_usac_winmap[0], usac_data->pstr_usac_winmap[2],
                          &ia_core_coder_samp_rate_info[usac_data->sampling_rate_idx],
                          usac_data->ccfl, usac_data->sfb_width_short, usac_data->sfb_width_long);

  for (ch = 0; ch < MAX_NUM_CHANNELS; ch++)
  {
    usac_data->num_subfrm =
        (usac_data->ccfl >> 8); //(MAX_NUM_SUBFR * usac_data->ccfl) / LEN_SUPERFRAME;
    usac_data->len_subfrm = (usac_data->ccfl >> 2);

    usac_data->str_tddec[ch] = &usac_data->arr_str_tddec[ch];

    if (ia_core_coder_init_acelp_data(usac_data, usac_data->str_tddec[ch], ch))
      return IA_MPEGH_DEC_INIT_FATAL_DEC_INIT_FAIL;

    usac_data->str_tddec[ch]->fd_synth = &usac_data->str_tddec[ch]->fd_synth_buf[LEN_FRAME];
    ia_core_coder_ltpf_init(&usac_data->str_tddec[ch]->ltpf_data,
                            pstr_stream_config->str_usac_config.usac_sampling_frequency,
                            usac_data->ccfl, codec_handle->pp_mem_mpeghd[MEMTYPE_SCRATCH]);
  }

  for (element_id = 0; element_id < num_elements; element_id++)
  {
    element_type = ptr_usac_config->str_usac_dec_config.usac_element_type[element_id];

    ia_usac_dec_element_config_struct *ptr_usac_ele_config =
        &ptr_usac_config->str_usac_dec_config.str_usac_element_config[element_id];

    if (ptr_usac_ele_config)
    {
      usac_data->noise_filling_config[element_id] = ptr_usac_ele_config->noise_filling;
    }

    ia_core_coder_igf_init(usac_data, ptr_usac_ele_config, element_type, element_id,
                           ptr_usac_config->usac_sampling_frequency, chan);
    switch (element_type)
    {
    case ID_USAC_EXT:
      break;

    case ID_USAC_CPE:
    {
      if (pstr_signals->inactive_signals[tot_chan])
      {
        tot_chan += 2;
        break;
      }
      tot_chan += 2;
      if (chan > MAX_NUM_CHANNELS - 2)
        return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
      for (typ = 0; typ < 2; typ++)
      {
        usac_data->str_tddec[chan + typ]->len_subfrm =
            usac_data->td_config[element_id].len_subfrm;
        usac_data->str_tddec[chan + typ]->fscale = usac_data->td_config[element_id].fscale;
      }

      /* LPD joint channels*/
      if (ptr_usac_ele_config->lpd_stereo_index == 1)
      {
        ia_core_coder_slpd_init(&usac_data->slpd_dec_data[slpd_index],
                                ptr_usac_config->usac_sampling_frequency,
                                ptr_usac_ele_config->full_band_lpd, usac_data->ccfl);
        slpd_index++;
      }
      usac_data->seed_value[chan] = 0x3039;
      chan++;

      usac_data->seed_value[chan] = 0x10932;
      chan++;
    }
    break;
    case ID_USAC_LFE:
    {
      if (pstr_signals->inactive_signals[tot_chan])
      {
        tot_chan += 1;
        break;
      }
      tot_chan += 1;
      /*fix for bug:119118558*/
      if (chan > MAX_NUM_CHANNELS - 1)
        return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
      usac_data->seed_value[chan] = 0x3039;
      chan++;
    }

    break;
    case ID_USAC_SCE:
    {
      if (pstr_signals->inactive_signals[tot_chan])
      {
        tot_chan += 1;
        break;
      }
      tot_chan += 1;
      if (chan > MAX_NUM_CHANNELS - 1)
        return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
      usac_data->seed_value[chan] = 0x3039;

      usac_data->str_tddec[chan]->len_subfrm = usac_data->td_config[element_id].len_subfrm;
      usac_data->str_tddec[chan]->fscale = usac_data->td_config[element_id].fscale;
      chan++;
    }
    break;
    default:
      return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_ELEM_INDEX;
      break;
    }
    if (chan > MAX_NUM_CHANNELS)
    {
      return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  ia_core_coder_dec_data_init
*
*  \brief Initialization of decoder data
*
*  \param [in,out] handle          Pointer to decoder handle.
*  \param [in,out] pstr_frame_data Pointer to frame data
*  \param [in,out] usac_data       Pointer to USAC data structure.
*
*  \return IA_ERRORCODE Error code if any.
*
*/
static IA_ERRORCODE ia_core_coder_dec_data_init(VOID *handle,
                                                ia_frame_data_struct *pstr_frame_data,
                                                ia_usac_data_struct *usac_data)
{
  ia_audio_specific_config_struct *pstr_stream_config, *layer_config;
  ia_usac_config_struct *ptr_usac_config =
      &(pstr_frame_data->str_audio_specific_config.str_usac_config);

  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 num_elements, i, element_id;
  UWORD32 element_type;

  pstr_stream_config = &pstr_frame_data->str_audio_specific_config;
  layer_config = &pstr_frame_data->str_audio_specific_config;

  pstr_stream_config->samp_frequency_index = layer_config->samp_frequency_index;
  pstr_stream_config->sampling_frequency = pstr_frame_data->str_layer.sample_rate_layer;

  usac_data->window_shape_prev[1] = WIN_SEL_0;
  usac_data->window_shape_prev[0] = WIN_SEL_0;

  usac_data->output_samples = OUT_FRAMELENGTH_1024;
  usac_data->ccfl = OUT_FRAMELENGTH_1024;

  num_elements = ptr_usac_config->str_usac_dec_config.num_elements;

  for (element_id = 0; element_id < num_elements; element_id++)
  {
    ia_usac_td_config_handle td_config = &usac_data->td_config[element_id];
    ia_usac_dec_element_config_struct *ptr_usac_ele_config =
        &(ptr_usac_config->str_usac_dec_config.str_usac_element_config[element_id]);

    element_type = ptr_usac_config->str_usac_dec_config.usac_element_type[element_id];
    if (ptr_usac_ele_config)
    {
      td_config->fscale_full_band = pstr_stream_config->sampling_frequency;
      td_config->len_frame_fb = usac_data->ccfl;
      td_config->lpd_stereo_idx =
          element_type == ID_USAC_CPE ? ptr_usac_ele_config->lpd_stereo_index : 0;
      td_config->full_band_lpd = ptr_usac_ele_config->full_band_lpd;

      if (!td_config->full_band_lpd)
      {
        WORD32 temp = (WORD32)((double)pstr_stream_config->sampling_frequency *
                               (double)FSCALE_DENOM / 12800.0f);
        td_config->fscale = ((temp)*usac_data->ccfl) / LEN_SUPERFRAME;
        td_config->num_frame = NUM_FRAMES;
        td_config->len_subfrm = (usac_data->ccfl) / td_config->num_frame;
        td_config->num_subfrm = (NUM_FRAMES * td_config->len_subfrm) / LEN_FRAME;
        td_config->len_frame = usac_data->ccfl;
        td_config->fac_fb = 1;
        td_config->igf_active = 0;
      }
      else
      {
        td_config->fscale =
            (pstr_stream_config->sampling_frequency * (usac_data->ccfl / 2)) / LEN_SUPERFRAME;
        td_config->igf_active = (element_type == ID_USAC_CPE || element_type == ID_USAC_SCE)
                                    ? ptr_usac_ele_config->enhanced_noise_filling
                                    : 0;
        td_config->num_frame = (NUM_FRAMES) >> 1;
        td_config->len_subfrm = (usac_data->ccfl >> 1) / td_config->num_frame;
        td_config->num_subfrm = (NUM_FRAMES * td_config->len_subfrm) / LEN_FRAME;
        td_config->len_frame = (usac_data->ccfl >> 1);
        td_config->fac_fb = 2;
      }
    }
  }

  for (i = MAX_NUM_CHANNELS - 1; i >= 0; i--)
  {
    usac_data->usac_first_frame_flag[i] = 1;

    usac_data->fac_data_present[i] = 0;
    usac_data->seed_value[i] = 0x0;

    usac_data->window_shape_prev[i] = WIN_SEL_0;
    usac_data->ms_used[i] = &usac_data->arr_ms_used[i][0];
    usac_data->pstr_tns[i] = &usac_data->arr_str_tns[i];
    usac_data->group_dis[i] = &usac_data->arr_group_dis[i][0];
    usac_data->factors[i] = &usac_data->arr_factors[i][0];
    usac_data->coef_save_float[i] = &usac_data->arr_coef_save_float[i][0];
    usac_data->coef[i] = &usac_data->arr_coef[i][0];
  }

  err_code = ia_core_coder_decode_init(handle, usac_data, pstr_stream_config,
                                       pstr_frame_data->str_layer.sample_rate_layer);
  return err_code;
}

/**
*  ia_core_coder_count_tracks_per_layer
*
*  \brief Updates track per layers
*
*  \param [in,out] max_layer       Pointer to max layers buffer.
*  \param [in,out] stream_count    Pointer to stream count buffer.
*  \param [in,out] tracks_in_layer Pointer to number of tracks in layer buffer.
*
*
*
*/
static VOID ia_core_coder_count_tracks_per_layer(WORD32 *max_layer, WORD32 *stream_count,
                                                 WORD32 *tracks_in_layer)
{
  WORD32 stream, num_layer, num_streams, layer = 0;

  if (stream_count != NULL)
    num_streams = *stream_count;
  else
    num_streams = 0;

  if (max_layer != NULL)
    num_layer = *max_layer;
  else
    num_layer = num_streams;

  if (num_layer < 0)
    num_layer = num_streams;

  for (stream = 0; (layer <= num_layer) && (stream < num_streams);)
  {
    layer++;
    stream += 1;
    *tracks_in_layer = 1;

    if (layer <= num_layer)
      *tracks_in_layer = 0;
  }

  if (stream_count)
    *stream_count = stream;
  if (max_layer)
    *max_layer = (layer - 1);
}

/**
*  ia_core_coder_frm_data_init
*
*  \brief Initializatoin of usac frame data
*
*  \param [in,out] pstr_audio_conf Pointer to audio specific config structure.
*  \param [in,out] pstr_dec_data   Pointer to decoder data structure.
*
*  \return WORD32 num_dec_streams
*
*/
WORD32 ia_core_coder_frm_data_init(ia_audio_specific_config_struct *pstr_audio_conf,
                                   ia_dec_data_struct *pstr_dec_data)

{
  ia_frame_data_struct *pstr_frame_data;
  WORD32 num_dec_streams, track, layer, stream_count = 1, max_layer = -1;

  memset(&pstr_dec_data->str_obj_ren_dec_state, 0, sizeof(pstr_dec_data->str_obj_ren_dec_state));

  memset(&pstr_dec_data->str_hoa_frame_data, 0, sizeof(pstr_dec_data->str_hoa_frame_data));
  memset(&pstr_dec_data->str_hoa_dec_handle, 0, sizeof(pstr_dec_data->str_hoa_dec_handle));

  memset(&pstr_dec_data->dec_bit_buf, 0, sizeof(pstr_dec_data->dec_bit_buf));

  if (1 == pstr_dec_data->str_local_setup_interaction.enable_ele_int)
  {
    pstr_dec_data->str_frame_data.str_audio_specific_config.cicp_spk_layout_idx =
        pstr_dec_data->str_local_setup_interaction.spk_config.cicp_spk_layout_idx;
  }
  if (!pstr_audio_conf->cicp_spk_layout_idx)
  {
    pstr_dec_data->str_obj_ren_dec_state.cicp_out_idx =
        pstr_dec_data->str_frame_data.str_audio_specific_config.ref_spk_layout
            .cicp_spk_layout_idx;
  }
  else
  {
    pstr_dec_data->str_obj_ren_dec_state.cicp_out_idx =
        pstr_dec_data->str_frame_data.str_audio_specific_config.cicp_spk_layout_idx;
  }


  memset(&pstr_dec_data->str_frame_data.str_layer, 0,
         sizeof(pstr_dec_data->str_frame_data.str_layer));
  memset(&pstr_dec_data->str_drc_payload, 0, sizeof(pstr_dec_data->str_drc_payload));

  pstr_dec_data->str_frame_data.tracks_in_layer = 0;
  pstr_dec_data->str_frame_data.scal_out_select = 0;
  pstr_dec_data->str_frame_data.stream_count = 0;
  pstr_dec_data->str_frame_data.scal_out_sampling_frequency = 48000;
  pstr_dec_data->str_frame_data.scal_out_num_channels = 0;

  pstr_frame_data = &(pstr_dec_data->str_frame_data);

  if (max_layer < 0)
    max_layer = stream_count - 1;

  ia_core_coder_count_tracks_per_layer(&max_layer, &stream_count,
                                       &pstr_frame_data->tracks_in_layer);

  num_dec_streams = track = 0;

  pstr_frame_data->stream_count = 0;

  pstr_frame_data->scal_out_select = max_layer;

  for (layer = 0; layer < (signed)pstr_frame_data->scal_out_select + 1; layer++)
  {
    pstr_frame_data->str_layer.sample_rate_layer =
        pstr_frame_data->str_audio_specific_config.sampling_frequency;
    num_dec_streams++;
    track += pstr_frame_data->tracks_in_layer;
  }

  pstr_frame_data->stream_count = num_dec_streams;

  return num_dec_streams;
}

/**
*  ia_core_coder_decode_create
*
*  \brief Creates decoder handle
*
*  \param [in,out] handle             Pointer to decoder api structure.
*  \param [in,out] pstr_dec_data      Pointer to decoder state structure.
*  \param [in]  tracks_for_decoder Number of decoder tracks.
*
*  \return IA_ERRORCODE Error code if any.
*
*/
IA_ERRORCODE ia_core_coder_decode_create(ia_mpegh_dec_api_struct *handle,
                                         ia_dec_data_struct *pstr_dec_data,
                                         WORD32 tracks_for_decoder)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 stream;

  ia_frame_data_struct *pstr_frame_data;
  WORD32 stream_count;
  ia_mpegh_dec_state_struct *mpegh_dec_handle = handle->p_state_mpeghd;
  pstr_frame_data = &(pstr_dec_data->str_frame_data);
  stream_count = pstr_frame_data->stream_count;
  pstr_frame_data->stream_count = tracks_for_decoder;

  for (stream = 0; stream < stream_count; stream++)
  {
    err = ia_core_coder_dec_data_init(handle, pstr_frame_data, &(pstr_dec_data->str_usac_data));
    if (err != IA_MPEGH_DEC_NO_ERROR)
    {
      return err;
    }
  }

  mpegh_dec_handle->decode_create_done = 1;
  pstr_dec_data->str_usac_data.scratch_int_buf = mpegh_dec_handle->mpeghd_scratch_mem_v;
  pstr_dec_data->str_usac_data.x_ac_dec_float =
      (FLOAT32 *)pstr_dec_data->str_usac_data.scratch_int_buf + 2048;
  pstr_dec_data->str_usac_data.scratch_buffer_float =
      pstr_dec_data->str_usac_data.x_ac_dec_float + 1024;
  pstr_dec_data->str_usac_data.ptr_fft_scratch =
      pstr_dec_data->str_usac_data.scratch_buffer_float + 1024;

  pstr_frame_data->scal_out_sampling_frequency =
      pstr_frame_data->str_audio_specific_config.sampling_frequency;
  pstr_frame_data->scal_out_num_channels =
      pstr_frame_data->str_audio_specific_config.str_usac_config.signals_3d.num_hoa_transport_ch +
      pstr_frame_data->str_audio_specific_config.str_usac_config.signals_3d.num_audio_obj +
      pstr_frame_data->str_audio_specific_config.channel_configuration;

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecProc */