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
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_defines.h"

#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_init.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"

#include "ia_core_coder_acelp_info.h"

#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_definitions.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "impeghd_memory_standards.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_dec.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_struct_def.h"
#include "ia_core_coder_headerdecode.h"

#include "impeghd_config_params.h"
#include "impeghd_error_codes.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_multichannel.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_binaural.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_format_conv_rom.h"
#include "impeghd_format_conv_data.h"
#include "impeghd_dmx_mtx_data.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_matrix_struct.h"
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
#include "impeghd_oam_dec.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_obj_ren_dec.h"
#include "impeghd_uni_drc_struct.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_struct.h"
#include "impeghd_peak_limiter_struct_def.h"
#include "impeghd_binaural.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_resampler.h"
#include "ia_core_coder_create.h"
#include "ia_core_coder_dec_main.h"
#include "impd_drc_host_params.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_metadata_preprocessor.h"
#include "impeghd_ext_rend_intrfc.h"
#include "impeghd_ext_rend_intrfc_util.h"

IA_ERRORCODE impeghd_peak_limiter_init(ia_peak_limiter_struct *peak_limiter, UWORD32 num_channels,
                                       UWORD32 sample_rate, FLOAT32 *buffer);
VOID impeghd_peak_limiter_process(ia_peak_limiter_struct *peak_limiter, FLOAT32 *samples,
                                  UWORD32 frame_len);
IA_ERRORCODE impeghd_uni_drc_dec_init(ia_audio_specific_config_struct *pstr_audio_specific_config,
                                      ia_dec_data_struct *pstr_dec_data, WORD32 tgt_loudness,
                                      WORD32 loud_norm_flag, WORD32 drc_effect_type,
                                      WORD8 preset_id, WORD32 ui_cicp_layout_idx, WORD32 index);

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  impd_match_downmix
 *
 *  \brief Dowmix id mapping function
 *
 *  \param [in] downmix_id     Command line based downmix id
 *  \param [in] dec_downmix_id Downmix Id value to be matched with
 *
 *  \return WORD32 Flag indicating id match.
 *
 */
static WORD32 impd_match_downmix(WORD32 downmix_id, WORD32 dec_downmix_id)
{
  WORD32 match = 0;

  switch (dec_downmix_id)
  {
  case 4:
    if (0 != downmix_id)
    {
      match = 1;
    }
    break;
  case 3:
    if ((0 != downmix_id) && (0x7F != downmix_id))
    {
      match = 1;
    }
    break;
  case 2:
    if (0x7F == downmix_id)
    {
      match = 1;
    }
    break;
  case 1:
    if ((0 == downmix_id) || (0x7F == downmix_id))
    {
      match = 1;
    }
    break;
  case 0:
    if (0 == downmix_id)
    {
      match = 1;
    }
    break;
  }
  return match;
}

/**
 *  ia_core_coder_samples_sat
 *
 *  \brief Function that converts PCM samples the final PCM output.
 *
 *  \param [out] outbuffer         Pointer to output buffer.
 *  \param [in]  num_samples_out   Number of output samples
 *  \param [in]  pcmsize           PCM size in bits.
 *  \param [in]  out_samples       Pointer to pcm samples buffer.
 *  \param [in]  out_ch_map        Pointer to output channel map.
 *  \param [in]  ptr_delay_samples Pointer to delays samples info.
 *  \param [out] out_bytes         Pointer to variable that carries out bytes info.
 *  \param [in]  num_channel_out   Number of output channels.
 *
 *
 *
 */
static VOID ia_core_coder_samples_sat(WORD8 *outbuffer, WORD32 num_samples_out, WORD32 pcmsize,
                                      FLOAT32 **out_samples, WORD32 *out_ch_map,
                                      WORD32 *ptr_delay_samples, WORD32 *out_bytes,
                                      WORD32 num_channel_out)
{
  WORD32 num, ch, sample, write_local, delay_samples = *ptr_delay_samples;
  WORD32 channel_pos[MAX_NUM_CHANNELS] = {0};

  FLOAT32 write_local_float;
  WORD16 *out_buf = (WORD16 *)outbuffer;

  if (delay_samples <= num_samples_out)
  {
    num = num_channel_out * (num_samples_out - delay_samples);
    *ptr_delay_samples = 0;
  }
  else
  {
    num = 0;
    *ptr_delay_samples = *ptr_delay_samples - num_samples_out;
  }

  for (ch = 0; ch < num_channel_out; ch++)
  {
    if (out_ch_map[ch] == -1)
      out_ch_map[ch] = ch;
    channel_pos[out_ch_map[ch]] = ch;
  }

  switch (pcmsize)
  {
  case 24:
  {
    WORD8 *out_24bit = (WORD8 *)out_buf;
    for (sample = delay_samples; sample < num_samples_out; sample++)
    {
      for (ch = 0; ch < num_channel_out; ch++)
      {
        write_local_float = (out_samples[channel_pos[ch]][sample] * MUL_FAC_PCM_24);

        if (ia_lt_flt(write_local_float, MIN_FLT_VAL_24))
        {
          write_local_float = MIN_FLT_VAL_24;
        }
        else if (ia_lt_flt(MAX_FLT_VAL_24, write_local_float))
        {
          write_local_float = MAX_FLT_VAL_24;
        }
        write_local = (WORD32)write_local_float;

        *out_24bit++ = (WORD32)write_local & 0xff;
        *out_24bit++ = ((WORD32)write_local >> 8) & 0xff;
        *out_24bit++ = ((WORD32)write_local >> 16) & 0xff;
      }
    }
  }
  break;
  case 16:
  {
    for (sample = delay_samples; sample < num_samples_out; sample++)
    {
      for (ch = 0; ch < num_channel_out; ch++)
      {
        write_local_float = (out_samples[channel_pos[ch]][sample]);

        if (ia_lt_flt(write_local_float, MIN_FLT_VAL_16))
        {
          write_local_float = MIN_FLT_VAL_16;
        }
        else if (ia_lt_flt(MAX_FLT_VAL_16, write_local_float))
        {
          write_local_float = MAX_FLT_VAL_16;
        }
        *out_buf++ = (WORD16)write_local_float;
      }
    }
  }
  break;
  default:
  { /* 32bit */
    WORD8 *out_32_buf = (WORD8 *)out_buf;
    for (sample = delay_samples; sample < num_samples_out; sample++)
    {
      for (ch = 0; ch < num_channel_out; ch++)
      {
        write_local_float = (out_samples[channel_pos[ch]][sample] * MUL_FAC_PCM_32);

        if (ia_lt_flt(write_local_float, MIN_FLT_VAL_32))
        {
          write_local_float = MIN_FLT_VAL_32;
        }
        else if (ia_lt_flt(MAX_FLT_VAL_32, write_local_float))
        {
          write_local_float = MAX_FLT_VAL_32;
        }
        write_local = (WORD32)write_local_float;

        *out_32_buf++ = (WORD32)write_local & 0xff;
        *out_32_buf++ = ((WORD32)write_local >> 8) & 0xff;
        *out_32_buf++ = ((WORD32)write_local >> 16) & 0xff;
        *out_32_buf++ = ((WORD32)write_local >> 24) & 0xff;
      }
    }
  }
  break;
  }
  *out_bytes = num * (pcmsize >> 3);
}

/**
 *  impeghd_hoa_dec_main_process
 *
 *  \brief HOA Decoder process main function.
 *
 *  \param [in]     pstr_usac_dec_cfg Pointer to USAC decoder config structure.
 *  \param [in,out] pstr_dec_data     Pointer to USAC decoder data structure.
 *  \param [in]     ele_idx           Extension element index.
 *  \param [out]    ptr_out_buf       Pointer to output buffer.
 *  \param [out]    num_out_channels  Pointer to number of output channels info.
 *  \param [in]     ch_offset         Offset from where HOA channels start.
 *
 *  \return IA_ERRORCODE           Error code in case of any processing errors.
 *
 */
static IA_ERRORCODE impeghd_hoa_dec_main_process(ia_usac_decoder_config_struct *pstr_usac_dec_cfg,
                                                 ia_dec_data_struct *pstr_dec_data,
                                                 WORD32 ele_idx, FLOAT32 *ptr_out_buf,
                                                 WORD32 *num_out_channels, WORD32 ch_offset,
                                                 WORD32 delay_flag)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  UWORD8 *ptr_bit_buf_base = NULL;
  WORD32 bit_buf_size = 0;

  FLOAT32 *ptr_prev_in_buf = &pstr_dec_data->str_usac_data.time_sample_hoa_vect[ch_offset][0];
  FLOAT32 *ptr_in_buf = &pstr_dec_data->str_usac_data.time_sample_vector[ch_offset][0];

  ia_bit_buf_struct bit_buf_str;
  impeghd_hoa_dec_struct *dec_handle_t_ptr = &pstr_dec_data->str_hoa_dec_handle;
  jmp_buf hoa_dec_jump_buff;
  err_code = setjmp(hoa_dec_jump_buff);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_HOA_BYTES;
  }

  if (pstr_dec_data->str_binaural_rendering.binaural_rep.brir_pairs != 0)
  {
    dec_handle_t_ptr->is_brir_rendering = 1;
  }
  else
  {
    dec_handle_t_ptr->is_brir_rendering = 0;
  }

  if (1 != delay_flag)
  {
    ptr_bit_buf_base = &pstr_usac_dec_cfg->usac_ext_gain_payload_buf[ele_idx][0];
    bit_buf_size = pstr_usac_dec_cfg->usac_ext_gain_payload_len[ele_idx];
  }
  else
  {
    ptr_bit_buf_base = &pstr_usac_dec_cfg->usac_ext_gain_payload_prev_buf[0];
    bit_buf_size = pstr_usac_dec_cfg->usac_ext_gain_payload_prev_len;
  }

  /* Create local pointers for HOA structures */
  ia_core_coder_create_init_bit_buf(&bit_buf_str, ptr_bit_buf_base, bit_buf_size);
  bit_buf_str.xmpeghd_jmp_buf = &hoa_dec_jump_buff;
  dec_handle_t_ptr->ia_hoa_frame = &pstr_dec_data->str_hoa_frame_data;
  dec_handle_t_ptr->ia_bit_buf = &bit_buf_str;

  err_code = impeghd_hoa_dec_decode(dec_handle_t_ptr, ptr_out_buf, ptr_in_buf, ptr_prev_in_buf,
                                    num_out_channels, delay_flag);

  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }
  if (1 == delay_flag)
  {
    memcpy(&pstr_usac_dec_cfg->usac_ext_gain_payload_prev_buf[0],
           &pstr_usac_dec_cfg->usac_ext_gain_payload_buf[ele_idx][0],
           pstr_usac_dec_cfg->usac_ext_gain_payload_len[ele_idx]);
    pstr_usac_dec_cfg->usac_ext_gain_payload_prev_len =
        pstr_usac_dec_cfg->usac_ext_gain_payload_len[ele_idx];
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_format_conv_earcon_init
 *
 *  \brief Format Converter module of earcon data init function
 *
 *  \param [in,out] p_obj_mpegh_dec       Pointer to the decoder API structure.
 *  \param [in]  pstr_audio_specific_config Pointer to audio specific config structure.
 *
 *  \return IA_ERRORCODE                    Error code in case of any processing errors.
 *
 */
IA_ERRORCODE
impeghd_format_conv_earcon_init(ia_mpegh_dec_api_struct *p_obj_mpegh_dec,
                                ia_audio_specific_config_struct *pstr_audio_specific_config)
{
  ia_mae_audio_scene_info *pstr_audio_info = &pstr_audio_specific_config->str_mae_asi;
  ia_format_conv_param *pstr_fc_params = NULL;
  ia_format_conv_struct *pstr_persistant_fs;
  ia_format_conv_state_struct *pstr_fc_state = NULL;
  ia_dec_data_struct *pstr_decoder = p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data;
  ia_earcon_info *pstr_ec_info = &pstr_audio_info->earcon_info;
  WORD32 spk_layout_cicp_idx = pstr_ec_info->earcon_cicp_spk_idx[0];
  ia_format_converter_scratch *pstr_format_converter_scratch =
      p_obj_mpegh_dec->p_state_mpeghd->mpeghd_scratch_mem_v;
  const WORD32 *ptr_ch_out = NULL;
  WORD32 num_chnl_in = 0;
  IA_ERRORCODE error_num = IA_MPEGH_DEC_NO_ERROR;
  WORD32 num_out_chn = 0;
  const WORD32 *ptr_chn_in_name;
  WORD32 *ptr_chn_in = NULL;
  WORD32 i, j;
  WORD32 index = 0;

  pstr_persistant_fs = &pstr_decoder->str_earcon_format_converter;
  pstr_fc_state = &p_obj_mpegh_dec->p_state_mpeghd->state_format_conv;
  ptr_chn_in = &pstr_fc_state->format_in_chan[0];
  if (spk_layout_cicp_idx == 0 || spk_layout_cicp_idx == 8 || spk_layout_cicp_idx > NUM_LS_CFGS)
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
  }
  num_chnl_in = impgehd_cicp_get_num_ls[spk_layout_cicp_idx];
  ptr_chn_in_name = ia_cicp_idx_ls_set_map_tbl[spk_layout_cicp_idx];
  for (j = 0; j < num_chnl_in; j++)
  {
    ptr_chn_in[index + j] = ptr_chn_in_name[j];
  }
  pstr_fc_params = &pstr_persistant_fs->fc_params;
  /* downmix mtx */
  pstr_fc_params->active_dmx_stft = &pstr_persistant_fs->active_dmx_stft;
  pstr_fc_params->downmix_mat = (FLOAT32 **)&pstr_persistant_fs->downmix_mat_ptr[0];
  // pstr_fc_params->downmix_mat_freq = &pstr_persistant_fs->downmix_mat_freq_ptr[0];
  pstr_fc_state->stft_in_buf = (FLOAT32 **)&pstr_persistant_fs->stft_input_buf_ptr[0];
  pstr_fc_state->stft_out_buf = (FLOAT32 **)&pstr_persistant_fs->stft_output_buf_ptr[0];
  for (i = 0; i < FC_MAX_CHANNELS; i++)
  {
    pstr_fc_params->downmix_mat[i] = &pstr_persistant_fs->downmix_mat[i * FC_MAX_CHANNELS];
    pstr_fc_state->stft_in_buf[i] = &pstr_persistant_fs->stft_input_buf[i * FC_STFT_FRAME];
    pstr_fc_state->stft_out_buf[i] = &pstr_persistant_fs->stft_output_buf[i * FC_STFT_FRAME];
  }
  pstr_fc_params->active_dmx_stft->downmix_mat =
      (FLOAT32 ***)&pstr_persistant_fs->active_downmix_mat_ptr[0];
  for (i = 0; i < FC_MAX_CHANNELS; i++)
  {
    pstr_fc_params->active_dmx_stft->downmix_mat[i] =
        (FLOAT32 **)&pstr_persistant_fs->active_downmix_mat_ch_ptr[i * FC_MAX_CHANNELS];

    for (j = 0; j < FC_MAX_CHANNELS; j++)
    {
      pstr_fc_params->active_dmx_stft->downmix_mat[i][j] =
          (FLOAT32 *)&pstr_persistant_fs
              ->active_downmix_mat_freq_ptr[(i * FC_MAX_CHANNELS * FC_BANDS) + j * FC_BANDS];
    }
  }

  pstr_fc_params->data_state = &pstr_persistant_fs->format_conv_params_int;
  if (spk_layout_cicp_idx == 0 || spk_layout_cicp_idx == 8 || spk_layout_cicp_idx > NUM_LS_CFGS)
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
  }
  num_out_chn = impgehd_cicp_get_num_ls[spk_layout_cicp_idx];
  ptr_ch_out = ia_cicp_idx_ls_set_map_tbl[spk_layout_cicp_idx];

  num_chnl_in = pstr_audio_specific_config->channel_configuration;
  pstr_fc_state->fc_params = pstr_fc_params;

  pstr_fc_params->samp_rate = p_obj_mpegh_dec->p_state_mpeghd->p_config->ui_samp_freq;
  pstr_fc_params->out_ch = &ptr_ch_out[0];
  pstr_fc_params->in_ch = &ptr_chn_in[0];
  pstr_fc_params->num_in_ch = num_chnl_in;
  pstr_fc_params->num_out_ch = num_out_chn;
  FLOAT32 *ptr_scratch2 = &pstr_format_converter_scratch->earcon_fc_data[0];
  ia_cicp_ls_geo_str *azi_elev_array = &pstr_format_converter_scratch->str_azi_elev_ec[0];
  error_num = impeghd_format_conv_init_data(ptr_scratch2, azi_elev_array, pstr_fc_params,
                                            &pstr_audio_specific_config->ref_spk_layout);
  if (error_num != IA_MPEGH_DEC_NO_ERROR)
  {
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_PARAM;
  }

  error_num = impeghd_format_conv_dmx_init(pstr_fc_params->active_dmx_stft, pstr_fc_params);

  if (error_num != IA_MPEGH_DEC_NO_ERROR)
    error_num = IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;

  return error_num;
}
/**
 *  impeghd_format_conv_init
 *
 *  \brief Format Converter module init function
 *
 *  \param [in,out] p_obj_mpegh_dec         Pointer to the decoder API structure.
 *  \param [in]  pstr_audio_specific_config Pointer to audio specific config structure.
 *  \param [out] num_channel_out            Pointer to output channel variable.
 *  \param [in]  ds_flag                    Domain Switcher flag.
 *
 *  \return IA_ERRORCODE                    Error code in case of any processing errors.
 *
 */
IA_ERRORCODE impeghd_format_conv_init(ia_mpegh_dec_api_struct *p_obj_mpegh_dec,
                                      ia_audio_specific_config_struct *pstr_audio_specific_config,
                                      WORD32 *num_channel_out, WORD32 ds_flag)
{
  WORD32 num_chnl_in = 0;
  IA_ERRORCODE error_num = IA_MPEGH_DEC_NO_ERROR;
  WORD32 num_out_chn = 0;
  const WORD32 *ptr_chn_in_name;
  WORD32 *ptr_chn_in = NULL;
  WORD32 i, j;
  const WORD32 *ptr_ch_out = NULL;

  WORD32 spk_layout_cicp_idx = pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx;
  ia_format_conv_struct *pstr_format_conv;
  ia_format_conv_param *pstr_fc_params = NULL;
  ia_usac_config_struct *pstr_usac_config = &(pstr_audio_specific_config->str_usac_config);
  ia_signals_3d *pstr_signals_3d = &pstr_usac_config->signals_3d;
  ia_ds_state_struct *p_ds_state;
  ia_domain_switcher_struct *pstr_persistant_ds = NULL;
  ia_format_conv_state_struct *pstr_fc_state = NULL;
  ia_dec_data_struct *pstr_decoder = p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data;
  ia_format_converter_scratch *pstr_format_converter_scratch =
      p_obj_mpegh_dec->p_state_mpeghd->mpeghd_scratch_mem_v;
  WORD32 index = 0;
  if (ds_flag)
  {

    p_ds_state = &p_obj_mpegh_dec->p_state_mpeghd->state_domain_switcher;
    pstr_persistant_ds = &pstr_decoder->str_domain_switcher;
    num_out_chn = num_chnl_in = pstr_signals_3d->num_audio_obj +
                                pstr_signals_3d->num_hoa_transport_ch + pstr_signals_3d->num_ch;
    pstr_persistant_ds->ds_params.num_in_ch = num_out_chn;
    pstr_persistant_ds->ds_params.num_out_ch = num_out_chn;
    p_ds_state->ds_params = &pstr_persistant_ds->ds_params;
    p_ds_state->stft_out_buf = (FLOAT32 **)&pstr_persistant_ds->stft_output_buf_ptr[0];
    p_ds_state->stft_in_buf = (FLOAT32 **)&pstr_persistant_ds->stft_input_buf_ptr[0];

    for (i = 0; i < FC_MAX_CHANNELS; i++)
    {
      p_ds_state->stft_out_buf[i] = &pstr_persistant_ds->stft_output_buf[i * FC_STFT_FRAME];
      p_ds_state->stft_in_buf[i] = &pstr_persistant_ds->stft_input_buf[i * FC_STFT_FRAME];
    }
  }
  else
  {
    pstr_fc_state = &p_obj_mpegh_dec->p_state_mpeghd->state_format_conv;
    pstr_format_conv = &pstr_decoder->str_format_converter;
    ptr_chn_in = &pstr_fc_state->format_in_chan[0];
    for (i = 0; i < (WORD32)pstr_signals_3d->num_sig_group; i++)
    {

      if (pstr_signals_3d->fixed_position[i] == 1)
      {
        WORD32 cicp_idx = pstr_signals_3d->audio_ch_layout[i].cicp_spk_layout_idx;

        if (pstr_signals_3d->differs_from_ref_layout[i] == 1)
        {
          if (cicp_idx == 0 || cicp_idx == 8 || cicp_idx > NUM_LS_CFGS)
          {
            return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
          }
          num_chnl_in = impgehd_cicp_get_num_ls[cicp_idx];
          ptr_chn_in_name = ia_cicp_idx_ls_set_map_tbl[cicp_idx];
        }
        else
        {
          WORD32 cicp_idx = spk_layout_cicp_idx;
          if (cicp_idx == 0 || cicp_idx == 8 || cicp_idx > NUM_LS_CFGS)
          {
            return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
          }
          num_chnl_in = impgehd_cicp_get_num_ls[cicp_idx];
          ptr_chn_in_name = ia_cicp_idx_ls_set_map_tbl[cicp_idx];
        }

        for (j = 0; j < num_chnl_in; j++)
        {
          ptr_chn_in[index + j] = ptr_chn_in_name[j];
        }
        index += num_chnl_in;
      }
    }

    pstr_fc_params = &pstr_format_conv->fc_params;

    /* downmix mtx */
    pstr_fc_params->active_dmx_stft = &pstr_format_conv->active_dmx_stft;
    // pstr_fc_params->downmix_mat_freq = (FLOAT32 ***)&pstr_format_conv->downmix_mat_freq_ptr[0];
    pstr_fc_params->downmix_mat = (FLOAT32 **)&pstr_format_conv->downmix_mat_ptr[0];
    pstr_fc_state->stft_out_buf = (FLOAT32 **)&pstr_format_conv->stft_output_buf_ptr[0];
    pstr_fc_state->stft_in_buf = (FLOAT32 **)&pstr_format_conv->stft_input_buf_ptr[0];

    for (i = 0; i < FC_MAX_CHANNELS; i++)
    {
      pstr_fc_state->stft_in_buf[i] = &pstr_format_conv->stft_input_buf[i * FC_STFT_FRAME];
      pstr_fc_params->downmix_mat[i] = &pstr_format_conv->downmix_mat[i * FC_MAX_CHANNELS];
      pstr_fc_state->stft_out_buf[i] = &pstr_format_conv->stft_output_buf[i * FC_STFT_FRAME];
    }

    pstr_fc_params->active_dmx_stft->downmix_mat =
        (FLOAT32 ***)&pstr_format_conv->active_downmix_mat_ptr[0];
    for (i = 0; i < FC_MAX_CHANNELS; i++)
    {
      pstr_fc_params->active_dmx_stft->downmix_mat[i] =
          (FLOAT32 **)&pstr_format_conv->active_downmix_mat_ch_ptr[i * FC_MAX_CHANNELS];

      for (j = 0; j < FC_MAX_CHANNELS; j++)
      {
        pstr_fc_params->active_dmx_stft->downmix_mat[i][j] =
            (FLOAT32 *)&pstr_format_conv
                ->active_downmix_mat_freq_ptr[(i * FC_MAX_CHANNELS * FC_BANDS) + j * FC_BANDS];
      }
    }

    pstr_fc_params->data_state = &pstr_format_conv->format_conv_params_int;

    if (p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx > 0)
    {
      spk_layout_cicp_idx = p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx;
    }
  }
  if (!ds_flag)
  {
    WORD32 cicp_idx = spk_layout_cicp_idx;
    if (cicp_idx == 0 || cicp_idx == 8 || cicp_idx > NUM_LS_CFGS)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
    }
    num_out_chn = impgehd_cicp_get_num_ls[cicp_idx];
    ptr_ch_out = ia_cicp_idx_ls_set_map_tbl[cicp_idx];

    num_chnl_in = pstr_audio_specific_config->channel_configuration;
    pstr_fc_state->fc_params = pstr_fc_params;

    pstr_fc_params->samp_rate = p_obj_mpegh_dec->p_state_mpeghd->p_config->ui_samp_freq;
    pstr_fc_params->out_ch = &ptr_ch_out[0];
    pstr_fc_params->in_ch = &ptr_chn_in[0];
    pstr_fc_params->num_in_ch = num_chnl_in;
    pstr_fc_params->num_out_ch = num_out_chn;
    FLOAT32 *ptr_scratch_1 = &pstr_format_converter_scratch->fc_data[0];
    ia_cicp_ls_geo_str *azi_elev_array = &pstr_format_converter_scratch->str_azi_elev[0];

    error_num = impeghd_format_conv_init_data(ptr_scratch_1, azi_elev_array, pstr_fc_params,
                                              &pstr_audio_specific_config->ref_spk_layout);
    if (error_num == IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_CHANNEL_NUM)
    {
      pWORD8 ptr_scratch = (pWORD8)p_obj_mpegh_dec->p_state_mpeghd->mpeghd_scratch_mem_v;
      ia_obj_ren_dec_state_struct *pstr_obj_renderer = (ia_obj_ren_dec_state_struct *)ptr_scratch;
      ptr_scratch += sizeof(*pstr_obj_renderer);
      ia_oam_dec_config_struct *p_obj_md_cfg = (ia_oam_dec_config_struct *)ptr_scratch;

      memset(&pstr_obj_renderer->str_obj_md_dec_state, 0, sizeof(*p_obj_md_cfg));

      pstr_obj_renderer->str_obj_md_dec_state.p_obj_md_cfg = p_obj_md_cfg;
      pstr_obj_renderer->str_obj_md_dec_state.num_objects = num_out_chn;
      pstr_obj_renderer->pstr_local_setup = &pstr_decoder->str_local_setup_interaction;
      pstr_obj_renderer->cicp_out_idx = cicp_idx;

      error_num = impeghd_obj_renderer_dec_init(pstr_obj_renderer,
                                                &pstr_audio_specific_config->ref_spk_layout);
      if (error_num != IA_MPEGH_DEC_NO_ERROR)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;
      }

      WORD32 num_speakers_lcl, num_speakers_out;
      WORD32 num_lfes_lcl, num_lfes_out;
      WORD32 chn, out_chn;
      const WORD32 *in_channel_names, *out_channel_names;
      ia_cicp_ls_geo_str *ptr_cicp_ls_geometry[CICP_MAX_NUM_LS];
      ia_cicp_ls_geo_str *ptr_cicp_out_geometry[CICP_MAX_NUM_LS];
      impeghd_cicpidx_2_ls_geometry(
          pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx,
          (const ia_cicp_ls_geo_str **)&ptr_cicp_ls_geometry[0], &num_speakers_lcl, &num_lfes_lcl,
          (const WORD32 **)&in_channel_names);
      if (error_num != IA_MPEGH_DEC_NO_ERROR)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;
      }

      impeghd_cicpidx_2_ls_geometry(
          cicp_idx, (const ia_cicp_ls_geo_str **)&ptr_cicp_out_geometry[0], &num_speakers_out,
          &num_lfes_out, (const WORD32 **)&out_channel_names);
      if (error_num != IA_MPEGH_DEC_NO_ERROR)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;
      }
      for (chn = 0; chn < num_speakers_lcl; chn++)
      {
        pstr_obj_renderer->str_obj_md_dec_state.gain_descaled[chn] = 1;
        pstr_obj_renderer->str_obj_md_dec_state.elevation_descaled[chn] =
            ptr_cicp_ls_geometry[chn]->ls_elevation;
        pstr_obj_renderer->str_obj_md_dec_state.azimuth_descaled[chn] =
            ptr_cicp_ls_geometry[chn]->ls_azimuth;
        pstr_obj_renderer->str_obj_md_dec_state.radius_descaled[chn] = 1;
        pstr_obj_renderer->str_obj_md_dec_state.spread_depth_descaled[chn] = 0;
        pstr_obj_renderer->str_obj_md_dec_state.spread_height_descaled[chn] = 0;
        pstr_obj_renderer->str_obj_md_dec_state.spread_width_descaled[chn] = 0;
        error_num = impegh_obj_ren_vbap_process(pstr_obj_renderer, chn);
        if (error_num != IA_MPEGH_DEC_NO_ERROR)
        {
          return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;
        }
        for (out_chn = 0; out_chn < num_out_chn; out_chn++)
        {
          pstr_fc_params->downmix_mat[chn][out_chn] = pstr_obj_renderer->final_gains[out_chn];
        }
      }

      error_num = impeghd_format_conv_map_lfes(
          pstr_fc_params, (const ia_cicp_ls_geo_str **)&ptr_cicp_ls_geometry[0],
          (const ia_cicp_ls_geo_str **)&ptr_cicp_out_geometry[0]);
      if (error_num != IA_MPEGH_DEC_NO_ERROR)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;
      }
      error_num = impeghd_format_conv_post_proc_dmx_mtx(pstr_fc_params);
      if (error_num != IA_MPEGH_DEC_NO_ERROR)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;
      }
    }
    else if (error_num != IA_MPEGH_DEC_NO_ERROR)
    {
      return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;
    }

    if (pstr_usac_config->str_usac_dec_config.downmix_ext_config_present)
    {
      UWORD32 mtx_id;
      pWORD8 ptr_scratch = (pWORD8)p_obj_mpegh_dec->p_state_mpeghd->mpeghd_scratch_mem_v;
      ia_dmx_mtx_params *pstr_dmx_params = (ia_dmx_mtx_params *)ptr_scratch;
      ptr_scratch += sizeof(*pstr_dmx_params);

      if (p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx > 0)
      {
        pstr_dmx_params->cicp_out_idx = p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx;
      }
      else
      {
        pstr_dmx_params->cicp_out_idx = 0;
      }
      pstr_dmx_params->num_out_ch = impgehd_cicp_get_num_ls[pstr_dmx_params->cicp_out_idx];
      pstr_dmx_params->num_in_ch = pstr_audio_specific_config->channel_configuration;
      pstr_dmx_params->spk_layout = &pstr_audio_specific_config->ref_spk_layout;
      pstr_dmx_params->cicp_in_idx =
          pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx;
      pstr_dmx_params->samp_freq = pstr_audio_specific_config->sampling_frequency;

      ia_core_coder_memset(pstr_dmx_params->dmx_mtx, MAX_NUM_SPEAKERS * MAX_NUM_SPEAKERS);

      for (mtx_id = 0; mtx_id < pstr_usac_config->str_usac_dec_config.dmx_cfg.downmix_id_count;
           mtx_id++)
      {
        if (pstr_dmx_params->cicp_out_idx ==
                pstr_usac_config->str_usac_dec_config.dmx_cfg.dmx_matrix[mtx_id]
                    .cicp_spk_layout_idx &&
            pstr_usac_config->str_usac_dec_config.dmx_cfg.dmx_matrix[mtx_id]
                    .dmx_matrix_len_bits[0] != 0)
        {
          ia_bit_buf_struct dmx_bitbuf;
          ia_core_coder_create_init_bit_buf(
              &dmx_bitbuf,
              (UWORD8 *)pstr_usac_config->str_usac_dec_config.dmx_cfg.dmx_matrix[mtx_id]
                  .downmix_matrix[0],
              (pstr_usac_config->str_usac_dec_config.dmx_cfg.dmx_matrix[mtx_id]
                   .dmx_matrix_len_bits[0] +
               7) /
                  8);
          dmx_bitbuf.xmpeghd_jmp_buf = p_obj_mpegh_dec->p_state_mpeghd->xmpeghd_jmp_buf;
          impeghd_decode_downmix_matrix(pstr_fc_params, pstr_dmx_params, &dmx_bitbuf,
                                        ptr_scratch);
        }
      }
    }

    error_num = impeghd_format_conv_dmx_init(pstr_fc_params->active_dmx_stft, pstr_fc_params);

    if (error_num != IA_MPEGH_DEC_NO_ERROR)
      error_num = IA_MPEGH_FORMAT_CONV_INIT_FATAL_INIT_FAIL;

    *num_channel_out = p_obj_mpegh_dec->mpeghd_config.ui_n_channels = num_out_chn;
  }

  return error_num;
}

/* audio pre roll frame parsing*/
/**
 *  ia_core_coder_audio_preroll_parsing
 *
 *  \brief Parses Pre roll data.
 *
 *  \param [in,out] pstr_dec_data        Pointer to decoder data structure.
 *  \param [in]     conf_buf             Pointer to buffer containing config data.
 *  \param [in,out] preroll_units        Pointer to number of preroll units buffer.
 *  \param [in,out] preroll_frame_offset Pointer to preroll frame offset.
 *  \param [in,out] config_len           Pointer to config_len.
 *
 *  \return IA_ERRORCODE Error code if any processing errors are there.
 *
 */
static IA_ERRORCODE ia_core_coder_audio_preroll_parsing(ia_dec_data_struct *pstr_dec_data,
                                                        UWORD8 *conf_buf, WORD32 *preroll_units,
                                                        WORD32 *preroll_frame_offset,
                                                        WORD32 *config_len)
{
  ia_bit_buf_struct *temp_buff = (ia_bit_buf_struct *)&(pstr_dec_data->dec_bit_buf);
  WORD32 ext_ele_present = 0, ext_ele_use_dflt_len = 0, ext_ele_payload_len = 0, read_word = 0,
         num_pre_roll_frames = 0;
  *config_len = 0;
  WORD32 len, frame = 0;

  if (pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config.str_usac_dec_config
          .usac_element_type[0] == ID_USAC_EXT)
  {
    read_word = ia_core_coder_show_bits_buf(temp_buff, 3);
    ext_ele_present = (read_word >> 1) & 0x1;
    ext_ele_use_dflt_len = read_word & 0x1;

    if (1 == ext_ele_present)
    {
      if (ext_ele_use_dflt_len != 0)
      {
        return IA_MPEGH_DEC_NO_ERROR;
      }

      ia_core_coder_skip_bits_buf(temp_buff, 3);

      ext_ele_payload_len = ia_core_coder_read_bits_buf(temp_buff, 8);

      if (ext_ele_payload_len == ESC_VAL_BITS_8)
      {
        ext_ele_payload_len = ia_core_coder_read_bits_buf(temp_buff, 16);
        ext_ele_payload_len = (UWORD32)((WORD32)ext_ele_payload_len + ESC_VAL_BITS_8 - 2);
      }
      *config_len = ia_core_coder_read_bits_buf(temp_buff, 4);
      if (*config_len == ESC_VAL_BITS_4)
      {
        *config_len = ia_core_coder_read_bits_buf(temp_buff, 4);
        if (*config_len == ESC_VAL_BITS_4)
        {
          *config_len = ia_core_coder_read_bits_buf(temp_buff, 8);
          *config_len += ESC_VAL_BITS_4;
        }
        *config_len += ESC_VAL_BITS_4;
      }

      for (len = 0; len < *config_len; len++)
        conf_buf[len] = (UWORD8)ia_core_coder_read_bits_buf(temp_buff, 8);

      ia_core_coder_skip_bits_buf(temp_buff, 2);

      // escapedValue(2, 4, 0);
      num_pre_roll_frames = ia_core_coder_read_bits_buf(temp_buff, 2);
      if (num_pre_roll_frames == ESC_VAL_BITS_2)
      {
        num_pre_roll_frames = ia_core_coder_read_bits_buf(temp_buff, 4);
        num_pre_roll_frames += ESC_VAL_BITS_2;
      }

      if (num_pre_roll_frames > MAX_AUDIO_PREROLLS)
      {
        return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_PRE_ROLLS;
      }

      for (frame = 0; frame < num_pre_roll_frames; frame++)
      {
        WORD32 au_len = 0; // escapedValued(16,16,0)
        au_len = ia_core_coder_read_bits_buf(temp_buff, 16);
        if (au_len == ESC_VAL_BITS_16)
        {
          au_len = ia_core_coder_read_bits_buf(temp_buff, 16);
          au_len += ESC_VAL_BITS_16;
        }
        preroll_frame_offset[frame] = temp_buff->size - temp_buff->cnt_bits;

        temp_buff->ptr_read_next += au_len;
        if ((au_len * 8) > temp_buff->cnt_bits)
        {
          temp_buff->cnt_bits = 0;
          return IA_MPEGH_DEC_INIT_FATAL_UNEXPECTED_ERROR;
        }
        temp_buff->cnt_bits -= au_len * 8;
      }
    }
  }
  *preroll_units = num_pre_roll_frames;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_mdp_dec_process
 *
 *  \brief Metadata PreProcessing decoder main function
 *
 *  \param [in]     pstr_asc         Pointer to audio specific config structure
 *  \param [in,out] pstr_dec_data    Pointer to decoder data structure
 *  \param [in]     num_out_channels Pointer to number of output channels info
 *  \param [in]     scratch_mem      Pointer to scratch memory.
 *  \param [in]     preset_id        Preset id value from command line.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE impeghd_mdp_dec_process(ia_audio_specific_config_struct *pstr_asc,
                                     ia_dec_data_struct *pstr_dec_data, WORD32 num_out_channels,
                                     FLOAT32 *scratch_mem, WORD32 preset_id)
{
  WORD32 channel, grp, mbr, cond, ccfl, num_decoded_groups, num_elements = 0;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;

  FLOAT32 *audio_in_buff[MAX_NUM_CHANNELS], *audio_out_buff[MAX_NUM_CHANNELS];

  ia_mae_audio_scene_info *ptr_mae_asi = &pstr_asc->str_mae_asi;

  ccfl = pstr_dec_data->str_usac_data.ccfl;
  num_decoded_groups = ptr_mae_asi->num_groups;

  for (channel = 0; channel < num_out_channels; channel++)
  {
    audio_out_buff[channel] = scratch_mem + (channel * ccfl);
    ia_core_coder_memset(audio_out_buff[channel], ccfl);
    audio_in_buff[channel] = &pstr_dec_data->str_usac_data.time_sample_vector[channel][0];
  }

  for (grp = 0; grp < num_decoded_groups; grp++)
  {
    num_elements += ptr_mae_asi->group_definition[grp].group_num_members;
  }

  /* Element interaction related processing has to happen here*/
  if (ptr_mae_asi->ei_present || pstr_dec_data->str_element_interaction.ei_data_present ||
      pstr_dec_data->str_scene_displacement.scene_dspl_data_present)
  {
    /* Element interaction related processing has to happen here*/
    err_code = impeghd_mdp_dec_ei_process(pstr_asc, pstr_dec_data, num_out_channels,
                                          (WORD8 *)scratch_mem, &preset_id);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  /* Gain Interactivity processing block*/
  if (-1 != preset_id)
  {
    WORD32 pr_grp_idx = -1;
    WORD32 grp_idx = -1;
    for (grp = 0; grp < num_decoded_groups; grp++)
    {
      if (ptr_mae_asi->group_presets_definition[grp].group_id == preset_id)
      {
        pr_grp_idx = grp;
        break;
      }
    }
    if (pr_grp_idx == -1)
    {
      /* Invalid Preset Id */
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_PRESET_ID;
    }
    for (grp = 0; grp < num_decoded_groups; grp++)
    {
      WORD32 grp_id = ptr_mae_asi->group_definition[grp].group_id;
      WORD32 member_id = 0;
      WORD32 num_members = ptr_mae_asi->group_definition[grp].group_num_members;
      WORD32 has_conjuct_mbrs;
      for (cond = 0; cond < ptr_mae_asi->group_presets_definition[pr_grp_idx].num_conditions;
           cond++)
      {
        if (ptr_mae_asi->group_presets_definition[pr_grp_idx].reference_id[cond] == grp_id)
        {
          grp_idx = cond;
          break;
        }
      }
      if (grp_idx == -1)
      {
        /* Invalid Group Id */
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_GRP_ID;
      }
      if (0 == ptr_mae_asi->group_presets_definition[pr_grp_idx].cond_on_off[grp_idx])
      {
        has_conjuct_mbrs = ptr_mae_asi->group_definition[grp].has_conjunct_members;
      }
      else
      {
        has_conjuct_mbrs = ptr_mae_asi->group_definition[cond].has_conjunct_members;
      }

      for (mbr = 0; mbr < num_members; mbr++)
      {
        if (0 == has_conjuct_mbrs)
        {
          member_id = ptr_mae_asi->group_definition[grp].metadata_ele_id[mbr];
        }
        else
        {
          member_id = ptr_mae_asi->group_definition[grp].start_id + mbr;
        }

        if (member_id < 0 || member_id >= num_out_channels)
        {
          return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_CHANNELS;
        }
        if (0 != ptr_mae_asi->group_presets_definition[pr_grp_idx].cond_on_off[grp_idx])
        {
          ia_core_coder_mem_cpy(audio_in_buff[member_id], audio_out_buff[member_id], ccfl);
        }
        else
        {
          ia_core_coder_memset(audio_out_buff[member_id], ccfl);
        }
      }
    }
  }
  else
  {
    for (grp = 0; grp < num_decoded_groups; grp++)
    {
      WORD32 num_members = ptr_mae_asi->group_definition[grp].group_num_members;
      WORD32 member_id = 0;

      for (mbr = 0; mbr < num_members; mbr++)
      {
        if (0 == ptr_mae_asi->group_definition[grp].has_conjunct_members)
        {
          member_id = ptr_mae_asi->group_definition[grp].metadata_ele_id[mbr];
        }
        else
        {
          member_id = ptr_mae_asi->group_definition[grp].start_id + mbr;
        }
        if (0 != ptr_mae_asi->group_definition[grp].default_on_off)
        {
          if (member_id < 0 || member_id >= num_out_channels)
          {
            return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_CHANNELS;
          }

          ia_core_coder_mem_cpy(audio_in_buff[member_id], audio_out_buff[member_id], ccfl);
        }
      }
    }
  }
  for (channel = 0; channel < num_out_channels; channel++)
  {
    ia_core_coder_mem_cpy(audio_out_buff[channel], audio_in_buff[channel], ccfl);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_intracoded_prod_meta_data_frame
 *
 *  \brief Intracoded production metadata frame data parsing
 *
 *  \param [in]      ptr_bit_buf_str          Pointer to bit buffer structure
 *  \param [in,out]  pstr_audio_specific_cfg  Pointer to audio specific config structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE
impeghd_intracoded_prod_meta_data_frame(ia_bit_buf_struct *ptr_bit_buf_str,
                                        ia_audio_specific_config_struct *pstr_audio_specific_cfg)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 obj, num_audio_obj;
  ia_signals_3d *ptr_signals_3d = &pstr_audio_specific_cfg->str_usac_config.signals_3d;
  ia_prod_meta_data_struct *ptr_prod_meta_data =
      &pstr_audio_specific_cfg->str_usac_config.str_prod_metat_data;

  num_audio_obj = (WORD32)ptr_signals_3d->num_audio_obj;
  if (0 == num_audio_obj)
  {
    ptr_prod_meta_data->position_distance[0] = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 9);
  }
  else
  {
    ptr_prod_meta_data->fixed_distance = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 1);
    if (0 == ptr_prod_meta_data->fixed_distance)
    {
      ptr_prod_meta_data->common_distance = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 1);
      if (0 == ptr_prod_meta_data->common_distance)
      {
        for (obj = 0; obj < num_audio_obj; obj++)
        {
          ptr_prod_meta_data->position_distance[obj] =
              ia_core_coder_read_bits_buf(ptr_bit_buf_str, 9);
        }
      }
      else
      {
        ptr_prod_meta_data->default_distance = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 9);
      }
    }
    else
    {
      ptr_prod_meta_data->default_distance = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 9);
    }
  }
  return err_code;
}
/**
 *  impeghd_single_dyn_prod_md_frame
 *
 *  \brief Single dynamic production metadata frame data parsing
 *
 *  \param [in]      ptr_bit_buf_str      Pointer to bit buffer structure
 *  \param [in,out]  ptr_prod_meta_data   Pointer to production metadata structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE impeghd_single_dyn_prod_md_frame(ia_bit_buf_struct *ptr_bit_buf_str,
                                              ia_prod_meta_data_struct *ptr_prod_meta_data)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 read_bits;

  if (ptr_prod_meta_data->flag_dist_absolute)
  {
    if (0 == ptr_prod_meta_data->fixed_distance)
    {
      ptr_prod_meta_data->flag_dist = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 1);
      if (ptr_prod_meta_data->flag_dist)
      {
        ptr_prod_meta_data->nbit_dist = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 3);
        read_bits = ptr_prod_meta_data->nbit_dist + 2;
        ptr_prod_meta_data->position_bits_dist_diff =
            ia_core_coder_read_bits_buf(ptr_bit_buf_str, read_bits);
      }
    }
  }
  else
  {
    if (0 == ptr_prod_meta_data->fixed_distance)
    {
      ptr_prod_meta_data->position_dist = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 9);
    }
  }
  return err_code;
}
/**
 *  impeghd_dyn_prod_meta_data_frame
 *
 *  \brief Dynamic production metadata frame data parsing
 *
 *  \param [in]      ptr_bit_buf_str           Pointer to bit buffer structure
 *  \param [in,out]  pstr_audio_specific_cfg   Pointer to audio specific config structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE
impeghd_dyn_prod_meta_data_frame(ia_bit_buf_struct *ptr_bit_buf_str,
                                 ia_audio_specific_config_struct *pstr_audio_specific_cfg)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 obj;
  WORD32 num_audio_obj;
  ia_signals_3d *ptr_signals_3d = &pstr_audio_specific_cfg->str_usac_config.signals_3d;
  ia_prod_meta_data_struct *ptr_prod_meta_data =
      &pstr_audio_specific_cfg->str_usac_config.str_prod_metat_data;

  ptr_prod_meta_data->flag_dist_absolute = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 1);

  if (ptr_prod_meta_data->obj_meta_data_present)
  {
    num_audio_obj = (WORD32)ptr_signals_3d->num_audio_obj;
    for (obj = 0; obj < num_audio_obj; obj++)
    {
      err_code = impeghd_single_dyn_prod_md_frame(ptr_bit_buf_str, ptr_prod_meta_data);
      if (err_code)
      {
        return err_code;
      }
    }
  }
  return err_code;
}

/**
 *  impeghd_prod_meta_data_frame
 *
 *  \brief Production metadata frame data parsing
 *
 *  \param [in]      ptr_bit_buf_str           Pointer to bit buffer structure
 *  \param [in,out]  pstr_audio_specific_cfg   Pointer to audio specific config structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE
impeghd_prod_meta_data_frame(ia_bit_buf_struct *ptr_bit_buf_str,
                             ia_audio_specific_config_struct *pstr_audio_specific_cfg)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 grp;

  ia_signals_3d *ptr_signals_3d = &pstr_audio_specific_cfg->str_usac_config.signals_3d;
  ia_prod_meta_data_struct *ptr_prod_meta_data =
      &pstr_audio_specific_cfg->str_usac_config.str_prod_metat_data;

  ptr_prod_meta_data->payload_length = (ptr_bit_buf_str->size >> 3);

  for (grp = 0; grp < ptr_signals_3d->num_obj_based_groups; grp++)
  {
    if (ptr_prod_meta_data->has_object_distance[grp])
    {
      ptr_prod_meta_data->has_intracoded_data = ia_core_coder_read_bits_buf(ptr_bit_buf_str, 1);
      if (0 == ptr_prod_meta_data->has_intracoded_data)
      {
        err_code = impeghd_dyn_prod_meta_data_frame(ptr_bit_buf_str, pstr_audio_specific_cfg);
      }
      else
      {
        err_code =
            impeghd_intracoded_prod_meta_data_frame(ptr_bit_buf_str, pstr_audio_specific_cfg);
      }
      if (err_code)
      {
        return err_code;
      }
    }
  }
  return err_code;
}
/**
 *  impeghd_product_meta_data_process
 *
 *  \brief Metadata frame process
 *
 *  \param [in]      pstr_usac_dec_cfg         Pointer to USAC decoder config structure
 *  \param [in,out]  pstr_audio_specific_cfg   Pointer to audio specific config structure
 *  \param [in]      ele_idx                   Element index
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE
impeghd_product_meta_data_process(ia_usac_decoder_config_struct *pstr_usac_dec_cfg,
                                  ia_audio_specific_config_struct *pstr_audio_specific_cfg,
                                  WORD32 ele_idx)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 bit_buf_size = pstr_usac_dec_cfg->usac_ext_gain_payload_len[ele_idx];

  UWORD8 *ptr_bit_buf_base = &pstr_usac_dec_cfg->usac_ext_gain_payload_buf[ele_idx][0];

  ia_bit_buf_struct bit_buf;
  jmp_buf meta_data_process_jump_buff;
  err_code = setjmp(meta_data_process_jump_buff);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_METADATA_BYTES;
  }

  if (bit_buf_size > 0)
  {
    ia_core_coder_create_init_bit_buf(&bit_buf, ptr_bit_buf_base, bit_buf_size);
    bit_buf.xmpeghd_jmp_buf = &meta_data_process_jump_buff;
    impeghd_prod_meta_data_frame(&bit_buf, pstr_audio_specific_cfg);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_earcon_obj_md_dec_ren_process
 *
 *  \brief Object metadata renderer main process function.
 *
 *  \param [in,out] pstr_dec_data     Pointer to decoder data structure.
 *  \param [out]    ptr_out_buf       Pointer to output buffer.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_earcon_obj_md_dec_ren_process(ia_dec_data_struct *pstr_dec_data,
                                                          FLOAT32 *ptr_out_buf)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 obj;
  WORD32 num_objects;
  WORD32 ccfl;

  FLOAT32 *ptr_in_buf;

  ia_audio_specific_config_struct *pstr_asc =
      &pstr_dec_data->str_frame_data.str_audio_specific_config;
  ia_pcm_data_config *pstr_pcm_data_config = &pstr_asc->str_mae_asi.pcm_data_config;
  ia_earcon_info *pstr_earcon_info = &pstr_asc->str_mae_asi.earcon_info;
  ia_obj_ren_dec_state_struct *pstr_obj_ren_dec_state = &pstr_dec_data->str_obj_ren_dec_state;
  ia_oam_dec_state_struct *pstr_obj_md_dec_state = &pstr_obj_ren_dec_state->str_obj_md_dec_state;

  num_objects = pstr_obj_md_dec_state->num_objects;
  ccfl = ia_pcm_frame_size_tbl[pstr_pcm_data_config->pcm_frame_size_idx];
  ptr_in_buf = &pstr_pcm_data_config->pcm_sample[0][0];

  ia_core_coder_memset(ptr_out_buf, (ccfl * pstr_obj_ren_dec_state->num_cicp_speakers));

  pstr_obj_md_dec_state->sub_frame_number = 1;
  pstr_obj_md_dec_state->num_objects = (pstr_pcm_data_config->bs_num_pcm_signals + 1);
  for (obj = 0; obj < pstr_obj_md_dec_state->num_objects; obj++)
  {
    pstr_obj_md_dec_state->radius[obj] = pstr_earcon_info->earcon_distance[0];
    pstr_obj_md_dec_state->elevation[obj] = pstr_earcon_info->earcon_elevation[0];
    pstr_obj_md_dec_state->azimuth[obj] = pstr_earcon_info->earcon_azimuth[0];

    if (pstr_earcon_info->earcon_has_gain)
    {
      pstr_obj_md_dec_state->gain[obj] = pstr_earcon_info->earcon_gain[0];
    }
    pstr_obj_md_dec_state->azimuth_descaled[obj] =
        (FLOAT32)pstr_obj_md_dec_state->azimuth[obj] * 1.5f;
    pstr_obj_md_dec_state->elevation_descaled[obj] =
        (FLOAT32)pstr_obj_md_dec_state->elevation[obj] * 3.0f;
    pstr_obj_md_dec_state->radius_descaled[obj] =
        (FLOAT32)pow(2.0f, (pstr_obj_md_dec_state->radius[obj] / 3.0f)) / 2.0f;
    pstr_obj_md_dec_state->gain_descaled[obj] =
        (FLOAT32)pow(10.0f, (pstr_obj_md_dec_state->gain[obj] - 32.0f) / 40.0f);
  }
  err_code = impeghd_obj_renderer_dec(pstr_obj_ren_dec_state, ptr_in_buf, ptr_out_buf, ccfl);
  if (err_code)
  {
    return err_code;
  }
  pstr_obj_md_dec_state->num_objects = num_objects;
  return err_code;
}

/**
 *  impeghd_obj_md_dec_ren_process
 *
 *  \brief Object metadata renderer main process function.
 *
 *  \param [in,out] pstr_dec_data     Pointer to decoder data structure.
 *  \param [out] ptr_out_buf       Pointer to output buffer.
 *  \param [in,out] num_out_channels  Pointer to number of output channels parameter.
 *  \param [in]  ch_offset         Offest for audio object output.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE
impeghd_obj_md_dec_ren_process(ia_dec_data_struct *pstr_dec_data, FLOAT32 *ptr_out_buf,
                               WORD32 *num_out_channels, WORD32 ch_offset)
{
  ia_audio_specific_config_struct *pstr_audio_specific_cfg =
      &pstr_dec_data->str_frame_data.str_audio_specific_config;
  ia_obj_ren_dec_state_struct *pstr_obj_ren_dec_state = &pstr_dec_data->str_obj_ren_dec_state;
  ia_oam_dec_state_struct *pstr_obj_md_dec_state = &pstr_obj_ren_dec_state->str_obj_md_dec_state;
  ia_oam_dec_config_struct *p_obj_md_cfg = &pstr_audio_specific_cfg->str_usac_config.obj_md_cfg;
  FLOAT32 *ptr_in_buf = &pstr_dec_data->str_usac_data.time_sample_vector[ch_offset][0];
  WORD32 ccfl = pstr_dec_data->str_usac_data.output_samples;
  WORD32 i;

  memset(ptr_out_buf, 0, (ccfl * sizeof(FLOAT32) * pstr_obj_ren_dec_state->num_cicp_speakers));

  if ((p_obj_md_cfg->frame_length != 256) && (p_obj_md_cfg->frame_length != 512) &&
      (p_obj_md_cfg->frame_length != 1024))
  {
    return IA_MPEGH_OAM_EXE_FATAL_UNSUPPORTED_FRAMELENGTH;
  }

  if (p_obj_md_cfg->frame_length >= ccfl)
  {
    pstr_obj_md_dec_state->sub_frame_number = 1;
    impeghd_obj_renderer_dec(pstr_obj_ren_dec_state, ptr_in_buf, ptr_out_buf, ccfl);
  }
  else
  {
    WORD32 num_iter = (ccfl / p_obj_md_cfg->frame_length);
    pstr_obj_md_dec_state->sub_frame_number = 0;
    for (i = 0; i < num_iter; i++)
    {
      if (pstr_obj_md_dec_state->sub_frame_obj_md_present[i])
      {
        pstr_obj_md_dec_state->sub_frame_number++;
        if (pstr_obj_md_dec_state->sub_frame_number > num_iter)
        {
          pstr_obj_md_dec_state->sub_frame_number =
              pstr_obj_md_dec_state->sub_frame_number % num_iter;
        }
      }
      else if (i == 0)
      {
        pstr_obj_md_dec_state->sub_frame_number = num_iter;
      }

      impeghd_obj_renderer_dec(pstr_obj_ren_dec_state,
                               ptr_in_buf + i * (p_obj_md_cfg->frame_length),
                               ptr_out_buf + i * (p_obj_md_cfg->frame_length), ccfl);
    }
  }

  *num_out_channels = pstr_obj_ren_dec_state->num_cicp_speakers;
  return 0;
}
/**
 *  impegh_dec_peak_limiter
 *
 *  \brief  peak limiter process function
 *  \param [in,out] ptr_config   Pointer to decoder config structure
 *  \param [in,out] pstr_dec_data   Pointer to decoder data structure
 *  \param [out] ptr_out_buf     Pointer to output buffer
 *  \param [out]  ptr_audio_buff Pointer to audio buffer
 *  \param [in]  frame_size    frame length.
 *
 *
 *
 */
VOID impegh_dec_peak_limiter(ia_mpegh_dec_config_struct *ptr_config,
                             ia_dec_data_struct *pstr_dec_data, WORD32 *num_out_channels,
                             FLOAT32 *ptr_out_buf, FLOAT32 **ptr_audio_buff, WORD32 frame_size)
{
  WORD32 i, j;
  pstr_dec_data->str_peak_limiter.num_channels = *num_out_channels;
  if (ptr_config->resample_output == 0)
  {
    for (i = 0; i < *num_out_channels; i++)
    {
      for (j = 0; j < frame_size; j++)
      {
        ptr_out_buf[j * (*num_out_channels) + i] = ptr_audio_buff[i][j];
      }
    }
  }
  else
  {
    FLOAT32 *ptr_temp = ptr_out_buf;
    ptr_out_buf += ((*num_out_channels) * (frame_size));
    for (i = 0; i < *num_out_channels; i++)
    {
      for (j = 0; j < frame_size; j++)
      {
        ptr_out_buf[j * (*num_out_channels) + i] = ptr_temp[i * frame_size + j];
      }
    }
  }
  impeghd_peak_limiter_process(&pstr_dec_data->str_peak_limiter, ptr_out_buf, frame_size);
  if (ptr_config->resample_output == 0)
  {
    for (i = 0; i < (*num_out_channels); i++)
    {
      for (j = 0; j < frame_size; j++)
      {
        ptr_audio_buff[i][j] = ptr_out_buf[j * (*num_out_channels) + i];
      }
    }
  }
  else
  {
    FLOAT32 *ptr_temp = ptr_out_buf;
    ptr_out_buf -= ((*num_out_channels) * (frame_size));
    for (i = 0; i < (*num_out_channels); i++)
    {
      for (j = 0; j < frame_size; j++)
      {
        ptr_out_buf[i * frame_size + j] = ptr_temp[j * (*num_out_channels) + i];
      }
    }
  }

  return;
}
/**
 *  impegh_dec_ln_pl_process
 *
 *  \brief Loudness normalizer and peak limiter process function
 *
 *  \param [in,out] pstr_dec_data      Pointer to decoder data structure
 *  \param [in]     pstr_mpegh_dec     Pointer to mpegh state structure
 *  \param [in]     gain_db_loudness_normalization Gain value to be applied in dB
 *  \param [in,out] ptr_num_out_chnl   Pointer to number of output channels info
 *
 *  \return IA_ERRORCODE Processing error code if any.
 *
 */
IA_ERRORCODE impegh_dec_ln_pl_process(ia_dec_data_struct *pstr_dec_data,
                                      ia_mpegh_dec_state_struct *pstr_mpegh_dec,
                                      FLOAT32 gain_db_loudness_normalization,
                                      WORD32 *ptr_num_out_chnl)
{
  ia_output_scratch *pstr_output_scratch;
  pstr_output_scratch = (ia_output_scratch *)pstr_mpegh_dec->mpeghd_scratch_mem_v;
  ia_mpegh_dec_config_struct *ptr_config = pstr_mpegh_dec->p_config;
  FLOAT32 *ptr_out_buf = &pstr_output_scratch->scratch_out_buf[0];
  FLOAT32 *ptr_audio_buff[MAX_NUM_CHANNELS];
  FLOAT32 gain;
  WORD32 i, j;
  WORD32 peak_lim_flag = (pstr_mpegh_dec->p_config->out_samp_freq == 48000) ? 1 : 0;
  WORD32 frame_length = 1024;
  peak_lim_flag = peak_lim_flag & (!pstr_mpegh_dec->p_config->resample_output);

  for (i = 0; i < *ptr_num_out_chnl; i++)
  {
    ptr_audio_buff[i] = &pstr_dec_data->str_usac_data.time_sample_vector[i][0];
  }
  if (ptr_config->resample_output)
  {
    FLOAT32 *ptr_resamp_scratch =
        &ptr_out_buf[(*ptr_num_out_chnl) * ptr_config->output_framelength];
    for (i = 0; i < *ptr_num_out_chnl; i++)
    {
      pstr_dec_data->str_resampler[i].fac_down = ptr_config->fac_down;
      pstr_dec_data->str_resampler[i].fac_up = ptr_config->fac_up;
      pstr_dec_data->str_resampler[i].input_length = pstr_dec_data->str_usac_data.ccfl;
      pstr_dec_data->str_resampler[i].output_length = ptr_config->output_framelength;
      impeghd_resample(&pstr_dec_data->str_resampler[i], ptr_audio_buff[i],
                       &ptr_out_buf[i * ptr_config->output_framelength], ptr_resamp_scratch);
    }
    frame_length = (frame_length * ptr_config->fac_up) / ptr_config->fac_down;
    pstr_mpegh_dec->ui_out_bytes =
        frame_length * (*ptr_num_out_chnl) * (ptr_config->ui_pcm_wdsz >> 3);
  }
  if (gain_db_loudness_normalization != 0.0f)
  {

    gain = (FLOAT32)pow(10.0, gain_db_loudness_normalization / 20.0);
    for (i = 0; i < *ptr_num_out_chnl; i++)
    {
      for (j = 0; j < frame_length; j++)
      {
        ptr_audio_buff[i][j] = ia_mul_flt(ptr_audio_buff[i][j], gain);
      }
    }
  }
  if (peak_lim_flag)
  {
    impegh_dec_peak_limiter(ptr_config, pstr_dec_data, ptr_num_out_chnl, ptr_out_buf,
                            &ptr_audio_buff[0], frame_length);
  }
  return 0;
}

/**
 *  impeghd_uni_drc_dec_process
 *
 *  \brief UniDRC decoder main process function
 *
 *  \param [in]     pstr_usac_dec_cfg Pointer to USAC decoder config structure.
 *  \param [in,out] pstr_dec_data     Pointer to USAC decoder data structure.
 *  \param [in]     ele_idx           Extension element index.
 *  \param [in]     dom_swi_flag      Domain switcher active flag.
 *  \param [in]     stft_fft_len      Short time fourier transfor length
 *  \param [in,out] dec_inst          DRC Decoder instance index
 *
 *  \return IA_ERRORCODE Processing error code if any else 0.
 *
 */
IA_ERRORCODE
impeghd_uni_drc_dec_process(ia_usac_decoder_config_struct *pstr_usac_dec_cfg,
                            ia_dec_data_struct *pstr_dec_data, WORD32 ele_idx,
                            UWORD32 dom_swi_flag, WORD32 stft_fft_len, WORD32 dec_inst)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  WORD32 num_channels;
  WORD32 bit_buf_size = pstr_usac_dec_cfg->usac_ext_gain_payload_len[ele_idx];

  UWORD8 *ptr_bit_buf_base = &pstr_usac_dec_cfg->usac_ext_gain_payload_buf[ele_idx][0];

  FLOAT32 *audio_buff[MAX_NUM_CHANNELS];
  FLOAT32 *audio_buff_imag[MAX_NUM_CHANNELS];

  ia_audio_specific_config_struct *pstr_asc =
      &pstr_dec_data->str_frame_data.str_audio_specific_config;
  ia_drc_payload_struct *pstr_drc_dec_payload = &pstr_dec_data->str_drc_payload;
  ia_drc_config *p_drc_dec_cfg = &pstr_asc->str_usac_config.uni_drc_cfg;
  ia_bit_buf_struct bit_buf_str;
  ia_bit_buf_struct *it_bit_buff = &bit_buf_str;

  num_channels = pstr_drc_dec_payload->str_gain_dec[dec_inst].audio_num_chan;

  if (bit_buf_size > 0)
  {
    ia_core_coder_create_init_bit_buf(it_bit_buff, ptr_bit_buf_base, bit_buf_size);
    bit_buf_str.xmpeghd_jmp_buf = pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf;
    err_code = impd_drc_read_uni_drc_gain(&pstr_drc_dec_payload->str_drc_gain, p_drc_dec_cfg,
                                          it_bit_buff, &pstr_drc_dec_payload->str_bitstream_dec);

    if (err_code != IA_MPEGH_DEC_NO_ERROR)
      return (err_code);
  }

  if (0 == dom_swi_flag)
  {
    for (i = 0; i < num_channels; i++)
    {
      audio_buff[i] = &pstr_dec_data->str_usac_data.time_sample_vector[i][0];
    }

    if (pstr_drc_dec_payload->pstr_drc_config == NULL)
    {
      return IA_MPEGD_DRC_INIT_FATAL_UNEXPECTED_ERROR;
    }

    err_code = impd_drc_td_process(
        audio_buff, &pstr_drc_dec_payload->str_gain_dec[dec_inst],
        pstr_drc_dec_payload->pstr_drc_config, &pstr_drc_dec_payload->str_drc_gain,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.loud_norm_gain_db,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.boost,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.compress,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.drc_characteristic_target);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return IA_MPEGD_DRC_EXE_FATAL_PROCESS_FAIL;
    }
  }
  else
  {
    for (i = 0; i < num_channels; i++)
    {
      audio_buff[i] = &pstr_dec_data->str_usac_data.time_sample_stft[i][0];
      audio_buff_imag[i] = &pstr_dec_data->str_usac_data.time_sample_stft[i][2 * stft_fft_len];
    }

    err_code = impd_drc_fd_process(
        audio_buff, audio_buff_imag, &pstr_drc_dec_payload->str_gain_dec[dec_inst],
        pstr_drc_dec_payload->pstr_drc_config, &pstr_drc_dec_payload->str_drc_gain,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.loud_norm_gain_db,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.boost,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.compress,
        pstr_drc_dec_payload->str_select_proc.uni_drc_sel_proc_output.drc_characteristic_target);
    if (err_code)
      return IA_MPEGD_DRC_EXE_FATAL_PROCESS_FAIL;
  }

  return err_code;
}

/**
 *  ia_core_coder_dec_process_frame_zero
 *
 *  \brief MPEG-H 3D Audio Low Complexity Profile decoder processing frame zero
 *
 *  \param [in,out] temp_handle     Pointer to decoder API handle.
 *  \param [out]    num_channel_out Pointer to number of output channels info.
 *
 *  \return IA_ERRORCODE Processing error if any else 0.
 *
 */
IA_ERRORCODE ia_core_coder_dec_process_frame_zero(VOID *temp_handle, WORD32 *num_channel_out)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 hoa_present = 0, oam_present = 0, tmp;
  WORD32 suitable_tracks = 1, delay, grp, channel, ele_idx, num_signals, drc_effect_type,
         num_elements;
  WORD32 ch_cnt = 0, sig_grp = 0, target_loudness, loudness_norm_flag, mct_cnt = 0;

  ia_mpegh_dec_api_struct *handle = (ia_mpegh_dec_api_struct *)temp_handle;
  ia_mpegh_dec_state_struct *mpegh_dec_handle = handle->p_state_mpeghd;
  ia_audio_specific_config_struct *pstr_asc =
      (ia_audio_specific_config_struct *)mpegh_dec_handle->ia_audio_specific_config;
  ia_dec_data_struct *pstr_dec_data;
  ia_signals_3d *ia_signals_3da = &pstr_asc->str_usac_config.signals_3d;
  ia_usac_decoder_config_struct *str_usac_dec_config =
      &pstr_asc->str_usac_config.str_usac_dec_config;

  num_elements = pstr_asc->str_usac_config.str_usac_dec_config.num_elements;
  target_loudness = handle->mpeghd_config.ui_target_loudness;
  loudness_norm_flag = handle->mpeghd_config.ui_loud_norm_flag;
  drc_effect_type = (handle->mpeghd_config.ui_effect_type);

  pstr_dec_data = (ia_dec_data_struct *)mpegh_dec_handle->pstr_dec_data;
  memset(&pstr_dec_data->str_usac_data, 0, sizeof(pstr_dec_data->str_usac_data));
  tmp = pstr_asc->channel_configuration;
  pstr_asc->cicp_spk_layout_idx = handle->mpeghd_config.ui_cicp_layout_idx;

  for (ele_idx = 0; ele_idx < num_elements; ele_idx++)
  {
    switch (str_usac_dec_config->usac_element_type[ele_idx])
    {
    case ID_USAC_CPE:
      ch_cnt += 2;
      break;
    case ID_USAC_SCE:
    case ID_USAC_LFE:
      ch_cnt++;
      break;
    }

    num_signals = 0;
    for (grp = 0; grp < MAX_NUM_SIGNALGROUPS; grp++)
    {
      num_signals += ia_signals_3da->num_sig[grp];
      if (num_signals > ch_cnt)
      {
        sig_grp = grp;
        break;
      }
    }

    if (str_usac_dec_config->usac_element_type[ele_idx] == ID_USAC_EXT &&
        str_usac_dec_config->ia_ext_ele_payload_type[ele_idx] == ID_MPEGH_EXT_ELE_MCT)
    {
      const WORD32 n_ch = ia_signals_3da->num_sig[sig_grp];
      WORD32 start_chn = 0;
      jmp_buf *xmpeghd_jmp_buf = mpegh_dec_handle->xmpeghd_jmp_buf;
      for (grp = 0; grp < sig_grp; grp++)
      {
        start_chn += ia_signals_3da->num_sig[grp];
      }

      err_code = impeghd_mc_init(str_usac_dec_config, xmpeghd_jmp_buf, n_ch, start_chn, ele_idx,
                                 mct_cnt);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }
      mct_cnt++;
    }
  }
  suitable_tracks = ia_core_coder_frm_data_init(pstr_asc, pstr_dec_data);
  pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf = mpegh_dec_handle->xmpeghd_jmp_buf;

  pstr_asc->channel_configuration = tmp;
  for (ele_idx = 0; ele_idx < (WORD32)str_usac_dec_config->num_elements; ele_idx++)
  {
    if (str_usac_dec_config->usac_element_type[ele_idx] == ID_USAC_EXT)
    {
      switch (str_usac_dec_config->ia_ext_ele_payload_type[ele_idx])
      {
      case ID_EXT_ELE_UNI_DRC:
      {
        pstr_dec_data->str_drc_payload.pstr_drc_config =
            &pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config.uni_drc_cfg;
        err_code =
            impeghd_uni_drc_dec_init(pstr_asc, pstr_dec_data, target_loudness, loudness_norm_flag,
                                     drc_effect_type, mpegh_dec_handle->p_config->i_preset_id,
                                     mpegh_dec_handle->p_config->ui_cicp_layout_idx, ele_idx);
        if (err_code)
        {
          return err_code;
        }
      }
      break;
      /* Object Metadata Decoding and rendering */
      case ID_MPEGH_EXT_ELE_OAM:
      {
        memset(&pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state, 0,
               sizeof(pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state));
        memset(&pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config.obj_md_cfg
                    .is_screen_rel_obj[0],
               0, sizeof(pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config
                             .obj_md_cfg.is_screen_rel_obj));

        pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state.p_obj_md_cfg =
            &pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config.obj_md_cfg;
        pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state.num_objects =
            pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config.signals_3d
                .num_audio_obj;
        pstr_dec_data->str_obj_ren_dec_state.pstr_local_setup =
            &pstr_dec_data->str_local_setup_interaction;
        err_code = impeghd_obj_renderer_dec_init(
            &pstr_dec_data->str_obj_ren_dec_state,
            &pstr_dec_data->str_frame_data.str_audio_specific_config.ref_spk_layout);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }
        else
        {
          oam_present = 1;
          *num_channel_out = pstr_dec_data->str_obj_ren_dec_state.num_cicp_speakers;
        }
      }
      break;
      case ID_MPEGH_EXT_ELE_HOA:
      {
        /* Speaker index */
        impeghd_hoa_dec_struct *dec_handle_t_ptr = &pstr_dec_data->str_hoa_dec_handle;
        WORD32 spk_idx;
        UWORD32 sampling_frequency = pstr_asc->str_usac_config.usac_sampling_frequency;

        if (handle->mpeghd_config.ui_cicp_layout_idx > 0)
        {
          spk_idx = handle->mpeghd_config.ui_cicp_layout_idx;
        }
        else
        {
          spk_idx = pstr_asc->ref_spk_layout.cicp_spk_layout_idx;
        }

        // Creare HOA decoder handle .c
        memset(dec_handle_t_ptr, 0, sizeof(*dec_handle_t_ptr));
        dec_handle_t_ptr->frame_length = MAXIMUM_FRAME_SIZE;

        dec_handle_t_ptr->ia_hoa_config =
            &(((((ia_audio_specific_config_struct
                      *)((ia_mpegh_dec_state_struct *)((handle)->pp_mem_mpeghd[PERSIST_IDX]))
                     ->ia_audio_specific_config)
                    ->str_usac_config)
                   .str_usac_dec_config)
                  .str_hoa_config);

        err_code = impeghd_hoa_dec_init(
            dec_handle_t_ptr, spk_idx, &pstr_asc->ref_spk_layout, sampling_frequency,
            mpegh_dec_handle->mpeghd_scratch_mem_v, str_usac_dec_config->mpegh_profile_lvl);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return IA_MPEGH_HOA_INIT_FATAL_RENDER_MATRIX_INIT_FAILED;
        }
        else
        {
          hoa_present = 1;
          *num_channel_out = dec_handle_t_ptr->hoa_renderer.num_out_channels;
        }
      }
      break;
      }
    }
  }

  if (pstr_asc->str_usac_config.signals_3d.format_converter_enable == 1)
  {
    impeghd_format_conv_init(handle, pstr_asc, num_channel_out, 0);
  }

  if (pstr_asc->str_usac_config.signals_3d.domain_switcher_enable == 1)
  {
    impeghd_format_conv_init(handle, pstr_asc, num_channel_out, 1);
  }
  if (suitable_tracks <= 0)
  {
    return IA_MPEGH_DEC_EXE_FATAL_NO_SUITABLE_TRACK;
  }

  pstr_dec_data = (ia_dec_data_struct *)mpegh_dec_handle->pstr_dec_data;

  mpegh_dec_handle->delay_in_samples = 0;
  if (mpegh_dec_handle->decode_create_done == 0)
  {
    err_code = ia_core_coder_decode_create(handle, pstr_dec_data,
                                           pstr_dec_data->str_frame_data.scal_out_select + 1);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }
  pstr_dec_data->dec_bit_buf.max_size = handle->p_mem_info_mpeghd[IA_MEMTYPE_INPUT].ui_size << 3;
  if (pstr_asc->str_usac_config.signals_3d.format_converter_enable == 0 && hoa_present == 0 &&
      oam_present == 0)
    *num_channel_out = pstr_dec_data->str_frame_data.scal_out_num_channels; // check
  if (hoa_present == 1)
  {
    // pstr_dec_data->str_frame_data.scal_out_num_channels = *num_channel_out;
    for (channel = 0; channel < *num_channel_out; channel++)
    {
      pstr_asc->str_usac_config.str_usac_dec_config.num_output_chns[channel] = channel;
    }
  }

  {
    WORD32 samp_freq = pstr_asc->str_usac_config.usac_sampling_frequency;
    if ((pstr_asc->str_usac_config.usac_sampling_frequency !=
         handle->p_state_mpeghd->p_config->out_samp_freq) &&
        handle->p_state_mpeghd->p_config->resample_output)
    {
      samp_freq = handle->p_state_mpeghd->p_config->out_samp_freq;
    }
    else
    {
      handle->p_state_mpeghd->p_config->resample_output = 0;
    }
    err_code = impeghd_peak_limiter_init(&pstr_dec_data->str_peak_limiter, *num_channel_out,
                                         samp_freq, &pstr_dec_data->str_peak_limiter.buffer[0]);
    delay = pstr_dec_data->str_peak_limiter.attack_time_samples;
    // BINAURAL_DEBUG
    if (handle->mpeghd_config.ui_binaural_flag)
    {
      delay = 0;
    }
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
    if (pstr_asc->str_usac_config.usac_sampling_frequency == 48000)
    {
      mpegh_dec_handle->delay_in_samples += delay;
    }
    else
    {
      mpegh_dec_handle->delay_in_samples += 0;
    }
    if (pstr_asc->str_usac_config.signals_3d.format_converter_enable == 1)
    {
      mpegh_dec_handle->delay_in_samples += FC_DELAY;
    }
    if (pstr_asc->str_usac_config.signals_3d.domain_switcher_enable == 1)
    {
      mpegh_dec_handle->delay_in_samples += FC_DELAY;
      if (hoa_present)
      {
        mpegh_dec_handle->delay_in_samples += 768;
      }
    }
  }
  if (*num_channel_out > MAX_NUM_CHANNELS)
    return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_dec_ext_ele_proc
 *
 *  \brief MPEG-H 3D Audio Low Complexity Profile extentsion elements processing
 *
 *  \param [in,out] temp_handle     Pointer to decoder API handle.
 *  \param [out]    num_channel_out Pointer to number of output channels info.
 *
 *  \return IA_ERRORCODE Processing error if any else 0.
 *
 */
IA_ERRORCODE ia_core_coder_dec_ext_ele_proc(VOID *temp_handle, WORD32 *num_channel_out)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 channel, ele, s, num_elements, ch_offset, num_samples_out;

  ia_mpegh_dec_api_struct *handle = (ia_mpegh_dec_api_struct *)temp_handle;
  ia_mpegh_dec_state_struct *mpegh_dec_handle = handle->p_state_mpeghd;
  ia_audio_specific_config_struct *pstr_asc =
      (ia_audio_specific_config_struct *)mpegh_dec_handle->ia_audio_specific_config;
  ia_dec_data_struct *pstr_dec_data = (ia_dec_data_struct *)mpegh_dec_handle->pstr_dec_data;
  ia_signals_3d *ia_signals_3da = &pstr_asc->str_usac_config.signals_3d;
  ia_usac_config_struct *pstr_usac_config =
      &pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config;
  FLOAT32 *ptr_out_buf_hoa = NULL;
  FLOAT32 *ptr_out_buf = NULL;

  ch_offset = pstr_usac_config->signals_3d.num_ch;
  num_samples_out = pstr_dec_data->str_usac_data.output_samples;

  if (ch_offset > MAX_NUM_CHANNELS)
  {
    return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_CHANNELS;
  }
  ia_usac_decoder_config_struct *pstr_usac_dec_cfg = &pstr_usac_config->str_usac_dec_config;
  num_elements = pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config
                     .str_usac_dec_config.num_elements;
  pstr_dec_data->str_enh_obj_md_frame.oam_has_been_decoded = 0;
  for (ele = 0; ele < num_elements; ele++)
  {
    if (pstr_usac_dec_cfg->usac_ext_ele_payload_present[ele])
    {
      WORD32 bit_buf_size = pstr_usac_dec_cfg->usac_ext_gain_payload_len[ele];
      UWORD8 *ptr_bit_buf_base = &pstr_usac_dec_cfg->usac_ext_gain_payload_buf[ele][0];
      switch (pstr_usac_dec_cfg->ia_ext_ele_payload_type[ele])
      {
      case ID_MPEGH_EXT_ELE_ENHANCED_OBJ_METADATA:
      {
        ia_audio_specific_config_struct *pstr_audio_specific_cfg =
            &pstr_dec_data->str_frame_data.str_audio_specific_config;
        ia_oam_dec_state_struct *pstr_obj_md_dec_state =
            &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
        ia_bit_buf_struct bit_buf_str;
        ia_core_coder_create_init_bit_buf(&bit_buf_str, ptr_bit_buf_base, bit_buf_size);
        bit_buf_str.xmpeghd_jmp_buf = pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf;
        pstr_dec_data->str_enh_obj_md_frame.p_enh_obj_md_cfg =
            &pstr_audio_specific_cfg->str_usac_config.enh_obj_md_cfg;
        err_code = impeghd_enh_obj_md_frame(&pstr_dec_data->str_enh_obj_md_frame, &bit_buf_str,
                                            pstr_obj_md_dec_state->num_objects,
                                            pstr_dec_data->str_usac_data.usac_independency_flg);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }
        pstr_dec_data->str_enh_obj_md_frame.oam_has_been_decoded = 1;
        break;
      }
      case ID_MPEGH_EXT_ELE_OAM:
      {
        WORD32 ccfl = pstr_dec_data->str_usac_data.output_samples;
        ia_bit_buf_struct bit_buf_str;
        ia_core_coder_create_init_bit_buf(&bit_buf_str, ptr_bit_buf_base, bit_buf_size);
        bit_buf_str.xmpeghd_jmp_buf = pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf;
        if (bit_buf_size > 0)
        {
          ia_audio_specific_config_struct *pstr_audio_specific_cfg =
              &pstr_dec_data->str_frame_data.str_audio_specific_config;
          ia_oam_dec_config_struct *p_obj_md_cfg =
              &pstr_audio_specific_cfg->str_usac_config.obj_md_cfg;
          ia_obj_ren_dec_state_struct *pstr_obj_ren_dec_state =
              &pstr_dec_data->str_obj_ren_dec_state;
          ia_oam_dec_state_struct *pstr_obj_md_dec_state =
              &pstr_obj_ren_dec_state->str_obj_md_dec_state;

          if ((p_obj_md_cfg->frame_length != 256) && (p_obj_md_cfg->frame_length != 512) &&
              (p_obj_md_cfg->frame_length != 1024))
          {
            return IA_MPEGH_OAM_EXE_FATAL_UNSUPPORTED_FRAMELENGTH;
          }

          if (p_obj_md_cfg->frame_length < ccfl)
          {
            WORD32 sub_frm;
            WORD32 num_sub_frms = (ccfl / p_obj_md_cfg->frame_length);
            pstr_obj_md_dec_state->sub_frame_number = 0;
            for (sub_frm = 0; sub_frm < num_sub_frms; sub_frm++)
            {
              pstr_obj_md_dec_state->sub_frame_obj_md_present[sub_frm] =
                  ia_core_coder_read_bits_buf(&bit_buf_str, OAM_FLAG_BITS);
              if (pstr_obj_md_dec_state->sub_frame_obj_md_present[sub_frm])
              {
                pstr_obj_md_dec_state->sub_frame_number++;
                err_code = impeghd_obj_md_dec(pstr_obj_md_dec_state, &bit_buf_str);
                if (err_code != IA_MPEGH_DEC_NO_ERROR)
                {
                  return err_code;
                }
              }
            }
          }
          else
          {
            pstr_obj_md_dec_state->sub_frame_number = 1;
            err_code = impeghd_obj_md_dec(pstr_obj_md_dec_state, &bit_buf_str);
            if (err_code != IA_MPEGH_DEC_NO_ERROR)
            {
              return err_code;
            }
          }
        }
        break;
      }
      }
    }
  }

  if (mpegh_dec_handle->p_config->extrn_rend_flag && ia_signals_3da->num_ch > 0)
  {
    ia_write_bit_buf_struct str_bit_buf;
    impeghd_create_write_bit_buffer(&str_bit_buf, mpegh_dec_handle->p_config->ptr_ch_md_bit_buf,
                                    768, 1);
    impeghd_write_ch_meta_data_for_ext_ren(&str_bit_buf, pstr_usac_config, ia_signals_3da,
                                           mpegh_dec_handle->p_config->ui_cicp_layout_idx);
    mpegh_dec_handle->p_config->ch_md_payload_length = (str_bit_buf.cnt_bits + 7) >> 3;
  }

  if (mpegh_dec_handle->p_config->extrn_rend_flag)
  {
    WORD32 pcm_sample;
    WORD8 *ptr_ext_ren_pcm = (WORD8 *)handle->mpeghd_config.ptr_ext_ren_pcm_buf;
    WORD32 num_cc_channels = ia_signals_3da->num_ch + ia_signals_3da->num_audio_obj +
                             ia_signals_3da->num_hoa_transport_ch;
    for (s = 0; s < pstr_dec_data->str_usac_data.ccfl; s++)
    {
      for (channel = 0; channel < num_cc_channels; channel++)
      {
        pcm_sample =
            (WORD32)(pstr_dec_data->str_usac_data.time_sample_vector[channel][s] * 256.0f);
        *ptr_ext_ren_pcm++ = (WORD32)pcm_sample & 0xff;
        *ptr_ext_ren_pcm++ = ((WORD32)pcm_sample >> 8) & 0xff;
        *ptr_ext_ren_pcm++ = ((WORD32)pcm_sample >> 16) & 0xff;
      }
    }
    handle->mpeghd_config.obj_offset = 0;
    handle->mpeghd_config.hoa_offset = 0;
    handle->mpeghd_config.pcm_data_length = 3 * AUDIO_CODEC_FRAME_SIZE_MAX * num_cc_channels;
  }

  if (pstr_dec_data->str_frame_data.str_audio_specific_config.str_mae_asi.asi_present)
  {
    err_code = impeghd_mdp_dec_process(
        &pstr_dec_data->str_frame_data.str_audio_specific_config, pstr_dec_data,
        pstr_dec_data->str_frame_data.str_audio_specific_config.channel_configuration,
        (FLOAT32 *)(mpegh_dec_handle->mpeghd_scratch_mem_v), handle->mpeghd_config.i_preset_id);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }
  for (ele = 0; ele < num_elements; ele++)
  {
    if ((ID_EXT_ELE_UNI_DRC == pstr_usac_dec_cfg->ia_ext_ele_payload_type[ele]) &&
        (pstr_usac_dec_cfg->usac_ext_ele_payload_present[ele]))
    {
      // domain switcher
      UWORD32 dom_swi_flag = pstr_asc->str_usac_config.signals_3d.domain_switcher_enable;
      FLOAT32 *ptr_scratch = (FLOAT32 *)mpegh_dec_handle->mpeghd_scratch_mem_v;
      if (dom_swi_flag)
      {
        for (channel = 0; channel < MAX_NUM_CHANNELS; channel++)
        {
          pstr_dec_data->str_usac_data.time_sample_stft[channel] =
              ptr_scratch + (channel * FC_STFT_FRAMEx2 * 4);
        }
        ptr_scratch += (MAX_NUM_CHANNELS * FC_STFT_FRAMEx2 * 4);

        ia_format_converter_scratch *pstr_ds_scratch = (ia_format_converter_scratch *)ptr_scratch;
        impeghd_domain_switcher_process(handle, num_channel_out, 1,
                                        &pstr_ds_scratch->scratch_ds[0]);
        pstr_dec_data->str_drc_payload.str_gain_dec[0].ia_drc_params_struct.gain_delay_samples =
            128;
        err_code = impeghd_uni_drc_dec_process(pstr_usac_dec_cfg, pstr_dec_data, ele,
                                               dom_swi_flag, FC_STFT_FRAMEx2, 0);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }

        pstr_ds_scratch = (ia_format_converter_scratch *)ptr_scratch;
        err_code = impeghd_domain_switcher_process(handle, num_channel_out, 0,
                                                   &pstr_ds_scratch->scratch_ds[0]);
      }
      else
      {
        err_code = impeghd_uni_drc_dec_process(pstr_usac_dec_cfg, pstr_dec_data, ele,
                                               dom_swi_flag, FC_STFT_FRAMEx2, 0);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }
      }
    }
  }

  for (ele = 0; ele < num_elements; ele++)
  {
    if (pstr_usac_dec_cfg->usac_ext_ele_payload_present[ele])
    {
      switch (pstr_usac_dec_cfg->ia_ext_ele_payload_type[ele])
      {
      case ID_MPEGH_EXT_ELE_OAM:
      {
        if (ptr_out_buf == NULL)
        {
          ptr_out_buf = (FLOAT32 *)mpegh_dec_handle->mpeghd_scratch_mem_v;
        }
        err_code = impeghd_obj_md_dec_ren_process(pstr_dec_data, ptr_out_buf, num_channel_out,
                                                  ch_offset);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }

        if (mpegh_dec_handle->p_config->extrn_rend_flag)
        {
          ia_write_bit_buf_struct str_bit_buf;
          mpegh_dec_handle->p_config->obj_offset = ch_offset;
          impeghd_create_write_bit_buffer(&str_bit_buf,
                                          mpegh_dec_handle->p_config->ptr_oam_md_bit_buf, 768, 1);
          impeghd_write_oam_meta_data_for_ext_ren(
              &str_bit_buf, pstr_usac_config, &pstr_dec_data->str_obj_ren_dec_state,
              ia_signals_3da, &pstr_dec_data->str_enh_obj_md_frame);
          mpegh_dec_handle->p_config->oam_md_payload_length = (str_bit_buf.cnt_bits + 7) >> 3;
        }
        if (pstr_usac_config->signals_3d.num_ch != 0)
        {
          for (channel = 0; channel < *num_channel_out; channel++)
          {
            for (s = 0; s < num_samples_out; s++)
            {
              pstr_dec_data->str_usac_data.time_sample_vector[channel][s] =
                  ia_add_flt(pstr_dec_data->str_usac_data.time_sample_vector[channel][s],
                             ptr_out_buf[channel * num_samples_out + s]);
            }
          }
          ch_offset += pstr_usac_config->signals_3d.num_audio_obj;
        }
        else if (pstr_usac_config->signals_3d.num_hoa_transport_ch != 0)
        {
          if (ptr_out_buf_hoa == NULL)
          {
            ptr_out_buf_hoa = ptr_out_buf + *num_channel_out * num_samples_out;
          }
          else
          {
            for (channel = 0; channel < *num_channel_out; channel++)
            {
              for (s = 0; s < num_samples_out; s++)
              {
                pstr_dec_data->str_usac_data.time_sample_vector[channel][s] =
                    ia_add_flt(ptr_out_buf_hoa[channel * num_samples_out + s],
                               ptr_out_buf[channel * num_samples_out + s]);
              }
            }
          }
          ch_offset += pstr_usac_config->signals_3d.num_audio_obj;
        }
        else
        {
          for (channel = 0; channel < *num_channel_out; channel++)
          {
            ia_core_coder_mem_cpy(&ptr_out_buf[channel * num_samples_out],
                                  pstr_dec_data->str_usac_data.time_sample_vector[channel],
                                  num_samples_out);
          }
        }

        break;
      }
      case ID_MPEGH_EXT_ELE_HOA:
      {
        if (ptr_out_buf_hoa == NULL)
        {
          ptr_out_buf_hoa = (FLOAT32 *)mpegh_dec_handle->mpeghd_scratch_mem_v;
        }
        if (pstr_dec_data->str_hoa_dec_handle.ia_hoa_config == NULL)
        {
          return IA_MPEGH_HOA_INIT_FATAL_RENDER_MATRIX_INIT_FAILED;
        }
        if (pstr_dec_data->str_hoa_dec_handle.ptr_scratch == NULL)
        {
          WORD32 spk_idx;
          ia_usac_decoder_config_struct *str_usac_dec_config =
              &pstr_asc->str_usac_config.str_usac_dec_config;
          UWORD32 sampling_frequency = pstr_asc->str_usac_config.usac_sampling_frequency;

          if (handle->mpeghd_config.ui_cicp_layout_idx > 0)
          {
            spk_idx = handle->mpeghd_config.ui_cicp_layout_idx;
          }
          else
          {
            spk_idx = pstr_asc->ref_spk_layout.cicp_spk_layout_idx;
          }
          err_code = impeghd_hoa_dec_init(&pstr_dec_data->str_hoa_dec_handle, spk_idx,
                                          &pstr_asc->ref_spk_layout, sampling_frequency,
                                          mpegh_dec_handle->mpeghd_scratch_mem_v,
                                          str_usac_dec_config->mpegh_profile_lvl);
          if (err_code != IA_MPEGH_DEC_NO_ERROR)
          {
            return IA_MPEGH_HOA_INIT_FATAL_RENDER_MATRIX_INIT_FAILED;
          }
        }
        pstr_dec_data->str_hoa_frame_data.ptr_config_data = &pstr_usac_dec_cfg->str_hoa_config;
        err_code = impeghd_hoa_dec_main_process(
            pstr_usac_dec_cfg, pstr_dec_data, ele, ptr_out_buf_hoa, num_channel_out, ch_offset,
            pstr_asc->str_usac_config.signals_3d.domain_switcher_enable);

        if (pstr_usac_config->signals_3d.num_ch != 0)
        {
          for (channel = 0; channel < *num_channel_out; channel++)
          {
            for (s = 0; s < num_samples_out; s++)
            {
              pstr_dec_data->str_usac_data.time_sample_vector[channel][s] =
                  ia_add_flt(pstr_dec_data->str_usac_data.time_sample_vector[channel][s],
                             ptr_out_buf[channel * num_samples_out + s]);
            }
          }
          ch_offset += pstr_usac_config->signals_3d.num_hoa_transport_ch;
        }
        else if (pstr_usac_config->signals_3d.num_audio_obj != 0)
        {
          if (ptr_out_buf != NULL)
          {
            for (channel = 0; channel < *num_channel_out; channel++)
            {
              for (s = 0; s < num_samples_out; s++)
              {
                pstr_dec_data->str_usac_data.time_sample_vector[channel][s] =
                    ia_add_flt(ptr_out_buf_hoa[channel * num_samples_out + s],
                               ptr_out_buf[channel * num_samples_out + s]);
              }
            }
          }
          else
          {
            ptr_out_buf = ptr_out_buf_hoa + *num_channel_out * num_samples_out;
          }
          ch_offset += pstr_usac_config->signals_3d.num_hoa_transport_ch;
        }
        else
        {
          for (channel = 0; channel < *num_channel_out; channel++)
          {
            ia_core_coder_mem_cpy(&ptr_out_buf_hoa[channel * num_samples_out],
                                  pstr_dec_data->str_usac_data.time_sample_vector[channel],
                                  num_samples_out);
          }
        }
        if (mpegh_dec_handle->p_config->extrn_rend_flag)
        {
          ia_write_bit_buf_struct str_bit_buf;
          mpegh_dec_handle->p_config->hoa_offset = ch_offset;
          impeghd_create_write_bit_buffer(&str_bit_buf,
                                          mpegh_dec_handle->p_config->ptr_hoa_md_bit_buf, 768, 1);
          impeghd_write_hoa_meta_data_for_ext_ren(&str_bit_buf, pstr_usac_config, ia_signals_3da,
                                                  &pstr_asc->str_mae_asi);
          mpegh_dec_handle->p_config->hoa_md_payload_length = (str_bit_buf.cnt_bits + 7) >> 3;
        }
        break;
      }
      case ID_EXT_ELE_PROD_METADATA:
      {
        pstr_asc->str_usac_config.str_prod_metat_data.prod_metadata_present = 1;
        pstr_asc->str_usac_config.str_prod_metat_data.obj_meta_data_present =
            pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state.sub_frame_obj_md_present[0];
        err_code = impeghd_product_meta_data_process(pstr_usac_dec_cfg, pstr_asc, ele);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }
        break;
      }
      default:
        break;
      }
    }
  }

  return err_code;
}
/**
 *  ia_core_coder_dec_main
 *
 *  \brief MPEG-H 3D Audio Low Complexity Profile main decoder function.
 *
 *  \param [in,out] temp_handle     Pointer to decoder API handle.
 *  \param [in]     inbuffer        Pointer to input buffer.
 *  \param [out]    outbuffer       Pointer to output buffer.
 *  \param [out]    out_bytes       Pointer to number of output bytes variables.
 *  \param [in]     frames_done     Number of frames processed
 *  \param [in]     pcmsize         PCM size in bits.
 *  \param [out]    num_channel_out Pointer to number of output channels info.
 *
 *  \return IA_ERRORCODE Processing error if any else 0.
 *
 */
IA_ERRORCODE ia_core_coder_dec_main(VOID *temp_handle, WORD8 *inbuffer, WORD8 *outbuffer,
                                    WORD32 *out_bytes, WORD32 frames_done, WORD32 pcmsize,
                                    WORD32 *num_channel_out)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD preroll_units = -1;
  WORD32 access_units = 0;
  WORD8 config_change = 0;
  WORD32 preroll_counter = 0;
  WORD32 suitable_tracks = 1;
  WORD32 num_samples_out;
  WORD32 grp, channel, s, sig, cnt, elemIdx;
  WORD32 mhas_offset = 0;
  WORD32 target_loudness;
  WORD32 loudness_norm_flag;
  WORD32 drc_effect_type;
  WORD32 config_len = 0;

  UWORD8 config[288] = {0};
  WORD preroll_frame_offset[4] = {0};
  WORD32 *out_ch_map;

  ia_mpegh_dec_api_struct *handle = (ia_mpegh_dec_api_struct *)temp_handle;
  ia_mpegh_dec_state_struct *mpegh_dec_handle = handle->p_state_mpeghd;
  ia_audio_specific_config_struct *pstr_asc =
      (ia_audio_specific_config_struct *)mpegh_dec_handle->ia_audio_specific_config;
  ia_dec_data_struct *pstr_dec_data;
  ia_mhas_pac_info pac_info;
  ia_signals_3d *ia_signals_3da = &pstr_asc->str_usac_config.signals_3d;
  ia_usac_decoder_config_struct *str_usac_dec_config =
      &pstr_asc->str_usac_config.str_usac_dec_config;

  target_loudness = handle->mpeghd_config.ui_target_loudness;
  loudness_norm_flag = handle->mpeghd_config.ui_loud_norm_flag;
  drc_effect_type = (handle->mpeghd_config.ui_effect_type);

  out_ch_map = pstr_asc->str_usac_config.str_usac_dec_config.num_output_chns;

  if (((pstr_asc->channel_configuration == 0) && (ia_signals_3da->num_hoa_transport_ch == 0) &&
       (ia_signals_3da->num_audio_obj == 0)) ||
      (pstr_asc->channel_configuration > 16))
  {
    return IA_MPEGH_DEC_EXE_FATAL_INVALID_CHAN_CONFIG;
  }
  if (frames_done == 0)
  {
    return ia_core_coder_dec_process_frame_zero(temp_handle, num_channel_out);
  }

  {
    WORD32 tot_out_bytes = 0;
    pstr_dec_data = (ia_dec_data_struct *)mpegh_dec_handle->pstr_dec_data;

    pstr_dec_data->dec_bit_buf.ptr_bit_buf_base = (UWORD8 *)inbuffer;
    pstr_dec_data->dec_bit_buf.size = mpegh_dec_handle->ui_in_bytes << 3;
    pstr_dec_data->dec_bit_buf.ptr_bit_buf_end =
        (UWORD8 *)inbuffer + mpegh_dec_handle->ui_in_bytes - 1;
    pstr_dec_data->dec_bit_buf.ptr_read_next = (UWORD8 *)inbuffer;
    pstr_dec_data->dec_bit_buf.bit_pos = 7;
    pstr_dec_data->dec_bit_buf.cnt_bits = pstr_dec_data->dec_bit_buf.size;
    pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf = mpegh_dec_handle->xmpeghd_jmp_buf;
    if (pstr_dec_data->dec_bit_buf.size > pstr_dec_data->dec_bit_buf.max_size)
      pstr_dec_data->dec_bit_buf.max_size = pstr_dec_data->dec_bit_buf.size;
    /* audio pre roll frame parsing*/
    if (0 == handle->mpeghd_config.ui_raw_flag)
    {
      err_code =
          impeghd_mhas_parse(&pac_info, &pstr_asc->str_mae_asi, &pstr_dec_data->dec_bit_buf);
      if (err_code)
      {
        return err_code;
      }
    }
    if (pstr_asc->str_mae_asi.earcon_info.earcon_present &&
        pstr_asc->str_mae_asi.earcon_info.earcon_cicp_spk_idx[0] > 0)
    {
      err_code = impeghd_format_conv_earcon_init(handle, pstr_asc);
      if (err_code)
      {
        return err_code;
      }
    }
    if (pstr_asc->str_mae_asi.pcm_data_config.pcm_packet_type_present)
    {
      if ((SIZE_T)pstr_dec_data->dec_bit_buf.ptr_read_next ==
          (SIZE_T)pstr_dec_data->dec_bit_buf.ptr_bit_buf_end)
      {
        handle->p_state_mpeghd->i_bytes_consumed = ((WORD32)pstr_dec_data->dec_bit_buf.size) >> 3;
      }
      else
      {
        handle->p_state_mpeghd->i_bytes_consumed =
            (WORD32)(((((SIZE_T)pstr_dec_data->dec_bit_buf.ptr_read_next -
                        (SIZE_T)pstr_dec_data->dec_bit_buf.ptr_bit_buf_base))
                      << 3) +
                     7 - pstr_dec_data->dec_bit_buf.bit_pos + 7) >>
            3;
      }
      return IA_MPEGH_DEC_NO_ERROR;
    }
    if ((pstr_asc->str_mae_asi.asi_present == 1) && (!pstr_asc->str_mae_asi.asi_config_set) &&
        ((handle->mpeghd_config.ui_target_loudness_set) ||
         (pstr_dec_data->str_drc_payload.pstr_drc_config != NULL)))
    {
      memset(&pstr_dec_data->str_drc_payload.str_select_proc, 0,
             sizeof(pstr_dec_data->str_drc_payload.str_select_proc));
      pstr_dec_data->str_drc_payload.pstr_drc_config =
          &pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config.uni_drc_cfg;
      err_code =
          impeghd_uni_drc_dec_init(pstr_asc, pstr_dec_data, target_loudness, loudness_norm_flag,
                                   drc_effect_type, mpegh_dec_handle->p_config->i_preset_id,
                                   mpegh_dec_handle->p_config->ui_cicp_layout_idx, 0);
      if (err_code)
      {
        return err_code;
      }
      pstr_asc->str_mae_asi.asi_config_set = 1;
    }
    if (0 == handle->mpeghd_config.ui_raw_flag)
    {
      if (MHAS_PAC_TYP_MPEGH3DAFRAME != pac_info.packet_type &&
          MHAS_PAC_TYP_MARKER != pac_info.packet_type &&
          MHAS_PAC_TYP_MPEGH3DACFG != pac_info.packet_type)
      {
        return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_MHAS_PKT_TYPE;
      }
      if (MHAS_PAC_TYP_MPEGH3DACFG == pac_info.packet_type)
      {
        WORD32 bytes_consumed;

        if (pstr_asc->str_usac_config.str_usac_dec_config.preroll_flag != 0)
        {
          ia_core_coder_decoder_flush_api(handle);
        }
        err_code = impegh_3daudio_config_dec(mpegh_dec_handle, &bytes_consumed,
                                             &pstr_dec_data->dec_bit_buf);
        if (handle->p_state_mpeghd->frame_counter ==
            1) /* config bytes passed with 1st decode call */
        {
          if (err_code < 0)
          {
            if (err_code == (WORD32)IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX)
            {
              mpegh_dec_handle->i_bytes_consumed = bytes_consumed;
              return err_code;
            }

            return err_code;
          }

          if (err_code == IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES)
          {
            mpegh_dec_handle->i_bytes_consumed = bytes_consumed;
            return err_code;
          }

          mpegh_dec_handle->i_bytes_consumed = bytes_consumed;

          if (err_code == IA_MPEGH_DEC_NO_ERROR)
          {
            {
              WORD32 pcm_size = handle->mpeghd_config.ui_pcm_wdsz;
              WORD8 *inbuffer = handle->pp_mem_mpeghd[INPUT_IDX];
              WORD8 *outbuffer = handle->pp_mem_mpeghd[OUTPUT_IDX];
              WORD32 out_bytes = 0;
              WORD32 frames_done = 0;
              mpegh_dec_handle->decode_create_done = 0;

              err_code =
                  ia_core_coder_dec_main(handle, inbuffer, outbuffer, &out_bytes, frames_done,
                                         pcm_size, &handle->p_state_mpeghd->num_of_output_ch);
              if (err_code != IA_MPEGH_DEC_NO_ERROR)
              {
                return err_code;
              }
              // handle->p_state_mpeghd->frame_counter++;

              handle->mpeghd_config.ui_n_channels = handle->p_state_mpeghd->num_of_output_ch;
            }
            if (err_code == 0)
              handle->p_state_mpeghd->ui_init_done = 1;
            // return err;
          }
        }
        if (err_code == 0)
        {
          mpegh_dec_handle->ui_init_done = 1;
        }
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }
        if (pstr_asc->str_usac_config.str_usac_dec_config.preroll_flag != 0)
        {
          err_code = ia_core_coder_decode_create(
              handle, pstr_dec_data, pstr_dec_data->str_frame_data.scal_out_select + 1);
          if (err_code != IA_MPEGH_DEC_NO_ERROR)
          {
            return err_code;
          }
          *num_channel_out = pstr_dec_data->str_frame_data.scal_out_num_channels;
        }
        return IA_MPEGH_DEC_NO_ERROR;
      }
      mhas_offset = pstr_dec_data->dec_bit_buf.size - pstr_dec_data->dec_bit_buf.cnt_bits;
    }
    if (access_units == 0 && pstr_asc->str_usac_config.str_usac_dec_config.preroll_flag)
    {
      err_code = ia_core_coder_audio_preroll_parsing(pstr_dec_data, &config[0], &preroll_units,
                                                     &preroll_frame_offset[0], &config_len);
      if (err_code)
        return err_code;
      if (preroll_units == -1)
        preroll_units = 0;
      preroll_frame_offset[preroll_units] = mhas_offset;
    }
    else
    {
      preroll_frame_offset[0] = mhas_offset;
      config_len = 0;
    }

    do
    {

      if (config_len != 0)
      {
        if (0 != mpegh_dec_handle->preroll_config_present)
        {
          config_change =
              (memcmp(mpegh_dec_handle->preroll_config_prev, config, sizeof(config)) != 0) ? 1
                                                                                           : 0;
          if (config_change)
            memcpy(mpegh_dec_handle->preroll_config_prev, config, sizeof(config));
          else
          {
            // memset(preroll_frame_offset, 0, sizeof(preroll_frame_offset));
            preroll_units = -1;
            preroll_frame_offset[preroll_units + 1] = mhas_offset;
          }
        }
        else
        {
          memcpy(mpegh_dec_handle->preroll_config_prev, config, sizeof(config));
          mpegh_dec_handle->preroll_config_present = 1;
          preroll_units = -1;
          preroll_frame_offset[preroll_units + 1] = mhas_offset;
        }
      }

      if (config_len != 0 && config_change == 1)
      {
        /* updating the config parameters*/
        ia_bit_buf_struct config_bit_buf = {0};

        config_bit_buf.ptr_bit_buf_base = config;
        config_bit_buf.size = config_len << 3;
        config_bit_buf.ptr_read_next = config_bit_buf.ptr_bit_buf_base;
        config_bit_buf.ptr_bit_buf_end = (UWORD8 *)config + config_len;
        config_bit_buf.bit_pos = 7;
        config_bit_buf.cnt_bits = config_bit_buf.size;
        config_bit_buf.xmpeghd_jmp_buf = mpegh_dec_handle->xmpeghd_jmp_buf;

        for (elemIdx = 0; elemIdx < (WORD32)str_usac_dec_config->num_elements; elemIdx++)
        {
          if (str_usac_dec_config->usac_element_type[elemIdx] == ID_USAC_EXT &&
              str_usac_dec_config->ia_ext_ele_payload_type[elemIdx] == ID_MPEGH_EXT_ELE_HOA)
          {
            /* Speaker index */
            impeghd_hoa_dec_struct *dec_handle_t_ptr = &pstr_dec_data->str_hoa_dec_handle;
            WORD32 spk_idx;
            UWORD32 sampling_frequency = pstr_asc->str_usac_config.usac_sampling_frequency;

            if (handle->mpeghd_config.ui_cicp_layout_idx > 0)
            {
              spk_idx = handle->mpeghd_config.ui_cicp_layout_idx;
            }
            else
            {
              spk_idx = pstr_asc->ref_spk_layout.cicp_spk_layout_idx;
            }
            memset(dec_handle_t_ptr, 0, sizeof(impeghd_hoa_dec_struct));
            dec_handle_t_ptr->frame_length = MAXIMUM_FRAME_SIZE;
            dec_handle_t_ptr->ia_hoa_config =
                &(((((ia_audio_specific_config_struct
                          *)((ia_mpegh_dec_state_struct *)((handle)->pp_mem_mpeghd[PERSIST_IDX]))
                         ->ia_audio_specific_config)
                        ->str_usac_config)
                       .str_usac_dec_config)
                      .str_hoa_config);

            err_code = impeghd_hoa_dec_init(
                dec_handle_t_ptr, spk_idx, &pstr_asc->ref_spk_layout, sampling_frequency,
                mpegh_dec_handle->mpeghd_scratch_mem_v, str_usac_dec_config->mpegh_profile_lvl);
            if (IA_MPEGH_DEC_NO_ERROR != err_code)
            {
              return err_code;
            }
          }
        }
        if (suitable_tracks <= 0)
        {
          return IA_MPEGH_DEC_EXE_FATAL_NO_SUITABLE_TRACK;
        }

        err_code = ia_core_coder_mpegh_3da_config(&config_bit_buf, mpegh_dec_handle,
                                                  &pstr_asc->str_mae_asi);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }

        pstr_dec_data->str_frame_data.str_audio_specific_config.sampling_frequency =
            pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config
                .usac_sampling_frequency;
      }

      mpegh_dec_handle->p_config->discard_au_preroll = 1;

      pstr_dec_data->dec_bit_buf.ptr_bit_buf_base = (UWORD8 *)inbuffer;
      pstr_dec_data->dec_bit_buf.size = mpegh_dec_handle->ui_in_bytes << 3;
      pstr_dec_data->dec_bit_buf.ptr_bit_buf_end =
          (UWORD8 *)inbuffer + mpegh_dec_handle->ui_in_bytes - 1;
      pstr_dec_data->dec_bit_buf.ptr_read_next = (UWORD8 *)inbuffer;
      pstr_dec_data->dec_bit_buf.bit_pos = 7;
      pstr_dec_data->dec_bit_buf.cnt_bits = pstr_dec_data->dec_bit_buf.size;
      pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf = mpegh_dec_handle->xmpeghd_jmp_buf;

      if (preroll_frame_offset[access_units])
      {
        pstr_dec_data->dec_bit_buf.cnt_bits =
            pstr_dec_data->dec_bit_buf.size - preroll_frame_offset[access_units];
        pstr_dec_data->dec_bit_buf.bit_pos = 7 - preroll_frame_offset[access_units] % 8;
        pstr_dec_data->dec_bit_buf.ptr_read_next =
            pstr_dec_data->dec_bit_buf.ptr_read_next + (preroll_frame_offset[access_units] / 8);
      }

      if (!mpegh_dec_handle->decode_create_done)
        return IA_MPEGH_DEC_INIT_FATAL_ERROR;
      err_code = ia_core_coder_usac_process(pstr_dec_data, mpegh_dec_handle);

      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }

      num_samples_out = pstr_dec_data->str_usac_data.output_samples;

      err_code = ia_core_coder_dec_ext_ele_proc(temp_handle, num_channel_out);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }

      if (pstr_asc->str_usac_config.signals_3d.format_converter_enable == 1)
      {
        FLOAT32 *ptr_fc_scratch = (FLOAT32 *)mpegh_dec_handle->mpeghd_scratch_mem_v;
        for (channel = 0; channel < MAX_NUM_CHANNELS; channel++)
        {
          pstr_dec_data->str_usac_data.time_sample_stft[channel] =
              ptr_fc_scratch + channel * FC_STFT_FRAMEx2 * 4;
        }
        ptr_fc_scratch += (MAX_NUM_CHANNELS * FC_STFT_FRAMEx2 * 4);

        ia_format_converter_scratch *pstr_fc_scratch;
        pstr_fc_scratch = (ia_format_converter_scratch *)ptr_fc_scratch;
        err_code = impeghd_format_converter_process(handle, num_channel_out,
                                                    &pstr_fc_scratch->scratch_fc[0]);
      }
      if ((pstr_dec_data->str_drc_payload.num_sets[1] > 0 ||
           pstr_dec_data->str_drc_payload.num_sets[2] > 0))
      {
        pstr_dec_data->str_drc_payload.str_gain_dec[1].ia_drc_params_struct.gain_delay_samples =
            0;
        if (pstr_asc->str_usac_config.signals_3d.format_converter_enable == 1)
        {
          pstr_dec_data->str_drc_payload.str_gain_dec[1]
              .ia_drc_params_struct.gain_delay_samples += 256;
        }
        if (pstr_asc->str_usac_config.signals_3d.domain_switcher_enable == 1)
        {
          pstr_dec_data->str_drc_payload.str_gain_dec[1]
              .ia_drc_params_struct.gain_delay_samples += 256;
        }
        err_code = impeghd_uni_drc_dec_process(&pstr_asc->str_usac_config.str_usac_dec_config,
                                               pstr_dec_data, 0, 0, FC_STFT_FRAMEx2, 1);
      }
      if ((pstr_asc->str_mae_asi.earcon_info.earcon_cicp_spk_idx[0] > 0) &&
          (!pstr_asc->str_mae_asi.earcon_info.earcon_fc_init_done))
      {
        err_code = impeghd_format_conv_earcon_init(handle, pstr_asc);
        if (err_code)
        {
          return err_code;
        }
        pstr_asc->str_mae_asi.earcon_info.earcon_fc_init_done = 1;
      }

      if (pstr_asc->str_mae_asi.pcm_data_config.pcm_config_present)
      {
        pstr_asc->str_mae_asi.pcm_data_config.pcm_config_present = 0;
        FLOAT32 *ptr_ec_scratch = (FLOAT32 *)mpegh_dec_handle->mpeghd_scratch_mem_v;
        if (pstr_asc->str_mae_asi.earcon_info.earcon_cicp_spk_idx[0] > 0)
        {
          ia_pcm_data_config *pstr_pcm_data = &pstr_asc->str_mae_asi.pcm_data_config;
          ia_format_converter_scratch *pstr_earcon_scratch;
          pstr_earcon_scratch = handle->p_state_mpeghd->mpeghd_scratch_mem_v;
          impeghd_format_conv_earcon_process(handle, pstr_pcm_data,
                                             &pstr_earcon_scratch->scratch_fc_ercon[0]);
          for (channel = 0; channel < (WORD32)handle->mpeghd_config.ui_n_channels; channel++)
          {
            ia_core_coder_mem_cpy(pstr_pcm_data->pcm_sample[channel],
                                  pstr_dec_data->str_usac_data.time_sample_vector[channel],
                                  pstr_dec_data->str_usac_data.ccfl);
          }
        }
        else
        {
          impeghd_earcon_obj_md_dec_ren_process(pstr_dec_data, ptr_ec_scratch);
          for (channel = 0; channel < (WORD32)handle->mpeghd_config.ui_n_channels; channel++)
          {
            ia_core_coder_mem_cpy(&ptr_ec_scratch[channel * pstr_dec_data->str_usac_data.ccfl],
                                  pstr_dec_data->str_usac_data.time_sample_vector[channel],
                                  pstr_dec_data->str_usac_data.ccfl);
          }
        }
      }

      if (handle->mpeghd_config.ui_binaural_flag)
      {
        for (sig = 0; sig < BINAURAL_TEST_LGR_SIGNAL; sig++)
        {
          for (channel = 0; channel < (*num_channel_out); channel++)
          {
            pstr_dec_data->binaural_handle.ptr_binaural_signal_in_renderer
                [pstr_dec_data->binaural_handle.channel_map[channel]][sig] =
                ia_div_q15_flt((pstr_dec_data->str_usac_data.time_sample_vector[channel][sig]));
          }
        }
        WORD32 num_samples_processed;
        num_samples_processed = impeghd_binaural_struct_process_ld_ola(
            pstr_dec_data->binaural_handle.ptr_binaural_signal_in_renderer,
            pstr_dec_data->binaural_handle.ptr_binaural_signal_out_renderer,
            BINAURAL_TEST_LGR_SIGNAL, pstr_dec_data->binaural_handle.ptr_work,
            &pstr_dec_data->binaural_handle.str_binaural,
            pstr_dec_data->str_binaural_rendering.ptr_scratch);
        if (num_samples_processed > 0)
        {
          for (grp = 0; grp < BINAURAL_NB_OUTPUT; grp++)
          {
            FLOAT32 *ptr_samples =
                pstr_dec_data->binaural_handle.ptr_binaural_signal_out_renderer[grp];
            for (s = 0; s < num_samples_processed; s++)
            {
              ptr_samples[s] = ia_mul_flt(ptr_samples[s], 32768.0f);
            }
            pstr_dec_data->ptr_binaural_output[grp] = ptr_samples;
          }
        }
        *num_channel_out = BINAURAL_NB_OUTPUT;
        num_samples_out = num_samples_processed;
      }
      if (preroll_units <= 0 || !mpegh_dec_handle->p_config->discard_au_preroll)
      {
        FLOAT32 loudness_gain_db = pstr_asc->str_usac_config.uni_drc_cfg.loudness_gain_db;

        err_code = impegh_dec_ln_pl_process(pstr_dec_data, mpegh_dec_handle, loudness_gain_db,
                                            num_channel_out);
      }
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }

      if (handle->mpeghd_config.ui_binaural_flag == 0)
      {
        if (handle->mpeghd_config.resample_output)
        {
          num_samples_out = mpegh_dec_handle->p_config->output_framelength;
          FLOAT32 *ptr_scratch = handle->p_state_mpeghd->mpeghd_scratch_mem_v;
          for (channel = 0; channel < *num_channel_out; channel++)
          {
            pstr_dec_data->ptr_binaural_output[channel] =
                &ptr_scratch[mpegh_dec_handle->p_config->output_framelength * channel];
          }
        }
        else
        {
          for (channel = 0; channel < *num_channel_out; channel++)
          {
            pstr_dec_data->ptr_binaural_output[channel] =
                &pstr_dec_data->str_usac_data.time_sample_vector[channel][0];
          }
        }
      }

      ia_core_coder_samples_sat((WORD8 *)outbuffer + tot_out_bytes, num_samples_out, pcmsize,
                                &pstr_dec_data->ptr_binaural_output[0], out_ch_map,
                                &mpegh_dec_handle->delay_in_samples, out_bytes, *num_channel_out);
      {
        WORD32 payload_buffer_offeset = 0;
        WORD32 copy_bytes =
            pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config
                .str_usac_dec_config.usac_ext_gain_payload_len[preroll_counter] *
            sizeof(WORD8);

        pstr_asc->str_usac_config.str_usac_dec_config.usac_ext_gain_payload_len[preroll_counter] =
            pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config
                .str_usac_dec_config.usac_ext_gain_payload_len[preroll_counter];

        for (cnt = 0; cnt < preroll_counter; cnt++)
        {
          payload_buffer_offeset +=
              pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config
                  .str_usac_dec_config.usac_ext_gain_payload_len[cnt] *
              sizeof(WORD8);
        }

        memcpy(pstr_asc->str_usac_config.str_usac_dec_config.usac_ext_gain_payload_buf[cnt] +
                   payload_buffer_offeset,
               pstr_dec_data->str_frame_data.str_audio_specific_config.str_usac_config
                       .str_usac_dec_config.usac_ext_gain_payload_buf[cnt] +
                   payload_buffer_offeset,
               copy_bytes);

        pstr_asc->str_usac_config.str_usac_dec_config.preroll_bytes[preroll_counter] = *out_bytes;

        preroll_counter++;

        if (preroll_counter > (MAX_AUDIO_PREROLLS + 1))
          return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_PRE_ROLLS;
      }

      access_units++;
      preroll_units--;
      tot_out_bytes += (*out_bytes);
    } while (preroll_units >= 0);
    *out_bytes = tot_out_bytes;
  }
  return err_code;
}

/**
 *  impeghd_uni_drc_dec_asi_init
 *
 *  \brief UniDRC decoder initalization.
 *
 *  \param [in]     pstr_asc                   Pointer to audio specific config structure.
 *  \param [in,out] pstr_dec_data              Pointer to decoder data structure.
 *  \param [in]     preset_id                  Preset id value obtained from command line.
 *  \param [in]     group_id_requested         Group IDs
 *  \param [in]     group_preset_id_requested  Group preset IDs
 *  \param [in]     num_mbrs_grp_pr_ids_req    Number of members in group preset IDs
 *
 *  \return IA_ERRORCODE Error code if any else 0.
 *
 */
IA_ERRORCODE impeghd_uni_drc_dec_asi_init(ia_audio_specific_config_struct *pstr_asc,
                                          ia_dec_data_struct *pstr_dec_data, WORD8 preset_id,
                                          WORD8 *group_id_requested,
                                          WORD8 *group_preset_id_requested,
                                          WORD8 *num_mbrs_grp_pr_ids_req)

{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 grp, cond;
  WORD32 grp_ids = 0;
  WORD32 on_off_arr[MAX_NUM_GROUPS] = {0};
  WORD32 num_group_preset_ids_requested;
  WORD32 num_group_ids_requested;

  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_drc_payload_struct *pstr_drc_payload = &pstr_dec_data->str_drc_payload;

  num_group_preset_ids_requested = pstr_mae_asi->num_group_presets;
  num_group_ids_requested = pstr_mae_asi->num_groups;

  err_code = impd_drc_mpegh_params_set_sel_process(
      &pstr_drc_payload->str_select_proc, num_group_ids_requested, group_id_requested,
      num_group_preset_ids_requested, group_preset_id_requested, num_mbrs_grp_pr_ids_req, 0);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }

  if (pstr_mae_asi->ei_present || pstr_dec_data->str_element_interaction.ei_data_present)
  {
    preset_id = pstr_dec_data->str_element_interaction.ei_group_preset_id;
  }

  if (preset_id != -1)
  {
    /*Ittiam: Assign the preset id from command line to appropriate fields*/
    WORD32 pr_grp_idx = -1;
    WORD32 grp_idx = -1;

    if (pstr_mae_asi->num_groups > MAX_NUM_GROUPS_PRESETS)
    {
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ASI_NUM_GROUPS_PRESETS;
    }

    for (grp = 0; grp < pstr_mae_asi->num_groups; grp++)
    {
      if (pstr_mae_asi->group_presets_definition[grp].group_id == preset_id)
      {
        pr_grp_idx = grp;
        break;
      }
    }
    if (pr_grp_idx == -1)
    {
      /* Invalid Preset Id */
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_PRESET_ID;
    }
    for (grp = 0; grp < pstr_mae_asi->num_groups; grp++)
    {
      WORD32 grp_id = pstr_mae_asi->group_definition[grp].group_id;
      for (cond = 0; cond < pstr_mae_asi->group_presets_definition[pr_grp_idx].num_conditions;
           cond++)
      {
        if (pstr_mae_asi->group_presets_definition[pr_grp_idx].reference_id[cond] == grp_id)
        {
          grp_idx = cond;
          break;
        }
      }
      if (grp_idx == -1)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_GRP_ID;
      }
      on_off_arr[grp] = pstr_mae_asi->group_presets_definition[pr_grp_idx].cond_on_off[grp_idx];
    }
  }
  else
  {
    if (pstr_mae_asi->num_group_presets <= 0)
    {
      preset_id = -1;
    }
    else
    {
      /* choose ID 0 or minimim preset ID if not present */
      WORD8 min_preset_id = MAX_NUM_GROUPS_PRESETS;
      for (grp = 0; grp < pstr_mae_asi->num_group_presets; grp++)
      {
        if (pstr_mae_asi->group_presets_definition[grp].group_id < min_preset_id)
        {
          min_preset_id = pstr_mae_asi->group_presets_definition[grp].group_id;
          if (min_preset_id == 0)
            break;
        }
      }
      preset_id = min_preset_id;
    }
    for (grp = 0; grp < pstr_mae_asi->num_groups; grp++)
    {
      on_off_arr[grp] = pstr_mae_asi->group_definition[grp].default_on_off;
    }
  }

  for (grp = 0; grp < pstr_mae_asi->num_groups; grp++)
  {
    if (on_off_arr[grp] == 1)
    {
      pstr_drc_payload->str_uni_drc_sel_proc_params.group_id_requested[grp_ids] =
          pstr_mae_asi->group_definition[grp].group_id;
      grp_ids++;
    }
  }
  pstr_drc_payload->str_uni_drc_sel_proc_params.num_group_ids_requested = (WORD8)grp_ids;
  if (preset_id == -1)
  {
    pstr_drc_payload->str_uni_drc_sel_proc_params.num_group_preset_ids_requested = 0;
  }
  else
  {
    pstr_drc_payload->str_uni_drc_sel_proc_params.num_group_preset_ids_requested = 1;
    pstr_drc_payload->str_uni_drc_sel_proc_params.group_preset_id_requested[0] = preset_id;
  }

  return err_code;
}
/**
 *  impeghd_uni_drc_dec_init
 *
 *  \brief UniDRC decoder initalization.
 *
 *  \param [in]     pstr_audio_specific_config Pointer to audio specific config structure.
 *  \param [in,out] pstr_dec_data              Pointer to decoder data structure.
 *  \param [in]     tgt_loudness               Target loudness value.
 *  \param [in]     loud_norm_flag             Flag indicating presence of command line loudness
 *                                          normalization info.
 *  \param [in]     drc_effect_type            DRC effect type.
 *  \param [in]     preset_id                  Preset id value obtained from command line.
 *  \param [in]     ui_cicp_layout_idx         CICP loudspeaker layout index.
 *  \param [in]     index                      Extension element index.
 *
 *  \return IA_ERRORCODE Error code if any else 0.
 *
 */
IA_ERRORCODE impeghd_uni_drc_dec_init(ia_audio_specific_config_struct *pstr_audio_specific_config,
                                      ia_dec_data_struct *pstr_dec_data, WORD32 tgt_loudness,
                                      WORD32 loud_norm_flag, WORD32 drc_effect_type,
                                      WORD8 preset_id, WORD32 ui_cicp_layout_idx, WORD32 index)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 drc_effect_type_request;
  WORD32 control_parameter_index = -1;
  WORD32 frame_size = 1024;
  WORD32 sampling_rate;
  WORD32 gain_delay_samples = 0;
  WORD32 delay_mode = 0;
  WORD32 sub_band_domain_mode = 0;
  WORD32 grp, inst, drc;
  WORD32 num_ch_in;
  WORD32 channel_offset = 0;
  WORD32 num_channels_process;
  WORD8 num_group_ids_requested;
  WORD8 num_group_preset_ids_requested;
  WORD32 num_ch;

  WORD8 group_id_requested[MAX_NUM_GROUPS];
  WORD8 group_preset_id_requested[MAX_NUM_GROUPS_PRESETS];
  WORD8 num_members_group_preset_ids_requested[MAX_NUM_GROUPS];
  WORD32 decDownmixIdList[NUM_GAIN_DEC_INSTANCES] = {0, 4};

  pVOID persistant_ptr = (VOID *)&pstr_dec_data->drc_persistent_buf[0];

  ia_usac_config_struct *pstr_usac_cfg = &pstr_audio_specific_config->str_usac_config;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_audio_specific_config->str_mae_asi;
  ia_usac_decoder_config_struct *pstr_usac_dec_cfg = &pstr_usac_cfg->str_usac_dec_config;
  ia_drc_payload_struct *pstr_drc_payload = &pstr_dec_data->str_drc_payload;
  ia_bit_buf_struct it_bit_buff;

  num_group_preset_ids_requested = pstr_mae_asi->num_group_presets;
  num_group_ids_requested = pstr_mae_asi->num_groups;
  num_channels_process = num_ch_in = num_ch =
      (pstr_usac_cfg->signals_3d.num_audio_obj + pstr_usac_cfg->signals_3d.num_ch +
       pstr_usac_cfg->signals_3d.num_hoa_transport_ch);
  sampling_rate = pstr_usac_cfg->usac_sampling_frequency;

  switch (drc_effect_type)
  {
  case 0:
    drc_effect_type_request = 0x432160;
    break;
  case 1:
    drc_effect_type_request = 0x543261;
    break;
  case 2:
    drc_effect_type_request = 0x543162;
    break;
  case 3:
    drc_effect_type_request = 0x542163;
    break;
  case 4:
    drc_effect_type_request = 0x531264;
    break;
  case 5:
    drc_effect_type_request = 0x432165;
    break;
  case 6:
    drc_effect_type_request = 0x543216;
    break;
  default:
    drc_effect_type_request = 0x543216;
    break;
  }

  for (grp = 0; grp < num_group_ids_requested; grp++)
  {
    num_members_group_preset_ids_requested[grp] =
        pstr_mae_asi->group_definition[grp].group_num_members;
    group_id_requested[grp] = pstr_mae_asi->group_definition[grp].group_id;
  }
  for (grp = 0; grp < num_group_preset_ids_requested; grp++)
  {
    group_preset_id_requested[grp] = pstr_mae_asi->group_presets_definition[grp].group_id;
  }

  for (inst = 0; inst < NUM_GAIN_DEC_INSTANCES; inst++)
  {
    sub_band_domain_mode =
        pstr_drc_payload->str_gain_dec[inst].ia_drc_params_struct.sub_band_domain_mode;
    gain_delay_samples =
        pstr_drc_payload->str_gain_dec[inst].ia_drc_params_struct.gain_delay_samples;
    err_code =
        impd_drc_init_decode(&pstr_drc_payload->str_gain_dec[inst], frame_size, sampling_rate,
                             gain_delay_samples, delay_mode, sub_band_domain_mode);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  memset(&pstr_drc_payload->str_drc_interface, 0, sizeof(pstr_drc_payload->str_drc_interface));

  if (control_parameter_index <= 0)
  {
    err_code =
        impd_drc_set_def_params_sel_process(&pstr_drc_payload->str_uni_drc_sel_proc_params);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }
  else
  {
    err_code =
        impd_drc_set_def_params_sel_process(&pstr_drc_payload->str_uni_drc_sel_proc_params);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }

    err_code = impd_drc_set_custom_params(control_parameter_index,
                                          &pstr_drc_payload->str_uni_drc_sel_proc_params);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  if (pstr_mae_asi->asi_present)
  {
    err_code = impeghd_uni_drc_dec_asi_init(
        pstr_audio_specific_config, pstr_dec_data, preset_id, (WORD8 *)&group_id_requested,
        (WORD8 *)&group_preset_id_requested, &num_members_group_preset_ids_requested[0]);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  if (drc_effect_type != -1)
  {
    pstr_drc_payload->str_uni_drc_sel_proc_params.dynamic_range_control_on = 1;
    pstr_drc_payload->str_uni_drc_sel_proc_params.num_drc_feature_requests = 1;
    pstr_drc_payload->str_uni_drc_sel_proc_params.drc_feature_req_type[0] = MATCH_EFFECT_TYPE;
    pstr_drc_payload->str_uni_drc_sel_proc_params.requested_num_drc_effects[0] = 0;
    pstr_drc_payload->str_uni_drc_sel_proc_params.desired_num_drc_effects_of_requested[0] = 1;

    impd_drc_update_effect_type_request(
        &pstr_drc_payload->str_uni_drc_sel_proc_params.requested_num_drc_effects[0],
        &pstr_drc_payload->str_uni_drc_sel_proc_params.requested_drc_effect_type[0][0],
        drc_effect_type_request);
  }

  pstr_drc_payload->str_select_proc.first_frame = 1;
  if (loud_norm_flag)
  {
    pstr_drc_payload->str_uni_drc_sel_proc_params.loudness_normalization_on = 1;

    pstr_drc_payload->str_uni_drc_sel_proc_params.target_loudness = (FLOAT32)tgt_loudness;
  }

  err_code = impd_drc_uni_selction_proc_init(
      &pstr_drc_payload->str_select_proc, &pstr_drc_payload->str_uni_drc_sel_proc_params,
      &pstr_drc_payload->str_drc_interface, sub_band_domain_mode);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }

  if (ui_cicp_layout_idx > 0)
  {
    pstr_drc_payload->str_select_proc.str_uni_drc_sel_proc_params.requested_dwnmix_id[0] =
        (WORD8)ui_cicp_layout_idx;
    pstr_drc_payload->str_select_proc.str_uni_drc_sel_proc_params.num_downmix_id_requests = 1;
    pstr_drc_payload->str_select_proc.str_uni_drc_sel_proc_params.target_config_request_type = 0;
  }

  pstr_drc_payload->str_loud_info.loudness_info_album_count = 0;
  pstr_drc_payload->str_loud_info.loudness_info_count = 0;
  pstr_drc_payload->str_loud_info.loudness_info_set_ext_present = 0;

  memset(pstr_drc_payload->pstr_drc_config, 0, sizeof(ia_drc_config));

  err_code = impd_drc_init_bitstream_config(pstr_drc_payload->pstr_drc_config);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }

  pstr_drc_payload->pstr_drc_config->channel_layout.base_channel_count = num_ch_in;

  err_code = impd_drc_init_bitstream_dec(&pstr_drc_payload->str_bitstream_dec, sampling_rate,
                                         frame_size, delay_mode, -1, 0);

  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }

  if (pstr_usac_cfg->str_usac_dec_config.downmix_ext_config_present)
  {
    WORD32 id;
    pstr_usac_cfg->uni_drc_cfg.ia_mpegh3da_dwnmix_instructions_count =
        pstr_usac_dec_cfg->dmx_cfg.downmix_id_count;
    for (id = 0; id < (WORD32)pstr_usac_dec_cfg->dmx_cfg.downmix_id_count; id++)
    {
      pstr_usac_cfg->uni_drc_cfg.mpegh3da_dwnmix_instructions[id].downmix_coefficients_present =
          0;
      pstr_usac_cfg->uni_drc_cfg.mpegh3da_dwnmix_instructions[id].downmix_id =
          pstr_usac_dec_cfg->dmx_cfg.dmx_matrix[id].dmx_id;
      pstr_usac_cfg->uni_drc_cfg.mpegh3da_dwnmix_instructions[id].target_layout =
          pstr_usac_dec_cfg->dmx_cfg.dmx_matrix[id].cicp_spk_layout_idx;
      pstr_usac_cfg->uni_drc_cfg.mpegh3da_dwnmix_instructions[id].target_channel_count =
          impgehd_cicp_get_num_ls[pstr_usac_dec_cfg->dmx_cfg.dmx_matrix[id].cicp_spk_layout_idx];
    }
  }

  if ((pstr_usac_dec_cfg->usac_ext_ele_payload_len[index] == 0) && loud_norm_flag)
  {
    UWORD8 bitbuf[4] = {0};
    ia_core_coder_create_init_bit_buf(&it_bit_buff, bitbuf, 4);
    it_bit_buff.xmpeghd_jmp_buf = pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf;
    pstr_usac_cfg->uni_drc_bs_params =
        pstr_dec_data->str_drc_payload.str_bitstream_dec.ia_drc_params_struct;
    err_code = impd_drc_parse_config(pstr_dec_data->str_drc_payload.pstr_drc_config,
                                     &pstr_usac_cfg->str_loudness_info, &it_bit_buff,
                                     &pstr_usac_cfg->uni_drc_bs_params, pstr_mae_asi);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }
  if (pstr_usac_dec_cfg->usac_ext_ele_payload_len[index] > 0)
  {
    ia_core_coder_create_init_bit_buf(&it_bit_buff,
                                      &pstr_usac_dec_cfg->usac_ext_ele_payload_buf[index][0],
                                      pstr_usac_dec_cfg->usac_ext_ele_payload_len[index]);
    it_bit_buff.xmpeghd_jmp_buf = pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf;
    pstr_usac_cfg->uni_drc_bs_params =
        pstr_dec_data->str_drc_payload.str_bitstream_dec.ia_drc_params_struct;
    err_code = impd_drc_parse_config(pstr_dec_data->str_drc_payload.pstr_drc_config,
                                     &pstr_usac_cfg->str_loudness_info, &it_bit_buff,
                                     &pstr_usac_cfg->uni_drc_bs_params, pstr_mae_asi);

    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  if (pstr_usac_dec_cfg->usac_cfg_ext_info_len[pstr_usac_dec_cfg->loudness_ext_config_idx] > 0)
  {
    ia_core_coder_create_init_bit_buf(
        &it_bit_buff,
        &pstr_usac_dec_cfg->usac_cfg_ext_info_buf[pstr_usac_dec_cfg->loudness_ext_config_idx][0],
        pstr_usac_dec_cfg->usac_cfg_ext_info_len[pstr_usac_dec_cfg->loudness_ext_config_idx]);
    it_bit_buff.xmpeghd_jmp_buf = pstr_dec_data->dec_bit_buf.xmpeghd_jmp_buf;

    err_code = impd_drc_mpegh3da_parse_loudness_info_set(&pstr_drc_payload->str_loud_info,
                                                         &it_bit_buff, pstr_mae_asi);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  if ((pstr_usac_dec_cfg->usac_ext_ele_payload_len[index] > 0) ||
      (pstr_usac_dec_cfg->usac_cfg_ext_info_len[pstr_usac_dec_cfg->loudness_ext_config_idx] > 0))
  {
    err_code = impd_drc_uni_sel_proc_process(
        &pstr_drc_payload->str_select_proc, pstr_drc_payload->pstr_drc_config,
        &pstr_drc_payload->str_loud_info,
        &pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }

    if (pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.base_channel_count != num_ch_in)
    {
      return IA_MPEGH_DEC_NO_ERROR;
    }
  }

  {
    WORD32 audio_num_chan = 0;
    WORD32 num_matching_drc_sets = 0;
    WORD32 matching_drc_set_ids[3], matching_dmx_ids[3];
    WORD32 num_sel_drc_sets =
        pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.num_sel_drc_sets;

    for (drc = 0; drc < num_sel_drc_sets; drc++)
    {
      if (impd_match_downmix(
              pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.sel_downmix_ids[drc],
              decDownmixIdList[0]))
      {
        matching_drc_set_ids[num_matching_drc_sets] =
            pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.sel_drc_set_ids[drc];
        matching_dmx_ids[num_matching_drc_sets] =
            pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.sel_downmix_ids[drc];
        num_matching_drc_sets++;
      }
    }
    audio_num_chan = num_ch_in;

    if (num_sel_drc_sets > 0)
    {
      err_code = impd_drc_init_decode_post_config(
          &pstr_drc_payload->str_gain_dec[0], audio_num_chan, matching_drc_set_ids,
          matching_dmx_ids, num_matching_drc_sets, channel_offset, num_channels_process,
          pstr_drc_payload->pstr_drc_config, &persistant_ptr);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
        return err_code;
      if (pstr_drc_payload->pstr_drc_config->multi_band_present)
        pstr_usac_cfg->signals_3d.domain_switcher_enable = 1;
    }
  }

  // get information about multiband DRC presence
  {
    ia_drc_config *ptr_drc_config = pstr_drc_payload->pstr_drc_config;
    for (drc = 0; drc < ptr_drc_config->drc_instructions_uni_drc_count; drc++)
    {
      if ((ptr_drc_config->str_drc_instruction_str[drc].downmix_id[0] == ID_FOR_BASE_LAYOUT) ||
          (ptr_drc_config->str_drc_instruction_str[drc].drc_apply_to_dwnmix == 0))
      {
        pstr_drc_payload->num_sets[0]++;
      }
      else if (ptr_drc_config->str_drc_instruction_str[drc].downmix_id[0] == ID_FOR_ANY_DOWNMIX)
      {
        pstr_drc_payload->num_sets[1]++;
      }
      else
      {
        pstr_drc_payload->num_sets[2]++;
      }
      for (grp = 0; grp < ptr_drc_config->str_drc_instruction_str[drc].num_drc_ch_groups; grp++)
      {
        WORD32 idx =
            ptr_drc_config->str_drc_instruction_str[drc].gain_set_index_for_channel_group[grp];
        if (ptr_drc_config->str_p_loc_drc_coefficients_uni_drc[0]
                .gain_set_params[idx]
                .band_count > 1)
        {
          if ((ptr_drc_config->str_drc_instruction_str[drc].downmix_id[0] ==
               ID_FOR_BASE_LAYOUT) ||
              (ptr_drc_config->str_drc_instruction_str[drc].drc_apply_to_dwnmix == 0))
          {
            pstr_drc_payload->multiband_drc_present[0] = 1;
          }
          else if (ptr_drc_config->str_drc_instruction_str[drc].downmix_id[0] ==
                   ID_FOR_ANY_DOWNMIX)
          {
            pstr_drc_payload->multiband_drc_present[1] = 1;
          }
          else
          {
            pstr_drc_payload->multiband_drc_present[2] = 1;
          }
        }
      }
    }
  }

  if (pstr_drc_payload->multiband_drc_present[0] > 0 ||
      pstr_drc_payload->multiband_drc_present[1] > 0 ||
      pstr_drc_payload->multiband_drc_present[2] > 0)
  {
    pstr_usac_cfg->signals_3d.domain_switcher_enable = 1;
    pstr_drc_payload->pstr_drc_config->multi_band_present = 1;
    pstr_drc_payload->str_gain_dec[0].ia_drc_params_struct.sub_band_domain_mode =
        SUBBAND_DOMAIN_MODE_STFT256;
  }

  if (pstr_drc_payload->num_sets[1] > 0 || pstr_drc_payload->num_sets[2] > 0)
  {
    WORD32 audio_num_chan = 0;
    WORD32 num_matching_drc_sets = 0;
    WORD32 matching_drc_set_ids[3] = {0}, matching_dmx_ids[3] = {0};
    WORD32 num_sel_drc_sets =
        pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.num_sel_drc_sets;
    WORD32 dec_downmix_id = 2;
    if (pstr_drc_payload->num_sets[2] > 0)
    {
      dec_downmix_id = 3;
    }

    for (drc = 0; drc < num_sel_drc_sets; drc++)
    {
      if (impd_match_downmix(
              pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.sel_downmix_ids[drc],
              dec_downmix_id))
      {
        matching_drc_set_ids[num_matching_drc_sets] =
            pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.sel_drc_set_ids[drc];
        matching_dmx_ids[num_matching_drc_sets] =
            pstr_drc_payload->str_select_proc.uni_drc_sel_proc_output.sel_downmix_ids[drc];
        num_matching_drc_sets++;
      }
    }
    audio_num_chan = num_ch_in;
    if (ui_cicp_layout_idx != IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL)
    {
      audio_num_chan = impgehd_cicp_get_num_ls[ui_cicp_layout_idx];
    }
    if ((pstr_usac_cfg->signals_3d.num_audio_obj > 0) ||
        (pstr_usac_cfg->signals_3d.num_hoa_transport_ch > 0))
    {
      audio_num_chan =
          impgehd_cicp_get_num_ls[pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx];
    }

    if (num_sel_drc_sets > 0)
    {
      err_code = impd_drc_init_decode_post_config(
          &pstr_drc_payload->str_gain_dec[1], audio_num_chan, matching_drc_set_ids,
          matching_dmx_ids, num_matching_drc_sets, channel_offset, audio_num_chan,
          pstr_drc_payload->pstr_drc_config, &persistant_ptr);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }
      if (pstr_drc_payload->pstr_drc_config->multi_band_present)
      {
        pstr_usac_cfg->signals_3d.domain_switcher_enable = 1;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecProc */
