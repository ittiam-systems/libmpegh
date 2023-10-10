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
#include "impeghd_api.h"
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_dec.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_api_defs.h"
#include "ia_core_coder_apicmd_standards.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"

#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_constants.h"
#include "ia_core_coder_definitions.h"
#include "ia_core_coder_env_extr.h"
#include "ia_core_coder_function_selector.h"
#include "impeghd_memory_standards.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_ver_number.h"

#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "ia_core_coder_config.h"
#include "impeghd_config_params.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_mae_mp4_intrfc.h"
#include "impeghd_error_codes.h"
#include "impeghd_fft_ifft_rom.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_format_conv_data.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_nfc_filtering.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_hoa_renderer.h"
#include "impeghd_hoa_decoder.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_hoa_spatial_decoder.h"
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_uni_drc_struct.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_peak_limiter_struct_def.h"
#include "impeghd_resampler.h"
#include "impeghd_cicp_2_geometry.h"
#include "ia_core_coder_struct.h"
#include "ia_core_coder_struct_def.h"
#include "ia_core_coder_create.h"
#include "ia_core_coder_headerdecode.h"

/**
 * @defgroup MPEGHDecAPIs MPEG-H 3D Audio Low Complexity Profile Decoder APIs
 * @ingroup  MPEGHDecAPIs
 * @brief MPEG-H 3D Audio Low Complexity Profile Decoder APIs
 *
 * @{
 */

/**
 *  impeghd_resampler_get_sampling_ratio
 *
 *  \brief Map the sample rate converter up,down ratios based on configuration
 *
 *  \param [in,out] ptr_config Pointer to decoder configuration structure
 *
 *  \return IA_ERRORCODE       Processing error if any
 *
 */
IA_ERRORCODE impeghd_resampler_get_sampling_ratio(ia_mpegh_dec_config_struct *ptr_config)
{
  ptr_config->output_framelength = AUDIO_CODEC_FRAME_SIZE_MAX;
  ptr_config->resample_output = 1;
  switch (ptr_config->stream_samp_freq)
  {
  case 48000:
    if (ptr_config->out_samp_freq != 48000)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 1;
    ptr_config->fac_up = 1;
    ptr_config->resample_output = 0;
    break;
  case 44100:
    if (ptr_config->out_samp_freq != 44100)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 1;
    ptr_config->fac_up = 1;
    ptr_config->resample_output = 0;
    break;
  case 32000:
    if (ptr_config->out_samp_freq != 48000)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 2;
    ptr_config->fac_up = 3;
    break;
  case 29400:
    if (ptr_config->out_samp_freq != 44100)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 2;
    ptr_config->fac_up = 3;
    break;
  case 24000:
    if (ptr_config->out_samp_freq != 48000)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 1;
    ptr_config->fac_up = 2;
    break;
  case 22050:
    if (ptr_config->out_samp_freq != 44100)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 1;
    ptr_config->fac_up = 2;
    break;
  case 16000:
    if (ptr_config->out_samp_freq != 48000)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 1;
    ptr_config->fac_up = 3;
    break;
  case 14700:
    if (ptr_config->out_samp_freq != 44100)
    {
      return IA_MPEGH_DEC_INIT_NONFATAL_INVALID_RESAMPLE_RATIO;
    }
    ptr_config->fac_down = 1;
    ptr_config->fac_up = 3;
    break;
  }
  ptr_config->output_framelength =
      (AUDIO_CODEC_FRAME_SIZE_MAX * ptr_config->fac_up) / ptr_config->fac_down;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_set_default_config
 *
 *  \brief initialize configuration structure with default values
 *
 *  \param [out] p_obj_mpegh_dec Pointer to decoder handle structure
 *
 *
 *
 */
static VOID impeghd_set_default_config(ia_mpegh_dec_api_struct *p_obj_mpegh_dec)
{
  p_obj_mpegh_dec->mpeghd_config.ui_pcm_wdsz = IMPEGHD_CONFIG_PARAM_PCM_WD_SZ_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.ui_effect_type = IMPEGHD_CONFIG_PARAM_EFFECT_TYPE_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.ui_target_loudness = IMPEGHD_CONFIG_PARAM_TGT_LOUDNESS_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.drc_apply = 0;
  p_obj_mpegh_dec->mpeghd_config.ui_loud_norm_flag = IMPEGHD_CONFIG_PARAM_LOUD_NORM_FLG_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx = IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.i_preset_id = IMPEGHD_CONFIG_PARAM_PRESET_ID_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.resample_output = 0;
  p_obj_mpegh_dec->mpeghd_config.output_framelength = AUDIO_CODEC_FRAME_SIZE_MAX;
  p_obj_mpegh_dec->mpeghd_config.fac_down = 1;
  p_obj_mpegh_dec->mpeghd_config.fac_up = 1;
  p_obj_mpegh_dec->mpeghd_config.out_samp_freq = IMPEGHD_CONFIG_PARAM_OUT_FS_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.stream_samp_freq = IMPEGHD_CONFIG_PARAM_OUT_FS_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.ui_max_channels = IMPEGHD_CONFIG_PARAM_MAX_CHANS_DFLT_VAL;
  p_obj_mpegh_dec->mpeghd_config.ui_n_channels = 2;
  p_obj_mpegh_dec->mpeghd_config.i_channel_mask = 3;
}

/**
 *  impeghd_set_config_params
 *
 *  \brief Initialize parameters with default values
 *
 *  \param [out] p_obj_mpegh_dec Pointer to decoder handle structure
 *  \param [in]  ptr_input_config Pointer to default input config structure
 *
 *  \return IA_ERRORCODE          Error
 *
 */
static IA_ERRORCODE impeghd_set_config_params(ia_mpegh_dec_api_struct *p_obj_mpegh_dec,
                                              ia_input_config *ptr_input_config)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  if (ptr_input_config->ui_mhas_flag == IMPEGHD_CONFIG_PARAM_MHAS_FLAG_DFLT_VAL)
  {
    p_obj_mpegh_dec->mpeghd_config.ui_mhas_flag = ptr_input_config->ui_mhas_flag;
  }

  if ((ptr_input_config->ui_cicp_layout_idx != IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL) &&
      (ptr_input_config->ui_cicp_layout_idx < NUM_LS_CFGS))
  {
    p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx = ptr_input_config->ui_cicp_layout_idx;
  }
  else if (ptr_input_config->ui_cicp_layout_idx != IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL)
  {
    return IA_MPEGH_DEC_CONFIG_NONFATAL_INVALID_CICP_INDEX;
  }

  if ((ptr_input_config->i_preset_id != IMPEGHD_CONFIG_PARAM_PRESET_ID_DFLT_VAL) &&
      (ptr_input_config->i_preset_id < IMPEGHD_CONFIG_PARAM_PRESET_ID_MAX_VAL))
  {
    p_obj_mpegh_dec->mpeghd_config.i_preset_id = ptr_input_config->i_preset_id;
  }
  else if (ptr_input_config->i_preset_id != IMPEGHD_CONFIG_PARAM_PRESET_ID_DFLT_VAL)
  {
    return IA_MPEGH_DEC_CONFIG_NONFATAL_INVALID_PRESET_ID;
  }

  if ((ptr_input_config->ui_pcm_wd_sz != 16) && (ptr_input_config->ui_pcm_wd_sz != 24) &&
      (ptr_input_config->ui_pcm_wd_sz != 32))
  {
    p_obj_mpegh_dec->mpeghd_config.ui_pcm_wdsz = 16;
    return (IA_MPEGH_DEC_CONFIG_NONFATAL_INVALID_PCM_WDSZ);
  }
  p_obj_mpegh_dec->mpeghd_config.ui_pcm_wdsz = ptr_input_config->ui_pcm_wd_sz;

  if (ptr_input_config->enable_resamp)
  {
    p_obj_mpegh_dec->mpeghd_config.resample_output = 1;
    p_obj_mpegh_dec->mpeghd_config.out_samp_freq = ptr_input_config->out_samp_freq;
  }
  if (ptr_input_config->ui_effect != IMPEGHD_CONFIG_PARAM_EFFECT_TYPE_DFLT_VAL)
  {
    if ((ptr_input_config->ui_effect > 8) || (ptr_input_config->ui_effect < 0))
    {
      p_obj_mpegh_dec->mpeghd_config.ui_effect_type = 0;
      p_obj_mpegh_dec->mpeghd_config.drc_apply = 1;
      return (IA_MPEGH_DEC_CONFIG_NONFATAL_INVALID_EFFECT_TYPE);
    }
    p_obj_mpegh_dec->mpeghd_config.ui_effect_type = ptr_input_config->ui_effect;
  }

  if (ptr_input_config->ui_target_loudness[0] == 1)
  {
    if (ptr_input_config->ui_target_loudness[1] >= 0)
    {
      p_obj_mpegh_dec->mpeghd_config.ui_loud_norm_flag = 1;
      p_obj_mpegh_dec->mpeghd_config.drc_apply = 1;
    }
    ptr_input_config->ui_target_loudness[1] = -(ptr_input_config->ui_target_loudness[1] >> 2);
    if (((ptr_input_config->ui_target_loudness[1]) > 0) ||
        ((ptr_input_config->ui_target_loudness[1]) < -63))
    {
      p_obj_mpegh_dec->mpeghd_config.ui_target_loudness = 0;
      p_obj_mpegh_dec->mpeghd_config.drc_apply = 0;
      return (IA_MPEGH_DEC_CONFIG_NONFATAL_INVALID_TARGET_LOUDNESS);
    }
    p_obj_mpegh_dec->mpeghd_config.ui_target_loudness = ptr_input_config->ui_target_loudness[1];
    p_obj_mpegh_dec->mpeghd_config.ui_target_loudness_set = 1;
  }
  return err_code;
}

/**
 *  impeghd_alloc_and_assign_mem
 *
 *  \brief Allocate memory and assign the pointers with allocated mem
 *
 *  \param [in,out] p_obj_mpegh_dec Pointer to decoder handle structure
 *  \param [in,out] ptr_out_cfg      Pointer to input-output config structure
 *
 *  \return IA_ERRORCODE             Processing error if any
 *
 */
static IA_ERRORCODE impeghd_alloc_and_assign_mem(ia_mpegh_dec_api_struct *p_obj_mpegh_dec,
                                                 ia_output_config *ptr_out_cfg)
{
  UWORD32 i_idx;
  pVOID pv_value;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  for (i_idx = 0; i_idx < NUM_MEMTABS; i_idx++)
  {
    UWORD32 *meminfo = (UWORD32 *)(p_obj_mpegh_dec->p_mem_info_mpeghd + i_idx);

    ptr_out_cfg->mem_info_table[i_idx].ui_size =
        *(meminfo + (IA_API_CMD_GET_MEM_INFO_SIZE - IA_API_CMD_GET_MEM_INFO_SIZE));
    ptr_out_cfg->mem_info_table[i_idx].ui_alignment =
        *(meminfo + (IA_API_CMD_GET_MEM_INFO_ALIGNMENT - IA_API_CMD_GET_MEM_INFO_SIZE));
    ptr_out_cfg->mem_info_table[i_idx].ui_type =
        *(meminfo + (IA_API_CMD_GET_MEM_INFO_TYPE - IA_API_CMD_GET_MEM_INFO_SIZE));

    ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count] =
        ptr_out_cfg->malloc_mpegh(ptr_out_cfg->mem_info_table[i_idx].ui_size,
                                  ptr_out_cfg->mem_info_table[i_idx].ui_alignment);

    if (NULL == ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count])
    {
      return IA_MPEGH_DEC_API_FATAL_MEM_ALLOC;
    }
    memset(ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count], 0,
           ptr_out_cfg->mem_info_table[i_idx].ui_size);

    ptr_out_cfg->ui_rem =
        (UWORD32)((SIZE_T)ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count] %
                  ptr_out_cfg->mem_info_table[i_idx].ui_alignment);

    pv_value = ptr_out_cfg->mem_info_table[i_idx].mem_ptr =
        (pVOID)((WORD8 *)ptr_out_cfg->arr_alloc_memory[ptr_out_cfg->malloc_count] +
                ptr_out_cfg->mem_info_table[i_idx].ui_alignment - ptr_out_cfg->ui_rem);

    p_obj_mpegh_dec->pp_mem_mpeghd[i_idx] = ptr_out_cfg->mem_info_table[i_idx].mem_ptr;
    memset(p_obj_mpegh_dec->pp_mem_mpeghd[i_idx], 0,
           p_obj_mpegh_dec->p_mem_info_mpeghd[i_idx].ui_size);

    if (i_idx == PERSIST_IDX)
    {
      pUWORD8 p_temp = pv_value;
      UWORD32 *ptr_meminfo = (UWORD32 *)p_obj_mpegh_dec->p_mem_info_mpeghd + i_idx;
      UWORD32 pers_size = ptr_meminfo[0];
      p_temp = p_temp + pers_size - (sizeof(ia_dec_data_struct) + IN_BUF_SIZE);
      p_obj_mpegh_dec->p_state_mpeghd = pv_value;

      p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data = p_temp;
      p_obj_mpegh_dec->p_state_mpeghd->ia_audio_specific_config =
          &(((ia_dec_data_struct *)(p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data))
                ->str_frame_data.str_audio_specific_config);
    }
    if (i_idx == INPUT_IDX)
    {
      ptr_out_cfg->ui_inp_buf_size = ptr_out_cfg->mem_info_table[i_idx].ui_size;
    }
    ptr_out_cfg->malloc_count++;
  }
  return err_code;
}

/**
 *  ia_core_coder_fill_mpeghd_mem_tables
 *
 *  \brief Fills the mem tables with size, alignment and type info
 *
 *  \param [in,out] p_obj_mpegh_dec Pointer to decoder handle structure
 *
 *
 *
 */
static VOID ia_core_coder_fill_mpeghd_mem_tables(ia_mpegh_dec_api_struct *p_obj_mpegh_dec)
{
  ia_mem_info_struct *p_mem_info_mpeghd;

  {
    p_mem_info_mpeghd = &p_obj_mpegh_dec->p_mem_info_mpeghd[PERSIST_IDX];
    p_mem_info_mpeghd->ui_size = sizeof(ia_mpegh_dec_state_struct);

    p_mem_info_mpeghd->ui_size += sizeof(ia_dec_data_struct);
    p_mem_info_mpeghd->ui_size += IN_BUF_SIZE;

    p_mem_info_mpeghd->ui_alignment = 8;
    p_mem_info_mpeghd->ui_type = MEMTYPE_PERSIST;
  }

  {
    p_mem_info_mpeghd = &p_obj_mpegh_dec->p_mem_info_mpeghd[SCRATCH_IDX];
    p_mem_info_mpeghd->ui_size = ((SAMPLES_PER_FRAME * MAX_NUM_CHANNELS) * sizeof(WORD32) * 40);
    p_mem_info_mpeghd->ui_size = ia_max_int(p_mem_info_mpeghd->ui_size, 34816 * 4);
    p_mem_info_mpeghd->ui_alignment = 8;
    p_mem_info_mpeghd->ui_type = MEMTYPE_SCRATCH;
  }
  {
    p_mem_info_mpeghd = &p_obj_mpegh_dec->p_mem_info_mpeghd[INPUT_IDX];

    p_mem_info_mpeghd->ui_size = IN_BUF_SIZE;

    p_mem_info_mpeghd->ui_alignment = 8;
    p_mem_info_mpeghd->ui_type = IA_MEMTYPE_INPUT;
  }
  {
    p_mem_info_mpeghd = &p_obj_mpegh_dec->p_mem_info_mpeghd[OUTPUT_IDX];
    p_mem_info_mpeghd->ui_size = OUT_BUF_SIZE;
    p_mem_info_mpeghd->ui_alignment = 8;
    p_mem_info_mpeghd->ui_type = IA_MEMTYPE_OUTPUT;
  }
  return;
}

/**
 *  impeghd_update_output_config
 *
 *  \brief Update output config structure
 *
 *  \param [in]  p_obj_mpegh_dec   Pointer to decoder handle structure
 *  \param [out] pstr_output_config Pointer to input config structure
 *
 *
 *
 */
static VOID impeghd_update_output_config(ia_mpegh_dec_api_struct *p_obj_mpegh_dec,
                                         ia_output_config *pstr_output_config)
{
  pstr_output_config->i_channel_mask = p_obj_mpegh_dec->mpeghd_config.i_channel_mask;
  pstr_output_config->i_samp_freq = (WORD32)p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
  pstr_output_config->i_num_chan = (WORD32)p_obj_mpegh_dec->mpeghd_config.ui_n_channels;
  pstr_output_config->i_pcm_wd_sz = (WORD32)p_obj_mpegh_dec->mpeghd_config.ui_pcm_wdsz;
  return;
}

/**
 *  ia_mpegh_dec_get_lib_id_strings
 *
 *  \brief Get library identification strings
 *
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Processing error if any
 *
 */
IA_ERRORCODE ia_mpegh_dec_get_lib_id_strings(pVOID pv_output)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;

  /*Update library name and version number*/
  pstr_output_config->p_lib_name = (WORD8 *)LIBNAME;
  pstr_output_config->p_version_num = (WORD8 *)MPEG_H_3D_AUD_DEC_ITTIAM_VER;

  return err_code;
}

/**
 *  ia_mpegh_dec_create
 *
 *  \brief Create decoder object
 *
 *  \param [in,out] pv_input  Pointer to input config structure
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Processing error if any
 *
 */
IA_ERRORCODE ia_mpegh_dec_create(pVOID pv_input, pVOID pv_output)
{
  pVOID pv_value;
  WORD32 ui_api_size;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  ia_input_config *pstr_input_config = (ia_input_config *)pv_input;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;
  ia_mpegh_dec_api_struct *p_obj_mpegh_dech;

  ui_api_size = sizeof(ia_mpegh_dec_api_struct);
  pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] =
      pstr_output_config->malloc_mpegh(ui_api_size, 4);
  if (NULL == pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count])
  {
    return IA_MPEGH_DEC_API_FATAL_MEM_ALLOC;
  }
  memset(pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count], 0, ui_api_size);

  /*Update library name and version number*/
  pstr_output_config->p_lib_name = (WORD8 *)LIBNAME;
  pstr_output_config->p_version_num = (WORD8 *)MPEG_H_3D_AUD_DEC_ITTIAM_VER;

  /*Errror*/
  pstr_output_config->ui_rem = (UWORD32)(
      (SIZE_T)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] & 3);
  pstr_output_config->pv_ia_process_api_obj =
      (pVOID)((WORD8 *)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] +
              4 - pstr_output_config->ui_rem);
  pstr_output_config->malloc_count++;

  p_obj_mpegh_dech = (ia_mpegh_dec_api_struct *)pstr_output_config->pv_ia_process_api_obj;
  memset(p_obj_mpegh_dech, 0, sizeof(*p_obj_mpegh_dech));
  impeghd_set_default_config(p_obj_mpegh_dech);

  {
    ia_mpegh_dec_tables_struct *pstr_mpeghd_tables = &p_obj_mpegh_dech->mpeghd_tables;
    pstr_mpeghd_tables->pstr_huffmann_tables =
        (ia_mpegh_dec_huffman_tables_struct *)&ia_core_coder_mpeghd_huffmann_tables;
    pstr_mpeghd_tables->pstr_block_tables =
        (ia_mpegh_dec_block_tables_struct *)&ia_core_coder_mpeghd_block_tables;
  }

  err_code = impeghd_set_config_params(p_obj_mpegh_dech, pstr_input_config);

  pstr_output_config->ui_proc_mem_tabs_size =
      (sizeof(ia_mem_info_struct) + sizeof(pVOID *)) * (NUM_MEMTABS);
  pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] =
      pstr_output_config->malloc_mpegh(pstr_output_config->ui_proc_mem_tabs_size, 4);
  if (NULL == pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count])
  {
    return IA_MPEGH_DEC_API_FATAL_MEM_ALLOC;
  }
  memset(pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count], 0,
         pstr_output_config->ui_proc_mem_tabs_size);

  pstr_output_config->ui_rem = (UWORD32)(
      (SIZE_T)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] & 3);
  pv_value =
      (pVOID)((WORD8 *)pstr_output_config->arr_alloc_memory[pstr_output_config->malloc_count] +
              4 - pstr_output_config->ui_rem);
  if (pv_value == NULL)
  {
    return IA_MPEGH_DEC_API_FATAL_MEM_ALLOC;
  }
  memset(pv_value, 0, (sizeof(ia_mem_info_struct) + sizeof(pVOID *)) * (NUM_MEMTABS));

  p_obj_mpegh_dech->p_mem_info_mpeghd = pv_value;
  p_obj_mpegh_dech->pp_mem_mpeghd =
      (pVOID *)((WORD8 *)pv_value + sizeof(ia_mem_info_struct) * NUM_MEMTABS);

  pstr_output_config->malloc_count++;

  ia_core_coder_fill_mpeghd_mem_tables(p_obj_mpegh_dech);

  err_code = impeghd_alloc_and_assign_mem(p_obj_mpegh_dech, pstr_output_config);
  if (err_code)
  {
    return err_code;
  }
  impeghd_update_output_config(p_obj_mpegh_dech, pstr_output_config);

  return err_code;
}

/**
 *  ia_mpegh_dec_init
 *
 *  \brief Initialize the decoder object
 *
 *  \param [in,out] p_ia_mpegh_dec_obj  Pointer to the decoder object
 *  \param [in,out] pv_input  Pointer to input config structure
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Processing error if any
 *
 */
IA_ERRORCODE ia_mpegh_dec_init(pVOID p_ia_mpegh_dec_obj, pVOID pv_input, pVOID pv_output)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  ia_mpegh_dec_api_struct *p_obj_mpegh_dec = p_ia_mpegh_dec_obj;
  ia_input_config *pstr_input_config = (ia_input_config *)pv_input;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;
  ia_mpegh_dec_state_struct *mpeghd_state_struct = p_obj_mpegh_dec->pp_mem_mpeghd[PERSIST_IDX];
  ia_audio_specific_config_struct *pstr_audio_specific_config =
      mpeghd_state_struct->ia_audio_specific_config;
  ia_mae_audio_scene_info *ptr_mae_audio_scene_info = &pstr_audio_specific_config->str_mae_asi;

  pstr_output_config->i_bytes_consumed = 0;
  p_obj_mpegh_dec->p_state_mpeghd->ui_in_bytes = pstr_input_config->num_inp_bytes;
  p_obj_mpegh_dec->mpeghd_config.ui_raw_flag = pstr_input_config->ui_raw_flag;
  p_obj_mpegh_dec->mpeghd_config.ui_binaural_flag = pstr_input_config->binaural_flag;
  if (pstr_input_config->ei_info_flag == 1 && pstr_input_config->ptr_ei_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf ei_jmp_buf;
    IA_ERRORCODE ei_file_read = setjmp(ei_jmp_buf);
    if (ei_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_EI_BYTES;
    }
    ia_dec_data_struct *pstr_dec_data =
        (ia_dec_data_struct *)p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data;
    ia_ele_intrctn *ptr_ele_interaction = &pstr_dec_data->str_element_interaction;
    ia_bit_buf_struct bit_buf_str;
    ia_core_coder_create_init_bit_buf(&bit_buf_str, pstr_input_config->ptr_ei_buf,
                                      pstr_input_config->ei_info_size);
    bit_buf_str.xmpeghd_jmp_buf = &ei_jmp_buf;
    err_code = impeghd_ele_interaction(&bit_buf_str, ptr_ele_interaction,
                                       &pstr_audio_specific_config->str_mae_asi);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }
  if (pstr_input_config->lsi_info_flag == 1 && pstr_input_config->ptr_ls_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf lsi_jmp_buf;
    IA_ERRORCODE lsi_file_read = setjmp(lsi_jmp_buf);
    if (lsi_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_LSI_BYTES;
    }
    ia_dec_data_struct *pstr_dec_data =
        (ia_dec_data_struct *)p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data;
    ia_local_setup_struct *ptr_ele_interface = &pstr_dec_data->str_local_setup_interaction;
    ptr_ele_interface->pstr_binaural_renderer = &pstr_dec_data->str_binaural_rendering;
    ptr_ele_interface->pstr_binaural_renderer->ptr_scratch =
        p_obj_mpegh_dec->pp_mem_mpeghd[SCRATCH_IDX];
    ia_bit_buf_struct bit_buf_str;
    ia_core_coder_create_init_bit_buf(&bit_buf_str, pstr_input_config->ptr_ls_buf,
                                      pstr_input_config->lsi_info_size);
    bit_buf_str.xmpeghd_jmp_buf = &lsi_jmp_buf;
    err_code = impeghd_3da_local_setup_information(
        &bit_buf_str, ptr_ele_interface,
        pstr_audio_specific_config->str_usac_config.signals_3d);
    if (err_code)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return err_code;
    }

    if (1 == ptr_ele_interface->is_brir_rendering)
    {
      p_obj_mpegh_dec->mpeghd_config.ui_binaural_flag = 1;
      memcpy(&ptr_ele_interface->spk_config,
             &ptr_ele_interface->pstr_binaural_renderer->binaural_rep.setup_spk_config_3d,
             sizeof(ia_interface_speaker_config_3d));
    }

    p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx =
        ptr_ele_interface->spk_config.cicp_spk_layout_idx;
  }
  if (pstr_input_config->extrn_rend_flag == 1)
  {
    p_obj_mpegh_dec->mpeghd_config.extrn_rend_flag = 1;
    p_obj_mpegh_dec->mpeghd_config.ptr_ch_md_bit_buf =
        pstr_input_config->ptr_ext_ren_ch_data_buf;
    p_obj_mpegh_dec->mpeghd_config.ptr_oam_md_bit_buf =
        pstr_input_config->ptr_ext_ren_oam_data_buf;
    p_obj_mpegh_dec->mpeghd_config.ptr_hoa_md_bit_buf =
        pstr_input_config->ptr_ext_ren_hoa_data_buf;
    p_obj_mpegh_dec->mpeghd_config.ptr_ext_ren_pcm_buf =
        (WORD8 *)pstr_input_config->ptr_ext_ren_pcm_buf;
  }
  if (pstr_input_config->binaural_flag == 1 && pstr_input_config->ptr_brir_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf brir_jmp_buf;
    IA_ERRORCODE brir_file_read = setjmp(brir_jmp_buf);
    if (brir_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_BRIR_BYTES;
    }
    ia_dec_data_struct *pstr_dec_data =
        (ia_dec_data_struct *)p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data;
    ia_binaural_renderer *pstr_binaural_rendering = &pstr_dec_data->str_binaural_rendering;
    pstr_binaural_rendering->ptr_scratch = p_obj_mpegh_dec->pp_mem_mpeghd[SCRATCH_IDX];
    ia_bit_buf_struct bit_buf_str;
    ia_core_coder_create_init_bit_buf(&bit_buf_str, pstr_input_config->ptr_brir_buf,
                                      pstr_input_config->binaural_data_len);
    bit_buf_str.xmpeghd_jmp_buf = &brir_jmp_buf;
    err_code = impeghd_read_brir_info(pstr_binaural_rendering, &bit_buf_str);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return err_code;
    }
  }
  if (pstr_input_config->maeg_flag == 1 && pstr_input_config->ptr_maeg_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf mae_jmp_buf;
    IA_ERRORCODE mae_file_read = setjmp(mae_jmp_buf);
    if (mae_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_MAE_BYTES;
    }

    ia_bit_buf_struct bit_buf_str;
    ia_core_coder_create_init_bit_buf(&bit_buf_str, pstr_input_config->ptr_maeg_buf,
                                      pstr_input_config->maeg_len);
    bit_buf_str.xmpeghd_jmp_buf = &mae_jmp_buf;
    impeghd_mp4_get_maeg(&bit_buf_str, ptr_mae_audio_scene_info);
    ptr_mae_audio_scene_info->asi_present = 1;
  }
  if (pstr_input_config->maes_flag == 1 && pstr_input_config->ptr_maes_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf mae_jmp_buf;
    IA_ERRORCODE mae_file_read = setjmp(mae_jmp_buf);
    if (mae_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_MAE_BYTES;
    }

    ia_bit_buf_struct bit_buf_str;
    ia_core_coder_create_init_bit_buf(&bit_buf_str, pstr_input_config->ptr_maes_buf,
                                      pstr_input_config->maes_len);
    bit_buf_str.xmpeghd_jmp_buf = &mae_jmp_buf;
    impeghd_mp4_get_maes(&bit_buf_str, ptr_mae_audio_scene_info);
    ptr_mae_audio_scene_info->asi_present = 1;
  }
  if (pstr_input_config->maep_flag == 1 && pstr_input_config->ptr_maep_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf mae_jmp_buf;
    IA_ERRORCODE mae_file_read = setjmp(mae_jmp_buf);
    if (mae_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_MAE_BYTES;
    }

    ia_bit_buf_struct bit_buf_str;
    ia_core_coder_create_init_bit_buf(&bit_buf_str, pstr_input_config->ptr_maep_buf,
                                      pstr_input_config->maep_len);
    bit_buf_str.xmpeghd_jmp_buf = &mae_jmp_buf;
    impeghd_mp4_get_maep(&bit_buf_str, ptr_mae_audio_scene_info);
    ptr_mae_audio_scene_info->asi_present = 1;
  }

  if (pstr_input_config->maei_flag == 1 && pstr_input_config->ptr_maei_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf mae_jmp_buf;
    IA_ERRORCODE mae_file_read = setjmp(mae_jmp_buf);
    if (mae_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_MAE_BYTES;
    }

    ia_bit_buf_struct bit_buf_str;
    ia_core_coder_create_init_bit_buf(&bit_buf_str, pstr_input_config->ptr_maei_buf,
                                      pstr_input_config->maei_len);
    bit_buf_str.xmpeghd_jmp_buf = &mae_jmp_buf;
    impeghd_mp4_get_mael(&bit_buf_str, ptr_mae_audio_scene_info);
    ptr_mae_audio_scene_info->asi_present = 1;
  }

  err_code = ia_core_coder_dec_init(p_obj_mpegh_dec);
  if (pstr_input_config->enable_resamp == 1)
  {
    IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
    if (p_obj_mpegh_dec->mpeghd_config.ui_samp_freq != pstr_input_config->out_samp_freq)
    {
      p_obj_mpegh_dec->mpeghd_config.out_samp_freq = pstr_input_config->out_samp_freq;
      p_obj_mpegh_dec->mpeghd_config.stream_samp_freq =
          p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
      err_code = impeghd_resampler_get_sampling_ratio(&p_obj_mpegh_dec->mpeghd_config);
      if (err_code)
      {
        pstr_input_config->enable_resamp = 0;
      }
    }
  }
  else
  {
    p_obj_mpegh_dec->mpeghd_config.out_samp_freq = p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
    p_obj_mpegh_dec->mpeghd_config.stream_samp_freq =
        p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
  }
  if (p_obj_mpegh_dec->mpeghd_config.ui_binaural_flag)
  {
    ia_binaural_in_stream_cfg_str wav_info;
    ia_dec_data_struct *pstr_dec_data =
        (ia_dec_data_struct *)p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data;
    ia_binaural_renderer *pstr_binaural_rendering = &pstr_dec_data->str_binaural_rendering;
    wav_info.fs_input = p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
    wav_info.num_speaker_expected = (p_obj_mpegh_dec->mpeghd_config.ui_n_channels);
    wav_info.num_channel_is_input = (p_obj_mpegh_dec->mpeghd_config.ui_n_channels);
    if (p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx == 0)
    {
      wav_info.cicp_spk_idx = pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx;
    }
    else
    {
      wav_info.cicp_spk_idx = p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx;
    }

    pstr_binaural_rendering->ptr_scratch =
        p_obj_mpegh_dec->p_state_mpeghd->mpeghd_scratch_mem_v;
    err_code = impeghd_binaural_renderer_init(pstr_binaural_rendering, &wav_info,
                                              &pstr_dec_data->binaural_handle);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }

    pstr_dec_data->binaural_handle.str_binaural.memory_index +=
        2048; // TODO: Replace 2048, with variable
  }

  pstr_output_config->i_bytes_consumed = p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed;
  if (pstr_output_config->i_bytes_consumed == 0)
  {
    pstr_output_config->i_bytes_consumed = 1;
  }

  pstr_output_config->ui_init_done =
      (p_obj_mpegh_dec->p_state_mpeghd->ui_init_done == 1) && (err_code == 0) ? 1 : 0;

  pstr_output_config->i_channel_mask = p_obj_mpegh_dec->mpeghd_config.i_channel_mask;
  if (pstr_input_config->enable_resamp)
  {
    pstr_output_config->i_samp_freq = p_obj_mpegh_dec->mpeghd_config.out_samp_freq;
  }
  else
  {
    pstr_output_config->i_samp_freq = p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
  }
  pstr_output_config->i_num_chan = p_obj_mpegh_dec->mpeghd_config.ui_n_channels;
  pstr_output_config->i_pcm_wd_sz = p_obj_mpegh_dec->mpeghd_config.ui_pcm_wdsz;
  pstr_output_config->i_drc_effect = p_obj_mpegh_dec->mpeghd_config.ui_effect_type;
  pstr_output_config->i_target_loudness = p_obj_mpegh_dec->mpeghd_config.ui_target_loudness;
  pstr_output_config->i_loud_norm = p_obj_mpegh_dec->mpeghd_config.ui_loud_norm_flag;
  if (pstr_input_config->extrn_rend_flag == 1)
  {
    pstr_output_config->ch_data_present = p_obj_mpegh_dec->mpeghd_config.ch_data_present;
    pstr_output_config->ch_md_payload_length =
        p_obj_mpegh_dec->mpeghd_config.ch_md_payload_length;
    pstr_output_config->oam_data_present = p_obj_mpegh_dec->mpeghd_config.oam_data_present;
    pstr_output_config->oam_md_payload_length =
        p_obj_mpegh_dec->mpeghd_config.oam_md_payload_length;
    pstr_output_config->hoa_data_present = p_obj_mpegh_dec->mpeghd_config.hoa_data_present;
    pstr_output_config->hoa_md_payload_length =
        p_obj_mpegh_dec->mpeghd_config.hoa_md_payload_length;
    pstr_output_config->pcm_payload_length = p_obj_mpegh_dec->mpeghd_config.pcm_data_length;
    pstr_output_config->oam_sample_offset = p_obj_mpegh_dec->mpeghd_config.obj_offset;
    pstr_output_config->hoa_sample_offset = p_obj_mpegh_dec->mpeghd_config.hoa_offset;
    pstr_output_config->pcm_bit_depth = 24;
  }

  /* Update speaker configuration in output config */
  if (pstr_output_config->ui_init_done)
  {
    WORD32 spk, spk_idx;
    ia_audio_specific_config_struct *pstr_asc =
        (ia_audio_specific_config_struct *)
            p_obj_mpegh_dec->p_state_mpeghd->ia_audio_specific_config;
    if (1 == p_obj_mpegh_dec->mpeghd_config.ui_binaural_flag)
    {
      pstr_output_config->cicp_index = -1;
      pstr_output_config->spk_layout = -1;
      pstr_output_config->num_speakers = 2;
      pstr_output_config->is_binaural_rendering = 1;
    }
    else
    {
      pstr_output_config->cicp_index = -1;
      pstr_output_config->is_binaural_rendering = 0;
      if (p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx > 0)
      {
        const WORD32 *speaker_table =
          ia_cicp_idx_ls_set_map_tbl[p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx];
        pstr_output_config->cicp_index = p_obj_mpegh_dec->mpeghd_config.ui_cicp_layout_idx;
        pstr_output_config->num_speakers =
            impgehd_cicp_get_num_ls[pstr_output_config->cicp_index];
        pstr_output_config->spk_layout = 0;
        for (spk = 0; spk < pstr_output_config->num_speakers; spk++)
        {
          spk_idx = speaker_table[spk];
          pstr_output_config->azimuth[spk] = (WORD16)ia_cicp_ls_geo_tbls[spk_idx].ls_azimuth;
          pstr_output_config->elevation[spk] =
            (WORD16)ia_cicp_ls_geo_tbls[spk_idx].ls_elevation;
          pstr_output_config->is_lfe[spk] = ia_cicp_ls_geo_tbls[spk_idx].lfe_flag;
        }
      }
      else
      {
        pstr_output_config->spk_layout = pstr_asc->ref_spk_layout.spk_layout_type;
        pstr_output_config->num_speakers = pstr_asc->ref_spk_layout.num_speakers;
        switch (pstr_asc->ref_spk_layout.spk_layout_type)
        {
        case 0:
          pstr_output_config->cicp_index = pstr_asc->ref_spk_layout.cicp_spk_layout_idx;
          pstr_output_config->num_speakers =
              impgehd_cicp_get_num_ls[pstr_output_config->cicp_index];
          const WORD32 *speaker_table =
              ia_cicp_idx_ls_set_map_tbl[pstr_output_config->cicp_index];
          for (spk = 0; spk < pstr_output_config->num_speakers; spk++)
          {
            spk_idx = speaker_table[spk];
            pstr_output_config->azimuth[spk] = (WORD16)ia_cicp_ls_geo_tbls[spk_idx].ls_azimuth;
            pstr_output_config->elevation[spk] =
                (WORD16)ia_cicp_ls_geo_tbls[spk_idx].ls_elevation;
            pstr_output_config->is_lfe[spk] = ia_cicp_ls_geo_tbls[spk_idx].lfe_flag;
          }
          break;
        case 1:
          for (spk = 0; spk < pstr_asc->ref_spk_layout.num_speakers; spk++)
          {
            spk_idx = pstr_asc->ref_spk_layout.cicp_spk_idx[spk];
            pstr_output_config->azimuth[spk] = (WORD16)ia_cicp_ls_geo_tbls[spk_idx].ls_azimuth;
            pstr_output_config->elevation[spk] =
                (WORD16)ia_cicp_ls_geo_tbls[spk_idx].ls_elevation;
            pstr_output_config->is_lfe[spk] = ia_cicp_ls_geo_tbls[spk_idx].lfe_flag;
          }
          break;
        case 2:
          for (spk = 0; spk < pstr_asc->ref_spk_layout.num_speakers; spk++)
          {
            pstr_output_config->azimuth[spk] =
                (WORD16)pstr_asc->ref_spk_layout.str_flex_spk.str_flex_spk_descr[spk]
                    .az_angle_idx;
            pstr_output_config->elevation[spk] =
                (WORD16)pstr_asc->ref_spk_layout.str_flex_spk.str_flex_spk_descr[spk]
                    .el_angle_idx;
            pstr_output_config->is_lfe[spk] =
                (WORD32)pstr_asc->ref_spk_layout.str_flex_spk.str_flex_spk_descr[spk].is_lfe;
          }
          break;
        case 3:
          for (spk = 0; spk < pstr_asc->ref_spk_layout.num_speakers; spk++)
          {
            pstr_output_config->azimuth[spk] = 0;
            pstr_output_config->elevation[spk] = 0;
            pstr_output_config->is_lfe[spk] = 0;
          }
          break;
        }
      }
    }
  }

  ia_dec_data_struct *pstr_dec_data = mpeghd_state_struct->pstr_dec_data;
  ia_ele_intrctn *ptr_ele_intrctn = &pstr_dec_data->str_element_interaction;
  if (ptr_ele_intrctn->ei_data_present && (pstr_audio_specific_config->str_mae_asi.asi_present) &&
      (pstr_audio_specific_config->str_mae_asi.num_groups > MAX_NUM_GROUPS_PRESETS))
  {
    return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ASI_NUM_GROUPS_PRESETS;
  }
  return err_code;
}

/**
 *  ia_mpegh_dec_execute
 *
 *  \brief Execute decoding preocess
 *
 *  \param [in,out] p_ia_mpegh_dec_obj  Pointer to the decoder object
 *  \param [in,out] pv_input  Pointer to input config structure
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Processing error if any
 *
 */
IA_ERRORCODE ia_mpegh_dec_execute(pVOID p_ia_mpegh_dec_obj, pVOID pv_input, pVOID pv_output)
{
  UWORD8 idx = 0;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  ia_mpegh_dec_api_struct *p_obj_mpegh_dec = p_ia_mpegh_dec_obj;
  ia_input_config *pstr_input_config = (ia_input_config *)pv_input;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;
  ia_audio_specific_config_struct *ptr_audio_specific_config;

  if (!p_obj_mpegh_dec->p_state_mpeghd->ui_init_done)
  {
    pstr_output_config->i_bytes_consumed = 1;
    p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed = 1;
    err_code = IA_MPEGH_DEC_INIT_FATAL_ERROR;
    return err_code;
  }

  if (pstr_input_config->sd_info_flag == 1 && pstr_input_config->ptr_sd_buf != NULL)
  {
    /* Create local pointers */
    jmp_buf sd_jmp_buf;
    IA_ERRORCODE sd_file_read = setjmp(sd_jmp_buf);
    if (sd_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      pstr_output_config->i_bytes_consumed = 1;
      return IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_SD_BYTES;
    }
    ia_mpegh_dec_api_struct *p_obj_mpegh_dech =
        (ia_mpegh_dec_api_struct *)pstr_output_config->pv_ia_process_api_obj;
    ia_dec_data_struct *pstr_dec_data =
        (ia_dec_data_struct *)p_obj_mpegh_dech->p_state_mpeghd->pstr_dec_data;
    ia_scene_disp_data *ptr_scene_disp_data = &pstr_dec_data->str_scene_displacement;
    if (ptr_scene_disp_data->sd_buf_init_done == 0)
    {
      ia_core_coder_create_init_bit_buf(&(ptr_scene_disp_data->sd_buf),
                                        pstr_input_config->ptr_sd_buf,
                                        pstr_input_config->sd_info_size);
      ptr_scene_disp_data->sd_buf_init_done = 1;
      ptr_scene_disp_data->min_bits_needed = BITS_FOR_PSDI_DATA;
      ptr_scene_disp_data->sd_buf.xmpeghd_jmp_buf = &sd_jmp_buf;
    }
    if (ptr_scene_disp_data->sd_buf.cnt_bits >= ptr_scene_disp_data->min_bits_needed)
    {
      err_code = impeghd_scene_displacement_data(ptr_scene_disp_data);
      if (err_code)
      {
        return err_code;
      }
    }
  }

  pstr_output_config->i_bytes_consumed = 0;
  p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed = 0;
  p_obj_mpegh_dec->p_state_mpeghd->ui_in_bytes = pstr_input_config->num_inp_bytes;

  err_code = ia_core_coder_dec_execute(p_obj_mpegh_dec);

  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    if (p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed == 0)
    {
      p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed = 1;
    }
    pstr_output_config->i_bytes_consumed = p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed;
  }
  pstr_output_config->i_bytes_consumed = p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed;

  ptr_audio_specific_config = ((ia_audio_specific_config_struct *)
                                   p_obj_mpegh_dec->p_state_mpeghd->ia_audio_specific_config);

  for (idx = 0; idx < MAX_AUDIO_PREROLLS + 1; idx++)
  {
    if (ptr_audio_specific_config->str_usac_config.str_usac_dec_config.preroll_bytes[idx] == 0)
    {
      break;
    }
  }
  pstr_output_config->num_preroll = idx;

  if (p_obj_mpegh_dec->mpeghd_config.discard_au_preroll == 1)
  {
    WORD32 preroll_frame_offset =
        ptr_audio_specific_config->str_usac_config.str_usac_dec_config.preroll_bytes[0];
    WORD8 *pb_out_buf = p_obj_mpegh_dec->pp_mem_mpeghd[OUTPUT_IDX];
    WORD32 num_preroll = (WORD32)pstr_output_config->num_preroll;
    while (num_preroll > 0)
    {
      preroll_frame_offset += ptr_audio_specific_config->str_usac_config.str_usac_dec_config
                                  .preroll_bytes[num_preroll];
      num_preroll--;
    }
    preroll_frame_offset =
        preroll_frame_offset -
        ptr_audio_specific_config->str_usac_config.str_usac_dec_config.preroll_bytes[0];
    pstr_output_config->num_out_bytes =
        ptr_audio_specific_config->str_usac_config.str_usac_dec_config.preroll_bytes[0];
    memmove(pb_out_buf, pb_out_buf + preroll_frame_offset, pstr_output_config->num_out_bytes);
  }
  else
  {
    pstr_output_config->num_out_bytes = p_obj_mpegh_dec->p_state_mpeghd->ui_out_bytes;
  }
  pstr_output_config->i_channel_mask = p_obj_mpegh_dec->mpeghd_config.i_channel_mask;
  if (pstr_input_config->enable_resamp)
  {
    pstr_output_config->i_samp_freq = p_obj_mpegh_dec->mpeghd_config.out_samp_freq;
  }
  else
  {
    pstr_output_config->i_samp_freq = p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
  }
  if (pstr_input_config->extrn_rend_flag == 1)
  {
    pstr_output_config->ch_md_payload_length =
        p_obj_mpegh_dec->mpeghd_config.ch_md_payload_length;
    pstr_output_config->oam_md_payload_length =
        p_obj_mpegh_dec->mpeghd_config.oam_md_payload_length;
    pstr_output_config->hoa_md_payload_length =
        p_obj_mpegh_dec->mpeghd_config.hoa_md_payload_length;
    pstr_output_config->pcm_payload_length = p_obj_mpegh_dec->mpeghd_config.pcm_data_length;
  }
  pstr_output_config->i_num_chan = p_obj_mpegh_dec->mpeghd_config.ui_n_channels;
  return err_code;
}

/**
 *  ia_mpegh_dec_delete
 *
 *  \brief Deletes the decoder object
 *
 *  \param [in,out] pv_output Pointer to output config structure
 *
 *  \return IA_ERRORCODE      Processing error if any
 *
 */
IA_ERRORCODE ia_mpegh_dec_delete(pVOID pv_output)
{
  WORD32 idx;
  ia_output_config *pstr_output_config = (ia_output_config *)pv_output;

  for (idx = pstr_output_config->malloc_count - 1; idx >= 0; idx--)
  {
    if (pstr_output_config->arr_alloc_memory[idx])
    {
      pstr_output_config->free_mpegh(pstr_output_config->arr_alloc_memory[idx]);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_decoder_flush_api
 *
 *  \brief Flush the remaining buffers
 *
 *  \param [in] p_obj_mpegh_dec Pointer to decoder handle structure
 *
 *  \return IA_ERRORCODE         Processing error if any
 *
 */
IA_ERRORCODE
ia_core_coder_decoder_flush_api(ia_mpegh_dec_api_struct *p_obj_mpegh_dec)
{
  IA_ERRORCODE err;
  WORD8 *ptr_scratch = (WORD8 *)p_obj_mpegh_dec->p_state_mpeghd->mpeghd_scratch_mem_v;
  ia_dec_data_struct *pstr_dec_data = p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data;

  memset(&pstr_dec_data->str_frame_data, 0, sizeof(pstr_dec_data->str_frame_data));
  memset(&pstr_dec_data->str_usac_data, 0, sizeof(pstr_dec_data->str_usac_data));
  memset(&pstr_dec_data->str_hoa_dec_handle, 0, sizeof(pstr_dec_data->str_hoa_dec_handle));
  memset(&pstr_dec_data->str_hoa_frame_data, 0, sizeof(pstr_dec_data->str_hoa_frame_data));
  memset(&pstr_dec_data->str_obj_ren_dec_state, 0, sizeof(pstr_dec_data->str_obj_ren_dec_state));
  memset(&pstr_dec_data->str_drc_payload, 0, sizeof(pstr_dec_data->str_drc_payload));
  memset(&pstr_dec_data->drc_persistent_buf[0], 0, sizeof(pstr_dec_data->drc_persistent_buf));
  memset(&pstr_dec_data->str_peak_limiter, 0, sizeof(pstr_dec_data->str_peak_limiter));
  memset(&pstr_dec_data->str_interaction_config, 0,
         sizeof(pstr_dec_data->str_interaction_config));
  memset(&pstr_dec_data->str_enh_obj_md_frame, 0, sizeof(pstr_dec_data->str_enh_obj_md_frame));
  memset(&pstr_dec_data->str_domain_switcher, 0, sizeof(pstr_dec_data->str_domain_switcher));
  memset(&pstr_dec_data->binaural_handle, 0, sizeof(pstr_dec_data->binaural_handle));
  memset(&pstr_dec_data->str_format_converter, 0, sizeof(pstr_dec_data->str_format_converter));
  memset(&pstr_dec_data->str_earcon_format_converter, 0,
         sizeof(pstr_dec_data->str_earcon_format_converter));
  memset(&pstr_dec_data->str_resampler, 0, sizeof(pstr_dec_data->str_resampler));
  memset(&pstr_dec_data->str_earcon_resampler, 0, sizeof(pstr_dec_data->str_earcon_resampler));
  p_obj_mpegh_dec->mpeghd_config.ui_samp_freq = 0;
  p_obj_mpegh_dec->p_state_mpeghd->frame_counter = 0;
  p_obj_mpegh_dec->p_state_mpeghd->decode_create_done = 0;

  err = ia_core_coder_dec_init(p_obj_mpegh_dec);
  p_obj_mpegh_dec->p_state_mpeghd->flush = 1;
  return err;
}

/**
 *  ia_core_coder_dec_init
 *
 *  \brief Initialize the decoder
 *
 *  \param [in,out] p_obj_mpegh_dec Pointer to decoder handle structure
 *
 *  \return IA_ERRORCODE            Processing error if any
 *
 */
IA_ERRORCODE
ia_core_coder_dec_init(ia_mpegh_dec_api_struct *p_obj_mpegh_dec)
{

  UWORD8 *in_buffer;

  ia_mpegh_dec_state_struct *p_state_mpegh_dec;

  IA_ERRORCODE error_code = IA_MPEGH_DEC_NO_ERROR;
  p_obj_mpegh_dec->p_state_mpeghd = p_obj_mpegh_dec->pp_mem_mpeghd[PERSIST_IDX];
  if (p_obj_mpegh_dec->pp_mem_mpeghd[PERSIST_IDX] == NULL)
  {
    return IA_MPEGH_DEC_INIT_FATAL_NULL_PTR_PERSIST_MEM;
  }
  jmp_buf ifile_init_jmp_buf;
  if (p_obj_mpegh_dec->p_state_mpeghd != NULL)
  {
    IA_ERRORCODE input_file_read = setjmp(ifile_init_jmp_buf);
    if (input_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      if (p_obj_mpegh_dec != NULL)
      {
        p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed =
            p_obj_mpegh_dec->p_state_mpeghd->ui_in_bytes;
        p_obj_mpegh_dec->p_state_mpeghd->ui_out_bytes = 0;
      }
      return IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_INPUT_BYTES;
    }
  }

  in_buffer = p_obj_mpegh_dec->pp_mem_mpeghd[INPUT_IDX];

  p_state_mpegh_dec = p_obj_mpegh_dec->p_state_mpeghd;
  p_state_mpegh_dec->xmpeghd_jmp_buf = &ifile_init_jmp_buf;
  p_state_mpegh_dec->mpeghd_scratch_mem_v = p_obj_mpegh_dec->pp_mem_mpeghd[SCRATCH_IDX];
  p_obj_mpegh_dec->p_state_mpeghd->huffman_code_book_scl =
      p_obj_mpegh_dec->mpeghd_tables.pstr_huffmann_tables->huffman_code_book_scl;
  p_obj_mpegh_dec->p_state_mpeghd->huffman_code_book_scl_index =
      p_obj_mpegh_dec->mpeghd_tables.pstr_huffmann_tables->huffman_code_book_scl_index;

  p_state_mpegh_dec->pstr_mpeghd_tables = &p_obj_mpegh_dec->mpeghd_tables;
  if (p_obj_mpegh_dec->mpeghd_config.header_dec_done == 0)
  {
    p_obj_mpegh_dec->p_state_mpeghd->p_config = &p_obj_mpegh_dec->mpeghd_config;
    p_state_mpegh_dec->mpeghd_persistent_mem_v =
        (pVOID)((SIZE_T)((pWORD8)p_obj_mpegh_dec->p_state_mpeghd +
                         sizeof(ia_mpegh_dec_state_struct) + sizeof(SIZE_T) - 1) &
                (SIZE_T)(~(sizeof(SIZE_T) - 1)));

    p_state_mpegh_dec->preroll_config_present = 0;
    memset(p_state_mpegh_dec->preroll_config_prev, 0,
           sizeof(p_state_mpegh_dec->preroll_config_prev));
    p_obj_mpegh_dec->mpeghd_config.header_dec_done = 1;
  }

  if (p_obj_mpegh_dec->p_state_mpeghd->header_dec_done == 0)
  {

    if (p_obj_mpegh_dec->mpeghd_config.ui_samp_freq == 0)
    {
      WORD32 header_bytes_consumed = 0, return_val = 0;

      if (p_state_mpegh_dec->ui_in_bytes == 0)
      {
        p_state_mpegh_dec->i_bytes_consumed = 0;
        return IA_MPEGH_DEC_NO_ERROR;
      }
      return_val = ia_core_coder_headerdecode(p_obj_mpegh_dec, (UWORD8 *)in_buffer,
                                              &header_bytes_consumed);
      if (return_val < 0)
      {
        if (return_val == (WORD32)IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX)
        {
          p_state_mpegh_dec->i_bytes_consumed = header_bytes_consumed;
          return return_val;
        }
        // p_state_mpegh_dec->i_bytes_consumed = 1;

        return return_val;
      }

      if (return_val == IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES)
      {
        p_state_mpegh_dec->i_bytes_consumed = header_bytes_consumed;
        return return_val;
      }

      p_state_mpegh_dec->i_bytes_consumed = header_bytes_consumed;

      if (return_val == 0)
      {
        {
          WORD32 pcm_size = p_obj_mpegh_dec->mpeghd_config.ui_pcm_wdsz;
          WORD8 *inbuffer = p_obj_mpegh_dec->pp_mem_mpeghd[INPUT_IDX];
          WORD8 *outbuffer = p_obj_mpegh_dec->pp_mem_mpeghd[OUTPUT_IDX];
          WORD32 out_bytes = 0;
          WORD32 frames_done = p_obj_mpegh_dec->p_state_mpeghd->frame_counter;
          p_obj_mpegh_dec->p_state_mpeghd->flush = 0;
          error_code = ia_core_coder_dec_main(p_obj_mpegh_dec, inbuffer, outbuffer, &out_bytes,
                                              frames_done, pcm_size,
                                              &p_obj_mpegh_dec->p_state_mpeghd->num_of_output_ch);
          if (error_code)
            return error_code;
          p_obj_mpegh_dec->p_state_mpeghd->frame_counter++;

          p_obj_mpegh_dec->mpeghd_config.ui_n_channels =
              p_obj_mpegh_dec->p_state_mpeghd->num_of_output_ch;
        }
        if (return_val == 0)
          p_obj_mpegh_dec->p_state_mpeghd->ui_init_done = 1;
        return return_val;
      }
    }
    else
    {
      p_obj_mpegh_dec->p_state_mpeghd->header_dec_done = 1;
      p_state_mpegh_dec->i_bytes_consumed = 0;

      p_state_mpegh_dec->sampling_rate = p_obj_mpegh_dec->mpeghd_config.ui_samp_freq;
    }

    p_state_mpegh_dec->pstr_bit_buf =
        ia_core_coder_create_bit_buf(&p_state_mpegh_dec->str_bit_buf, (UWORD8 *)in_buffer,
                                     p_obj_mpegh_dec->p_mem_info_mpeghd[INPUT_IDX].ui_size);
    p_state_mpegh_dec->pstr_bit_buf->xmpeghd_jmp_buf = p_state_mpegh_dec->xmpeghd_jmp_buf;
  }
  else
  {
    struct ia_bit_buf_struct temp_bit_buff = {0};
    struct ia_bit_buf_struct *it_bit_buff;

    it_bit_buff = p_state_mpegh_dec->pstr_bit_buf;

    p_obj_mpegh_dec->p_state_mpeghd->ui_out_bytes = 0;

    if (p_state_mpegh_dec->ui_in_bytes == 0)
    {
      p_state_mpegh_dec->i_bytes_consumed = 0;
      return IA_MPEGH_DEC_NO_ERROR;
    }
    ia_core_coder_create_init_bit_buf(it_bit_buff, in_buffer, p_state_mpegh_dec->ui_in_bytes);
    p_state_mpegh_dec->pstr_bit_buf->xmpeghd_jmp_buf = p_state_mpegh_dec->xmpeghd_jmp_buf;

    memcpy(&temp_bit_buff, it_bit_buff, sizeof(struct ia_bit_buf_struct));
  }
  return error_code;
}

/**
 *  ia_core_coder_dec_execute
 *
 *  \brief Main processing function
 *
 *  \param [in,out] p_obj_mpegh_dec Pointer to decoder handle structure
 *
 *  \return IA_ERRORCODE            Processing error if any
 *
 */
IA_ERRORCODE
ia_core_coder_dec_execute(ia_mpegh_dec_api_struct *p_obj_mpegh_dec)
{
  ia_mpegh_dec_state_struct *p_state_mpegh_dec;
  IA_ERRORCODE error_code = IA_MPEGH_DEC_NO_ERROR;
  jmp_buf ifile_init_jmp_buf;
  IA_ERRORCODE input_file_read = setjmp(ifile_init_jmp_buf);
  if (p_obj_mpegh_dec->p_state_mpeghd != NULL)
  {
    if (input_file_read != IA_MPEGH_DEC_NO_ERROR)
    {
      p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed =
          p_obj_mpegh_dec->p_state_mpeghd->ui_in_bytes;
      p_obj_mpegh_dec->p_state_mpeghd->ui_out_bytes = 0;
      return IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES;
    }
  }
  p_state_mpegh_dec = p_obj_mpegh_dec->p_state_mpeghd;
  p_state_mpegh_dec->mpeghd_scratch_mem_v = p_obj_mpegh_dec->pp_mem_mpeghd[SCRATCH_IDX];

  p_state_mpegh_dec->i_bytes_consumed = 0;
  p_state_mpegh_dec->xmpeghd_jmp_buf = &ifile_init_jmp_buf;

  WORD32 pcm_size = p_obj_mpegh_dec->mpeghd_config.ui_pcm_wdsz;
  WORD8 *inbuffer = (WORD8 *)(p_obj_mpegh_dec->pp_mem_mpeghd[INPUT_IDX]);
  WORD8 *outbuffer = (WORD8 *)(p_obj_mpegh_dec->pp_mem_mpeghd[OUTPUT_IDX]);
  WORD32 out_bytes = 0;

  WORD32 frames_done = p_obj_mpegh_dec->p_state_mpeghd->frame_counter;

  ia_dec_data_struct *pstr_dec_data =
      (ia_dec_data_struct *)(p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data);
  {
    ia_audio_specific_config_struct *ptr_audio_specific_config =
        ((ia_audio_specific_config_struct *)
             p_obj_mpegh_dec->p_state_mpeghd->ia_audio_specific_config);

    ptr_audio_specific_config->str_usac_config.str_usac_dec_config.preroll_counter = 0;
    {
      WORD32 iii = 0;
      for (iii = 0; iii < (MAX_AUDIO_PREROLLS + 1); iii++)
      {
        ((ia_dec_data_struct *)(p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data))
            ->str_frame_data.str_audio_specific_config.str_usac_config.str_usac_dec_config
            .usac_ext_gain_payload_len[iii] = 0;
        ptr_audio_specific_config->str_usac_config.str_usac_dec_config
            .usac_ext_gain_payload_len[iii] = 0; // reinitilize the payload len buff
        ptr_audio_specific_config->str_usac_config.str_usac_dec_config.preroll_bytes[iii] = 0;
      }
    }

    ((ia_dec_data_struct *)(p_obj_mpegh_dec->p_state_mpeghd->pstr_dec_data))
        ->str_frame_data.str_audio_specific_config.str_usac_config.str_usac_dec_config
        .preroll_counter = 0;

    if (p_obj_mpegh_dec->p_state_mpeghd->num_of_output_ch > MAX_NUM_CHANNELS)
      return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
    error_code =
        ia_core_coder_dec_main(p_obj_mpegh_dec, inbuffer, outbuffer, &out_bytes, frames_done,
                               pcm_size, &p_obj_mpegh_dec->p_state_mpeghd->num_of_output_ch);

    if (error_code != IA_MPEGH_DEC_NO_ERROR)
    {
      if (pstr_dec_data->dec_bit_buf.size != 0 && pstr_dec_data->dec_bit_buf.cnt_bits >= 0)
      {
        p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed =
            (pstr_dec_data->dec_bit_buf.size - pstr_dec_data->dec_bit_buf.cnt_bits + 7) >> 3;
      }
      else
      {
        p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed = 1;
      }
      return error_code;
    }
  }

  if (p_obj_mpegh_dec->p_state_mpeghd->flush == 0)
  {
    p_obj_mpegh_dec->p_state_mpeghd->frame_counter++;
    if (pstr_dec_data->dec_bit_buf.size != 0 && pstr_dec_data->dec_bit_buf.cnt_bits >= 0)
    {
      p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed =
          (pstr_dec_data->dec_bit_buf.size - pstr_dec_data->dec_bit_buf.cnt_bits + 7) >> 3;
    }
    else
    {
      p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed = 1;
    }
  }

  p_obj_mpegh_dec->p_state_mpeghd->ui_out_bytes = out_bytes;
  p_obj_mpegh_dec->mpeghd_config.ui_n_channels =
      p_obj_mpegh_dec->p_state_mpeghd->num_of_output_ch;

  return 0;
}
/** @} */ /* End of MPEGHDecAPIs */
