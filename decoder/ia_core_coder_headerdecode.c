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
#include <ia_core_coder_constants.h>
#include "ia_core_coder_defines.h"

#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"

#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include <ia_core_coder_info.h>
#include <ia_core_coder_rom.h>
#include "ia_core_coder_dec.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_definitions.h"
#include "ia_core_coder_env_extr.h"
#include "ia_core_coder_function_selector.h"
#include "impeghd_memory_standards.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_struct_def.h"
#include "ia_core_coder_headerdecode.h"
#include "impeghd_error_codes.h"
#include "impeghd_mhas_parse.h"
#include "ia_core_coder_struct.h"

/**
 * @defgroup CoreDecConfParse Core Decoder Config Data Parsing
 * @ingroup  CoreDecConfParse
 * @brief Core Decoder Config Data Parsing
 *
 * @{
 */

/**
 *  impegh_3daudio_config_dec
 *
 *  \brief Decode 3d audio config
 *
 *  \param[in,out]    mpeghd_state_struct  decoder state structure
 *  \param [out]    bytes_consumed    Header bytes consumed
 *  \param [in]      it_bit_buff      bit stream buffer
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE
impegh_3daudio_config_dec(ia_mpegh_dec_state_struct *mpeghd_state_struct, WORD32 *bytes_consumed,
                          struct ia_bit_buf_struct *it_bit_buff)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 grp;
  ia_audio_specific_config_struct *pstr_asc = mpeghd_state_struct->ia_audio_specific_config;
  ia_usac_config_struct *pstr_usac_conf = &(pstr_asc->str_usac_config);
  if (!pstr_asc->str_mae_asi.asi_present)
  {
    memset(mpeghd_state_struct->ia_audio_specific_config, 0,
           sizeof(ia_audio_specific_config_struct));
  }

  ia_core_coder_conf_default(&(pstr_asc->str_usac_config));

  err = ia_core_coder_mpegh_3da_config(it_bit_buff, mpeghd_state_struct, &pstr_asc->str_mae_asi);
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }
  if (pstr_usac_conf->signals_3d.signal_grp_info_present != 1)
  {
    for (grp = 0; grp < (WORD32)pstr_usac_conf->signals_3d.num_sig_group; grp++)
    {
      pstr_usac_conf->signals_3d.group_priority[grp] = 7;
      pstr_usac_conf->signals_3d.fixed_position[grp] = 1;
    }
  }

  if ((SIZE_T)it_bit_buff->ptr_read_next == (SIZE_T)it_bit_buff->ptr_bit_buf_base)
  {
    *bytes_consumed = ((WORD32)it_bit_buff->size) >> 3;
  }
  else
  {
    *bytes_consumed =
        (WORD32)(((((SIZE_T)it_bit_buff->ptr_read_next - (SIZE_T)it_bit_buff->ptr_bit_buf_base))
                  << 3) +
                 7 - it_bit_buff->bit_pos + 7) >>
        3;
  }
  return err;
}
/**
 *  ia_core_coder_headerdecode
 *
 *  \brief Decode header
 *
 *  \param [in]  p_obj_mpegh_dec Decoder api structure
 *  \param [in]  buffer               Input bit buffer stream
 *  \param [out] bytes_consumed       Header bytes read
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_headerdecode(ia_mpegh_dec_api_struct *p_obj_mpegh_dec, UWORD8 *buffer,
                                        WORD32 *bytes_consumed)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  struct ia_bit_buf_struct it_bit_buff = {0}, *handle_bit_buff;
  WORD32 header_len;
  WORD32 bit_offset;
  WORD32 is_ga_header = 0;
  WORD32 ui_raw_flag;

  ia_mpegh_dec_state_struct *mpeghd_state_struct = p_obj_mpegh_dec->pp_mem_mpeghd[PERSIST_IDX];

  if (buffer == 0)
  {
    return IA_MPEGH_DEC_INIT_FATAL_DEC_INIT_FAIL;
  }
  ui_raw_flag = p_obj_mpegh_dec->mpeghd_config.ui_raw_flag;
  is_ga_header = p_obj_mpegh_dec->mpeghd_config.ui_mhas_flag | ui_raw_flag;

  header_len = mpeghd_state_struct->ui_in_bytes;

  handle_bit_buff =
      ia_core_coder_create_bit_buf(&it_bit_buff, (UWORD8 *)buffer, (WORD16)header_len);
  handle_bit_buff->cnt_bits += (header_len << 3);
  handle_bit_buff->xmpeghd_jmp_buf = mpeghd_state_struct->xmpeghd_jmp_buf;
  if (1 == is_ga_header)
  {
    ia_mhas_pac_info info;
    ia_audio_specific_config_struct *pstr_asc =
        (ia_audio_specific_config_struct *)mpeghd_state_struct->ia_audio_specific_config;

    info.packet_lbl = 0;
    info.packet_length = 0;
    info.packet_type = 0;

    if (0 == ui_raw_flag)
    {
      impeghd_mhas_parse(&info, &pstr_asc->str_mae_asi, handle_bit_buff);
      if (MHAS_PAC_TYP_MPEGH3DACFG != info.packet_type)
      {
        p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed = info.packet_length;
        return IA_MPEGH_DEC_INIT_FATAL_3DACONFIG_DATA_NOT_FOUND;
      }
    }
    {
      bit_offset = handle_bit_buff->cnt_bits;
      if (mpeghd_state_struct->prev_cfg_len == 0)
      {
        WORD32 i = 0;
        WORD32 bit_pos = handle_bit_buff->bit_pos;
        WORD32 cnt_bits = handle_bit_buff->cnt_bits;
        UWORD8 *ptr_read_next = handle_bit_buff->ptr_read_next;
        mpeghd_state_struct->prev_cfg_len = info.packet_length;
        for (i = 0; i < info.packet_length; i++)
        {
          mpeghd_state_struct->prev_cfg_data[i] = ia_core_coder_read_bits_buf(handle_bit_buff, 8);
        }
        handle_bit_buff->bit_pos = bit_pos;
        handle_bit_buff->cnt_bits = cnt_bits;
        handle_bit_buff->ptr_read_next = ptr_read_next;
      }
      err = impegh_3daudio_config_dec(mpeghd_state_struct, bytes_consumed, handle_bit_buff);
      if (err != IA_MPEGH_DEC_NO_ERROR)
      {
        p_obj_mpegh_dec->p_state_mpeghd->i_bytes_consumed = info.packet_length;
        return err;
      }
      if (0 == ui_raw_flag)
      {
        bit_offset = bit_offset - handle_bit_buff->cnt_bits + 7;
        bit_offset = bit_offset >> 3;
        if ((bit_offset > info.packet_length))
        {
          return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
        }
      }
    }
    return err;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecConfParse */