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

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include <ia_core_coder_interface.h>
#include <ia_core_coder_info.h>
#include "ia_core_coder_rom.h"
#include "ia_core_coder_dec.h"

#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "impeghd_memory_standards.h"
#include "impeghd_config_params.h"
#include "impeghd_error_codes.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_struct.h"
#include "ia_core_coder_struct_def.h"
/* Decoder interfaces */
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_hoa_config.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_oam_dec.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_obj_ren_dec.h"

/**
 * @defgroup CoreDecInit Core Decoder initialization
 * @ingroup  CoreDecInit
 * @brief Core Decoder initialization
 *
 * @{
 */

/**
 *  impeghd_prod_metadata_config
 *
 *  \brief Production metadata config
 *
 *  \param [in]       ptr_bit_buf              Bitstrean buffer
 *  \param [in,out]   ptr_usac_config_struct   USAC config structure
 *
 *  \return IA_ERRORCODE    error code
 *
 */
static IA_ERRORCODE impeghd_prod_metadata_config(ia_bit_buf_struct *ptr_bit_buf,
                                                 ia_usac_config_struct *ptr_usac_config_struct)
{
  WORD32 grp_cnt;
  ptr_usac_config_struct->str_prod_metat_data.has_reference_distance =
      ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  if (!ptr_usac_config_struct->str_prod_metat_data.has_reference_distance)
  {
    ptr_usac_config_struct->str_prod_metat_data.bs_reference_distance =
        DIFFAULT_PROD_METADATA_DISTANCE;
  }
  else
  {
    ptr_usac_config_struct->str_prod_metat_data.bs_reference_distance =
        ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
  }

  for (grp_cnt = 0; grp_cnt < (WORD32)ptr_usac_config_struct->signals_3d.num_obj_based_groups;
       grp_cnt++)
  {
    ptr_usac_config_struct->str_prod_metat_data.has_object_distance[grp_cnt] =
        ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }
  for (grp_cnt = 0; grp_cnt < (WORD32)ptr_usac_config_struct->signals_3d.num_ch_based_groups;
       grp_cnt++)
  {
    ptr_usac_config_struct->str_prod_metat_data.direct_head_phone[grp_cnt] =
        ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_read_escape_value
 *
 *  \brief Method to read an integer value using varying number of bits. 2level escape mechanism
* which
*   allows to extend the representable range of values by successive reception of additional bits.
 *
 *  \param [in]    it_bit_buff     Bitstrean buffer
 *  \param [out]   ext_ele_value   Value read from the stream
 *  \param [in]    num_bits_1      First set of Bits
 *  \param [in]    num_bits_2      Second set of Bits
 *  \param [in]    num_bits_3      Third set of Bits
 *
 *
 *
 */
VOID ia_core_coder_read_escape_value(ia_bit_buf_struct *it_bit_buff, UWORD32 *ext_ele_value,
                                     UWORD32 num_bits_1, UWORD32 num_bits_2, UWORD32 num_bits_3)
{
  UWORD32 value = 0;
  UWORD32 esc_val_1 = (1 << num_bits_1) - 1;
  UWORD32 esc_val_2 = (1 << num_bits_2) - 1;

  value = ia_core_coder_read_bits_buf(it_bit_buff, num_bits_1);

  if (value == esc_val_1)
  {
    value = ia_core_coder_read_bits_buf(it_bit_buff, num_bits_2);

    if (value == esc_val_2)
    {
      value = ia_core_coder_read_bits_buf(it_bit_buff, num_bits_3);

      value += esc_val_2;
    }

    value += esc_val_1;
  }

  *ext_ele_value = value;
}

/**
 *  ia_core_coder_ext_element_config
 *
 *  \brief Configure extension elements in USAC config structure
 *
 *  \param [in]    it_bit_buff            Bit stream buffer
 *  \param [in,out]  pstr_usac_element_config    Decoder element config structure
 *  \param [in]    ptr_usac_ext_ele_payload    Pointer buffer of extension
 * payload
 *  \param [in]    ptr_usac_ext_ele_payload_len  Payload length of extension element
 *  \param [in]    preroll_flag          Flag to indicate pre-roll
 * frame
 *  \param [in,out]  ia_ext_ele_payload_type      Extension payload type
 *  \param [in]    pstr_usac_conf          USAC config structure
 *  \param [in]    ccfl              Frame Length
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_ext_element_config(
    ia_bit_buf_struct *it_bit_buff, ia_usac_dec_element_config_struct *pstr_usac_element_config,
    UWORD8 *ptr_usac_ext_ele_payload, WORD32 *ptr_usac_ext_ele_payload_len, WORD32 *preroll_flag,
    WORD32 *ia_ext_ele_payload_type, ia_usac_config_struct *pstr_usac_conf, WORD32 ccfl,
    WORD32 elem_idx)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 cnt_bits, skip_bits;
  UWORD32 usac_ext_element_type, usac_ext_element_config_length, flag;
  UWORD32 i;
  UWORD32 num_aud_preroll = 0, num_uni_drc = 0;
  ia_oam_dec_config_struct *p_obj_md_cfg = &pstr_usac_conf->obj_md_cfg;
  ia_hoa_config_struct *pstr_hoa_config = &pstr_usac_conf->str_usac_dec_config.str_hoa_config;
  WORD32 num_objects = pstr_usac_conf->signals_3d.num_audio_obj;

  ia_core_coder_read_escape_value(it_bit_buff, &(usac_ext_element_type), 4, 8, 16);
  *ia_ext_ele_payload_type = usac_ext_element_type;

  ia_core_coder_read_escape_value(it_bit_buff, &(usac_ext_element_config_length), 4, 8, 16);
  if (usac_ext_element_config_length >= MAX_EXT_ELE_PAYLOAD)
  {
    return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
  }

  *ptr_usac_ext_ele_payload_len = 0;

  flag = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (!flag)
  {
    pstr_usac_element_config->usac_ext_eleme_def_len = 0;
  }
  else
  {
    ia_core_coder_read_escape_value(
        it_bit_buff, (UWORD32 *)(&(pstr_usac_element_config->usac_ext_eleme_def_len)), 8, 16, 0);
    pstr_usac_element_config->usac_ext_eleme_def_len += 1;
  }

  pstr_usac_element_config->usac_ext_elem_pld_frag = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  switch (usac_ext_element_type)
  {
  case ID_EXT_ELE_PROD_METADATA:
  {
    err_code = impeghd_prod_metadata_config(it_bit_buff, pstr_usac_conf);
    if (err_code)
    {
      return err_code;
    }
    break;
  }
  case ID_MPEGH_EXT_ELE_ENHANCED_OBJ_METADATA:
  {
    ia_enh_oam_config_struct *p_enh_obj_md_cfg = &pstr_usac_conf->enh_obj_md_cfg;
    cnt_bits = it_bit_buff->cnt_bits;
    if (num_objects > MAX_NUM_OAM_OBJS)
    {
      return IA_MPEGH_OAM_INIT_FATAL_UNSUPPORTED_NUM_OBJS;
    }
    /*Enhanced Object Metadata parsing function will be called here*/
    err_code = impeghd_enh_obj_md_config(p_enh_obj_md_cfg, it_bit_buff, num_objects);
    if (err_code)
    {
      return err_code;
    }
    cnt_bits = cnt_bits - it_bit_buff->cnt_bits;
    if (cnt_bits > (WORD32)(usac_ext_element_config_length << 3))
    {
      return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
    }
    else if (cnt_bits < (WORD32)(usac_ext_element_config_length << 3))
    {
      skip_bits = (usac_ext_element_config_length << 3) - cnt_bits;
      ia_core_coder_skip_bits_buf(it_bit_buff, skip_bits);
    }
    pstr_usac_conf->enh_obj_md_present = 1;
    break;
  }
  case ID_MPEGH_EXT_ELE_HOA:
  {
    *ptr_usac_ext_ele_payload_len = usac_ext_element_config_length;
    pstr_hoa_config->core_coder_frame_length = ccfl;
    cnt_bits = it_bit_buff->cnt_bits;
    err_code = impeghd_fill_config(it_bit_buff, pstr_hoa_config);
    if (err_code)
      return err_code;
    cnt_bits = (cnt_bits - it_bit_buff->cnt_bits);
    if (cnt_bits > (WORD32)(usac_ext_element_config_length << 3))
    {
      return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
    }
    else if (cnt_bits < (WORD32)(usac_ext_element_config_length << 3))
    {
      skip_bits = (usac_ext_element_config_length << 3) - cnt_bits;
      ia_core_coder_skip_bits_buf(it_bit_buff, skip_bits);
    }
    break;
  }
  case ID_MPEGH_EXT_ELE_OAM:
  {
    *ptr_usac_ext_ele_payload_len = usac_ext_element_config_length;
    cnt_bits = it_bit_buff->cnt_bits;
    impeghd_obj_md_cfg(p_obj_md_cfg, it_bit_buff, ccfl, num_objects);
    cnt_bits = cnt_bits - it_bit_buff->cnt_bits;
    if (cnt_bits > (WORD32)(usac_ext_element_config_length << 3))
    {
      return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
    }
    else if (cnt_bits < (WORD32)(usac_ext_element_config_length << 3))
    {
      skip_bits = (usac_ext_element_config_length << 3) - cnt_bits;
      ia_core_coder_skip_bits_buf(it_bit_buff, skip_bits);
    }
    break;
  }
  case ID_EXT_ELE_UNI_DRC:
  case ID_MPEGH_EXT_ELE_MCT:
    if (ID_EXT_ELE_UNI_DRC == usac_ext_element_type)
    {
      num_uni_drc++;
      if (num_uni_drc > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_EXT_ELEM_CONFIG;
      }
    }
    if (pstr_usac_element_config->usac_ext_elem_pld_frag != 0)
    {
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_EXT_ELEM_CONFIG;
    }
    *ptr_usac_ext_ele_payload_len = usac_ext_element_config_length;
    if ((ID_MPEGH_EXT_ELE_MCT == usac_ext_element_type) && (*ptr_usac_ext_ele_payload_len == 0))
    {
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_EXT_ELEM_CONFIG;
    }
    for (i = 0; i < usac_ext_element_config_length; i++)
    {
      ptr_usac_ext_ele_payload[i] = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 8);
    }
    break;
  case ID_EXT_ELE_AUDIOPREROLL:
    num_aud_preroll++;
    if ((elem_idx != 0) || (pstr_usac_element_config->usac_ext_elem_pld_frag != 0) ||
        (flag != 0) || (usac_ext_element_config_length != 0) || (num_aud_preroll > 1))
    {
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_EXT_ELEM_CONFIG;
    }
    *preroll_flag = 1;
    break;
  case ID_EXT_ELE_FILL:
    if ((usac_ext_element_config_length != 0) ||
        (pstr_usac_element_config->usac_ext_elem_pld_frag != 0))
    {
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_EXT_ELEM_CONFIG;
    }
    break;
  default:
    if ((it_bit_buff->cnt_bits >> 3) < (WORD32)usac_ext_element_config_length)
    {
      return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
    }
    it_bit_buff->ptr_read_next += usac_ext_element_config_length;
    it_bit_buff->cnt_bits -= (usac_ext_element_config_length << 3);
    break;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_map_ele_to_channel
 *
 *  \brief Maps element to number of output channel
 *
 *  \param [in,out]    num_output_chns    Number of output channel
 *  \param [in]      shift_index      Shift index
 *  \param [in]      shift_channel    Shift channel
 *
 *
 *
 */
static VOID ia_core_coder_map_ele_to_channel(WORD32 *num_output_chns, WORD32 shift_index,
                                             WORD32 shift_channel, WORD32 *index)
{
  WORD32 i, j, next_idx = 0, channel = 0;

  for (i = 0; i < 16; i++)
  {
    for (j = 0; j < 16; j++)
    {
      if (num_output_chns[j] == channel)
      {
        channel++;
      }
    }
  }
  for (i = 0; i < 16; i++)
  {
    if (num_output_chns[i] < 0)
    {
      next_idx = i;
      break;
    }
  }
  if (shift_index <= 0)
  {
    num_output_chns[next_idx] = channel;
  }
  else
  {
    num_output_chns[next_idx] = channel + shift_channel + 1;
  }
  *index = next_idx;
}

/**
 *  ia_core_coder_cpe_config
 *
 *  \brief Update cpe configurations in USAC elementconfig structure
 *
 *  \param [in]    it_bit_buff            Bit stream buffer
 *  \param [in,out]  pstr_usac_element_config    USAC decoder element config
 * structure
 *  \param [in]    sbr_ratio_index          sbr ratio index
 *  \param [in]    num_output_chns          number of output channels
 *  \param [in]    ia_signals_3da          3DA structure in USAC
 * config
 * structure
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_cpe_config(
    ia_bit_buf_struct *it_bit_buff, ia_usac_dec_element_config_struct *pstr_usac_element_config,
    WORD32 sbr_ratio_index, WORD32 *num_output_chns, ia_signals_3d *ia_signals_3da)
{
  WORD32 val = 0, nbits, shift_idx1 = 0, shift_idx0 = 0, shift_channel1 = 0, shift_channel0 = 0,
         qce_index, index = 0;
  if ((ia_signals_3da->num_saoc_transport_ch + ia_signals_3da->num_hoa_transport_ch +
       ia_signals_3da->num_ch + ia_signals_3da->num_audio_obj) != 0)
  {
    switch (((ia_signals_3da->num_saoc_transport_ch + ia_signals_3da->num_hoa_transport_ch +
              ia_signals_3da->num_ch + ia_signals_3da->num_audio_obj) -
             1))
    {
    case 15:
    case 14:
    case 13:
    case 12:
    case 11:
    case 10:
    case 9:
    case 8:
      val = 3;
      break;
    case 7:
    case 6:
    case 5:
    case 4:
      val = 2;
      break;
    case 3:
    case 2:
      val = 1;
      break;
    case 1:
      val = 0;
      break;
    }
  }
  nbits = (val + 1);

  pstr_usac_element_config->tw_mdct = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  /*The variable tw_mdct shall be 0*/
  if (pstr_usac_element_config->tw_mdct != 0)
  {
    return IA_MPEGD_DEC_INIT_FATAL_INVALID_CONFIG_FOR_LC_PROFILE;
  }
  pstr_usac_element_config->full_band_lpd = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  pstr_usac_element_config->noise_filling = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  pstr_usac_element_config->enhanced_noise_filling = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  if (pstr_usac_element_config->enhanced_noise_filling)
  {
    pstr_usac_element_config->str_usac_ele_igf_init_config.igf_use_enf =
        (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
    pstr_usac_element_config->str_usac_ele_igf_init_config.igf_use_high_resolution =
        (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
    pstr_usac_element_config->str_usac_ele_igf_init_config.igf_use_whitening =
        (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
    pstr_usac_element_config->str_usac_ele_igf_init_config.igf_after_tns_synth =
        (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
    pstr_usac_element_config->str_usac_ele_igf_init_config.igf_start_index =
        (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 5);
    pstr_usac_element_config->str_usac_ele_igf_init_config.igf_stop_index =
        (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 4);
    pstr_usac_element_config->str_usac_ele_igf_init_config.igf_independent_tilling =
        (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
  }

  qce_index = ia_core_coder_read_bits_buf(it_bit_buff, 2); // qceindex
  /*The variable qceIndex shall be 0*/
  if (qce_index != 0)
  {
    return IA_MPEGD_DEC_INIT_FATAL_INVALID_CONFIG_FOR_LC_PROFILE;
  }

  if (qce_index > 0)
  {
    shift_idx0 = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    if (shift_idx0 > 0)
    {
      shift_channel0 = ia_core_coder_read_bits_buf(it_bit_buff, nbits);
    }
  }
  ia_core_coder_map_ele_to_channel(num_output_chns, shift_idx0, shift_channel0, &index);
  /*shiftChannel0 a shifted channel shall not exceed the maximum
      channel index of the signal group it is associated with*/
  if (shift_channel0 > num_output_chns[index])
  {
    return IA_MPEGD_DEC_INIT_FATAL_INVALID_SHIFTED_CH;
  }

  shift_idx1 = ia_core_coder_read_bits_buf(it_bit_buff, 1); // do shift
  if (shift_idx1 > 0)
  {
    shift_channel1 = ia_core_coder_read_bits_buf(it_bit_buff, nbits);
  }
  ia_core_coder_map_ele_to_channel(num_output_chns, shift_idx1, shift_channel1, &index);
  /*shiftChannel1 a shifted channel shall not exceed the maximum
    channel index of the signal group it is associated with*/
  if (shift_channel1 > num_output_chns[index])
  {
    return IA_MPEGD_DEC_INIT_FATAL_INVALID_SHIFTED_CH;
  }
  if (qce_index == 0 && sbr_ratio_index == 0)
    pstr_usac_element_config->lpd_stereo_index = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_decoder_config
 *
 *  \brief Update USAC config structure fro bitstream
 *
 *  \param [in]      it_bit_buff          bit stream buffer
 *  \param [in,out]    pstr_usac_conf        USAC config structure
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_decoder_config(ia_bit_buf_struct *it_bit_buff,
                                                 ia_usac_config_struct *pstr_usac_conf)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  UWORD32 elem_idx = 0, sbr_ratio_index = 0;
  WORD32 index = 0;
  ia_usac_decoder_config_struct *pstr_usac_decoder_config;
  ia_usac_dec_element_config_struct *pstr_usac_element_config;
  pstr_usac_decoder_config = &pstr_usac_conf->str_usac_dec_config;
  memset(pstr_usac_decoder_config->num_output_chns, -1, 16 * sizeof(WORD32));

  ia_core_coder_read_escape_value(it_bit_buff, &(pstr_usac_decoder_config->num_elements), 4, 8,
                                  16);

  pstr_usac_decoder_config->preroll_flag = 0;
  pstr_usac_decoder_config->num_elements += 1;

  if (pstr_usac_decoder_config->num_elements > MAX_ELEMENTS_USAC)
  {
    return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
  }

  pstr_usac_decoder_config->ele_length_present =
      ia_core_coder_read_bits_buf(it_bit_buff, 1); // dummy m_elementLengthPresent

  for (elem_idx = 0; elem_idx < pstr_usac_decoder_config->num_elements; elem_idx++)
  {
    pstr_usac_element_config = &(pstr_usac_decoder_config->str_usac_element_config[elem_idx]);

    pstr_usac_decoder_config->usac_element_type[elem_idx] =
        ia_core_coder_read_bits_buf(it_bit_buff, 2);

    switch (pstr_usac_decoder_config->usac_element_type[elem_idx])
    {
    case ID_USAC_EXT:
    {
      err = ia_core_coder_ext_element_config(
          it_bit_buff, pstr_usac_element_config,
          &pstr_usac_decoder_config->usac_ext_ele_payload_buf[elem_idx][0],
          &pstr_usac_decoder_config->usac_ext_ele_payload_len[elem_idx],
          &(pstr_usac_decoder_config->preroll_flag),
          &pstr_usac_decoder_config->ia_ext_ele_payload_type[elem_idx], pstr_usac_conf, 1024,
          elem_idx);

      if (err != IA_MPEGH_DEC_NO_ERROR)
      {
        return err;
      }

      if (pstr_usac_decoder_config->usac_ext_ele_payload_len[elem_idx] <= 0)
      {
        pstr_usac_decoder_config->usac_ext_ele_payload_present[elem_idx] = 0;
      }
      else
      {
        pstr_usac_decoder_config->usac_ext_ele_payload_present[elem_idx] = 1;
      }
      break;
    }
    case ID_USAC_LFE:
    {
      pstr_usac_element_config->stereo_config_index = 0;
      pstr_usac_element_config->noise_filling = 0;
      pstr_usac_element_config->tw_mdct = 0;
      ia_core_coder_map_ele_to_channel(pstr_usac_decoder_config->num_output_chns, 0, 0, &index);
      break;
    }
    case ID_USAC_CPE:
    {
      err = ia_core_coder_cpe_config(it_bit_buff, pstr_usac_element_config, sbr_ratio_index,
                                     pstr_usac_decoder_config->num_output_chns,
                                     &pstr_usac_conf->signals_3d);
      if (err != IA_MPEGH_DEC_NO_ERROR)
      {
        return err;
      }

      break;
    }

    case ID_USAC_SCE:
    {
      pstr_usac_element_config->tw_mdct = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      /*The variable tw_mdct shall be 0*/
      if (pstr_usac_element_config->tw_mdct != 0)
      {
        return IA_MPEGD_DEC_INIT_FATAL_INVALID_CONFIG_FOR_LC_PROFILE;
      }
      pstr_usac_element_config->full_band_lpd = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      pstr_usac_element_config->noise_filling = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      pstr_usac_element_config->enhanced_noise_filling =
          ia_core_coder_read_bits_buf(it_bit_buff, 1);
      if (pstr_usac_element_config->enhanced_noise_filling)
      {
        pstr_usac_element_config->str_usac_ele_igf_init_config.igf_use_enf =
            (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
        pstr_usac_element_config->str_usac_ele_igf_init_config.igf_use_high_resolution =
            (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
        pstr_usac_element_config->str_usac_ele_igf_init_config.igf_use_whitening =
            (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
        pstr_usac_element_config->str_usac_ele_igf_init_config.igf_after_tns_synth =
            (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
        pstr_usac_element_config->str_usac_ele_igf_init_config.igf_start_index =
            (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 5);
        pstr_usac_element_config->str_usac_ele_igf_init_config.igf_stop_index =
            (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 4);
        pstr_usac_element_config->str_usac_ele_igf_init_config.igf_independent_tilling = 1;
      }
      pstr_usac_element_config->stereo_config_index = 0;
      ia_core_coder_map_ele_to_channel(pstr_usac_decoder_config->num_output_chns, 0, 0, &index);

      break;
    }
    default:
      return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_ELEM_INDEX;
      break;
    }
  }
  return err;
}

/**
 *  impeghd_sig_group_info
 *
 *  \brief Group signal informations
 *
 *  \param [in]    it_bit_buff      bit stream buffer
 *  \param [out]  ptr_signal_3d    Parse 3DA parameters from bitstream and update
 * USAC
 * 3DA signal configuration
 *
 *
 *
 */
static VOID impeghd_sig_group_info(ia_bit_buf_struct *it_bit_buff, ia_signals_3d *ptr_signal_3d)
{
  WORD32 grp;

  for (grp = 0; grp <= (WORD32)ptr_signal_3d->num_sig_group; grp++)
  {
    ptr_signal_3d->group_priority[grp] = ia_core_coder_read_bits_buf(it_bit_buff, 3);
    ptr_signal_3d->fixed_position[grp] = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  }
}

/**
 *  impeghd_parse_dmx_ext_config
 *
 *  \brief Parse downmix extension parameters from bitstream and update USAC extension payload
 * downmix cfg structure
 *
 *  \param [in]    it_bit_buff      bit stream buffer
 *  \param [in]    ui_cicp_layout_idx  cicp layout index to decide speaker layout
 *  \param [in,out]  ptr_dmx_cfg      USAC extension payload downmix cfg
 * structure
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE impeghd_parse_dmx_ext_config(ia_bit_buf_struct *it_bit_buff,
                                                 WORD32 ui_cicp_layout_idx,
                                                 ia_usac_ext_cfg_dmx_cfg *ptr_dmx_cfg)
{
  WORD32 i = 0, l, m, byte_count, dmx_matrix_len_bytes, dmx_matrix_bits_left;
  UWORD32 temp;
  ptr_dmx_cfg->dmx_config_type = ia_core_coder_read_bits_buf(it_bit_buff, 2);

  /*downmixConfigType shall not be 3*/
  if (ptr_dmx_cfg->dmx_config_type == 3)
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_DMX_CONF_TYPE;
  }

  if (ptr_dmx_cfg->dmx_config_type == 0 || ptr_dmx_cfg->dmx_config_type == 2)
  {
    ptr_dmx_cfg->passive_dmx_flag = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    if (ptr_dmx_cfg->passive_dmx_flag == 0)
    {
      ptr_dmx_cfg->phase_align_strength = ia_core_coder_read_bits_buf(it_bit_buff, 3);
      /*phaseAlignStrength shall be 0*/
      if (ptr_dmx_cfg->phase_align_strength)
      {
        return IA_MPEGD_DEC_INIT_FATAL_INVALID_CONFIG_FOR_LC_PROFILE;
      }
    }
    ptr_dmx_cfg->immersive_downmix_flag = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  }

  if (ptr_dmx_cfg->dmx_config_type == 1 || ptr_dmx_cfg->dmx_config_type == 2)
  {
    /* DownmixMatrixConfig() */
    ptr_dmx_cfg->downmix_id_count = ia_core_coder_read_bits_buf(it_bit_buff, 5);
    if (ptr_dmx_cfg->downmix_id_count > ASCPARSER_MAX_DMX_MATRIX_ELEMENTS)
    {
      return IA_MPEGH_DEC_EXE_FATAL_MAX_DMX_MATRIX_ELEMENTS_EXCEEDED;
    }

    for (i = 0; i < (WORD32)ptr_dmx_cfg->downmix_id_count; ++i)
    {
      ptr_dmx_cfg->dmx_matrix[i].dmx_id = (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 7);
      ptr_dmx_cfg->dmx_matrix[i].dmx_type = ia_core_coder_read_bits_buf(it_bit_buff, 2);

      if ((ptr_dmx_cfg->dmx_matrix[i].dmx_id == 0) || (ptr_dmx_cfg->dmx_matrix[i].dmx_id == 0x7F))
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_DMX_ID;
      }

      if (ptr_dmx_cfg->dmx_matrix[i].dmx_type == 1)
      {
        ptr_dmx_cfg->dmx_matrix[i].cicp_spk_layout_idx =
            ia_core_coder_read_bits_buf(it_bit_buff, 6);
        ia_core_coder_read_escape_value(it_bit_buff, &temp, 1, 3, 0);
        ptr_dmx_cfg->dmx_matrix[i].downmix_mtx_count = temp + 1;
        if (ptr_dmx_cfg->dmx_matrix[i].downmix_mtx_count > ASCPARSER_MAX_DMX_MATRICES_PER_ID)
        {
          return IA_MPEGH_DEC_EXE_FATAL_ASCPARSER_MAX_DMX_MATRICES_PER_ID_EXCEEDED;
        }
        for (l = 0; l < ptr_dmx_cfg->dmx_matrix[i].downmix_mtx_count; l++)
        {
          ia_core_coder_read_escape_value(it_bit_buff, &temp, 1, 4, 4);

          if (temp >= ASCPARSER_MAX_DMX_MAX_GROUPS_ASSIGNED)
          {
            return IA_MPEGH_DEC_EXE_FATAL_MAX_ASSIGNED_GRP_IDS_EXCEEDED;
          }
          ptr_dmx_cfg->dmx_matrix[i].num_assigned_group_ids[l] = temp + 1;

          for (m = 0; m < ptr_dmx_cfg->dmx_matrix[i].num_assigned_group_ids[l]; m++)
          {
            ptr_dmx_cfg->dmx_matrix[i].signal_group_id[l][m] =
                ia_core_coder_read_bits_buf(it_bit_buff, 5);
          }
          ia_core_coder_read_escape_value(it_bit_buff, &temp, 8, 8, 12);
          ptr_dmx_cfg->dmx_matrix[i].dmx_matrix_len_bits[l] = temp;

          byte_count = 0;
          dmx_matrix_bits_left = ptr_dmx_cfg->dmx_matrix[i].dmx_matrix_len_bits[l] % 8;
          dmx_matrix_len_bytes = ptr_dmx_cfg->dmx_matrix[i].dmx_matrix_len_bits[l] / 8;
          while (dmx_matrix_len_bytes--)
          {
            ptr_dmx_cfg->dmx_matrix[i].downmix_matrix[l][byte_count++] =
                (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 8);
          }
          if (dmx_matrix_bits_left)
          {
            ptr_dmx_cfg->dmx_matrix[i].downmix_matrix[l][byte_count] =
                (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, dmx_matrix_bits_left)
                << (8 - dmx_matrix_bits_left);
          }
        }
      }
      else if (ptr_dmx_cfg->dmx_matrix[i].dmx_type == 0)
      {
        ptr_dmx_cfg->dmx_matrix[i].cicp_spk_layout_idx =
            ia_core_coder_read_bits_buf(it_bit_buff, 6);
      }
      else
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_DMX_TYPE;
      }

      /* ISO/IEC 23091-3 section 6.2 (21-63 reserved)*/
      if (ptr_dmx_cfg->dmx_matrix[i].cicp_spk_layout_idx > MAX_CICP_INDEX)
      {
        return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_LAYOUT_INDEX;
      }

      if (ui_cicp_layout_idx == ptr_dmx_cfg->dmx_matrix[i].cicp_spk_layout_idx)
      {
        ptr_dmx_cfg->dmx_id = ptr_dmx_cfg->dmx_matrix[i].dmx_id;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_config_extension
 *
 *  \brief Configure extension element in USAC config structure
 *
 *  \param [in]      it_bit_buff          bit stream buffer
 *  \param [in,out]    pstr_usac_decoder_config  USAC decoder config
 *  \param [in]      ptr_signal_3d        Signal 3d structure in usac
 *  \param [in]      pstr_mae_asi        meta audio element audio scene info
 *  \param [in]      ui_cicp_layout_idx      cicp layout index
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_config_extension(
    ia_bit_buf_struct *it_bit_buff, ia_usac_decoder_config_struct *pstr_usac_decoder_config,
    ia_signals_3d *ptr_signal_3d, VOID *pstr_mae_asi, WORD32 ui_cicp_layout_idx)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  UWORD8 byte_val;
  UWORD32 i, j, num_config_extensions, usac_config_ext_type, usac_config_ext_len, fill_byte_val;
  WORD32 cnt_bits, skip_bits;
  ia_mae_audio_scene_info *ptr_mae_asi = (ia_mae_audio_scene_info *)pstr_mae_asi;

  ia_core_coder_read_escape_value(it_bit_buff, &(num_config_extensions), 2, 4, 8);
  num_config_extensions += 1;
  if (MAX_CONFIG_EXTENSIONS < num_config_extensions)
  {
    return IA_MPEGH_DEC_INIT_FATAL_MAX_CONF_EXT_EXCEEDED;
  }

  pstr_usac_decoder_config->downmix_ext_config_present = 0;
  memset(pstr_usac_decoder_config->usac_cfg_ext_info_present, 0,
         MAX_CONFIG_EXTENSIONS * sizeof(WORD32));
  memset(pstr_usac_decoder_config->usac_cfg_ext_info_len, 0,
         MAX_CONFIG_EXTENSIONS * sizeof(WORD32));
  pstr_usac_decoder_config->num_config_extensions = num_config_extensions;

  for (j = 0; j < num_config_extensions; j++)
  {
    ia_core_coder_read_escape_value(it_bit_buff, &(usac_config_ext_type), 4, 8, 16);
    ia_core_coder_read_escape_value(it_bit_buff, &(usac_config_ext_len), 4, 8, 16);

    pstr_usac_decoder_config->usac_cfg_ext_info_type[j] = usac_config_ext_type;

    if (usac_config_ext_len > MAX_EXT_ELE_PAYLOAD)
      return IA_MPEGH_DEC_INIT_FATAL_CONFIG_EXT_LEN_EXCEEDED;

    switch (usac_config_ext_type)
    {
    case ID_CONFIG_EXT_HOA_MATRIX:
    {
      pstr_usac_decoder_config->str_hoa_config.matrix_spk_id = pstr_usac_decoder_config->cicp_idx;
      pstr_usac_decoder_config->hoa_matrix_ext_config_present = 1;
      cnt_bits = it_bit_buff->cnt_bits;

      if (pstr_usac_decoder_config->cicp_idx > CICP2GEOMETRY_MAX_SPKIDX)
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;

      err = impeghd_hoa_matrix(it_bit_buff, pstr_usac_decoder_config->str_hoa_config.num_coeffs,
                               pstr_usac_decoder_config->str_hoa_config.matrix_spk_id,
                               &pstr_usac_decoder_config->str_hoa_config.matrix_present,
                               &pstr_usac_decoder_config->str_hoa_config.matrix_index,
                               (FLOAT32 *)&(pstr_usac_decoder_config->str_hoa_config.matrix),
                               &pstr_usac_decoder_config->str_hoa_config.str_hoa_matrix);

      if (err)
        return err;
      pstr_usac_decoder_config->usac_cfg_ext_info_present[j] = 1;
      pstr_usac_decoder_config->usac_cfg_ext_info_len[j] = usac_config_ext_len;
      cnt_bits = cnt_bits - it_bit_buff->cnt_bits;
      if (cnt_bits < (WORD32)(usac_config_ext_len << 3))
      {
        skip_bits = (usac_config_ext_len << 3) - cnt_bits;
        ia_core_coder_skip_bits_buf(it_bit_buff, skip_bits);
      }
      break;
    }
    case ID_CONFIG_EXT_DOWNMIX:
    {
      memset(&pstr_usac_decoder_config->dmx_cfg, 0, sizeof(pstr_usac_decoder_config->dmx_cfg));
      pstr_usac_decoder_config->downmix_ext_config_present = 1;
      pstr_usac_decoder_config->downmix_ext_config_idx = j;
      cnt_bits = it_bit_buff->cnt_bits;

      IA_ERRORCODE err_code = impeghd_parse_dmx_ext_config(it_bit_buff, ui_cicp_layout_idx,
                                                           &pstr_usac_decoder_config->dmx_cfg);
      if (IA_MPEGH_DEC_NO_ERROR != err_code)
      {
        return err_code;
      }
      pstr_usac_decoder_config->usac_cfg_ext_info_present[j] = 1;
      pstr_usac_decoder_config->usac_cfg_ext_info_len[j] = usac_config_ext_len;
      cnt_bits = cnt_bits - it_bit_buff->cnt_bits;
      if (cnt_bits < (WORD32)(usac_config_ext_len << 3))
      {
        skip_bits = (usac_config_ext_len << 3) - cnt_bits;
        ia_core_coder_skip_bits_buf(it_bit_buff, skip_bits);
      }
      break;
    }
    case ID_CONFIG_EXT_LOUDNESS_INFO:
    {
      pstr_usac_decoder_config->loudness_ext_config_present = 1;
      pstr_usac_decoder_config->loudness_ext_config_idx = j;
      pstr_usac_decoder_config->usac_cfg_ext_info_present[j] = 1;
      pstr_usac_decoder_config->usac_cfg_ext_info_len[j] = usac_config_ext_len;
      for (i = 0; i < usac_config_ext_len; i++)
      {
        byte_val = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 8);
        pstr_usac_decoder_config->usac_cfg_ext_info_buf[j][i] = byte_val;
      }
      break;
    }
    case ID_CONFIG_EXT_AUDIOSCENE_INFO:
    {
      /*The variable elementLengthPresent shall be 1, if the Configuration Extension type
      ID_CONFIG_EXT_AUDIOSCENE_INFO exists and the value mae_numSwitchGroups in bitstream
      structure mae_AudioSceneInfo() is larger than 0.*/
      if ((ptr_mae_asi->num_switch_groups > 0) &&
          (pstr_usac_decoder_config->ele_length_present != 1))
      {
        return IA_MPEGH_DEC_INIT_FATAL_ERROR;
      }

      cnt_bits = it_bit_buff->cnt_bits;
      ptr_mae_asi->asi_present = 1;
      impeghd_mae_asi_parse(ptr_mae_asi, it_bit_buff);
      cnt_bits = cnt_bits - it_bit_buff->cnt_bits;
      if (cnt_bits < (WORD32)(usac_config_ext_len << 3))
      {
        skip_bits = (usac_config_ext_len << 3) - cnt_bits;
        ia_core_coder_skip_bits_buf(it_bit_buff, skip_bits);
      }
      break;
    }
    case ID_CONFIG_EXT_SIG_GROUP_INFO:
    {
      impeghd_sig_group_info(it_bit_buff, ptr_signal_3d);
      ptr_signal_3d->signal_grp_info_present = 1;
      break;
    }
    case ID_CONFIG_EXT_FILL:
    {
      for (i = 0; i < usac_config_ext_len; i++)
      {
        fill_byte_val = ia_core_coder_read_bits_buf(it_bit_buff, 8);
        if (fill_byte_val != CONFIG_EXT_FILL_BYTE)
          return IA_MPEGH_DEC_INIT_FATAL_INVALID_FILLBYTE;
      }
      break;
    }
    default:
    {
      if ((WORD32)usac_config_ext_len > (it_bit_buff->cnt_bits >> 3))
      {
        return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
      }
      else
      {
        for (i = 0; i < usac_config_ext_len; i++)
        {
          ia_core_coder_skip_bits_buf(it_bit_buff, 8);
        }
      }
      break;
    }
    }
  }
  return err;
}

/**
*  impeghd_spk_description
*
*  \brief Update 3d speaker config data from bitstream
*
*  \param [in]  buf_handle        Buffer handle
*  \param [in]  pstr_spk_desc     Pointer to flexible speaker config structure
*  \param [in]  angular_precision Angular precision
*
*  \return IA_ERRORCODE              Error code
*
*/
static IA_ERRORCODE impeghd_spk_description(VOID *buf_handle, ia_flex_spk_cfg_str *pstr_spk_desc,
                                            WORD32 angular_precision)
{
  WORD32 az_ang_idx_to_deg;
  ia_bit_buf_struct *ia_bit_buf = (ia_bit_buf_struct *)buf_handle;
  pstr_spk_desc->is_cicp_spk_idx = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 1);

  if (!pstr_spk_desc->is_cicp_spk_idx)
  {
    pstr_spk_desc->el_class = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 2);
    if (pstr_spk_desc->el_class == 3)
    {
      if (angular_precision)
      {
        pstr_spk_desc->el_angle_idx = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 7);
        /*if angularPrecision == 1, shall have a value between and including 0 and 90*/
        if (pstr_spk_desc->el_angle_idx > 90)
        {
          return IA_MPEGH_DEC_INIT_FATAL_INVALID_EL_ANG_IDX;
        }
      }
      else
      {
        pstr_spk_desc->el_angle_idx = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 5);
        /*if angularPrecision == 0, shall have a value between and including 0 and 18*/
        if (pstr_spk_desc->el_angle_idx > 18)
        {
          return IA_MPEGH_DEC_INIT_FATAL_INVALID_EL_ANG_IDX;
        }
      }

      if (impeghd_elev_idx_degree(pstr_spk_desc->el_angle_idx, pstr_spk_desc->el_direction,
                                  angular_precision))
      {
        pstr_spk_desc->el_direction = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 1);
        if (pstr_spk_desc->el_direction)
          pstr_spk_desc->el_angle_idx = -pstr_spk_desc->el_angle_idx;
      }
    }

    if (angular_precision)
    {
      pstr_spk_desc->az_angle_idx = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 8);
      /*if angularPrecision == 1, shall have a value between and including 0 and 180*/
      if (pstr_spk_desc->az_angle_idx > 180)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_AZ_ANG_IDX;
      }
    }
    else
    {
      pstr_spk_desc->az_angle_idx = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 6);
      /*if angularPrecision == 0, shall have a value between and including 0 and 36*/
      if (pstr_spk_desc->az_angle_idx > 36)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_AZ_ANG_IDX;
      }
    }

    az_ang_idx_to_deg = impeghd_azi_idx_degree(pstr_spk_desc->az_angle_idx,
                                               pstr_spk_desc->az_direction, angular_precision);
    if ((az_ang_idx_to_deg != 0) && (az_ang_idx_to_deg != 180))
    {
      pstr_spk_desc->az_direction = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 1);
    }
    pstr_spk_desc->is_lfe = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 1);
  }
  else
  {
    pstr_spk_desc->cicp_spk_idx = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 7);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_flexispk_config
*
*  \brief Update 3d speaker config data from bitstream
*
*  \param [in]  buf_handle          Buffer handle
*  \param [in]  pstr_flex_spk_data  Pointer to flexible speaker data structure
*  \param [in]  num_spk             Number of speakers
*
*  \return IA_ERRORCODE              Error code
*
*/
static IA_ERRORCODE impeghd_flexispk_config(VOID *buf_handle,
                                            ia_flex_spk_data_str *pstr_flex_spk_data,
                                            WORD32 num_spk)
{
  WORD32 i = 0, in_num_channels = 0, in_num_lfes = 0, azimuth;
  ia_bit_buf_struct *ia_bit_buf = (ia_bit_buf_struct *)buf_handle;
  pstr_flex_spk_data->angular_precision = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 1);

  while (i < num_spk)
  {
    azimuth = 0;
    impeghd_spk_description(buf_handle, &(pstr_flex_spk_data->str_flex_spk_descr[i]),
                            pstr_flex_spk_data->angular_precision);

    if (!(pstr_flex_spk_data->str_flex_spk_descr[i].is_cicp_spk_idx))
    {
      azimuth = impeghd_azi_idx_degree(pstr_flex_spk_data->str_flex_spk_descr[i].az_angle_idx,
                                       pstr_flex_spk_data->str_flex_spk_descr[i].az_direction,
                                       pstr_flex_spk_data->angular_precision);
      pstr_flex_spk_data->str_flex_spk_descr[i].az_angle_idx = azimuth;
    }
    else if ((pstr_flex_spk_data->str_flex_spk_descr[i].cicp_spk_idx < NUM_LS_CFGS) &&
             (pstr_flex_spk_data->str_flex_spk_descr[i].cicp_spk_idx != 0) &&
             (pstr_flex_spk_data->str_flex_spk_descr[i].cicp_spk_idx != 8))
    {
      const ia_cicp_ls_geo_str *ptr_cicp_ls_geometry[CICP_MAX_NUM_LS];

      ptr_cicp_ls_geometry[i] =
          &ia_cicp_ls_geo_tbls[pstr_flex_spk_data->str_flex_spk_descr[i].cicp_spk_idx];
      if (!ptr_cicp_ls_geometry[i]->lfe_flag)
      {
        in_num_channels++;
      }
      else
      {
        in_num_lfes++;
      }
      azimuth = (WORD32)(ptr_cicp_ls_geometry[i]->ls_azimuth);
    }

    if ((azimuth != 0) && (azimuth != 180))
    {
      pstr_flex_spk_data->also_add_symetric_pair = ia_core_coder_read_bits_buf(ia_bit_buf, 1);
      if (pstr_flex_spk_data->also_add_symetric_pair)
      {
        i++;
        pstr_flex_spk_data->str_flex_spk_descr[i] = pstr_flex_spk_data->str_flex_spk_descr[i - 1];
        pstr_flex_spk_data->str_flex_spk_descr[i].az_direction =
            1 - pstr_flex_spk_data->str_flex_spk_descr[i].az_direction;
      }
    }
    i++;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_mpegh_3da_config
 *
 *  \brief Configure audio specif config based on MPEGH 3D audio bit stream parameters
 *
 *  \param [in]    it_bit_buff      bit stream buffer
 *  \param [in,out]  mpeghd_state_struct  decoder state struct
 *  \param [in]    ptr_mae_asi      meta audio element audio scene info
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_mpegh_3da_config(ia_bit_buf_struct *it_bit_buff,
                                            ia_mpegh_dec_state_struct *mpeghd_state_struct,
                                            VOID *ptr_mae_asi)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  UWORD32 k, mpegh_profile_lvl;
  WORD32 i, tmp = 0, num_ch_based_grps = 0, num_sig, num_obj_based_grps = 0,
            num_hoa_based_grps = 0, audio_ch_layout_cntr = 0;
  WORD32 ui_cicp_layout_idx;
  WORD32 dec_proc_core_chans = 0;
  WORD32 ref_layout_chans = 0;
  ia_audio_specific_config_struct *pstr_audio_specific_config =
      mpeghd_state_struct->ia_audio_specific_config;
  ia_usac_config_struct *pstr_usac_conf = &(pstr_audio_specific_config->str_usac_config);
  ia_signals_3d *ia_signals_3da = &pstr_usac_conf->signals_3d;

  mpeghd_state_struct->is_base_line_profile_3b = 0;
  mpegh_profile_lvl =
      ia_core_coder_read_bits_buf(it_bit_buff, 8); // mpegh_3da_profile_lvl_indication
  pstr_usac_conf->str_usac_dec_config.mpegh_profile_lvl = mpegh_profile_lvl;

  pstr_usac_conf->usac_sampling_frequency_index = ia_core_coder_read_bits_buf(it_bit_buff, 5);

  if (0x1f != pstr_usac_conf->usac_sampling_frequency_index)
  {
    if (pstr_usac_conf->usac_sampling_frequency_index > 0x08)
      return IA_MPEGH_DEC_INIT_FATAL_SAMP_FREQ_NOT_SUPPORTED;

    pstr_usac_conf->usac_sampling_frequency =
        ia_sampling_rate_tbl[pstr_usac_conf->usac_sampling_frequency_index];
  }
  else
  {
    pstr_usac_conf->usac_sampling_frequency = ia_core_coder_read_bits_buf(it_bit_buff, 24);
  }

  if ((pstr_usac_conf->usac_sampling_frequency > MAX_SAMPLE_RATE) ||
      (pstr_usac_conf->usac_sampling_frequency == 0))
  {
    return IA_MPEGH_DEC_INIT_FATAL_SAMP_FREQ_NOT_SUPPORTED;
  }

  mpeghd_state_struct->p_config->ui_samp_freq = pstr_usac_conf->usac_sampling_frequency;
  pstr_audio_specific_config->sampling_frequency = pstr_usac_conf->usac_sampling_frequency;

  for (i = 0; i < (WORD32)sizeof(ia_core_coder_sample_freq_idx_table) /
                      (WORD32)sizeof(ia_core_coder_sample_freq_idx_table[0]);
       i++)
  {
    if (ia_core_coder_sample_freq_idx_table[i] ==
        (WORD32)(pstr_audio_specific_config->sampling_frequency))
    {
      tmp = i;
      break;
    }
  }
  pstr_audio_specific_config->samp_frequency_index = (UINT32)tmp;

  pstr_usac_conf->core_sbr_framelength_index = ia_core_coder_read_bits_buf(it_bit_buff, 3);

  if (pstr_usac_conf->core_sbr_framelength_index > MAX_CORE_SBR_FRAME_LEN_IDX)
    return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CORECODER_FRAMELENGTH_IDX;

  ia_core_coder_skip_bits_buf(it_bit_buff, 1); // dummy read cfg_reserved
  pstr_usac_conf->receiver_delay_compensation = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  pstr_audio_specific_config->ref_spk_layout.spk_layout_type =
      ia_core_coder_read_bits_buf(it_bit_buff, 2); // speaker3D config

  if (0 != pstr_audio_specific_config->ref_spk_layout.spk_layout_type)
  {
    ia_core_coder_read_escape_value(
        it_bit_buff, (UWORD32 *)&pstr_audio_specific_config->ref_spk_layout.num_speakers, 5, 8,
        16);
    pstr_audio_specific_config->ref_spk_layout.num_speakers++;
    if (pstr_audio_specific_config->ref_spk_layout.num_speakers > MAX_NUM_SPEAKERS)
    {
      pstr_audio_specific_config->ref_spk_layout.num_speakers = 0;
      return IA_MPEGH_DEC_INIT_FATAL_MAX_NUM_SPKS_EXCEEDED;
    }
    if (pstr_audio_specific_config->ref_spk_layout.spk_layout_type == 2)
    {
      impeghd_flexispk_config(it_bit_buff,
                              &pstr_audio_specific_config->ref_spk_layout.str_flex_spk,
                              pstr_audio_specific_config->ref_spk_layout.num_speakers);
      pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx =
          pstr_audio_specific_config->ref_spk_layout.num_speakers;
      if (pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx <= 0 ||
          pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx > MAX_CICP_INDEX)
      {
        pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx =
            IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL;
        return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_LAYOUT_INDEX;
      }
    }
    else if (pstr_audio_specific_config->ref_spk_layout.spk_layout_type == 1)
    {
      for (i = 0; i < pstr_audio_specific_config->ref_spk_layout.num_speakers; i++)
      {
        pstr_audio_specific_config->ref_spk_layout.cicp_spk_idx[i] =
            ia_core_coder_read_bits_buf(it_bit_buff, 7);
        if (pstr_audio_specific_config->ref_spk_layout.cicp_spk_idx[i] > CICP_MAX_CH)
        {
          return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_SPK_INDEX;
        }
      }
    }
  }
  else
  {
    pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx =
        ia_core_coder_read_bits_buf(it_bit_buff, 6);
    if (pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx <= 0 ||
        pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx > MAX_CICP_INDEX)
    {
      pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx =
          IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL;
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_LAYOUT_INDEX;
    }
  }
  /* Signal3d parsing */ // Added for MCT
  ia_signals_3da->num_sig_group = ia_core_coder_read_bits_buf(it_bit_buff, 5) + 1;
  if (ia_signals_3da->num_sig_group > MAX_NUM_SIGNALGROUPS)
  {
    ia_signals_3da->num_sig_group = 1;
    return IA_MPEGH_DEC_INIT_FATAL_MAX_NUM_SIGNALGROUPS_EXCEEDED;
  }
  pstr_usac_conf->num_out_channels = 0;
  ia_signals_3da->num_hoa_transport_ch = 0;
  ia_signals_3da->num_saoc_transport_ch = 0;
  ia_signals_3da->num_audio_obj = 0;
  ia_signals_3da->num_ch = 0;
  audio_ch_layout_cntr = 0;

  /*if(pstr_audio_specific_config->ref_spk_layout.spk_layout_type == 3),num_sig_group=0,
  group_type=0,
  differs_from_ref_layout=0*/
  if ((pstr_audio_specific_config->ref_spk_layout.spk_layout_type == 3) &&
      (ia_signals_3da->num_sig_group != 1))
  {
    return IA_MPEGH_DEC_INIT_FATAL_ERROR;
  }
  for (i = 0; i < (WORD32)ia_signals_3da->num_sig_group; i++)
  {
    ia_signals_3da->group_type[i] =
        ia_core_coder_read_bits_buf(it_bit_buff, 3); // get signal group type
    ia_core_coder_read_escape_value(it_bit_buff, (UWORD32 *)&num_sig, 5, 8, 16); // no. of signals
    ia_signals_3da->num_sig[i] = num_sig + 1;

    switch (ia_signals_3da->group_type[i])
    {
    case 3: /*Signal Group Type HOA */
    {
      pstr_usac_conf->str_usac_dec_config.str_hoa_config.core_coder_frame_length =
          AUDIO_CODEC_FRAME_SIZE_MAX; // Hardcoded in ref code.
      num_hoa_based_grps++;
      ia_signals_3da->num_hoa_transport_ch += (num_sig + 1);
      pstr_usac_conf->str_usac_dec_config.str_hoa_config.num_transport_ch =
          ia_signals_3da->num_hoa_transport_ch;
      break;
    }
    case 2: /*Signal Group Type SAOC */
    {
      if (ia_core_coder_read_bits_buf(it_bit_buff, 1))
        ia_core_coder_skip_bits_buf(it_bit_buff, 2);
      break;
    }
    case 1: /*Signal Group Type Objects */
    {
      ia_signals_3da->num_audio_obj += (num_sig + 1);
      num_obj_based_grps++;
      break;
    }
    case 0:
    {
      num_ch_based_grps++;
      ia_signals_3da->num_ch +=
          num_sig + 1; /* "+=" because there could be more than one instance */
      pstr_usac_conf->num_out_channels += num_sig + 1; // need to check?
      if (ia_signals_3da->num_ch > MAX_NUM_CHANNELS ||
          pstr_usac_conf->num_out_channels > MAX_NUM_OUT_CHANNELS)
      {
        return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
      }
      ia_signals_3da->differs_from_ref_layout[i] = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      if (ia_signals_3da->differs_from_ref_layout[i] == 1)
      {
        // ascparser
        ia_speaker_config_3d *speaker_config_3d =
            &ia_signals_3da->audio_ch_layout[audio_ch_layout_cntr];
        speaker_config_3d->spk_layout_type = ia_core_coder_read_bits_buf(it_bit_buff, 2);
        if (0 != speaker_config_3d->spk_layout_type)
        {
          ia_core_coder_read_escape_value(
              it_bit_buff, (UWORD32 *)&(speaker_config_3d->num_speakers), 5, 8, 16);
          speaker_config_3d->num_speakers++;
          if (speaker_config_3d->num_speakers > MAX_NUM_SPEAKERS)
          {
            return IA_MPEGH_DEC_INIT_FATAL_MAX_NUM_SPKS_EXCEEDED;
          }
          switch (speaker_config_3d->spk_layout_type)
          {
          case 2:
          {
            impeghd_flexispk_config(it_bit_buff, &(speaker_config_3d->str_flex_spk),
                                    speaker_config_3d->num_speakers);
            break;
          }
          case 1:
          {
            for (i = 0; i < speaker_config_3d->num_speakers; i++)
            {
              speaker_config_3d->cicp_spk_idx[i] = ia_core_coder_read_bits_buf(it_bit_buff, 7);
              if (speaker_config_3d->cicp_spk_idx[i] > MAX_CICP_SPK_INDEX)
              {
                return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_SPK_INDEX;
              }
            }
            break;
          }
          }
        }
        else
        {
          speaker_config_3d->cicp_spk_layout_idx = ia_core_coder_read_bits_buf(it_bit_buff, 6);
          if (speaker_config_3d->cicp_spk_layout_idx > MAX_CICP_INDEX)
          {
            return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_LAYOUT_INDEX;
          }
        }
      }
      audio_ch_layout_cntr++;
      break;
    }
    default:
      // Reserved value of Signal group type
      break;
    }
  }

  if (ia_signals_3da->num_sig_group > 1 && pstr_usac_conf->receiver_delay_compensation == 0)
  {
    for (k = 1; k < ia_signals_3da->num_sig_group; k++)
    {
      if (ia_signals_3da->group_type[k - 1] != ia_signals_3da->group_type[k])
        return IA_MPEGH_DEC_INIT_FATAL_RECEIVER_COMP_DELAY;
    }
  }

  ia_signals_3da->num_obj_based_groups = num_obj_based_grps;
  ia_signals_3da->num_hoa_based_groups = num_hoa_based_grps;
  if (num_ch_based_grps < 1 ||
      mpeghd_state_struct->p_config->ui_cicp_layout_idx == IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL)
  {
    ia_signals_3da->format_converter_enable = 0;
  }
  else
  {
    ia_signals_3da->format_converter_enable = 1;
  }
  if (num_hoa_based_grps > 0)
  {
    mpeghd_state_struct->p_config->hoa_data_present = 1;
  }
  if (num_obj_based_grps > 0)
  {
    mpeghd_state_struct->p_config->oam_data_present = 1;
  }
  if (num_ch_based_grps > 0)
  {
    mpeghd_state_struct->p_config->ch_data_present = 1;
  }
  ia_signals_3da->num_ch_based_groups = num_ch_based_grps;

  mpeghd_state_struct->ch_config = pstr_usac_conf->num_out_channels;
  pstr_audio_specific_config->channel_configuration = pstr_usac_conf->num_out_channels;

  dec_proc_core_chans = ia_signals_3da->num_hoa_transport_ch + ia_signals_3da->num_audio_obj +
                        ia_signals_3da->num_ch;
  if (pstr_audio_specific_config->ref_spk_layout.spk_layout_type == 0)
  {
    ref_layout_chans =
        impgehd_cicp_get_num_ls[pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx];
  }
  else
  {
    ref_layout_chans = pstr_audio_specific_config->ref_spk_layout.num_speakers;
  }

  switch (mpegh_profile_lvl)
  {
  case MPEGH_PROFILE_LC_LVL_1:
  case MPEGH_PROFILE_BP_LVL_1:
    if (dec_proc_core_chans > 5 || ref_layout_chans > 5)
    {
      return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
    }
    break;
  case MPEGH_PROFILE_LC_LVL_2:
  case MPEGH_PROFILE_BP_LVL_2:
    if (dec_proc_core_chans > 9 || ref_layout_chans > 9)
    {
      return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
    }
    break;
  case MPEGH_PROFILE_BP_LVL_3:
    if (dec_proc_core_chans > 24 || ref_layout_chans > 24 ||
        (dec_proc_core_chans > 16 && (num_hoa_based_grps != 0 || num_ch_based_grps != 0)) ||
        (ref_layout_chans > 16 && (num_hoa_based_grps != 0 || num_ch_based_grps != 0)))
    {
      return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
    }
    break;
  default:
    if (dec_proc_core_chans > 16 || ref_layout_chans > 16)
    {
      return IA_MPEGH_DEC_INIT_FATAL_STREAM_CHAN_GT_MAX;
    }
    break;
  }

  err = ia_core_coder_decoder_config(it_bit_buff, pstr_usac_conf);
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }

  if (dec_proc_core_chans > 16 || ref_layout_chans > 16)
  {
    ia_usac_decoder_config_struct *pstr_usac_decoder_config =
        &pstr_usac_conf->str_usac_dec_config;
    for (UWORD32 elem_idx = 0; elem_idx < pstr_usac_decoder_config->num_elements; elem_idx++)
    {
      ia_usac_dec_element_config_struct *pstr_usac_element_config =
          &(pstr_usac_decoder_config->str_usac_element_config[elem_idx]);
      switch (pstr_usac_decoder_config->usac_element_type[elem_idx])
      {
      case ID_USAC_SCE:
        if (pstr_usac_element_config->noise_filling != 0 ||
            pstr_usac_element_config->enhanced_noise_filling != 0)
        {
          return IA_MPEGH_DEC_INIT_FATAL_INVALID_PROFILE_CONFIG;
        }
        break;
      case ID_USAC_EXT:
        if (pstr_usac_decoder_config->ia_ext_ele_payload_type[elem_idx] == ID_MPEGH_EXT_ELE_MCT)
        {
          return IA_MPEGH_DEC_INIT_FATAL_INVALID_PROFILE_CONFIG;
        }
        break;
      default:
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_PROFILE_CONFIG;
        break;
      }
    }
    mpeghd_state_struct->is_base_line_profile_3b = 1;
  }

  tmp = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (tmp)
  {
    if (IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL !=
        mpeghd_state_struct->p_config->ui_cicp_layout_idx)
    {
      ui_cicp_layout_idx = mpeghd_state_struct->p_config->ui_cicp_layout_idx;
    }
    else
    {
      ui_cicp_layout_idx = pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx;
    }
    if (ui_cicp_layout_idx <= 0 || ui_cicp_layout_idx > MAX_CICP_INDEX)
    {
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_LAYOUT_INDEX;
    }

    pstr_usac_conf->str_usac_dec_config.cicp_idx = ui_cicp_layout_idx;

    err = ia_core_coder_config_extension(it_bit_buff, &pstr_usac_conf->str_usac_dec_config,
                                         &pstr_usac_conf->signals_3d, ptr_mae_asi,
                                         ui_cicp_layout_idx);
    if (err)
      return err;
    if ((ui_cicp_layout_idx == pstr_audio_specific_config->ref_spk_layout.cicp_spk_layout_idx) ||
        !(pstr_usac_conf->str_usac_dec_config.downmix_ext_config_present) ||
        1 == pstr_usac_conf->str_usac_dec_config.hoa_matrix_ext_config_present)
    {
      pstr_usac_conf->signals_3d.format_converter_enable = 0;
    }
    else
    {
      pstr_usac_conf->signals_3d.format_converter_enable = 1;
    }
    if ((mpeghd_state_struct->ch_config !=
         (UWORD32)(impgehd_cicp_get_num_ls[ui_cicp_layout_idx])) &&
        (pstr_usac_conf->str_usac_dec_config.hoa_matrix_ext_config_present != 1) &&
        (mpeghd_state_struct->ch_config != 0) &&
        (ui_cicp_layout_idx != IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL))
    {
      pstr_usac_conf->signals_3d.format_converter_enable = 1;
    }
  }
  else if (pstr_usac_conf->signals_3d.num_ch_based_groups > 1)
  {
    pstr_usac_conf->signals_3d.format_converter_enable = 1;
  }
  return err;
}

/**
 *  ia_core_coder_conf_default
 *
 *  \brief Configure decoder with default values
 *
 *  \param [in,out] pstr_usac_conf    USAC config structure
 *
 *
 *
 */
VOID ia_core_coder_conf_default(ia_usac_config_struct *pstr_usac_conf)
{
  WORD32 ele;

  for (ele = 0; ele < MAX_ELEMENTS_USAC; ele++)
    pstr_usac_conf->str_usac_dec_config.usac_element_type[ele] = ID_USAC_INVALID;

  pstr_usac_conf->str_usac_dec_config.num_elements = 0;
  pstr_usac_conf->num_out_channels = 0;
}

/**
*  impeghd_speaker_config_3d
*
*  \brief Update 3d speaker config data from bitstream
*
*  \param [in]  buf_handle          Pointer to input configuration
* structure
*  \param [in]  spk_config_3d        Pointer to brir 3d speaker configuretion
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_speaker_config_3d(VOID *buf_handle,
                                       ia_interface_speaker_config_3d *spk_config_3d)
{
  UWORD32 i;
  ia_bit_buf_struct *ia_bit_buf = (ia_bit_buf_struct *)buf_handle;
  spk_config_3d->spk_layout_type = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 2);
  if (spk_config_3d->spk_layout_type == 0)
  {
    spk_config_3d->cicp_spk_layout_idx = (WORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 6);
  }
  else if (spk_config_3d->spk_layout_type < 3)
  {
    ia_core_coder_read_escape_value(ia_bit_buf, &spk_config_3d->num_speakers, 5, 8, 16);
    spk_config_3d->num_speakers += 1;
    if (spk_config_3d->num_speakers > MAX_NUM_SPEAKERS)
    {
      spk_config_3d->num_speakers = 0;
      return IA_MPEGH_DEC_INIT_FATAL_MAX_NUM_SPKS_EXCEEDED;
    }
    spk_config_3d->cicp_spk_layout_idx = (WORD16)-1;
    switch (spk_config_3d->spk_layout_type)
    {
    case 2:
    {
      impeghd_flexispk_config(buf_handle, &(spk_config_3d->str_flex_spk_data),
                              spk_config_3d->num_speakers);
      break;
    }
    case 1:
    {
      for (i = 0; i < spk_config_3d->num_speakers; i++)
      {
        spk_config_3d->cicp_spk_idx[i] = (UWORD16)ia_core_coder_read_bits_buf(ia_bit_buf, 7);
      }
      break;
    }
    }
  }
  impeghd_get_geom_frm_spk_cfg(spk_config_3d, &spk_config_3d->geometry[0],
                               &(spk_config_3d->num_ch), &(spk_config_3d->num_lfes));

  spk_config_3d->num_speakers = spk_config_3d->num_ch + spk_config_3d->num_lfes;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_azi_idx_degree
*
*  \brief Maps azimuth index to degrees based on direction and precision
*
*  \param [in]  idx        azimuth Index
*  \param [in]  direction  azimuth direction
*  \param [in]  precision  Angular precision
*
*  \return WORD32  azimuth angle in degrees
*
*/
WORD32 impeghd_azi_idx_degree(WORD32 idx, WORD32 direction, WORD32 precision)
{
  WORD32 ret_val = 0;

  if (precision)
  {
    ret_val = idx;
  }
  else
  {
    ret_val = idx * 5;
  }

  if (direction == 1)
  {
    ret_val *= -1;
  }

  return ret_val;
}

/**
*  impeghd_elev_idx_degree
*
*  \brief Maps elevation index to degrees based on direction and precision
*
*  \param [in]  idx        azimuth Index
*  \param [in]  direction  azimuth direction
*  \param [in]  precision  Angular precision
*
*  \return WORD32      Elevation in degrees
*
*/
WORD32 impeghd_elev_idx_degree(WORD32 idx, WORD32 direction, WORD32 precision)
{
  WORD32 ret_val = 0;

  if (precision)
  {
    ret_val = idx;
  }
  else
  {
    ret_val = idx * 5;
  }

  if (direction == 1)
  {
    ret_val *= -1;
  }

  return ret_val;
}

/**
*  impeghd_fill_ls_geom_frm_cicp_spk_idx
*
*  \brief Fills loudspeaker geometry information based on CICP index of the loudspeaker.
*
*  \param [in]  cicp_idx     CICP index.
*  \param [out] ptr_geometry Pointer to channel geometery structure.
*  \param [out] num_channels Pointer to data that carries channels information.
*  \param [out] num_lfes     Pointer to data that carries LFE channels information.
*
*  \return IA_ERRORCODE Error code if any else 0.
*
*/
IA_ERRORCODE impeghd_fill_ls_geom_frm_cicp_spk_idx(WORD32 cicp_idx, ia_ch_geometry *ptr_geometry,
                                                   pWORD32 num_channels, pWORD32 num_lfes)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  WORD32 num_speakers_lcl;
  WORD32 num_lfes_lcl;
  const WORD32 *in_channel_names;
  ia_cicp_ls_geo_str *ptr_cicp_ls_geometry[CICP_MAX_NUM_LS];
  err = impeghd_cicpidx_2_ls_geometry(
      cicp_idx, (const ia_cicp_ls_geo_str **)&ptr_cicp_ls_geometry[0], &num_speakers_lcl,
      &num_lfes_lcl, (const WORD32 **)&in_channel_names);
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }
  for (i = 0; i < num_speakers_lcl; i++)
  {
    ptr_geometry[i].cicp_loudspeaker_idx = in_channel_names[i];
    ptr_geometry[i].lfe = ptr_cicp_ls_geometry[i]->lfe_flag;
    ptr_geometry[i].screen_relative = ptr_cicp_ls_geometry[i]->screen_rel_flag;
    ptr_geometry[i].loudspeaker_type = CICP2GEOMETRY_LOUDSPEAKER_KNOWN;
    ptr_geometry[i].el = (WORD32)ptr_cicp_ls_geometry[i]->ls_elevation;
    ptr_geometry[i].az = (WORD32)ptr_cicp_ls_geometry[i]->ls_azimuth;
  }

  *num_lfes = num_lfes_lcl;
  *num_channels = num_speakers_lcl - num_lfes_lcl;
  return err;
}

/**
*  impeghd_get_geom_frm_spk_cfg
*
*  \brief Update binaural renderer 3dspk config data from bitstream
*
*  \param [in]  speaker_config_3d   Pointer to brir 3d speaker configuretion
*  \param [in]  ptr_geometry        Pointer to channel geometry
*  \param [in]  num_channels        Pointer to number of channels
*  \param [in]  num_lfes            Pointer to number of lfes
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_get_geom_frm_spk_cfg(ia_interface_speaker_config_3d *speaker_config_3d,
                                          ia_ch_geometry *ptr_geometry, WORD32 *num_channels,
                                          WORD32 *num_lfes)
{
  UWORD32 i;
  ia_cicp_ls_geo_str *ptr_cicp_ls_geometry[CICP_MAX_NUM_LS];
  WORD32 in_num_channels, in_num_lfes;
  if (speaker_config_3d->spk_layout_type == 0)
  {
    impeghd_fill_ls_geom_frm_cicp_spk_idx(speaker_config_3d->cicp_spk_layout_idx, ptr_geometry,
                                          num_channels, num_lfes);
  }
  else if (speaker_config_3d->spk_layout_type < 3)
  {
    switch (speaker_config_3d->spk_layout_type)
    {
    case 2:
    {
      *num_lfes = 0;
      *num_channels = speaker_config_3d->num_speakers;
      for (i = 0; i < speaker_config_3d->num_speakers; i++)
      {
        if (!(speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].is_cicp_spk_idx))
        {
          ptr_geometry[i].el =
              speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].el_angle_idx;
          ptr_geometry[i].az =
              speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].az_angle_idx;
          ptr_geometry[i].lfe = speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].is_lfe;
          ptr_geometry[i].cicp_loudspeaker_idx = -1;
        }
        else
        {
          const WORD32 **in_channel_names;
          impeghd_cicpidx_2_ls_geometry(
              speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].cicp_spk_idx,
              (const ia_cicp_ls_geo_str **)&ptr_cicp_ls_geometry[i], &in_num_channels,
              &in_num_lfes, (const WORD32 **)&in_channel_names);
          ptr_geometry[i].cicp_loudspeaker_idx =
              speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].cicp_spk_idx;
          ptr_geometry[i].lfe = ptr_cicp_ls_geometry[i]->lfe_flag;
          ptr_geometry[i].screen_relative = ptr_cicp_ls_geometry[i]->screen_rel_flag;
          ptr_geometry[i].loudspeaker_type = CICP2GEOMETRY_LOUDSPEAKER_KNOWN;
          ptr_geometry[i].el = (WORD32)ptr_cicp_ls_geometry[i]->ls_elevation;
          ptr_geometry[i].az = (WORD32)ptr_cicp_ls_geometry[i]->ls_azimuth;
        }
        if (speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].is_lfe != 0)
        {
          (*num_lfes)++;
          (*num_channels)--;
        }
      }
      break;
    }
    case 1:
    {
      *num_lfes = 0;
      *num_channels = speaker_config_3d->num_speakers;
      for (i = 0; i < speaker_config_3d->num_speakers; i++)
      {
        const WORD32 **in_channel_names;
        impeghd_cicpidx_2_ls_geometry(speaker_config_3d->cicp_spk_idx[i],
                                      (const ia_cicp_ls_geo_str **)&ptr_cicp_ls_geometry[i],
                                      &in_num_channels, &in_num_lfes,
                                      (const WORD32 **)&in_channel_names);
        ptr_geometry[i].cicp_loudspeaker_idx =
            speaker_config_3d->str_flex_spk_data.str_flex_spk_descr[i].cicp_spk_idx;
        ptr_geometry[i].lfe = ptr_cicp_ls_geometry[i]->lfe_flag;
        ptr_geometry[i].screen_relative = ptr_cicp_ls_geometry[i]->screen_rel_flag;
        ptr_geometry[i].loudspeaker_type = CICP2GEOMETRY_LOUDSPEAKER_KNOWN;
        ptr_geometry[i].el = (WORD32)ptr_cicp_ls_geometry[i]->ls_elevation;
        ptr_geometry[i].az = (WORD32)ptr_cicp_ls_geometry[i]->ls_azimuth;
        if ((speaker_config_3d->cicp_spk_idx[i] == 3) ||
            (speaker_config_3d->cicp_spk_idx[i] == 26))
        {
          (*num_lfes)++;
          (*num_channels)--;
        }
      }
      break;
    }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecInit */