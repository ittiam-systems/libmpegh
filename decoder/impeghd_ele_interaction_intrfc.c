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

#include "math.h"
#include <stdio.h>
#include <stdlib.h>

#include <impeghd_type_def.h>
#include "ia_core_coder_bitbuffer.h"
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_oam_dec.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_obj_ren_dec.h"

/**
 * @defgroup EleIntrIfc Interaction Interfaces Payload Parsing
 * @ingroup  EleIntrIfc
 * @brief Interaction Interfaces Payload Parsing
 *
 * @{
 */

/*
mpegh3dapstr_local_setup_dataInformation
        LoudspeakerRendering
                SpeakerConfig3d
                        mpegh3daFlexibleSpeakerConfig
        BinauralRendering
                SpeakerConfig3d
                        mpegh3daFlexibleSpeakerConfig
                BinauralFIRData
                FdBinauralRendererParam
                        VoffBrirParam
                        SfrBrirParam
                        QtdlBrirParam
                TdBinauralRendererParam
        LocalScreenSizeInformation
*/

/**
*  impeghd_loud_spk_ren
*
*  \brief Update speaker renderer info
*
*  \param [in]  ptr_bit_buf				Pointer to bit buffer
*
*  \param [in]  pstr_local_setup_data   Pointer to local setup data struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_loud_spk_ren(ia_bit_buf_struct *ptr_bit_buf,
                                         ia_local_setup_struct *pstr_local_setup_data)
{
  UWORD32 ld_spk;
  UWORD32 num_loudspk = 0, has_loud_spk_dist = 0, has_loud_spk_calibration_gain = 0;
  num_loudspk = ia_core_coder_read_bits_buf(ptr_bit_buf, 16);
  impeghd_speaker_config_3d(ptr_bit_buf, &pstr_local_setup_data->spk_config);
  has_loud_spk_dist = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  has_loud_spk_calibration_gain = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  pstr_local_setup_data->use_tracking_mode = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  pstr_local_setup_data->ptr_spk_dist = NULL;
  pstr_local_setup_data->ptr_spk_gain = NULL;
  if (has_loud_spk_dist)
  {
    pstr_local_setup_data->ptr_spk_dist = &pstr_local_setup_data->loud_spk_dist[0];
  }
  if (has_loud_spk_calibration_gain)
  {
    pstr_local_setup_data->ptr_spk_gain = &pstr_local_setup_data->loud_spk_calibration_gain[0];
  }
  for (ld_spk = 0; ld_spk < num_loudspk; ld_spk++)
  {
    if (pstr_local_setup_data->spk_config.spk_layout_type <= 1)
    {
      pstr_local_setup_data->has_knwn_pos[ld_spk] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      if (pstr_local_setup_data->has_knwn_pos[ld_spk])
      {
        pstr_local_setup_data->loud_spk_azimuth[ld_spk] =
            (FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 9);
        pstr_local_setup_data->loud_spk_elevation[ld_spk] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
      }
    }
    if (has_loud_spk_dist)
    {
      pstr_local_setup_data->loud_spk_dist[ld_spk] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

      /*loudspeakerDistance shall not be 0*/
      if (pstr_local_setup_data->loud_spk_dist[ld_spk] == 0)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_SPK_DIST;
      }
    }
    if (has_loud_spk_calibration_gain)
    {
      pstr_local_setup_data->loud_spk_calibration_gain[ld_spk] =
          (FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    }
  }
  pstr_local_setup_data->ext_dist_compensation = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  pstr_local_setup_data->num_spk = num_loudspk;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_loud_spk_inv_quant
*
*  \brief Loud speaker inverse quant
*
*  \param [in]  pstr_local_setup_data   Pointer to local setup data struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_loud_spk_inv_quant(ia_local_setup_struct *pstr_local_setup_data)
{
  UWORD32 ld_spk;
  UWORD32 num_loudspk = 0;
  FLOAT32 temp;
  num_loudspk = pstr_local_setup_data->num_spk;
  if (pstr_local_setup_data->ptr_spk_gain != NULL)
  {
    for (ld_spk = 0; ld_spk < num_loudspk; ld_spk++)
    {
      temp = pstr_local_setup_data->loud_spk_calibration_gain[ld_spk];
      temp = (temp - 64) * 0.5f;
      temp = ia_max_flt(ia_min_flt(temp, 31.5f), -32.0f);
      pstr_local_setup_data->loud_spk_calibration_gain[ld_spk] = temp;
    }
  }
  if (pstr_local_setup_data->spk_config.spk_layout_type <= 1)
  {
    for (ld_spk = 0; ld_spk < num_loudspk; ld_spk++)
    {
      if (pstr_local_setup_data->has_knwn_pos[ld_spk] == 1)
      {
        temp = pstr_local_setup_data->loud_spk_azimuth[ld_spk];
        temp = (FLOAT32)(temp - 256);
        temp = ia_max_flt(ia_min_flt(temp, 180.0f), -180.0f);
        pstr_local_setup_data->loud_spk_azimuth[ld_spk] = temp;
      }
    }
  }
  if (pstr_local_setup_data->spk_config.spk_layout_type <= 1)
  {
    for (ld_spk = 0; ld_spk < num_loudspk; ld_spk++)
    {
      if (pstr_local_setup_data->has_knwn_pos[ld_spk] == 1)
      {
        temp = (FLOAT32)pstr_local_setup_data->loud_spk_elevation[ld_spk];
        temp = (FLOAT32)(temp - 128);
        temp = ia_max_flt(ia_min_flt(temp, 90.0f), -90.0f);
        pstr_local_setup_data->loud_spk_elevation[ld_spk] = (WORD32)temp;
      }
    }
  }
  if (pstr_local_setup_data->spk_config.spk_layout_type == 2)
  {
    for (ld_spk = 0; ld_spk < num_loudspk; ld_spk++)
    {
      if (pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
              .az_direction == 1)
      {
        pstr_local_setup_data->spk_config.str_flex_spk_data.azimuth[ld_spk] = (FLOAT32)(
            pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
                .az_angle_idx *
            pstr_local_setup_data->spk_config.str_flex_spk_data.angular_precision * -1.0f);
      }
      else if (pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
                   .az_direction == 0)
      {
        pstr_local_setup_data->spk_config.str_flex_spk_data.azimuth[ld_spk] = (FLOAT32)(
            pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
                .az_angle_idx *
            pstr_local_setup_data->spk_config.str_flex_spk_data.angular_precision);
      }

      if (pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
              .el_direction == 1)
      {
        pstr_local_setup_data->spk_config.str_flex_spk_data.elevation[ld_spk] = (FLOAT32)(
            pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
                .el_angle_idx *
            pstr_local_setup_data->spk_config.str_flex_spk_data.angular_precision * -1.0f);
      }
      else if (pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
                   .el_direction == 0)
      {

        pstr_local_setup_data->spk_config.str_flex_spk_data.elevation[ld_spk] = (FLOAT32)(
            pstr_local_setup_data->spk_config.str_flex_spk_data.str_flex_spk_descr[ld_spk]
                .el_angle_idx *
            pstr_local_setup_data->spk_config.str_flex_spk_data.angular_precision);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_lcl_scrn_sz_info
*
*  \brief Update local screen size info
*
*  \param [in]  ptr_bit_buf				Pointer to bit buffer
*
*  \param [in]  pstr_local_setup_data   Pointer to local setup data struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_lcl_scrn_sz_info(ia_bit_buf_struct *ptr_bit_buf,
                                             ia_local_setup_struct *pstr_local_setup_data)
{
  UWORD8 is_centered_azimuth = 0, has_lcl_scrn_elevation_info = 0;
  is_centered_azimuth = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (!(is_centered_azimuth))
  {
    pstr_local_setup_data->lcl_scrn_sz_left_az = ia_core_coder_read_bits_buf(ptr_bit_buf, 10);
    pstr_local_setup_data->lcl_scrn_sz_right_az = ia_core_coder_read_bits_buf(ptr_bit_buf, 10);
  }
  else
  {
    pstr_local_setup_data->lcl_scrn_sz_az = ia_core_coder_read_bits_buf(ptr_bit_buf, 9);
  }
  has_lcl_scrn_elevation_info = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  pstr_local_setup_data->has_lcl_scrn_elevation_info = has_lcl_scrn_elevation_info;
  if (has_lcl_scrn_elevation_info)
  {
    pstr_local_setup_data->lcl_scrn_sz_top_el = ia_core_coder_read_bits_buf(ptr_bit_buf, 9);
    pstr_local_setup_data->lcl_scrn_sz_bottom_el = ia_core_coder_read_bits_buf(ptr_bit_buf, 9);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_ei_grp_intrctvty_status
*
*  \brief Update element interaction group interactivity status
*
*  \param [in]  ptr_bit_buf				Pointer to bit buffer
*  \param [in]  ptr_ele_intrctn			Pointer to element interaction struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_ei_grp_intrctvty_status(ia_bit_buf_struct *ptr_bit_buf,
                                                    ia_ele_intrctn *ptr_ele_intrctn)
{
  WORD32 num_grps = ptr_ele_intrctn->ei_num_groups;
  for (WORD32 grp = 0; grp < num_grps; grp++)
  {
    ptr_ele_intrctn->ei_grp_id[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
    ptr_ele_intrctn->ei_on_off[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    ptr_ele_intrctn->ei_route_to_wire[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    if (ptr_ele_intrctn->ei_route_to_wire[grp] != 1)
    {
      ptr_ele_intrctn->route_to_wire_id[grp] = -1;
    }
    else
    {
      ptr_ele_intrctn->route_to_wire_id[grp] =
          (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 16);
    }
    if (ptr_ele_intrctn->ei_on_off[grp] == 1)
    {
      ptr_ele_intrctn->ei_change_pos[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      if (ptr_ele_intrctn->ei_change_pos[grp])
      {
        ptr_ele_intrctn->ei_az_offset[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
        ptr_ele_intrctn->ei_el_offset[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 6);
        ptr_ele_intrctn->ei_dist_fact[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
      }
      ptr_ele_intrctn->ei_change_gain[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      if (ptr_ele_intrctn->ei_change_gain[grp])
      {
        ptr_ele_intrctn->ei_gain[grp] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_ele_interaction_data
*
*  \brief Update element interaction data
*
*  \param [in]  ptr_bit_buf				Pointer to bit buffer
*  \param [in]  ptr_ele_intrctn			Pointer to element interaction struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_ele_interaction_data(ia_bit_buf_struct *ptr_bit_buf,
                                                 ia_ele_intrctn *ptr_ele_intrctn)
{
  ptr_ele_intrctn->ei_intrctn_mode = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  ptr_ele_intrctn->ei_num_groups = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
  if (ptr_ele_intrctn->ei_intrctn_mode != 0)
  {
    ptr_ele_intrctn->ei_group_preset_id = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 5);
  }
  impeghd_ei_grp_intrctvty_status(ptr_bit_buf, ptr_ele_intrctn);
  ptr_ele_intrctn->ei_data_present = 1;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_lcl_zoom_area_size
*
*  \brief Update zoom area size
*
*  \param [in]  ptr_bit_buf				Pointer to bit buffer
*  \param [in]  ptr_ele_intrctn			Pointer to element interaction struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_lcl_zoom_area_size(ia_bit_buf_struct *ptr_bit_buf,
                                               ia_ele_intrctn *ptr_ele_intrctn)
{
  ptr_ele_intrctn->zoom_az_center = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 10);
  ptr_ele_intrctn->zoom_az = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 10);
  ptr_ele_intrctn->zoom_el_center = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 9);
  ptr_ele_intrctn->zoom_el = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 10);
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_ele_interaction
*
*  \brief Update element interaction struct
*
*  \param [in]  ptr_bit_buf				Pointer to bit buffer
*  \param [in]  ptr_ele_intrctn			Pointer to element interaction struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
IA_ERRORCODE impeghd_ele_interaction(ia_bit_buf_struct *ptr_bit_buf,
                                     ia_ele_intrctn *ptr_ele_intrctn)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 c;
  ptr_ele_intrctn->ei_intrctn_sign_data_len = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
  if (ptr_ele_intrctn->ei_intrctn_sign_data_len > 0)
  {
    ptr_ele_intrctn->ei_intrctn_sign_data_type =
        (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    for (c = 0; c < ptr_ele_intrctn->ei_intrctn_sign_data_len; c++)
    {
      ptr_ele_intrctn->ei_intrctn_sign_data[c] =
          (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    }
  }
  impeghd_ele_interaction_data(ptr_bit_buf, ptr_ele_intrctn);
  ptr_ele_intrctn->has_zoom_area_sz = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (ptr_ele_intrctn->has_zoom_area_sz)
  {
    err_code = impeghd_lcl_zoom_area_size(ptr_bit_buf, ptr_ele_intrctn);
    if (err_code)
    {
      return err_code;
    }
  }
  ptr_ele_intrctn->ei_data_present = 1;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_scene_displacement_data
*
*  \brief Update scene displacement data structure
*
*  \param [in/out]  ptr_scene_disp_data   Pointer to scene displacement data structure
*
*  \return error IA_ERRORCODE if any
*
*/
IA_ERRORCODE impeghd_scene_displacement_data(ia_scene_disp_data *ptr_scene_disp_data)
{
  ptr_scene_disp_data->sd_yaw = ia_core_coder_read_bits_buf(&ptr_scene_disp_data->sd_buf, 9);
  ptr_scene_disp_data->sd_pitch = ia_core_coder_read_bits_buf(&ptr_scene_disp_data->sd_buf, 9);
  ptr_scene_disp_data->sd_roll = ia_core_coder_read_bits_buf(&ptr_scene_disp_data->sd_buf, 9);
  ptr_scene_disp_data->sd_azimuth = ia_core_coder_read_bits_buf(&ptr_scene_disp_data->sd_buf, 8);
  ptr_scene_disp_data->sd_elevation =
      ia_core_coder_read_bits_buf(&ptr_scene_disp_data->sd_buf, 6);
  ptr_scene_disp_data->sd_radius = ia_core_coder_read_bits_buf(&ptr_scene_disp_data->sd_buf, 4);
  ptr_scene_disp_data->scene_dspl_data_present = 1;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_3da_local_setup_information
*
*  \brief Update local setup struct from bitstream
*
*  \param [in]  ptr_bit_buf				Pointer to bit buffer
*
*  \param [in]  pstr_local_setup_data   Pointer to local setup data struct
* description
*
*  \return error IA_ERRORCODE if any
*
*/
IA_ERRORCODE impeghd_3da_local_setup_information(ia_bit_buf_struct *ptr_bit_buf,
                                                 ia_local_setup_struct *pstr_local_setup_data)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  UWORD32 ren_type = 0, num_wire_out = 0, has_lcl_scrn_sz_info = 0;
  ren_type = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  pstr_local_setup_data->enable_ele_int = 1;
  pstr_local_setup_data->ren_type = ren_type;
  pstr_local_setup_data->is_brir_rendering = 0;

  switch (ren_type)
  {
  case 1:
    pstr_local_setup_data->is_brir_rendering = 1;
    err_code = impeghd_read_brir_info(pstr_local_setup_data->pstr_binaural_renderer, ptr_bit_buf);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
    break;
  case 0:
    impeghd_loud_spk_ren(ptr_bit_buf, pstr_local_setup_data);
    impeghd_loud_spk_inv_quant(pstr_local_setup_data);
    break;
  }
  num_wire_out = ia_core_coder_read_bits_buf(ptr_bit_buf, 16);
  if (num_wire_out > 0)
  {
    for (UWORD32 n = 0; n < num_wire_out; n++)
    {
      pstr_local_setup_data->wire_id[n] = ia_core_coder_read_bits_buf(ptr_bit_buf, 16);
    }
  }
  has_lcl_scrn_sz_info = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  pstr_local_setup_data->has_local_screen_size_info = has_lcl_scrn_sz_info;
  if (has_lcl_scrn_sz_info)
  {
    err_code = impeghd_lcl_scrn_sz_info(ptr_bit_buf, pstr_local_setup_data);
  }
  return err_code;
}
/** @} */ /* End of EleIntrIfc */