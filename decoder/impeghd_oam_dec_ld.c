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
#include "impeghd_error_codes.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_config.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_oam_dec.h"

/**
 * @defgroup OAMDec Object Audio Metadata Decoder
 * @ingroup  OAMDec
 * @brief Object Audio Metadata Decoder
 *
 * @{
 */

/**
 *  impeghd_limit_range_ld_obj_md
 *
 *  \brief Checks if the OMD parameters are within the valid value range.
 *
 *  \param [in,out] ptr_oam_dec_state Object metadata decoder state structure.
 *
 *
 *
 */
static VOID impeghd_limit_range_ld_obj_md(ia_oam_dec_state_struct *ptr_oam_dec_state)
{
  WORD32 n, num_elements;
  FLOAT32 minval, maxval;
  num_elements = ptr_oam_dec_state->num_objects;
  /* radius */
  maxval = 16.f;
  minval = 0.5f;
  for (n = 0; n < num_elements; n++)
  {
    ptr_oam_dec_state->radius_descaled[n] =
        ia_min_int(ia_max_int(ptr_oam_dec_state->radius_descaled[n], minval), maxval);
  }
  /* elevation */
  maxval = 90;
  minval = -90;
  for (n = 0; n < num_elements; n++)
  {
    ptr_oam_dec_state->elevation_descaled[n] =
        ia_min_int(ia_max_int(ptr_oam_dec_state->elevation_descaled[n], minval), maxval);
  }
  /* azimuth */
  maxval = 180;
  minval = -180;
  for (n = 0; n < num_elements; n++)
  {
    ptr_oam_dec_state->azimuth_descaled[n] =
        ia_min_int(ia_max_int(ptr_oam_dec_state->azimuth_descaled[n], minval), maxval);
  }
  /* gain */
  maxval = 5.957f;
  minval = 0.004f;
  for (n = 0; n < num_elements; n++)
  {
    ptr_oam_dec_state->gain_descaled[n] =
        ia_min_int(ia_max_int(ptr_oam_dec_state->gain_descaled[n], minval), maxval);
  }
  /* spread */
  maxval = 180;
  minval = 0;
  for (n = 0; n < num_elements; n++)
  {
    ptr_oam_dec_state->spread_width_descaled[n] =
        ia_min_int(ia_max_int(ptr_oam_dec_state->spread_width_descaled[n], minval), maxval);
  }
  if (!ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present)
  {
    /* spread depth */
    maxval = 15.5f;
    minval = 0.0f;
    for (n = 0; n < num_elements; n++)
    {
      ptr_oam_dec_state->spread_depth_descaled[n] =
          ia_min_int(ia_max_int(ptr_oam_dec_state->spread_depth_descaled[n], minval), maxval);
    }
    /* spread height */
    maxval = 90;
    minval = 0;
    for (n = 0; n < num_elements; n++)
    {
      ptr_oam_dec_state->spread_height_descaled[n] =
          ia_min_int(ia_max_int(ptr_oam_dec_state->spread_height_descaled[n], minval), maxval);
    }
  }
  /* dynamic object priority */
  maxval = 7.0f;
  minval = 0.0f;
  for (n = 0; n < num_elements; n++)
  {
    ptr_oam_dec_state->dyn_obj_priority_descaled[n] =
        ia_min_int(ia_max_int(ptr_oam_dec_state->dyn_obj_priority_descaled[n], minval), maxval);
  }
}

/**
 *  impeghd_obj_md_descale_limit
 *
 *  \brief Object Metadata descaling and limiting
 *
 *  \param [in,out] ptr_oam_dec_state Object metadata decoder state structure.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
static IA_ERRORCODE impeghd_obj_md_descale_limit(ia_oam_dec_state_struct *ptr_oam_dec_state)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;

  impeghd_descale_ld_obj_md(ptr_oam_dec_state);

  impeghd_limit_range_ld_obj_md(ptr_oam_dec_state);

  return err_code;
}

/**
 *  impeghd_oam_read_sign_param
 *
 *  \brief Reads a signed object metadata bitstream param.
 *
 *  \param [in] ptr_bit_buf Pointer to bit buffer structure.
 *  \param [in] num_bits    Number of bits corresponding to parameter.
 *
 *  \return WORD32     Signed bit stream value read.
 *
 */
static WORD32 impeghd_oam_read_sign_param(ia_bit_buf_struct *ptr_bit_buf, WORD32 num_bits)
{
  WORD32 param_val = ia_core_coder_read_bits_buf(ptr_bit_buf, num_bits);
  if (param_val >= (1 << (num_bits - 1)))
  {
    param_val = param_val - (1 << num_bits);
  }
  return param_val;
}

/**
 *  impeghd_ic_oam_ld_signed_param_dec
 *
 *  \brief Reads signed bit stream parameter of intracoded object metadata.
 *
 *  \param [in]  fix_val_flag Flag to indicate if all objects take same param value.
 *  \param [in]  param_bits   Number of bits corresponding to the param.
 *  \param [out] param_buf    Buffer to store decoded param values.
 *  \param [in]  num_objects  Number of objects.
 *  \param [in]  ptr_bit_buf  Pointer to bit buffer structure
 *
 *
 *
 */
static VOID impeghd_ic_oam_ld_signed_param_dec(WORD32 fix_val_flag, WORD32 param_bits,
                                               WORD32 *param_buf, WORD32 num_objects,
                                               ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 i;
  WORD32 param_val;
  if (fix_val_flag)
  {
    param_val = impeghd_oam_read_sign_param(ptr_bit_buf, param_bits);
    for (i = 0; i < num_objects; i++)
    {
      param_buf[i] = param_val;
    }
  }
  else
  {
    if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
    {
      param_val = impeghd_oam_read_sign_param(ptr_bit_buf, param_bits);
      for (i = 0; i < num_objects; i++)
      {
        param_buf[i] = param_val;
      }
    }
    else
    {
      for (i = 0; i < num_objects; i++)
      {
        param_val = impeghd_oam_read_sign_param(ptr_bit_buf, param_bits);
        param_buf[i] = param_val;
      }
    }
  }
  return;
}

/**
 *  impeghd_ic_oam_ld_unsigned_param_dec
 *
 *  \brief Reads unsigned bit stream parameter of intracoded object metadata.
 *
 *  \param [in]  fix_val_flag Flag to indicate if all objects take same param value.
 *  \param [in]  param_bits   Number of bits corresponding to the param.
 *  \param [out] param_buf    Buffer to store decoded param values.
 *  \param [in]  num_objects  Number of objects.
 *  \param [in]  ptr_bit_buf  Pointer to bit buffer structure
 *
 *
 *
 */
static VOID impeghd_ic_oam_ld_unsigned_param_dec(WORD32 fix_val_flag, WORD32 param_bits,
                                                 WORD32 *param_buf, WORD32 num_objects,
                                                 ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 i;
  WORD32 param_val;

  if (fix_val_flag)
  {
    param_val = ia_core_coder_read_bits_buf(ptr_bit_buf, param_bits);
    for (i = 0; i < num_objects; i++)
    {
      param_buf[i] = param_val;
    }
  }
  else
  {

    if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
    {
      param_val = ia_core_coder_read_bits_buf(ptr_bit_buf, param_bits);
      for (i = 0; i < num_objects; i++)
      {
        param_buf[i] = param_val;
      }
    }
    else
    {
      for (i = 0; i < num_objects; i++)
      {
        param_val = ia_core_coder_read_bits_buf(ptr_bit_buf, param_bits);
        param_buf[i] = param_val;
      }
    }
  }
  return;
}

/**
 *  impeghd_single_dyn_obj_md
 *
 *  \brief Reads single dynamic object metadata index.
 *
 *  \param [out] ptr_oam_dec_state Object metadata decoder state structure.
 *  \param [in]  ptr_bit_buf       Pointer to bit buffer structure.
 *  \param [in]  flag_absolute     Flag used for processing by the function.
 *  \param [in]  obj_idx           Index of the object that is processed.
 *
 *  \return IA_ERRORCODE           Bit stream parsing error if any.
 *
 */
static IA_ERRORCODE impeghd_single_dyn_obj_md(ia_oam_dec_state_struct *ptr_oam_dec_state,
                                              ia_bit_buf_struct *ptr_bit_buf,
                                              WORD32 flag_absolute, WORD32 obj_idx)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  if (!flag_absolute)
  {
    WORD32 param_val;
    WORD32 nbits = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SINGLE_DYN_OBJ_NBITS);
    nbits = nbits + 2;
    if (!ptr_oam_dec_state->azimuth_fixed)
    {
      if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
      {
        WORD32 num_bits = nbits;
        param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
        ptr_oam_dec_state->azimuth[obj_idx] += param_val;
      }
    }
    if (!ptr_oam_dec_state->elevation_fixed)
    {
      if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
      {
        WORD32 num_bits = ia_min_int(OAM_ELEVATION_BITS + 1, nbits);
        param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
        ptr_oam_dec_state->elevation[obj_idx] += param_val;
      }
    }
    if (!ptr_oam_dec_state->radius_fixed)
    {
      if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
      {
        WORD32 num_bits = ia_min_int(OAM_RADIUS_BITS + 1, nbits);
        param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
        ptr_oam_dec_state->radius[obj_idx] += param_val;
      }
    }
    if (!ptr_oam_dec_state->gain_fixed)
    {
      if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
      {
        WORD32 num_bits = ia_min_int(OAM_GAIN_BITS + 1, nbits);
        param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
        ptr_oam_dec_state->gain[obj_idx] += param_val;
      }
    }
    if (!ptr_oam_dec_state->spread_fixed)
    {
      if (!ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present)
      {
        WORD32 num_bits = ia_min_int(OAM_SPREAD_WIDTH_BITS + 1, nbits);
        if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
        {
          param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
          ptr_oam_dec_state->spread_width[obj_idx] += param_val;
        }
        num_bits = ia_min_int(OAM_SPREAD_HEIGHT_BITS + 1, nbits);
        if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
        {
          param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
          ptr_oam_dec_state->spread_height[obj_idx] += param_val;
        }

        num_bits = ia_min_int(OAM_SPREAD_DEPTH_BITS + 1, nbits);
        if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
        {
          param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
          ptr_oam_dec_state->spread_depth[obj_idx] += param_val;
        }
      }
      else
      {
        if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
        {
          WORD32 num_bits = ia_min_int(OAM_SPREAD_WIDTH_BITS + 1, nbits);
          param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
          ptr_oam_dec_state->spread_width[obj_idx] += param_val;
        }
      }
    }
    else
    {
      ptr_oam_dec_state->spread_width[obj_idx] = 0;
      ptr_oam_dec_state->spread_height[obj_idx] = 0;
      ptr_oam_dec_state->spread_depth[obj_idx] = 0;
    }
    if (ptr_oam_dec_state->p_obj_md_cfg->dyn_obj_priority_present)
    {
      if (ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS))
      {
        WORD32 num_bits = ia_min_int(OAM_DYN_OBJ_PRI_BITS + 1, nbits);
        param_val = impeghd_oam_read_sign_param(ptr_bit_buf, num_bits);
        ptr_oam_dec_state->dyn_obj_priority[obj_idx] += param_val;
      }
    }
  }
  else
  {
    if (ptr_oam_dec_state->azimuth_fixed)
    {
      ptr_oam_dec_state->azimuth[obj_idx] = 0;
    }
    else
    {
      ptr_oam_dec_state->azimuth[obj_idx] =
          impeghd_oam_read_sign_param(ptr_bit_buf, OAM_AZIMUTH_BITS);
    }
    if (ptr_oam_dec_state->elevation_fixed)
    {
      ptr_oam_dec_state->elevation[obj_idx] = 0;
    }
    else
    {
      ptr_oam_dec_state->elevation[obj_idx] =
          impeghd_oam_read_sign_param(ptr_bit_buf, OAM_ELEVATION_BITS);
    }
    if (ptr_oam_dec_state->radius_fixed)
    {
      ptr_oam_dec_state->radius[obj_idx] = 0;
    }
    else
    {
      ptr_oam_dec_state->radius[obj_idx] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_RADIUS_BITS);
    }
    if (ptr_oam_dec_state->gain_fixed)
    {
      ptr_oam_dec_state->gain[obj_idx] = 0;
    }
    else
    {
      ptr_oam_dec_state->gain[obj_idx] = impeghd_oam_read_sign_param(ptr_bit_buf, OAM_GAIN_BITS);
    }
    if (ptr_oam_dec_state->spread_fixed)
    {
      ptr_oam_dec_state->spread_width[obj_idx] = 0;
      ptr_oam_dec_state->spread_height[obj_idx] = 0;
      ptr_oam_dec_state->spread_depth[obj_idx] = 0;
    }
    else
    {
      if (!ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present)
      {
        ptr_oam_dec_state->spread_width[obj_idx] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
        ptr_oam_dec_state->spread_height[obj_idx] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_HEIGHT_BITS);
        ptr_oam_dec_state->spread_depth[obj_idx] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_HEIGHT_BITS);
      }
      else
      {
        ptr_oam_dec_state->spread_width[obj_idx] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
      }
    }
    if (ptr_oam_dec_state->p_obj_md_cfg->dyn_obj_priority_present)
    {
      if (!ptr_oam_dec_state->dynamic_obj_priority_fixed)
      {
        ptr_oam_dec_state->dyn_obj_priority[obj_idx] = 0;
      }
      else
      {
        ptr_oam_dec_state->dyn_obj_priority[obj_idx] = 0;
        ptr_oam_dec_state->dyn_obj_priority[obj_idx] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_DYN_OBJ_PRI_BITS);
      }
    }
  }
  return err_code;
}

/**
 *  impeghd_ic_obj_md_ld_dec
 *
 *  \brief Intracoded object metadata decoding function.
 *
 *  \param [out] ptr_oam_dec_state Object metadata decoder state structure.
 *  \param [in]  ptr_bit_buf       Pointer to bit buffer structure.
 *
 *  \return IA_ERRORCODE Error if any
 *
 */
static IA_ERRORCODE impeghd_ic_obj_md_ld_dec(ia_oam_dec_state_struct *ptr_oam_dec_state,
                                             ia_bit_buf_struct *ptr_bit_buf)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 num_objects = ptr_oam_dec_state->num_objects;
  WORD32 obj_idx_start = (ptr_oam_dec_state->sub_frame_number - 1) * num_objects;
  WORD32 obj_idx_end = (ptr_oam_dec_state->sub_frame_number) * num_objects;
  if (!(num_objects > 1))
  {
    /* Azimuth */
    ptr_oam_dec_state->azimuth[obj_idx_start] =
        impeghd_oam_read_sign_param(ptr_bit_buf, OAM_AZIMUTH_BITS);
    ;

    /* Elevation */
    ptr_oam_dec_state->elevation[obj_idx_start] =
        impeghd_oam_read_sign_param(ptr_bit_buf, OAM_ELEVATION_BITS);

    /* Radius */
    ptr_oam_dec_state->radius[obj_idx_start] =
        ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_RADIUS_BITS);

    /* Gain */
    ptr_oam_dec_state->gain[obj_idx_start] =
        impeghd_oam_read_sign_param(ptr_bit_buf, OAM_GAIN_BITS);

    /* Spread */
    if (!ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present)
    {
      ptr_oam_dec_state->spread_width[obj_idx_start] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
      ptr_oam_dec_state->spread_height[obj_idx_start] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_HEIGHT_BITS);
      ptr_oam_dec_state->spread_depth[obj_idx_start] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_DEPTH_BITS);
    }
    else
    {
      ptr_oam_dec_state->spread_width[obj_idx_start] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
    }
    /* Dynamic Object Priority */
    if (ptr_oam_dec_state->p_obj_md_cfg->dyn_obj_priority_present)
    {
      ptr_oam_dec_state->dyn_obj_priority[obj_idx_start] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_DYN_OBJ_PRI_BITS);
    }
    ptr_oam_dec_state->azimuth_fixed = 0;
    ptr_oam_dec_state->elevation_fixed = 0;
    ptr_oam_dec_state->radius_fixed = 0;
    ptr_oam_dec_state->gain_fixed = 0;
    ptr_oam_dec_state->spread_fixed = 0;
    ptr_oam_dec_state->dynamic_obj_priority_fixed = 0;
  }
  else
  {
    LOOPIDX i;
    WORD32 param_val;
    WORD32 com_val_flag;
    WORD32 fix_val_flag;
    /* Azimuth */
    fix_val_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
    ptr_oam_dec_state->azimuth_fixed = fix_val_flag;
    impeghd_ic_oam_ld_signed_param_dec(fix_val_flag, OAM_AZIMUTH_BITS,
                                       &ptr_oam_dec_state->azimuth[obj_idx_start], num_objects,
                                       ptr_bit_buf);

    /* Elevation */
    fix_val_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
    ptr_oam_dec_state->elevation_fixed = fix_val_flag;
    impeghd_ic_oam_ld_signed_param_dec(fix_val_flag, OAM_ELEVATION_BITS,
                                       &ptr_oam_dec_state->elevation[obj_idx_start], num_objects,
                                       ptr_bit_buf);

    /* Radius */
    fix_val_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
    ptr_oam_dec_state->radius_fixed = fix_val_flag;
    impeghd_ic_oam_ld_unsigned_param_dec(fix_val_flag, OAM_RADIUS_BITS,
                                         &ptr_oam_dec_state->radius[obj_idx_start], num_objects,
                                         ptr_bit_buf);

    /* Gain */
    fix_val_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
    ptr_oam_dec_state->gain_fixed = fix_val_flag;
    impeghd_ic_oam_ld_signed_param_dec(fix_val_flag, OAM_GAIN_BITS,
                                       &ptr_oam_dec_state->gain[obj_idx_start], num_objects,
                                       ptr_bit_buf);

    /* Spread */
    fix_val_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
    ptr_oam_dec_state->spread_fixed = fix_val_flag;
    if (!fix_val_flag)
    {
      com_val_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
      if (com_val_flag)
      {
        if (ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present)
        {
          param_val = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
          for (i = obj_idx_start; i < obj_idx_end; i++)
          {
            ptr_oam_dec_state->spread_width[i] = param_val;
          }
        }
        else
        {
          WORD32 spread_width = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
          WORD32 spread_height = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_HEIGHT_BITS);
          WORD32 spread_depth = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_DEPTH_BITS);
          for (i = obj_idx_start; i < obj_idx_end; i++)
          {
            ptr_oam_dec_state->spread_width[i] = spread_width;
            ptr_oam_dec_state->spread_height[i] = spread_height;
            ptr_oam_dec_state->spread_depth[i] = spread_depth;
          }
        }
      }
      else
      {
        if (ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present)
        {
          for (i = obj_idx_start; i < obj_idx_end; i++)
          {
            param_val = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
            ptr_oam_dec_state->spread_width[i] = param_val;
          }
        }
        else
        {
          for (i = obj_idx_start; i < obj_idx_end; i++)
          {
            WORD32 spread_width = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
            WORD32 spread_height =
                ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_HEIGHT_BITS);
            WORD32 spread_depth = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_DEPTH_BITS);
            ptr_oam_dec_state->spread_width[i] = spread_width;
            ptr_oam_dec_state->spread_height[i] = spread_height;
            ptr_oam_dec_state->spread_depth[i] = spread_depth;
          }
        }
      }
    }
    else
    {
      if (ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present)
      {
        param_val = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
        for (i = 0; i < num_objects; i++)
        {
          ptr_oam_dec_state->spread_width[i] = param_val;
        }
      }
      else
      {
        WORD32 spread_width = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_WIDTH_BITS);
        WORD32 spread_height = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_HEIGHT_BITS);
        WORD32 spread_depth = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_SPREAD_DEPTH_BITS);
        for (i = obj_idx_start; i < obj_idx_end; i++)
        {
          ptr_oam_dec_state->spread_width[i] = spread_width;
          ptr_oam_dec_state->spread_height[i] = spread_height;
          ptr_oam_dec_state->spread_depth[i] = spread_depth;
        }
      }
    }
    /* Dynamic Object Priority */
    if (ptr_oam_dec_state->p_obj_md_cfg->dyn_obj_priority_present)
    {
      ptr_oam_dec_state->dynamic_obj_priority_fixed =
          ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
      impeghd_ic_oam_ld_unsigned_param_dec(
          ptr_oam_dec_state->dynamic_obj_priority_fixed, OAM_DYN_OBJ_PRI_BITS,
          &ptr_oam_dec_state->dyn_obj_priority[obj_idx_start], num_objects, ptr_bit_buf);
    }
  }
  return err_code;
}

/**
 *  impeghd_descale_ld_obj_md
 *
 *  \brief Descale extracted bit stream parameters
 *
 *  \param [in,out] ptr_oam_dec_state Pointer to oam dec state structure
 *
 *
 *
 */
VOID impeghd_descale_ld_obj_md(ia_oam_dec_state_struct *ptr_oam_dec_state)
{
  WORD32 o;
  WORD32 num_objects = ptr_oam_dec_state->num_objects;
  WORD32 obj_start_idx = (ptr_oam_dec_state->sub_frame_number - 1) * num_objects;
  WORD32 obj_end_idx = (ptr_oam_dec_state->sub_frame_number) * num_objects;
  for (o = obj_start_idx; o < obj_end_idx; o++)
  {
    ptr_oam_dec_state->dyn_obj_priority_descaled[o] =
        (FLOAT32)ptr_oam_dec_state->dyn_obj_priority[o];
  }
  if (ptr_oam_dec_state->p_obj_md_cfg->uniform_spread_present != 1)
  {
    for (o = obj_start_idx; o < obj_end_idx; o++)
    {
      ptr_oam_dec_state->spread_width_descaled[o] =
          (FLOAT32)(ptr_oam_dec_state->spread_width[o] * 1.5f);
      ptr_oam_dec_state->spread_height_descaled[o] =
          (FLOAT32)(ptr_oam_dec_state->spread_height[o] * 3.0f);
      ptr_oam_dec_state->spread_depth_descaled[o] =
          (FLOAT32)(pow(2.0f, (ptr_oam_dec_state->spread_depth[o] / 3.0)) / 2.0f) - 0.5f;
    }
  }
  else
  {

    for (o = obj_start_idx; o < obj_end_idx; o++)
    {
      ptr_oam_dec_state->spread_width_descaled[o] = ptr_oam_dec_state->spread_width[o] * 1.5f;
    }
  }
  for (o = obj_start_idx; o < obj_end_idx; o++)
  {
    ptr_oam_dec_state->radius_descaled[o] =
        (FLOAT32)pow(2.0f, (ptr_oam_dec_state->radius[o] / 3.0f)) / 2.0f;
    ptr_oam_dec_state->elevation_descaled[o] = (FLOAT32)ptr_oam_dec_state->elevation[o] * 3.0f;
    ptr_oam_dec_state->azimuth_descaled[o] = (FLOAT32)ptr_oam_dec_state->azimuth[o] * 1.5f;
    ptr_oam_dec_state->gain_descaled[o] =
        (FLOAT32)pow(10.0f, (ptr_oam_dec_state->gain[o] - 32.0f) / 40.0f);
  }

  return;
}

/**
 *  impeghd_obj_md_low_delay_dec
 *
 *  \brief Object Metadata bit stream decoder for low delay mode
 *
 *  \param [in,out] ptr_oam_dec_state Object metadata decoder state structure.
 *  \param [in]  ptr_bit_buf       Pointer to bit buffer structure.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE impeghd_obj_md_low_delay_dec(ia_oam_dec_state_struct *ptr_oam_dec_state,
                                          ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 i;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 intra_obj_md_present = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);

  if (!intra_obj_md_present)
  {
    WORD32 flag_absolute = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
    WORD32 num_objects = ptr_oam_dec_state->num_objects;
    WORD32 obj_start_idx = (ptr_oam_dec_state->sub_frame_number - 1) * num_objects;
    WORD32 obj_end_idx = (ptr_oam_dec_state->sub_frame_number) * num_objects;
    for (i = obj_start_idx; i < obj_end_idx; i++)
    {
      WORD32 obj_md_present = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
      ptr_oam_dec_state->has_obj_md[obj_start_idx] = obj_md_present;
      if (obj_md_present)
      {
        err_code = impeghd_single_dyn_obj_md(ptr_oam_dec_state, ptr_bit_buf, flag_absolute, i);
        if (err_code)
        {
          return err_code;
        }
      }
    }
  }
  else
  {
    err_code = impeghd_ic_obj_md_ld_dec(ptr_oam_dec_state, ptr_bit_buf);
  }

  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_obj_md_descale_limit(ptr_oam_dec_state);
  for (i = 0; i < ptr_oam_dec_state->num_objects; i++)
    ptr_oam_dec_state->sample[i] += ptr_oam_dec_state->p_obj_md_cfg->frame_length;

  return err_code;
}

/** @} */ /* End of OAMDec */