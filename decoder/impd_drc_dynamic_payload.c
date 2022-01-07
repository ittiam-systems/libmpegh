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

#include <impeghd_type_def.h>
#include "ia_core_coder_bitbuffer.h"
#include "impd_drc_common.h"
#include "impeghd_error_codes.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_parser.h"
#include "impd_drc_rom.h"

/**
 * @defgroup DRCFrameParse DRCFrameParse
 * @ingroup  DRCFrameParse
 * @brief DRC Frame payload parsing
 *
 * @{
 */

/**
 *  impd_drc_get_initial_gain
 *
 *  \brief Reading initial gain value from bit-stream
 *
 *  \param [out] gain_initial  gain value
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  gain_coding_prof  gain coding profile value
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_get_initial_gain(FLOAT32 *gain_initial, ia_bit_buf_struct *pstr_it_bit_buff,
                                       const WORD32 gain_coding_prof)
{
  WORD32 sign, magn;
  *gain_initial = 0;
  switch (gain_coding_prof)
  {
  case IA_DRC_GAIN_CODING_PROFILE_CONSTANT:
    break;
  case IA_DRC_GAIN_CODING_PROFILE_CLIPPING:
    sign = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    if (sign)
    {
      magn = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);
      *gain_initial = -(magn + 1) * 0.125f;
    }
    break;
  case IA_DRC_GAIN_CODING_PROFILE_FADING:
    sign = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    if (sign)
    {
      magn = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 10);
      *gain_initial = -(magn + 1) * 0.125f;
    }
    break;
  case IA_DRC_GAIN_CODING_PROFILE_REGULAR:
    sign = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    magn = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);
    *gain_initial = magn * 0.125f;
    if (sign)
      *gain_initial = -*gain_initial;
    break;
  default:
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_update_value
 *
 *  \brief Helper function to decode gain, slope and time values
 *
 *  \param [out] value decoded gain, slope or time value
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  table   Pointer to parameter(gain, slope or time) code table
 *  \param [in]  no_entries  Number of table entries
 *  \param [in]  update_time  Flag indicates function is called to update time value
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE impd_drc_update_value(FLOAT32 *value, ia_bit_buf_struct *pstr_it_bit_buff,
                                          const ia_drc_delta_param_table_entry_struct *table,
                                          WORD32 no_entries, WORD32 update_time)
{
  WORD32 num_bits_read = 0, idx = 0;
  WORD32 code = 0, size;
  WORD32 bit;
  WORD32 code_found = 0, exit_cnt = 0;

  if (update_time == 1)
  {
    idx = 1;
  }

  while ((idx < no_entries) && (!code_found))
  {
    if (update_time == 1)
    {
      exit_cnt++;
      if (exit_cnt > 100000)
      {
        return IA_MPEGD_DRC_INIT_FATAL_UNEXPECTED_ERROR;
      }
    }
    for (size = 0; size < table[idx].size - num_bits_read; size++)
    {
      bit = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
      num_bits_read++;
      code = (code << 1) + bit;
    }
    while (num_bits_read == table[idx].size)
    {
      if (code == table[idx].code)
      {
        code_found = 1;
        *value = table[idx].value;
        break;
      }
      idx++;
      if (idx > no_entries)
      {
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      }
    }
  }
  if (code_found == 0)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impd_drc_decode_gains
 *
 *  \brief decode gain values
 *
 *  \param [out] pstr_node  Pointer to node struct
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  no_nodes   No of node
 *  \param [in]  gain_coding_prof  gain coding profile value
 *
 *  \return IA_ERRORCODE error
 *
 */
WORD32 impd_drc_decode_gains(ia_drc_node_struct *pstr_node, ia_bit_buf_struct *pstr_it_bit_buff,
                             WORD32 no_nodes, WORD32 gain_coding_prof)
{
  const ia_drc_delta_param_table_entry_struct *ptr_delta_gain_code_table;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 node;
  WORD32 no_delta_gain_entries;
  FLOAT32 drc_gain_delta = 0;

  err_code =
      impd_drc_get_initial_gain(&(pstr_node[0].loc_gain_db), pstr_it_bit_buff, gain_coding_prof);
  if (err_code)
    return (err_code);

  impd_drc_create_delta_gain_code_tbl(&ptr_delta_gain_code_table, &no_delta_gain_entries,
                                      gain_coding_prof);
  for (node = 1; node < no_nodes; node++)
  {
    err_code = impd_drc_update_value(&drc_gain_delta, pstr_it_bit_buff, ptr_delta_gain_code_table,
                                     no_delta_gain_entries, 0);
    if (err_code)
    {
      return err_code;
    }

    pstr_node[node].loc_gain_db = pstr_node[node - 1].loc_gain_db + drc_gain_delta;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_decode_slopes
 *
 *  \brief decode slope values
 *
 *  \param [out] pstr_node  Pointer to node struct
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  no_nodes   No of node
 *  \param [in]  gain_interp_type  gain interpolation type
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE impd_drc_decode_slopes(ia_drc_node_struct *pstr_node,
                                           ia_bit_buf_struct *pstr_it_bit_buff, WORD32 *no_nodes,
                                           WORD32 gain_interp_type)
{
  const ia_drc_delta_param_table_entry_struct *ptr_slope_code_table;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 node = 0;
  WORD32 num_slope_code_entries;
  FLOAT32 slope_val = 0;
  bool end_marker_flag = 0;

  while (end_marker_flag != 1)
  {
    end_marker_flag = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    node++;
  }

  if (node > NODE_COUNT_MAX)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
  }

  /*nNodes	shall	be	restricted	to	a	maximum	value	of	32,
  where nNodes	is	the	number	of	encoded	gain values	in
  the	current	DRC	frame*/
  if (node > MAX_NODE)
  {
    return IA_MPEGD_DRC_INIT_FATAL_INVALID_DRC_PARAM_FOR_LC_PROFILE;
  }

  *no_nodes = node;

  ptr_slope_code_table = &(ia_drc_slope_code_tbl_entries_by_size[0]);
  num_slope_code_entries = NUM_SLOPE_TBL_ENTRIES;

  if (gain_interp_type == IA_DRC_GAIN_INTERPOLATION_TYPE_LINEAR)
  {
    for (node = 0; node < *no_nodes; node++)
    {
      pstr_node[node].slope = 0;
    }
  }
  else
  {
    for (node = 0; node < *no_nodes; node++)
    {
      err_code = impd_drc_update_value(&slope_val, pstr_it_bit_buff, ptr_slope_code_table,
                                       num_slope_code_entries, 0);
      if (err_code)
      {
        return err_code;
      }
      pstr_node[node].slope = slope_val;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_decode_times
 *
 *  \brief decode time values
 *
 *  \param [out] pstr_node          Pointer to node struct
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  pstr_tables        Pointer to time code tables
 *  \param [in]  delta_tmin         Delta min value
 *  \param [in]  drc_frame_size     Drc frame size
 *  \param [in]  full_frame         Flag to identify full frame
 *  \param [in]  no_of_nodes        No of nodes
 *  \param [in]  time_offset        Time offset
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_decode_times(ia_drc_node_struct *pstr_node,
                                   ia_bit_buf_struct *pstr_it_bit_buff,
                                   ia_drc_tables_struct *pstr_tables, WORD32 delta_tmin,
                                   WORD32 drc_frame_size, WORD32 full_frame, WORD32 no_of_nodes,
                                   WORD32 time_offset)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 node;
  FLOAT32 delta_time = 0;
  WORD32 time_off = time_offset;
  WORD32 node_time_temp;
  bool frame_end;
  bool node_res;

  if (full_frame != 0)
  {
    frame_end = 1;
  }
  else
  {
    frame_end = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
  }

  if (frame_end == 1)
  {
    node_res = 0;
    for (node = 0; node < no_of_nodes - 1; node++)
    {
      err_code =
          impd_drc_update_value(&delta_time, pstr_it_bit_buff, pstr_tables->delta_time_code_table,
                                N_DELTA_TIME_CODE_TABLE_ENTRIES_MAX, 1);
      if (err_code)
      {
        return err_code;
      }
      node_time_temp = time_off + (WORD32)delta_time * delta_tmin;
      if (node_time_temp <= drc_frame_size + time_offset)
      {
        pstr_node[node].time = node_time_temp;
      }
      else
      {
        pstr_node[node + 1].time = node_time_temp;
        if (node_res == 0)
        {
          pstr_node[node].time = drc_frame_size + time_offset;
          node_res = 1;
        }
      }
      time_off = node_time_temp;
    }
    if (node_res == 0)
    {
      pstr_node[node].time = drc_frame_size + time_offset;
    }
  }
  else
  {
    for (node = 0; node < no_of_nodes; node++)
    {
      err_code =
          impd_drc_update_value(&delta_time, pstr_it_bit_buff, pstr_tables->delta_time_code_table,
                                N_DELTA_TIME_CODE_TABLE_ENTRIES_MAX, 1);
      if (err_code)
      {
        return err_code;
      }

      time_off += (WORD32)delta_time * delta_tmin;
      if (time_off >= (2 * AUDIO_CODEC_FRAME_SIZE_MAX - drc_frame_size))
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      pstr_node[node].time = time_off;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_read_uni_drc_gain_ext
 *
 *  \brief Parse uni drc gain extension payload
 *
 *  \param [out] pstr_uni_drc_gain_ext  Pointer to uni drc gain ext struct
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE
impd_drc_read_uni_drc_gain_ext(ia_drc_uni_drc_gain_ext_struct *pstr_uni_drc_gain_ext,
                               ia_bit_buf_struct *pstr_it_bit_buff)
{
  WORD32 cnt = 0;
  WORD32 bit_size_len, ext_size_len, ext_bit_size;

  pstr_uni_drc_gain_ext->uni_drc_gain_ext_type[cnt] =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

  while (pstr_uni_drc_gain_ext->uni_drc_gain_ext_type[cnt] != UNIDRCGAINEXT_TERM)
  {
    if (cnt >= (EXT_COUNT_MAX - 1))
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;

    bit_size_len = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 3);
    ext_size_len = bit_size_len + 4;

    ext_bit_size = ia_core_coder_read_bits_buf(pstr_it_bit_buff, ext_size_len);
    pstr_uni_drc_gain_ext->ext_bit_size[cnt] = ext_bit_size + 1;

    ia_core_coder_skip_bits_buf(pstr_it_bit_buff, pstr_uni_drc_gain_ext->ext_bit_size[cnt]);
    cnt++;

    pstr_uni_drc_gain_ext->uni_drc_gain_ext_type[cnt] =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_read_uni_drc_gain
 *
 *  \brief Parse uni drc gain
 *
 *  \param [out] pstr_uni_drc_gain  Pointer to uni drc gain struct
 *  \param [in,out] pstr_drc_config  Pointer to drc config struct
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  pstr_drc_uni_bs_dec   Pointer to uni bs dec struct
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_read_uni_drc_gain(ia_drc_gain_struct *pstr_uni_drc_gain,
                                        ia_drc_config *pstr_drc_config,
                                        ia_bit_buf_struct *pstr_it_bit_buff,
                                        ia_drc_bits_dec_struct *pstr_drc_uni_bs_dec)
{
  ia_drc_spline_nodes_struct *pstr_spline_nodes = {0};
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 band_cnt;
  WORD32 cnt;
  WORD32 gain_set_cnt = pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].gain_set_count;
  static WORD32 pkt_loss_frm_cnt = 0;

  for (cnt = 0; cnt < gain_set_cnt; cnt++)
  {
    ia_drc_gain_set_params_struct *pstr_gain_set_params =
        &(pstr_drc_config->str_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt]);
    if (pstr_gain_set_params->gain_coding_profile != IA_DRC_GAIN_CODING_PROFILE_CONSTANT)
    {
      band_cnt =
          pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].gain_set_params[cnt].band_count;
      pstr_uni_drc_gain->drc_gain_sequence[cnt].band_count = band_cnt;
      err_code = impd_drc_read_drc_gain_sequence(&(pstr_uni_drc_gain->drc_gain_sequence[cnt]),
                                                 pstr_it_bit_buff, pstr_drc_uni_bs_dec,
                                                 pstr_gain_set_params);
      if (err_code)
        return (err_code);
    }
    else
    {
      pstr_uni_drc_gain->drc_gain_sequence[cnt].band_count = 1;
      pstr_spline_nodes = &(pstr_uni_drc_gain->drc_gain_sequence[cnt].str_spline_nodes[0]);
      pstr_spline_nodes->str_node[0].loc_gain_db = 0;
      pstr_spline_nodes->str_node[0].slope = 0;
      pstr_spline_nodes->str_node[0].time =
          (pstr_drc_uni_bs_dec->ia_drc_params_struct).drc_frame_size - 1;
      pstr_spline_nodes->no_of_nodes = 1;
    }
  }

  if (pstr_it_bit_buff->ptr_bit_buf_base != NULL)
  {
    pstr_uni_drc_gain->uni_drc_gain_ext_flag = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    if (pstr_uni_drc_gain->uni_drc_gain_ext_flag == 1)
    {
      err_code = impd_drc_read_uni_drc_gain_ext(&(pstr_uni_drc_gain->uni_drc_gain_ext),
                                                pstr_it_bit_buff);
      if (err_code)
        return (err_code);
    }
    pstr_drc_config->apply_drc = 1;
    pkt_loss_frm_cnt = 0;
  }
  else
  {
    pkt_loss_frm_cnt++;

    if (pkt_loss_frm_cnt * (FLOAT32)pstr_drc_uni_bs_dec->ia_drc_params_struct.drc_frame_size /
            pstr_drc_config->sampling_rate >
        MAXPACKETLOSSTIME)
    {
      pstr_drc_config->apply_drc = 0;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_read_spline_nodes
 *
 *  \brief Parse spline node
 *
 *  \param [out] pstr_spline_nodes  Pointer to spline node struct
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  pstr_drc_uni_bs_dec   Pointer to bits dec struct
 *  \param [in]  pstr_gain_set_params   Pointer to gain set params struct
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE
impd_drc_read_spline_nodes(ia_drc_spline_nodes_struct *pstr_spline_nodes,
                           ia_bit_buf_struct *pstr_it_bit_buff,
                           ia_drc_bits_dec_struct *pstr_drc_uni_bs_dec,
                           ia_drc_gain_set_params_struct *pstr_gain_set_params)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 time_off;

  if (pstr_gain_set_params->time_alignment != 0)
  {
    if (pstr_gain_set_params->time_delt_min_flag == 0)
    {
      time_off = -pstr_drc_uni_bs_dec->ia_drc_params_struct.delta_tmin_def +
                 (pstr_drc_uni_bs_dec->ia_drc_params_struct.delta_tmin_def - 1) / 2;
    }
    else
    {
      time_off = -pstr_gain_set_params->time_delt_min_val +
                 (pstr_gain_set_params->time_delt_min_val - 1) / 2;
    }
  }
  else
  {
    time_off = -1;
  }

  if (pstr_it_bit_buff->ptr_bit_buf_base != NULL)
  {
    if (pstr_it_bit_buff->cnt_bits < 1)
    {
      if ((pstr_spline_nodes->no_of_nodes < 1) ||
          (pstr_spline_nodes->no_of_nodes > NODE_COUNT_MAX))
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;

      pstr_spline_nodes->str_node[0].slope = 0;
      pstr_spline_nodes->str_node[0].time =
          (pstr_drc_uni_bs_dec->ia_drc_params_struct).drc_frame_size + time_off;
      pstr_spline_nodes->str_node[0].loc_gain_db =
          pstr_spline_nodes->str_node[pstr_spline_nodes->no_of_nodes - 1].loc_gain_db;
      pstr_spline_nodes->drc_gain_coding_mode = 0;
      pstr_spline_nodes->no_of_nodes = 1;
    }
    else
    {
      pstr_spline_nodes->drc_gain_coding_mode = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    }
    if (pstr_spline_nodes->drc_gain_coding_mode != 0)
    {
      err_code = impd_drc_decode_slopes(pstr_spline_nodes->str_node, pstr_it_bit_buff,
                                        &pstr_spline_nodes->no_of_nodes,
                                        pstr_gain_set_params->gain_interp_type);
      if (err_code)
        return (err_code);
      if (pstr_gain_set_params->time_delt_min_flag == 0)
      {
        err_code = impd_drc_decode_times(
            pstr_spline_nodes->str_node, pstr_it_bit_buff, &pstr_drc_uni_bs_dec->tables_default,
            (pstr_drc_uni_bs_dec->ia_drc_params_struct).delta_tmin_def,
            (pstr_drc_uni_bs_dec->ia_drc_params_struct).drc_frame_size,
            pstr_gain_set_params->full_frame, pstr_spline_nodes->no_of_nodes, time_off);
        if (err_code)
          return (err_code);
        err_code = impd_drc_decode_gains(pstr_spline_nodes->str_node, pstr_it_bit_buff,
                                         pstr_spline_nodes->no_of_nodes,
                                         pstr_gain_set_params->gain_coding_profile);
        if (err_code)
          return (err_code);
      }
      else
      {
        err_code = impd_drc_decode_times(
            pstr_spline_nodes->str_node, pstr_it_bit_buff, &pstr_gain_set_params->str_tables,
            pstr_gain_set_params->time_delt_min_val,
            (pstr_drc_uni_bs_dec->ia_drc_params_struct).drc_frame_size,
            pstr_gain_set_params->full_frame, pstr_spline_nodes->no_of_nodes, time_off);
        if (err_code)
          return (err_code);
        err_code = impd_drc_decode_gains(pstr_spline_nodes->str_node, pstr_it_bit_buff,
                                         pstr_spline_nodes->no_of_nodes,
                                         pstr_gain_set_params->gain_coding_profile);
        if (err_code)
          return (err_code);
      }
    }
    else
    {
      err_code =
          impd_drc_get_initial_gain(&(pstr_spline_nodes->str_node[0].loc_gain_db),
                                    pstr_it_bit_buff, pstr_gain_set_params->gain_coding_profile);

      if (err_code)
        return (err_code);

      pstr_spline_nodes->str_node[0].slope = 0;
      pstr_spline_nodes->str_node[0].time =
          (pstr_drc_uni_bs_dec->ia_drc_params_struct).drc_frame_size + time_off;
      pstr_spline_nodes->no_of_nodes = 1;
    }
  }
  else
  {
    if ((pstr_spline_nodes->no_of_nodes < 1) || (pstr_spline_nodes->no_of_nodes > NODE_COUNT_MAX))
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;

    FLOAT32 prev_gain_db =
        pstr_spline_nodes->str_node[pstr_spline_nodes->no_of_nodes - 1].loc_gain_db;

    if (prev_gain_db < 0)
    {
      pstr_spline_nodes->str_node[0].loc_gain_db = prev_gain_db;
    }
    else
    {
      pstr_spline_nodes->str_node[0].loc_gain_db = 0;
    }

    pstr_spline_nodes->str_node[0].slope = 0;
    pstr_spline_nodes->str_node[0].time =
        (pstr_drc_uni_bs_dec->ia_drc_params_struct).drc_frame_size + time_off;
    pstr_spline_nodes->drc_gain_coding_mode = 0;
    pstr_spline_nodes->no_of_nodes = 1;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impd_drc_read_drc_gain_sequence
 *
 *  \brief Parse drc gain sequence
 *
 *  \param [out] pstr_drc_gain_sequence  Pointer to drc gain sequence struct
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer
 *  \param [in]  pstr_drc_uni_bs_dec   Pointer to bits dec struct
 *  \param [in]  pstr_gain_set_params   Pointer to gain set params struct
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_read_drc_gain_sequence(ia_drc_gain_sequence_struct *pstr_drc_gain_sequence,
                                             ia_bit_buf_struct *pstr_it_bit_buff,
                                             ia_drc_bits_dec_struct *pstr_drc_uni_bs_dec,
                                             ia_drc_gain_set_params_struct *pstr_gain_set_params)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 band;
  WORD32 node, cur_node;
  WORD32 node_reservoir_nodes = 0, cur_nodes = 0;
  WORD32 time_buf_prev_frame[NODE_COUNT_MAX], time_buf_cur_frame[NODE_COUNT_MAX];

  if (((pstr_drc_uni_bs_dec->ia_drc_params_struct).delay_mode == DELAY_MODE_LOW) &&
      (pstr_gain_set_params->full_frame == 0))
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_PARAM_ERROR);
  }
  for (band = 0; band < pstr_gain_set_params->band_count; band++)
  {
    err_code =
        impd_drc_read_spline_nodes(&(pstr_drc_gain_sequence->str_spline_nodes[band]),
                                   pstr_it_bit_buff, pstr_drc_uni_bs_dec, pstr_gain_set_params);
    if (err_code)
      return (err_code);

    node_reservoir_nodes = 0;
    cur_nodes = 0;

    for (node = 0; node < pstr_drc_gain_sequence->str_spline_nodes[band].no_of_nodes; node++)
    {
      if (pstr_drc_gain_sequence->str_spline_nodes[band].str_node[node].time <
          pstr_drc_uni_bs_dec->ia_drc_params_struct.drc_frame_size)
      {
        time_buf_cur_frame[cur_nodes] =
            pstr_drc_gain_sequence->str_spline_nodes[band].str_node[node].time;
        cur_nodes++;
      }
      else
      {
        time_buf_prev_frame[node_reservoir_nodes] =
            pstr_drc_gain_sequence->str_spline_nodes[band].str_node[node].time;
        node_reservoir_nodes++;
      }
    }
    for (node = 0; node < node_reservoir_nodes; node++)
    {
      WORD32 time_value = time_buf_prev_frame[node] -
                          2 * pstr_drc_uni_bs_dec->ia_drc_params_struct.drc_frame_size;
      if (time_value >= (2 * AUDIO_CODEC_FRAME_SIZE_MAX -
                         pstr_drc_uni_bs_dec->ia_drc_params_struct.drc_frame_size))
      {
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      }
      pstr_drc_gain_sequence->str_spline_nodes[band].str_node[node].time = time_value;
    }
    for (cur_node = 0; cur_node < cur_nodes; cur_node++, node++)
    {
      pstr_drc_gain_sequence->str_spline_nodes[band].str_node[node].time =
          time_buf_cur_frame[cur_node];
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of DRCFrameParse */
