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
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>

#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"

#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_windows.h"
#include "ia_core_coder_vec_baisc_ops.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"

#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_func_def.h"
#include "ia_core_coder_igf_dec.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_get_mode_lpc
 *
 *  \brief Get LPC mode.
 *
 *  \param [in] lpc_set     LPC coeffecient set index value.
 *  \param [in] ptr_bit_buf Pointer to bit buffer structure.
 *  \param [in] ptr_nk_mode Pointer to code book nk mode.
 *
 *  \return WORD32          LPC mode value.
 *
 */
static WORD32 ia_core_coder_get_mode_lpc(WORD32 lpc_set, ia_bit_buf_struct *ptr_bit_buf,
                                         WORD32 *ptr_nk_mode)
{
  WORD32 mode_lpc = 0;
  switch (lpc_set)
  {
  case 0:
  case 2:
    mode_lpc = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    if (mode_lpc == 1)
    {
      *ptr_nk_mode = 3;
    }
    break;
  case 1:
    if (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 0)
    {
      mode_lpc = *ptr_nk_mode = 2;
    }
    else
    {
      if (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 0)
        mode_lpc = *ptr_nk_mode = 0;
      else
        mode_lpc = *ptr_nk_mode = 1;
    }
    break;
  case 3:
    if (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 0)
    {
      mode_lpc = *ptr_nk_mode = 1;
    }
    else
    {
      if (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 0)
      {
        mode_lpc = *ptr_nk_mode = 0;
      }
      else
      {
        mode_lpc = (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 0) ? 2 : 3;
        *ptr_nk_mode = 2;
      }
    }
    break;
  case 4:
    mode_lpc = 0;
    break;
  }
  return mode_lpc;
}

/**
 *  ia_core_coder_qn_data
 *
 *  \brief Extraction of codebook number data.
 *
 *  \param [in]  nk_mode       Codebook nk mode.
 *  \param [out] ptr_qn        Pointer to codebook number.
 *  \param [in]  ptr_bit_buf   Pointer to bit buffer structure.
 *  \param [in]  full_band_lpd Flag that indicates full band LPD.
 *
 *
 *
 */
VOID ia_core_coder_qn_data(WORD32 nk_mode, WORD32 *ptr_qn, ia_bit_buf_struct *ptr_bit_buf,
                           WORD32 full_band_lpd)
{
  WORD32 idx;
  switch (nk_mode)
  {
  case 1:
    for (idx = 0; idx < 2; idx++)
    {
      while (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 1)
      {
        ptr_qn[idx] += 1;
      }
      if (ptr_qn[idx] > 0)
      {
        ptr_qn[idx] += 1;
      }
    }
    break;
  case 0:
  case 2:
  case 3:
    for (idx = 0; idx < 2; idx++)
    {
      ptr_qn[idx] = 2 + ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
    }
    if ((nk_mode == 2) && !full_band_lpd)
    {
      for (idx = 0; idx < 2; idx++)
      {
        if (ptr_qn[idx] > 4)
        {
          ptr_qn[idx] = 0;
          while (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 1)
          {
            ptr_qn[idx] += 1;
          }
          if (ptr_qn[idx] > 0)
          {
            ptr_qn[idx] += 4;
          }
        }
      }
    }
    else
    {
      for (idx = 0; idx < 2; idx++)
      {
        if (ptr_qn[idx] > 4)
        {
          WORD32 qn_ext = 0;
          while (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 1)
          {
            qn_ext += 1;
          }

          switch (qn_ext)
          {
          case 0:
            ptr_qn[idx] = 5;
            break;
          case 1:
            ptr_qn[idx] = 6;
            break;
          case 2:
            ptr_qn[idx] = 0;
            break;
          default:
            ptr_qn[idx] = qn_ext + 4;
            break;
          }
        }
      }
    }
    break;
  }
  return;
}

/**
 *  ia_core_coder_code_book_indices
 *
 *  \brief Compute code book indices.
 *
 *  \param [in,out] pstr_td_frame_data Pointer to time domain frame data.
 *  \param [in]     nk_mode            Code book nk mode.
 *  \param [in]     ptr_pos            Pointer to position parameter.
 *  \param [in]     ptr_bit_buf        Pointer to bit buffer sturcture.
 *  \param [in]     full_band_lpd      Flag that indicates full band LPD.
 *
 *
 *
 */
static VOID ia_core_coder_code_book_indices(ia_td_frame_data_struct *pstr_td_frame_data,
                                            WORD32 nk_mode, WORD32 *ptr_pos,
                                            ia_bit_buf_struct *ptr_bit_buf, WORD32 full_band_lpd)
{
  WORD32 idx1, idx2, nk, n;
  WORD32 qn[2] = {0, 0};

  ia_core_coder_qn_data(nk_mode, &qn[0], ptr_bit_buf, full_band_lpd);

  pstr_td_frame_data->lpc_first_approx_idx[(*ptr_pos)++] = qn[0];
  pstr_td_frame_data->lpc_first_approx_idx[(*ptr_pos)++] = qn[1];

  for (idx1 = 0; idx1 < 2; idx1++)
  {
    if (qn[idx1] > 0)
    {
      if (qn[idx1] > 4)
      {
        nk = (qn[idx1] - 3) >> 1;
        n = qn[idx1] - nk * 2;
      }
      else
      {
        nk = 0;
        n = qn[idx1];
      }
      pstr_td_frame_data->lpc_first_approx_idx[(*ptr_pos)++] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, 4 * n);

      for (idx2 = 0; idx2 < 8; idx2++)
      {
        pstr_td_frame_data->lpc_first_approx_idx[(*ptr_pos)++] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, nk);
      }
    }
  }
  return;
}

/**
 *  ia_core_coder_lpc_data
 *
 *  \brief Extracts all LPC filter parameter sets required to decode the current superframe.
 *
 *  \param [in]     first_lpd_flag     Flag to indicate if the current superframe is first of a
 * sequence of superframes.
 *  \param [in]     ptr_mod            Pointer to mode of the previously decoded ACELP frame or
 * TCX frame.
 *  \param [in,out] pstr_td_frame_data Pointer to time domain frame data
 *  \param [in]     ptr_bit_buf        Pointer to bit buffer structure
 *  \param [in]     td_config          Pointer to time domain config structure
 *
 *
 *
 */
VOID ia_core_coder_lpc_data(WORD32 first_lpd_flag, WORD32 *ptr_mod,
                            ia_td_frame_data_struct *pstr_td_frame_data,
                            ia_bit_buf_struct *ptr_bit_buf, ia_usac_td_config_handle td_config)
{
  WORD32 mode_lpc;
  WORD32 nk_mode = 0;
  WORD32 idx = 0;

  pstr_td_frame_data->lpc_first_approx_idx[idx++] = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
  ia_core_coder_code_book_indices(pstr_td_frame_data, nk_mode, &idx, ptr_bit_buf, 0);
  if (first_lpd_flag)
  {
    mode_lpc = ia_core_coder_get_mode_lpc(0, ptr_bit_buf, &nk_mode);
    pstr_td_frame_data->lpc_first_approx_idx[idx++] = mode_lpc;

    if (mode_lpc == 0)
    {
      pstr_td_frame_data->lpc_first_approx_idx[idx++] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    }

    ia_core_coder_code_book_indices(pstr_td_frame_data, nk_mode, &idx, ptr_bit_buf, 0);
  }
  if (td_config->num_frame == 4 && ptr_mod[0] < 3)
  {
    mode_lpc = ia_core_coder_get_mode_lpc(2, ptr_bit_buf, &nk_mode);
    pstr_td_frame_data->lpc_first_approx_idx[idx++] = mode_lpc;

    if (mode_lpc == 0)
    {
      pstr_td_frame_data->lpc_first_approx_idx[idx++] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    }

    ia_core_coder_code_book_indices(pstr_td_frame_data, nk_mode, &idx, ptr_bit_buf, 0);
  }

  if (ptr_mod[0] < 2)
  {
    mode_lpc = ia_core_coder_get_mode_lpc(1, ptr_bit_buf, &nk_mode);
    pstr_td_frame_data->lpc_first_approx_idx[idx++] = mode_lpc;

    if (mode_lpc == 0)
    {
      pstr_td_frame_data->lpc_first_approx_idx[idx++] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    }

    if (mode_lpc != 1)
    {
      ia_core_coder_code_book_indices(pstr_td_frame_data, nk_mode, &idx, ptr_bit_buf,
                                      td_config->full_band_lpd);
    }
  }
  if (td_config->num_frame != 2 && ptr_mod[2] < 2)
  {
    mode_lpc = ia_core_coder_get_mode_lpc(3, ptr_bit_buf, &nk_mode);
    pstr_td_frame_data->lpc_first_approx_idx[idx++] = mode_lpc;

    if (mode_lpc == 0)
    {
      pstr_td_frame_data->lpc_first_approx_idx[idx++] =
          ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    }

    ia_core_coder_code_book_indices(pstr_td_frame_data, nk_mode, &idx, ptr_bit_buf, 0);
  }
  return;
}

/**
 *  ia_core_coder_fac_decoding
 *
 *  \brief Extracts data for the forward aliaising cancellation (FAC) tool.
 *
 *  \param [in]  fac_length  Length of FAC transform.
 *  \param [in]  k           Index value.
 *  \param [out] ptr_fac_prm Pointer to FAC parameter array.
 *  \param [in]  ptr_bit_buf Pointer to bit buffer structure.
 *
 *
 *
 */
VOID ia_core_coder_fac_decoding(WORD32 fac_length, WORD32 k, WORD32 *ptr_fac_prm,
                                ia_bit_buf_struct *ptr_bit_buf)
{
  WORD32 idx1, idx2, n, qn, nk, kv[8];
  long code_book_index;

  for (idx1 = 0; idx1 < fac_length; idx1 += 8)
  {
    qn = 0;
    while (ia_core_coder_read_bits_buf(ptr_bit_buf, 1) == 1)
    {
      qn += 1;
    }
    if (qn != 0)
    {
      qn += 1;
    }

    nk = 0;
    n = qn;
    if (qn > 4)
    {
      nk = (qn - 3) >> 1;
      n = qn - nk * 2;
    }

    code_book_index = ia_core_coder_read_bits_buf(ptr_bit_buf, 4 * n);

    for (idx2 = 0; idx2 < 8; idx2++)
    {
      kv[idx2] = ia_core_coder_read_bits_buf(ptr_bit_buf, nk);
    }

    ia_core_coder_rotated_gosset_mtx_dec(qn, code_book_index, kv,
                                         &ptr_fac_prm[k * FAC_LENGTH + idx1]);
  }
  return;
}

/**
 *  ia_core_coder_acelp_decoding
 *
 *  \brief Extracts all data to decode one frame of ACELP excitation.
 *
 *  \param [in]     k                  Index Parameter.
 *  \param [in,out] ptr_usac_data      Pointer to USAC data structure.
 *  \param [in,out] pstr_td_frame_data Pointer to TD frame data.
 *  \param [in]     ptr_bit_buf        Pointer to bit buffer structure.
 *  \param [in]     chan               Channel index.
 *  \param [in]     full_band_lpd      Flag that indicates full band LPD.
 *
 *
 *
 */
VOID ia_core_coder_acelp_decoding(WORD32 k, ia_usac_data_struct *ptr_usac_data,
                                  ia_td_frame_data_struct *pstr_td_frame_data,
                                  ia_bit_buf_struct *ptr_bit_buf, WORD32 chan,
                                  WORD32 full_band_lpd)
{
  WORD32 sfr, kk, idx;
  WORD32 nb_subfr = ptr_usac_data->num_subfrm;
  const UWORD8 *ptr_num_bits =
      &ia_core_coder_num_bits_acelp_coding[pstr_td_frame_data->acelp_core_mode][0];
  ia_usac_lpd_decoder_handle td_decoder = ptr_usac_data->str_tddec[chan];

  pstr_td_frame_data->mean_energy[k] = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);

  for (sfr = 0; sfr < nb_subfr; sfr++)
  {
    kk = k * 4 + sfr;

    if (((nb_subfr == 4) && (sfr == 2)) || (sfr == 0))
    {
      pstr_td_frame_data->acb_index[kk] = ia_core_coder_read_bits_buf(ptr_bit_buf, 9);
    }
    else
    {
      pstr_td_frame_data->acb_index[kk] = ia_core_coder_read_bits_buf(ptr_bit_buf, 6);
    }

    pstr_td_frame_data->ltp_filtering_flag[kk] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

    if (pstr_td_frame_data->acelp_core_mode != 5)
    {
      for (idx = 0; idx < 4; idx++)
      {
        pstr_td_frame_data->icb_index[kk][idx] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, ptr_num_bits[idx]);
      }
    }
    else
    {
      for (idx = 0; idx < 4; idx++)
      {
        pstr_td_frame_data->icb_index[kk][idx] = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
      }
      for (idx = 4; idx < 8; idx++)
      {
        pstr_td_frame_data->icb_index[kk][idx] = ia_core_coder_read_bits_buf(ptr_bit_buf, 14);
      }
    }

    pstr_td_frame_data->gains[kk] = ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
  }

  if (full_band_lpd)
  {
    WORD32 tbe_frm_class = (k == 0) ? 0 : 1;
    ia_core_coder_tbe_data(&td_decoder->tbe_dec_data, ptr_bit_buf, tbe_frm_class);
  }
  return;
}

/**
 *  ia_core_coder_tcx_coding
 *
 *  \brief Extracts all data to decode one frame of MDCT based transform codec excitation (TCX)
 *
 *  \param [in,out] ptr_usac_data      Pointer to USAC data structure.
 *  \param [in]     ptr_quant          Pointer to quantization data.
 *  \param [in]     k                  Index value.
 *  \param [in]     first_tcx_flag     Flag to indicate current TCX frame is the first in the
 * superframe.
 *  \param [in,out] pstr_td_frame_data Pointer to TD frame data.
 *  \param [in]     ptr_bit_buf        Pointer to bit buffer structure.
 *  \param [in]     chan               Channel index.
 *  \param [in]     elem_idx           Element index.
 *
 *  \return IA_ERRORCODE               Processing error if any.
 *
 */
IA_ERRORCODE ia_core_coder_tcx_coding(ia_usac_data_struct *ptr_usac_data, pWORD32 ptr_quant,
                                      WORD32 k, WORD32 first_tcx_flag,
                                      ia_td_frame_data_struct *pstr_td_frame_data,
                                      ia_bit_buf_struct *ptr_bit_buf, WORD32 chan,
                                      WORD32 elem_idx)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  ia_usac_lpd_decoder_handle td_decoder = ptr_usac_data->str_tddec[chan];
  ia_usac_td_config_handle td_config = &ptr_usac_data->td_config[elem_idx];
  if (ptr_usac_data->noise_filling_config[elem_idx])
  {
    pstr_td_frame_data->noise_factor[k] = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
  }
  else
  {
    pstr_td_frame_data->noise_factor[k] = 8;
  }

  pstr_td_frame_data->global_gain[k] = ia_core_coder_read_bits_buf(ptr_bit_buf, 7);

  switch (pstr_td_frame_data->mod[k])
  {
  case 1:
    pstr_td_frame_data->tcx_lg[k] = ptr_usac_data->len_subfrm;
    break;
  case 2:
    pstr_td_frame_data->tcx_lg[k] = 2 * (ptr_usac_data->len_subfrm);
    break;
  case 3:
    pstr_td_frame_data->tcx_lg[k] = 4 * (ptr_usac_data->len_subfrm);
    break;
  }

  pstr_td_frame_data->tcx_lg[k] *= td_config->fac_fb;

  if ((pstr_td_frame_data->mod[k] == 3 && !td_config->full_band_lpd) ||
      (pstr_td_frame_data->mod[k] == 2 && td_config->full_band_lpd))
  {
    td_decoder->ltpf_data.data_present = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    if (td_decoder->ltpf_data.data_present == 1)
    {
      if (1 == ptr_usac_data->is_base_line_profile_3b)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_PROFILE_CONFIG;
      }
      td_decoder->ltpf_data.pitch_lag_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 9);
      td_decoder->ltpf_data.gain_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
    }
    else
    {
      td_decoder->ltpf_data.data_present = 0;
    }
  }
  else
  {
    td_decoder->ltpf_data.data_present = 0;
  }

  if (((pstr_td_frame_data->mod[k] == 2 && td_config->full_band_lpd) ||
       (pstr_td_frame_data->mod[k] == 3 && !td_config->full_band_lpd)) &&
      !ptr_usac_data->usac_independency_flg)
  {
    td_decoder->fdp_data_present = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    if (td_decoder->fdp_data_present)
    {
      if (1 == ptr_usac_data->is_base_line_profile_3b)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_PROFILE_CONFIG;
      }
      td_decoder->fdp_spacing_index = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
    }
  }
  else
  {
    td_decoder->fdp_data_present = 0;
  }

  if (td_config->igf_active)
  {
    WORD32 num_windows = 1;
    WORD32 mode = pstr_td_frame_data->mod[k];
    WORD8 igf_win_type = (WORD8)(mode + 1);
    WORD32 igf_frm_id = (3 == igf_win_type) ? 4 : (mode + 1 + k);
    WORD32 chn = ptr_usac_data->present_chan;
    ia_usac_igf_config_struct *igf_config = &ptr_usac_data->igf_config[elem_idx];
    WORD32 igf_all_zero = ia_core_coder_get_igf_levels(
        ptr_bit_buf, ptr_usac_data, igf_config, num_windows, chn, igf_frm_id, igf_win_type);

    if (igf_all_zero == -1)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_IGF_ALL_ZERO;
    }

    if (!igf_all_zero)
    {
      ia_core_coder_get_igf_data(ptr_bit_buf, ptr_usac_data, igf_config, chn, igf_frm_id,
                                 igf_win_type);
    }
  }
  if (td_decoder->max_sfb < 0)
  {
    return IA_MPEGH_DEC_EXE_FATAL_INVALID_MAX_SFB;
  }
  if (td_decoder->tns_data_present)
  {
    err = ia_core_coder_read_tns_tcx(td_decoder->max_sfb, &td_decoder->str_tns_info_td,
                                     ptr_bit_buf, k);
    if (err)
    {
      return err;
    }
  }
  if (first_tcx_flag)
  {
    pstr_td_frame_data->arith_reset_flag =
        (ptr_usac_data->usac_independency_flg) ? 1 : ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }

  err = ia_core_coder_arith_data(pstr_td_frame_data, ptr_quant, ptr_usac_data, ptr_bit_buf,
                                 (first_tcx_flag), k);
  return err;
}

/**
 *  ia_core_coder_lpd_channel_stream
 *
 *  \brief Extracts all necessary information to decode one frame of linear prediction domain
 * coded signal.
 *
 *  \param [in,out] ptr_usac_data      Pointer to USAC data structure.
 *  \param [in,out] pstr_td_frame_data Pointer to TD frame data structure.
 *  \param [in]     ptr_bit_buf        Pointer to bit buffer structure.
 *  \param [in,out] ptr_synth          Pointer to synthesis buffer.
 *  \param [in]     elem_idx           Element index.
 *
 *  \return IA_ERRORCODE                     Processing error if any.
 *
 */
IA_ERRORCODE ia_core_coder_lpd_channel_stream(ia_usac_data_struct *ptr_usac_data,
                                              ia_td_frame_data_struct *pstr_td_frame_data,
                                              ia_bit_buf_struct *ptr_bit_buf, FLOAT32 *ptr_synth,
                                              WORD32 elem_idx)
{
  WORD32 lpd_mode, idx, cnt, ii;
  WORD32 first_tcx_flag;
  WORD32 *ptr_quant;
  WORD32 core_mode_last, fac_data_present;
  WORD32 *ptr_fac_data;
  WORD32 first_lpd_flag;
  WORD32 short_fac_flag = 0;
  WORD32 bpf_control_info;
  WORD32 chan = ptr_usac_data->present_chan;
  WORD32 last_lpd_mode = ptr_usac_data->str_tddec[chan]->mode_prev;
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  ia_usac_lpd_decoder_handle p_lpd_dec = ptr_usac_data->str_tddec[chan];

  if (ptr_usac_data->td_config[elem_idx].full_band_lpd)
  {
    p_lpd_dec->tns_data_present = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

    if (p_lpd_dec->tns_data_present || ptr_usac_data->noise_filling_config[elem_idx])
    {
      p_lpd_dec->window_shape = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      p_lpd_dec->max_sfb = ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
    }
    else
    {
      p_lpd_dec->window_shape = p_lpd_dec->max_sfb = 0;
    }
    pstr_td_frame_data->acelp_core_mode = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
    lpd_mode = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);

    /*If fullbandLpd == 1, then lpd_mode shall have a
      value between and including 0 and 4.*/

    if (lpd_mode > 4)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_LPD_MODE;
    }

    if (lpd_mode == 4)
    {
      pstr_td_frame_data->mod[0] = pstr_td_frame_data->mod[1] = 2;
    }
    else
    {
      pstr_td_frame_data->mod[0] = lpd_mode & 1;
      pstr_td_frame_data->mod[1] = (lpd_mode >> 1) & 1;
    }
    bpf_control_info = 1;
  }
  else
  {
    p_lpd_dec->tns_data_present = 0;
    pstr_td_frame_data->acelp_core_mode = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
    lpd_mode = ia_core_coder_read_bits_buf(ptr_bit_buf, 5);

    /*if fullbandLpd == 0, then lpd_mode shall have a
      value between and including 0 and 25.*/

    if (lpd_mode > 25)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_LPD_MODE;
    }

    if (lpd_mode == 25)
    {
      pstr_td_frame_data->mod[0] = pstr_td_frame_data->mod[1] = pstr_td_frame_data->mod[2] =
          pstr_td_frame_data->mod[3] = 3;
    }
    else if (lpd_mode == 24)
    {
      pstr_td_frame_data->mod[0] = pstr_td_frame_data->mod[1] = pstr_td_frame_data->mod[2] =
          pstr_td_frame_data->mod[3] = 2;
    }
    else
    {
      if (lpd_mode >= 20)
      {
        pstr_td_frame_data->mod[0] = lpd_mode & 1;
        pstr_td_frame_data->mod[1] = (lpd_mode >> 1) & 1;
        pstr_td_frame_data->mod[2] = pstr_td_frame_data->mod[3] = 2;
      }
      else if (lpd_mode >= 16)
      {
        pstr_td_frame_data->mod[0] = pstr_td_frame_data->mod[1] = 2;
        pstr_td_frame_data->mod[2] = lpd_mode & 1;
        pstr_td_frame_data->mod[3] = (lpd_mode >> 1) & 1;
      }
      else
      {
        for (idx = 0; idx < 4; idx++)
        {
          pstr_td_frame_data->mod[idx] = (lpd_mode >> idx) & 1;
        }
      }
    }

    bpf_control_info = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }
  core_mode_last = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  fac_data_present = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  /*fac_data_present shall be 0, if mod[0] of the current frame is > 0 and the core_mode of the
   * preceding frame of the same channel was 0 ("TCX follows FD")*/

  if (((pstr_td_frame_data->mod[0] > 0) && (core_mode_last == 0)) && (fac_data_present != 0))
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_FAC_DATA_PRES;
  }

  first_lpd_flag = (core_mode_last == 0) ? 1 : 0;

  ptr_quant = pstr_td_frame_data->x_tcx_invquant;
  first_tcx_flag = 1;
  idx = 0;
  while (idx < ptr_usac_data->td_config[elem_idx].num_frame)
  {
    if (idx != 0)
    {
      if (((last_lpd_mode > 0) && (pstr_td_frame_data->mod[idx] == 0)) ||
          ((last_lpd_mode == 0) && (pstr_td_frame_data->mod[idx] > 0)))
      {
        ia_core_coder_fac_decoding(((ptr_usac_data->len_subfrm) >> 1), idx,
                                   pstr_td_frame_data->fac, ptr_bit_buf);
      }
    }
    else
    {
      if ((core_mode_last == 1) && (fac_data_present == 1))
      {
        ia_core_coder_fac_decoding(((ptr_usac_data->len_subfrm) >> 1), idx,
                                   pstr_td_frame_data->fac, ptr_bit_buf);
      }
    }

    if (pstr_td_frame_data->mod[idx] != 0)
    {
      err = ia_core_coder_tcx_coding(ptr_usac_data, ptr_quant, idx, first_tcx_flag,
                                     pstr_td_frame_data, ptr_bit_buf, chan, elem_idx);
      if (err)
      {
        return err;
      }
      last_lpd_mode = pstr_td_frame_data->mod[idx];
      ptr_quant += pstr_td_frame_data->tcx_lg[idx];

      cnt = 1 << (pstr_td_frame_data->mod[idx] - 1);

      for (ii = 0; ii < cnt - 1; ii++)
      {
        pstr_td_frame_data->tcx_lg[idx + 1 + ii] = 0;
      }

      idx += cnt;
      first_tcx_flag = 0;
    }
    else
    {
      ia_core_coder_acelp_decoding(idx, ptr_usac_data, pstr_td_frame_data, ptr_bit_buf, chan,
                                   ptr_usac_data->td_config[elem_idx].full_band_lpd);
      last_lpd_mode = 0;
      pstr_td_frame_data->tcx_lg[idx] = 0;
      p_lpd_dec->ltpf_data.data_present = 0;
      p_lpd_dec->ltpf_data.gain_idx = 0;
      p_lpd_dec->ltpf_data.pitch_lag_idx = 0;
      idx += 1;
    }
  }
  ia_core_coder_lpc_data(first_lpd_flag, pstr_td_frame_data->mod, pstr_td_frame_data, ptr_bit_buf,
                         &ptr_usac_data->td_config[elem_idx]);

  if ((core_mode_last == 0) && (fac_data_present == 1))
  {
    WORD32 fac_length;
    short_fac_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

    /*shall be encoded with a value of 1 if the window_sequence of the previous frame was 2
    (EIGHT_SHORT_SEQUENCE). Otherwise short_fac_flag shall
    be encoded with a value of 0.*/
    if (ptr_usac_data->window_sequence_last[chan] == EIGHT_SHORT_SEQUENCE)
    {
      if (short_fac_flag != 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_SHORT_FAC_FLAG;
      }
    }
    else
    {
      if (short_fac_flag != 0)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_SHORT_FAC_FLAG;
      }
    }

    fac_length = (short_fac_flag) ? (ptr_usac_data->td_config[elem_idx].len_frame >> 4)
                                  : (ptr_usac_data->td_config[elem_idx].len_frame >> 3);

    ptr_fac_data = pstr_td_frame_data->fd_fac;
    ptr_fac_data[0] = ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
    ia_core_coder_fac_decoding(fac_length, 0, &ptr_fac_data[1], ptr_bit_buf);
  }

  if (ptr_usac_data->usac_first_frame_flag[chan] == 1)
  {
    ptr_usac_data->usac_first_frame_flag[chan] = 0;
    ptr_usac_data->str_tddec[chan]->window_shape_prev =
        ptr_usac_data->str_tddec[chan]->window_shape;
  }

  err = ia_core_coder_lpd_dec(ptr_usac_data, ptr_usac_data->str_tddec[chan], pstr_td_frame_data,
                              ptr_synth, first_lpd_flag, short_fac_flag, bpf_control_info,
                              elem_idx, chan);

  return err;
}

/**
*  ia_core_coder_tw_buff_update
*
*  \brief Time warping MDCT buffer updation.
*
*  \param [in,out] ptr_usac_data Pointer to USAC data structure.
*  \param [in]     i             Index value.
*  \param [in,out] ptr_st        Pointer to LPD decoder handle structure.
*
*  \return IA_ERRORCODE                Processing error if any.
*
*/
IA_ERRORCODE ia_core_coder_tw_buff_update(ia_usac_data_struct *ptr_usac_data, WORD32 i,
                                          ia_usac_lpd_decoder_handle ptr_st)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 td_frame_prev = ptr_usac_data->td_frame_prev[i];
  WORD32 window_sequence_last = ptr_usac_data->window_sequence_last[i];
  FLOAT32 *p_ioverlap = ptr_usac_data->overlap_data_ptr[i];
  if (!td_frame_prev)
  {
    err_code = ia_core_coder_reset_acelp_data(ptr_usac_data, ptr_st, ptr_usac_data->td_config,
                                              p_ioverlap,
                                              (window_sequence_last == EIGHT_SHORT_SEQUENCE), i);
  }
  return err_code;
}

/**
 *  ia_core_coder_td_frm_dec
 *
 *  \brief TD frame decoding step
 *
 *  \param [in,out] ptr_usac_data Pointer to USAC data structure.
 *  \param [in]     k             Index value.
 *  \param [in]     mod0          Mode value.
 *
 *
 *
 */
VOID ia_core_coder_td_frm_dec(ia_usac_data_struct *ptr_usac_data, WORD32 k, WORD32 mod0)
{
  WORD32 idx;
  WORD32 lfac = 0;
  WORD32 nlong = ptr_usac_data->ccfl;
  WORD32 nshort = nlong >> 3;
  WORD32 window_sequence_last = ptr_usac_data->window_sequence_last[k];
  WORD32 td_frame_prev = ptr_usac_data->td_frame_prev[k];
  FLOAT32 *p_out_idata = ptr_usac_data->time_sample_vector[k];
  FLOAT32 *p_ioverlap = ptr_usac_data->overlap_data_ptr[k];
  FLOAT32 *p_in_idata = p_out_idata;

  if (!td_frame_prev)
  {
    lfac = (EIGHT_SHORT_SEQUENCE != window_sequence_last) ? nshort : nshort >> 1;
    if (mod0 == 0)
    {
      ia_core_coder_memset(p_in_idata, (nlong >> 1) - lfac - LEN_SUBFR);
      ia_core_coder_memset(p_ioverlap + ((nlong >> 1) - lfac - (LEN_SUBFR)), lfac + LEN_SUBFR);
    }
    else if (mod0 > 0)
    {
      ia_core_coder_memset(p_in_idata, (nlong >> 1) - lfac);
    }
    ia_core_coder_memset(p_ioverlap + ((nlong >> 1) - lfac), (nlong >> 1) + lfac);
  }

  for (idx = 0; idx < nlong; idx++)
  {
    p_out_idata[idx] = ia_add_flt(p_ioverlap[idx], p_in_idata[idx]);
  }
  ia_core_coder_memset(p_ioverlap, nlong);
  return;
}
/** @} */ /* End of CoreDecProc */