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
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_func_def.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

#define LSF_GAP 50.0f
#define FREQ_MAX 6400.0f
#define FREQ_DIV 400.0f

static const FLOAT32 factor_table[4] = {60.0f, 65.0f, 64.0f, 63.0f};

/**
 *  ia_core_coder_decoding_avq_tool
 *
 *  \brief Performs avq decoding
 *
 *  \param [in]    read_arr  LPC approximaton index
 *  \param [in,out]  nvecq    Vector quantized values
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_decoding_avq_tool(WORD32 *read_arr, WORD32 *nvecq)
{
  WORD32 i, qn, code_book_idx, position = 2;
  WORD32 kv[8] = {0};
  WORD32 *ptr_kv;

  for (i = 0; i < 2; i++)
  {
    qn = read_arr[i];

    if (qn <= 0)
    {
      code_book_idx = 0;
      ptr_kv = &kv[0];
    }
    else
    {
      code_book_idx = read_arr[position++];

      ptr_kv = &read_arr[position];
      position += 8;
    }

    ia_core_coder_rotated_gosset_mtx_dec(qn, code_book_idx, ptr_kv, &nvecq[i * 8]);
  }

  return position;
}

/**
 *  ia_core_coder_avq_first_approx_abs
 *
 *  \brief gives avq first stage approximation of LSF based on wt table
 *
 *  \param [in,out]  lsf      LSF coefficients
 *  \param [in]    indx    LPC approximation index
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_avq_first_approx_abs(FLOAT32 *lsf, WORD32 *indx)
{
  WORD32 ord, position = 0, idx_0_mul_order;
  WORD32 avq[ORDER];
  FLOAT32 lsf_min;
  const FLOAT32 *ptr_w;

  idx_0_mul_order = (indx[0] * ORDER);
  ptr_w = &ia_core_coder_weight_table_avq[idx_0_mul_order];

  position++;

  ia_core_coder_mem_cpy(&ia_core_coder_dico_lsf_abs_8b_flt[idx_0_mul_order], lsf, ORDER);

  position += ia_core_coder_decoding_avq_tool(&indx[position], avq);

  lsf_min = LSF_GAP;
  for (ord = 0; ord < ORDER; ord++)
  {
    lsf[ord] = ia_add_flt(lsf[ord], ia_mul_flt(ptr_w[ord], (FLOAT32)avq[ord]));
    lsf[ord] = ia_max_flt(lsf[ord], lsf_min);
    lsf_min = ia_add_flt(lsf[ord], LSF_GAP);
  }

  return position;
}
/**
 *  ia_core_coder_avq_first_approx_rel
 *
 *  \brief gives avq first stage approximation of LSF coeff
 *
 *  \param [in,out]  lsf      LSF coefficients
 *  \param [in]    indx    LPC index
 *  \param [in]    mode    LPD mode
 *
 *  \return WORD32
 *
 */
WORD32 ia_core_coder_avq_first_approx_rel(FLOAT32 *lsf, WORD32 *indx, WORD32 mode)
{
  WORD32 i;
  FLOAT32 w[ORDER];
  FLOAT32 d[ORDER + 1];
  WORD32 avq[ORDER];
  WORD32 position = 0;
  FLOAT32 lsf_min;

  d[0] = lsf[0];
  d[ORDER] = ia_sub_flt(FREQ_MAX, lsf[ORDER - 1]);
  for (i = 1; i < ORDER; i++)
  {
    d[i] = ia_sub_flt(lsf[i], lsf[i - 1]);
  }

  for (i = 0; i < ORDER; i++)
  {
    w[i] = (FLOAT32)(factor_table[mode] / (FREQ_DIV / ia_sqrt_flt(ia_mul_flt(d[i], d[i + 1]))));
  }

  position = ia_core_coder_decoding_avq_tool(indx, avq);

  lsf_min = LSF_GAP;

  for (i = 0; i < ORDER; i++)
  {
    lsf[i] = ia_add_flt(lsf[i], ia_mul_flt(w[i], (FLOAT32)avq[i]));
    lsf[i] = ia_max_flt(lsf[i], lsf_min);
    lsf_min = ia_add_flt(lsf[i], LSF_GAP);
  }

  return position;
}

/**
 *  ia_core_coder_alg_vec_dequant
 *
 *  \brief Vector dequantization
 *
 *  \param [in,out]    pstr_td_frame_data    Time domain frame data structure
 *  \param [in]      first_lpd_flag      Flag to detect first LPD frame
 *  \param [in,out]    lsf            Line spectral
 * Frequency
 * coeffs
 *  \param [in]      mod            LPD mode
 *  \param [in]      num_div          Number of division in a
 * frame
 *
 *
 *
 */
VOID ia_core_coder_alg_vec_dequant(ia_td_frame_data_struct *pstr_td_frame_data,
                                   WORD32 first_lpd_flag, FLOAT32 *lsf, WORD32 *mod,
                                   WORD32 num_div)
{
  WORD32 i;
  WORD32 *lpc_index, mode_lpc, pos = 0;

  // lpc4
  lpc_index = pstr_td_frame_data->lpc_first_approx_idx;
  pos = ia_core_coder_avq_first_approx_abs(&lsf[num_div * ORDER], &lpc_index[0]);

  lpc_index += pos;

  if (first_lpd_flag)
  { // lpc0
    mode_lpc = lpc_index[0];
    lpc_index++;

    if (mode_lpc == 1)
    {
      ia_core_coder_mem_cpy(&lsf[num_div * ORDER], lsf, ORDER);
      pos = ia_core_coder_avq_first_approx_rel(&lsf[0], &lpc_index[0], 3);
    }
    else if (mode_lpc == 0)
    {
      pos = ia_core_coder_avq_first_approx_abs(&lsf[0], &lpc_index[0]);
    }

    lpc_index += pos;
  }

  if (mod[0] < 3 && num_div == 4)
  { // lpc2
    mode_lpc = lpc_index[0];
    lpc_index++;

    if (mode_lpc == 1)
    {
      ia_core_coder_mem_cpy(&lsf[4 * ORDER], &lsf[2 * ORDER], ORDER);
      pos = ia_core_coder_avq_first_approx_rel(&lsf[2 * ORDER], &lpc_index[0], 3);
    }
    else if (mode_lpc == 0)
    {
      pos = ia_core_coder_avq_first_approx_abs(&lsf[2 * ORDER], &lpc_index[0]);
    }

    lpc_index += pos;
  }

  if (mod[0] < 2)
  { // lpc1
    mode_lpc = lpc_index[0];
    lpc_index++;

    if (mode_lpc == 1)
    {
      for (i = 0; i < ORDER; i++)
        lsf[ORDER + i] = ia_mul_flt(0.5f, ia_add_flt(lsf[i], lsf[2 * ORDER + i]));
    }
    else
    {
      if (mode_lpc == 2)
      {
        ia_core_coder_mem_cpy(&lsf[2 * ORDER], &lsf[ORDER], ORDER);
        pos = ia_core_coder_avq_first_approx_rel(&lsf[ORDER], &lpc_index[0], 2);
      }
      else if (mode_lpc == 0)
      {
        pos = ia_core_coder_avq_first_approx_abs(&lsf[ORDER], &lpc_index[0]);
      }

      lpc_index += pos;
    }
  }

  if (mod[2] < 2 && num_div == 4)
  { // lpc3
    mode_lpc = lpc_index[0];
    lpc_index++;

    if (mode_lpc == 3)
    {
      ia_core_coder_mem_cpy(&lsf[4 * ORDER], &lsf[3 * ORDER], ORDER);
      pos = ia_core_coder_avq_first_approx_rel(&lsf[3 * ORDER], &lpc_index[0], 2);
    }
    else if (mode_lpc == 2)
    {
      ia_core_coder_mem_cpy(&lsf[2 * ORDER], &lsf[3 * ORDER], ORDER);
      pos = ia_core_coder_avq_first_approx_rel(&lsf[3 * ORDER], &lpc_index[0], 2);
    }
    else if (mode_lpc == 1)
    {
      for (i = 0; i < ORDER; i++)
        lsf[3 * ORDER + i] = ia_mul_flt(0.5f, ia_add_flt(lsf[2 * ORDER + i], lsf[4 * ORDER + i]));
      pos = ia_core_coder_avq_first_approx_rel(&lsf[3 * ORDER], &lpc_index[0], 1);
    }
    else if (mode_lpc == 0)
    {
      pos = ia_core_coder_avq_first_approx_abs(&lsf[3 * ORDER], &lpc_index[0]);
    }
    (VOID) pos;
  }
}
/** @} */ /* End of CoreDecProc */