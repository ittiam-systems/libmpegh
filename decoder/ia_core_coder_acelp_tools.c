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
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_info.h"
#include <ia_core_coder_constants.h>
#include <ia_core_coder_basic_ops32.h>
#include <ia_core_coder_basic_ops40.h>
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_acelp_com.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_func_def.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_preemphsis_tool
 *
 *  \brief Performs pre-emphasis filtering
 *
 *  \param [in,out] ptr_signal Pointer to input signal.
 *  \param [in]  mu     Only coeff on filter.
 *  \param [in]  length    Input length.
 *  \param [in]  mem    Filter states memory.
 *
 *
 *
 */
VOID ia_core_coder_preemphsis_tool(FLOAT32 *ptr_signal, FLOAT32 mu, WORD32 length, FLOAT32 mem)
{
  WORD32 idx;
  for (idx = length - 1; idx > 0; idx--)
  {
    ptr_signal[idx] -= ia_mul_flt(mu, ptr_signal[idx - 1]);
  }
  ptr_signal[0] -= ia_mul_flt(mu, mem);
  return;
}

/**
 *  ia_core_coder_deemphsis_tool
 *
 *  \brief Performs de-emphasis filtering
 *
 *  \param [in,out] ptr_signal Pointer to input ptr_signal.
 *  \param [in]  length    Input length.
 *  \param [in]  mem    Filter states memory.
 *
 *
 *
 */
VOID ia_core_coder_deemphsis_tool(FLOAT32 *ptr_signal, WORD32 length, FLOAT32 mem)
{
  WORD32 idx;
  ptr_signal[0] += ia_mul_flt(PREEMPH_FILT_FAC, mem);
  for (idx = 1; idx < length; idx++)
  {
    ptr_signal[idx] += ia_mul_flt(PREEMPH_FILT_FAC, ptr_signal[idx - 1]);
  }
  return;
}
/**
 *  ia_core_coder_lpc_wt_synthesis_tool
 *
 *  \brief Weighted synthesis filtering.
 *
 *  \param [in]  ptr_a Pointer to input signal.
 *  \param [out] ptr_x Pointer to ouptut signal.
 *  \param [in]  l Length of input.
 *
 *
 *
 */
VOID ia_core_coder_lpc_wt_synthesis_tool(FLOAT32 *ptr_a, FLOAT32 *ptr_x, WORD32 l)
{
  FLOAT32 s;
  WORD32 idx, ord;

  for (idx = 0; idx < l; idx++)
  {
    s = ptr_x[idx];
    for (ord = 1; ord <= ORDER; ord += 4)
    {
      s -= ia_mul_flt(ia_mul_flt(ptr_a[ord], ia_core_coder_gamma_table[ord]), ptr_x[idx - ord]);
      s -= ia_mul_flt(ia_mul_flt(ptr_a[ord + 1], ia_core_coder_gamma_table[ord + 1]),
                      ptr_x[idx - (ord + 1)]);
      s -= ia_mul_flt(ia_mul_flt(ptr_a[ord + 2], ia_core_coder_gamma_table[ord + 2]),
                      ptr_x[idx - (ord + 2)]);
      s -= ia_mul_flt(ia_mul_flt(ptr_a[ord + 3], ia_core_coder_gamma_table[ord + 3]),
                      ptr_x[idx - (ord + 3)]);
    }
    ptr_x[idx] = s;
  }

  return;
}

/**
 *  ia_core_coder_synthesis_tool
 *
 *  \brief Weighted synthesis filtering.
 *
 *  \param [in]  ptr_a       Pointer to input signal.
 *  \param [out] ptr_x       Pointer to output signal.
 *  \param [out] ptr_y       Pointer to output buffer
 *  \param [in]  l       Length of input.
 *  \param [in,out] mem     Pointer to filter states memory.
 *  \param [in]  scratch Pointer to scratch buffer.
 *
 *
 *
 */
VOID ia_core_coder_synthesis_tool(FLOAT32 *ptr_a, FLOAT32 *ptr_x, FLOAT32 *ptr_y, WORD32 l,
                                  FLOAT32 *mem, FLOAT32 *scratch)
{
  WORD32 idx, ord;
  FLOAT32 s;
  FLOAT32 *ptr_yy;
  FLOAT32 *ptr_buf;
  ptr_buf = (FLOAT32 *)&scratch[0];
  ia_core_coder_mem_cpy(mem, ptr_buf, ORDER * sizeof(FLOAT32));
  ptr_yy = &ptr_buf[ORDER];
  for (idx = 0; idx < l; idx++)
  {
    s = ptr_x[idx];
    for (ord = 1; ord <= ORDER; ord += 4)
    {
      s -= ia_mul_flt(ptr_a[ord], ptr_yy[idx - ord]);
      s -= ia_mul_flt(ptr_a[ord + 1], ptr_yy[idx - (ord + 1)]);
      s -= ia_mul_flt(ptr_a[ord + 2], ptr_yy[idx - (ord + 2)]);
      s -= ia_mul_flt(ptr_a[ord + 3], ptr_yy[idx - (ord + 3)]);
    }
    ptr_yy[idx] = s;
    ptr_y[idx] = s;
  }

  return;
}
/**
 *  ia_core_coder_synthesis_tool1
 *
 *  \brief Weighted synthesis filtering.
 *
 *  \param [in]  ptr_a Pointer to input signal.
 *  \param [out] ptr_x Pointer to ouptut signal.
 *  \param [in]  l Length of input.
 *
 *
 *
 */
VOID ia_core_coder_synthesis_tool1(FLOAT32 *ptr_a, FLOAT32 *ptr_x, WORD32 l)
{
  WORD32 idx, ord;
  FLOAT32 s;
  for (idx = 0; idx < l; idx++)
  {
    s = ptr_x[idx];
    for (ord = 1; ord <= ORDER; ord += 4)
    {
      s -= ia_mul_flt(ptr_a[ord], ptr_x[idx - ord]);
      s -= ia_mul_flt(ptr_a[ord + 1], ptr_x[idx - (ord + 1)]);
      s -= ia_mul_flt(ptr_a[ord + 2], ptr_x[idx - (ord + 2)]);
      s -= ia_mul_flt(ptr_a[ord + 3], ptr_x[idx - (ord + 3)]);
    }
    ptr_x[idx] = s;
  }

  return;
}
/**
 *  ia_core_coder_residual_tool
 *
 *  \brief Residual filtering.
 *
 *  \param [in]  ptr_a          Weights/ coeffs
 *  \param [in]  ptr_x          Pointer to input buffer.
 *  \param [out] ptr_y          Pointer to ouptut buffer.
 *  \param [in]  l          Length of the input
 *  \param [in]  loop_count Loop count variable
 *  \param [in]  incr_val   Step size for incrementing.
 *
 *
 *
 */
VOID ia_core_coder_residual_tool(FLOAT32 *ptr_a, FLOAT32 *ptr_x, FLOAT32 *ptr_y, WORD32 l,
                                 WORD32 loop_count, WORD32 incr_val)
{
  FLOAT32 s;
  WORD32 idx, l_cnt;
  for (l_cnt = 0; l_cnt < loop_count; l_cnt++)
  {
    for (idx = 0; idx < l; idx++)
    {
      s = ptr_x[idx];
      for (WORD32 i = 1; i < 17; i++)
      {
        s += ia_mul_flt(ptr_a[i], ptr_x[idx - i]);
      }
      ptr_y[idx] = s;
    }
    ptr_a += incr_val;
    ptr_x += l;
    ptr_y += l;
  }
  return;
}
/** @} */ /* End of CoreDecProc */