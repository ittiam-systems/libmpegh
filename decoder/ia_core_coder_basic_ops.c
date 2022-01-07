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

#include "impeghd_intrinsics_flt.h"
#include <impeghd_type_def.h>
#include <ia_core_coder_constants.h>
#include "ia_core_coder_windows.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_memset
 *
 *  \brief Wrapper memset function.
 *
 *  \param [in,out] ptr_x Pointer to input buffer.
 *  \param [in] n Length of the input buffer.
 *
 *
 *
 */
VOID ia_core_coder_memset(FLOAT32 *ptr_x, WORD32 n)
{
  memset(ptr_x, 0, n * sizeof(FLOAT32));
  return;
}
/**
 *  ia_core_coder_mem_cpy
 *
 *  \brief Wrapper function for memcpy.
 *
 *  \param [in]  ptr_x Pointer to source buffer.
 *  \param [out] ptr_y Pointer to destination buffer.
 *  \param [in]  n Length of the buffer.
 *
 *
 *
 */
VOID ia_core_coder_mem_cpy(const FLOAT32 *ptr_x, FLOAT32 *ptr_y, WORD32 n)
{
  memcpy(ptr_y, ptr_x, n * sizeof(FLOAT32));
  return;
}

/**
 *  ia_core_coder_vec_cnst_mul
 *
 *  \brief Helper function to multiply vectors with a constant value
 *
 *  \param [in]  a Constant multiplication factor
 *  \param [in]  ptr_x Input vector
 *  \param [out] ptr_z Destination / Output vector
 *  \param [in]  n Length of vector
 *
 *
 *
 */
VOID ia_core_coder_vec_cnst_mul(FLOAT32 a, FLOAT32 *ptr_x, FLOAT32 *ptr_z, WORD32 n)
{
  WORD32 i;
  for (i = 0; i < n; i++)
  {
    ptr_z[i] = ia_mul_flt((FLOAT32)a, ptr_x[i]);
  }
  return;
}

/**
 *  ia_core_coder_windowing_long1
 *
 *  \brief Windowing for window sequence:
 *    ONLY_LONG_SEQUENCE and LONG_START_SEQUENCE
 *
 *  \param [in]  ptr_src1    Pointer to source buffer 1.
 *  \param [in]  ptr_src2    Pointer to source buffer 2.
 *  \param [in]  ptr_win_fwd Pointer to forward window buffer.
 *  \param [in]  ptr_win_rev Pointer to reverse window buffer.
 *  \param [out] ptr_dest    Pointer to destination buffer.
 *  \param [in]  transform_kernel_type Transform kernel type indicator
 *  \param [in]  vlen    Vector length parameter.
 *
 *
 *
 */
VOID ia_core_coder_windowing_long1(FLOAT32 *ptr_src1, FLOAT32 *ptr_src2,
                                   const FLOAT32 *ptr_win_fwd, const FLOAT32 *ptr_win_rev,
                                   FLOAT32 *ptr_dest, WORD16 transform_kernel_type, WORD32 vlen)
{
  WORD32 i;
  FLOAT32 *rsrc2 = ptr_src2 + vlen - 1;

  for (i = 0; i < vlen / 2; i++)
  {
    *ptr_dest = ia_mac_flt(ia_mul_flt(*ptr_src1, *ptr_win_fwd), *ptr_src2, *ptr_win_rev);
    switch (transform_kernel_type)
    {
    case 2:
    case 3:
      *(ptr_dest + (vlen - (2 * i)) - 1) =
          ia_mac_flt(ia_mul_flt(*ptr_src1, *ptr_win_rev), *rsrc2, *ptr_win_fwd);
      break;
    default:
      *(ptr_dest + (vlen - (2 * i)) - 1) =
          ia_mac_flt(ia_mul_flt(ia_negate_flt(*ptr_src1), *ptr_win_rev), *rsrc2, *ptr_win_fwd);
      break;
    }
    ptr_src1++;
    ptr_src2++;
    ptr_win_fwd++;
    ptr_win_rev--;
    rsrc2--;
    ptr_dest++;
  }
}

/**
 *  ia_core_coder_windowing_long2
 *
 *  \brief Windowing for window sequence:
 *      STOP_START_SEQUENCE and LONG_STOP_SEQUENCE
 *      with FAC(Forward aliasing cancellation) present
 *
 *  \param [in]  ptr_src1         Pointer to source/input buffer.
 *  \param [in]  ptr_win_fwd      Pointer to forward windowing buffer.
 *  \param [out] fac_data_out Pointer to output data buffer.
 *  \param [in]  ptr_over_lap     Pointer to overlap buffer pointer.
 *  \param [out] ptr_out_buffer Pointer to output buffer.
 *  \param [in]  pstr_drc_offset Pointer to offset data buffer.
 *
 *
 *
 */
VOID ia_core_coder_windowing_long2(FLOAT32 *ptr_src1, const FLOAT32 *ptr_win_fwd,
                                   FLOAT32 *fac_data_out, FLOAT32 *ptr_over_lap,
                                   FLOAT32 *ptr_out_buffer, offset_lengths *pstr_drc_offset)
{
  WORD32 i;
  FLOAT32 *ptr_dest = ptr_out_buffer;

  ptr_win_fwd += pstr_drc_offset->lfac;
  memcpy(ptr_dest, ptr_over_lap,
         pstr_drc_offset->n_flat_ls + pstr_drc_offset->lfac * sizeof(FLOAT32));

  for (i = pstr_drc_offset->n_flat_ls + (pstr_drc_offset->lfac * 3); i < pstr_drc_offset->n_long;
       i++)
  {
    ptr_dest[i] =
        ia_negate_flt(ptr_src1[pstr_drc_offset->n_long / 2 + pstr_drc_offset->n_flat_ls +
                               pstr_drc_offset->lfac - i - 1]);
  }
  for (i = pstr_drc_offset->n_flat_ls + pstr_drc_offset->n_trans_ls;
       i < pstr_drc_offset->n_flat_ls + (pstr_drc_offset->lfac * 3); i++)
  {
    ptr_dest[i] = ia_add_flt(
        ia_negate_flt(ptr_src1[pstr_drc_offset->n_long / 2 + pstr_drc_offset->n_flat_ls +
                               pstr_drc_offset->lfac - i - 1]),
        (*fac_data_out));
    fac_data_out++;
  }
  for (i = pstr_drc_offset->n_flat_ls + pstr_drc_offset->lfac;
       i < pstr_drc_offset->n_flat_ls + pstr_drc_offset->n_trans_ls; i++)
  {
    ptr_dest[i] =
        (*fac_data_out) +
        ia_mul_flt(
            ia_negate_flt(ptr_src1[pstr_drc_offset->n_long / 2 + pstr_drc_offset->n_flat_ls +
                                   pstr_drc_offset->lfac - i - 1]),
            *ptr_win_fwd);
    ptr_win_fwd++;
    fac_data_out++;
  }
}

/**
 *  ia_core_coder_windowing_long3
 *
 *  \brief Windowing for window sequence:
 *      STOP_START_SEQUENCE and LONG_STOP_SEQUENCE
 *      with FAC(Forward aliasing cancellation) absent
 *
 *  \param [in]  ptr_src1         Pointer to source/input buffer.
 *  \param [in]  ptr_win_fwd      Pointer to forward windowing coefficients buffer.
 *  \param [in]  ptr_over_lap     Pointer to overlap buffer pointer.
 *  \param [out] ptr_out_buffer Pointer to output buffer.
 *  \param [in]  ptr_win_rev      Pointer to reverse windowing coefficients buffer.
 *  \param [in]  pstr_drc_offset Pointer to offset data buffer.
 *
 *
 *
 */
VOID ia_core_coder_windowing_long3(FLOAT32 *ptr_src1, const FLOAT32 *ptr_win_fwd,
                                   FLOAT32 *ptr_over_lap, FLOAT32 *ptr_out_buffer,
                                   const FLOAT32 *ptr_win_rev, offset_lengths *pstr_drc_offset,
                                   WORD32 transform_kernel_type)
{
  WORD32 i;
  FLOAT32 *ptr_dest = ptr_out_buffer;
  {
    memcpy(ptr_dest, ptr_over_lap, (pstr_drc_offset->n_flat_ls) * sizeof(FLOAT32));
    for (i = pstr_drc_offset->n_flat_ls; i < pstr_drc_offset->n_long / 2; i++)
    {
      ptr_dest[i] =
          ia_mul_flt(ptr_src1[i], *ptr_win_fwd) + ia_mul_flt(ptr_over_lap[i], *ptr_win_rev);
      ptr_win_fwd++;
      ptr_win_rev--;
    }
    if (transform_kernel_type != 3)
    {
      for (i = pstr_drc_offset->n_long / 2;
           i < pstr_drc_offset->n_flat_ls + pstr_drc_offset->n_trans_ls; i++)
      {
        ptr_dest[i] =
            ia_mul_flt(ia_negate_flt(ptr_src1[pstr_drc_offset->n_long - i - 1]), *ptr_win_fwd) +
            ia_mul_flt(ptr_over_lap[i], *ptr_win_rev);
        ptr_win_fwd++;
        ptr_win_rev--;
      }

      for (; i < pstr_drc_offset->n_long; i++)
      {
        ptr_dest[i] = ia_negate_flt(ptr_src1[pstr_drc_offset->n_long - i - 1]);
      }
    }
    else
    {
      for (i = pstr_drc_offset->n_long / 2;
           i < pstr_drc_offset->n_flat_ls + pstr_drc_offset->n_trans_ls; i++)
      {
        ptr_dest[i] = ia_mul_flt(ptr_src1[pstr_drc_offset->n_long - i - 1], *ptr_win_fwd) +
                      ia_mul_flt(ptr_over_lap[i], *ptr_win_rev);
        ptr_win_fwd++;
        ptr_win_rev--;
      }

      for (; i < pstr_drc_offset->n_long; i++)
      {
        ptr_dest[i] = (ptr_src1[pstr_drc_offset->n_long - i - 1]);
      }
    }
  }
}

/**
 *  ia_core_coder_windowing_short1
 *
 *  \brief Windowing for window sequence:
 *      EIGHT_SHORT_SEQUENCE
 *      with FAC(Forward aliasing cancellation) present
 *
 *  \param [in]  ptr_src1 Pointer to source/input buffer.
 *  \param [in]  ptr_src2 Pointer to source/input buffer.
 *  \param [out] ptr_fp   Pointer to output buffer.
 *  \param [in]  pstr_drc_offset Pointer to offset data buffer.
 *
 *
 *
 */
VOID ia_core_coder_windowing_short1(FLOAT32 *ptr_src1, FLOAT32 *ptr_src2, FLOAT32 *ptr_fp,
                                    offset_lengths *pstr_drc_offset)
{
  WORD32 i;
  FLOAT32 *ptr_dest = ptr_fp;

  if (pstr_drc_offset->n_short <= pstr_drc_offset->lfac)
  {
    memset(&ptr_dest[pstr_drc_offset->lfac], 0,
           (pstr_drc_offset->n_flat_ls + pstr_drc_offset->lfac) * sizeof(FLOAT32));
  }
  else
  {
    for (i = pstr_drc_offset->lfac; i < pstr_drc_offset->n_short; i++)
    {
      ptr_dest[i] =
          ia_mul_flt(ia_negate_flt(ptr_src1[pstr_drc_offset->n_short - i - 1]), ptr_src2[i]);
    }
    memset(&ptr_dest[i], 0,
           (pstr_drc_offset->n_flat_ls + pstr_drc_offset->lfac) * sizeof(FLOAT32));
  }
}

/**
 *  ia_core_coder_windowing_short2
 *
 *  \brief Windowing for window sequence:
 *      EIGHT_SHORT_SEQUENCE
 *      with FAC(Forward aliasing cancellation) absent
 *
 *  \param [in]  ptr_src1    Pointer to source/input buffer.
 *  \param [in]  ptr_win_fwd Pointer to forward windowing coefficients.
 *  \param [out] ptr_fp      Pointer to output buffer.
 *  \param [in]  pstr_drc_offset Pointer to offset data buffer.
 *
 *
 *
 */
VOID ia_core_coder_windowing_short2(FLOAT32 *ptr_src1, FLOAT32 *ptr_win_fwd, FLOAT32 *ptr_fp,
                                    offset_lengths *pstr_drc_offset, WORD32 transform_kernel_type)
{
  WORD32 i;

  FLOAT32 *ptr_win_rev = ptr_win_fwd + pstr_drc_offset->n_short - 1;
  if (transform_kernel_type != 3)
  {
    for (i = 0; i < pstr_drc_offset->n_short / 2; i++)
    {
      ptr_fp[i] = ia_mul_flt(ptr_src1[i], *ptr_win_fwd) + ia_mul_flt(ptr_fp[i], *ptr_win_rev);
      ptr_fp[pstr_drc_offset->n_short - i - 1] =
          ia_mul_flt(ia_negate_flt(ptr_src1[i]), *ptr_win_rev) +
          ia_mul_flt(ptr_fp[pstr_drc_offset->n_short - i - 1], *ptr_win_fwd);
      ptr_win_fwd++;
      ptr_win_rev--;
    }
  }
  else
  {
    for (i = 0; i < pstr_drc_offset->n_short / 2; i++)
    {
      ptr_fp[i] = ia_mul_flt(ptr_src1[i], *ptr_win_fwd) + ia_mul_flt(ptr_fp[i], *ptr_win_rev);
      ptr_fp[pstr_drc_offset->n_short - i - 1] =
          ia_mul_flt((ptr_src1[i]), *ptr_win_rev) +
          ia_mul_flt(ptr_fp[pstr_drc_offset->n_short - i - 1], *ptr_win_fwd);
      ptr_win_fwd++;
      ptr_win_rev--;
    }
  }
  memset(&ptr_fp[pstr_drc_offset->n_short], 0,
         (pstr_drc_offset->n_flat_ls + pstr_drc_offset->n_short) * sizeof(FLOAT32));
}

/**
 *  ia_core_coder_windowing_short3
 *
 *  \brief Helper funciton for Windowing of window sequence:
 *      EIGHT_SHORT_SEQUENCE
 *
 *  \param [in]  ptr_src1    Pointer to source/input buffer.
 *  \param [in]  ptr_win_rev Pointer to forward windowing coefficients.
 *  \param [out] ptr_fp      Pointer to output buffer.
 *  \param [in]  n_short Length of the input.
 *
 *
 *
 */
VOID ia_core_coder_windowing_short3(FLOAT32 *ptr_src1, FLOAT32 *ptr_win_rev, FLOAT32 *ptr_fp,
                                    WORD32 n_short, WORD32 transform_kernel_type)
{
  WORD32 i;
  const FLOAT32 *ptr_win_fwd = ptr_win_rev - n_short + 1;

  if (transform_kernel_type != 3)
  {
    for (i = 0; i < n_short / 2; i++)
    {
      ptr_fp[i] += ia_mul_flt(ia_negate_flt(ptr_src1[n_short / 2 - i - 1]), *ptr_win_rev);
      ptr_fp[n_short - i - 1] +=
          ia_mul_flt(ia_negate_flt(ptr_src1[n_short / 2 - i - 1]), *ptr_win_fwd);
      ptr_win_fwd++;
      ptr_win_rev--;
    }
  }
  else
  {
    for (i = 0; i < n_short / 2; i++)
    {
      ptr_fp[i] = ia_mul_flt(ptr_src1[n_short / 2 - i - 1], *ptr_win_rev + ptr_fp[i]);
      ptr_fp[n_short - i - 1] +=
          ia_mul_flt(ia_negate_flt(ptr_src1[n_short / 2 - i - 1]), *ptr_win_fwd);
      ptr_win_fwd++;
      ptr_win_rev--;
    }
  }
}

/**
 *  ia_core_coder_windowing_short4
 *
 *  \brief Helper funciton for Windowing of window sequence:
 *      EIGHT_SHORT_SEQUENCE
 *
 *  \param [in]  ptr_src1      Pointer to source/input buffer.
 *  \param [in]  ptr_win_fwd   Pointer to forward windowing coefficients.
 *  \param [out] ptr_fp        Pointer to output buffer.
 *  \param [in]  ptr_win_fwd1  Pointer to forward windowing coefficients.
 *  \param [in]  n_short   Length of the data.
 *  \param [in]  flag      Data processing related flag.
 *
 *
 *
 */
VOID ia_core_coder_windowing_short4(FLOAT32 *ptr_src1, FLOAT32 *ptr_win_fwd, FLOAT32 *ptr_fp,
                                    FLOAT32 *ptr_win_fwd1, WORD32 n_short, WORD32 flag,
                                    WORD32 transform_kernel_type)
{
  WORD32 i;
  const FLOAT32 *win_rev1 = ptr_win_fwd1 - n_short + 1;
  const FLOAT32 *ptr_win_rev = ptr_win_fwd + n_short - 1;

  if (transform_kernel_type != 3)
  {
    for (i = 0; i < n_short / 2; i++)
    {
      ptr_fp[i] += ia_mul_flt(ptr_src1[n_short / 2 + i], *ptr_win_fwd);

      ptr_fp[n_short - i - 1] +=
          ia_mul_flt(ia_negate_flt(ptr_src1[n_short / 2 + i]), *ptr_win_rev);

      ptr_win_fwd++;
      ptr_win_rev--;
    }

    if (flag != 1)
    {
      for (; i < n_short; i++)
      {
        ptr_fp[i + n_short / 2] += ia_negate_flt(ptr_src1[n_short - i - 1]);
        ptr_fp[3 * n_short - n_short / 2 - i - 1] += ia_negate_flt(ptr_src1[n_short - i - 1]);
      }
    }
    else
    {
      for (; i < n_short; i++)
      {
        ptr_fp[i + n_short / 2] +=
            ia_mul_flt(ia_negate_flt(ptr_src1[n_short - i - 1]), *ptr_win_fwd1);

        ptr_fp[3 * n_short - n_short / 2 - i - 1] +=

            ia_mul_flt(ia_negate_flt(ptr_src1[n_short - i - 1]), (*win_rev1));

        ptr_win_fwd1--;
        win_rev1++;
      }
    }
  }
  else
  {
    for (i = 0; i < n_short / 2; i++)
    {
      ptr_fp[i] += ia_mul_flt(ptr_src1[n_short / 2 + i], *ptr_win_fwd);

      ptr_fp[n_short - i - 1] += ia_mul_flt(ptr_src1[n_short / 2 + i], *ptr_win_rev);

      ptr_win_fwd++;
      ptr_win_rev--;
    }
    if (flag == 1)
    {
      for (; i < n_short; i++)
      {
        ptr_fp[i + n_short / 2] += ia_mul_flt((ptr_src1[n_short - i - 1]), *ptr_win_fwd1);

        ptr_fp[3 * n_short - n_short / 2 - i - 1] +=

            ia_mul_flt(ia_negate_flt(ptr_src1[n_short - i - 1]), *win_rev1);

        ptr_win_fwd1--;
        win_rev1++;
      }
    }
    else
    {
      for (; i < n_short; i++)
      {
        ptr_fp[i + n_short / 2] += (ptr_src1[n_short - i - 1]);
        ptr_fp[3 * n_short - n_short / 2 - i - 1] += ia_negate_flt(ptr_src1[n_short - i - 1]);
      }
    }
  }
}
/** @} */ /* End of CoreDecProc */