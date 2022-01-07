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

#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include <impeghd_type_def.h>

#include "impeghd_intrinsics_flt.h"

#include "ia_core_coder_acelp_info.h"
#include <ia_core_coder_basic_ops32.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_function_selector.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "impeghd_error_codes.h"

#define sfb_offset(x) (((x) > 0) ? sfb_top[(x)-1] : 0)

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  impeghd_tns_decode_coeff_ar_filter
 *
 *  \brief Calculate transmitted coefficients for one TNS filter
 *         Performs tns filtering with an all pole filter of order 'order'
 *
 *  \param [in]    order    tns max order
 *  \param [in]    coef_res residual coeff
 *  \param [in]    coef     tns coeff
 *  \param [in,out]  spec    Input Spectrum which will be filtered
 *  \param [in]    size    Number of samples
 *  \param [in]    inc    Index increment
 *
 *
 *
 */
static VOID impeghd_tns_decode_coeff_ar_filter(WORD32 order, WORD32 coef_res, WORD16 *coef,
                                               FLOAT32 *spec, WORD32 size, WORD32 inc)
{
  WORD32 i, j, m;
  FLOAT32 iqfac, iqfac_m, y;
  FLOAT32 tmp[TNS_MAX_ORDER + 1], b[TNS_MAX_ORDER + 1], lpc[TNS_MAX_ORDER + 1];
  FLOAT32 state[TNS_MAX_ORDER];
  const FLOAT32 pi2 = (FLOAT32)PI / 2;

  /* Inverse quantization */
  iqfac = (FLOAT32)(ia_sub_flt((FLOAT32)(1 << (coef_res - 1)), 0.5) / (pi2));
  iqfac_m = (FLOAT32)(ia_add_flt((FLOAT32)(1 << (coef_res - 1)), 0.5) / (pi2));
  for (i = 0; i < order; i++)
  {
    tmp[i + 1] = (FLOAT32)sin(coef[i] / ((coef[i] >= 0) ? iqfac : iqfac_m));
  }

  /* Decoding co-efficients */
  lpc[0] = 1;
  for (m = 1; m <= order; m++)
  {
    b[0] = lpc[0];
    for (i = 1; i < m; i++)
    {
      b[i] = ia_mac_flt(lpc[i], tmp[m], lpc[m - i]);
    }
    b[m] = tmp[m];
    for (i = 0; i <= m; i++)
    {
      lpc[i] = b[i];
    }
  }

  /* AR filter */
  ia_core_coder_memset(state, order);

  if (inc == -1)
    spec += size - 1;

  for (i = 0; i < size; i++)
  {
    y = *spec;
    for (j = 0; j < order; j++)
      y = ia_msu_flt(y, lpc[j + 1], state[j]);
    for (j = order - 1; j > 0; j--)
      state[j] = state[j - 1];
    state[0] = y;
    *spec = y;
    spec += inc;
  }
}

/**
 *  ia_core_coder_igf_nbands
 *
 *  \brief Update number of bands if IGF is active
 *
 *  \param [in]    nbands      Number of bands
 *  \param [in]    pstr_sfb_info  sfb info structure
 *  \param [in]    igf_config    IGF config structure
 *
 *
 *
 */
static VOID ia_core_coder_igf_nbands(WORD32 *nbands, ia_sfb_info_struct *pstr_sfb_info,
                                     ia_usac_igf_config_struct *igf_config)
{
  if (igf_config->igf_active)
  {
    if (igf_config->igf_after_tns_synth)
    {
      if (pstr_sfb_info->islong)
      {
        if (igf_config->igf_grid[0].igf_sfb_start < *nbands)
        {
          *nbands = igf_config->igf_grid[0].igf_sfb_start;
        }
      }
      else
      {
        if (igf_config->igf_grid[1].igf_sfb_start < *nbands)
        {
          *nbands = igf_config->igf_grid[1].igf_sfb_start;
        }
      }
    }
    else
    {
      if (pstr_sfb_info->islong)
      {
        if (igf_config->igf_grid[0].igf_sfb_stop > *nbands)
        {
          *nbands = igf_config->igf_grid[0].igf_sfb_stop;
        }
      }
      else
      {
        if (igf_config->igf_grid[1].igf_sfb_stop > *nbands)
        {
          *nbands = igf_config->igf_grid[1].igf_sfb_stop;
        }
      }
    }
  }
}

/**
 *  ia_core_coder_tns_apply
 *
 *  \brief TNS decoding of one frame in a channel
 *
 *  \param [in]    usac_data    USAC data structure
 *  \param [in,out]  spec        Input Spectrum which will be filtered
 *  \param [in]    nbands      Number of bands
 *  \param [in]    pstr_sfb_info  sfb info structure
 *  \param [in]    pstr_tns      tns frame info_struct
 *  \param [in]    igf_config    IGF config structure
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_tns_apply(ia_usac_data_struct *usac_data, FLOAT32 *spec, WORD32 nbands,
                                     ia_sfb_info_struct *pstr_sfb_info,
                                     ia_tns_frame_info_struct *pstr_tns,
                                     ia_usac_igf_config_struct *igf_config)
{
  WORD32 f, start, stop, size, inc;
  WORD32 n_filt, coef_res, order;
  FLOAT32 *ptr_spec;

  ia_tns_filter_struct *filt;
  short *coef;
  const WORD16 *sfb_top;

  WORD32 nbins = (pstr_sfb_info->islong) ? 1024 : 128;
  WORD32 j;

  WORD32 idx = (pstr_sfb_info->islong) ? 0 : 1;
  WORD32 tmp = (*usac_data->tns_max_bands_tbl_usac)[usac_data->sampling_rate_idx][idx];

  ptr_spec = &usac_data->scratch_buffer_float[0];

  ia_core_coder_igf_nbands(&nbands, pstr_sfb_info, igf_config);

  for (j = 0; j < pstr_tns->n_subblocks; j++)
  {
    sfb_top = pstr_sfb_info->ptr_sfb_tbl;

    ia_core_coder_mem_cpy(spec, ptr_spec, nbins);

    if (pstr_tns->str_tns_info[j].n_filt)
    {
      n_filt = pstr_tns->str_tns_info[j].n_filt;

      for (f = 0; f < n_filt; f++)
      {
        coef_res = pstr_tns->str_tns_info[j].coef_res;
        filt = &pstr_tns->str_tns_info[j].str_filter[f];
        order = filt->order;
        start = filt->start_band;
        stop = filt->stop_band;
        coef = filt->coef;
        if (!order)
          continue;

        start = ia_min_int(ia_min_int(start, tmp), nbands);
        if (start > pstr_sfb_info->sfb_per_sbk)
          return IA_MPEGH_DEC_EXE_FATAL_INVALID_TNS_SFB;

        start = sfb_offset(start);

        if (igf_config->igf_active)
        {
          stop = ia_min_int(stop, nbands);
        }
        else
        {
          stop = ia_min_int(ia_min_int(stop, tmp), nbands);
        }
        if (stop > pstr_sfb_info->sfb_per_sbk)
          return IA_MPEGH_DEC_EXE_FATAL_INVALID_TNS_SFB;

        stop = sfb_offset(stop);

        if ((size = stop - start) <= 0)
          continue;

        if (0 == filt->direction)
        {
          inc = 1;
        }

        else
        {
          inc = -1;
        }

        impeghd_tns_decode_coeff_ar_filter(order, coef_res, coef, &ptr_spec[start], size, inc);

        ia_core_coder_mem_cpy(ptr_spec, spec, nbins);
      }
    }

    spec += pstr_sfb_info->bins_per_sbk;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecProc */