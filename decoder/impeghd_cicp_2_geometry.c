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
#include <stdlib.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"

/**
 * @defgroup CICPUtils CICP Utililty functions
 * @ingroup  CICPUtils
 * @brief CICP Utililty functions
 *
 * @{
 */

/**
 *  impeghd_cicpidx_2_ls_geometry
 *
 *  \brief Fetch channels, lfe and other details from table
 *
 *  \param [in]     cicp_idx                CICP index
 *  \param [in,out] pp_cicp_ls_geometry     Pointer to CICP Loudspeaker geometry
 *  \param [out]    num_channels            Pointer to number of channels
 *  \param [out]    num_lfe                 Pointer to number of low-frequency elements
 *  \param [out]    channel_names           Pointer to channel names
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impeghd_cicpidx_2_ls_geometry(WORD32 cicp_idx,
                                           const ia_cicp_ls_geo_str **pp_cicp_ls_geometry,
                                           WORD32 *num_channels, WORD32 *num_lfe,
                                           const WORD32 **channel_names)
{
  WORD32 ch;
  WORD32 ch_idx;
  if (cicp_idx > NUM_LS_CFGS || cicp_idx == 8 || cicp_idx == 0)
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
  }
  *num_lfe = 0;
  *channel_names = ia_cicp_idx_ls_set_map_tbl[cicp_idx];
  *num_channels = impgehd_cicp_get_num_ls[cicp_idx];

  for (ch = 0; ch < *num_channels; ch++)
  {
    ch_idx = channel_names[0][ch];
    pp_cicp_ls_geometry[ch] = &ia_cicp_ls_geo_tbls[ch_idx];
    *num_lfe += pp_cicp_ls_geometry[ch]->lfe_flag;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/** @} */ /* End of CICPUtils */