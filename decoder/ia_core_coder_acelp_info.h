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

#ifndef IA_CORE_CODER_ACELP_INFO_H
#define IA_CORE_CODER_ACELP_INFO_H

typedef struct
{
  WORD32 acelp_core_mode;
  WORD32 mod[NUM_FRAMES];

  WORD32 fac[NUM_FRAMES * FAC_LENGTH];
  WORD32 fd_fac[FAC_LENGTH + 1];
  WORD32 mean_energy[NUM_FRAMES];
  WORD32 acb_index[NUM_SUBFR_SUPERFRAME];
  WORD32 noise_factor[NUM_FRAMES];
  WORD32 global_gain[NUM_FRAMES];
  WORD32 arith_reset_flag;
  WORD32 x_tcx_invquant[LEN_SUPERFRAME];
  WORD32 tcx_lg[4 * NUM_FRAMES];
  WORD32 ltp_filtering_flag[NUM_SUBFR_SUPERFRAME];
  WORD32 icb_index[NUM_SUBFR_SUPERFRAME][8];
  WORD32 gains[NUM_SUBFR_SUPERFRAME];
  WORD32 mode_lpc[NUM_FRAMES];
  WORD32 lpc_first_approx_idx[110];
} ia_td_frame_data_struct;

typedef struct
{
  WORD32 islong;
  WORD32 max_win_len;
  WORD32 samp_per_bk;
  WORD32 sfb_per_bk;
  WORD32 bins_per_sbk;
  WORD32 sfb_per_sbk;

  const WORD16 *ptr_sfb_tbl;
  pWORD16 sfb_width;
  WORD16 sfb_idx_tbl[125];
  WORD32 num_groups;
  WORD16 group_len[8];

} ia_sfb_info_struct;

#endif /* IA_CORE_CODER_ACELP_INFO_H */
