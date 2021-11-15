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

#ifndef _IA_CORE_CODER_IGF_DATA_STRUCT_
#define _IA_CORE_CODER_IGF_DATA_STRUCT_

#define MAX_IGF_TILES (4)
#define MAX_IGF_LEN (2048)

typedef struct
{
  UWORD8 igf_sfb_start;
  UWORD8 igf_sfb_stop;
  UWORD8 igf_num_tiles;
  UWORD8 igf_min;
} ia_usac_igf_grid_config_struct;

typedef struct
{
  WORD8 igf_active;
  WORD8 use_high_res;
  WORD8 use_inf;
  WORD8 use_tnf;
  WORD8 use_whitening;
  WORD8 igf_after_tns_synth;
  WORD8 igf_stereo_filling;
  WORD8 igf_independent_tiling;
  ia_usac_igf_grid_config_struct igf_grid[4];
} ia_usac_igf_config_struct;

typedef struct
{
  WORD32 igf_all_zero;
  WORD8 igf_tile_num[MAX_IGF_TILES];
  FLOAT32 igf_levels_curr_float[MAX_SHORT_WINDOWS][NSFB_LONG];
  WORD32 igf_whitening_level[MAX_IGF_TILES];
  WORD8 igf_is_tnf;
} ia_usac_igf_bitstream_struct;

typedef struct
{
  WORD8 prev_tile_num[MAX_IGF_TILES];
  WORD32 prev_whitening_level[MAX_IGF_TILES];
  WORD8 prev_win_type;
} ia_usac_igf_mem_struct;

typedef struct
{
  WORD32 igf_arith_time_idx;
  WORD32 igf_prev_win_ctx;
  WORD32 igf_levels_prev_fix[NSFB_LONG];
  FLOAT32 igf_sn_float[NSFB_LONG];
  FLOAT32 igf_pn_float[NSFB_LONG];
} ia_usac_igf_envelope_struct;

typedef struct
{
  ia_usac_igf_mem_struct igf_memory;
  ia_usac_igf_bitstream_struct igf_bitstream[2];
  ia_usac_igf_envelope_struct igf_envelope;
  FLOAT32 igf_input_spec[4 * MAX_IGF_LEN];
  FLOAT32 igf_tnf_spec_float[MAX_IGF_LEN];
  WORD8 igf_tnf_mask[MAX_IGF_LEN];
  FLOAT32 *igf_scratch;
} ia_usac_igf_dec_data_struct;

#endif
