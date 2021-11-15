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

#ifndef IA_CORE_CODER_CHANNELINFO_H
#define IA_CORE_CODER_CHANNELINFO_H
#define MAX_SFB_SHORT 16
#define MAX_QUANTIZED_VALUE 8191

#define OVERLAP_BUFFER_SIZE 512

#define JOINT_STEREO_MAX_GROUPS 8
#define JOINT_STEREO_MAX_BANDS 64

typedef struct
{
  WORD16 window_shape;
  WORD16 window_sequence;
  WORD16 max_sfb;
  WORD16 num_swb_window;
  WORD16 sampling_rate_index;
  WORD16 num_window_groups;
  WORD8 window_group_length[8];
  WORD16 frame_length;
  WORD32 frame_size;
  WORD16 predictor_data_present;
} ia_ics_info_struct;

typedef struct
{
  WORD16 start_band;
  WORD16 stop_band;
  WORD8 direction;
  WORD8 resolution;
  WORD8 order;
  WORD8 coef[MAX_ORDER];
} ia_filter_info_struct;

typedef struct
{
  FLAG tns_data_present;
  WORD8 n_filt[MAX_WINDOWS];
  ia_filter_info_struct str_filter[MAX_WINDOWS][MAX_FILTERS];
} ia_tns_info_mpeghd_struct;

#define LINES_PER_UNIT 4

#define MAX_SFB_HCR (((1024 / 8) / LINES_PER_UNIT) * 8)
#define NUMBER_OF_UNIT_GROUPS (LINES_PER_UNIT * 8)
#define LINES_PER_UNIT_GROUP (1024 / NUMBER_OF_UNIT_GROUPS)

#define FROM_LEFT_TO_RIGHT 0
#define FROM_RIGHT_TO_LEFT 1

#define MAX_CB_PAIRS 23
#define MAX_HCR_SETS 14

#define ESCAPE_VALUE 16
#define POSITION_OF_FLAG_A 21
#define POSITION_OF_FLAG_B 20

#define MAX_CB 32

#define MAX_CB_CHECK 32
#define WORD_BITS 32

#define THIRTYTWO_LOG_DIV_TWO_LOG 5
#define EIGHT_LOG_DIV_TWO_LOG 3
#define FOUR_LOG_DIV_TWO_LOG 2

#define CPE_TOP_LENGTH 12288
#define SCE_TOP_LENGTH 6144
#define LEN_OF_LONGEST_CW_TOP_LENGTH 49
#define Q_VALUE_INVALID 8192
#define NODE_MASK 0x400

typedef struct
{
  WORD16 *ptr_scale_factor;
  WORD8 *ptr_code_book;
  WORD32 *ptr_spec_coeff;
  ia_ics_info_struct str_ics_info;
  ia_tns_info_mpeghd_struct str_tns_info;
  WORD16 common_window;
  WORD16 element_instance_tag;
  WORD16 global_gain;
  WORD32 *scratch_buf_ptr;
  WORD32 *pulse_scratch;
  WORD16 *ltp_buf;
  UWORD16 ltp_lag;
} ia_mpegh_dec_channel_info_struct;

#endif /* #ifndef IA_CORE_CODER_CHANNELINFO_H */
