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

#ifndef IMPEGHD_OAM_DEC_STRUCT_DEF_H
#define IMPEGHD_OAM_DEC_STRUCT_DEF_H

typedef struct
{
  ia_enh_oam_config_struct *p_enh_obj_md_cfg;
  WORD32 oam_has_been_decoded;
  WORD32 keep_diffuseness[MAX_NUM_OAM_OBJS];
  WORD32 keep_exclusion[MAX_NUM_OAM_OBJS];
  WORD32 keep_divergence[MAX_NUM_OAM_OBJS];

  WORD32 closest_spkr_playout[MAX_NUM_OAM_OBJS];
  WORD32 num_exclusion_sectors[MAX_NUM_OAM_OBJS];
  WORD32 use_predefined_sector[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  WORD32 exclude_sector_index[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 diffuseness[MAX_NUM_OAM_OBJS];
  FLOAT32 divergence[MAX_NUM_OAM_OBJS];
  FLOAT32 exclude_sector_min_az[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 exclude_sector_max_az[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 exclude_sector_min_ele[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 exclude_sector_max_ele[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];

  WORD32 prev_has_divergence[MAX_NUM_OAM_OBJS];
  WORD32 prev_num_exclusion_sectors[MAX_NUM_OAM_OBJS];
  WORD32 prev_use_predefined_sector[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  WORD32 prev_exclude_sector_index[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 prev_diffuseness[MAX_NUM_OAM_OBJS];
  FLOAT32 prev_divergence[MAX_NUM_OAM_OBJS];
  FLOAT32 prev_exclude_sector_min_az[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 prev_exclude_sector_max_az[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 prev_exclude_sector_min_ele[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
  FLOAT32 prev_exclude_sector_max_ele[MAX_NUM_OAM_OBJS][MAX_NUM_EXCLUDED_SECTORS];
} ia_enh_obj_md_frame_str;

typedef struct
{
  ia_oam_dec_config_struct *p_obj_md_cfg;
  WORD32 has_obj_md[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 sub_frame_obj_md_present[MAX_OAM_FRAMES];
  WORD32 num_objects;
  WORD32 sub_frame_number;
  WORD32 intra_frame_period;
  WORD32 sample[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 azimuth[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 elevation[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 radius[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 gain[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 spread_width[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 spread_height[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 spread_depth[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  WORD32 dyn_obj_priority[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 azimuth_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 elevation_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 radius_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 gain_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 spread_width_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 spread_height_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 spread_depth_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLOAT32 dyn_obj_priority_descaled[MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS];
  FLAG azimuth_fixed;
  FLAG elevation_fixed;
  FLAG radius_fixed;
  FLAG gain_fixed;
  FLAG spread_fixed;
  FLAG dynamic_obj_priority_fixed;
} ia_oam_dec_state_struct;

#endif /* IMPEGHD_OAM_STRUCT_DEF_H */
