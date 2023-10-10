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

#ifndef IMPEGHD_OBJ_REN_DEC_STRUCT_DEF_H
#define IMPEGHD_OBJ_REN_DEC_STRUCT_DEF_H

/*
Each Triangle is formed by a set of 3 speakers.
Each speaker is associated with an index value.
The triangle is identified using the same
*/
typedef struct
{
  WORD32 spk_index[3];
} ia_renderer_ls_triangle;

typedef struct
{
  FLOAT32 spk_azi;
  WORD32 spk_order;
} ia_spk_azi_idx_str;

typedef struct
{
  FLOAT32 ls_azimuth;
  FLOAT32 ls_elevation;
  WORD32 ls_index;
  WORD32 ls_order;
  ia_cart_coord_str ls_cart_coord;
  FLOAT32 offset;
} ia_renderer_ls_params;

typedef struct
{
  WORD32 ls_index;
  WORD32 ls_order;
} ia_ls_ord_idx_params;

typedef struct
{
  WORD32 ls_order_vtx_a;
  WORD32 ls_order_vtx_b;
} ia_ls_edge_params;

typedef struct
{
  ia_cart_coord_str row[3];
} ia_renderer_ls_inv_mtx;

typedef struct
{
  FLOAT32 radius_descaled[MAX_NUM_OAM_OBJS];
  FLOAT32 elevation_descaled[MAX_NUM_OAM_OBJS];
  FLOAT32 azimuth_descaled[MAX_NUM_OAM_OBJS];
  FLOAT32 gain_descaled[MAX_NUM_OAM_OBJS];
  FLOAT32 spread_params[MAX_NUM_OAM_OBJS][OAM_MAX_SPREAD_PARAMS];
} ia_renderer_prev_frame_params;

typedef struct
{
  WORD32 tri_ls_index[CICP_MAX_NUM_LS];
  WORD32 height_spks_present;
  WORD32 cicp_idx_present;
  WORD32 cicp_out_idx;
  WORD32 num_cicp_speakers;
  WORD32 num_non_lfe_ls;
  WORD32 num_imag_ls;
  WORD32 num_init_polyhedron_vtx;
  WORD32 num_vertices;
  WORD32 num_triangles;
  WORD32 first_frame_flag[MAX_NUM_OAM_OBJ_ENTITIES_PER_FRAME];
  FLOAT32 downmix_mtx[CICP_MAX_NUM_LS][CICP_MAX_NUM_LS];
  FLOAT32 initial_gains[MAX_NUM_OAM_OBJ_ENTITIES_PER_FRAME][CICP_MAX_NUM_LS];
  FLOAT32 final_gains[CICP_MAX_NUM_LS];
  FLOAT32 ls_gains[CICP_MAX_NUM_LS];

  ia_cicp_ls_geo_str *ptr_cicp_ls_geo[CICP_MAX_NUM_LS];
  ia_oam_dec_state_struct str_obj_md_dec_state;
  ia_renderer_ls_params non_lfe_ls_str[CICP_MAX_NUM_LS];
  ia_renderer_ls_triangle ls_triangle[CICP_MAX_NUM_LS];
  ia_renderer_ls_inv_mtx ls_inv_mtx[CICP_MAX_NUM_LS];

  // Scratch buffers
  ia_ls_edge_params edge_params[CICP_MAX_NUM_LS];
  ia_ls_edge_params non_dup_edges[CICP_MAX_NUM_LS];
  WORD32 num_non_dup_edges;
  ia_local_setup_struct *pstr_local_setup;
  ia_cicp_ls_geo_str array_cicp_ls_geo[CICP_MAX_NUM_LS];
} ia_obj_ren_dec_state_struct;

#endif /* IMPEGHD_OBJ_REN_DEC_STRUCT_DEF_H */