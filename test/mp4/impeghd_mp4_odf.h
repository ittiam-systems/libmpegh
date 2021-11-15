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

#ifndef IMPEGHD_MP4_ODF_H
#define IMPEGHD_MP4_ODF_H

typedef UWORD8 ia_mp4_oci_desc;

typedef UWORD8 ia_mp4_extension_desc;

typedef struct
{
  UWORD32 format_identifier;
  UWORD8 *additional_identification_info;
} ia_mp4_registration_desc;

typedef UWORD8 ia_mp4_qos_qualifier;

typedef struct
{
  UWORD8 predefined;
  ia_mp4_qos_qualifier *qualifiers;
} ia_mp4_qos_desc;

typedef struct
{
  UWORD32 lang_code : 24;
} ia_mp4_language_desc;

typedef struct n1
{
  UWORD8 tag;
  UWORD32 length;
  UWORD8 ipmp_desc_id;
  struct n1 *next;
} ia_mp4_ipmp_desc_pointer;

typedef UWORD8 ia_mp4_ip_identification_data_set;

typedef struct
{
  UWORD8 tag;
  UWORD32 length;
  UWORD16 ipi_es_id;
} ia_mp4_ipi_desc_pointer;

typedef struct
{
  UWORD8 tag;
  UWORD32 length;
  UWORD8 predefined;
  UWORD32 use_access_unit_start_flag : 1;
  UWORD32 use_access_unit_end_flag : 1;
  UWORD32 use_random_access_point_flag : 1;
  UWORD32 has_rand_access_units_only_flag : 1;
  UWORD32 use_padding_flag : 1;
  UWORD32 use_time_stamps_flag : 1;
  UWORD32 use_idle_flag : 1;
  UWORD32 duration_flag : 1;
  UWORD32 time_stamp_res;
  UWORD32 ocr_res;
  UWORD8 time_stamp_len;
  UWORD8 ocr_len;
  UWORD8 au_len;
  UWORD8 instant_bitrate_len;
  UWORD32 degradation_priority_length : 4;
  UWORD32 au_seq_num_len : 5;
  UWORD32 packet_seq_num_len : 5;
  UWORD32 reserved : 2;
  UWORD32 time_scale;
  UWORD16 access_unit_duration;
  UWORD16 composition_unit_duration;
  UWORD32 start_decoding_time_stamp;
  UWORD32 start_composition_time_stamp;
} ia_mp4_sl_config_desc;

typedef struct pl
{
  UWORD8 tag;
  UWORD8 profile_lvl_indication_idx;
  struct pl *next_profile_lvl_indication_entry;
} ia_mp4_profile_lvl_indication_idc_desc;

typedef struct
{
  UWORD8 tag;
  UWORD32 length;
  UWORD8 *dec_info;
} ia_mp4_decoder_specific_info;

typedef struct
{
  UWORD8 tag;
  UWORD32 length;
  UWORD8 obj_type_indication;
  UWORD32 stream_type : 6;
  UWORD32 up_stream : 1;
  UWORD32 reserved : 1; /* =1 */
  UWORD32 buf_size_db : 24;
  UWORD32 max_bitrate;
  UWORD32 avg_bitrate;
  ia_mp4_decoder_specific_info dec_specific_info;
  ia_mp4_profile_lvl_indication_idc_desc *profile_lvl_indication_idx_desc;
} ia_mp4_decoder_config_desc;

typedef struct mh_descr
{
  UWORD32 mpegh_3da_profile_lvl_indication : 8;
  UWORD32 ref_ch_layout : 8;
  UWORD32 mpegh_3da_config_length : 16;
  UWORD32 mpegh_3da_config;
  ia_mp4_decoder_specific_info dec_specific_info;
} ia_mp4_mh_desc;

typedef struct es_descr
{
  UWORD8 tag;
  UWORD32 length;
  UWORD16 es_id;
  UWORD32 stream_dependence_flag : 1;
  UWORD32 url_flag : 1;
  UWORD32 ocr_stream_flag : 1;
  UWORD32 stream_priority : 5;
  UWORD16 depends_on_es_id;
  UWORD8 url_len;
  UWORD8 *url_string;
  UWORD8 *url_string_media;
  UWORD16 ocr_es_id;
  struct odu *obj_desc_update;

  ia_mp4_decoder_config_desc dec_config_desc;
  ia_mp4_sl_config_desc sl_config_desc;
  ia_mp4_ipi_desc_pointer ipi_point;
  ia_mp4_ipmp_desc_pointer *ipmp_desc_ptr;
  ia_mp4_ip_identification_data_set *ip_ids_ptr;
  ia_mp4_language_desc *lang_desc_ptr;
  ia_mp4_qos_desc *qos_descr_ptr;
  ia_mp4_registration_desc *reg_descr_ptr;
  ia_mp4_extension_desc *ext_desc_ptr;
  struct es_descr *next;
} ia_mp4_es_desc;

typedef struct es_id_inc
{
  UWORD8 tag; /* ES_ID_INC_TAG */
  UWORD32 length;
  UWORD32 track_id;
  struct es_id_inc *next;
} ia_mp4_es_id_inc_desc;

typedef struct es_id_ref
{
  UWORD8 tag;
  UWORD32 length;
  UWORD16 ref_index;
  struct es_id_ref *next;
} ia_mp4_es_id_ref_desc;

typedef struct obj_descr
{
  UWORD8 tag;
  UWORD32 length;
  UWORD32 obj_descriptor_id : 10;
  UWORD32 url_flag : 1;
  UWORD32 reserved : 5;

  union {
    struct
    {
      UWORD8 url_len;
      UWORD8 *url_string;
    } url;
    struct
    {
      ia_mp4_es_id_ref_desc *es_id_ref;
      ia_mp4_es_desc *es_desc_ptr;
      ia_mp4_oci_desc *oci_desc_ptr;
      ia_mp4_ipmp_desc_pointer *ipmp_desc_ptr;
    } descr;
  } od;
  ia_mp4_extension_desc ext_desc_ptr;
  struct obj_descr *next;
} ia_mp4_obj_desc;

typedef struct
{
  UWORD8 tag;
  UWORD32 length;
  UWORD32 obj_descriptor_id : 10;
  UWORD32 url_flag : 1;
  UWORD32 inc_inline_profile_flag : 1;
  UWORD32 reserved : 4;

  struct
  {
    struct
    {
      UWORD8 url_len;
      UWORD8 *url_string;
    } url;
    struct
    {
      UWORD8 od_profile_lvl_indication;
      UWORD8 scene_profile_lvl_indication;
      UWORD8 audio_profile_lvl_indication;
      UWORD8 visual_profile_lvl_indication;
      UWORD8 graphics_profile_lvl_indication;
      ia_mp4_es_id_inc_desc *es_id_inc;
      ia_mp4_es_desc *es_desc_ptr;
      ia_mp4_oci_desc *oci_desc_ptr;
      ia_mp4_ipmp_desc_pointer *ipmp_desc_ptr;
    } profile;
  } iod;
  ia_mp4_extension_desc ext_desc_ptr;
} ia_mp4_initial_obj_desc;

typedef struct odu
{
  UWORD8 tag;
  UWORD32 length;
  ia_mp4_obj_desc *od;
} ia_mp4_obj_desc_update;

typedef struct
{
  UWORD16 obj_desc_id;
  UWORD16 ref_track;
} ia_mp4_es_id_ref_descr;

#endif /* IMPEGHD_MP4_ODF_H */
