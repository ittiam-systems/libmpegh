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

#ifndef IMPEGHD_MP4_ATOMS_H
#define IMPEGHD_MP4_ATOMS_H

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
} ia_mp4_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
} ia_mp4_full_atom;

typedef ia_mp4_atom ia_mp4_movie_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  pUWORD8 data;
} ia_mp4_media_data_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  union info {
    struct
    {
      UWORD32 creation_time;
      UWORD32 modification_time;
      UWORD32 time_scale;
      UWORD32 duration;
    } version1;
    struct
    {
      UWORD32 creation_time;
      UWORD32 modification_time;
      UWORD32 time_scale;
      UWORD32 duration;
    } version0;
  } uinfo;

  UWORD32 reserved1;
  UWORD16 reserved2;
  UWORD16 reserved3;
  UWORD32 reserved4[2];
  UWORD32 reserved5[9];
  UWORD32 reserved6[6];
  UWORD32 next_track_ID;
} ia_mp4_movie_header_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  ia_mp4_obj_desc od;
  ia_mp4_initial_obj_desc iod;
} ia_mp4_obj_desc_atom;

typedef ia_mp4_atom ia_mp4_trak_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  union nfo {
    struct
    {
      UWORD32 creation_time;
      UWORD32 modification_time;
      UWORD32 track_id;
      UWORD32 creserved;
      UWORD32 duration;
    } version1;
    struct
    {
      UWORD32 creation_time;
      UWORD32 modification_time;
      UWORD32 track_id;
      UWORD32 creserved;
      UWORD32 duration;
    } version0;
  } unfo;

  UWORD32 reserved1[3];
  UWORD16 reserved2;
  UWORD16 reserved3;
  UWORD32 reserved4[9];
  UWORD32 reserved5;
  UWORD32 reserved6;
} ia_mp4_trak_header_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 *track_Ids;
} ia_mp4_trak_ref_type_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  ia_mp4_trak_ref_type_atom ref_type;
} ia_mp4_trak_ref_atom;

typedef ia_mp4_atom ia_mp4_media_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  union fo {
    struct
    {
      UWORD32 creation_time;
      UWORD32 modification_time;
      UWORD32 time_scale;
      UWORD32 duration;
    } version1;
    struct
    {
      UWORD32 creation_time;
      UWORD32 modification_time;
      UWORD32 time_scale;
      UWORD32 duration;
    } version0;
  } ufo;
  UWORD32 pad : 1;
  UWORD32 lang : 15;
  UWORD16 reserved;
} ia_mp4_media_header_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 handler_type;
  UWORD32 reserved1;
  UWORD8 reserved2[12];
  UWORD8 *name;
} ia_mp4_handler_ref_atom;

typedef ia_mp4_atom ia_mp4_media_info_atom;

typedef ia_mp4_atom ia_mp4_data_info_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 entry_count;
} ia_mp4_data_ref_atom;

typedef ia_mp4_atom ia_mp4_sample_table_atom;

typedef struct
{
  UWORD32 sample_count;
  WORD32 sample_delta;
} decoding_time;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 entry_count;
  decoding_time *next_entry;
} ia_mp4_time_to_sample_atom;

typedef struct
{
  UWORD32 sample_count;
  WORD32 sample_offset;
} ia_mp4_composition_time;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 entry_count;
  ia_mp4_composition_time *next_entry;
} ia_mp4_composition_time_to_sample_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  ia_mp4_mh_desc *mh_descr;
} ia_mp4_mhac_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD8 maei_read_buf[6144 / 8];
} ia_mp4_maei_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
} ia_mp4_maeg_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
} ia_mp4_maes_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
} ia_mp4_maep_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
} ia_mp4_mael_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  ia_mp4_es_desc *es_descr;
} esd_atom;

typedef struct se
{
  struct
  {
    UWORD32 size;
    UWORD32 type;
    UWORD32 large_size;
    UWORD8 reserved[6];
    UWORD16 data_ref_idx;
  } sample;

  union {
    struct
    {
      UWORD32 reserved1[2];
      UWORD16 reserved2;
      UWORD16 reserved3;
      UWORD32 reserved4;
      UWORD16 time_scale;
      UWORD16 reserved5;
    } audio;
    struct
    {
      UWORD32 reserved1[4];
      UWORD32 reserved2;
      UWORD32 reserved3;
      UWORD32 reserved4;
      UWORD32 reserved5;
      UWORD16 reserved6;
      UWORD8 reserved7[32];
      UWORD16 reserved8;
      WORD16 reserved9;
    } video;
    struct
    {
      UWORD8 *data;
      UWORD16 ht_ver;
      UWORD16 last_compatible_ht_ver;
      UWORD32 max_packet_size;
      UWORD32 tims;
      UWORD32 tsro;
      UWORD32 snro;
      UWORD8 rely;
    } hint;
  } stream;
  struct se *ptr_next_sample_entry;
  esd_atom es;
  ia_mp4_mhac_atom mh;
  ia_mp4_maei_atom maei;
  ia_mp4_maeg_atom maeg;
  ia_mp4_maes_atom maes;
  ia_mp4_maep_atom maep;
  ia_mp4_mael_atom mael;
} ia_mp4_sample_entry;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 entry_count;
  ia_mp4_sample_entry *ptr_sample_entry;
} ia_mp4_sample_desc_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 sample_size;
  UWORD32 sample_count;
  UWORD32 *entry_size;
} ia_mp4_sample_size_atom;

typedef struct
{
  UWORD32 first_chunk;
  UWORD32 samples_per_chunk;
  UWORD32 sample_descr_index;
} ia_mp4_chunk_entry;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 entry_count;
  ia_mp4_chunk_entry *chunk;
} ia_mp4_sample_to_chunk_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 reserved;
} ia_mp4_video_media_header_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 reserved;
} ia_mp4_sound_media_header_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD16 max_pdu_size;
  UWORD16 avg_pdu_size;
  UWORD32 max_bitrate;
  UWORD32 avg_bitrate;
  UWORD32 sliding_avg_bitrate;
} ia_mp4_hint_media_header_atom;

typedef ia_mp4_full_atom ia_mp4_mpeg_media_header_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 entry_count;
  UWORD32 *chunk_offset;
} ia_mp4_chunk_offset_atom;

typedef struct
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 version : 8;
  UWORD32 flags : 24;
  UWORD32 entry_count;
  UWORD32 *sample_number;
} ia_mp4_sync_sample_atom;

typedef ia_mp4_atom ia_mp4_free_atom;

typedef ia_mp4_atom ia_mp4_skip_atom;

typedef struct ia_mp4_udta_name_atom
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD8 *tracks_name;
} ia_mp4_udta_name_atom;

typedef struct ia_mp4_udta_hinf_atom
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD32 trpy;
  UWORD32 nump;
  UWORD32 tpyl;
  UWORD32 maxr_g;
  UWORD32 maxr_m;
  UWORD32 dmed;
  UWORD32 dimm;
  UWORD32 drep;
  UWORD32 tmin;
  UWORD32 tmax;
  UWORD32 pmax;
  UWORD32 dmax;
  UWORD32 payt;
  UWORD8 pay_str_count;
  UWORD8 *pay_str;
} ia_mp4_udta_hinf_atom;

typedef struct ia_mp4_hnti_sdp_atom
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  UWORD8 *sdp_info;
} ia_mp4_hnti_sdp_atom;

typedef struct ia_mp4_udta_hnti_atom
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  struct ia_mp4_hnti_sdp_atom *hnti_sdp;
} ia_mp4_udta_hnti_atom;

typedef struct ia_mp4_udta_info
{
  ia_mp4_udta_name_atom *name;
  ia_mp4_udta_hinf_atom *hinf;
  ia_mp4_udta_hnti_atom *hnti;
} ia_mp4_udta_info;

typedef struct ia_mp4_udta_atom
{
  UWORD32 size;
  UWORD32 type;
  UWORD32 large_size;
  ia_mp4_udta_info udta_info;
} ia_mp4_udta_atom;

#endif /* IMPEGHD_MP4_ATOMS_H */
