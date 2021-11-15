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

#ifndef IMPEGHD_MP4_PROTO_H
#define IMPEGHD_MP4_PROTO_H

typedef struct n
{
  pVOID data;
  char descr[5];
  struct n *parent;
  struct n *sibling;
  struct n *child;
  struct n *prev;
} ia_mp4_node;

typedef enum { MP4_FORMAT, MOV_FORMAT } ia_file_format;
enum tref_ref_type
{
  TREF_HINT,
  TREF_DPND,
  TREF_IPIR,
  TREF_MPOD,
  TREF_SYNC
};

typedef struct ia_mp4_tref_ref
{
  enum tref_ref_type ref_type;
  UWORD16 count;
  UWORD32 *track_id;
} ia_mp4_tref_ref;

typedef struct a
{
  UWORD32 trak_id;
  UWORD32 trak_type;
  UWORD32 handler_type;
  WORD32 time_scale;
  WORD32 duration;
  struct ia_mp4_tref_ref tref;
  ia_mp4_udta_info *udta_info;
  UWORD32 stbl_offset;
  struct a *next;
} ia_mp4_trak_init;

typedef struct
{
  UWORD32 count;
  WORD32 index;
  UWORD32 offset;
  WORD32 read;
  UWORD16 total;
} ia_mp4_buf_ctxt;

typedef struct
{
  UWORD32 trak_id;
  UWORD32 es_id;
  UWORD32 trak_type;
  UWORD32 stream_type;
  WORD32 time_scale;
  WORD32 duration;
  UWORD32 sample_type;
  UWORD32 object_type;
  ia_mp4_buf_ctxt stco_cntxt;
  UWORD32 stco_entries[IT_MP4_BUF_SIZE];
  UWORD32 samples_read_curr_chunk;
  UWORD32 stsc_count;
  WORD32 stsc_index;
  ia_mp4_chunk_entry stsc_entries[IT_MP4_BUF_SIZE];
  WORD32 read_stsc_entries;
  UWORD16 total_entries_in_stsc_buffer;
  UWORD32 samples_last_stsc_entry;
  UWORD32 stsc_offset;
  UWORD32 stss_count;
  WORD32 stss_index;
  UWORD32 stss_entries[IT_MP4_BUF_SIZE];
  WORD32 read_stss_entries;
  UWORD16 total_entries_in_stss_buffer;
  UWORD32 stss_offset;
  UWORD32 stts_count;
  WORD32 stts_index;
  decoding_time stts_entries[IT_MP4_BUF_SIZE];
  WORD32 read_stts_entries;
  UWORD32 sample_read_last_stts_entry;
  UWORD16 total_entries_in_stts_buffer;
  UWORD32 stts_offset;
  UWORD32 ctts_count;
  WORD32 ctts_index;
  ia_mp4_composition_time ctts_entries[IT_MP4_BUF_SIZE];
  WORD32 read_ctts_entries;
  UWORD32 sample_read_last_ctts_entry;
  UWORD16 total_entries_in_ctts_buffer;
  UWORD32 ctts_offset;
  UWORD32 sample_size;
  UWORD32 stsz_count;
  WORD32 stsz_index;
  UWORD32 stsz_entries[IT_MP4_BUF_SIZE];
  WORD32 read_stsz_entries;
  UWORD32 stsz_offset;
  UWORD16 total_entries_in_stsz_buffer;
  UWORD32 dec_info_length;
  pUWORD8 dec_info;
  UWORD32 avg_bitrate;
  ia_mp4_sample_desc_atom *stsd_atom;
  UWORD32 read_incurrent_chunk;
  UWORD32 read_incurrent_sample;
  UWORD32 stbl_offset;
  WORD32 stbl_size;
  UWORD32 current_trak;
  UWORD8 end_of_trak;
  UWORD32 handler_type;
  struct ia_mp4_tref_ref tref;
  ia_mp4_udta_info *udta_info;
  UWORD32 frame_num;
  it_uint64 elap_time;
} trak_info;
typedef struct 
{
  UWORD8 *ptr_maei_buf;
  UWORD8 *ptr_maeg_buf;
  UWORD8 *ptr_maes_buf;
  UWORD8 *ptr_maep_buf;
  WORD32 maei_flag;
  WORD32 maeg_flag;
  WORD32 maes_flag;
  WORD32 maep_flag;
  WORD32 maei_len;
  WORD32 maeg_len;
  WORD32 maes_len;
  WORD32 maep_len;
  WORD8 init;
}maei_info;
typedef struct
{
  ia_mp4_node *root;
  ia_mp4_trak_init *trak_init_info;
  trak_info *imp_trak_info[6];
  ia_mp4_mem_node *ptr_mem;
  pVOID fp;
  WORD32 time_scale;
  WORD32 duration;
  ia_mp4_udta_info *udta_info;
  WORD32 trak_end_offset;
  ia_file_format file_type;
  maei_info st_maei_info;
} mp4_info;

WORD32 impeghd_mp4_fill_imp_trak_info(pVOID fp, trak_info *m, UWORD32 req_child,
                                      ia_mp4_mem_node **n, maei_info *ptr_mae_info);
WORD32 impeghd_mp4_get_needed_trak(trak_info *m, ia_mp4_trak_init **n, UWORD32 mediatype);
VOID impeghd_mp4_clear_buffer(trak_info *m);
WORD32 impeghd_mp4_init_wrap(mp4_info *m);
WORD32 impeghd_mp4_init(mp4_info *m);
VOID impeghd_mp4_free_all_nodes(ia_mp4_mem_node **m);

#endif /* IMPEGHD_MP4_PROTO_H */
