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

#ifndef IMPEGHD_MEMORY_STANDARDS_H
#define IMPEGHD_MEMORY_STANDARDS_H

/*****************************************************************************/
/* type definitions                                                          */
/*****************************************************************************/
/* standard memory table descriptor for libraries */
typedef struct
{
  UWORD32 ui_size;         /* size of the memory in bytes   */
  UWORD32 ui_alignment;    /* alignment in bytes            */
  UWORD32 ui_type;         /* type of memory                */
  UWORD32 ui_placement[2]; /* 64 bit placement info     */
  UWORD32 ui_priority;     /* the importance for placement  */
  UWORD32 ui_placed[2];    /* the o_red location for placement  */
} ia_mem_info_struct;

typedef struct
{
  UWORD32 ui_size;
  UWORD32 ui_alignment;
  UWORD32 ui_type;
  pVOID mem_ptr;
} ia_mem_info_table;

typedef struct
{
  UWORD32 ui_pcm_wd_sz;
  WORD32 ui_effect;
  WORD32 ui_target_loudness[2];
  WORD32 ui_mhas_flag;
  WORD32 ui_raw_flag;
  WORD32 ui_cicp_layout_idx;
  WORD8 i_preset_id;
  WORD32 num_inp_bytes;
  WORD32 lsi_info_flag; // Local set up information.
  WORD32 ei_info_flag;  // Element interaction.
  WORD32 sd_info_flag;  // Scene displacement.
  WORD32 lsi_info_size; // Local set up information.
  WORD32 ei_info_size;  // Element interaction.
  WORD32 sd_info_size;  // Scene displacement.
  WORD32 extrn_rend_flag;
  UWORD8 *ptr_ei_buf;
  UWORD8 *ptr_ls_buf;
  UWORD8 *ptr_sd_buf;
  UWORD8 *ptr_ext_ren_ch_data_buf;
  UWORD8 *ptr_ext_ren_oam_data_buf;
  UWORD8 *ptr_ext_ren_hoa_data_buf;
  UWORD8 *ptr_ext_ren_pcm_buf;
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
  WORD32 binaural_flag;
  WORD32 binaural_data_len;
  UWORD8 *ptr_brir_buf;
  WORD32 enable_resamp;
  UWORD32 out_samp_freq;
} ia_input_config;

typedef struct
{
  WORD32 i_samp_freq;
  WORD32 i_num_chan;
  WORD32 i_pcm_wd_sz;
  WORD32 i_drc_effect;
  WORD32 i_target_loudness;
  WORD32 i_loud_norm;
  WORD32 num_preroll;

  UWORD32 ui_init_done;

  WORD32 num_out_bytes;
  WORD32 i_bytes_consumed;
  UWORD32 ui_inp_buf_size;

  UWORD32 malloc_count;
  UWORD32 ui_rem;
  UWORD32 ui_proc_mem_tabs_size;

  pVOID pv_ia_process_api_obj;
  pVOID arr_alloc_memory[100];

  WORD8 *p_lib_name;
  WORD8 *p_version_num;

  pVOID (*malloc_mpegh)(UWORD32, UWORD32);
  VOID (*free_mpegh)(pVOID);
  pVOID (*de_malloc_xc)(pVOID);

  ia_mem_info_table mem_info_table[4];

  WORD32 oam_data_present;
  WORD32 oam_md_payload_length;
  WORD32 hoa_data_present;
  WORD32 hoa_md_payload_length;
  WORD32 ch_data_present;
  WORD32 ch_md_payload_length;
  WORD32 pcm_payload_length;
  WORD32 pcm_bit_depth;
  WORD32 hoa_sample_offset;
  WORD32 oam_sample_offset;

  WORD32 num_speakers;
  WORD32 spk_layout;
  WORD32 cicp_index;
  WORD32 is_binaural_rendering;
  WORD32 is_lfe[24];
  WORD16 azimuth[24];
  WORD16 elevation[24];
} ia_output_config;

typedef struct
{
  ia_input_config input_config;
  ia_output_config output_config;
} ia_mpeghd_api_struct;

/*****************************************************************************/
/* Constant hash defines                                                     */
/*****************************************************************************/

/* ittiam standard memory types */
/* to be used inter frames */
#define MEMTYPE_PERSIST 0x00
/* read write, to be used intra frames */
#define MEMTYPE_SCRATCH 0x01
/* read only memory, intra frame */
#define IA_MEMTYPE_INPUT 0x02
/* read-write memory, for usable output, intra frame */
#define IA_MEMTYPE_OUTPUT 0x03

#endif /* IMPEGHD_MEMORY_STANDARDS_H */
