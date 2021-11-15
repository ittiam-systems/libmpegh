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

#ifndef __IMPEGHD_HOA_DECODER_H__
#define __IMPEGHD_HOA_DECODER_H__

typedef struct impeghd_hoa_dec_struct_t
{
  ia_spatial_dec_str spat_dec_handle;
  ia_render_hoa_str hoa_renderer;

  ia_hoa_config_struct *ia_hoa_config;
  ia_hoa_frame_struct *ia_hoa_frame;
  ia_bit_buf_struct *ia_bit_buf;

  pVOID ptr_scratch;
  UWORD32 frame_length;
  UWORD32 spat_delay_frames;
  WORD32 is_brir_rendering;

} impeghd_hoa_dec_struct;

IA_ERRORCODE impeghd_hoa_dec_init(impeghd_hoa_dec_struct *dec_handle, WORD32 spk_idx,
                                  ia_speaker_config_3d *ref_spk_layout, UWORD32 samp_freq,
                                  pVOID scratch, UWORD32 mpegh_profile_lvl);
IA_ERRORCODE impeghd_hoa_dec_decode(impeghd_hoa_dec_struct *dec_handle, pFLOAT32 ptr_out_buf,
                                    pFLOAT32 ptr_in_buf, pFLOAT32 ptr_prev_in_buf,
                                    pWORD32 num_out_ch, WORD32 ds_flag);
#endif /* __IMPEGHD_HOA_DECODER_H__ */
