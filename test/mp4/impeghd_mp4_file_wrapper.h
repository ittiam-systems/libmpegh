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

#ifndef IMPEGHD_MP4_FILE_WRAPPER_H
#define IMPEGHD_MP4_FILE_WRAPPER_H

typedef struct ia_file_wrapper
{
  pVOID mp4_cntxt;
  pVOID file_cntxt;
  pVOID interim_buffer;
  WORD32 avail_buffer;
  it_mp4_header_cntxt header_cntxt;
  it_mp4_frame_cntxt frame_cntxt;
  WORD32 header_given;
  UWORD32 is_mp4_file;
  UWORD32 is_mp4_mhm1;
  FILE *input_file;

} ia_file_wrapper;

ia_file_wrapper *impeghd_mp4_fw_open(WORD8 fileName[]);
WORD32 impeghd_mp4_fw_read(ia_file_wrapper *transport, pUWORD8 buffer, WORD32 bufSize,
                           pUWORD32 len);
UWORD32 impeghd_mp4_fw_close(ia_file_wrapper *transport);
UWORD32 impeghd_mp4_fw_is_mp4_file(ia_file_wrapper *transport);
pVOID impeghd_mp4_fopen(pUWORD8 file_name, UWORD8 with_file, WORD32 size);
WORD32 impeghd_mp4_fclose(pVOID itf);
WORD32 impeghd_mp4_parse_mae_boxes(ia_file_wrapper *g_pf_inp_str, pVOID ptr_dec_api);

#endif /* IMPEGHD_MP4_FILE_WRAPPER_H */
