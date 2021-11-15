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

#ifndef IMPEGHD_MP4_PARSER_H
#define IMPEGHD_MP4_PARSER_H

#define MAE_BUFF_SIZE 768

typedef struct
{
  pUWORD8 frame;
  UWORD32 frame_length;
  UWORD32 presentation_time;
} it_mp4_frame_cntxt;

typedef struct
{
  pUWORD8 header;
  UWORD32 header_length;
} it_mp4_header_cntxt;

pVOID impeghd_mp4_parser_init(pVOID mp4_cntxt);
WORD32 impeghd_mp4_get_audio_header(pVOID mp4_cntxt, pVOID audioheader);
WORD32 impeghd_mp4_stsz_valid(pVOID mp4_cntxt);
WORD32 impeghd_mp4_get_audio(pVOID mp4_cntxt, pVOID frame_cntxt);
WORD32 impeghd_mp4_parser_close(pVOID mp4);
WORD32 impeghd_mp4_get_datamp4(VOID *mp4_cntxt, WORD32 *offset, pUWORD8 buffer, WORD32 buf_size,
                               UWORD32 *length, WORD32 *size, WORD32 *loc);

#endif /* IMPEGHD_MP4_PARSER_H */
