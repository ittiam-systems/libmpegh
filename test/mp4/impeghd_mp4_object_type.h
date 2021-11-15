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

#ifndef IMPEGHD_MP4_OBJECT_TYPE_H
#define IMPEGHD_MP4_OBJECT_TYPE_H

/* ISO/IEC14496-1 ObjectTypeIndication Values */
#define Forbidden 0x00
#define MPEG4_SYSTEM1 0x01
#define MPEG4_SYSTEM2 0x02
#define MPEG4_AUDIO 0x40

#define MPEG4_VIDEO 0x20
#define MPEG2_VIDEO 0x61
#define MP3_AUDIO1 0x6B
#define ADPCM 0x41 /* not a valid format acc'ing to MPEG4 standrd */
#define IMA_ADPCM 0x42
#define MP3_AUDIO2 0x69

#define CHECK_AUDIO(x)                                                                           \
  (((x) == MPEG4_AUDIO) || ((x) == MP3_AUDIO1) || ((x) == MP3_AUDIO2) || ((x) == ADPCM) ||       \
   (x) == IMA_ADPCM)

#define CHECK_VIDEO(x) (((x) == MPEG4_VIDEO) || ((x) == MPEG2_VIDEO))

/* ISO/IEC14496-1 StreamType Values */
/*#define Forbidden           0x00 */
#define ODSM 0x01
#define CRSM 0x02
#define SDSM 0x03
#define VISUAL 0x04
#define AUDIO 0x05
#define MPEG7 0x06
#define IPMP_STREAM 0x07
#define OCI_STREAM 0x08
#define MPEGJ 0x09

/* Non Standrad declaration used for hint */
#define MPEG4_HINT_VIDEO 4
#define MPEG4_HINT_AUDIO 5

#endif /* IMPEGHD_MP4_OBJECT_TYPE_H */
