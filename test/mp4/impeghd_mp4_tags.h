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

#ifndef IMPEGHD_MP4_TAGS_H
#define IMPEGHD_MP4_TAGS_H

#define FORBIDDEN_TAG 0x00
#define OBJECT_DESC_TAG 0x01
#define INITIAL_OBJECT_DESC_TAG 0x02
#define ES_DESC_TAG 0x03
#define DEC_CONFIG_DESC_TAG 0x04
#define DEC_SPECIFIC_INFO_TAG 0x05
#define SL_CONFIG_DESC_TAG 0x06
#define CONTENT_IDENT_DESC_TAG 0x07
#define SUPPL_CONTENT_IDENT_DESC_TAG 0x08
#define IPI_DESC_POINTER_TAG 0x09
#define IPMP_DESC_POINTER_TAG 0x0a
#define IPMP_DESC_TAG 0x0b
#define QOS_DESC_TAG 0x0c
#define REG_DESC_TAG 0x0d
#define ES_ID_INC_TAG 0x0e
#define ES_ID_REF_TAG 0x0f
#define MP4_IOD_TAG 0x10
#define MP4_OD_TAG 0x11
#define IPL_DESC_POINTER_REF_TAG 0x12
#define EXTENDED_PROF_LVL_DESC_TAG 0x13
#define PROF_LVL_INDICATION_INDEX_DESC_TAG 0x14
#define CONTENT_CLASSIFICATION_DESC_TAG 0x40
#define KEY_WORD_DESC_TAG 0x41
#define RATING_DESC_TAG 0x42
#define LANG_DESC_TAG 0x43
#define SHORT_TEXTUAL_DESC_TAG 0x44
#define EXPANDED_TEXTUAL_DESC_TAG 0x45
#define CONTENT_CREATOR_NAME_DESC_TAG 0x46
#define CONTENT_CREATION_DATE_DESC_TAG 0x47
#define OCI_CREATOR_NAME_DESC_TAG 0x48
#define OCI_CREATION_DATE_DESC_TAG 0x49
#define SMPTE_CAMERA_POS_DESC_TAG 0x4a
#define OBJECT_DESC_UPDATE_TAG 0x01
#define OBJ_DESC_REMOVE_TAG 0x02
#define ES_DESC_UPDATE_TAG 0x03
#define ES_DESC_REMOVE_TAG 0x04
#define IPMP_DESC_UPDATE_TAG 0x05
#define IPMP_DESC_REMOVE_TAG 0x06
#define ES_DESC_REMOVE_REF_TAG 0x07

#endif /* IMPEGHD_MP4_TAGS_H */
