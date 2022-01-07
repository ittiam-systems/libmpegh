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

#include <stdio.h>
#include <stdlib.h>
#include <impeghd_type_def.h>
#include "impeghd_mp4_parser.h"
#include "impeghd_mp4_defs.h"
#include "impeghd_mp4_odf.h"
#include "impeghd_mp4_atoms.h"
#include "impeghd_mp4_file.h"

#include "impeghd_mp4_utils.h"
#include "impeghd_mp4_object_type.h"
#include "impeghd_mp4_proto.h"
#include "impeghd_mp4_init.h"
#include "impeghd_mp4_make_video.h"

/**
 * @defgroup MP4Parser MP4 Parser Utility
 * @ingroup  MP4Parser
 * @brief MP4 Parser Utility
 *
 * @{
 */

/**
*  impeghd_mp4_parser_init
*
*  \brief Initializes mp4 parser
*
*  \param [in] fp Pointer to file context
*
*  \return pVOID
*
*/
pVOID impeghd_mp4_parser_init(pVOID fp)
{
  WORD32 ret;
  mp4_info *m_mp4;

  m_mp4 = (mp4_info *)impeghd_mp4_malloc_wrapper(sizeof(mp4_info));
  m_mp4->imp_trak_info[0] = NULL;
  m_mp4->imp_trak_info[1] = NULL;
  m_mp4->fp = (pVOID)fp;

  ret = impeghd_mp4_init_wrap(m_mp4);
  if (ret != IT_OK)
  {
    impeghd_mp4_free_wrapper((pVOID)m_mp4);
    return (NULL);
  }
  else
  {
    return ((pVOID)m_mp4);
  }
}

/**
*  impeghd_mp4_get_audio_header
*
*  \brief Gets audio header
*
*  \param [in] mp4_cntxt      Pointer to mp4 context
*  \param [in,out] aud_header Pointer to audio header context
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_get_audio_header(pVOID mp4_cntxt, pVOID aud_header)
{
  mp4_info *m_mp4 = (mp4_info *)mp4_cntxt;
  it_mp4_header_cntxt *audioheader = (it_mp4_header_cntxt *)aud_header;
  if (m_mp4->imp_trak_info[1])
  {
    audioheader->header_length =
        impeghd_mp4_read_header_info(audioheader->header, m_mp4->imp_trak_info[1]);
    return IT_OK;
  }
  return IT_ERROR;
}

/**
*  impeghd_mp4_stsz_valid
*
*  \brief Validates stsz
*
*  \param [in] mp4_cntxt      Pointer to mp4 context
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_stsz_valid(pVOID mp4_cntxt)
{
  mp4_info *m_mp4 = (mp4_info *)mp4_cntxt;
  if (m_mp4->imp_trak_info[1] == NULL)
    return IT_ERROR;
  if (m_mp4->imp_trak_info[1]->stsz_count == 0)
  {
    return IT_DASH;
  }
  return IT_OK;
}

WORD32 impeghd_mp4_get_datamp4(VOID *mp4_cntxt, WORD32 *offset, pUWORD8 buffer, WORD32 buf_size,
                               UWORD32 *length, WORD32 *size, WORD32 *loc)
{
  mp4_info *m_mp4 = (mp4_info *)mp4_cntxt;
  WORD32 err;
  if (*loc == 0)
  {
    err = impeghd_mp4_fseek(m_mp4->fp, *offset, SEEK_SET);
    if (err)
    {
      return err;
    }
    *size = impeghd_mp4_find_mdat(m_mp4->fp, offset);
    *loc = 1;
  }
  err = impeghd_mp4_read_mdat(m_mp4->fp, buffer, buf_size, size, length, offset);
  if (*size == 0)
  {
    *loc = 0;
  }
  return err;
}

/**
*  impeghd_mp4_get_audio
*
*  \brief Gets MP4 audio
*
*  \param [in] mp4_cntxt      Pointer to mp4 context
*  \param [in,out] frm_cntxt  Pointer to frame context
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_get_audio(pVOID mp4_cntxt, pVOID frm_cntxt)
{
  mp4_info *m_mp4 = (mp4_info *)mp4_cntxt;
  it_mp4_frame_cntxt *frame_cntxt = (it_mp4_frame_cntxt *)frm_cntxt;
  if (m_mp4->imp_trak_info[1] == NULL)
    return IT_ERROR;

  if ((frame_cntxt->frame_length = impeghd_mp4_read_media_sample(
           (UWORD8 **)&frame_cntxt->frame, 0x7FFFFFFF, m_mp4->imp_trak_info[1], m_mp4->fp)) == 0)
  {
    return IT_EXIT;
  }
  if ((WORD32)frame_cntxt->frame_length == IT_ERROR)
  {
    return IT_ERROR;
  }

  frame_cntxt->presentation_time = impeghd_mp4_get_audio_cts(mp4_cntxt);
  return IT_OK;
}

/**
*  impeghd_mp4_parser_close
*
*  \brief Clears the MP4 parser context
*
*  \param [in] mp4_cntxt      Pointer to mp4 context
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_parser_close(pVOID mp4_cntxt)
{
  mp4_info *m_mp4 = (mp4_info *)mp4_cntxt;
  /* Loopback */
  impeghd_mp4_clear_buffer(m_mp4->imp_trak_info[0]);
  impeghd_mp4_clear_buffer(m_mp4->imp_trak_info[1]);
  impeghd_mp4_free_all_nodes(&(m_mp4->ptr_mem));
  impeghd_mp4_free_wrapper((pVOID)mp4_cntxt);
  return IT_OK;
}

/** @} */ /* End of MP4Parser */