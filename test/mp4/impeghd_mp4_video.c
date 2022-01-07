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
#include "impeghd_mp4_odf.h"
#include "impeghd_mp4_file.h"
#include "impeghd_mp4_utils.h"
#include "impeghd_mp4_defs.h"
#include "impeghd_mp4_atoms.h"
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
*  impeghd_mp4_clear_buffer
*
*  \brief Clears the contents of trak_info structure, which is used
*         to store the important trak information.
*
*  \param [out] m Pointer to track info structure
*
*
*
*/
VOID impeghd_mp4_clear_buffer(trak_info *m)
{
  if (m)
  {
    m->read_stsc_entries = 0;
    m->stco_cntxt.read = 0;
    m->read_stsz_entries = 0;
    m->samples_read_curr_chunk = 0;
    m->read_incurrent_chunk = 0;
    m->read_incurrent_sample = 0;
    m->stss_count = 0;
    m->read_stts_entries = 0;
    m->sample_read_last_stts_entry = 0;
    m->read_ctts_entries = 0;
    m->sample_read_last_ctts_entry = 0;
    m->ctts_count = 0;
    m->end_of_trak = 0;
  }
}

/**
*  impeghd_mp4_get_needed_trak
*
*  \brief Fills the trak_info structure with the ia_mp4_trak_init info of
*         trak_type = mediatype (gets the trak by mediatype)
*
*  \param [out] m Pointer to track info structure
*  \param [in]  n Pointer to mp4 track init structure
*  \param [in]  media_type Media type
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_get_needed_trak(trak_info *m, ia_mp4_trak_init **n, UWORD32 media_type)
{
  while (*n)
  {
    if ((*n)->trak_type == media_type)
    {
      {
        m->trak_id = (*n)->trak_id;
        m->trak_type = (*n)->trak_type;
        m->read_incurrent_chunk = 0;
        m->read_incurrent_sample = 0;
        m->current_trak = (*n)->trak_id;
        m->stbl_offset = (*n)->stbl_offset;
        m->stts_index = 0;
        m->ctts_index = 0;
        m->stsz_index = 0;
        m->stco_cntxt.index = 0;
        m->stsc_index = 0;
        m->stss_index = 0;
        m->time_scale = (*n)->time_scale;
        m->duration = (*n)->duration;
        m->tref = (*n)->tref;
        m->udta_info = (*n)->udta_info;
        m->handler_type = (*n)->handler_type;
        return IT_OK;
      }
    }
    (*n) = (*n)->next;
  }
  return IT_ERROR;
}

/**
*  impeghd_mp4_get_stco
*
*  \brief Function to fill the 'stco' atom (ChunkOffset) info in trak_info
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*
*  \return WORD32  Error if any
*
*/
static WORD32 impeghd_mp4_get_stco(pVOID fp, trak_info *m)
{
  UWORD32 ret, j, to_read;
  UWORD8 charbuf[4];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  to_read = m->stco_cntxt.count - m->stco_cntxt.index;
  if (to_read == 0)
  {
    return IT_ERROR;
  }
  if (to_read > IT_MP4_BUF_SIZE)
  {
    to_read = IT_MP4_BUF_SIZE;
  }
  impeghd_mp4_fseek(fp, m->stco_cntxt.offset + (4 * (m->stco_cntxt.index)), SEEK_SET);
  for (j = 0; j < to_read; j++)
  {
    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    m->stco_entries[j] = impeghd_mp4_rev32(*data_size);
  }
  m->stco_cntxt.index = m->stco_cntxt.index + to_read;
  m->stco_cntxt.total = (UWORD16)to_read;
  m->stco_cntxt.read = 0;
  return IT_OK;
}

/**
*  impeghd_mp4_get_stss
*
*  \brief Presently it skips the stss atom [Sync(key, I-frame)], needs to be incorporated
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*
*  \return WORD32  Error if any
*
*/
static WORD32 impeghd_mp4_get_stss(pVOID fp, trak_info *m)
{
  UWORD32 ret, to_read, j;
  UWORD8 charbuf[10];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  to_read = m->stss_count - m->stss_index;
  if (to_read == 0)
  {
    m->total_entries_in_stss_buffer = 0;
    m->read_stss_entries = 0;
    return IT_ERROR;
  }
  if (to_read > IT_MP4_BUF_SIZE)
  {
    to_read = IT_MP4_BUF_SIZE;
  }
  impeghd_mp4_fseek(fp, m->stss_offset + (4 * (m->stss_index)), SEEK_SET);
  for (j = 0; j < to_read; j++)
  {
    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
    {
      return IT_ERROR;
    }
    m->stss_entries[j] = impeghd_mp4_rev32(*data_size);
  }
  m->stss_index = m->stss_index + to_read;
  m->total_entries_in_stss_buffer = (UWORD16)to_read;
  m->read_stss_entries = 0;
  return IT_OK;
}

/**
*  impeghd_mp4_get_stsz
*
*  \brief Function to fill the 'stsz' atom (Samplesize) info in trak_info
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*
*  \return WORD32  Error if any
*
*/
static WORD32 impeghd_mp4_get_stsz(pVOID fp, trak_info *m)
{
  UWORD32 ret, toRead, j;
  UWORD8 charbuf[10];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  if (m->sample_size == 0)
  {
    toRead = m->stsz_count - m->stsz_index;
    if (toRead == 0)
    {
      return IT_ERROR;
    }
    if (toRead > IT_MP4_BUF_SIZE)
    {
      toRead = IT_MP4_BUF_SIZE;
    }
    impeghd_mp4_fseek(fp, m->stsz_offset + (4 * (m->stsz_index)), SEEK_SET);
    for (j = 0; j < toRead; j++)
    {
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
      {
        return IT_ERROR;
      }
      m->stsz_entries[j] = impeghd_mp4_rev32(*data_size);
    }
    impeghd_mp4_fseek(fp, m->stsz_offset + (4 * toRead), SEEK_SET);
    m->stsz_index += toRead;
    m->total_entries_in_stsz_buffer = (UWORD16)toRead;
    m->read_stsz_entries = 0;
  }
  return IT_OK;
}

/**
*  impeghd_mp4_fill_imp_trak_info
*
*  \brief Fills the important trak info structure
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*  \param [in]  req_child required child
*  \param [in]  n  Pointer to mp4 mem node structure pointer
*  \param [in]  ptr_mae_info  Pointer to mae info structure
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_fill_imp_trak_info(pVOID fp, trak_info *m, UWORD32 req_child,
                                      ia_mp4_mem_node **n, maei_info *ptr_mae_info)
{
  UWORD32 ret, size, type, child_size, child_type;
  WORD32 large_size;
  UWORD8 charbuf[10];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];

  if (m == NULL)
  {
    return IT_ERROR;
  }
  impeghd_mp4_fseek(fp, m->stbl_offset, SEEK_SET);
  ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)
    return IT_ERROR;
  size = impeghd_mp4_rev32(*data_size);
  m->stbl_size = (WORD32)size;
  ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)
    return IT_ERROR;
  type = impeghd_mp4_rev32(*data_size);
  m->stbl_size = m->stbl_size - 8;
  if (size == 1)
  {
    ret = impeghd_mp4_fread(charbuf, 1, 8, fp);
    if (ret < 8)
      return IT_ERROR;
    large_size = impeghd_mp4_rev64(*data_size);
    m->stbl_size = large_size;
    m->stbl_size = m->stbl_size - 8;
  }
  if (type != IT_STBL)
    return IT_ERROR;

  do
  {
    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    child_size = impeghd_mp4_rev32(*data_size);

    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    child_type = impeghd_mp4_rev32(*data_size);

    if (child_type == IT_STTS)
    {
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      m->stts_count = impeghd_mp4_rev32(*data_size);
      m->stts_offset = (UWORD32)impeghd_mp4_ftell(fp);
      if ((req_child == IT_ALL) || (req_child == IT_STTS))
      {

        if (m->stts_count)
        {
          if ((impeghd_mp4_get_stts(fp, m)) != IT_OK)
            return IT_ERROR;
        }
        impeghd_mp4_fseek(fp, 8 * (m->stts_count - m->stts_index), SEEK_CUR);
      }
      else
        impeghd_mp4_fseek(fp, child_size - 16, SEEK_CUR);
    }
    else if (child_type == IT_CTTS)
    {
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      m->ctts_count = impeghd_mp4_rev32(*data_size);
      m->ctts_offset = (UWORD32)impeghd_mp4_ftell(fp);
      if ((req_child == IT_ALL) || (req_child == IT_CTTS))
      {
        if ((impeghd_mp4_get_ctts(fp, m)) != IT_OK)
        {
          return IT_ERROR;
        }
        impeghd_mp4_fseek(fp, 8 * (m->ctts_count - m->ctts_index), SEEK_CUR);
      }
      else
        impeghd_mp4_fseek(fp, child_size - 16, SEEK_CUR);
    }
    else if (child_type == IT_STSD)
    {
      if (((req_child == IT_ALL) || (req_child == IT_STSD)) && (m->stsd_atom == NULL))
      {
        if ((impeghd_mp4_get_stsd(fp, m, n, ptr_mae_info)) != IT_OK)         
			return IT_ERROR;
        if (m->stsd_atom)
        {
          m->stsd_atom->type = child_type;
          m->stsd_atom->size = child_size;
        }
      }

      else
        impeghd_mp4_fseek(fp, child_size - 8, SEEK_CUR);
    }
    else if (child_type == IT_STSZ)
    {
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;

      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      m->sample_size = impeghd_mp4_rev32(*data_size);

      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      m->stsz_count = impeghd_mp4_rev32(*data_size);
      m->stsz_offset = impeghd_mp4_ftell(fp);

      if ((req_child == IT_ALL) || (req_child == IT_STSZ))
      {
        if (m->sample_size == 0)
        {
          if ((impeghd_mp4_get_stsz(fp, m)) != IT_OK)
            return IT_ERROR;
          impeghd_mp4_fseek(fp, 4 * (m->stsz_count - m->stsz_index), SEEK_CUR);
        }
        else
        {
          impeghd_mp4_fseek(fp, child_size - 20, SEEK_CUR);
        }
      }
      else
        impeghd_mp4_fseek(fp, child_size - 20, SEEK_CUR);
    }
    else if (child_type == IT_STSC)
    {
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;

      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      m->stsc_count = impeghd_mp4_rev32(*data_size);
      m->stsc_offset = impeghd_mp4_ftell(fp);
      if ((req_child == IT_ALL) || (req_child == IT_STSC))
      {
        if (m->stsc_count)
        {
          if ((impeghd_mp4_get_stsc(fp, m)) != IT_OK)
            return IT_ERROR;
        }
        impeghd_mp4_fseek(fp, 12 * (m->stsc_count - m->stsc_index), SEEK_CUR);
      }
      else
        impeghd_mp4_fseek(fp, child_size - 16, SEEK_CUR);
    }
    else if (child_type == IT_STCO)
    {
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;

      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      m->stco_cntxt.count = impeghd_mp4_rev32(*data_size);
      m->stco_cntxt.offset = impeghd_mp4_ftell(fp);
      if ((req_child == IT_ALL) || (req_child == IT_STCO))
      {
        if (m->stco_cntxt.count)
        {
          if ((impeghd_mp4_get_stco(fp, m)) != IT_OK)
            return IT_ERROR;
        }
        impeghd_mp4_fseek(fp, 4 * (m->stco_cntxt.count - m->stco_cntxt.index), SEEK_CUR);
      }
      else
        impeghd_mp4_fseek(fp, child_size - 16, SEEK_CUR);
    }
    else if (child_type == IT_STSS)
    {
      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;

      ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
      if (ret < 4)
        return IT_ERROR;
      m->stss_count = impeghd_mp4_rev32(*data_size);
      m->stss_offset = impeghd_mp4_ftell(fp);
      if (((req_child == IT_ALL) || (req_child == IT_STSS)) && (m->stss_count > 0))
      {
        if ((impeghd_mp4_get_stss(fp, m)) != IT_OK)
        {
          return IT_ERROR;
        }
        impeghd_mp4_fseek(fp, 4 * (m->stss_count - m->stss_index), SEEK_CUR);
      }
      else
        impeghd_mp4_fseek(fp, child_size - 8, SEEK_CUR);
    }
    else if (child_type == IT_STSH)
    {
      if ((req_child == IT_ALL) || (req_child == IT_STSH))
        impeghd_mp4_fseek(fp, child_size - 8, SEEK_CUR);
      else
        impeghd_mp4_fseek(fp, child_size - 8, SEEK_CUR);
    }
    else if (child_type == IT_STDP)
    {
      if ((req_child == IT_ALL) || (req_child == IT_STSH))
        impeghd_mp4_fseek(fp, child_size - 8, SEEK_CUR);
      else
        impeghd_mp4_fseek(fp, child_size - 8, SEEK_CUR);
    }
    else
    {
      impeghd_mp4_fseek(fp, child_size - 8, SEEK_CUR);
    }
    m->stbl_size = m->stbl_size - (WORD32)child_size;
  } while ((m->stbl_size) > 0 && (child_size > 0));

  return IT_OK;
}

/**
*  impeghd_mp4_get_stts
*
*  \brief Fills the 'stts' atom (DecodingTimeStamp) info in
*         trak_info.
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_get_stts(pVOID fp, trak_info *m)
{
  UWORD32 ret, to_read, j;
  UWORD8 charbuf[10];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  to_read = m->stts_count - m->stts_index;
  if (to_read == 0)
  {
    return IT_ERROR;
  }
  if (to_read > IT_MP4_BUF_SIZE)
  {
    to_read = IT_MP4_BUF_SIZE;
  }
  impeghd_mp4_fseek(fp, m->stts_offset + 8 * (m->stts_index), SEEK_SET);
  for (j = 0; j < to_read; j++)
  {
    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    m->stts_entries[j].sample_count = impeghd_mp4_rev32(*data_size);

    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    m->stts_entries[j].sample_delta = impeghd_mp4_rev32(*data_size);
  }
  m->stts_index += to_read;
  m->total_entries_in_stts_buffer = (UWORD16)to_read;
  m->read_stts_entries = 0;
  m->sample_read_last_stts_entry = 0;
  return IT_OK;
}

/**
*  impeghd_mp4_get_ctts
*
*  \brief fills the 'ctts' atom (CompositionTimeStamp) info in
*         trak_info.
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_get_ctts(pVOID fp, trak_info *m)
{
  WORD32 ret, toRead, j;
  UWORD8 charbuf[10];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  toRead = m->ctts_count - m->ctts_index;
  if (toRead <= 0)
  {
    return IT_ERROR;
  }
  if (toRead > IT_MP4_BUF_SIZE)
  {
    toRead = IT_MP4_BUF_SIZE;
  }
  impeghd_mp4_fseek(fp, m->ctts_offset + (8 * (m->ctts_index)), SEEK_SET);
  for (j = 0; j < toRead; j++)
  {
    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    m->ctts_entries[j].sample_count = impeghd_mp4_rev32(*data_size);

    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    m->ctts_entries[j].sample_offset = impeghd_mp4_rev32(*data_size);
  }
  m->ctts_index += toRead;
  m->total_entries_in_ctts_buffer = (UWORD16)toRead;
  m->read_ctts_entries = 0;
  m->sample_read_last_ctts_entry = 0;
  return IT_OK;
}

/**
*  impeghd_mp4_get_stsc
*
*  \brief Fills the 'stsc' atom info in trak_info
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_get_stsc(pVOID fp, trak_info *m)
{
  UWORD32 ret, j;
  UWORD8 charbuf[4];
  UWORD32 to_read = 0;
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  to_read = m->stsc_count - m->stsc_index;
  if (to_read == 0)
  {
    return IT_ERROR;
  }
  if (to_read > IT_MP4_BUF_SIZE)
  {
    to_read = IT_MP4_BUF_SIZE;
  }
  impeghd_mp4_fseek(fp, m->stsc_offset + (12 * (m->stsc_index)), SEEK_SET);
  for (j = 0; j < to_read; j++)
  {
    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    m->stsc_entries[j].first_chunk = impeghd_mp4_rev32(*data_size);

    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    m->stsc_entries[j].samples_per_chunk = impeghd_mp4_rev32(*data_size);

    ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
    if (ret < 4)
      return IT_ERROR;
    m->stsc_entries[j].sample_descr_index = impeghd_mp4_rev32(*data_size);
  }
  m->stsc_index = m->stsc_index + to_read;
  m->total_entries_in_stsc_buffer = (UWORD16)to_read;
  m->read_stsc_entries = 0;

  return IT_OK;
}

/**
*  impeghd_mp4_get_stsd
*
*  \brief Reads the 'stsd' atom
*
*  \param [in]  fp Pointer to file context
*  \param [out] m  Pointer to trak info
*  \param [in]  n  Pointer to mp4 mem node structure pointer
*  \param [in]  ptr_mae_info  Pointer to mae info structure
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_get_stsd(pVOID fp, trak_info *m, ia_mp4_mem_node **n, maei_info *ptr_mae_info)
{

  UWORD32 ret, i;
  UWORD8 charbuf[10];
  UWORD32 *data_size = (UWORD32 *)&charbuf[0];
  ia_mp4_sample_entry *ptr1;

  ia_mp4_sample_desc_atom *fa =
      (ia_mp4_sample_desc_atom *)impeghd_mp4_mem_node_malloc(n, sizeof(ia_mp4_sample_desc_atom));

  m->stsd_atom = fa;
  ret = impeghd_mp4_fread(charbuf, 1, 1, fp);
  if (ret < 1)
  {
    return IT_ERROR;
  }
  fa->version = charbuf[0];
  ret = impeghd_mp4_fread(&(charbuf[1]), 1, 3, fp);
  if (ret < 3)
  {
    return IT_ERROR;
  }
  charbuf[0] = 0;
  fa->flags = impeghd_mp4_rev32(*data_size);

  ret = impeghd_mp4_fread(charbuf, 1, 4, fp);
  if (ret < 4)
  {
    return IT_ERROR;
  }
  fa->entry_count = impeghd_mp4_rev32(*data_size);

  ptr1 = (ia_mp4_sample_entry *)impeghd_mp4_mem_node_malloc(n, sizeof(ia_mp4_sample_entry));
  if (ptr1 == NULL)
    return IT_ERROR;
  fa->ptr_sample_entry = ptr1;
  ptr1->ptr_next_sample_entry = NULL;

  for (i = 0; i < fa->entry_count; i++)
  {
    if (impeghd_mp4_read_samples(fp, &ptr1, n, m, ptr_mae_info) != IT_OK)      
		return IT_ERROR;
    if (i != (fa->entry_count - 1))
    {
      ptr1->ptr_next_sample_entry =
          (ia_mp4_sample_entry *)impeghd_mp4_mem_node_malloc(n, sizeof(ia_mp4_sample_entry));
      if (ptr1->ptr_next_sample_entry == NULL)
        return IT_ERROR;
      ptr1 = ptr1->ptr_next_sample_entry;
      ptr1->ptr_next_sample_entry = NULL;
    }
  }
  return IT_OK;
}

/**
*  impeghd_mp4_get_dts
*
*  \brief Function to get the decoding time stamp
*
*  \param [in] m_mp4 Pointer to MP4 info
*  \param [in] media Media type
*
*  \return WORD32    Decoding time stamp or error if any
*
*/
static WORD32 impeghd_mp4_get_dts(mp4_info *m_mp4, UWORD32 media)
{
  WORD32 dts = -1;
  trak_info *trak;

  if (CHECK_VIDEO(media))
  {
    trak = m_mp4->imp_trak_info[0];
  }
  else if (CHECK_AUDIO(media))
  {
    trak = m_mp4->imp_trak_info[1];
  }
  else if (media == MPEG4_HINT_VIDEO)
  {
    trak = m_mp4->imp_trak_info[4];
  }
  else if (media == MPEG4_HINT_AUDIO)
  {
    trak = m_mp4->imp_trak_info[5];
  }
  else
  {
    return IT_ERROR;
  }

  while (1)
  {
    if (trak->sample_read_last_stts_entry < trak->stts_entries[trak->read_stts_entries].sample_count)
    {
      dts = trak->stts_entries[trak->read_stts_entries].sample_delta;
      trak->sample_read_last_stts_entry++;
      return dts;
    }
    else
    {
      trak->read_stts_entries++;
      if (trak->read_stts_entries >= trak->total_entries_in_stts_buffer)
      {
        if ((impeghd_mp4_get_stts(m_mp4->fp, trak)) != IT_OK)
        {
          return IT_ERROR;
        }
      }
      else
      {
        trak->sample_read_last_stts_entry = 0;
      }
    }
  }
}

/**
*  impeghd_mp4_get_cts
*
*  \brief Function to get the composition time stamp
*
*  \param [in] m_mp4 Pointer to MP4 info
*  \param [in] media Media type
*
*  \return WORD32    Composition time stamp or error if any
*
*/
static WORD32 impeghd_mp4_get_cts(mp4_info *m_mp4, UWORD32 media)
{
  WORD32 cts = 0;
  if ((cts = impeghd_mp4_get_dts(m_mp4, media)) == IT_ERROR)
  {
    return IT_ERROR;
  }
  return cts;
}

/**
*  impeghd_mp4_update_stss
*
*  \brief Function to update 'stss' atom
*
*  \param [in] m  Pointer to trak info
*  \param [in] fp Pointer to file context
*
*
*
*/
static VOID impeghd_mp4_update_stss(trak_info *m, pVOID fp)
{
  if (m->stss_count)
  {
    if (m->frame_num >= m->stss_entries[m->read_stss_entries])
    {
      m->read_stss_entries++;
      if (m->read_stss_entries >= m->total_entries_in_stss_buffer)
      {
        if ((impeghd_mp4_get_stss(fp, m)) != IT_OK)
        {
          return;
        }
      }
    }
  }

  return;
}

/**
*  impeghd_mp4_read_media_sample
*
*  \brief Fills the 'buf' with Data frame by frame
* 
*  \param [out] buf Pointer to buffer to be filled
*  \param [in]  to_write Number of bytes to write to buf
*  \param [out] m  Pointer to trak info
*  \param [in]  fp Pointer to file context
*
*  \return WORD32
*
*/
WORD32 impeghd_mp4_read_media_sample(UWORD8 **buf, WORD32 to_write, trak_info *m, pVOID fp)
{
  WORD32 read_bytes = 0;
  UWORD32 ret, offset, last_chunk_number_of_curr_stsc_entry = 0;
  UWORD32 current_stco_index = 0;

  while (1)
  {
    if (m->read_stsc_entries >= m->total_entries_in_stsc_buffer)
    {
      m->samples_last_stsc_entry = m->stsc_entries[m->read_stsc_entries - 1].samples_per_chunk;
      if ((impeghd_mp4_get_stsc(fp, m)) != IT_OK)
      {
        last_chunk_number_of_curr_stsc_entry = m->stco_cntxt.count;
      }
      else
      {
        last_chunk_number_of_curr_stsc_entry =
            m->stsc_entries[m->read_stsc_entries].first_chunk - 1;
      }
    }
    else
    {
      last_chunk_number_of_curr_stsc_entry =
          m->stsc_entries[m->read_stsc_entries].first_chunk - 1;
    }
    current_stco_index = m->stco_cntxt.index - (m->stco_cntxt.total - m->stco_cntxt.read);
    if (current_stco_index == m->stco_cntxt.count)
    {
      break;
    }
    while (current_stco_index < last_chunk_number_of_curr_stsc_entry)
    {
      if (m->stco_cntxt.read >= m->stco_cntxt.total)
      {
        if ((impeghd_mp4_get_stco(fp, m)) != IT_OK)
        {
          return 0;
        }
      }
      offset = (UWORD32)((m->stco_entries[m->stco_cntxt.read]) + (m->read_incurrent_chunk));

      while (m->samples_read_curr_chunk < m->samples_last_stsc_entry)
      {
        if (m->read_stsz_entries >= m->total_entries_in_stsz_buffer)
        {
          if ((impeghd_mp4_get_stsz(fp, m)) != IT_OK)
          {
            return 0;
          }
        }
        impeghd_mp4_fseek(fp, offset, SEEK_SET);

        if (m->sample_size != 0)
        {
          if (((UWORD32)to_write) < (m->sample_size - m->read_incurrent_sample))
          {
            if (m->sample_type == IT_MP4V)
            {
              ret = impeghd_mp4_fread_buf((WORD32 **)buf, 1, to_write, fp);
            }
            else
            {
              ret = impeghd_mp4_fread(*buf, 1, to_write, fp);
            }
            if (ret < (UWORD32)to_write)
              return IT_ERROR;

            m->read_incurrent_chunk += ret;
            m->read_incurrent_sample += ret;
            read_bytes += ret;
          }
          else
          {
            if (m->sample_type != IT_MP4V)
            {
              ret = impeghd_mp4_fread_buf((WORD32 **)buf, 1,
                                          m->sample_size - m->read_incurrent_sample, fp);
            }
            else
            {
              ret = impeghd_mp4_fread(*buf, 1, m->sample_size - m->read_incurrent_sample, fp);
            }
            if (ret < m->sample_size - m->read_incurrent_sample)
              return IT_ERROR;

            m->read_incurrent_chunk += ret;
            m->read_incurrent_sample = 0;
            m->samples_read_curr_chunk++;
            read_bytes += ret;
          }
        }
        else
        {
          if (((UWORD32)to_write) <
              (m->stsz_entries[m->read_stsz_entries] - m->read_incurrent_sample))
          {
            if (m->sample_type == IT_MP4V)
            {
              ret = impeghd_mp4_fread_buf((WORD32 **)buf, 1, to_write, fp);
            }
            else
            {
              ret = impeghd_mp4_fread(*buf, 1, to_write, fp);
            }
            if (ret < (UWORD32)to_write)
              return IT_ERROR;
            m->read_incurrent_chunk += ret;
            m->read_incurrent_sample += ret;
            read_bytes += ret;
          }
          else
          {
            if (m->sample_type == IT_MP4V)
            {
              ret = impeghd_mp4_fread_buf(
                  (WORD32 **)buf, 1,
                  ((m->stsz_entries[m->read_stsz_entries]) - m->read_incurrent_sample), fp);
            }
            else
            {
              ret = impeghd_mp4_fread(
                  *buf, 1, ((m->stsz_entries[m->read_stsz_entries]) - m->read_incurrent_sample),
                  fp);
            }
            if (ret < ((m->stsz_entries[m->read_stsz_entries]) - m->read_incurrent_sample))
              return IT_ERROR;

            m->read_incurrent_chunk += ret;
            m->read_incurrent_sample = 0;
            m->read_stsz_entries++;
            m->samples_read_curr_chunk++;
            read_bytes += ret;
          }
        }
        m->frame_num++;
        impeghd_mp4_update_stss(m, fp);
        return (read_bytes);
      }
      m->samples_read_curr_chunk = 0;
      m->read_incurrent_chunk = 0;
      m->stco_cntxt.read++;
      current_stco_index = m->stco_cntxt.index - (m->stco_cntxt.total - m->stco_cntxt.read);
    }
    m->read_stsc_entries++;
  }
  return read_bytes;
}

/**
*  impeghd_mp4_get_audio_cts
*
*  \brief Gets composition time stamp
*
*  \param [in,out] mp4_cntxt Pointer to mp4 context
*
*  \return UWORD32
*
*/
UWORD32 impeghd_mp4_get_audio_cts(pVOID mp4_cntxt)
{
  mp4_info *m_mp4 = (mp4_info *)mp4_cntxt;
  it_uint64 temp;
  WORD32 cts = impeghd_mp4_get_cts(m_mp4, MPEG4_AUDIO);
  if (cts != IT_ERROR)
  {
    impeghd_mp4_add64(&m_mp4->imp_trak_info[1]->elap_time, m_mp4->imp_trak_info[1]->elap_time,
                      cts);

    impeghd_mp4_multi_1k(&temp, m_mp4->imp_trak_info[1]->elap_time);
    impeghd_mp4_div64((pUWORD32)&cts, temp, m_mp4->imp_trak_info[1]->time_scale);
  }
  return (cts);
}

/** @} */ /* End of MP4Parser */