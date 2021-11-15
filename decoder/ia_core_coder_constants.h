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

#ifndef IA_CORE_CODER_CONSTANTS_H
#define IA_CORE_CODER_CONSTANTS_H

/*****************************************************************************/
/* constant macros                                                           */
/*****************************************************************************/
#define Q0 1
#define Q1 2
#define Q2 4
#define Q3 8
#define Q4 16
#define Q5 32
#define Q6 64
#define Q7 128
#define Q8 256
#define Q9 512
#define Q10 1024
#define Q11 2048
#define Q12 4096
#define Q13 8192
#define Q14 16384
#define Q15 32768
#define Q16 65536
#define Q17 131072
#define Q18 262144
#define Q19 524288
#define Q20 1048576
#define Q21 2097152
#define Q22 4194304
#define Q23 8388608
#define Q24 16777216
#define Q25 33554432
#define Q26 67108864
#define Q27 134217728
#define Q28 268435456
#define Q29 536870912
#define Q30 1073741824
#define Q31 2147483647
#define Q32 4294967296
#define Q35 34359738368
#define Q38 274877906944
#define Q39 549755813887
#define Q40 Q39

#define MAX_64 (WORD64)0x7fffffffffffffffLL
#define MIN_64 (WORD64)0x8000000000000000LL

#define MAX_32 (WORD32)0x7fffffffL
#define MIN_32 (WORD32)0x80000000L

#define MAX_16 (WORD16)0x7fff
#define MIN_16 (WORD16)0x8000

#define NULLPTR ((VOID *)0)

#define IT_NULL ((VOID *)0)

#define ADJ_SCALE 11

/*****************************************************************************/
/* function macros                                                           */
/*****************************************************************************/

#endif /* IA_CORE_CODER_CONSTANTS_H */
