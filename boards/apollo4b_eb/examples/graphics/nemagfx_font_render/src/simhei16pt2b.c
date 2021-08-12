//*****************************************************************************
//
//! @file simhei16pt2b.c
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2021, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision b0-release-20210111-1514-g6a1d4008b7 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#ifndef SIMHEI16PT2B_C
#define SIMHEI16PT2B_C

#include "simhei16pt2b.h"

#ifndef NEMA_GPU_MEM
#define NEMA_GPU_MEM
#endif // NEMA_GPU_MEM

// This will be read by the GPU only
const uint8_t NEMA_GPU_MEM g_sSimhei16pt2bBitmaps[] = {
  // 0x4f8b - 0x4f8b
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00,
  0x00, 0x00, 0x06, 0x40, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x0B, 0x80,
  0x00, 0x0B, 0xCB, 0xFF, 0xFF, 0x00, 0x0B, 0x80, 0x00, 0x0F, 0x8B, 0xFF,
  0xFF, 0x00, 0x0B, 0x80, 0x00, 0x1F, 0x00, 0x3D, 0x00, 0x15, 0x0B, 0x80,
  0x00, 0x3F, 0x00, 0x3C, 0x00, 0x3F, 0x0B, 0x80, 0x00, 0x7E, 0x00, 0x7C,
  0x00, 0x2E, 0x0B, 0x80, 0x00, 0xFE, 0x00, 0x7C, 0x00, 0x2E, 0x0B, 0x80,
  0x01, 0xFE, 0x00, 0xBC, 0x00, 0x2E, 0x0B, 0x80, 0x03, 0xFE, 0x00, 0xFF,
  0xFE, 0x2E, 0x0B, 0x82, 0x0B, 0xFE, 0x00, 0xFA, 0xBD, 0x2E, 0x0B, 0x87,
  0x1F, 0xEE, 0x02, 0xF0, 0x3D, 0x2E, 0x0B, 0x8B, 0x2F, 0x2E, 0x03, 0xE0,
  0x3D, 0x2E, 0x0B, 0x81, 0x06, 0x2E, 0x0B, 0xC0, 0x3C, 0x2E, 0x0B, 0x80,
  0x00, 0x2E, 0x1F, 0x80, 0x7C, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x3F, 0x3C,
  0xBC, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x1D, 0x7F, 0xF8, 0x2E, 0x0B, 0x80,
  0x00, 0x2E, 0x00, 0x0F, 0xF4, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x02,
  0xF0, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x03, 0xE0, 0x2E, 0x0B, 0x80,
  0x00, 0x2E, 0x00, 0x07, 0xC0, 0x19, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x0F,
  0x80, 0x00, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x3F, 0x00, 0x00, 0x0B, 0x80,
  0x00, 0x2E, 0x00, 0xBE, 0x00, 0x00, 0x0B, 0x80, 0x00, 0x2E, 0x03, 0xFC,
  0x00, 0x00, 0x0F, 0x80, 0x00, 0x2E, 0x1F, 0xF0, 0x00, 0x03, 0xFF, 0x40,
  0x00, 0x3E, 0x0F, 0xC0, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x2E, 0x02, 0x00,
  0x00, 0x01, 0xA4, 0x00,
  // 0x662f - 0x662f
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF,
  0xFF, 0xFE, 0x00, 0x00, 0x00, 0x1F, 0xAA, 0xAA, 0xAA, 0xBE, 0x00, 0x00,
  0x00, 0x1F, 0x00, 0x00, 0x00, 0x3D, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00,
  0x00, 0x3D, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFD, 0x00, 0x00,
  0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFD, 0x00, 0x00, 0x00, 0x1F, 0x40, 0x00,
  0x00, 0x3D, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x3D, 0x00, 0x00,
  0x00, 0x1F, 0x55, 0x55, 0x55, 0x7E, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF,
  0xFF, 0xFE, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFD, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03,
  0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x1F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x40, 0x1F, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0B, 0xC0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x1F,
  0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x1F, 0x40, 0x1F, 0xFF, 0xFF, 0xC0, 0x00,
  0x00, 0x3F, 0xD0, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xF8, 0x1F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xBF, 0x6F, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xF4, 0x1F, 0xFF, 0x50, 0x00, 0x15, 0x40, 0x0F, 0xE0, 0x02, 0xFF,
  0xFF, 0xFF, 0xFF, 0x43, 0x3F, 0xC0, 0x00, 0x1B, 0xFF, 0xFF, 0xFF, 0x00,
  0x0B, 0x00, 0x00, 0x00, 0x00, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  // 0x6c49 - 0x6c49
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0B, 0xD0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x07, 0xF8, 0x1F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00,
  0x00, 0xBF, 0x1F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x2F, 0x00, 0xF4,
  0x00, 0x03, 0xE0, 0x00, 0x00, 0x05, 0x00, 0xF4, 0x00, 0x03, 0xD0, 0x00,
  0x00, 0x00, 0x00, 0xB8, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x7C,
  0x00, 0x07, 0xC0, 0x0B, 0x2E, 0x00, 0x00, 0x3C, 0x00, 0x0F, 0x80, 0x0F,
  0x3F, 0xD0, 0x00, 0x3D, 0x00, 0x0F, 0x00, 0x02, 0x0B, 0xF8, 0x00, 0x2E,
  0x00, 0x1F, 0x00, 0x00, 0x01, 0xFE, 0x00, 0x1F, 0x00, 0x3E, 0x00, 0x00,
  0x00, 0x3C, 0x00, 0x0F, 0x40, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B,
  0xC0, 0xBC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0xF4, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0xE2, 0xF0, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00,
  0xF7, 0xD0, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0xBF, 0xC0, 0x00, 0x00,
  0x00, 0x3E, 0x00, 0x00, 0x3F, 0x40, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00,
  0x7F, 0x40, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x01, 0xFF, 0xD0, 0x00, 0x00,
  0x02, 0xF0, 0x00, 0x07, 0xE2, 0xF4, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x2F,
  0x80, 0xFE, 0x00, 0x02, 0x0B, 0xC0, 0x00, 0xFD, 0x00, 0x2F, 0xC0, 0x07,
  0x1F, 0x80, 0x0B, 0xF4, 0x00, 0x0B, 0xF8, 0x0F, 0x3F, 0x00, 0xBF, 0x80,
  0x00, 0x01, 0xFF, 0x41, 0x06, 0x02, 0xFD, 0x00, 0x00, 0x00, 0x7F, 0x00,
  0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  // 0x5b57 - 0x5b57
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xC0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
  0xF8, 0x00, 0x00, 0x00, 0x1A, 0xAA, 0xAA, 0xFE, 0xAA, 0xAA, 0xA0, 0x3F,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x3E, 0x55, 0x55, 0x55, 0x55, 0x56,
  0xF0, 0x2E, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3E, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xF0, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x00, 0x3F,
  0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xF0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x2F, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x55,
  0x55, 0x55, 0xBD, 0x55, 0x55, 0x54, 0xBF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFD, 0x7E, 0xAA, 0xAA, 0xFF, 0xAA, 0xAA, 0xBD, 0x00, 0x00, 0x00, 0x7C,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x2A, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F,
  0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x5e93 - 0x5e93
  0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xC0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0B, 0xD0, 0x00, 0x00, 0x00, 0x02, 0xAA, 0xAA, 0xAF,
  0xFA, 0xAA, 0xAB, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
  0x03, 0xE0, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x0F,
  0x40, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xE0, 0x00, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE7, 0xFF, 0xFF,
  0xFF, 0xFF, 0xF8, 0x00, 0x03, 0xD7, 0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0x00,
  0x03, 0xD0, 0x02, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xD0, 0x03, 0xC0,
  0x64, 0x00, 0x00, 0x00, 0x03, 0xD0, 0x0F, 0x80, 0xB8, 0x00, 0x00, 0x00,
  0x03, 0xD0, 0x2F, 0x00, 0xB8, 0x00, 0x00, 0x00, 0x03, 0xD0, 0xBE, 0xAA,
  0xFE, 0xAA, 0xA0, 0x00, 0x03, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00,
  0x03, 0xC0, 0x69, 0x55, 0xFD, 0x55, 0x50, 0x00, 0x03, 0xC0, 0x00, 0x00,
  0xB8, 0x00, 0x00, 0x01, 0x07, 0xC0, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x02,
  0x0B, 0xC0, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x02, 0x0B, 0x9F, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0x03, 0x0F, 0x5F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x07,
  0x1F, 0x00, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x0B, 0x2F, 0x00, 0x00, 0x00,
  0xB8, 0x00, 0x00, 0x0F, 0x3E, 0x00, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x2F,
  0xBD, 0x00, 0x00, 0x00, 0xB8, 0x00, 0x00, 0x06, 0x18, 0x00, 0x00, 0x00,
  0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
  // 0x56fe - 0x56fe
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0xF4, 0x00, 0x04,
  0x00, 0x00, 0x07, 0xCF, 0xF4, 0x00, 0x2F, 0x40, 0x00, 0x07, 0xCF, 0xF4,
  0x00, 0x3E, 0x00, 0x00, 0x07, 0xCF, 0xF4, 0x00, 0xFF, 0xFF, 0xFC, 0x07,
  0xCF, 0xF4, 0x02, 0xFF, 0xFF, 0xF8, 0x07, 0xCF, 0xF4, 0x07, 0xF0, 0x02,
  0xF0, 0x07, 0xCF, 0xF4, 0x1F, 0xFC, 0x07, 0xC0, 0x07, 0xCF, 0xF4, 0x7E,
  0x3F, 0x1F, 0x40, 0x07, 0xCF, 0xF4, 0x1C, 0x0F, 0xFD, 0x00, 0x07, 0xCF,
  0xF4, 0x00, 0x07, 0xFC, 0x00, 0x07, 0xCF, 0xF4, 0x00, 0x7F, 0xFF, 0x90,
  0x07, 0xCF, 0xF4, 0x1B, 0xFD, 0x0B, 0xFF, 0x87, 0xCF, 0xF4, 0xFF, 0xD3,
  0x40, 0xBF, 0xC7, 0xCF, 0xF4, 0x38, 0x0B, 0xF8, 0x06, 0x47, 0xCF, 0xF4,
  0x00, 0x02, 0xFF, 0x80, 0x07, 0xCF, 0xF4, 0x00, 0x00, 0x1F, 0x00, 0x07,
  0xCF, 0xF4, 0x00, 0x79, 0x00, 0x00, 0x07, 0xCF, 0xF4, 0x00, 0xFF, 0xE4,
  0x00, 0x07, 0xCF, 0xF4, 0x00, 0x1B, 0xFF, 0x80, 0x07, 0xCF, 0xF4, 0x00,
  0x00, 0x6F, 0x40, 0x07, 0xCF, 0xF4, 0x00, 0x00, 0x01, 0x00, 0x07, 0xCF,
  0xFE, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xCF, 0xF9, 0x55, 0x55, 0x55, 0x55, 0x5B, 0xCF, 0xF4, 0x00, 0x00,
  0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x5f62 - 0x5f62
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x40, 0x02, 0x0A, 0xAA, 0xAA, 0xAA,
  0x40, 0x03, 0xF0, 0x03, 0x0F, 0xFF, 0xFF, 0xFF, 0xC0, 0x0B, 0xE0, 0x02,
  0x09, 0x7D, 0x57, 0xE5, 0x40, 0x0F, 0xC0, 0x00, 0x00, 0x3C, 0x03, 0xD0,
  0x00, 0x3F, 0x00, 0x00, 0x00, 0x3C, 0x03, 0xD0, 0x00, 0xBD, 0x00, 0x00,
  0x00, 0x3C, 0x03, 0xD0, 0x02, 0xF4, 0x00, 0x00, 0x00, 0x3C, 0x03, 0xD0,
  0x0B, 0xE0, 0x00, 0x00, 0x00, 0x3C, 0x03, 0xD0, 0x2F, 0x80, 0x00, 0x00,
  0x00, 0x3C, 0x03, 0xD0, 0x2F, 0x00, 0x60, 0x00, 0x00, 0x3C, 0x03, 0xD0,
  0x04, 0x00, 0xFC, 0x00, 0x00, 0x3C, 0x03, 0xD0, 0x00, 0x03, 0xF4, 0x0F,
  0x3F, 0xFF, 0xFF, 0xFF, 0xC0, 0x0F, 0xD0, 0x0F, 0x3F, 0xFF, 0xFF, 0xFF,
  0xC0, 0x3F, 0x40, 0x00, 0x00, 0x3C, 0x03, 0xD0, 0x00, 0xFE, 0x00, 0x00,
  0x00, 0x7C, 0x03, 0xD0, 0x03, 0xF8, 0x00, 0x00, 0x00, 0x7C, 0x03, 0xD0,
  0x2F, 0xD0, 0x00, 0x00, 0x00, 0xB8, 0x03, 0xD0, 0xFF, 0x40, 0x08, 0x00,
  0x00, 0xB8, 0x03, 0xD0, 0x3C, 0x00, 0x3F, 0x00, 0x00, 0xF4, 0x03, 0xD0,
  0x00, 0x00, 0xFE, 0x00, 0x00, 0xF0, 0x03, 0xD0, 0x00, 0x03, 0xF8, 0x00,
  0x02, 0xF0, 0x03, 0xD0, 0x00, 0x1F, 0xD0, 0x00, 0x03, 0xE0, 0x03, 0xD0,
  0x00, 0xBF, 0x40, 0x02, 0x0B, 0xC0, 0x03, 0xD0, 0x03, 0xFC, 0x00, 0x03,
  0x0F, 0x80, 0x03, 0xD0, 0x2F, 0xE0, 0x00, 0x0F, 0x3F, 0x00, 0x03, 0xD0,
  0xFF, 0x40, 0x00, 0x0B, 0x2E, 0x00, 0x02, 0x90, 0x7C, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  // 0x7684 - 0x7684
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x74,
  0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x3E, 0x00,
  0x00, 0xBC, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x00, 0xF4, 0x00, 0x00, 0x00,
  0x7C, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x00, 0xB8, 0x00, 0x02, 0xFA, 0xAA,
  0xA0, 0x3F, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0xF4, 0x3F, 0xFF, 0xFF, 0x07,
  0xEA, 0xAA, 0xF4, 0x3D, 0x00, 0x2F, 0x0F, 0x80, 0x00, 0xF0, 0x3D, 0x00,
  0x2F, 0x1F, 0x00, 0x00, 0xF0, 0x3D, 0x00, 0x2F, 0x3E, 0x00, 0x00, 0xF0,
  0x3D, 0x00, 0x2F, 0xBC, 0x00, 0x00, 0xF0, 0x3D, 0x00, 0x2F, 0x14, 0x20,
  0x00, 0xF0, 0x3D, 0x00, 0x2F, 0x01, 0xF4, 0x00, 0xF0, 0x3F, 0xFF, 0xFF,
  0x00, 0xFC, 0x00, 0xF0, 0x3F, 0xFF, 0xFF, 0x00, 0x7D, 0x00, 0xF0, 0x3D,
  0x00, 0x2F, 0x00, 0x3E, 0x00, 0xF0, 0x3D, 0x00, 0x2F, 0x00, 0x2F, 0x00,
  0xF0, 0x3D, 0x00, 0x2F, 0x00, 0x0F, 0x80, 0xF0, 0x3D, 0x00, 0x2F, 0x00,
  0x09, 0x01, 0xF0, 0x3D, 0x00, 0x2F, 0x00, 0x00, 0x01, 0xF0, 0x3D, 0x00,
  0x2F, 0x00, 0x00, 0x01, 0xF0, 0x3E, 0xAA, 0xBF, 0x00, 0x00, 0x02, 0xE0,
  0x3F, 0xFF, 0xFF, 0x00, 0x00, 0x02, 0xE0, 0x3E, 0xAA, 0xBF, 0x00, 0x00,
  0x07, 0xE0, 0x3D, 0x00, 0x2F, 0x00, 0x0A, 0xFF, 0xC0, 0x3D, 0x00, 0x1F,
  0x00, 0x0B, 0xFF, 0x40, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE4, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  // 0x4f8b - 0x4f8b
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x00,
  0x00, 0x00, 0x06, 0x40, 0x00, 0x07, 0xE0, 0x00, 0x00, 0x00, 0x0B, 0x80,
  0x00, 0x0B, 0xCB, 0xFF, 0xFF, 0x00, 0x0B, 0x80, 0x00, 0x0F, 0x8B, 0xFF,
  0xFF, 0x00, 0x0B, 0x80, 0x00, 0x1F, 0x00, 0x3D, 0x00, 0x15, 0x0B, 0x80,
  0x00, 0x3F, 0x00, 0x3C, 0x00, 0x3F, 0x0B, 0x80, 0x00, 0x7E, 0x00, 0x7C,
  0x00, 0x2E, 0x0B, 0x80, 0x00, 0xFE, 0x00, 0x7C, 0x00, 0x2E, 0x0B, 0x80,
  0x01, 0xFE, 0x00, 0xBC, 0x00, 0x2E, 0x0B, 0x80, 0x03, 0xFE, 0x00, 0xFF,
  0xFE, 0x2E, 0x0B, 0x82, 0x0B, 0xFE, 0x00, 0xFA, 0xBD, 0x2E, 0x0B, 0x87,
  0x1F, 0xEE, 0x02, 0xF0, 0x3D, 0x2E, 0x0B, 0x8B, 0x2F, 0x2E, 0x03, 0xE0,
  0x3D, 0x2E, 0x0B, 0x81, 0x06, 0x2E, 0x0B, 0xC0, 0x3C, 0x2E, 0x0B, 0x80,
  0x00, 0x2E, 0x1F, 0x80, 0x7C, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x3F, 0x3C,
  0xBC, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x1D, 0x7F, 0xF8, 0x2E, 0x0B, 0x80,
  0x00, 0x2E, 0x00, 0x0F, 0xF4, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x02,
  0xF0, 0x2E, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x03, 0xE0, 0x2E, 0x0B, 0x80,
  0x00, 0x2E, 0x00, 0x07, 0xC0, 0x19, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x0F,
  0x80, 0x00, 0x0B, 0x80, 0x00, 0x2E, 0x00, 0x3F, 0x00, 0x00, 0x0B, 0x80,
  0x00, 0x2E, 0x00, 0xBE, 0x00, 0x00, 0x0B, 0x80, 0x00, 0x2E, 0x03, 0xFC,
  0x00, 0x00, 0x0F, 0x80, 0x00, 0x2E, 0x1F, 0xF0, 0x00, 0x03, 0xFF, 0x40,
  0x00, 0x3E, 0x0F, 0xC0, 0x00, 0x03, 0xFF, 0x00, 0x00, 0x2E, 0x02, 0x00,
  0x00, 0x01, 0xA4, 0x00,
  // 0x7a0b - 0x7a0b
  0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0xF8, 0x00,
  0x00, 0x00, 0x00, 0x06, 0x1B, 0xFF, 0xFD, 0x1F, 0xFF, 0xFF, 0xFC, 0x0B,
  0x2F, 0xFF, 0x40, 0x1F, 0xFF, 0xFF, 0xFC, 0x02, 0x09, 0x1F, 0x00, 0x0F,
  0x00, 0x00, 0x7C, 0x00, 0x00, 0x0F, 0x00, 0x0F, 0x00, 0x00, 0x7C, 0x00,
  0x00, 0x0F, 0x00, 0x0F, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x0F, 0x00, 0x1F,
  0x00, 0x00, 0x7C, 0x00, 0x00, 0x0F, 0x00, 0x1F, 0x55, 0x55, 0xBC, 0x0A,
  0x2A, 0xAF, 0xA9, 0x1F, 0xFF, 0xFF, 0xFC, 0x0F, 0x3F, 0xFF, 0xFE, 0x1F,
  0xFF, 0xFF, 0xFC, 0x0A, 0x29, 0x7F, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0x00,
  0x00, 0xFF, 0x7C, 0x3F, 0xFF, 0xFF, 0xFF, 0x00, 0x02, 0xFF, 0x3D, 0x00,
  0x03, 0xD0, 0x00, 0x00, 0x03, 0xEF, 0x2F, 0x00, 0x03, 0xD0, 0x00, 0x03,
  0x0F, 0xCF, 0x0F, 0x40, 0x03, 0xD0, 0x00, 0x0B, 0x2F, 0x4F, 0x09, 0x05,
  0x57, 0xE5, 0x54, 0x2F, 0xBE, 0x0F, 0x00, 0x0F, 0xFF, 0xFF, 0xFC, 0x1F,
  0x7C, 0x0F, 0x00, 0x0F, 0xFF, 0xFF, 0xFC, 0x05, 0x14, 0x0F, 0x00, 0x00,
  0x03, 0xD0, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x03, 0xD0, 0x00, 0x00,
  0x00, 0x0F, 0x00, 0x00, 0x03, 0xD0, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00,
  0x03, 0xD0, 0x00, 0x00, 0x00, 0x1F, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0,
  0x00, 0x1F, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x1F, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs0[] = {
  {     0,  29,  31,    0,  -25 },   // 0x4F8B
  {   232,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs1[] = {
  {   232,  30,  31,    1,  -24 },   // 0x662F
  {   464,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs2[] = {
  {   464,  29,  31,    1,  -24 },   // 0x6C49
  {   696,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs3[] = {
  {   696,  28,  31,    2,  -25 },   // 0x5B57
  {   906,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs4[] = {
  {   906,  29,  31,    1,  -25 },   // 0x5E93
  {  1146,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs5[] = {
  {  1146,  26,  31,    3,  -24 },   // 0x56FE
  {  1349,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs6[] = {
  {  1349,  29,  31,    1,  -24 },   // 0x5F62
  {  1573,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs7[] = {
  {  1573,  27,  31,    2,  -25 },   // 0x7684
  {  1783,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs8[] = {
  {  1783,  29,  31,    0,  -25 },   // 0x4F8B
  {  2015,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_glyph_t g_sSimhei16pt2bGlyphs9[] = {
  {  2015,  29,  31,    1,  -25 },   // 0x7A0B
  {  2255,   0,   0,    0,    0 }
};

// This struct will be read by the CPU only
static const nema_font_range_t simhei16pt2b_ranges[] = {
  {0x00004f8b, 0x00004f8b, g_sSimhei16pt2bGlyphs0},
  {0x0000662f, 0x0000662f, g_sSimhei16pt2bGlyphs1},
  {0x00006c49, 0x00006c49, g_sSimhei16pt2bGlyphs2},
  {0x00005b57, 0x00005b57, g_sSimhei16pt2bGlyphs3},
  {0x00005e93, 0x00005e93, g_sSimhei16pt2bGlyphs4},
  {0x000056fe, 0x000056fe, g_sSimhei16pt2bGlyphs5},
  {0x00005f62, 0x00005f62, g_sSimhei16pt2bGlyphs6},
  {0x00007684, 0x00007684, g_sSimhei16pt2bGlyphs7},
  {0x00004f8b, 0x00004f8b, g_sSimhei16pt2bGlyphs8},
  {0x00007a0b, 0x00007a0b, g_sSimhei16pt2bGlyphs9},
  {0, 0, NULL}
};

// This struct will be read by the CPU only
nema_font_t simhei16pt2b = {
  {
    .base_virt = (void *) g_sSimhei16pt2bBitmaps,
    .base_phys = (uintptr_t) g_sSimhei16pt2bBitmaps,
    .size      = (int) sizeof(g_sSimhei16pt2bBitmaps)
  },
  simhei16pt2b_ranges,
  (int)sizeof(g_sSimhei16pt2bBitmaps),
  g_sSimhei16pt2bBitmaps,
  0,
  16, 36, 26, 2
};
#endif // SIMHEI16PT2B_C

