/*
Copyright 2015 Esri
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
A local copy of the license and additional notices are located with the
source distribution at:
http://github.com/Esri/lerc/
Contributors:
    Thomas Maurer
    Lucian Plesea
*/

#pragma once
// For std::pair
#include <utility>
#include <cstddef>

#ifdef GDAL_COMPILATION
#include "marfa.h"
#else
#define NAMESPACE_MRF_START
#define NAMESPACE_MRF_END
#endif

NAMESPACE_MRF_START

typedef unsigned char Byte;

// unsigned long pair sortable by first
struct Quant : public std::pair<unsigned long, unsigned long>
{
    // This is default behavior in C++14, but not before
    bool operator<(const Quant& other) const {
	return first < other.first;
    }
};

#ifdef SWAPB    // define this on big endian system

#define LITTLE2LOCAL_ENDIAN_WORD(x) \
  ( ((*((uint16_t *)(&x)) & 0xff00) >>  8) \
  | ((*((uint16_t *)(&x)) & 0x00ff) <<  8) )

#define LITTLE2LOCAL_ENDIAN_DWORD(x) \
  ( ((*((uint32_t *)(&x)) & 0xff000000) >> 24) \
  | ((*((uint32_t *)(&x)) & 0x00ff0000) >>  8) \
  | ((*((uint32_t *)(&x)) & 0x0000ff00) <<  8) \
  | ((*((uint32_t *)(&x)) & 0x000000ff) << 24) )

#define LITTLE2LOCAL_ENDIAN_QWORD(x) \
  ( ((*((uint64_t *)(&x)) & 0xff00000000000000) >> 56) \
  | ((*((uint64_t *)(&x)) & 0x00ff000000000000) >> 40) \
  | ((*((uint64_t *)(&x)) & 0x0000ff0000000000) >> 24) \
  | ((*((uint64_t *)(&x)) & 0x000000ff00000000) >>  8) \
  | ((*((uint64_t *)(&x)) & 0x00000000ff000000) <<  8) \
  | ((*((uint64_t *)(&x)) & 0x0000000000ff0000) << 24) \
  | ((*((uint64_t *)(&x)) & 0x000000000000ff00) << 40) \
  | ((*((uint64_t *)(&x)) & 0x00000000000000ff) << 56) )

#define SWAP_2(x) *((uint16_t *)&x) = LITTLE2LOCAL_ENDIAN_WORD(x)
#define SWAP_4(x) *((uint32_t *)&x) = LITTLE2LOCAL_ENDIAN_DWORD(x)
#define SWAP_8(x) *((uint64_t *)&x) = LITTLE2LOCAL_ENDIAN_QWORD(x)

#else // SWAPB

#define SWAP_2(x)
#define SWAP_4(x)
#define SWAP_8(x)

#endif // SWAPB

NAMESPACE_MRF_END
