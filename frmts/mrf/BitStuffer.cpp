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
Contributors:  Thomas Maurer
*/

#include "BitStuffer.h"
#include <cstring>

// -------------------------------------------------------------------------- ;

using namespace std;

NAMESPACE_MRF_START

// -------------------------------------------------------------------------- ;

// see the old stream IO functions below on how to call.
// if you change write(...) / read(...), don't forget to update computeNumBytesNeeded(...).

bool BitStuffer::write(Byte** ppByte, const vector<unsigned int>& dataVec) const
{
//  const string fctName = "Error in BitStuffer::write(...): ";

  if (!ppByte || dataVec.empty())
    return false;

  unsigned int maxElem = findMax(dataVec);

  int numBits = 0;
  while (maxElem >> numBits)
    numBits++;
  Byte numBitsByte = (Byte)numBits;
  unsigned int numElements = (unsigned int)dataVec.size();
  unsigned int numULongs = (numElements * numBits + 31) / 32;

  // use the upper 2 bits to encode the type used for numElements: Byte, ushort, or uint
  int n = numBytesULong(numElements);
  int bits67 = (n == 4) ? 0 : 3 - n;
  numBitsByte |= bits67 << 6;

  **ppByte = numBitsByte;
  (*ppByte)++;

  if (!writeULong(ppByte, numElements, n))
    return false;

  if (numULongs > 0)    // numBits can be 0, then we only write the header
  {
    unsigned int numBytes = numULongs * sizeof(unsigned int);
    unsigned int* arr = (unsigned int*)(*ppByte);

    memset(arr, 0, numBytes);

    // do the stuffing
    const unsigned int* srcPtr = &dataVec[0];
    unsigned int* dstPtr = arr;
    int bitPos = 0;

    for (unsigned int i = 0; i < numElements; i++)
    {
      if (32 - bitPos >= numBits)
      {
        *dstPtr |= (*srcPtr++) << (32 - bitPos - numBits);
        bitPos += numBits;
        if (bitPos == 32)    // shift >= 32 is undefined
        {
          bitPos = 0;
          dstPtr++;
        }
      }
      else
      {
        int n = numBits - (32 - bitPos);
        *dstPtr++ |= (*srcPtr  ) >> n;
        *dstPtr   |= (*srcPtr++) << (32 - n);
        bitPos = n;
      }
    }

    // save the 0-3 bytes not used in the last ULong
    unsigned int numBytesNotNeeded = numTailBytesNotNeeded(numElements, numBits);
    unsigned int n = numBytesNotNeeded;
    while (n--)
      *dstPtr >>= 8;

    dstPtr = arr;
    for (unsigned int i = 0; i < numULongs; i++)
    {
      SWAP_4(*dstPtr);
      dstPtr++;
    }

    *ppByte += numBytes - numBytesNotNeeded;
  }

  return true;
}

// -------------------------------------------------------------------------- ;

bool BitStuffer::read(Byte** ppByte, vector<unsigned int>& dataVec) const
{
//   const string fctName = "Error in BitStuffer::read(...): ";

  if (!ppByte)
    return false;

  Byte numBitsByte = **ppByte;
  (*ppByte)++;

  int bits67 = numBitsByte >> 6;
  int n = (bits67 == 0) ? 4 : 3 - bits67;

  numBitsByte &= 63;    // bits 0-5;

  unsigned int numElements = 0;
  if (!readULong(ppByte, numElements, n))
    return false;

  if (numBitsByte >= 32)
    return false;

  int numBits = numBitsByte;
  unsigned int numULongs = (numElements * numBits + 31) / 32;
  dataVec.resize(numElements, 0);    // init with 0

  if (numULongs > 0)    // numBits can be 0
  {
    unsigned int numBytes = numULongs * sizeof(unsigned int);
    unsigned int* arr = (unsigned int*)(*ppByte);

    unsigned int* srcPtr = arr;
    for (unsigned int i = 0; i < numULongs; i++)
    {
      SWAP_4(*srcPtr);
      srcPtr++;
    }

    // needed to save the 0-3 bytes not used in the last ULong
    srcPtr--;
    unsigned int lastULong = *srcPtr;
    unsigned int numBytesNotNeeded = numTailBytesNotNeeded(numElements, numBits);
    unsigned int n = numBytesNotNeeded;
    while (n--)
      *srcPtr <<= 8;

    // do the un-stuffing
    srcPtr = arr;
    unsigned int* dstPtr = &dataVec[0];
    int bitPos = 0;

    for (unsigned int i = 0; i < numElements; i++)
    {
      if (32 - bitPos >= numBits)
      {
        unsigned int n = (*srcPtr) << bitPos;
        *dstPtr++ = n >> (32 - numBits);
        bitPos += numBits;
        if (bitPos == 32)    // shift >= 32 is undefined
        {
          bitPos = 0;
          srcPtr++;
        }
      }
      else
      {
        unsigned int n = (*srcPtr++) << bitPos;
        *dstPtr = n >> (32 - numBits);
        bitPos -= (32 - numBits);
        *dstPtr++ |= (*srcPtr) >> (32 - bitPos);
      }
    }

    if (numBytesNotNeeded > 0)
      *srcPtr = lastULong;    // restore the last ULong

    *ppByte += numBytes - numBytesNotNeeded;
  }

  return true;
}

unsigned int BitStuffer::computeNumBytesNeeded(unsigned int numElem, unsigned int maxElem)
{
  int numBits = 0;
  while (maxElem >> numBits)
    numBits++;
  unsigned int numULongs = (numElem * numBits + 31) / 32;
  unsigned int numBytes = 1 + numBytesULong(numElem) + numULongs * sizeof(unsigned int) -
    numTailBytesNotNeeded(numElem, numBits);

  return numBytes;
}

// -------------------------------------------------------------------------- ;
// -------------------------------------------------------------------------- ;
unsigned int BitStuffer::findMax(const vector<unsigned int>& dataVec) const
{
  unsigned int maxElem = 0;
  for (size_t i = 0; i < dataVec.size(); i++)
      maxElem = max(maxElem, dataVec[i]);
  return maxElem;
}

// -------------------------------------------------------------------------- ;

bool BitStuffer::writeULong(Byte** ppByte, unsigned int k, int numBytes) const
{
  Byte* ptr = *ppByte;

  if (numBytes == 1)
  {
    *ptr = (Byte)k;
  }
  else if (numBytes == 2)
  {
    unsigned short s = (unsigned short)k;
    SWAP_2(s);
    *((unsigned short*)ptr) = s;
  }
  else if (numBytes == 4)
  {
    SWAP_4(k);
    *((unsigned int*)ptr) = k;
  }
  else
    return false;

  *ppByte = ptr + numBytes;
  return true;
}

// -------------------------------------------------------------------------- ;

bool BitStuffer::readULong(Byte** ppByte, unsigned int& k, int numBytes) const
{
  Byte* ptr = *ppByte;

  if (numBytes == 1)
  {
    k = *ptr;
  }
  else if (numBytes == 2)
  {
    unsigned short s = *((unsigned short*)ptr);
    SWAP_2(s);
    k = s;
  }
  else if (numBytes == 4)
  {
    k = *((unsigned int*)ptr);
    SWAP_4(k);
  }
  else
    return false;

  *ppByte = ptr + numBytes;
  return true;
}

// -------------------------------------------------------------------------- ;

unsigned int BitStuffer::numTailBytesNotNeeded(unsigned int numElem, int numBits)
{
  int numBitsTail = (numElem * numBits) & 31;
  int numBytesTail = (numBitsTail + 7) >> 3;
  return (numBytesTail > 0) ? 4 - numBytesTail : 0;
}

// -------------------------------------------------------------------------- ;

NAMESPACE_MRF_END
