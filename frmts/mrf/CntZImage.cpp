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

#include "CntZImage.h"
#include "BitStuffer.h"
#include "BitMask.h"
#include "Defines.h"
#include <cmath>
#include <cfloat>
#include <cstring>

#if defined(LERC_DEBUG)
#define PRINT(X) X
#include <cstdio>
#else
#define PRINT(X)
#endif

NAMESPACE_MRF_START

// -------------------------------------------------------------------------- ;

using namespace std;

// -------------------------------------------------------------------------- ;

CntZImage::CntZImage()
{
  type_ = CNT_Z;
  memset(&m_infoFromComputeNumBytes, 0, sizeof(m_infoFromComputeNumBytes));
};

// -------------------------------------------------------------------------- ;

bool CntZImage::resizeFill0(int width, int height)
{
  if (!resize(width, height))
    return false;

  memset(getData(), 0, width * height * sizeof(CntZ));
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::hasValidPixel() const
{
  for (int i = 0; i < height_; i++)
  {
    const CntZ* ptr = data_ + i * width_;
    for (int j = 0; j < width_; j++)
    {
      if (ptr->cnt > 0)
        return true;
      ptr++;
    }
  }
  return false;
}

// -------------------------------------------------------------------------- ;

void CntZImage::normalize()
{
  for (int i = 0; i < height_; i++)
  {
    CntZ* ptr = data_ + i * width_;
    for (int j = 0; j < width_; j++)
    {
      if (ptr->cnt > 0)
      {
        ptr->z /= ptr->cnt;
        ptr->cnt = 1;
      }
      ptr++;
    }
  }
}

// -------------------------------------------------------------------------- ;

// computes the size of a CntZImage of any width and height, but all void / invalid,
// and then compressed

unsigned int CntZImage::computeNumBytesNeededToWriteVoidImage()
{
  unsigned int cnt = 0;

  CntZImage zImg;
  cnt += (unsigned int)zImg.getTypeString().length();    // "CntZImage ", 10 bytes
  cnt += 2 * sizeof(int);
  cnt += 2 * sizeof(int);
  cnt += 1 * sizeof(double);

  // cnt part
  cnt += 3 * sizeof(int);
  cnt += sizeof(float);
//  cnt += zImg.numBytesCntTile(width * height, 0, 0, false);

  // z part
  cnt += 3 * sizeof(int);
  cnt += sizeof(float);
  cnt += zImg.numBytesZTile(0, 0, 0, 0);    // = 1

  return cnt;
}

// -------------------------------------------------------------------------- ;

unsigned int CntZImage::computeNumBytesNeededToWrite(double maxZError,
                                                      bool onlyZPart,
                                                      InfoFromComputeNumBytes& info) const
{
  const string fctName = "Error in CntZImage::computeNumBytesNeededToWrite(...): ";

  unsigned int cnt = 0;

  cnt += (unsigned int)getTypeString().length();
  cnt += 2 * sizeof(int);
  cnt += 2 * sizeof(int);
  cnt += 1 * sizeof(double);

  int numTilesVert, numTilesHori, numBytesOpt;
  float maxValInImg;

  // cnt part first
  if (!onlyZPart)
  {
    float cntMin, cntMax;
    if (!computeCntStats(0, height_, 0, width_, cntMin, cntMax))
      return false;

    bool bCntsNoInt = true;
    numTilesVert = 0;    // no tiling
    numTilesHori = 0;
    maxValInImg = cntMax;

    if (cntMin == cntMax)    // cnt part is const
    {
      bCntsNoInt = fabsf(cntMax - (int)(cntMax + 0.5f)) > 0.0001;
      numBytesOpt = 0;    // nothing else to encode
    }
    else
    {
      bCntsNoInt = cntsNoInt();
      if (!bCntsNoInt && cntMin == 0 && cntMax == 1)    // cnt part is binary mask, use fast RLE class
      {
        // convert to bit mask
        BitMask bitMask(width_, height_);
	// in case bitMask allocation fails
	if (!bitMask.Size()) return 0;
        const CntZ* srcPtr = getData();
        for (int k = 0; k < width_ * height_ ; k++, srcPtr++)
        {
          if (srcPtr->cnt <= 0)
            bitMask.SetInvalid(k);
	  else
	    bitMask.SetValid(k);
        }

        // determine numBytes needed to encode
	numBytesOpt = bitMask.RLEsize();
      }
      else
      {
        if (!findTiling(false, 0, bCntsNoInt, numTilesVert, numTilesHori, numBytesOpt, maxValInImg))
          return 0;
      }
    }

    info.cntsNoInt       = bCntsNoInt;
    info.numTilesVertCnt = numTilesVert;
    info.numTilesHoriCnt = numTilesHori;
    info.numBytesCnt     = numBytesOpt;
    info.maxCntInImg     = maxValInImg;

    cnt += 3 * sizeof(int);
    cnt += sizeof(float);
    cnt += numBytesOpt;
  }

  // z part second
  {
    if (!findTiling(true, maxZError, false, numTilesVert, numTilesHori, numBytesOpt, maxValInImg))
    {
      return 0;
    }

    info.maxZError     = maxZError;
    info.numTilesVertZ = numTilesVert;
    info.numTilesHoriZ = numTilesHori;
    info.numBytesZ     = numBytesOpt;
    info.maxZInImg     = maxValInImg;

    cnt += 3 * sizeof(int);
    cnt += sizeof(float);
    cnt += numBytesOpt;
  }

  return cnt;
}

// -------------------------------------------------------------------------- ;
// if you change the file format, don't forget to update not only write and 
// read functions, and the file version number, but also the computeNumBytes...
// and numBytes... functions

bool CntZImage::write(Byte** ppByte,
                      double maxZError,
                      bool useInfoFromPrevComputeNumBytes,
                      bool onlyZPart) const
{
  const string fctName = "Error in CntZImage::write(...): ";

  const int version = 11;

  assert(ppByte && *ppByte);
  if (getSize() == 0)
    return false;

  int versionSwap = version;
  int typeSwap    = type_;
  int heightSwap = height_;
  int widthSwap  = width_;
  double maxZErrorSwap = maxZError;

  SWAP_4(versionSwap);
  SWAP_4(typeSwap);
  SWAP_4(heightSwap);
  SWAP_4(widthSwap);
  SWAP_8(maxZErrorSwap);

  Byte* ptr = *ppByte;

  memcpy(ptr, getTypeString().c_str(), getTypeString().length());
  ptr += getTypeString().length();

  *((int*)ptr) = versionSwap;  ptr += sizeof(int);
  *((int*)ptr) = typeSwap;     ptr += sizeof(int);
  *((int*)ptr) = heightSwap;  ptr += sizeof(int);
  *((int*)ptr) = widthSwap;   ptr += sizeof(int);
  *((double*)ptr) = maxZErrorSwap;  ptr += sizeof(double);

  *ppByte = ptr;

  InfoFromComputeNumBytes info;
  memset(&info, 0, sizeof(InfoFromComputeNumBytes));

  if (useInfoFromPrevComputeNumBytes && (maxZError == m_infoFromComputeNumBytes.maxZError))
    info = m_infoFromComputeNumBytes;
  else if (0 == computeNumBytesNeededToWrite(maxZError, onlyZPart, info))
    return false;

  for (int iPart = 0; iPart < 2; iPart++)
  {
    bool zPart = iPart ? true : false;    // first cnt part, then z part

    if (!zPart && onlyZPart)
      continue;

    bool bCntsNoInt = false;
    int numTilesVert, numTilesHori, numBytesOpt, numBytesWritten = 0;
    float maxValInImg;

    if (!zPart)
    {
      bCntsNoInt   = info.cntsNoInt;
      numTilesVert = info.numTilesVertCnt;
      numTilesHori = info.numTilesHoriCnt;
      numBytesOpt  = info.numBytesCnt;
      maxValInImg  = info.maxCntInImg;
    }
    else
    {
      numTilesVert = info.numTilesVertZ;
      numTilesHori = info.numTilesHoriZ;
      numBytesOpt  = info.numBytesZ;
      maxValInImg  = info.maxZInImg;
    }

    int numTilesVertSwap = numTilesVert;
    int numTilesHoriSwap = numTilesHori;
    int numBytesOptSwap  = numBytesOpt;
    float maxValInImgSwap = maxValInImg;

    SWAP_4(numTilesVertSwap);
    SWAP_4(numTilesHoriSwap);
    SWAP_4(numBytesOptSwap);
    SWAP_4(maxValInImgSwap);

    ptr = *ppByte;
    *((int*)ptr) = numTilesVertSwap;  ptr += sizeof(int);
    *((int*)ptr) = numTilesHoriSwap;  ptr += sizeof(int);
    *((int*)ptr) = numBytesOptSwap;   ptr += sizeof(int);
    *((float*)ptr) = maxValInImgSwap;  ptr += sizeof(float);

    *ppByte = ptr;
    Byte* bArr = ptr;

    if (!zPart && numTilesVert == 0 && numTilesHori == 0)    // no tiling for cnt part
    {
      if (numBytesOpt > 0)    // cnt part is binary mask, use fast RLE class
      {
        // convert to bit mask
        BitMask bitMask(width_, height_);
        const CntZ* srcPtr = getData();
        for (int k = 0; k < width_ * height_ ; k++, srcPtr++)
        {
	  if (srcPtr->cnt <= 0)
	    bitMask.SetInvalid(k);
	  else
	    bitMask.SetValid(k);
        }

        // RLE encoding, update numBytesWritten
	numBytesWritten = bitMask.RLEcompress(bArr);
      }
    }
    else
    {
      // encode tiles to buffer
      float maxVal;
      if (!writeTiles(zPart, maxZError, bCntsNoInt, numTilesVert, numTilesHori, 
	  bArr, numBytesWritten, maxVal))
        return false;
    }

    if (numBytesWritten != numBytesOpt)
      return false;

    *ppByte += numBytesWritten;
  }

  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::read(Byte** ppByte,
                     double maxZError,
                     bool onlyHeader,
                     bool onlyZPart)
{
  const string fctName = "Error in CntZImage::read(...): ";
  PRINT(fprintf(stderr,"\nIn CntZImage::read\n"););

  assert(ppByte && *ppByte);

  size_t len = getTypeString().length();
  string typeStr(len, '0');

  memcpy(&typeStr[0], *ppByte, len);
  *ppByte += len;

  if (typeStr != getTypeString())
  {
    PRINT(fprintf(stderr,"\nZImage: Wrong Sig\n"););
    return false;
  }

  int version = 0, type = 0;
  int width = 0, height = 0;
  double maxZErrorInFile = 0;

  Byte* ptr = *ppByte;

  version = *((const int*)ptr);   ptr += sizeof(int);
  type = *((const int*)ptr);   ptr += sizeof(int);
  height = *((const int*)ptr);   ptr += sizeof(int);
  width = *((const int*)ptr);   ptr += sizeof(int);
  maxZErrorInFile = *((const double*)ptr);  ptr += sizeof(double);

  *ppByte = ptr;

  SWAP_4(version);
  SWAP_4(type);
  SWAP_4(height);
  SWAP_4(width);
  SWAP_8(maxZErrorInFile);

  assert(version >= 11);
  assert(type == type_);

  if (width > 20000 || height > 20000)
  {
    PRINT(fprintf(stderr,"\nZImage: Too big %d %d\n", width, height););
    return false;
  }

  if (maxZErrorInFile > maxZError)
  {
    PRINT(fprintf(stderr,"\nZImage: Z too big\n"););
    return false;
  }

  if (onlyHeader)
    return true;

  if (!onlyZPart && !resizeFill0(width, height))
    return false;

  for (int iPart = 0; iPart < 2; iPart++)
  {
    bool zPart = iPart ? true : false;    // first cnt part, then z part

    if (!zPart && onlyZPart)
      continue;

    int numTilesVert = 0, numTilesHori = 0, numBytes = 0;
    float maxValInImg = 0;

    ptr = *ppByte;
    numTilesVert = *((int*)ptr);  ptr += sizeof(int);
    numTilesHori = *((int*)ptr);  ptr += sizeof(int);
    numBytes     = *((int*)ptr);  ptr += sizeof(int);
    maxValInImg  = *((float*)ptr); ptr += sizeof(float);

    *ppByte = ptr;
    Byte *bArr = ptr;

    SWAP_4(numTilesVert);
    SWAP_4(numTilesHori);
    SWAP_4(numBytes);
    SWAP_4(maxValInImg);

    if (!zPart && numTilesVert == 0 && numTilesHori == 0)    // no tiling for this cnt part
    {
      if (numBytes == 0)    // cnt part is const
      {
        CntZ* dstPtr = getData();
        for (int i = 0; i < height_; i++)
          for (int j = 0; j < width_; j++)
          {
            dstPtr->cnt = maxValInImg;
            dstPtr++;
          }
      }

      if (numBytes > 0)    // cnt part is binary mask, RLE compressed
      {
        // Read bit mask
        BitMask bitMask(width_, height_);
	if (!bitMask.RLEdecompress(bArr))
	    return false;

	CntZ* dstPtr = getData();
	for (int k = 0; k < width_ * height_; k++, dstPtr++)
	  dstPtr->cnt = bitMask.IsValid(k) ? 1:0;
      }
    }
    else if (!readTiles(zPart, maxZErrorInFile, numTilesVert, numTilesHori, maxValInImg, bArr))
    {
      PRINT(fprintf(stderr,"\nZImage: Can't read tiles \n"););
      return false;
    }

    *ppByte += numBytes;
  }

  m_tmpDataVec.clear();
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::findTiling(bool zPart, double maxZError, bool cntsNoInt,
                           int& numTilesVertA,
                           int& numTilesHoriA,
                           int& numBytesOptA,
                           float& maxValInImgA) const
{
  const string fctName = "Error in CntZImage::findTiling(...): ";

  const int tileWidthArr[] = {8, 11, 15, 20, 32, 64};
  const int numConfigs = 6;

  // first, do the entire image as 1 block
  numTilesVertA = 1;
  numTilesHoriA = 1;
  if (!writeTiles(zPart, maxZError, cntsNoInt, 1, 1, 0, numBytesOptA, maxValInImgA))
  {
//    cout << fctName << "write tiles failed" << endl;
    return false;
  }

  // if cnt part is constant 1 (all valid), or 0 or -1 (all invalid),
  // or if all is invalid so z part is empty, then we have to write the header only
  if (numBytesOptA == (zPart ? numBytesZTile(0, 0, 0, 0) : numBytesCntTile(0, 0, 0, false)))
  {
    //printf("block size = %d,  bpp = %f\n", height_ / numTilesVertA, (float)numBytesOptA * 8 / (height_ * width_));
    return true;
  }

  int numBytesPrev = 0;

  for (int k = 0; k < numConfigs; k++)
  {
    int tileWidth = tileWidthArr[k];

    int numTilesVert = height_ / tileWidth;
    int numTilesHori = width_  / tileWidth;

    if (numTilesVert * numTilesHori < 2)
    {
      return true;
    }

    int numBytes = 0;
    float maxVal;
    if (!writeTiles(zPart, maxZError, cntsNoInt, numTilesVert, numTilesHori, 0, numBytes, maxVal))
      return false;

    if (numBytes < numBytesOptA)
    {
      numTilesVertA = numTilesVert;
      numTilesHoriA = numTilesHori;
      numBytesOptA = numBytes;
    }

    //printf("  block size = %d,  bpp = %f\n", height_ / numTilesVert, (float)numBytes * 8 / (height_ * width_));

    if (k > 0 && numBytes > numBytesPrev)    // we stop once things get worse by further increasing the block size
    {
      //printf("block size = %d,  bpp = %f\n", height_ / numTilesVertA, (float)numBytesOptA * 8 / (height_ * width_));
      return true;
    }

    numBytesPrev = numBytes;
  }

  //printf("block size = %d,  bpp = %f\n", height_ / numTilesVertA, (float)numBytesOptA * 8 / (height_ * width_));
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::writeTiles(bool zPart, double maxZError, bool cntsNoInt,
                           int numTilesVert, int numTilesHori,
                           Byte* bArr, int& numBytes, float& maxValInImg) const
{
  Byte* ptr = bArr;
  numBytes = 0;
  maxValInImg = -FLT_MAX;

  for (int iTile = 0; iTile <= numTilesVert; iTile++)
  {
    int tileH = height_ / numTilesVert;
    int i0 = iTile * tileH;
    if (iTile == numTilesVert)
      tileH = height_ % numTilesVert;

    if (tileH == 0)
      continue;

    for (int jTile = 0; jTile <= numTilesHori; jTile++)
    {
      int tileW = width_ / numTilesHori;
      int j0 = jTile * tileW;
      if (jTile == numTilesHori)
        tileW = width_ % numTilesHori;

      if (tileW == 0)
        continue;

      float cntMin = 0, cntMax = 0, zMin = 0, zMax = 0;
      int numValidPixel = 0;

      bool rv = zPart ? computeZStats(  i0, i0 + tileH, j0, j0 + tileW, zMin, zMax, numValidPixel) :
                        computeCntStats(i0, i0 + tileH, j0, j0 + tileW, cntMin, cntMax);
      if (!rv)
        return false;

      maxValInImg = zPart ? max(zMax, maxValInImg) : max(cntMax, maxValInImg);

      int numBytesNeeded = zPart ? numBytesZTile(numValidPixel, zMin, zMax, maxZError) :
                                    numBytesCntTile(tileH * tileW, cntMin, cntMax, cntsNoInt);
      numBytes += numBytesNeeded;

      if (bArr)
      {
        int numBytesWritten;
        bool rv = zPart ? writeZTile(  &ptr, numBytesWritten, i0, i0 + tileH, j0, j0 + tileW, numValidPixel, zMin, zMax, maxZError) :
                          writeCntTile(&ptr, numBytesWritten, i0, i0 + tileH, j0, j0 + tileW, cntMin, cntMax, cntsNoInt);
        if (!rv)
          return false;
        if (numBytesWritten != numBytesNeeded)
          return false;
      }
    }
  }

  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::readTiles(bool zPart, double maxZErrorInFile,
                          int numTilesVert, int numTilesHori, float maxValInImg,
                          Byte* bArr)
{
//  const string fctName = "Error in CntZImage::readTiles(...): ";
  Byte* ptr = bArr;

  for (int iTile = 0; iTile <= numTilesVert; iTile++)
  {
    int tileH = height_ / numTilesVert;
    int i0 = iTile * tileH;
    if (iTile == numTilesVert)
      tileH = height_ % numTilesVert;

    if (tileH == 0)
      continue;

    for (int jTile = 0; jTile <= numTilesHori; jTile++)
    {
      int tileW = width_ / numTilesHori;
      int j0 = jTile * tileW;
      if (jTile == numTilesHori)
        tileW = width_ % numTilesHori;

      if (tileW == 0)
        continue;

      bool rv = zPart ? readZTile(  &ptr, i0, i0 + tileH, j0, j0 + tileW, maxZErrorInFile, maxValInImg) :
                        readCntTile(&ptr, i0, i0 + tileH, j0, j0 + tileW);

      if (!rv)
        return false;
    }
  }

  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::cntsNoInt() const
{
  float cntMaxErr = 0;
  for (int i = 0; i < height_; i++)
  {
    const CntZ* ptr = data_ + i * width_;
    for (int j = 0; j < width_; j++)
    {
      float cntErr = fabsf(ptr->cnt - (int)(ptr->cnt + 0.5f));
      cntMaxErr = max(cntErr, cntMaxErr);
      ptr++;
    }
  }
  return (cntMaxErr > 0.0001);
}

// -------------------------------------------------------------------------- ;

bool CntZImage::computeCntStats(int i0, int i1, int j0, int j1,
                                float& cntMinA, float& cntMaxA) const
{
//  const string fctName = "Error in CntZImage::computeCntStats(...): ";

  if (i0 < 0 || j0 < 0 || i1 > height_ || j1 > width_)
    return false;

  // determine cnt ranges
  float cntMin =  FLT_MAX;
  float cntMax = -FLT_MAX;

  for (int i = i0; i < i1; i++)
  {
    const CntZ* ptr = data_ + i * width_ + j0;
    for (int j = j0; j < j1; j++)
    {
      cntMin = min(ptr->cnt, cntMin);
      cntMax = max(ptr->cnt, cntMax);
      ptr++;
    }
  }

  cntMinA = cntMin;
  cntMaxA = cntMax;
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::computeZStats(int i0, int i1, int j0, int j1,
                              float& zMinA, float& zMaxA, int& numValidPixelA) const
{

  if (i0 < 0 || j0 < 0 || i1 > height_ || j1 > width_)
    return false;

  // determine z ranges
  float zMin =  FLT_MAX;
  float zMax = -FLT_MAX;
  int numValidPixel = 0;

  for (int i = i0; i < i1; i++)
  {
    const CntZ* ptr = data_ + i * width_ + j0;
    for (int j = j0; j < j1; j++)
    {
      if (ptr->cnt > 0)    // cnt <= 0 means ignore z
      {
        zMin = min(ptr->z, zMin);
        zMax = max(ptr->z, zMax);
        numValidPixel++;
      }
      ptr++;
    }
  }

  if (zMin > zMax)
  {
    zMin = 0;
    zMax = 0;
  }

  zMinA = zMin;
  zMaxA = zMax;
  numValidPixelA = numValidPixel;

  return true;
}

// -------------------------------------------------------------------------- ;

int CntZImage::numBytesCntTile(int numPixel, float cntMin, float cntMax, bool cntsNoInt) const
{
  if (cntMin == cntMax && (cntMin == 0 || cntMin == -1 || cntMin == 1))
    return 1;

  if (cntsNoInt || (cntMax - cntMin) > (1 << 28))
  {
    return(int)(1 + numPixel * sizeof(float));
  }
  else
  {
    unsigned int maxElem = (unsigned int)(cntMax - cntMin + 0.5f);
    return 1 + numBytesFlt(floorf(cntMin + 0.5f)) + BitStuffer::computeNumBytesNeeded(numPixel, maxElem);
  }
}

// -------------------------------------------------------------------------- ;

int CntZImage::numBytesZTile(int numValidPixel, float zMin, float zMax, double maxZError) const
{
  if (numValidPixel == 0 || (zMin == 0 && zMax == 0))
    return 1;

  if (maxZError == 0 || (double)(zMax - zMin) / (2 * maxZError) > (1 << 28))
  {
    return(int)(1 + numValidPixel * sizeof(float));
  }
  else
  {
    unsigned int maxElem = (unsigned int)((double)(zMax - zMin) / (2 * maxZError) + 0.5);
    if (maxElem == 0)
      return 1 + numBytesFlt(zMin);
    else
      return 1 + numBytesFlt(zMin) + BitStuffer::computeNumBytesNeeded(numValidPixel, maxElem);
  }
}

// -------------------------------------------------------------------------- ;

bool CntZImage::writeCntTile(Byte** ppByte, int& numBytes,
                             int i0, int i1, int j0, int j1,
                             float cntMin, float cntMax, bool cntsNoInt) const
{
  Byte* ptr = *ppByte;
  int numPixel = (i1 - i0) * (j1 - j0);

  if (cntMin == cntMax && (cntMin == 0 || cntMin == -1 || cntMin == 1))    // special case all constant 0, -1, or 1
  {
    if (cntMin == 0)
      *ptr++ = 2;
    else if (cntMin == -1)
      *ptr++ = 3;
    else if (cntMin == 1)
      *ptr++ = 4;

    numBytes = 1;
    *ppByte = ptr;
    return true;
  }

  if (cntsNoInt || cntMax - cntMin > (1 << 28))
  {
    // write cnt's as flt arr uncompressed
    *ptr++ = 0;
    float* dstPtr = (float*)ptr;

    for (int i = i0; i < i1; i++)
    {
      const CntZ* srcPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
      {
        *dstPtr = srcPtr->cnt;
        SWAP_4(*dstPtr);
        srcPtr++;
        dstPtr++;
      }
    }

    ptr += numPixel * sizeof(float);
  }
  else
  {
    // write cnt's as int arr bit stuffed
    Byte flag = 1;
    float offset = floorf(cntMin + 0.5f);
    int n = numBytesFlt(offset);
    int bits67 = (n == 4) ? 0 : 3 - n;
    flag |= bits67 << 6;

    *ptr++ = flag;

    if (!writeFlt(&ptr, offset, n))
      return false;

    vector<unsigned int> dataVec(numPixel, 0);
    unsigned int* dstPtr = &dataVec[0];

    for (int i = i0; i < i1; i++)
    {
      const CntZ* srcPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
      {
        *dstPtr++ = (int)(srcPtr->cnt - offset + 0.5f);
        srcPtr++;
      }
    }

    BitStuffer bitStuffer;
    if (!bitStuffer.write(&ptr, dataVec))
      return false;
  }

  numBytes = (int)(ptr - *ppByte);
  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::writeZTile(Byte** ppByte, int& numBytes, 
                           int i0, int i1, int j0, int j1,
                           int numValidPixel,
                           float zMin, float zMax, double maxZError) const
{
//  const string fctName = "Error in CntZImage::writeZTile(...): ";

  Byte* ptr = *ppByte;
  int cntPixel = 0;

  if (numValidPixel == 0 || (zMin == 0 && zMax == 0))    // special cases
  {
    *ptr++ = 2;    // set compression flag to 2 to mark tile as constant 0
    numBytes = 1;
    *ppByte = ptr;
    return true;
  }

  if (maxZError == 0 ||                                       // user asks lossless OR
      (double)(zMax - zMin) / (2 * maxZError) > (1 << 28))    // we'd need > 28 bit
  {
    // write z's as flt arr uncompressed
    *ptr++ = 0;
    float* dstPtr = (float*)ptr;

    for (int i = i0; i < i1; i++)
    {
      const CntZ* srcPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
      {
        if (srcPtr->cnt > 0)
        {
          *dstPtr = srcPtr->z;
          SWAP_4(*dstPtr);
          dstPtr++;
          cntPixel++;
        }
        srcPtr++;
      }
    }

    if (cntPixel != numValidPixel)
      return false;

    ptr += numValidPixel * sizeof(float);
  }
  else
  {
    // write z's as int arr bit stuffed
    Byte flag = 1;
    unsigned int maxElem = (unsigned int)((double)(zMax - zMin) / (2 * maxZError) + 0.5);
    if (maxElem == 0)
    {
      flag = 3;    // set compression flag to 3 to mark tile as constant zMin
    }

    int n = numBytesFlt(zMin);
    int bits67 = (n == 4) ? 0 : 3 - n;
    flag |= bits67 << 6;

    *ptr++ = flag;

    if (!writeFlt(&ptr, zMin, n))
      return false;

    if (maxElem > 0)
    {
      vector<unsigned int> dataVec(numValidPixel, 0);
      unsigned int* dstPtr = &dataVec[0];
      double scale = 1 / (2 * maxZError);

      for (int i = i0; i < i1; i++)
      {
        const CntZ* srcPtr = getData() + i * width_ + j0;
        for (int j = j0; j < j1; j++)
        {
          if (srcPtr->cnt > 0)
          {
            *dstPtr++ = (unsigned int)((srcPtr->z - zMin) * scale + 0.5);
            cntPixel++;
          }
          srcPtr++;
        }
      }

      if (cntPixel != numValidPixel)
        return false;

      BitStuffer bitStuffer;
      if (!bitStuffer.write(&ptr, dataVec))
        return false;
    }
  }

  numBytes = (int)(ptr - *ppByte);
  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::readCntTile(Byte** ppByte, int i0, int i1, int j0, int j1)
{
  Byte* ptr = *ppByte;
  int numPixel = (i1 - i0) * (j1 - j0);

  Byte comprFlag = *ptr++;

  if (comprFlag == 2)    // entire tile is constant 0 (invalid)
  {                      // here we depend on resizeFill0()
    *ppByte = ptr;
    return true;
  }

  if (comprFlag == 3 || comprFlag == 4)    // entire tile is constant -1 (invalid) or 1 (valid)
  {
    CntZ cz1m = {-1, 0};
    CntZ cz1p = { 1, 0};
    CntZ cz1 = (comprFlag == 3) ? cz1m : cz1p;

    for (int i = i0; i < i1; i++)
    {
      CntZ* dstPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
        *dstPtr++ = cz1;
    }

    *ppByte = ptr;
    return true;
  }

  if ((comprFlag & 63) > 4)
    return false;

  if (comprFlag == 0)
  {
    // read cnt's as flt arr uncompressed
    const float* srcPtr = (const float*)ptr;

    for (int i = i0; i < i1; i++)
    {
      CntZ* dstPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
      {
        dstPtr->cnt = *srcPtr++;
        SWAP_4(dstPtr->cnt);
        dstPtr++;
      }
    }

    ptr += numPixel * sizeof(float);
  }
  else
  {
    // read cnt's as int arr bit stuffed
    int bits67 = comprFlag >> 6;
    int n = (bits67 == 0) ? 4 : 3 - bits67;

    float offset = 0;
    if (!readFlt(&ptr, offset, n))
      return false;

    vector<unsigned int>& dataVec = m_tmpDataVec;
    BitStuffer bitStuffer;
    if (!bitStuffer.read(&ptr, dataVec))
      return false;

    unsigned int* srcPtr = &dataVec[0];

    for (int i = i0; i < i1; i++)
    {
      CntZ* dstPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
      {
        dstPtr->cnt = offset + (float)(*srcPtr++);
        dstPtr++;
      }
    }
  }

  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::readZTile(Byte** ppByte, int i0, int i1, int j0, int j1,
                          double maxZErrorInFile, float maxZInImg)
{
//  const string fctName = "Error in CntZImage::readZTile(...): ";

  Byte* ptr = *ppByte;
  int numPixel = 0;

  Byte comprFlag = *ptr++;
  int bits67 = comprFlag >> 6;
  comprFlag &= 63;

  if (comprFlag == 2)    // entire zTile is constant 0 (if valid or invalid doesn't matter)
  {
    for (int i = i0; i < i1; i++)
    {
      CntZ* dstPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
      {
        if (dstPtr->cnt > 0)
          dstPtr->z = 0;
        dstPtr++;
      }
    }

    *ppByte = ptr;
    return true;
  }

  if (comprFlag > 3)
    return false;

  if (comprFlag == 0)
  {
    // read z's as flt arr uncompressed
    const float* srcPtr = (const float*)ptr;

    for (int i = i0; i < i1; i++)
    {
      CntZ* dstPtr = getData() + i * width_ + j0;
      for (int j = j0; j < j1; j++)
      {
        if (dstPtr->cnt > 0)
        {
          dstPtr->z = *srcPtr++;
          SWAP_4(dstPtr->z);
          numPixel++;
        }
        dstPtr++;
      }
    }

    ptr += numPixel * sizeof(float);
  }
  else
  {
    // read z's as int arr bit stuffed
    int n = (bits67 == 0) ? 4 : 3 - bits67;
    float offset = 0;
    if (!readFlt(&ptr, offset, n))
      return false;

    if (comprFlag == 3)
    {
      for (int i = i0; i < i1; i++)
      {
        CntZ* dstPtr = getData() + i * width_ + j0;
        for (int j = j0; j < j1; j++)
        {
          if (dstPtr->cnt > 0)
            dstPtr->z = offset;
          dstPtr++;
        }
      }
    }
    else
    {
      vector<unsigned int>& dataVec = m_tmpDataVec;
      BitStuffer bitStuffer;
      if (!bitStuffer.read(&ptr, dataVec))
        return false;

      double invScale = 2 * maxZErrorInFile;
      unsigned int* srcPtr = &dataVec[0];

      for (int i = i0; i < i1; i++)
      {
        CntZ* dstPtr = getData() + i * width_ + j0;
        for (int j = j0; j < j1; j++)
        {
          if (dstPtr->cnt > 0)
          {
            float z = (float)(offset + *srcPtr++ * invScale);
            dstPtr->z = min(z, maxZInImg);    // make sure we stay in the orig range
          }
          dstPtr++;
        }
      }
    }
  }

  *ppByte = ptr;
  return true;
}

// -------------------------------------------------------------------------- ;

int CntZImage::numBytesFlt(float z) const
{
  short s = (short)z;
  signed char c = (signed char)s;
  return ((float)c == z) ? 1 : ((float)s == z) ? 2 : 4;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::writeFlt(Byte** ppByte, float z, int numBytes) const
{
  Byte* ptr = *ppByte;

  if (numBytes == 1)
  {
    char c = (char)z;
    *((char*)ptr) = c;
  }
  else if (numBytes == 2)
  {
    short s = (short)z;
    SWAP_2(s);
    *((short*)ptr) = s;
  }
  else if (numBytes == 4)
  {
    SWAP_4(z);
    *((float*)ptr) = z;
  }
  else
    return false;

  *ppByte = ptr + numBytes;
  return true;
}

// -------------------------------------------------------------------------- ;

bool CntZImage::readFlt(Byte** ppByte, float& z, int numBytes) const
{
  Byte* ptr = *ppByte;

  if (numBytes == 1)
  {
    char c = *((char*)ptr);
    z = c;
  }
  else if (numBytes == 2)
  {
    short s = *((short*)ptr);
    SWAP_2(s);
    z = s;
  }
  else if (numBytes == 4)
  {
    z = *((float*)ptr);
    SWAP_4(z);
  }
  else
    return false;

  *ppByte = ptr + numBytes;
  return true;
}

// ADJUST goes LSB to MSB in bytes, regardless of endianess
// POFFSET is the relative byte offset for a given type, for native endian
#if defined BIG_ENDIAN
#define ADJUST(ptr) (--(ptr))
#define POFFSET(T) sizeof(T)
#else
#define ADJUST(ptr) ((ptr)++)
#define POFFSET(T) 0
#endif

#define NEXTBYTE (*(*ppByte)++)

// endianess and alignment safe
// returns the number of bytes it wrote, adjusts *ppByte
int CntZImage::writeVal(Byte **ppByte, float z, int numBytes) const
{
    assert(ppByte && *ppByte);
    assert(0 == numBytes || 1 == numBytes || 2 == numBytes || 4 == numBytes);

    short s = (short)z;
    // Calculate numBytes if needed
    if (0 == numBytes) 
	numBytes = (z != (float)s) ? 4 : (s != (signed char)s) ? 2 : 1;

    if (4 == numBytes) {
	// Store as floating point, 4 bytes in LSB order
	Byte *fp = (Byte *)&z + POFFSET(float);
	NEXTBYTE = *ADJUST(fp);
	NEXTBYTE = *ADJUST(fp);
	NEXTBYTE = *ADJUST(fp);
	NEXTBYTE = *ADJUST(fp);
	return 4;
    }

    // Int types, 2 or 1 bytes
    NEXTBYTE = (Byte)s; // Lower byte first
    if (2 == numBytes)
	NEXTBYTE = (Byte)(s >> 8); // Second byte
    return numBytes;
}

// endianess and alignment safe, not alliasing safe
void CntZImage::readVal(Byte **ppByte, float &val, int numBytes) const
{
    assert(numBytes == 4 || numBytes == 2 || numBytes == 1);
    assert(ppByte && *ppByte);
    assert(abs((Byte *)&val - *ppByte) >= 4); // Alliasing check

    // Floating point, read the 4 bytes in LSB first order
    if (4 == numBytes) {
	Byte *vp = ((Byte*)&val) + POFFSET(float);
	*ADJUST(vp) = NEXTBYTE;
	*ADJUST(vp) = NEXTBYTE;
	*ADJUST(vp) = NEXTBYTE;
	*ADJUST(vp) = NEXTBYTE;
	return;
    }

    int v = (int)((signed char)NEXTBYTE); // Low byte, signed extended
    if (2 == numBytes)
	v = (256 * (signed char)NEXTBYTE) | (v && 0xff);
    val = v;
}

NAMESPACE_MRF_END
