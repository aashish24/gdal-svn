/******************************************************************************
 * $Id$
 *
 * Project:  PDF driver
 * Purpose:  GDALDataset driver for PDF dataset.
 * Author:   Even Rouault, <even dot rouault at mines dash paris dot org>
 *
 ******************************************************************************
 * Copyright (c) 2012, Even Rouault
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#ifndef PDFCREATECOPY_H_INCLUDED
#define PDFCREATECOPY_H_INCLUDED

#include "pdfobject.h"
#include "gdal_priv.h"
#include <vector>

typedef enum
{
    COMPRESS_NONE,
    COMPRESS_DEFLATE,
    COMPRESS_JPEG,
    COMPRESS_JPEG2000
} PDFCompressMethod;

/************************************************************************/
/*                          GDALPDFWriter                               */
/************************************************************************/

class GDALXRefEntry
{
    public:
        vsi_l_offset    nOffset;
        int             nGen;
        int             bFree;

        GDALXRefEntry() : nOffset(0), nGen(0), bFree(FALSE) {}
        GDALXRefEntry(vsi_l_offset nOffsetIn, int nGenIn = 0) : nOffset(nOffsetIn), nGen(nGen), bFree(FALSE) {}
        GDALXRefEntry(const GDALXRefEntry& oOther) : nOffset(oOther.nOffset), nGen(oOther.nGen), bFree(oOther.bFree) {}
        GDALXRefEntry& operator= (const GDALXRefEntry& oOther) { nOffset = oOther.nOffset; nGen = oOther.nGen; bFree = oOther.bFree; return *this; }
};

class GDALPDFWriter
{
    VSILFILE* fp;
    std::vector<GDALXRefEntry> asXRefEntries;
    std::vector<int> asPageId;

    int nInfoId;
    int nInfoGen;
    int nPageResourceId;
    int nCatalogId;
    int nCatalogGen;
    int nXMPId;
    int nXMPGen;
    int bInWriteObj;

    int nLastStartXRef;
    int nLastXRefSize;
    int bCanUpdate;

    void    Init();

    void    StartObj(int nObjectId, int nGen = 0);
    void    EndObj();
    void    WriteXRefTableAndTrailer();
    void    WritePages();
    int     WriteBlock( GDALDataset* poSrcDS,
                        int nXOff, int nYOff, int nReqXSize, int nReqYSize,
                        int nColorTableId,
                        PDFCompressMethod eCompressMethod,
                        int nPredictor,
                        int nJPEGQuality,
                        const char* pszJPEG2000_DRIVER,
                        GDALProgressFunc pfnProgress,
                        void * pProgressData );
    int     WriteMask(GDALDataset* poSrcDS,
                      int nXOff, int nYOff, int nReqXSize, int nReqYSize,
                      PDFCompressMethod eCompressMethod);

    int     AllocNewObject();

    public:
        GDALPDFWriter(VSILFILE* fpIn, int bAppend = FALSE);
       ~GDALPDFWriter();

       void Close();

       int  GetCatalogNum() { return nCatalogId; }
       int  GetCatalogGen() { return nCatalogGen; }

       int  ParseTrailerAndXRef();
       void UpdateProj(GDALDataset* poSrcDS,
                       double dfDPI,
                       GDALPDFDictionaryRW* poPageDict,
                       int nPageNum, int nPageGen);
       void UpdateInfo(GDALDataset* poSrcDS);
       void UpdateXMP (GDALDataset* poSrcDS,
                       GDALPDFDictionaryRW* poCatalogDict);

       int     WriteSRS_ISO32000(GDALDataset* poSrcDS,
                                double dfUserUnit,
                                const char* pszNEATLINE);
       int     WriteSRS_OGC_BP(GDALDataset* poSrcDS,
                                double dfUserUnit,
                                const char* pszNEATLINE);

       int  WritePage(GDALDataset* poSrcDS,
                      double dfDPI,
                      const char* pszGEO_ENCODING,
                      const char* pszNEATLINE,
                      PDFCompressMethod eCompressMethod,
                      int nPredictor,
                      int nJPEGQuality,
                      const char* pszJPEG2000_DRIVER,
                      int nBlockXSize, int nBlockYSize,
                      GDALProgressFunc pfnProgress,
                      void * pProgressData);
       int  SetInfo(GDALDataset* poSrcDS,
                    char** papszOptions);
       int  SetXMP(GDALDataset* poSrcDS,
                   const char* pszXMP);
};

GDALDataset         *GDALPDFCreateCopy( const char *, GDALDataset *,
                                        int, char **,
                                        GDALProgressFunc pfnProgress,
                                        void * pProgressData );

#endif // PDFCREATECOPY_H_INCLUDED
