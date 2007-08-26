/******************************************************************************
 * $Id: rpftocdataset.cpp
 *
 * Project:  RPF TOC read Translator
 * Purpose:  Implementation of RPFTOCDataset and RPFTOCSubDataset.
 * Author:   Even Rouault, even.rouault at mines-paris.org
 *
 ******************************************************************************
 * Copyright (c) 2007, Even Rouault
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

#include "gdal_pam.h"
#include "rpftoclib.h"
#include "ogr_spatialref.h"
#include "cpl_string.h"
#include "vrtdataset.h"

#define GEOTRSFRM_TOPLEFT_X            0
#define GEOTRSFRM_WE_RES               1
#define GEOTRSFRM_ROTATION_PARAM1      2
#define GEOTRSFRM_TOPLEFT_Y            3
#define GEOTRSFRM_ROTATION_PARAM2      4
#define GEOTRSFRM_NS_RES               5

CPL_CVSID("$Id: rpftocdataset.cpp rouault $");


/************************************************************************/
/* ==================================================================== */
/*                            RPFTOCDataset                             */
/* ==================================================================== */
/************************************************************************/

class RPFTOCDataset : public GDALPamDataset
{
  char	    **papszSubDatasets;
  char       *pszProjection;
  int         bGotGeoTransform;
  double      adfGeoTransform[6];

  public:
    RPFTOCDataset()
    {
        papszSubDatasets = NULL;
        pszProjection = NULL;
        bGotGeoTransform = FALSE;
    }
    
    ~RPFTOCDataset()
    {
        CSLDestroy( papszSubDatasets );
        CPLFree(pszProjection);
    }
    
    virtual char      **GetMetadata( const char * pszDomain = "" );
    
    void                AddSubDataset(const char* pszFilename, RPFTocEntry* tocEntry );
    
    void SetSize(int rasterXSize, int rasterYSize)
    {
        nRasterXSize = rasterXSize;
        nRasterYSize = rasterYSize;
    }
    
    virtual CPLErr GetGeoTransform( double * padfGeoTransform)
    {
        if (bGotGeoTransform)
        {
            memcpy(padfGeoTransform, adfGeoTransform, 6 * sizeof(double));
            return CE_None;
        }
        return CE_Failure;
    }
    
    virtual CPLErr SetGeoTransform( double * padfGeoTransform)
    {
        bGotGeoTransform = TRUE;
        memcpy(adfGeoTransform, padfGeoTransform, 6 * sizeof(double));
        return CE_None;
    }
    
    virtual CPLErr SetProjection( const char * projectionRef )
    {
        CPLFree(pszProjection);
        pszProjection = CPLStrdup(projectionRef);
        return CE_None;
    }
    
    virtual const char *GetProjectionRef(void)
    {
        return (pszProjection) ? pszProjection : "";
    }
    
    static int IsNITFFileTOC(NITFFile *psFile);
    static int IsNonNITFFileTOC(GDALOpenInfo * poOpenInfo, const char* pszFilename );
    static GDALDataset* OpenFileTOC(NITFFile *psFile,
                                    const char* pszFilename,
                                    const char* entryName);
    
    static int Identify( GDALOpenInfo * poOpenInfo );
    static GDALDataset* Open( GDALOpenInfo * poOpenInfo );
};

/************************************************************************/
/* ==================================================================== */
/*                            RPFTOCSubDataset                          */
/* ==================================================================== */
/************************************************************************/

class RPFTOCSubDataset : public VRTDataset
{
  public:
    RPFTOCSubDataset(int nXSize, int nYSize) : VRTDataset(nXSize, nYSize)
    {
        /* Don't try to write a VRT file */
        SetWritable(FALSE);

        /* The driver is set to VRT in VRTDataset constructor. */
        /* We have to set it to the expected value ! */
        poDriver = (GDALDriver *) GDALGetDriverByName( "RPFTOC" );
    }
    
    ~RPFTOCSubDataset()
    {
    }
    
    static GDALDataset* CreateDataSetFromTocEntry(RPFTocEntry* entry, int isRGBA);
};
        
/************************************************************************/
/* ==================================================================== */
/*                            RPFTOCGDALDatasetCache                      */
/* ==================================================================== */
/************************************************************************/

class RPFTOCGDALDatasetCache;
static RPFTOCGDALDatasetCache* singleton = NULL;
static int refCount = 0;

typedef struct
{
    const char* fileName;
    GDALDataset* ds;
} CacheEntry;

class RPFTOCGDALDatasetCache
{
    int size;
    CacheEntry* entries;
    
    const char* cachedTileFileName;
    int cachedTileBlockXOff;
    int cachedTileBlockYOff;
    void* cachedTileImage;
    int cachedTileByteSize;
    
    RPFTOCGDALDatasetCache(int size)
    {
        this->size = size;
        entries = (CacheEntry*)CPLMalloc(size * sizeof(CacheEntry));
        memset(entries, 0, size * sizeof(CacheEntry));
        
        cachedTileFileName = NULL;
        cachedTileImage = NULL;
        cachedTileByteSize = 0;
    }

    ~RPFTOCGDALDatasetCache()
    {
        int i;
        for(i=0;i<size;i++)
        {
            if (entries[i].ds)
            {
                GDALClose(entries[i].ds);
            }
        }
        if (cachedTileImage)
            CPLFree(cachedTileImage);
        CPLFree(entries);
    }
    
    GDALDataset* _GetDataset(const char* fileName)
    {
        int i;
        for(i=0;i<size;i++)
        {
            if (entries[i].fileName == NULL)
            {
                if (i != 0)
                    memmove(&entries[1], &entries[0], sizeof(CacheEntry) * i);
                entries[0].fileName = fileName;
                entries[0].ds = (GDALDataset *) GDALOpenShared( fileName, GA_ReadOnly );
                return entries[0].ds;
            }
            /* Optimization here : we can compare the strings by address.... */
            else if (entries[i].fileName == fileName)
            {
                if (i != 0)
                {
                    GDALDataset* ds = entries[i].ds;
                    memmove(&entries[1], &entries[0], sizeof(CacheEntry) * i);
                    entries[0].ds = ds;
                    entries[0].fileName = fileName;
                }
                return entries[0].ds;
            }
        }
        GDALClose(entries[size-1].ds);
        memmove(&entries[1], &entries[0], sizeof(CacheEntry) * (size-1));
        entries[0].fileName = fileName;
        entries[0].ds = (GDALDataset *) GDALOpenShared( fileName, GA_ReadOnly );
        return entries[0].ds;
    }
    
    void _AddTileToCache(const char* fileName, int nBlockXOff, int nBlockYOff,
                        const void* pImage, int byteSize )
    {
        cachedTileFileName = fileName;
        cachedTileBlockXOff = nBlockXOff;
        cachedTileBlockYOff = nBlockYOff;
        if (byteSize != cachedTileByteSize)
        {
            if (cachedTileImage)
                CPLFree(cachedTileImage);
            cachedTileImage = CPLMalloc(byteSize);
        }
        memcpy(cachedTileImage, pImage, byteSize);
        cachedTileByteSize = byteSize;
    }
    
    void* _GetTileFromCache(const char* fileName, int nBlockXOff, int nBlockYOff)
    {
        if (fileName == cachedTileFileName && 
            cachedTileBlockXOff == nBlockXOff &&
            cachedTileBlockYOff == nBlockYOff)
        {
            return cachedTileImage;
        }
        else
        {
            return NULL;
        }
    }
        
    public:
        static void Ref()
        {
            if (singleton == NULL)
                singleton = new RPFTOCGDALDatasetCache(100);
            refCount++;
        }

        static void Unref()
        {
            refCount--;
            if (refCount == 0)
            {
                delete singleton;
                singleton = NULL;
            }
        }

        static GDALDataset* GetDataset(const char* fileName)
        {
            if (! singleton) return NULL;
            return singleton->_GetDataset(fileName);
        }

        static void AddTileToCache(const char* fileName, int nBlockXOff, int nBlockYOff,
                                   const void* pImage, int byteSize )
        {
            if (! singleton) return;
            singleton->_AddTileToCache(fileName, nBlockXOff, nBlockYOff, pImage, byteSize);
        }
        
        static void* GetTileFromCache(const char* fileName, int nBlockXOff, int nBlockYOff)
        {
            if (! singleton) return NULL;
            return singleton->_GetTileFromCache(fileName, nBlockXOff, nBlockYOff);
        }
};


/************************************************************************/
/* ==================================================================== */
/*                        RPFTOCProxyRasterDataSet                       */
/* ==================================================================== */
/************************************************************************/

class RPFTOCProxyRasterDataSet : public GDALDataset
{
    char* fileName;
    
    /* The following parameters are only for sanity checking */
    int checkDone;
    int checkOK;
    const char* projectionRef;
    double nwLong, nwLat;
    GDALColorTable* colorTableRef;
    
    public:
        RPFTOCProxyRasterDataSet(const char* fileName,
                                int nRasterXSize, int nRasterYSize,
                                int nBlockXSize, int nBlockYSize,
                                const char* projectionRef, double nwLong, double nwLat,
                                int nBands);

        ~RPFTOCProxyRasterDataSet()
        {
            if (fileName)
                CPLFree(fileName);
        }
        
        void SetReferenceColorTable(GDALColorTable* colorTableRef) { this->colorTableRef = colorTableRef;}
        
        const GDALColorTable* GetReferenceColorTable() { return colorTableRef; }

        const char* GetFileName() { return fileName; }
        
        int SanityCheckOK(GDALDataset* sourceDS);
};

/************************************************************************/
/* ==================================================================== */
/*                     RPFTOCProxyRasterBandRGBA                        */
/* ==================================================================== */
/************************************************************************/

class RPFTOCProxyRasterBandRGBA : public GDALPamRasterBand
{
    int initDone;
    unsigned char colorTable[256];
    int blockByteSize;

    private:
        void Expand(void* pImage, const void* srcImage);

    public:
        RPFTOCProxyRasterBandRGBA(int nRasterXSize, int nRasterYSize,
                                  int nBlockXSize, int nBlockYSize)
        {
            RPFTOCGDALDatasetCache::Ref();
            this->eAccess = GA_ReadOnly;
            this->eDataType = GDT_Byte;
            this->nBlockXSize = nBlockXSize;
            this->nBlockYSize = nBlockYSize;
            this->nRasterXSize = nRasterXSize;
            this->nRasterYSize = nRasterYSize;
            blockByteSize = nBlockXSize * nBlockYSize;
            initDone = FALSE;
        }

        ~RPFTOCProxyRasterBandRGBA()
        {
            RPFTOCGDALDatasetCache::Unref();
        }

    protected:
        virtual CPLErr IReadBlock( int nBlockXOff, int nBlockYOff,
                                   void * pImage );
};

/************************************************************************/
/*                    Expand()                                          */
/************************************************************************/

/* Expand the  array or indexed colors to an array of their corresponding R,G,B or A component */
void  RPFTOCProxyRasterBandRGBA::Expand(void* pImage, const void* srcImage)
{
    int i;
    if ((blockByteSize & (~3)) != 0)
    {
        for(i=0;i<blockByteSize;i++)
        {
            ((unsigned char*)pImage)[i] = colorTable[((unsigned char*)srcImage)[i]];
        }
    }
    else
    {
        int nIter = blockByteSize/4;
        for(i=0;i<nIter;i++)
        {
            unsigned int four_pixels = ((unsigned int*)srcImage)[i];
            ((unsigned int*)pImage)[i] =
                    (colorTable[four_pixels >> 24] << 24) |
                    (colorTable[(four_pixels >> 16) & 0xFF] << 16) |
                    (colorTable[(four_pixels >> 8) & 0xFF] << 8) |
                    colorTable[four_pixels & 0xFF];
        }
    }

}

/************************************************************************/
/*                    IReadBlock()                                      */
/************************************************************************/

CPLErr RPFTOCProxyRasterBandRGBA::IReadBlock( int nBlockXOff, int nBlockYOff,
                                         void * pImage )
{
    CPLErr ret;
    const char* fileName = ((RPFTOCProxyRasterDataSet*)poDS)->GetFileName();
    GDALDataset* ds = RPFTOCGDALDatasetCache::GetDataset(fileName);
    if (ds)
    {
        if (((RPFTOCProxyRasterDataSet*)poDS)->SanityCheckOK(ds) == FALSE)
            return CE_Failure;

        GDALRasterBand* srcBand = ds->GetRasterBand(1);
        if (initDone == FALSE)
        {
            GDALColorTable* srcColorTable = srcBand->GetColorTable();
            int i;
            int bHasNoDataValue;
            int noDataValue = (int)srcBand->GetNoDataValue(&bHasNoDataValue);
            int nEntries = srcColorTable->GetColorEntryCount();
            for(i=0;i<nEntries;i++)
            {
                const GDALColorEntry* entry = srcColorTable->GetColorEntry(i);
                if (nBand == 1)
                    colorTable[i] = (unsigned char)entry->c1;
                else if (nBand == 2)
                    colorTable[i] = (unsigned char)entry->c2;
                else if (nBand == 3)
                    colorTable[i] = (unsigned char)entry->c3;
                else
                {
                    colorTable[i] = (bHasNoDataValue && i == noDataValue) ? 0 : (unsigned char)entry->c4;
                }
            }
            if (bHasNoDataValue && nEntries == noDataValue)
                colorTable[nEntries] = 0;
            initDone = TRUE;
        }

        /* We use a 1-tile cache as the same source tile will be consecutively asked for */
        /* computing the R tile, the G tile, the B tile and the A tile */
        void* cachedImage =
                RPFTOCGDALDatasetCache::GetTileFromCache(fileName, nBlockXOff, nBlockYOff);
        if (cachedImage == NULL)
        {
            ret = srcBand->ReadBlock(nBlockXOff, nBlockYOff, pImage);
            if (ret == CE_None)
            {
                RPFTOCGDALDatasetCache::AddTileToCache
                        (fileName, nBlockXOff, nBlockYOff, pImage, blockByteSize);
                Expand(pImage, pImage);
            }
        }
        else
        {
            Expand(pImage, cachedImage);
            ret = CE_None;
        }

    }
    else
        ret = CE_Failure;
    return ret;
}

/************************************************************************/
/* ==================================================================== */
/*                 RPFTOCProxyRasterBandPalette                         */
/* ==================================================================== */
/************************************************************************/

class RPFTOCProxyRasterBandPalette : public GDALPamRasterBand
{
    int initDone;
    int blockByteSize;
    int samePalette;
    unsigned char remapLUT[256];

    public:
        RPFTOCProxyRasterBandPalette(int nRasterXSize, int nRasterYSize,
                                     int nBlockXSize, int nBlockYSize)
        {
            RPFTOCGDALDatasetCache::Ref();
            this->eAccess = GA_ReadOnly;
            this->eDataType = GDT_Byte;
            this->nBlockXSize = nBlockXSize;
            this->nBlockYSize = nBlockYSize;
            this->nRasterXSize = nRasterXSize;
            this->nRasterYSize = nRasterYSize;
            blockByteSize = nBlockXSize * nBlockYSize;
            initDone = FALSE;
        }

        ~RPFTOCProxyRasterBandPalette()
        {
            RPFTOCGDALDatasetCache::Unref();
        }

    protected:
        virtual CPLErr IReadBlock( int nBlockXOff, int nBlockYOff,
                                   void * pImage );
};

/************************************************************************/
/*                    IReadBlock()                                      */
/************************************************************************/

CPLErr RPFTOCProxyRasterBandPalette::IReadBlock( int nBlockXOff, int nBlockYOff,
                                                 void * pImage )
{
    CPLErr ret;
    const char* fileName = ((RPFTOCProxyRasterDataSet*)poDS)->GetFileName();
    GDALDataset* ds = RPFTOCGDALDatasetCache::GetDataset(fileName);
    if (ds)
    {
        if (((RPFTOCProxyRasterDataSet*)poDS)->SanityCheckOK(ds) == FALSE)
            return CE_Failure;

        GDALRasterBand* srcBand = ds->GetRasterBand(1);
        if (initDone == FALSE)
        {
            GDALColorTable* srcColorTable = srcBand->GetColorTable();
            int i,j;
            int bHasNoDataValue;
            int noDataValue = (int)srcBand->GetNoDataValue(&bHasNoDataValue);
            int nEntries = srcColorTable->GetColorEntryCount();
            const GDALColorTable* colorTableRef = ((RPFTOCProxyRasterDataSet*)poDS)->GetReferenceColorTable();
            samePalette = TRUE;
            for(i=0;i<nEntries;i++)
            {
                const GDALColorEntry* entry = srcColorTable->GetColorEntry(i);
                const GDALColorEntry* entryRef = colorTableRef->GetColorEntry(i);
                if (entry->c1 != entryRef->c1 ||
                    entry->c2 != entryRef->c2 ||
                    entry->c3 != entryRef->c3)
                {
                    samePalette = FALSE;
                }
            }
            if (samePalette == FALSE)
            {
                /* Trying to remap the product palette on the subdataset palette */
                int nRefEntries = srcColorTable->GetColorEntryCount();
                int first_msg = 1;
                for(i=0;i<nEntries;i++)
                {
                    const GDALColorEntry* entry = srcColorTable->GetColorEntry(i);
                    for(j=0;j<nRefEntries;j++)
                    {
                        const GDALColorEntry* entryRef = colorTableRef->GetColorEntry(j);
                        if (entry->c1 == entryRef->c1 &&
                            entry->c2 == entryRef->c2 &&
                            entry->c3 == entryRef->c3)
                        {
                            remapLUT[i] = j;
                            break;
                        }
                    }
                    if (j == nEntries)
                    {
                        /* No exact match. Looking for closest color now... */
                        if (first_msg)
                        {
                            CPLError( CE_Failure, CPLE_AppDefined,
                                    "Palette for %s is different from reference palette. "
                                     "Coudln't remap color %d. Trying to find closest match.\n", fileName, i);
                            first_msg = 0;
                        }
                        int best_j = 0;
                        int best_distance = 0;
                        for(j=0;j<nRefEntries;j++)
                        {
                            const GDALColorEntry* entryRef = colorTableRef->GetColorEntry(j);
                            int distance = (entry->c1 - entryRef->c1) * (entry->c1 - entryRef->c1) +
                                           (entry->c2 - entryRef->c2) * (entry->c2 - entryRef->c2) +
                                           (entry->c3 - entryRef->c3) * (entry->c3 - entryRef->c3);
                            if (j == 0 || distance < best_distance)
                            {
                                best_j = j;
                                best_distance = distance;
                            }
                        }
                        remapLUT[i] = best_j;
                    }
                }
                if (bHasNoDataValue)
                    remapLUT[noDataValue] = noDataValue;
            }
            initDone = TRUE;
        }


        ret = srcBand->ReadBlock(nBlockXOff, nBlockYOff, pImage);
        if (samePalette == FALSE)
        {
            unsigned char* data = (unsigned char*)pImage;
            int i;
            for(i=0;i<blockByteSize;i++)
            {
                data[i] = remapLUT[data[i]];
            }
        }

    }
    else
        ret = CE_Failure;
    return ret;
}

/************************************************************************/
/*                    RPFTOCProxyRasterDataSet()                         */
/************************************************************************/

RPFTOCProxyRasterDataSet::RPFTOCProxyRasterDataSet
        (const char* fileName,
         int nRasterXSize, int nRasterYSize,
         int nBlockXSize, int nBlockYSize,
         const char* projectionRef, double nwLong, double nwLat,
         int nBands)
{
    int i;
    this->fileName = CPLStrdup(fileName);
    this->nRasterXSize = nRasterXSize;
    this->nRasterYSize = nRasterYSize;
    this->eAccess = GA_ReadOnly;
    this->projectionRef = projectionRef;
    this->nwLong = nwLong;
    this->nwLat = nwLat;
    colorTableRef = NULL;
    bShared = TRUE;
    checkDone = FALSE;
    checkOK = FALSE;
    if (nBands == 4)
    {
        for(i=0;i<4;i++)
        {
            SetBand(i+1, new RPFTOCProxyRasterBandRGBA(nRasterXSize, nRasterYSize, nBlockXSize, nBlockYSize));
        }
    }
    else
        SetBand(1, new RPFTOCProxyRasterBandPalette(nRasterXSize, nRasterYSize, nBlockXSize, nBlockYSize));
}

/************************************************************************/
/*                    SanityCheckOK()                                   */
/************************************************************************/

#define WARN_CHECK_DS(x) do { if (!(x)) { CPLError(CE_Warning, CPLE_AppDefined, "For %s, assert '" #x "' failed", fileName); checkOK = FALSE; } } while(0)

int RPFTOCProxyRasterDataSet::SanityCheckOK(GDALDataset* sourceDS)
{
    int src_nBlockXSize, src_nBlockYSize;
    int nBlockXSize, nBlockYSize;
    double adfGeoTransform[6];
    if (checkDone)
        return checkOK;
    
    checkOK = TRUE;
    checkDone = TRUE;
    
    sourceDS->GetGeoTransform(adfGeoTransform);
    WARN_CHECK_DS(fabs(adfGeoTransform[GEOTRSFRM_TOPLEFT_X] - nwLong) < 1e-10);
    WARN_CHECK_DS(fabs(adfGeoTransform[GEOTRSFRM_TOPLEFT_Y] - nwLat) < 1e-10);
    WARN_CHECK_DS(adfGeoTransform[GEOTRSFRM_ROTATION_PARAM1] == 0 &&
                  adfGeoTransform[GEOTRSFRM_ROTATION_PARAM2] == 0); /* No rotation */
    WARN_CHECK_DS(sourceDS->GetRasterCount() == 1); /* Just 1 band */
    WARN_CHECK_DS(sourceDS->GetRasterXSize() == nRasterXSize);
    WARN_CHECK_DS(sourceDS->GetRasterYSize() == nRasterYSize);
    WARN_CHECK_DS(EQUAL(sourceDS->GetProjectionRef(), projectionRef));
    sourceDS->GetRasterBand(1)->GetBlockSize(&src_nBlockXSize, &src_nBlockYSize);
    GetRasterBand(1)->GetBlockSize(&nBlockXSize, &nBlockYSize);
    WARN_CHECK_DS(src_nBlockXSize == nBlockXSize);
    WARN_CHECK_DS(src_nBlockYSize == nBlockYSize);
    WARN_CHECK_DS(sourceDS->GetRasterBand(1)->GetColorInterpretation() == GCI_PaletteIndex);
    WARN_CHECK_DS(sourceDS->GetRasterBand(1)->GetRasterDataType() == GDT_Byte);

    return checkOK;
}

/************************************************************************/
/*                           MakeTOCEntryName()                         */
/************************************************************************/

static const char* MakeTOCEntryName(RPFTocEntry* tocEntry )
{
    char* str;
    if (tocEntry->seriesAbbreviation)
        str = (char*)CPLSPrintf( "%s_%s_%s_%s_%d", tocEntry->type, tocEntry->seriesAbbreviation, tocEntry->scale, tocEntry->zone, tocEntry->boundaryId );
    else
        str = (char*)CPLSPrintf( "%s_%s_%s_%d", tocEntry->type, tocEntry->scale, tocEntry->zone, tocEntry->boundaryId );
    char* c = str;
    while(*c)
    {
        if (*c == ':')
            *c = '_';
        c++;
    }
    return str;
}


/************************************************************************/
/*                           AddSubDataset()                            */
/************************************************************************/

void RPFTOCDataset::AddSubDataset( const char* pszFilename,  RPFTocEntry* tocEntry )

{
    char	szName[80];
    int		nCount = CSLCount(papszSubDatasets ) / 2;

    sprintf( szName, "SUBDATASET_%d_NAME", nCount+1 );
    papszSubDatasets = 
        CSLSetNameValue( papszSubDatasets, szName, 
              CPLSPrintf( "NITF_TOC_ENTRY:%s:%s", MakeTOCEntryName(tocEntry), pszFilename ) );

    sprintf( szName, "SUBDATASET_%d_DESC", nCount+1 );
    if (tocEntry->seriesName && tocEntry->seriesAbbreviation)
        papszSubDatasets = 
        CSLSetNameValue( papszSubDatasets, szName,
               CPLSPrintf( "%s:%s:%s:%s:%s:%d", tocEntry->type, tocEntry->seriesAbbreviation, tocEntry->seriesName, tocEntry->scale, tocEntry->zone, tocEntry->boundaryId ));
    else
        papszSubDatasets = 
            CSLSetNameValue( papszSubDatasets, szName,
                CPLSPrintf( "%s:%s:%s:%d", tocEntry->type, tocEntry->scale, tocEntry->zone, tocEntry->boundaryId ));
}

/************************************************************************/
/*                            GetMetadata()                             */
/************************************************************************/

char **RPFTOCDataset::GetMetadata( const char *pszDomain )

{
    if( pszDomain != NULL && EQUAL(pszDomain,"SUBDATASETS") )
        return papszSubDatasets;

    return GDALPamDataset::GetMetadata( pszDomain );
}

/************************************************************************/
/*                  NITFCreateVRTDataSetFromTocEntry()                  */
/************************************************************************/


#define ASSERT_CREATE_VRT(x) do { if (!(x)) { CPLError(CE_Failure, CPLE_AppDefined, "For %s, assert '" #x "' failed", entry->frameEntries[i].fullFilePath); if (poSrcDS) GDALClose(poSrcDS); CPLFree(projectionRef); return NULL;} } while(0)

/* Builds a RPFTOCSubDataset from the set of files of the toc entry */
GDALDataset* RPFTOCSubDataset::CreateDataSetFromTocEntry(RPFTocEntry* entry, int isRGBA)
{
    int i, j;
    GDALDriver *poDriver;
    GDALDataset *poVirtualDS;
    int sizeX, sizeY;
    int nBlockXSize = 0, nBlockYSize = 0;
    double geoTransf[6];
    char* projectionRef = NULL;
    int N;
    int index = 0;

    poDriver = GetGDALDriverManager()->GetDriverByName("VRT");
    if( poDriver == NULL )
        return NULL;

    N = entry->nVertFrames * entry->nHorizFrames;

    /* This may not be reliable. See below */
    sizeX = (int)((entry->seLong - entry->nwLong) / (entry->nHorizFrames * entry->horizInterval) + 0.5);
    sizeY = (int)((entry->nwLat - entry->seLat) / (entry->nVertFrames * entry->vertInterval) + 0.5);

    for(i=0;i<N; i++)
    {
        if (!entry->frameEntries[i].fileExists)
            continue;

        if (index == 0)
        {
            int ds_sizeX, ds_sizeY;
            /* Open the first available file to get its geotransform, projection ref and block size */
            /* Do a few sanity checks too */
            /* Ideally we should make these sanity checks now on ALL files, but it would be too slow */
            /* for large datasets. So these sanity checks will be done at the time we really need */
            /* to access the file (see SanityCheckOK metho) */
            GDALDataset *poSrcDS = (GDALDataset *) GDALOpenShared( entry->frameEntries[i].fullFilePath, GA_ReadOnly );
            ASSERT_CREATE_VRT(poSrcDS);
            poSrcDS->GetGeoTransform(geoTransf);
            projectionRef = CPLStrdup(poSrcDS->GetProjectionRef());
            ASSERT_CREATE_VRT(geoTransf[GEOTRSFRM_ROTATION_PARAM1] == 0 &&
                              geoTransf[GEOTRSFRM_ROTATION_PARAM2] == 0); /* No rotation */
            ASSERT_CREATE_VRT(poSrcDS->GetRasterCount() == 1); /* Just 1 band */
            
            /* Tolerance of 1%... This is necessary for CADRG_L22/RPF/A.TOC for example */
            ASSERT_CREATE_VRT((entry->horizInterval - geoTransf[GEOTRSFRM_WE_RES]) /
                                entry->horizInterval < 0.01); /* X interval same as in TOC */
            ASSERT_CREATE_VRT((entry->vertInterval - (-geoTransf[GEOTRSFRM_NS_RES])) /
                                entry->horizInterval < 0.01); /* Y interval same as in TOC */

            ds_sizeX = poSrcDS->GetRasterXSize();
            ds_sizeY = poSrcDS->GetRasterYSize();
            /* In the case the east longitude is 180, there's a great chance that it is in fact */
            /* truncated in the A.TOC. Thus, the only reliable way to find out the tile width, is to */
            /* read it from the tile dataset itself... */
            /* This is the case for the GNCJNCN dataset that has world coverage */
            if (entry->seLong == 180.00)
                sizeX = ds_sizeX;
            else
                ASSERT_CREATE_VRT(sizeX == ds_sizeX);
            ASSERT_CREATE_VRT(sizeY == ds_sizeY);
            poSrcDS->GetRasterBand(1)->GetBlockSize(&nBlockXSize, &nBlockYSize);
            ASSERT_CREATE_VRT(poSrcDS->GetRasterBand(1)->GetColorInterpretation() == GCI_PaletteIndex);
            ASSERT_CREATE_VRT(poSrcDS->GetRasterBand(1)->GetRasterDataType() == GDT_Byte);
            GDALClose(poSrcDS);
        }

        index++;
    }

    if (index == 0)
        return NULL;

    /* ------------------------------------ */
    /* Create the VRT with the overall size */
    /* ------------------------------------ */
    poVirtualDS = new RPFTOCSubDataset( sizeX * entry->nHorizFrames,
                                        sizeY * entry->nVertFrames);

    if( poVirtualDS == NULL )
    {
        return NULL;
    }

    poVirtualDS->SetProjection(projectionRef);

    geoTransf[GEOTRSFRM_TOPLEFT_X] = entry->nwLong;
    geoTransf[GEOTRSFRM_TOPLEFT_Y] = entry->nwLat;
    poVirtualDS->SetGeoTransform(geoTransf);
    
    int nBands;
    
    /* In most cases, all the files inside a TOC entry share the same */
    /* palette and we could use it for the VRT. */
    /* In other cases like for CADRG801_France_250K (TOC entry CADRG_250K_2_2), */
    /* the file for Corsica and the file for Sardegna do not share the same palette */
    /* however they contain the same RGB triplets and are just ordered differently */
    /* So we can use the same palette */
    /* In the unlikely event where palettes would be incompatible, we can use the RGBA */
    /* option through the config option RPFTOC_FORCE_RGBA */
    if (isRGBA == FALSE)
    {
        poVirtualDS->AddBand(GDT_Byte, NULL);
        GDALRasterBand *poBand = poVirtualDS->GetRasterBand( 1 );
        poBand->SetColorInterpretation(GCI_PaletteIndex);
        nBands = 1;
        
        for(i=0;i<N; i++)
        {
            if (!entry->frameEntries[i].fileExists)
                continue;
            
            GDALDataset *poSrcDS = (GDALDataset *) GDALOpenShared( entry->frameEntries[i].fullFilePath, GA_ReadOnly );
            poBand->SetColorTable(poSrcDS->GetRasterBand(1)->GetColorTable());
            
            int bHasNoDataValue;
            double noDataValue = poSrcDS->GetRasterBand(1)->GetNoDataValue(&bHasNoDataValue);
            if (bHasNoDataValue)
                poBand->SetNoDataValue(noDataValue);
            GDALClose(poSrcDS);
            break;
        }
    }
    else
    {
        for (i=0;i<4;i++)
        {
            poVirtualDS->AddBand(GDT_Byte, NULL);
            GDALRasterBand *poBand = poVirtualDS->GetRasterBand( i + 1 );
            poBand->SetColorInterpretation((GDALColorInterp)(GCI_RedBand+i));
        }
        nBands = 4;
    }

    CPLFree(projectionRef);
    projectionRef = NULL;

    for(i=0;i<N; i++)
    {
        if (! entry->frameEntries[i].fileExists)
            continue;

        /* We create proxy datasets and raster bands */
        /* Using real datasets and raster bands is possible in theory */
        /* However for large datasets, a TOC entry can include several hundreds of files */
        /* and we finally reach the limit of maximum file descriptors open at the same time ! */
        /* So the idea is to warp the datasets into a proxy and open the underlying dataset only when it is */
        /* needed (IRasterIO operation). To improve a bit efficiency, we have a cache of opened */
        /* underlying datasets */
        RPFTOCProxyRasterDataSet* ds = new RPFTOCProxyRasterDataSet(
                entry->frameEntries[i].fullFilePath,
                sizeX, sizeY,
                nBlockXSize, nBlockYSize,
                poVirtualDS->GetProjectionRef(),
                entry->nwLong + entry->frameEntries[i].frameCol * entry->horizInterval * sizeX, 
                entry->nwLat - entry->frameEntries[i].frameRow * entry->vertInterval * sizeY,
                nBands);
        if (nBands == 1)
        {
            ds->SetReferenceColorTable(poVirtualDS->GetRasterBand( 1 )->GetColorTable());
        }

        for(j=0;j<nBands;j++)
        {
            VRTSourcedRasterBand *poBand = (VRTSourcedRasterBand*)poVirtualDS->GetRasterBand( j + 1 );
            /* Place the raster band at the right position in the VRT */
            poBand->AddSimpleSource(ds->GetRasterBand(j + 1),
                                    0, 0, sizeX, sizeY,
                                    entry->frameEntries[i].frameCol * sizeX, 
                                    entry->frameEntries[i].frameRow * sizeY,
                                    sizeX, sizeY);
        }

        /* The RPFTOCProxyRasterDataSet will be destroyed when its last raster band will be */
        /* destroyed */
        ds->Dereference();
    }

    return poVirtualDS;
}

/* Check whether the file is a TOC file without NITF header */
int RPFTOCDataset::IsNonNITFFileTOC(GDALOpenInfo * poOpenInfo, const char* pszFilename )
{
    char pattern[] = { 0, 0, '0', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'A', '.', 'T', 'O', 'C', 0 };
    if (poOpenInfo)
    {
        if (poOpenInfo->nHeaderBytes < 48 )
            return FALSE;
        return memcmp(pattern, poOpenInfo->pabyHeader, 16) == 0;
    }
    else
    {
        char buffer[48];
        FILE* fp = NULL;
        fp = VSIFOpenL( pszFilename, "rb" );
        if( fp == NULL )
        {
            return FALSE;
        }

        int ret = (VSIFReadL(buffer, 1, 48, fp) == 48) &&
                   memcmp(pattern, buffer, 16) == 0;
        VSIFCloseL(fp);
        return ret;
    }
}

/* Check whether this NITF file is a TOC file */
int RPFTOCDataset::IsNITFFileTOC(NITFFile *psFile)
{
    const char* fileTitle = CSLFetchNameValue(psFile->papszMetadata, "NITF_FTITLE");
    while(fileTitle && *fileTitle)
    {
        if (EQUAL(fileTitle, "A.TOC"))
        {
            return TRUE;
        }
        fileTitle++;
    }
    return FALSE;
}

/* Create a dataset from a TOC file */
/* If psFile == NULL, the TOC file has no NITF header */
/* If entryName != NULL, the dataset will be made just of the entry of the TOC file */
GDALDataset* RPFTOCDataset::OpenFileTOC(NITFFile *psFile,
                                        const char* pszFilename,
                                        const char* entryName)
{
    char buffer[48];
    FILE* fp = NULL;
    if (psFile == NULL)
    {
        fp = VSIFOpenL( pszFilename, "rb" );

        if( fp == NULL )
        {
            CPLError( CE_Failure, CPLE_OpenFailed, 
                    "Failed to open file %s.", 
                    pszFilename );
            return NULL;
        }
        VSIFReadL(buffer, 1, 48, fp);
    }
    int isRGBA = CSLTestBoolean(CPLGetConfigOption("RPFTOC_FORCE_RGBA", "NO"));
    RPFToc* toc = (psFile) ? RPFTOCRead( pszFilename, psFile ) :
                              RPFTOCReadFromBuffer( pszFilename, fp, buffer);

    if (entryName != NULL)
    {
        if (toc)
        {
            int i, j;
            for(i=0;i<toc->nEntries;i++)
            {
                if (EQUAL(entryName, MakeTOCEntryName(&toc->entries[i])))
                {
                    GDALDataset* ds = RPFTOCSubDataset::CreateDataSetFromTocEntry(&toc->entries[i], isRGBA);
                    if (ds)
                    {
                        int iFile = 0;
                        if (psFile)
                            ds->SetMetadata(psFile->papszMetadata);
                        for (j=0; j< (int)toc->entries[i].nVertFrames * (int)toc->entries[i].nHorizFrames; j++)
                        {
                            if ( toc->entries[i].frameEntries[j].fileExists)
                            {
                                ds->SetMetadataItem(CPLSPrintf("FILENAME_%d", iFile),
                                                    toc->entries[i].frameEntries[j].fullFilePath);
                                iFile++;
                            }
                        }
                        ds->SetMetadataItem("NITF_SCALE", toc->entries[i].scale);
                        ds->SetMetadataItem("NITF_SERIES_ABBREVIATION",
                                            (toc->entries[i].seriesAbbreviation) ? toc->entries[i].seriesAbbreviation : "Unknown");
                        ds->SetMetadataItem("NITF_SERIES_NAME",
                                            (toc->entries[i].seriesName) ? toc->entries[i].seriesName : "Unknown");
                    }
                    if (fp) VSIFCloseL(fp);
                    RPFTOCFree(toc);
                    return ds;
                }
            }
            CPLError( CE_Failure, CPLE_AppDefined, 
                        "The entry %s does not exist in file %s.", entryName, pszFilename );
        }
        if (fp) VSIFCloseL(fp);
        return NULL;
    }

    if (toc)
    {
        RPFTOCDataset* ds = new RPFTOCDataset();
        int i;
        int ok = FALSE;
        char* projectionRef = NULL;
        double nwLong = 0, nwLat = 0, seLong = 0, seLat = 0;
        double adfGeoTransform[6];
        for(i=0;i<toc->nEntries;i++)
        {
            if (!toc->entries[i].isOverview)
            {
                GDALDataset* tmpDS = RPFTOCSubDataset::CreateDataSetFromTocEntry(&toc->entries[i], isRGBA);
                if (tmpDS)
                {
                    tmpDS->GetGeoTransform(adfGeoTransform);
                    if (projectionRef == NULL)
                    {
                        ok = TRUE;
                        projectionRef = CPLStrdup(tmpDS->GetProjectionRef());
                        nwLong = adfGeoTransform[GEOTRSFRM_TOPLEFT_X];
                        nwLat = adfGeoTransform[GEOTRSFRM_TOPLEFT_Y];
                        seLong = nwLong + adfGeoTransform[GEOTRSFRM_WE_RES] * tmpDS->GetRasterXSize();
                        seLat = nwLat + adfGeoTransform[GEOTRSFRM_NS_RES] * tmpDS->GetRasterYSize();
                    }
                    else if (ok)
                    {
                        double _nwLong = adfGeoTransform[GEOTRSFRM_TOPLEFT_X];
                        double _nwLat = adfGeoTransform[GEOTRSFRM_TOPLEFT_Y];
                        double _seLong = _nwLong + adfGeoTransform[GEOTRSFRM_WE_RES] * tmpDS->GetRasterXSize();
                        double _seLat = _nwLat + adfGeoTransform[GEOTRSFRM_NS_RES] * tmpDS->GetRasterYSize();
                        if (! EQUAL(projectionRef, tmpDS->GetProjectionRef()) )
                            ok = FALSE;
                        if (_nwLong < nwLong)
                            nwLong = _nwLong;
                        if (_nwLat > nwLat)
                            nwLat = _nwLat;
                        if (_seLong > seLong)
                            seLong = _seLong;
                        if (_seLat < seLat)
                            seLat = _seLat;
                    }
                    GDALClose(tmpDS);
                    ds->AddSubDataset(pszFilename, &toc->entries[i]);
                }
            }
        }
        if (ok)
        {
            adfGeoTransform[GEOTRSFRM_TOPLEFT_X] = nwLong;
            adfGeoTransform[GEOTRSFRM_TOPLEFT_Y] = nwLat;
            ds->SetSize((int)(0.5 + (seLong - nwLong) / adfGeoTransform[GEOTRSFRM_WE_RES]), 
                        (int)(0.5 + (seLat - nwLat) / adfGeoTransform[GEOTRSFRM_NS_RES]));
            ds->SetGeoTransform(adfGeoTransform);
            ds->SetProjection(projectionRef);
        }
        CPLFree(projectionRef);
        RPFTOCFree(toc);
        if (fp) VSIFCloseL(fp);
        return ds;
    }
    else
    {
        if (fp) VSIFCloseL(fp);
        return NULL;
    }
}

/************************************************************************/
/*                              Identify()                              */
/************************************************************************/

int RPFTOCDataset::Identify( GDALOpenInfo * poOpenInfo )

{
    const char *pszFilename = poOpenInfo->pszFilename;

/* -------------------------------------------------------------------- */
/*      Is this a sub-dataset selector? If so, it is obviously RPFTOC.        */
/* -------------------------------------------------------------------- */

    if( EQUALN(pszFilename, "NITF_TOC_ENTRY:",strlen("NITF_TOC_ENTRY:")))
        return TRUE;

/* -------------------------------------------------------------------- */
/*	First we check to see if the file has the expected header	*/
/*	bytes.								*/    
/* -------------------------------------------------------------------- */
    if( poOpenInfo->nHeaderBytes < 48 )
        return FALSE;
    
    if ( IsNonNITFFileTOC( poOpenInfo, pszFilename) )
        return TRUE;
        
    if( !EQUALN((char *) poOpenInfo->pabyHeader,"NITF",4) 
        && !EQUALN((char *) poOpenInfo->pabyHeader,"NSIF",4)
        && !EQUALN((char *) poOpenInfo->pabyHeader,"NITF",4) )
        return FALSE;
    
    int i;
    /* If it's a NITF A.TOC file, it must contain A.TOC in it's header */
    for(i=0;i<(int)poOpenInfo->nHeaderBytes-(int)strlen("A.TOC");i++)
    {
        if (EQUALN((const char*)poOpenInfo->pabyHeader + i, "A.TOC", strlen("A.TOC")))
            return TRUE;
    }

    return FALSE;
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *RPFTOCDataset::Open( GDALOpenInfo * poOpenInfo )

{
    const char *pszFilename = poOpenInfo->pszFilename;
    char* entryName = NULL;

    if( !Identify( poOpenInfo ) )
        return NULL;

    if( EQUALN(pszFilename, "NITF_TOC_ENTRY:",strlen("NITF_TOC_ENTRY:")))
    {
        pszFilename += strlen("NITF_TOC_ENTRY:");
        entryName = CPLStrdup(pszFilename);
        char* c = entryName;
        while( *c != '\0' && *c != ':' )
            c++;
        if( *c != ':' )
        {
            CPLFree(entryName);
            return NULL;
        }
        *c = 0;

        while( *pszFilename != '\0' && *pszFilename != ':' )
            pszFilename++;
        pszFilename++;
    }
    
    if (IsNonNITFFileTOC((entryName != NULL) ? NULL : poOpenInfo, pszFilename))
    {
        GDALDataset* poDS = OpenFileTOC(NULL, pszFilename, entryName);
        CPLFree(entryName);

        return poDS;
    }

/* -------------------------------------------------------------------- */
/*      Open the file with library.                                     */
/* -------------------------------------------------------------------- */
    NITFFile *psFile;

    psFile = NITFOpen( pszFilename, poOpenInfo->eAccess == GA_Update );
    if( psFile == NULL )
    {
        CPLFree(entryName);
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Check if it is a TOC file .                                     */
/* -------------------------------------------------------------------- */
    if (IsNITFFileTOC(psFile))
    {
        GDALDataset* poDS = OpenFileTOC(psFile, pszFilename, entryName);
        NITFClose( psFile );
        CPLFree(entryName);

        return poDS;
    }
    else
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                          "File %s is not a TOC file.", pszFilename );
        NITFClose( psFile );
        CPLFree(entryName);
        return NULL;
    }

}

/************************************************************************/
/*                          GDALRegister_RPFTOC()                         */
/************************************************************************/

void GDALRegister_RPFTOC()

{
    GDALDriver	*poDriver;

    if( GDALGetDriverByName( "RPFTOC" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "RPFTOC" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "Raster Product Format TOC format" );
        
        poDriver->pfnIdentify = RPFTOCDataset::Identify;
        poDriver->pfnOpen = RPFTOCDataset::Open;

        /* TODO */
        /* poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, "frmt_rpftoc.html" ); */
        poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "toc" );

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}
