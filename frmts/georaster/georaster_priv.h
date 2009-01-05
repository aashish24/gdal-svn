/******************************************************************************
 * $Id: $
 *
 * Name:     georaster_priv.h
 * Project:  Oracle Spatial GeoRaster Driver
 * Purpose:  Define C++/Private declarations
 * Author:   Ivan Lucena [ivan.lucena@pmldnet.com]
 *
 ******************************************************************************
 * Copyright (c) 2008, Ivan Lucena
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files ( the "Software" ),
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
 *****************************************************************************/

#include "gdal.h"
#include "gdal_priv.h"
#include "gdal_rat.h"
#include "oci_wrapper.h"
#include "ogr_spatialref.h"
#include "cpl_minixml.h"

class GeoRasterDriver;
class GeoRasterDataset;
class GeoRasterRasterBand;
class GeoRasterWrapper;

//  ---------------------------------------------------------------------------
//  Calculate the row index where a block is stored
//  ---------------------------------------------------------------------------

#define CALCULATEBLOCK( bn, xo, yo, bb, tc, tr ) \
    ( ( (int) ceil( (double) ( ( bn - 1 ) / bb ) ) * tc * tr ) + ( yo * tc ) + xo )

//  ---------------------------------------------------------------------------
//  Link between bands+pyramid levels and modified <shrinked> georaster object
//  ---------------------------------------------------------------------------

struct OverviewLink {
    int nLevel;
    GeoRasterRasterBand* poOverview;
};

//  ---------------------------------------------------------------------------
// Geographic system without EPSG parameters
//  ---------------------------------------------------------------------------

#define UNKNOWN_CRS 999999

//  ---------------------------------------------------------------------------
//  GeoRasterDriver, extends GDALDriver to support GeoRaster Server Connections
//  ---------------------------------------------------------------------------

class GeoRasterDriver : public GDALDriver
{
    friend class GeoRasterDataset;

public:
                        GeoRasterDriver();
    virtual            ~GeoRasterDriver();

private:

    OWConnection**      papoConnection;
    int                 nRefCount;

public:

    OWConnection*       GetConnection( const char* pszUser,
                            const char* pszPassword,
                            const char* pszServer );
};

//  ---------------------------------------------------------------------------
//  GeoRasterDataset, extends GDALDataset to support GeoRaster Datasets
//  ---------------------------------------------------------------------------

class GeoRasterDataset : public GDALDataset
{
    friend class GeoRasterRasterBand;

public:
                        GeoRasterDataset();
    virtual            ~GeoRasterDataset();

private:

    GeoRasterWrapper*   poGeoRaster;
    bool                bGeoTransform;
    bool                bForcedSRID;
    char*               pszProjection;
    char**              papszSubdatasets;
    double              adfGeoTransform[6];
    int                 nGCPCount;
    GDAL_GCP*           pasGCPList;
    int                 nOverviewCount;

public:

    void                SetSubdatasets( GeoRasterWrapper* poGRW );

    static int          Identify( GDALOpenInfo* poOpenInfo );
    static GDALDataset* Open( GDALOpenInfo* poOpenInfo );
    static CPLErr       Delete( const char *pszFilename );
    static GDALDataset* Create( const char* pszFilename,
                            int nXSize,
                            int nYSize,
                            int nBands,
                            GDALDataType eType,
                            char** papszOptions );
    static GDALDataset* CreateCopy( const char* pszFilename, 
                            GDALDataset* poSrcDS,
                            int bStrict,
                            char** papszOptions,
                            GDALProgressFunc pfnProgress, 
                            void* pProgressData );
    virtual CPLErr      GetGeoTransform( double* padfTransform );
    virtual CPLErr      SetGeoTransform( double* padfTransform );
    virtual const char* GetProjectionRef( void );
    virtual CPLErr      SetProjection( const char* pszProjString );
    virtual char**      GetMetadata( const char* pszDomain );
    virtual void        FlushCache( void );
    virtual CPLErr      IRasterIO( GDALRWFlag eRWFlag, 
                            int nXOff, int nYOff, int nXSize, int nYSize,
                            void *pData, int nBufXSize, int nBufYSize, 
                            GDALDataType eBufType,
                            int nBandCount, int *panBandMap, 
                            int nPixelSpace, int nLineSpace, int nBandSpace );
    virtual int         GetGCPCount() { return nGCPCount; }
    virtual const char* GetGCPProjection();
    virtual const GDAL_GCP*
                        GetGCPs() { return pasGCPList; }
    virtual CPLErr      SetGCPs(
                            int nGCPCount,
                            const GDAL_GCP *pasGCPList,
                            const char *pszGCPProjection );
    virtual CPLErr      IBuildOverviews(
                            const char* pszResampling,
                            int nOverviews,
                            int* panOverviewList,
                            int nBands,
                            int* panBandList,
                            GDALProgressFunc pfnProgress,
                            void* pProgressData );
};

//  ---------------------------------------------------------------------------
//  GeoRasterRasterBand, extends GDALRasterBand to support GeoRaster Band
//  ---------------------------------------------------------------------------

class GeoRasterRasterBand : public GDALRasterBand
{
    friend class GeoRasterDataset;

public:
                        GeoRasterRasterBand( GeoRasterDataset* poGDS, 
                            int nBand,
                            int nLevel );
    virtual            ~GeoRasterRasterBand();

private:

    GeoRasterWrapper*   poGeoRaster;
    GDALColorTable*     poColorTable;
    GDALRasterAttributeTable*
                        poDefaultRAT;
    double              dfMin;
    double              dfMax;
    double              dfMean;
    double              dfStdDev;
    bool                bValidStats;
    double              dfNoData;
    char*               pszVATName;
    int                 nOverviewLevel;
    OverviewLink**      paphOverviewLinks;
    int                 nOverviewLinks;

public:

    virtual double      GetNoDataValue( int *pbSuccess = NULL );
    virtual CPLErr      SetNoDataValue( double dfNoDataValue );
    virtual double      GetMinimum( int* pbSuccess = NULL );
    virtual double      GetMaximum( int* pbSuccess = NULL );
    virtual GDALColorTable* 
                        GetColorTable();
    virtual CPLErr      SetColorTable( GDALColorTable *poInColorTable ); 
    virtual GDALColorInterp   
                        GetColorInterpretation();
    virtual CPLErr      IReadBlock( int nBlockXOff, int nBlockYOff, 
                            void *pImage );
    virtual CPLErr      IWriteBlock( int nBlockXOff, int nBlockYOff, 
                            void *pImage );
    virtual CPLErr      SetStatistics( double dfMin, double dfMax, 
                            double dfMean, double dfStdDev );
    virtual CPLErr      GetStatistics( int bApproxOK, int bForce,
                            double* pdfMin, double* pdfMax, 
                            double* pdfMean, double* pdfStdDev );
    virtual const       GDALRasterAttributeTable *GetDefaultRAT();
    virtual CPLErr      SetDefaultRAT( const GDALRasterAttributeTable * );
    virtual int         GetOverviewCount();
    virtual GDALRasterBand*
                        GetOverview( int );
};

//  ---------------------------------------------------------------------------
//  GeoRasterWrapper, an interface for Oracle Spatial SDO_GEORASTER objects
//  ---------------------------------------------------------------------------

class GeoRasterWrapper
{

public:

                        GeoRasterWrapper();
    virtual            ~GeoRasterWrapper();

private:

    OCILobLocator**     pahLocator;
    int                 nBlockCount;
    unsigned long       nBlockBytes;
    unsigned long       nBlockBytesGDAL;
    GByte*              pabyBlockBuf;
    OWStatement*        poStmtRead;
    OWStatement*        poStmtWrite;

    int                 nCurrentBlock;
    int                 nCurrentLevel;

    int                 nCellSizeBits;
    int                 nCellSizeGDAL;

    bool                bIOInitialized;
    bool                bBlobInitialized;
    bool                bFlushMetadata;
    bool                bFlushBuffer;

    void                InitializeLayersNode();
    bool                InitializeIO( int nLevel, bool bUpdate = false );
    bool                FlushMetadata();

    void                UnpackNBits( GByte* pabyData );
    void                PackNBits( GByte* pabyOutBuf, void* pData );
    void                CompressJpeg();
    void                UncompressJpeg( unsigned long nInSize );
    bool                UncompressDeflate( unsigned long nDestLen,
                            unsigned long nSourceLen );
    bool                CompressDeflate( void* pData );

public:

    static char**       ParseIdentificator( const char* pszStringID );
    static GeoRasterWrapper*
                        Open( const char* pszStringID );
    bool                Create(
                            char* pszDescription,
                            char* pszInsert,
                            bool bUpdate );
    bool                Delete( void );
    void                GetRasterInfo( void );
    bool                GetImageExtent( double *padfTransform );
    bool                GetStatistics( int nBand,
                            double dfMin,
                            double dfMax,
                            double dfMean,
                            double dfStdDev );
    bool                SetStatistics( double dfMin,
                            double dfMax,
                            double dfMean,
                            double dfStdDev,
                            int nBand );
    bool                HasColorMap( int nBand );
    void                GetColorMap( int nBand, GDALColorTable* poCT );
    void                SetColorMap( int nBand, GDALColorTable* poCT );
    bool                SetGeoReference( int nSRIDIn );
    char*               GetWKText( int nSRIDin );
    bool                GetDataBlock(
                            int nBand,
                            int nLevel,
                            int nXOffset,
                            int nYOffset,
                            void* pData );
    bool                SetDataBlock(
                            int nBand,
                            int nLevel,
                            int nXOffset,
                            int nYOffset,
                            void* pData );
    bool                GetNoData( double* pdfNoDataValue );
    bool                SetNoData( double dfNoDataValue );
    CPLXMLNode*         GetMetadata() { return phMetadata; };
    bool                SetVAT( int nBand, const char* pszName );
    char*               GetVAT( int nBand );
    bool                GeneratePyramid(
                            int nLevels,
                            const char* pszResampling,
                            bool bNodata = false );

public:

    OWConnection*       poConnection;

    char*               pszTable;
    char*               pszColumn;
    char*               pszDataTable;
    int                 nRasterId;
    char*               pszWhere;

    bool                bRDTRIDOnly;

    int                 nSRID;
    CPLXMLNode*         phMetadata;
    char*               pszCellDepth;
    char*               pszCompressionType;

    int                 nRasterColumns;
    int                 nRasterRows;
    int                 nRasterBands;

    char                szInterleaving[4];
    bool                bIsReferenced;

    double              dfXCoefficient[3];
    double              dfYCoefficient[3];

    int                 nColumnBlockSize;
    int                 nRowBlockSize;
    int                 nBandBlockSize;

    int                 nTotalColumnBlocks;
    int                 nTotalRowBlocks;
    int                 nTotalBandBlocks;

    int                 iDefaultRedBand;
    int                 iDefaultGreenBand;
    int                 iDefaultBlueBand;

    int                 nPyramidMaxLevel;

    int                 bPackingOrCompress;
};
