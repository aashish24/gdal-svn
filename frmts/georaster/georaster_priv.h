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
    char*               pszSpatialRef;
    char**              papszSubdatasets;
    double              adfGeoTransform[6];

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
};

//  ---------------------------------------------------------------------------
//  GeoRasterRasterBand, extends GDALRasterBand to support GeoRaster Band
//  ---------------------------------------------------------------------------

class GeoRasterRasterBand : public GDALRasterBand
{
    friend class GeoRasterDataset;

public:
                        GeoRasterRasterBand( GeoRasterDataset* poGDS, 
                            int nBand );
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
    int                 nBandBlock;
    int                 nBlockCount;
    int                 nBlockBytes;
    int                 nBlockBytesGDAL;
    int                 nBandBytes;
    GByte*              pabyBlockBuf;
    OWStatement*        poStmtRead;
    OWStatement*        poStmtWrite;

    int                 nCurrentBandBlock;
    int                 nCurrentBlock;

    int                 nPyraLevel;
    int                 nCellSize;
    int                 nCellSizeGDAL;

    bool                bIOInitialized;
    bool                bBlobInitialized;
    bool                bFlushMetadata;

    void                InitializeLayersNode();
    bool                InitializeIO();
    bool                FlushMetadata();

public:

    static GeoRasterWrapper*
                        Open( const char* pszStringID );
    bool                Create( char* pszDescription, char* pszInsert );
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
    bool                HasColorTable( int nBand );
    void                GetColorTable( int nBand, GDALColorTable* poCT );
    void                SetColorTable( int nBand, GDALColorTable* poCT );
    bool                SetGeoReference( int nSRIDIn );
    char*               GetWKText( int nSRIDin );
    bool                GetBandBlock( 
                            int nBand, 
                            int nXOffset, 
                            int nYOffset, 
                            void* pData );
    bool                SetBandBlock( 
                            int nBand, 
                            int nXOffset, 
                            int nYOffset, 
                            void* pData );
    bool                GetNoData( double* pdfNoDataValue );
    bool                SetNoData( double dfNoDataValue );
    CPLXMLNode*         GetMetadata() { return phMetadata; };

public:

    OWConnection*       poConnection;

    char*               pszTable;
    char*               pszColumn;
    char*               pszDataTable;
    int                 nRasterId;
    char*               pszWhere;

    int                 nSRID;
    CPLXMLNode*         phMetadata;
    char*               pszCellDepth;

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
};
