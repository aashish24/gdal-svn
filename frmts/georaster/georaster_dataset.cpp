/******************************************************************************
 * $Id: $
 *
 * Name:     georaster_dataset.cpp
 * Project:  Oracle Spatial GeoRaster Driver
 * Purpose:  Implement GeoRasterDataset Methods
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

#include "georaster_priv.h"

CPL_C_START
void CPL_DLL GDALRegister_GEOR(void);
CPL_C_END

bool    OWIsNumeric( const char *pszText );

//  ---------------------------------------------------------------------------
//                                                           GeoRasterDataset()
//  ---------------------------------------------------------------------------

GeoRasterDataset::GeoRasterDataset()
{
    bGeoTransform       = false;
    bForcedSRID         = false;
    poGeoRaster         = NULL;
    papszSubdatasets    = NULL;
    adfGeoTransform[0]  = 0.0;
    adfGeoTransform[1]  = 1.0;
    adfGeoTransform[2]  = 0.0;
    adfGeoTransform[3]  = 0.0;
    adfGeoTransform[4]  = 0.0;
    adfGeoTransform[5]  = 1.0;
    pszProjection       = NULL;
    nGCPCount           = 0;
    pasGCPList          = NULL;
    poDriver            = (GDALDriver *) GDALGetDriverByName( "GEORASTER" );
}

//  ---------------------------------------------------------------------------
//                                                          ~GeoRasterDataset()
//  ---------------------------------------------------------------------------

GeoRasterDataset::~GeoRasterDataset()
{
    if( nGCPCount > 0 )
    {
        GDALDeinitGCPs( nGCPCount, pasGCPList );
        CPLFree( pasGCPList );
    }

    delete poGeoRaster;
    CPLFree( pszProjection );
    CSLDestroy( papszSubdatasets );
}

//  ---------------------------------------------------------------------------
//                                                                   Identify()
//  ---------------------------------------------------------------------------

int GeoRasterDataset::Identify( GDALOpenInfo* poOpenInfo )
{
    //  -------------------------------------------------------------------
    //  Verify georaster prefix
    //  -------------------------------------------------------------------

    char* pszFilename = poOpenInfo->pszFilename;

    if( EQUALN( pszFilename, "georaster:", 10 ) == false &&
        EQUALN( pszFilename, "geor:", 5 )       == false )
    {
        return false;
    }

    //  -------------------------------------------------------------------
    //  Parse arguments
    //  -------------------------------------------------------------------

    char** papszParam = GeoRasterWrapper::ParseIdentificator( 
        poOpenInfo->pszFilename );

    int nArgc = CSLCount( papszParam );

    //  -------------------------------------------------------------------
    //  Check mandatory arguments
    //  -------------------------------------------------------------------

    if ( nArgc < 2 ||
         nArgc > 6 ||
         EQUAL( papszParam[0], "" ) ||
         EQUAL( papszParam[1], "" ) )
    {
        CPLError( CE_Warning, CPLE_IllegalArg,
        "Invalid georaster identification\n"
        "Usage:\n"
        "    {georaster/geor}:<user>{,/}<pwd>{,@}[db],[table],[column],[where]\n"
        "    {georaster/geor}:<user>{,/}<pwd>{,@}[db],<rdt>:<rid>\n"
        "    user   - user's login\n"
        "    pwd    - user's password\n"
        "    db     - connection string ( default is $ORACLE_SID )\n"
        "    table  - name of a georaster table\n"
        "    column - name of a georaster column\n"
        "    where  - simple where clause\n"
        "    rdt    - raster data table name\n"
        "    rid    - georaster numeric identification\n"
        "Examples:\n"
        "    geor:scott/tiger@demodb,table,column,id=1\n"
        "    geor:scott/tiger@server.company.com:1521/survey,table,column,id=1\n"
        "    \"georaster:scott,tiger,demodb,table,column,city='london'\"\n"
        "    georaster:scott,tiger,,rdt_10$,10\n" );
        CSLDestroy( papszParam );
        return false;
    }
    CSLDestroy( papszParam );
    return true;
}

//  ---------------------------------------------------------------------------
//                                                           GeoRasterDataset()
//  ---------------------------------------------------------------------------

GDALDataset* GeoRasterDataset::Open( GDALOpenInfo* poOpenInfo )
{
    //  -------------------------------------------------------------------
    //  It shouldn't have an open file pointer
    //  -------------------------------------------------------------------

    if( poOpenInfo->fp != NULL )
    {
        return NULL;
    }

    //  -------------------------------------------------------------------
    //  Check identification string and usage
    //  -------------------------------------------------------------------

    if( ! Identify( poOpenInfo ) )
    {
        return NULL;
    }

    //  -------------------------------------------------------------------
    //  Create a GeoRaster wrapper object
    //  -------------------------------------------------------------------

    GeoRasterWrapper* poGRW = GeoRasterWrapper::Open(poOpenInfo->pszFilename);

    if( ! poGRW )
    {
        return NULL;
    }

    //  -------------------------------------------------------------------
    //  Create a corresponding GDALDataset
    //  -------------------------------------------------------------------

    GeoRasterDataset *poGRD;

    poGRD = new GeoRasterDataset();

    if( ! poGRD )
    {
        return NULL;
    }

    poGRD->eAccess       = poOpenInfo->eAccess;
    poGRD->poGeoRaster   = poGRW;

    //  -------------------------------------------------------------------
    //  Assign GeoRaster information
    //  -------------------------------------------------------------------

    if( poGRW->GetMetadata() != NULL )
    {
        poGRD->nRasterXSize  = poGRW->nRasterColumns;
        poGRD->nRasterYSize  = poGRW->nRasterRows;
        poGRD->nBands        = poGRW->nRasterBands;
        poGRD->poGeoRaster   = poGRW;
    }
    else
    {
        poGRD->SetSubdatasets( poGRW );

        if( CSLCount( poGRD->papszSubdatasets ) == 0 &&
            poGRD->eAccess == GA_ReadOnly )
        {
            delete poGRD;
            poGRD = NULL;
    }

        return (GDALDataset*) poGRD;
    }

    //  -------------------------------------------------------------------
    //  Create bands
    //  -------------------------------------------------------------------

    int i = 0;
    int nBand = 0;

    for( i = 0; i < poGRD->nBands; i++ )
    {
        nBand = i + 1;
        poGRD->SetBand( nBand, new GeoRasterRasterBand( poGRD, nBand ) );
    }

    //  -------------------------------------------------------------------
    //  Set objectInfo metadata
    //  -------------------------------------------------------------------

    poGRD->SetMetadataItem("objectInfo.rasterType", CPLGetXMLValue(
        poGRW->phMetadata, "objectInfo.rasterType", "NONE" ), "ORACLE" );

    poGRD->SetMetadataItem("objectInfo.isBlank", CPLGetXMLValue(
        poGRW->phMetadata, "objectInfo.isBlank", "NONE" ), "ORACLE" );

    poGRD->SetMetadataItem("objectInfo.defaultRed", CPLGetXMLValue(
        poGRW->phMetadata, "objectInfo.defaultRed", "NONE" ), "ORACLE" );

    poGRD->SetMetadataItem("objectInfo.defaultGreen", CPLGetXMLValue(
        poGRW->phMetadata, "objectInfo.defaultGreen", "NONE" ), "ORACLE" );

    poGRD->SetMetadataItem("objectInfo.defaultBlue", CPLGetXMLValue(
        poGRW->phMetadata, "objectInfo.defaultBlue", "NONE" ), "ORACLE" );

    //  -------------------------------------------------------------------
    //  Set rasterInfo metadata
    //  -------------------------------------------------------------------

    poGRD->SetMetadataItem("rasterInfo.cellDepth", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.cellDepth", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.totalDimensions", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.totalDimensions", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.dimensionSize.row", CPLSPrintf( "%d",
        poGRD->nRasterYSize),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.dimensionSize.column", CPLSPrintf( "%d",
        poGRD->nRasterXSize),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.dimensionSize.band", CPLSPrintf( "%d",
        poGRD->nBands),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.ULTCoordinate.row", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.ULTCoordinate.row", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.ULTCoordinate.column", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.ULTCoordinate.column", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.ULTCoordinate.band", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.ULTCoordinate.band", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.blocking.type", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.blocking.type", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.blocking.totalRowBlocks", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.blocking.totalRowBlocks", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.blocking.totalColumnBlocks", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.blocking.totalColumnBlocks", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.blocking.totalBandBlocks", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.blocking.totalBandBlocks", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.blocking.rowBlockSize", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.blocking.rowBlockSize", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.blocking.columnBlockSize", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.blocking.columnBlockSize", "NONE" ),
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.blocking.bandBlockSize", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.blocking.bandBlockSize", "NONE" ), 
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.interleaving", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.interleaving", "NONE" ), 
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.pyramid.type", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.pyramid.type", "NONE" ), 
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.pyramid.maxLevel", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.pyramid.maxLevel", "NONE" ), 
        "ORACLE" );

    poGRD->SetMetadataItem("rasterInfo.compression.type", CPLGetXMLValue(
        poGRW->phMetadata, "rasterInfo.compression.type", "NONE" ), 
        "ORACLE" );

    //  -------------------------------------------------------------------
    //  Set Spatial Reference metadata
    //  -------------------------------------------------------------------

    poGRD->SetMetadataItem("spatialReferenceInfo.isReferenced", CPLGetXMLValue(
        poGRW->phMetadata, "spatialReferenceInfo.isReferenced", "FALSE" ), 
        "ORACLE" );

    poGRD->SetMetadataItem("spatialReferenceInfo.SRID", CPLGetXMLValue(
        poGRW->phMetadata, "spatialReferenceInfo.SRID", "0" ), 
        "ORACLE" );

    //  -------------------------------------------------------------------
    //  Set RDT/RID metadata
    //  -------------------------------------------------------------------

    poGRD->SetMetadataItem("georasterTable", poGRW->pszTable, "ORACLE" );
    poGRD->SetMetadataItem("georasterColumn", poGRW->pszColumn, "ORACLE" );
    poGRD->SetMetadataItem("rasterDataTable", poGRW->pszDataTable, "ORACLE" );
    poGRD->SetMetadataItem("rasterId", CPLSPrintf( "%d", poGRW->nRasterId ),
        "ORACLE" );

    //  -------------------------------------------------------------------
    //  Set IMAGE_STRUCTURE metadata information
    //  -------------------------------------------------------------------

    if( EQUAL( poGRW->szInterleaving, "BSQ" ) )
    {
        poGRD->SetMetadataItem( "INTERLEAVE", "BAND", "IMAGE_STRUCTURE" );
    }
    else if( EQUAL( poGRW->szInterleaving, "BIP" ) )
    {
        poGRD->SetMetadataItem( "INTERLEAVE", "PIXEL", "IMAGE_STRUCTURE" );
    }
    else if( EQUAL( poGRW->szInterleaving, "BIL" ) )
    {
        poGRD->SetMetadataItem( "INTERLEAVE", "LINE", "IMAGE_STRUCTURE" );
    }

    poGRD->SetMetadataItem( "COMPRESSION", CPLGetXMLValue( poGRW->phMetadata,
        "rasterInfo.compression.type", "NONE" ), "IMAGE_STRUCTURE" );

    if( EQUAL( poGRW->pszCellDepth, "1BIT" ) )
    {
        poGRD->SetMetadataItem( "NBITS", "1", "IMAGE_STRUCTURE" );
    }

    if( EQUAL( poGRW->pszCellDepth, "2BIT" ) )
    {
        poGRD->SetMetadataItem( "NBITS", "2", "IMAGE_STRUCTURE" );
    }

    if( EQUAL( poGRW->pszCellDepth, "4BIT" ) )
    {
        poGRD->SetMetadataItem( "NBITS", "4", "IMAGE_STRUCTURE" );
    }

    //  -------------------------------------------------------------------
    //  Return a GDALDataset
    //  -------------------------------------------------------------------

    return (GDALDataset*) poGRD;
}

//  ---------------------------------------------------------------------------
//                                                                     Create()
//  ---------------------------------------------------------------------------

GDALDataset *GeoRasterDataset::Create( const char *pszFilename,
                                       int nXSize,
                                       int nYSize,
                                       int nBands, 
                                       GDALDataType eType,
                                       char **papszOptions )
{
    //  -------------------------------------------------------------------
    //  Check for supported Data types
    //  -------------------------------------------------------------------

    const char* pszCellDepth = OWSetDataType( eType );

    if( EQUAL( pszCellDepth, "Unknown" ) )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
            "Attempt to create GeoRaster with unsupported data type (%s)",
            GDALGetDataTypeName( eType ) );
        return NULL;
    }

    //  -------------------------------------------------------------------
    //  Open the Dataset
    //  -------------------------------------------------------------------

    GeoRasterDataset* poGRD = NULL;

    poGRD = (GeoRasterDataset*) GDALOpen( pszFilename, GA_Update );

    if( ! poGRD )
    {
        return NULL;
    }

    //  -------------------------------------------------------------------
    //  Get the GeoRaster
    //  -------------------------------------------------------------------

    GeoRasterWrapper* poGRW = poGRD->poGeoRaster;

    if( ! poGRW )
    {
        delete poGRD;
        return NULL;
    }

    //  -------------------------------------------------------------------
    //  Check for overwriting
    //  -------------------------------------------------------------------

    if( poGRW->GetMetadata() != NULL )
    {
        //  ---------------------------------------------------------------
        //  Overwriting an existing GeoRaster
        //  ---------------------------------------------------------------

        //  TODO: Allow users to change the format before overwriting

        return (GeoRasterDataset *) poGRD;
    }

    //  -------------------------------------------------------------------
    //  Set basic information and default values
    //  -------------------------------------------------------------------

    poGRW->nRasterColumns       = nXSize;
    poGRW->nRasterRows          = nYSize;
    poGRW->nRasterBands         = nBands;
    poGRW->pszCellDepth         = CPLStrdup( OWSetDataType( eType ) );

    //  -------------------------------------------------------------------
    //  Check the create options to use in initialization
    //  -------------------------------------------------------------------

    const char* pszFetched      = "";
    char* pszDescription        = NULL;
    char* pszInsert             = NULL;

    if( poGRW->pszTable )
    {
        pszFetched = CSLFetchNameValue( papszOptions, "DESCRIPTION" );

        if( pszFetched )
        {
            pszDescription      = CPLStrdup( pszFetched );
        }
    }

    if( ! poGRW->pszTable )
    {
        poGRW->pszTable         = CPLStrdup( "GDAL_IMPORT" );
    }

    if( ! poGRW->pszColumn )
    {
        poGRW->pszColumn        = CPLStrdup( "RASTER" );
    }

    pszFetched = CSLFetchNameValue( papszOptions, "INSERT" );

    if( pszFetched )
    {
        pszInsert               = CPLStrdup( pszFetched );
    }

    pszFetched = CSLFetchNameValue( papszOptions, "BLOCKXSIZE" );

    if( pszFetched )
    {
        poGRW->nColumnBlockSize = atoi( pszFetched );
    }

    pszFetched = CSLFetchNameValue( papszOptions, "BLOCKYSIZE" );

    if( pszFetched )
    {
        poGRW->nRowBlockSize    = atoi( pszFetched );
    }

    pszFetched = CSLFetchNameValue( papszOptions, "BLOCKBSIZE" );

    if( pszFetched )
    {
        poGRW->nBandBlockSize   = atoi( pszFetched );
    }

    if( poGRW->nRowBlockSize == 0 )
    {
        poGRW->nRowBlockSize    = 256;
    }

    if( poGRW->nColumnBlockSize == 0 )
    {
        poGRW->nColumnBlockSize = 256;
    }

    if( poGRW->nBandBlockSize == 0 )
    {
        poGRW->nBandBlockSize   = nBands;
    }

    pszFetched = CSLFetchNameValue( papszOptions, "INTERLEAVE" );

    if( pszFetched )
    {
        if( EQUAL( pszFetched, "BAND" ) ||  EQUAL( pszFetched, "BSQ" ) )
        {
            strcpy( poGRW->szInterleaving, "BSQ" );
        }
        if( EQUAL( pszFetched, "LINE" ) ||  EQUAL( pszFetched, "BIL" ) )
        {
            strcpy( poGRW->szInterleaving, "BIL" );
        }
        if( EQUAL( pszFetched, "PIXEL" ) ||  EQUAL( pszFetched, "BIP" ) )
        {
            strcpy( poGRW->szInterleaving, "BIP" );
        }
    }

    bool bSucced = poGRW->Create( pszDescription, pszInsert );

    CPLFree( pszInsert );
    CPLFree( pszDescription );

    if( ! bSucced )
    {
        delete poGRD;
        return NULL;
    }

    //  -------------------------------------------------------------------
    //  Pepare a identification string
    //  -------------------------------------------------------------------

    char szStringId[OWTEXT];

    strcpy( szStringId, CPLSPrintf( "georaster:%s,%s,%s,%s,%d",
        poGRW->poConnection->GetUser(),
        poGRW->poConnection->GetPassword(),
        poGRW->poConnection->GetServer(),
        poGRW->pszDataTable,
        poGRW->nRasterId ) );

    //  -------------------------------------------------------------------
    //  Load the GeoRaster
    //  -------------------------------------------------------------------

    delete poGRD;

    poGRD = (GeoRasterDataset*) GDALOpen( szStringId, GA_Update );

    //  -------------------------------------------------------------------
    //  Load aditional options
    //  -------------------------------------------------------------------

    pszFetched = CSLFetchNameValue( papszOptions, "SRID" );

    if( pszFetched )
    {
        poGRD->bForcedSRID = true; /* ignore others methods */
        poGRD->poGeoRaster->SetGeoReference( atoi( pszFetched ) );
    }

    //  -------------------------------------------------------------------
    //  Return a new Dataset
    //  -------------------------------------------------------------------

    return (GDALDataset*) poGRD;
}

//  ---------------------------------------------------------------------------
//                                                                 CreateCopy()
//  ---------------------------------------------------------------------------

GDALDataset *GeoRasterDataset::CreateCopy( const char* pszFilename,
                                           GDALDataset* poSrcDS,
                                           int bStrict,
                                           char** papszOptions,
                                           GDALProgressFunc pfnProgress,
                                           void* pProgressData )
{
    (void) bStrict;

    GDALRasterBand* poBand = poSrcDS->GetRasterBand( 1 );
    GDALDataType    eType  = poBand->GetRasterDataType();

    //  -----------------------------------------------------------
    //  Create a GeoRaster on the server or select one to overwrite
    //  -----------------------------------------------------------

    GeoRasterDataset *poGRD;

    poGRD = (GeoRasterDataset *) GeoRasterDataset::Create( pszFilename,
        poSrcDS->GetRasterXSize(),
        poSrcDS->GetRasterYSize(),
        poSrcDS->GetRasterCount(),
        eType, papszOptions );

    if( poGRD == NULL )
    {
        return NULL;
    }

    //  -----------------------------------------------------------
    //  Copy information to the dataset
    //  -----------------------------------------------------------

    double adfTransform[6];

    poSrcDS->GetGeoTransform( adfTransform );

    poGRD->SetGeoTransform( adfTransform );

    if( ! poGRD->bForcedSRID ) /* forced by create option SRID */
    {
        poGRD->SetProjection( poSrcDS->GetProjectionRef() );
    }

    // --------------------------------------------------------------------
    //      Copy information to the raster bands
    // --------------------------------------------------------------------

    int    bHasNoDataValue = FALSE;
    double dfNoDataValue = 0.0;
    double dfMin, dfMax, dfStdDev, dfMean;
    int    iBand = 0;

    for( iBand = 1; iBand <= poGRD->nBands; iBand++ )
    {
        GDALRasterBand *poSrcBand = poSrcDS->GetRasterBand( iBand );
        GDALRasterBand *poDstBand = poGRD->GetRasterBand( iBand );

        poSrcBand = poSrcDS->GetRasterBand( iBand );
        poDstBand = (GeoRasterRasterBand*) poGRD->GetRasterBand( iBand );

        if( ( (GeoRasterRasterBand*)
            poDstBand)->poColorTable->GetColorEntryCount() == 0 )
        {
            poDstBand->SetColorTable( poSrcBand->GetColorTable() );
        }

        if( poDstBand->GetStatistics( false, false, &dfMin, &dfMax,
            &dfStdDev, &dfMean ) == CE_None )
        {
            poDstBand->SetStatistics( dfMin, dfMax, dfStdDev, dfMean );
        }

        const GDALRasterAttributeTable *poRAT = poSrcBand->GetDefaultRAT();

        if( poRAT != NULL )
        {
            poDstBand->SetDefaultRAT( poRAT );
        }

        dfNoDataValue = poSrcBand->GetNoDataValue( &bHasNoDataValue );

        if( bHasNoDataValue )
        {
            poDstBand->SetNoDataValue( dfNoDataValue );
        }
    }

    // --------------------------------------------------------------------
    //  Copy actual imagery.
    // --------------------------------------------------------------------

    int nXSize = poGRD->GetRasterXSize();
    int nYSize = poGRD->GetRasterYSize();

    int nBlockXSize = 0;
    int nBlockYSize = 0;

    poGRD->GetRasterBand( 1 )->GetBlockSize( &nBlockXSize, &nBlockYSize );

    void *pData = VSIMalloc( nBlockXSize * nBlockYSize *
        GDALGetDataTypeSize( eType ) / 8 );

    if( pData == NULL )
    {
        CPLError( CE_Failure, CPLE_OutOfMemory,
            "GeoRaster::CreateCopy : Out of memory " );
        delete poGRD;
        return NULL;
    }

    int iYOffset = 0;
    int iXOffset = 0;
    int iXBlock  = 0;
    int iYBlock  = 0;
    int nBlockCols = 0;
    int nBlockRows = 0;
    CPLErr eErr = CE_None;

    poGRD->poGeoRaster->SetOptimizedWriting();

    int nPixelSize = GDALGetDataTypeSize( 
        poSrcDS->GetRasterBand(1)->GetRasterDataType() ) / 8;

    for( iYOffset = 0, iYBlock = 0;
         iYOffset < nYSize;
         iYOffset += nBlockYSize, iYBlock++ )
    {
        for( iXOffset = 0, iXBlock = 0;
             iXOffset < nXSize;
             iXOffset += nBlockXSize, iXBlock++ )
        {
            nBlockCols = MIN( nBlockXSize, nXSize - iXOffset );
            nBlockRows = MIN( nBlockYSize, nYSize - iYOffset );

            for( iBand = 1; 
                 iBand <= poSrcDS->GetRasterCount(); 
                 iBand++ )
            {
                GDALRasterBand *poSrcBand = poSrcDS->GetRasterBand( iBand );
                GDALRasterBand *poDstBand = poGRD->GetRasterBand( iBand );

                eErr = poSrcBand->RasterIO( GF_Read,
                    iXOffset, iYOffset,
                    nBlockCols, nBlockRows, pData,
                    nBlockCols, nBlockRows, eType, 1, ( nBlockXSize * nPixelSize ) );

                if( eErr != CE_None )
                {
                    return NULL;
                }

                eErr = poDstBand->WriteBlock( iXBlock, iYBlock, pData );
                if( eErr != CE_None )
                {
                    return NULL;
                }
            }

        }

        if( ( eErr == CE_None ) && ( ! pfnProgress(
            ( iYOffset + nBlockRows ) / (double) nYSize, NULL, pProgressData ) ) )
        {
            eErr = CE_Failure;
            CPLError( CE_Failure, CPLE_UserInterrupt,
                "User terminated CreateCopy()" );
        }
    }

    CPLFree( pData );

    // --------------------------------------------------------------------
    //      Finalize
    // --------------------------------------------------------------------

    poGRD->FlushCache();

    if( pfnProgress )
    {
        printf( "Ouput dataset: (georaster:%s,%s,%s,%s,%d) on %s,%s",
            poGRD->poGeoRaster->poConnection->GetUser(),
            poGRD->poGeoRaster->poConnection->GetPassword(),
            poGRD->poGeoRaster->poConnection->GetServer(),
            poGRD->poGeoRaster->pszDataTable,
            poGRD->poGeoRaster->nRasterId,
            poGRD->poGeoRaster->pszTable,
            poGRD->poGeoRaster->pszColumn );
    }

    return poGRD;
}

//  ---------------------------------------------------------------------------
//                                                            GetGeoTransform()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterDataset::GetGeoTransform( double *padfTransform )
{
    if( bGeoTransform )
    {
        memcpy( padfTransform, adfGeoTransform, sizeof(double) * 6 );
        return CE_None;
    }

    if( ! poGeoRaster->bIsReferenced )
    {
        return CE_Failure;
    }

    if( ! poGeoRaster->GetImageExtent( adfGeoTransform ) )
    {
        return CE_Failure;
    }

    bGeoTransform = true;

    memcpy( padfTransform, adfGeoTransform, sizeof(double) * 6 );

    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                           GetProjectionRef()
//  ---------------------------------------------------------------------------

const char* GeoRasterDataset::GetProjectionRef( void )
{
    if( ! poGeoRaster->bIsReferenced )
    {
        return NULL;
    }

    if( pszProjection )
    {
        return pszProjection;
    }

    OGRSpatialReference oSRS;

    // --------------------------------------------------------------------
    // Try to interprete the EPSG code
    // --------------------------------------------------------------------

    if( oSRS.importFromEPSG( poGeoRaster->nSRID ) == OGRERR_NONE )
    {
        oSRS.exportToWkt( &pszProjection );
        return pszProjection;
    }

    // --------------------------------------------------------------------
    // Try to interpreter the WKT text
    // --------------------------------------------------------------------

    char* pszWKText = CPLStrdup( poGeoRaster->GetWKText( poGeoRaster->nSRID ) );

    if( oSRS.importFromWkt( &pszWKText ) != OGRERR_NONE || 
        oSRS.GetRoot() == NULL )
    {
        return FALSE;
    }

    // --------------------------------------------------------------------
    // Try to extract EPGS authority codes
    // --------------------------------------------------------------------

    int nSpher = OWParseEPSG( oSRS.GetAttrValue("GEOGCS|DATUM|SPHEROID") );
    if( nSpher > 0 )
    {
        oSRS.SetAuthority( "GEOGCS|DATUM|SPHEROID", "EPSG", nSpher );
    }

    int nDatum = OWParseEPSG( oSRS.GetAttrValue("GEOGCS|DATUM") );
    if( nDatum > 0 )
    {
        oSRS.SetAuthority( "GEOGCS|DATUM", "EPSG", nDatum );
    }

    int nProjc = OWParseEPSG( oSRS.GetAttrValue("PROJECTION") );
    if( nProjc > 0 )
    {
        oSRS.SetAuthority( "PROJECTION", "EPSG", nProjc );
    }

    oSRS.SetAuthority( oSRS.GetRoot()->GetValue(), "EPSG", poGeoRaster->nSRID );
    oSRS.exportToWkt( &pszProjection );

    return pszProjection;
}

//  ---------------------------------------------------------------------------
//                                                                  IRasterIO()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterDataset::IRasterIO( GDALRWFlag eRWFlag,
                                    int nXOff, int nYOff, int nXSize, int nYSize,
                                    void *pData, int nBufXSize, int nBufYSize,
                                    GDALDataType eBufType,
                                    int nBandCount, int *panBandMap,
                                    int nPixelSpace, int nLineSpace, int nBandSpace )

{
    if( nBandCount > 1 )
    {
        return GDALDataset::BlockBasedRasterIO( eRWFlag,
            nXOff, nYOff, nXSize, nYSize,
            pData, nBufXSize, nBufYSize, eBufType,
            nBandCount, panBandMap, nPixelSpace,
            nLineSpace, nBandSpace );
    }
    else
    {
        return GDALDataset::IRasterIO( eRWFlag,
            nXOff, nYOff, nXSize, nYSize,
            pData, nBufXSize, nBufYSize, eBufType,
            nBandCount, panBandMap,
            nPixelSpace, nLineSpace, nBandSpace );
    }
}

//  ---------------------------------------------------------------------------
//                                                                 FlushCache()
//  ---------------------------------------------------------------------------

void GeoRasterDataset::FlushCache()
{
    GDALDataset::FlushCache();
}

//  ---------------------------------------------------------------------------
//                                                            SetGeoTransform()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterDataset::SetGeoTransform( double *padfTransform )
{
    memcpy( adfGeoTransform, padfTransform, sizeof( double ) * 6 );

    poGeoRaster->dfXCoefficient[0] = adfGeoTransform[1];
    poGeoRaster->dfXCoefficient[1] = adfGeoTransform[2];
    poGeoRaster->dfXCoefficient[2] = adfGeoTransform[0];
    poGeoRaster->dfYCoefficient[0] = adfGeoTransform[4];
    poGeoRaster->dfYCoefficient[1] = adfGeoTransform[5];
    poGeoRaster->dfYCoefficient[2] = adfGeoTransform[3];

    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                              SetProjection()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterDataset::SetProjection( const char *pszProjString )
{
    OGRSpatialReference oSRS;

    char* pszWKT = CPLStrdup( pszProjString );

    OGRErr eOGRErr = oSRS.importFromWkt( &pszWKT );

    if( eOGRErr != OGRERR_NONE )
    {
        return CE_Failure;
    }

    // --------------------------------------------------------------------
    // Try to extract EPGS authority code
    // --------------------------------------------------------------------

    const char *pszAuthName = NULL, *pszAuthCode = NULL;

    if( oSRS.IsGeographic() )
    {
        pszAuthName = oSRS.GetAuthorityName( "GEOGCS" );
        pszAuthCode = oSRS.GetAuthorityCode( "GEOGCS" );
    }
    else if( oSRS.IsProjected() )
    {
        pszAuthName = oSRS.GetAuthorityName( "PROJCS" );
        pszAuthCode = oSRS.GetAuthorityCode( "PROJCS" );
    }

    if( pszAuthName != NULL && pszAuthCode != NULL )
    {
        if( EQUAL( pszAuthName, "Oracle" ) || 
            EQUAL( pszAuthName, "EPSG" ) )
        {
            if( ! poGeoRaster->SetGeoReference( atoi( pszAuthCode ) ) )
            {
               return CE_Failure;
            }
            return CE_None;
        }
    }

    // ----------------------------------------------------------------
    // Convert SRS into old style format (SF-SQL 1.0)
    // ----------------------------------------------------------------

    OGRSpatialReference *poSRS2 = oSRS.Clone();
    poSRS2->StripCTParms();

    double dfAngularUnits = poSRS2->GetAngularUnits( NULL );
    if( fabs(dfAngularUnits - 0.0174532925199433) < 0.0000000000000010 )
    {
        poSRS2->SetAngularUnits( "Decimal Degree", 0.0174532925199433 );
    }

    char* pszCloneWKT = NULL;

    if( poSRS2->exportToWkt( &pszCloneWKT ) != OGRERR_NONE )
    {
        delete poSRS2;
        return CE_Failure;
    }

    //TODO: Try to find a correspondent WKT on the server

    CPLDebug("GEORASTER","WKT:%s", pszCloneWKT);

    CPLFree( pszCloneWKT );

    delete poSRS2;

    return CE_Failure;
}

//  ---------------------------------------------------------------------------
//                                                                GetMetadata()
//  ---------------------------------------------------------------------------

char **GeoRasterDataset::GetMetadata( const char *pszDomain )
{
    if( pszDomain != NULL && EQUALN( pszDomain, "SUBDATASETS", 11 ) )
        return papszSubdatasets;
    else
        return GDALDataset::GetMetadata( pszDomain );
}

//  ----------------------------------------------------------------------------
//                                                                      Delete()
//  ----------------------------------------------------------------------------

CPLErr GeoRasterDataset::Delete( const char* pszFilename )
{
    (void) pszFilename;

    //TODO: Should I?

    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                             SetSubdatasets()
//  ---------------------------------------------------------------------------

void GeoRasterDataset::SetSubdatasets( GeoRasterWrapper* poGRW )
{
    if( eAccess == GA_Update )
    {
        return;
    }

    OWConnection* poConnection  = poGRW->poConnection;
    OWStatement*  poStmt        = NULL;

    char* papszToken[10];
    papszToken[0] = poConnection->GetUser();
    papszToken[1] = poConnection->GetPassword();
    papszToken[2] = poConnection->GetServer();
    papszToken[3] = papszToken[4] = papszToken[5] = 
                    papszToken[6] = papszToken[7] = 
                    papszToken[8] = papszToken[9] = "";

    //  -----------------------------------------------------------
    //  List all the GeoRaster Tables of that User/Database
    //  -----------------------------------------------------------

    char szTable[OWNAME]      = "";
    char szColumn[OWNAME]     = "";
    char szDataTable[OWNAME]  = "";
    char szRasterId[OWNAME]   = "";

    if( poGRW->pszTable  == NULL && 
        poGRW->pszColumn == NULL )
    {
        poStmt = poConnection->CreateStatement(
            "SELECT DISTINCT TABLE_NAME FROM USER_SDO_GEOR_SYSDATA" );

        poStmt->Define( szTable );

        papszToken[3] = papszToken[7] = szTable;
        papszToken[6] = "Table:";
    }

    //  -----------------------------------------------------------
    //  List all the GeoRaster Columns of that Table
    //  -----------------------------------------------------------

    if( poGRW->pszTable  != NULL && 
        poGRW->pszColumn == NULL )
    {
        poStmt = poConnection->CreateStatement(
            "SELECT DISTINCT COLUMN_NAME FROM USER_SDO_GEOR_SYSDATA\n"
            "WHERE  TABLE_NAME = UPPER(:1)" );

        poStmt->Bind( poGRW->pszTable );
        poStmt->Define( szColumn );

        papszToken[3] = poGRW->pszTable;
        papszToken[4] = ",";
        papszToken[5] = papszToken[7] = szColumn;
        papszToken[6] = "Column:";
    }

    //  -----------------------------------------------------------
    //  List all the rows that contains GeoRaster on Table/Column/Where
    //  -----------------------------------------------------------

    if( poGRW->pszTable  != NULL && 
        poGRW->pszColumn != NULL )
    {
        if( poGRW->pszWhere == NULL )
        {
            poStmt = poConnection->CreateStatement( CPLSPrintf(
                "SELECT T.%s.RASTERDATATABLE, T.%s.RASTERID FROM %s T",
                poGRW->pszColumn, poGRW->pszColumn, poGRW->pszTable ) );
        }
        else
        {
            poStmt = poConnection->CreateStatement( CPLSPrintf(
                "SELECT T.%s.RASTERDATATABLE, T.%s.RASTERID FROM %s T\n"
                "WHERE  %s",
                poGRW->pszColumn, poGRW->pszColumn, poGRW->pszTable,
                poGRW->pszWhere ) );
        }

        poStmt->Define( szDataTable );
        poStmt->Define( szRasterId );

        papszToken[3] = papszToken[7] = szDataTable;
        papszToken[4] = ",";
        papszToken[5] = papszToken[9] = szRasterId;
        papszToken[6] = "DataTable:";
        papszToken[8] = "RasterId:";
    }

    if( poStmt->Execute() == false )
    {
        return;
    }

    //  -----------------------------------------------------------
    //  Format subdataset list
    //  -----------------------------------------------------------

    int nCount = 1;

    while( poStmt->Fetch() )
    {
        papszSubdatasets = CSLSetNameValue( papszSubdatasets, 
            CPLSPrintf( "SUBDATASET_%d_NAME", nCount ),
            CPLSPrintf( "georaster:%s,%s,%s,%s%s%s",
            papszToken[0], papszToken[1], papszToken[2], papszToken[3], 
            papszToken[4], papszToken[5] ) );

        papszSubdatasets = CSLSetNameValue( papszSubdatasets, 
            CPLSPrintf( "SUBDATASET_%d_DESC", nCount ),
            CPLSPrintf( "%s%s %s%s", 
            papszToken[6], papszToken[7], papszToken[8], papszToken[9] ) );

        nCount++;
    }
}

//  ---------------------------------------------------------------------------
//                                                                    SetGCPs()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterDataset::SetGCPs( int, const GDAL_GCP *, const char * )
{
    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                           GetGCPProjection()
//  ---------------------------------------------------------------------------

const char* GeoRasterDataset::GetGCPProjection()

{
    if( nGCPCount > 0 )
        return pszProjection;
    else
        return "";
}

/*****************************************************************************/
/*                          GDALRegister_GEOR                                */
/*****************************************************************************/

void CPL_DLL GDALRegister_GEOR()
{
    GeoRasterDriver* poDriver;

    if( GDALGetDriverByName( "GeoRaster" ) == NULL )
    {
        poDriver = new GeoRasterDriver();

        poDriver->SetDescription(  "GeoRaster" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "Oracle Spatial GeoRaster" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, "frmt_georaster.html" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONDATATYPES, 
                                   "Byte UInt16 Int16 UInt32 Int32 Float32 "
                                   "Float64 CFloat32 CFloat64" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONOPTIONLIST, 
"<CreationOptionList>"
"  <Option name='DESCRIPTION' type='string' description='Table Description'/>"
"  <Option name='INSERT'      type='string' description='Column Values'/>"
"  <Option name='BLOCKXSIZE'  type='int'    description='Column Block Size'/>"
"  <Option name='BLOCKYSIZE'  type='int'    description='Row Block Size'/>"
"  <Option name='BLOCKBSIZE'  type='int'    description='Band Block Size'/>"
"  <Option name='INTERLEAVE'  type='string-select' default='BAND'>"
"       <Value>BAND</Value>"
"       <Value>PIXEL</Value>"
"       <Value>LINE</Value>"
"       <Value>BSQ</Value>"
"       <Value>BIP</Value>"
"       <Value>BIL</Value>"
"   </Option>"
"  <Option name='SRID'        type='int'    description='Overwrite EPSG code'/>"
"</CreationOptionList>" );
        poDriver->pfnOpen       = GeoRasterDataset::Open;
        poDriver->pfnCreate     = GeoRasterDataset::Create;
        poDriver->pfnCreateCopy = GeoRasterDataset::CreateCopy;
        poDriver->pfnIdentify   = GeoRasterDataset::Identify;
        poDriver->pfnDelete     = GeoRasterDataset::Delete;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}
