/******************************************************************************
 * $Id$
 *
 * Project:  ESRI ArcSDE Raster reader
 * Purpose:  Dataset implementaion for ESRI ArcSDE Rasters
 * Author:   Howard Butler, hobu@hobu.net
 *
 ******************************************************************************
 * Copyright (c) 2007, Howard Butler <hobu@hobu.net>
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

#include "sdedataset.h"


/************************************************************************/
/*                          GetRastercount()                            */
/************************************************************************/

int SDEDataset::GetRasterCount( void )

{
    return nBands;
}    

/************************************************************************/
/*                          GetRasterXSize()                            */
/************************************************************************/

int SDEDataset::GetRasterXSize( void )

{
    return nRasterXSize;
}  

/************************************************************************/
/*                          GetRasterYSize()                            */
/************************************************************************/

int SDEDataset::GetRasterYSize( void )

{
    return nRasterYSize;
}


/************************************************************************/
/*                          ComputeRasterInfo()                         */
/************************************************************************/
CPLErr SDEDataset::ComputeRasterInfo() {
    long nSDEErr;
    SE_RASTERINFO raster;
    
    nSDEErr = SE_rasterinfo_create(&raster);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rasterinfo_create" );
        return CE_Fatal;
    }
    
    long nRasterColumnId = 0;

    nSDEErr = SE_rascolinfo_get_id(hRasterColumn, &nRasterColumnId);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rascolinfo_get_id" );
        return CE_Fatal;
    }        

    nSDEErr = SE_raster_get_info_by_id(hConnection, nRasterColumnId, 1, raster);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rascolinfo_get_id" );
        return CE_Fatal;
    }
    nSDEErr = SE_raster_get_bands(hConnection, raster, &paohSDERasterBands, (long*)&nBands);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_raster_get_bands" );
        return CE_Fatal;
    }
    
    SE_RASBANDINFO band;
    
    // grab our other stuff from the first band and hope for the best
    band = paohSDERasterBands[0];
    
    
    nSDEErr = SE_rasbandinfo_get_band_size(band, (long*)&nRasterXSize, (long*)&nRasterYSize);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rasbandinfo_get_band_size" );
        return CE_Fatal;
    }
    
    SE_ENVELOPE extent;
    nSDEErr = SE_rasbandinfo_get_extent(band, &extent);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rasbandinfo_get_extent" );
        return CE_Fatal;
    }
    dfMinX = extent.minx;
    dfMinY = extent.miny;
    dfMaxX = extent.maxx;
    dfMaxY = extent.maxy;
    
    
    nSDEErr = SE_rasterattr_create(&hAttributes, false);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rasterattr_create" );
        return CE_Fatal;
    }
    
    // Grab the pointer for our member variable

    nSDEErr = SE_stream_create(hConnection, &hStream);
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_stream_create" );
        return CE_Fatal;
    }

    
    for (int i=0; i < nBands; i++) {
        SetBand( i+1, new SDERasterBand( this, i+1, -1, &(paohSDERasterBands[i]) ));
    }

    GDALRasterBand* b = GetRasterBand(1);
    
    eDataType = b->GetRasterDataType();
    

    
    SE_rasterinfo_free(raster);


    return CE_None;
}


/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr SDEDataset::GetGeoTransform( double * padfTransform )

{
    
    if (dfMinX == 0.0 && dfMinY == 0.0 && dfMaxX == 0.0 && dfMaxY == 0.0)
        return CE_Fatal;
 
    padfTransform[0] = dfMinX - 0.5*(dfMaxX - dfMinX) / (GetRasterXSize()-1);
    padfTransform[3] = dfMaxY + 0.5*(dfMaxY - dfMinY) / (GetRasterYSize()-1);
    padfTransform[1] = (dfMaxX - dfMinX) / (GetRasterXSize()-1);
    padfTransform[2] = 0.0;
        
    padfTransform[4] = 0.0;
    padfTransform[5] = -1 * (dfMaxY - dfMinY) / (GetRasterYSize()-1);
    
    return CE_None;
}

/************************************************************************/
/*                          GetProjectionRef()                          */
/************************************************************************/

const char *SDEDataset::GetProjectionRef()

{
    long nSDEErr;
    SE_COORDREF coordref;
    nSDEErr = SE_coordref_create(&coordref);

    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_coordref_create" );
        return FALSE;
    }
    
    if (!hRasterColumn){
        CPLError ( CE_Failure, CPLE_AppDefined,
                   "Raster Column not defined");        
        return ("");   
    }
    
    nSDEErr = SE_rascolinfo_get_coordref(hRasterColumn, coordref);

    if (nSDEErr == SE_NO_COORDREF) {
        return ("");
    }
    
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rascolinfo_get_coordref" );
    }    
    
    char szWKT[SE_MAX_SPATIALREF_SRTEXT_LEN];
    nSDEErr = SE_coordref_get_description(coordref, szWKT);
    if (nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_coordref_get_description");
    }
    SE_coordref_free(coordref);

    OGRSpatialReference *poSRS;
    poSRS = new OGRSpatialReference(szWKT);
    poSRS->morphFromESRI();

    poSRS->exportToWkt(&pszWKT);
    poSRS->Release();
    
    return pszWKT;
}

/************************************************************************/
/*                                SDEDataset()                          */
/************************************************************************/

SDEDataset::SDEDataset(  )

{
    hConnection         = NULL;
    nSubDataCount       = 0;
    pszLayerName        = NULL;
    hAttributes         = NULL;
    pszColumnName       = NULL;
    paohSDERasterColumns  = NULL;
    paohSDERasterBands  = NULL;
    hStream             = NULL;
    hRasterColumn       = NULL;
    nBands              = 0;
    nRasterXSize        = 0;
    nRasterYSize        = 0;
    
    dfMinX              = 0.0;
    dfMinY              = 0.0;
    dfMaxX              = 0.0;
    dfMaxY              = 0.0;
    SE_rascolinfo_create(&hRasterColumn);

}



/************************************************************************/
/*                            ~SDEDataset()                             */
/************************************************************************/

SDEDataset::~SDEDataset()

{

    if (paohSDERasterBands)
        SE_rasterband_free_info_list(nBands, paohSDERasterBands);

    if (hRasterColumn)
        SE_rascolinfo_free(hRasterColumn);
    
    if (hStream)
        SE_stream_free(hStream);
        
    if (hAttributes)
        SE_rasterattr_free(hAttributes);

    if (hConnection)
        SE_connection_free(hConnection);
        
    if (pszWKT)
        CPLFree(pszWKT);
    
    if (pszLayerName)
        CPLFree(pszLayerName);

    if (pszColumnName)
        CPLFree(pszColumnName);
}


/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *SDEDataset::Open( GDALOpenInfo * poOpenInfo )

{
    


/* -------------------------------------------------------------------- */
/*      If we aren't prefixed with SDE: then ignore this datasource.    */
/* -------------------------------------------------------------------- */
    if( !EQUALN(poOpenInfo->pszFilename,"SDE:",4) )
        return FALSE;

/* -------------------------------------------------------------------- */
/*      Parse arguments on comma.  We expect (layer is optional):       */
/*        SDE:server,instance,database,username,password,layer          */
/* -------------------------------------------------------------------- */
    char **papszTokens = CSLTokenizeStringComplex( poOpenInfo->pszFilename+4, ",",
                                                   TRUE, TRUE );
    CPLDebug( "SDERASTER", "Open(\"%s\") revealed %d tokens.", poOpenInfo->pszFilename,
              CSLCount( papszTokens ) );


    if( CSLCount( papszTokens ) < 5 || CSLCount( papszTokens ) > 6 )
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "SDE connect string had wrong number of arguments.\n"
                  "Expected 'SDE:server,instance,database,username,password,layer'\n"
                  "The layer name value is optional.\n"
                  "Got '%s'", 
                  poOpenInfo->pszFilename );
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */

    SDEDataset *poDS;

    poDS = new SDEDataset();
/* -------------------------------------------------------------------- */
/*      Try to establish connection.                                    */
/* -------------------------------------------------------------------- */
    int         nSDEErr;
    SE_ERROR    hSDEErrorInfo;
    nSDEErr = SE_connection_create( papszTokens[0], 
                                    papszTokens[1], 
                                    papszTokens[2], 
                                    papszTokens[3],
                                    papszTokens[4],
                                    &(hSDEErrorInfo), &(poDS->hConnection) );

    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_connection_create" );
        return FALSE;
    }


/* -------------------------------------------------------------------- */
/*      Set unprotected concurrency policy, suitable for single         */
/*      threaded access.                                                */
/* -------------------------------------------------------------------- */
    nSDEErr = SE_connection_set_concurrency( poDS->hConnection,
                                             SE_UNPROTECTED_POLICY);

    if( nSDEErr != SE_SUCCESS) {
        IssueSDEError( nSDEErr, NULL );
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      If we were given a layer name, use that directly, otherwise     */
/*      query for subdatasets.                                          */
/* -------------------------------------------------------------------- */


    if (CSLCount( papszTokens ) == 6 ) {
//
        poDS->pszLayerName = CPLStrdup( papszTokens[5] );
//        
//        // FIXME this needs to be a configuration option or allow it to
//        // come in via the arguments
        poDS->pszColumnName = CPLStrdup( "RASTER" );
        
        nSDEErr =   SE_rascolinfo_create  (&(poDS->hRasterColumn));
        if( nSDEErr != SE_SUCCESS )
        {
            IssueSDEError( nSDEErr, "SE_rastercolumn_create" );
            return FALSE;
        }
        CPLDebug( "SDERASTER", "'%s' raster layer specified... "\
                               "using it directly with '%s' as the raster column name.", 
                  poDS->pszLayerName,
                  poDS->pszColumnName);
        nSDEErr = SE_rastercolumn_get_info_by_name(poDS->hConnection, 
                                                    poDS->pszLayerName, 
                                                    poDS->pszColumnName, 
                                                    poDS->hRasterColumn);
        if( nSDEErr != SE_SUCCESS )
        {
            IssueSDEError( nSDEErr, "SE_rastercolumn_get_info_by_name" );
            return FALSE;
        }
        poDS->ComputeRasterInfo();



    } else {
 
        nSDEErr = SE_rastercolumn_get_info_list(poDS->hConnection, 
                                                &(poDS->paohSDERasterColumns), 
                                                &(poDS->nSubDataCount));
        if( nSDEErr != SE_SUCCESS )
        {
            IssueSDEError( nSDEErr, "SE_rascolinfo_get_info_list" );
            return FALSE;
        }

        CPLDebug( "SDERASTER", "No layername specified, %d subdatasets available.", 
                  poDS->nSubDataCount);
                  

        for (int i = 0; i < poDS->nSubDataCount; i++) {

              char         szTableName[SE_QUALIFIED_TABLE_NAME+1];
              char         szColumnName[SE_MAX_COLUMN_LEN+1];
            nSDEErr = SE_rascolinfo_get_raster_column (poDS->paohSDERasterColumns[i], 
                                                       szTableName, 
                                                       szColumnName); 
            CPLDebug("SDERASTER", "Layer '%s' with column '%s' found.", szTableName, szColumnName);

            if( nSDEErr != SE_SUCCESS )
            {
                IssueSDEError( nSDEErr, "SE_rascolinfo_get_raster_column" );
                return FALSE;
            }
        }

    return FALSE;
    }
    CSLDestroy( papszTokens);
    return( poDS );
}

/************************************************************************/
/*                          GDALRegister_SDE()                          */
/************************************************************************/

void GDALRegister_SDE()

{
    GDALDriver  *poDriver;

    if( GDALGetDriverByName( "SDE" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "SDE" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "ESRI ArcSDE" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, 
                                   "frmt_various.html#SDE" );
       // poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "mem" );

        poDriver->pfnOpen = SDEDataset::Open;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}
