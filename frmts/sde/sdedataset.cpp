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
/*                           IssueSDEError()                            */
/* from ogrsdedatasource.cpp											*/
/************************************************************************/

void IssueSDEError( int nErrorCode, 
                    const char *pszFunction )

{
    char szErrorMsg[SE_MAX_MESSAGE_LENGTH+1];

    if( pszFunction == NULL )
        pszFunction = "SDERASTER";

    SE_error_get_string( nErrorCode, szErrorMsg );

    CPLError( CE_Failure, CPLE_AppDefined, 
              "%s: %d/%s", 
              pszFunction, nErrorCode, szErrorMsg );
}

/************************************************************************/
/*                           IssueSDEExtendedError()                    */
/************************************************************************/
void IssueSDEExtendedError ( int nErrorCode,
                           const char *pszFunction,
                           SE_CONNECTION* connection,
                           SE_STREAM* stream) {
 
    SE_ERROR err;
    char szErrorMsg[SE_MAX_MESSAGE_LENGTH+1];

    if( pszFunction == NULL )
        pszFunction = "SDERASTER";

    SE_error_get_string( nErrorCode, szErrorMsg );

        
    if (connection)
        SE_connection_get_ext_error( *connection, &err );
    if (stream)
        SE_stream_get_ext_error( *stream, &err );
    
    if (connection || stream) {
        CPLError ( CE_Failure, CPLE_AppDefined,
                   "%s: %d/%s ---- %s ---- %s ---- %s ---- %s",
                   pszFunction, nErrorCode, szErrorMsg, 
                   err.sde_error, err.ext_error,
                   err.err_msg1, err.err_msg2 );           

    } else {
        CPLError ( CE_Failure, CPLE_AppDefined,
           "%s: %d/%s",
           pszFunction, nErrorCode, szErrorMsg );
    } 
}


/************************************************************************/
/*                           SDERasterBand()                            */
/************************************************************************/

SDERasterBand::SDERasterBand( SDEDataset *poDS, int nBand )

{
    this->poDS = poDS;
    this->nBand = nBand;
    
    eDataType = GDT_Float32;

    nBlockXSize = poDS->GetRasterXSize();
    nBlockYSize = 1;
}

/************************************************************************/
/*                             IReadBlock()                             */
/************************************************************************/

CPLErr SDERasterBand::IReadBlock( int nBlockXOff, int nBlockYOff,
                                  void * pImage )

{
    SDEDataset *poGDS = (SDEDataset *) poDS;
    char  *pszRecord;
    int   nRecordSize = nBlockXSize*5 + 9 + 2;
    int   i;



    return CE_None;
}

/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr SDEDataset::GetGeoTransform( double * padfTransform )

{
//    double  dfLLLat, dfLLLong, dfURLat, dfURLong;
//
//    dfLLLat = JDEMGetAngle( (char *) abyHeader + 29 );
//    dfLLLong = JDEMGetAngle( (char *) abyHeader + 36 );
//    dfURLat = JDEMGetAngle( (char *) abyHeader + 43 );
//    dfURLong = JDEMGetAngle( (char *) abyHeader + 50 );
//    
//    padfTransform[0] = dfLLLong;
//    padfTransform[3] = dfURLat;
//    padfTransform[1] = (dfURLong - dfLLLong) / GetRasterXSize();
//    padfTransform[2] = 0.0;
//        
//    padfTransform[4] = 0.0;
//    padfTransform[5] = -1 * (dfURLat - dfLLLat) / GetRasterYSize();


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
    
    if (!pohSDERasterColumn){
        CPLError ( CE_Failure, CPLE_AppDefined,
                   "Raster Column not defined");        
        return ("");   
    }
    
    nSDEErr = SE_rascolinfo_get_coordref(pohSDERasterColumn, coordref);

    if (nSDEErr == SE_NO_COORDREF) {
        return ("");
    }
    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_rascolinfo_get_coordref" );
      //  return FALSE;
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
    char* pszWKT;
    poSRS->exportToWkt(&pszWKT);
    poSRS->Release();
    return CPLStrdup(pszWKT);
}

/************************************************************************/
/*                                SDEDataset()                          */
/************************************************************************/

SDEDataset::SDEDataset( SE_CONNECTION* connection )

{
    hConnection         = connection;
    nSubDataCount       = 0;
    pszLayerName        = NULL;
    pszColumnName       = NULL;
    paohSDERasterColumns  = NULL;
    pohSDERasterColumn  = NULL;
    SE_rascolinfo_create(&pohSDERasterColumn);

}



/************************************************************************/
/*                            ~SDEDataset()                             */
/************************************************************************/

SDEDataset::~SDEDataset()

{
//    if (paohSDERasterColumns != NULL)
//        SE_rastercolumn_free_info_list(nSubDataCount,
//                                   paohSDERasterColumns);
    if (pohSDERasterColumn)
        SE_rascolinfo_free(pohSDERasterColumn);
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
/*      Try to establish connection.                                    */
/* -------------------------------------------------------------------- */
    int 	    nSDEErr;
    SE_ERROR    sSDEErrorInfo;

	SE_CONNECTION connection = NULL;
		
    nSDEErr = SE_connection_create( papszTokens[0], 
                                    papszTokens[1], 
                                    papszTokens[2], 
                                    papszTokens[3],
                                    papszTokens[4],
                                    &sSDEErrorInfo, &connection );

    if( nSDEErr != SE_SUCCESS )
    {
        IssueSDEError( nSDEErr, "SE_connection_create" );
        return FALSE;
    }


/* -------------------------------------------------------------------- */
/*      Set unprotected concurrency policy, suitable for single         */
/*      threaded access.                                                */
/* -------------------------------------------------------------------- */
    nSDEErr = SE_connection_set_concurrency( connection,
                                             SE_UNPROTECTED_POLICY);

    if( nSDEErr != SE_SUCCESS) {
        IssueSDEError( nSDEErr, NULL );
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */

    SDEDataset *poDS;

    poDS = new SDEDataset(&connection);

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
        CPLDebug( "SDERASTER", "'%s' raster layer specified... "\
                               "using it directly with '%s' as the raster column name.", 
                  poDS->pszLayerName,
                  poDS->pszColumnName);
        nSDEErr = SE_rastercolumn_get_info_by_name(*(poDS->hConnection), 
                                                    poDS->pszLayerName, 
                                                    poDS->pszColumnName, 
                                                    poDS->pohSDERasterColumn);
        if( nSDEErr != SE_SUCCESS )
        {
            IssueSDEError( nSDEErr, "SE_rastercolumn_get_info_by_name" );
            return FALSE;
        }

    } else {
//
//    	SE_RASCOLINFO* columns;
//      	nSDEErr = SE_rastercolumn_get_info_list(connection, 
//      	                                        &(poDS->paohSDERasterColumns), 
//      	                                        &(poDS->nSubDataCount));
//        if( nSDEErr != SE_SUCCESS )
//        {
//            IssueSDEError( nSDEErr, "SE_rascolinfo_get_info_list" );
//            return FALSE;
//        }
//
//        CPLDebug( "SDERASTER", "No layername specified, %d subdatasets available.", 
//                  poDS->nSubDataCount);
//                  
//
//        for (int i = 0; i < poDS->nSubDataCount; i++) {
//
//              char         szTableName[SE_QUALIFIED_TABLE_NAME+1];
//              char         szColumnName[SE_MAX_COLUMN_LEN+1];
//            nSDEErr = SE_rascolinfo_get_raster_column (poDS->paohSDERasterColumns[i], 
//                                                       szTableName, 
//                                                       szColumnName); 
//            CPLDebug("SDERASTER", "Layer '%s' with column '%s' found.", szTableName, szColumnName);
//
//            if( nSDEErr != SE_SUCCESS )
//            {
//                IssueSDEError( nSDEErr, "SE_rascolinfo_get_raster_column" );
//                return FALSE;
//            }
//        }
//
    }
    
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
