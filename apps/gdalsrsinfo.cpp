/******************************************************************************
 * $Id$
 *
 * Project:  GDAL Utilities
 * Purpose:  Commandline application to list info about a given CRS.
 *           Outputs a number of formats (WKT, PROJ.4, etc.).
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *           Etienne Tourigny, etourigny.dev-at-gmail-dot-com       
 *
 * ****************************************************************************
 * Copyright (c) 1998, Frank Warmerdam
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

#include "gdal_priv.h"
#include "cpl_string.h"
#include "ogr_spatialref.h"
#include "ogr_api.h"
#include "ogrsf_frmts.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*                               Usage()                                */
/************************************************************************/

void Usage()

{
    printf( "\nUsage: gdalsrsinfo [options] srs_def\n"
            "\n"
            "srs_def may be the filename of a dataset supported by GDAL/OGR "
            "from which to extract SRS information\n"
            "OR any of the usual GDAL/OGR forms "
            "(complete WKT, PROJ.4, EPSG:n or a file containing the SRS)\n"
            "\n"          
            "Options: \n"
            "   [--help-general] [-h]  Show help and exit\n"
            "   [-p]                   Pretty-print where applicable (e.g. WKT)\n"
            "   [-V]                   Validate SRS\n"
            "   [-o out_type]          Output type, default all\n"
            "                          {all,proj4,wkt,wkt_simple,wkt_old,wkt_esri,mapinfo,xml}\n\n" ); 
    exit( 1 );
}

/************************************************************************/
/*                                main()                                */
/************************************************************************/

int main( int argc, char ** argv ) 

{
    int            i;
    int            bGotSRS = FALSE;
    int            bPretty = FALSE;
    int            bOutputAll = FALSE;
    int            bValidate = FALSE;

    const char     *pszInput = NULL;
    const char     *pszOutputType = "all";
    char           *pszOutput = NULL;

    OGRSpatialReference  oSRS;

    VSILFILE      *fp = NULL;
    GDALDataset	  *poGDALDS = NULL; 
    OGRDataSource *poOGRDS = NULL;
    OGRLayer      *poLayer = NULL;
    char           *pszProjection = NULL;
    int            bDebug = FALSE;
    CPLErrorHandler oErrorHandler = NULL;
    int bIsFile = FALSE;

    /* Check strict compilation and runtime library version as we use C++ API */
    if (! GDAL_CHECK_VERSION(argv[0]))
        exit(1);

    /* Must process GDAL_SKIP before GDALAllRegister(), but we can't call */
    /* GDALGeneralCmdLineProcessor before it needs the drivers to be registered */
    /* for the --format or --formats options */
    for( i = 1; i < argc; i++ )
    {
        if( EQUAL(argv[i],"--config") && i + 2 < argc && EQUAL(argv[i + 1], "GDAL_SKIP") )
        {
            CPLSetConfigOption( argv[i+1], argv[i+2] );

            i += 2;
        }
    }

    argc = GDALGeneralCmdLineProcessor( argc, &argv, 0 );
    if( argc < 1 )
        exit( -argc );

/* -------------------------------------------------------------------- */
/*      Parse arguments.                                                */
/* -------------------------------------------------------------------- */
    for( i = 1; i < argc; i++ )
    {
        CPLDebug( "gdalsrsinfo", "got arg #%d : [%s]", i, argv[i] );

        if( EQUAL(argv[i], "--utility_version") )
        {
            printf("%s was compiled against GDAL %s and is running against GDAL %s\n",
                   argv[0], GDAL_RELEASE_NAME, GDALVersionInfo("RELEASE_NAME"));
            return 0;
        }
        else if( EQUAL(argv[i], "-h") )
            Usage();
        else if( EQUAL(argv[i], "-o") && i < argc - 1)
            pszOutputType = argv[++i];
        else if( EQUAL(argv[i], "-p") )
            bPretty = TRUE;
        else if( EQUAL(argv[i], "-V") )
            bValidate = TRUE;
        else if( argv[i][0] == '-' )
        {
            CSLDestroy( argv );
            Usage();
        }
        else  
            pszInput = argv[i];
    }

    if ( pszInput == NULL ) {
        CSLDestroy( argv );
        Usage();
    }

    /* Register drivers */
    GDALAllRegister();
    OGRRegisterAll();

    /* Search for SRS */
      
    /* temporarily supress error messages we may get from xOpen() */
    bDebug = CSLTestBoolean(CPLGetConfigOption("CPL_DEBUG", "OFF"));
    if ( ! bDebug )
        oErrorHandler = CPLSetErrorHandler ( CPLQuietErrorHandler );
    
    /* If argument is a file, try to open it with GDALOpen() and get the projection */
    fp = VSIFOpenL( pszInput, "r" );
    if ( fp )  {
        
        bIsFile = TRUE;
        VSIFCloseL( fp );
        
        /* try to open with GDAL */
        CPLDebug( "gdalsrsinfo", "trying to open with GDAL" );
        poGDALDS =  (GDALDataset *) GDALOpen( pszInput, GA_ReadOnly );
        if ( poGDALDS != NULL && poGDALDS->GetProjectionRef( ) != NULL ) {
            pszProjection = (char *) poGDALDS->GetProjectionRef( );
            if( oSRS.importFromWkt( &pszProjection ) == CE_None ) {
                CPLDebug( "gdalsrsinfo", "got SRS from GDAL" );
                bGotSRS = TRUE;
            }
            GDALClose( (GDALDatasetH) poGDALDS );
        }
        if ( ! bGotSRS ) 
            CPLDebug( "gdalsrsinfo", "did not open with GDAL" );
        
        /* if unsuccessful, try to open with OGR */
        if ( ! bGotSRS ) {
            CPLDebug( "gdalsrsinfo", "trying to open with OGR" );
            poOGRDS = OGRSFDriverRegistrar::Open( pszInput, FALSE, NULL );
            if( poOGRDS != NULL ) {
                poLayer = poOGRDS->GetLayer( 0 );
                if ( poLayer != NULL ) {
                    OGRSpatialReference *poSRS = poLayer->GetSpatialRef( );
                    if ( poSRS != NULL ) {
                        CPLDebug( "gdalsrsinfo", "got SRS from OGR" );
                        bGotSRS = TRUE;
                        OGRSpatialReference* poSRSClone = poSRS->Clone();
                        oSRS = *poSRSClone;
                        OGRSpatialReference::DestroySpatialReference( poSRSClone );
                    }
                }
                OGRDataSource::DestroyDataSource( poOGRDS );
                poOGRDS = NULL;
            } 
            if ( ! bGotSRS ) 
                CPLDebug( "gdalsrsinfo", "did not open with OGR" );
            
            /* OGR_DS_Destroy( hOGRDS ); */
        }
        
    }
    
    /* If didn't get the projection from the file, try OSRSetFromUserInput() */
    /* File might not be a dataset, but contain projection info (e.g. .prf files) */
    if ( ! bGotSRS ) {
        CPLDebug( "gdalsrsinfo", 
                  "trying to get SRS from user input [%s]", pszInput );
        if( oSRS.SetFromUserInput( pszInput ) != OGRERR_NONE ) {
            CPLDebug( "gdalsrsinfo", "did not get SRS from user input" );
        }
        else {
            CPLDebug( "gdalsrsinfo", "got SRS from user input" );
            bGotSRS = TRUE;
        }
    }
    
    /* restore error messages */
    if ( ! bDebug )
        CPLSetErrorHandler ( oErrorHandler );	

    CPLDebug( "gdalsrsinfo", 
              "bGotSRS: %d bValidate: %d pszOutputType: %s bPretty: %d",
              bGotSRS, bValidate, pszOutputType, bPretty  );

    /* Make sure we got a SRS */
    if ( ! bGotSRS ) {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "ERROR - failed to load SRS definition from %s",
                  pszInput );
    }

    else {

        /* Validate - not well tested!*/
        if ( bValidate ) {
            OGRErr eErr = oSRS.Validate( );
            if ( eErr != OGRERR_NONE ) {
                printf( "\nValidate Fails" );
                if ( eErr == OGRERR_CORRUPT_DATA )
                    printf( " - SRS is not well formed");
                else if ( eErr == OGRERR_UNSUPPORTED_SRS )
                    printf(" - contains non-standard PROJECTION[] values");
                printf("\n");
            }
            else
                printf( "\nValidate Succeeds\n" );
        }
        
        /* Output */
        if ( EQUAL("all", pszOutputType ) ) {
            bOutputAll = TRUE;
            bPretty = TRUE;
        }
        
        printf("\n");
        
        if ( bOutputAll || EQUAL("proj4", pszOutputType ) ) {
            if ( bOutputAll ) printf("PROJ.4 : ");
            oSRS.exportToProj4( &pszOutput );
            printf("\'%s\'\n\n",pszOutput);
            CPLFree( pszOutput );
        }
        
        if ( bOutputAll || EQUAL("wkt", pszOutputType ) ) {
            if ( bOutputAll ) printf("OGC WKT :\n");
            if ( bPretty ) 
                oSRS.exportToPrettyWkt( &pszOutput, FALSE );
            else
                oSRS.exportToWkt( &pszOutput );
            printf("%s\n\n",pszOutput);
            CPLFree( pszOutput );
        }
        
        if ( bOutputAll || EQUAL("wkt_simple", pszOutputType ) ) {
            if ( bOutputAll ) printf("OGC WKT (simple) :\n");
            oSRS.exportToPrettyWkt( &pszOutput, TRUE );
            printf("%s\n\n",pszOutput);
            CPLFree( pszOutput );
        }
        
        if ( EQUAL("wkt_old", pszOutputType ) ) {
            if (  bOutputAll ) printf("OGC WKT (old) :\n");
            oSRS.StripCTParms( );
            if ( bPretty ) 
                oSRS.exportToPrettyWkt( &pszOutput, FALSE );
            else
                oSRS.exportToWkt( &pszOutput );
            printf("%s\n\n",pszOutput);
            CPLFree( pszOutput );
        }
        
        if ( bOutputAll || EQUAL("wkt_esri", pszOutputType ) ) {
            if ( bOutputAll ) printf("ESRI WKT :\n");
            OGRSpatialReference *poSRS = oSRS.Clone();
            poSRS->morphToESRI( );
            if ( bPretty ) 
                poSRS->exportToPrettyWkt( &pszOutput, FALSE );
            else
                poSRS->exportToWkt( &pszOutput );
            printf("%s\n\n",pszOutput);
            CPLFree( pszOutput );
            OGRSpatialReference::DestroySpatialReference( poSRS );
        }
        
        /* mapinfo and xml are not output with "all" */
        if ( EQUAL("mapinfo", pszOutputType ) ) {
            if ( bOutputAll ) printf("MAPINFO : ");
            oSRS.exportToMICoordSys( &pszOutput );
            printf("\'%s\'\n\n",pszOutput);
            CPLFree( pszOutput );
        }
        
        if ( EQUAL("xml", pszOutputType ) ) {
            if ( bOutputAll ) printf("XML :\n");
            oSRS.exportToXML( &pszOutput, NULL );
            printf("%s\n\n",pszOutput);
            CPLFree( pszOutput );
        }

    }

    /* cleanup anything left */
    GDALDestroyDriverManager();
    OGRCleanupAll();
    CSLDestroy( argv );

    return 0;

}
