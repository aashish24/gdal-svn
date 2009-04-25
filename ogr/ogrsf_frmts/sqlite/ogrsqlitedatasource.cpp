/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements OGRSQLiteDataSource class.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2003, Frank Warmerdam <warmerdam@pobox.com>
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

#include "ogr_sqlite.h"
#include "cpl_conv.h"
#include "cpl_string.h"

CPL_CVSID("$Id$");
/************************************************************************/
/*                        OGRSQLiteDataSource()                         */
/************************************************************************/

OGRSQLiteDataSource::OGRSQLiteDataSource()

{
    pszName = NULL;
    papoLayers = NULL;
    nLayers = 0;

    nSoftTransactionLevel = 0;

    nKnownSRID = 0;
    panSRID = NULL;
    papoSRS = NULL;

    bHaveGeometryColumns = FALSE;
    bIsSpatiaLite = FALSE;
}

/************************************************************************/
/*                        ~OGRSQLiteDataSource()                        */
/************************************************************************/

OGRSQLiteDataSource::~OGRSQLiteDataSource()

{
    int         i;

    CPLFree( pszName );

    for( i = 0; i < nLayers; i++ )
        delete papoLayers[i];
    
    CPLFree( papoLayers );

    for( i = 0; i < nKnownSRID; i++ )
    {
        if( papoSRS[i] != NULL )
            papoSRS[i]->Release();
    }
    CPLFree( panSRID );
    CPLFree( papoSRS );

    if( hDB != NULL )
        sqlite3_close( hDB );
}

/************************************************************************/
/*                     SpatiaLiteToOGRGeomType()                        */
/*      Map SpatiaLite geometry format strings to corresponding         */
/*      OGR constants.                                                  */
/************************************************************************/

OGRwkbGeometryType
OGRSQLiteDataSource::SpatiaLiteToOGRGeomType( const char *pszGeomType )
{
    if ( EQUAL(pszGeomType, "POINT") )
        return wkbPoint;
    else if ( EQUAL(pszGeomType, "LINESTRING") )
        return wkbLineString;
    else if ( EQUAL(pszGeomType, "POLYGON") )
        return wkbPolygon;
    else if ( EQUAL(pszGeomType, "MULTIPOINT") )
        return wkbMultiPoint;
    else if ( EQUAL(pszGeomType, "MULTILINESTRING") )
        return wkbMultiLineString;
    else if ( EQUAL(pszGeomType, "MULTIPOLYGON") )
        return wkbMultiPolygon;
    else if ( EQUAL(pszGeomType, "GEOMETRYCOLLECTION") )
        return wkbGeometryCollection;
    else
        return wkbUnknown;
}

/************************************************************************/
/*                     OGRToSpatiaLiteGeomType()                        */
/*      Map OGR geometry format constants to corresponding              */
/*      SpatiaLite strings                                              */
/************************************************************************/

const char *
OGRSQLiteDataSource::OGRToSpatiaLiteGeomType( OGRwkbGeometryType eGeomType )
{
    switch ( wkbFlatten(eGeomType) )
    {
        case wkbUnknown:
            return "GEOMETRY";
        case wkbPoint:
            return "POINT";
        case wkbLineString:
            return "LINESTRING";
        case wkbPolygon:
            return "POLYGON";
        case wkbMultiPoint:
            return "MULTIPOINT";
        case wkbMultiLineString:
            return "MULTILINESTRING";
        case wkbMultiPolygon:
            return "MULTIPOLYGON";
        case wkbGeometryCollection:
            return "GEOMETRYCOLLECTION";
        default:
            return "";
    }
}

/************************************************************************/
/*                                Open()                                */
/*                                                                      */
/*      Note, the Open() will implicitly create the database if it      */
/*      does not already exist.                                         */
/************************************************************************/

int OGRSQLiteDataSource::Open( const char * pszNewName )

{
    CPLAssert( nLayers == 0 );

    pszName = CPLStrdup( pszNewName );

/* -------------------------------------------------------------------- */
/*      Try to open the sqlite database properly now.                   */
/* -------------------------------------------------------------------- */
    int rc;

    hDB = NULL;
    rc = sqlite3_open( pszNewName, &hDB );
    if( rc != SQLITE_OK )
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "sqlite3_open(%s) failed: %s", 
                  pszNewName, sqlite3_errmsg( hDB ) );
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      If we have a GEOMETRY_COLUMNS tables, initialize on the basis   */
/*      of that.                                                        */
/* -------------------------------------------------------------------- */
    char **papszResult;
    int nRowCount, iRow, nColCount;
    char *pszErrMsg;

    rc = sqlite3_get_table( 
        hDB,
        "SELECT f_table_name, f_geometry_column, geometry_type, coord_dimension, geometry_format, srid"
        " FROM geometry_columns",
        &papszResult, &nRowCount, &nColCount, &pszErrMsg );

    if( rc == SQLITE_OK )
    {
        bHaveGeometryColumns = TRUE;

        for( iRow = 0; iRow < nRowCount; iRow++ )
        {
            char **papszRow = papszResult + iRow * 6 + 6;
            OGRwkbGeometryType eGeomType = wkbUnknown;
            int nSRID = 0;

            eGeomType = (OGRwkbGeometryType) atoi(papszRow[2]);

            if( atoi(papszRow[3]) > 2 )
                eGeomType = (OGRwkbGeometryType) (((int)eGeomType) | wkb25DBit);

            if( papszRow[5] != NULL )
                nSRID = atoi(papszRow[5]);

            OpenTable( papszRow[0], papszRow[1], eGeomType, papszRow[4],
                       FetchSRS( nSRID ) );
        }

        sqlite3_free_table(papszResult);

        return TRUE;
    }

/* -------------------------------------------------------------------- */
/*      Otherwise we can deal with SpatiaLite database.                 */
/* -------------------------------------------------------------------- */
    sqlite3_free( pszErrMsg );
    rc = sqlite3_get_table( hDB,
                            "SELECT f_table_name, f_geometry_column, "
                            "type, coord_dimension, srid, "
                            "spatial_index_enabled FROM geometry_columns",
                            &papszResult, &nRowCount, 
                            &nColCount, &pszErrMsg );

    if ( rc == SQLITE_OK )
    {
        bIsSpatiaLite = TRUE;
        bHaveGeometryColumns = TRUE;

        for ( iRow = 0; iRow < nRowCount; iRow++ )
        {
            char **papszRow = papszResult + iRow * 6 + 6;
            OGRwkbGeometryType eGeomType;
            int nSRID = 0;

            eGeomType = SpatiaLiteToOGRGeomType(papszRow[2]);

            if( atoi(papszRow[3]) > 2 )
                eGeomType = (OGRwkbGeometryType) (((int)eGeomType) | wkb25DBit);

            if( papszRow[4] != NULL )
                nSRID = atoi(papszRow[4]);

            OpenTable( papszRow[0], papszRow[1], eGeomType, "SpatiaLite",
                       FetchSRS( nSRID ), nSRID );
        }

        sqlite3_free_table(papszResult);

        return TRUE;
    }

/* -------------------------------------------------------------------- */
/*      Otherwise our final resort is to return all tables and views    */
/*      as non-spatial tables.                                          */
/* -------------------------------------------------------------------- */
    sqlite3_free( pszErrMsg );
    rc = sqlite3_get_table( hDB,
                            "SELECT name FROM sqlite_master "
                            "WHERE type IN ('table','view') "
                            "UNION ALL "
                            "SELECT name FROM sqlite_temp_master "
                            "WHERE type IN ('table','view') "
                            "ORDER BY 1",
                            &papszResult, &nRowCount, 
                            &nColCount, &pszErrMsg );

    if( rc != SQLITE_OK )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Unable to fetch list of tables: %s", 
                  pszErrMsg );
        sqlite3_free( pszErrMsg );
        return FALSE;
    }
    
    for( iRow = 0; iRow < nRowCount; iRow++ )
        OpenTable( papszResult[iRow+1] );
    
    sqlite3_free_table(papszResult);

    return TRUE;
}

/************************************************************************/
/*                             OpenTable()                              */
/************************************************************************/

int OGRSQLiteDataSource::OpenTable( const char *pszNewName, 
                                    const char *pszGeomCol,
                                    OGRwkbGeometryType eGeomType,
                                    const char *pszGeomFormat,
                                    OGRSpatialReference *poSRS, int nSRID )

{
/* -------------------------------------------------------------------- */
/*      Create the layer object.                                        */
/* -------------------------------------------------------------------- */
    OGRSQLiteTableLayer  *poLayer;

    poLayer = new OGRSQLiteTableLayer( this );

    if( poLayer->Initialize( pszNewName, pszGeomCol, 
                             eGeomType, pszGeomFormat,
                             poSRS, nSRID ) )
    {
        delete poLayer;
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Add layer to data source layer list.                            */
/* -------------------------------------------------------------------- */
    papoLayers = (OGRSQLiteLayer **)
        CPLRealloc( papoLayers,  sizeof(OGRSQLiteLayer *) * (nLayers+1) );
    papoLayers[nLayers++] = poLayer;
    
    return TRUE;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRSQLiteDataSource::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,ODsCCreateLayer) )
        return TRUE;
    else
        return FALSE;
}

/************************************************************************/
/*                              GetLayer()                              */
/************************************************************************/

OGRLayer *OGRSQLiteDataSource::GetLayer( int iLayer )

{
    if( iLayer < 0 || iLayer >= nLayers )
        return NULL;
    else
        return papoLayers[iLayer];
}

/************************************************************************/
/*                             ExecuteSQL()                             */
/************************************************************************/

OGRLayer * OGRSQLiteDataSource::ExecuteSQL( const char *pszSQLCommand,
                                          OGRGeometry *poSpatialFilter,
                                          const char *pszDialect )

{
    if( pszDialect != NULL && EQUAL(pszDialect,"OGRSQL") )
        return OGRDataSource::ExecuteSQL( pszSQLCommand, 
                                          poSpatialFilter, 
                                          pszDialect );

/* -------------------------------------------------------------------- */
/*      Special case DELLAYER: command.                                 */
/* -------------------------------------------------------------------- */
    if( EQUALN(pszSQLCommand,"DELLAYER:",9) )
    {
        const char *pszLayerName = pszSQLCommand + 9;

        while( *pszLayerName == ' ' )
            pszLayerName++;

        DeleteLayer( pszLayerName );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Prepare statement.                                              */
/* -------------------------------------------------------------------- */
    int rc;
    sqlite3_stmt *hSQLStmt = NULL;

    rc = sqlite3_prepare( GetDB(), pszSQLCommand, strlen(pszSQLCommand),
                          &hSQLStmt, NULL );

    if( rc != SQLITE_OK )
    {
        if( hSQLStmt != NULL )
            sqlite3_finalize( hSQLStmt );

        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Do we get a resultset?                                          */
/* -------------------------------------------------------------------- */
    rc = sqlite3_step( hSQLStmt );
    if( rc != SQLITE_ROW )
    {
        sqlite3_finalize( hSQLStmt );
        return NULL;
    }
    
/* -------------------------------------------------------------------- */
/*      Create layer.                                                   */
/* -------------------------------------------------------------------- */
    OGRSQLiteSelectLayer *poLayer = NULL;
        
    poLayer = new OGRSQLiteSelectLayer( this, hSQLStmt );

    if( poSpatialFilter != NULL )
        poLayer->SetSpatialFilter( poSpatialFilter );
    
    return poLayer;
}

/************************************************************************/
/*                          ReleaseResultSet()                          */
/************************************************************************/

void OGRSQLiteDataSource::ReleaseResultSet( OGRLayer * poLayer )

{
    delete poLayer;
}

/************************************************************************/
/*                            CreateLayer()                             */
/************************************************************************/

OGRLayer *
OGRSQLiteDataSource::CreateLayer( const char * pszLayerNameIn,
                                  OGRSpatialReference *poSRS,
                                  OGRwkbGeometryType eType,
                                  char ** papszOptions )

{
    char                *pszLayerName;
    const char          *pszGeomFormat;

    if( CSLFetchBoolean(papszOptions,"LAUNDER",TRUE) )
        pszLayerName = LaunderName( pszLayerNameIn );
    else
        pszLayerName = CPLStrdup( pszLayerNameIn );
    
    pszGeomFormat = CSLFetchNameValue( papszOptions, "FORMAT" );
    if( pszGeomFormat == NULL )
    {
        if ( !bIsSpatiaLite )
            pszGeomFormat = "WKB";
        else
            pszGeomFormat = "SpatiaLite";
    }

    if( !EQUAL(pszGeomFormat,"WKT") 
        && !EQUAL(pszGeomFormat,"WKB")
        && !EQUAL(pszGeomFormat, "SpatiaLite") )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "FORMAT=%s not recognised or supported.", 
                  pszGeomFormat );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Do we already have this layer?  If so, should we blow it        */
/*      away?                                                           */
/* -------------------------------------------------------------------- */
    int iLayer;

    for( iLayer = 0; iLayer < nLayers; iLayer++ )
    {
        if( EQUAL(pszLayerName,papoLayers[iLayer]->GetLayerDefn()->GetName()) )
        {
            if( CSLFetchNameValue( papszOptions, "OVERWRITE" ) != NULL
                && !EQUAL(CSLFetchNameValue(papszOptions,"OVERWRITE"),"NO") )
            {
                DeleteLayer( pszLayerName );
            }
            else
            {
                CPLError( CE_Failure, CPLE_AppDefined, 
                          "Layer %s already exists, CreateLayer failed.\n"
                          "Use the layer creation option OVERWRITE=YES to "
                          "replace it.",
                          pszLayerName );
                CPLFree( pszLayerName );
                return NULL;
            }
        }
    }

/* -------------------------------------------------------------------- */
/*      Try to get the SRS Id of this spatial reference system,         */
/*      adding to the srs table if needed.                              */
/* -------------------------------------------------------------------- */
    int nSRSId = -1;

    if( poSRS != NULL )
        nSRSId = FetchSRSId( poSRS );

/* -------------------------------------------------------------------- */
/*      Create a basic table with the FID.  Also include the            */
/*      geometry if this is not a PostGIS enabled table.                */
/* -------------------------------------------------------------------- */
    int rc;
    char *pszErrMsg;
    const char *pszGeomCol = NULL;
    CPLString osCommand;

    if( eType == wkbNone )
        osCommand.Printf( 
            "CREATE TABLE '%s' ( OGC_FID INTEGER PRIMARY KEY )", 
            pszLayerName );
    else
    {
        if( EQUAL(pszGeomFormat,"WKT") )
        {
            pszGeomCol = "WKT_GEOMETRY";
            osCommand.Printf(
                "CREATE TABLE '%s' ( "
                "  OGC_FID INTEGER PRIMARY KEY,"
                "  %s VARCHAR )", 
                pszLayerName, pszGeomCol );
        }
        else
        {
            pszGeomCol = "GEOMETRY";
            osCommand.Printf(
                "CREATE TABLE '%s' ( "
                "  OGC_FID INTEGER PRIMARY KEY,"
                "  %s BLOB )", 
                pszLayerName, pszGeomCol );
        }
    }

#ifdef DEBUG
    CPLDebug( "OGR_SQLITE", "exec(%s)", osCommand.c_str() );
#endif

    rc = sqlite3_exec( hDB, osCommand, NULL, NULL, &pszErrMsg );
    if( rc != SQLITE_OK )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Unable to create table %s: %s",
                  pszLayerName, pszErrMsg );
        sqlite3_free( pszErrMsg );
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Eventually we should be adding this table to a table of         */
/*      "geometric layers", capturing the WKT projection, and           */
/*      perhaps some other housekeeping.                                */
/* -------------------------------------------------------------------- */
    if( bHaveGeometryColumns )
    {
        int nCoordDim;

        /* Sometimes there is an old cruft entry in the geometry_columns
         * table if things were not properly cleaned up before.  We make
         * an effort to clean out such cruft.
         */
        osCommand.Printf(
            "DELETE FROM geometry_columns WHERE f_table_name = '%s'", 
            pszLayerName );
                 
#ifdef DEBUG
        CPLDebug( "OGR_SQLITE", "exec(%s)", osCommand.c_str() );
#endif

        rc = sqlite3_exec( hDB, osCommand, NULL, NULL, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            sqlite3_free( pszErrMsg );
            return FALSE;
        }
        
        if( eType == wkbFlatten(eType) )
            nCoordDim = 2;
        else
            nCoordDim = 3;

        if( nSRSId > 0 )
        {
            if ( bIsSpatiaLite )
                osCommand.Printf(
                    "INSERT INTO geometry_columns "
                    "(f_table_name, f_geometry_column, type, "
                    "coord_dimension, srid, spatial_index_enabled) "
                    "VALUES ('%s','%s', '%s', %d, %d, 0)", 
                    pszLayerName, pszGeomCol, OGRToSpatiaLiteGeomType(eType),
                    nCoordDim, nSRSId );
            else
                osCommand.Printf(
                    "INSERT INTO geometry_columns "
                    "(f_table_name, f_geometry_column, geometry_format, "
                    "geometry_type, coord_dimension, srid) VALUES "
                    "('%s','%s','%s', %d, %d, %d)", 
                    pszLayerName, pszGeomCol, pszGeomFormat,
                    (int) wkbFlatten(eType), nCoordDim, nSRSId );
        }
        else
        {
            if ( bIsSpatiaLite )
                osCommand.Printf(
                    "INSERT INTO geometry_columns "
                    "(f_table_name, f_geometry_column, type, "
                    "coord_dimension, spatial_index_enabled) "
                    "VALUES ('%s','%s', '%s', %d, 0)", 
                    pszLayerName, pszGeomCol, OGRToSpatiaLiteGeomType(eType),
                    nCoordDim );
            else
                osCommand.Printf(
                    "INSERT INTO geometry_columns "
                    "(f_table_name, f_geometry_column, geometry_format, "
                    "geometry_type, coord_dimension) VALUES "
                    "('%s','%s','%s', %d, %d)", 
                    pszLayerName, pszGeomCol, pszGeomFormat,
                    (int) wkbFlatten(eType), nCoordDim );
        }

#ifdef DEBUG
        CPLDebug( "OGR_SQLITE", "exec(%s)", osCommand.c_str() );
#endif

        rc = sqlite3_exec( hDB, osCommand, NULL, NULL, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Unable to add %s table to geometry_columns:\n%s",
                      pszLayerName, pszErrMsg );
            sqlite3_free( pszErrMsg );
            return FALSE;
        }
    }

/* -------------------------------------------------------------------- */
/*      Create the layer object.                                        */
/* -------------------------------------------------------------------- */
    OGRSQLiteTableLayer     *poLayer;

    poLayer = new OGRSQLiteTableLayer( this );

    poLayer->Initialize( pszLayerName, pszGeomCol, eType, pszGeomFormat, 
                         FetchSRS(nSRSId), nSRSId );

    poLayer->SetLaunderFlag( CSLFetchBoolean(papszOptions,"LAUNDER",TRUE) );

/* -------------------------------------------------------------------- */
/*      Add layer to data source layer list.                            */
/* -------------------------------------------------------------------- */
    papoLayers = (OGRSQLiteLayer **)
        CPLRealloc( papoLayers,  sizeof(OGRSQLiteLayer *) * (nLayers+1) );
    
    papoLayers[nLayers++] = poLayer;

    CPLFree( pszLayerName );

    return poLayer;
}

/************************************************************************/
/*                            LaunderName()                             */
/************************************************************************/

char *OGRSQLiteDataSource::LaunderName( const char *pszSrcName )

{
    char    *pszSafeName = CPLStrdup( pszSrcName );
    int     i;

    for( i = 0; pszSafeName[i] != '\0'; i++ )
    {
        pszSafeName[i] = (char) tolower( pszSafeName[i] );
        if( pszSafeName[i] == '-' || pszSafeName[i] == '#' )
            pszSafeName[i] = '_';
    }

    return pszSafeName;
}

/************************************************************************/
/*                            DeleteLayer()                             */
/************************************************************************/

void OGRSQLiteDataSource::DeleteLayer( const char *pszLayerName )

{
    int iLayer;

/* -------------------------------------------------------------------- */
/*      Try to find layer.                                              */
/* -------------------------------------------------------------------- */
    for( iLayer = 0; iLayer < nLayers; iLayer++ )
    {
        if( EQUAL(pszLayerName,papoLayers[iLayer]->GetLayerDefn()->GetName()) )
            break;
    }

    if( iLayer == nLayers )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Attempt to delete layer '%s', but this layer is not known to OGR.", 
                  pszLayerName );
        return;
    }

/* -------------------------------------------------------------------- */
/*      Blow away our OGR structures related to the layer.  This is     */
/*      pretty dangerous if anything has a reference to this layer!     */
/* -------------------------------------------------------------------- */
    CPLDebug( "OGR_SQLITE", "DeleteLayer(%s)", pszLayerName );

    delete papoLayers[iLayer];
    memmove( papoLayers + iLayer, papoLayers + iLayer + 1, 
             sizeof(void *) * (nLayers - iLayer - 1) );
    nLayers--;

/* -------------------------------------------------------------------- */
/*      Remove from the database.                                       */
/* -------------------------------------------------------------------- */
    int rc;
    char *pszErrMsg;

    rc = sqlite3_exec( hDB, CPLSPrintf( "DROP TABLE '%s'", pszLayerName ),
                       NULL, NULL, &pszErrMsg );
    if( rc != SQLITE_OK )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Unable to drop table %s: %s",
                  pszLayerName, pszErrMsg );
        sqlite3_free( pszErrMsg );
        return;
    }

/* -------------------------------------------------------------------- */
/*      Drop from geometry_columns table.                               */
/* -------------------------------------------------------------------- */
    if( bHaveGeometryColumns )
    {
        CPLString osCommand;

        osCommand.Printf( 
            "DELETE FROM geometry_columns WHERE f_table_name = '%s'",
            pszLayerName );
        
        rc = sqlite3_exec( hDB, osCommand, NULL, NULL, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            CPLError( CE_Warning, CPLE_AppDefined,
                      "Removal from geometry_columns failed.\n%s: %s", 
                      osCommand.c_str(), pszErrMsg );
            sqlite3_free( pszErrMsg );
        }
    }
}

/************************************************************************/
/*                        SoftStartTransaction()                        */
/*                                                                      */
/*      Create a transaction scope.  If we already have a               */
/*      transaction active this isn't a real transaction, but just      */
/*      an increment to the scope count.                                */
/************************************************************************/

OGRErr OGRSQLiteDataSource::SoftStartTransaction()

{
    nSoftTransactionLevel++;

    if( nSoftTransactionLevel == 1 )
    {
        int rc;
        char *pszErrMsg;
        
#ifdef DEBUG
        CPLDebug( "OGR_SQLITE", "BEGIN Transaction" );
#endif

        rc = sqlite3_exec( hDB, "BEGIN", NULL, NULL, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            nSoftTransactionLevel--;
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "BEGIN transaction failed: %s",
                      pszErrMsg );
            sqlite3_free( pszErrMsg );
            return OGRERR_FAILURE;
        }
    }

    return OGRERR_NONE;
}

/************************************************************************/
/*                             SoftCommit()                             */
/*                                                                      */
/*      Commit the current transaction if we are at the outer           */
/*      scope.                                                          */
/************************************************************************/

OGRErr OGRSQLiteDataSource::SoftCommit()

{
    if( nSoftTransactionLevel <= 0 )
    {
        CPLDebug( "OGR_SQLITE", "SoftCommit() with no transaction active." );
        return OGRERR_FAILURE;
    }

    nSoftTransactionLevel--;

    if( nSoftTransactionLevel == 0 )
    {
        int rc;
        char *pszErrMsg;
        
#ifdef DEBUG
        CPLDebug( "OGR_SQLITE", "COMMIT Transaction" );
#endif

        rc = sqlite3_exec( hDB, "COMMIT", NULL, NULL, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "COMMIT transaction failed: %s",
                      pszErrMsg );
            sqlite3_free( pszErrMsg );
            return OGRERR_FAILURE;
        }
    }

    return OGRERR_NONE;
}

/************************************************************************/
/*                            SoftRollback()                            */
/*                                                                      */
/*      Force a rollback of the current transaction if there is one,    */
/*      even if we are nested several levels deep.                      */
/************************************************************************/

OGRErr OGRSQLiteDataSource::SoftRollback()

{
    if( nSoftTransactionLevel <= 0 )
    {
        CPLDebug( "OGR_SQLITE", "SoftRollback() with no transaction active." );
        return OGRERR_FAILURE;
    }

    nSoftTransactionLevel = 0;

    int rc;
    char *pszErrMsg;
    
#ifdef DEBUG
    CPLDebug( "OGR_SQLITE", "ROLLBACK Transaction" );
#endif

    rc = sqlite3_exec( hDB, "ROLLBACK", NULL, NULL, &pszErrMsg );
    if( rc != SQLITE_OK )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "ROLLBACK transaction failed: %s",
                  pszErrMsg );
        sqlite3_free( pszErrMsg );
        return OGRERR_FAILURE;
    }
    
    return OGRERR_NONE;
}

/************************************************************************/
/*                        FlushSoftTransaction()                        */
/*                                                                      */
/*      Force the unwinding of any active transaction, and it's         */
/*      commit.                                                         */
/************************************************************************/

OGRErr OGRSQLiteDataSource::FlushSoftTransaction()

{
    if( nSoftTransactionLevel <= 0 )
        return OGRERR_NONE;

    nSoftTransactionLevel = 1;

    return SoftCommit();
}

/************************************************************************/
/*                             FetchSRSId()                             */
/*                                                                      */
/*      Fetch the id corresponding to an SRS, and if not found, add     */
/*      it to the table.                                                */
/************************************************************************/

int OGRSQLiteDataSource::FetchSRSId( OGRSpatialReference * poSRS )

{
    int                 nSRSId = -1;
    const char          *pszAuthorityName, *pszAuthorityCode = NULL;
    CPLString           osCommand;
    char *pszErrMsg;
    int   rc;
    char **papszResult;
    int nRowCount, nColCount;

    if( poSRS == NULL )
        return -1;

    pszAuthorityName = poSRS->GetAuthorityName(NULL);

/* -------------------------------------------------------------------- */
/*      Check whether the EPSG authority code is already mapped to a    */
/*      SRS ID.                                                         */
/* -------------------------------------------------------------------- */
    if( pszAuthorityName != NULL && strlen(pszAuthorityName) > 0 )
    {
        pszAuthorityCode = poSRS->GetAuthorityCode(NULL);

        if ( pszAuthorityCode != NULL && strlen(pszAuthorityCode) > 0 )
        {
            // XXX: We are using case insensitive comparison for "auth_name"
            // values, because there are variety of options exist. By default
            // the driver uses 'EPSG' in upper case, but SpatiaLite extension
            // uses 'epsg' in lower case.
            osCommand.Printf( "SELECT srid FROM spatial_ref_sys WHERE "
                              "auth_name = '%s' COLLATE NOCASE AND auth_srid = '%s'",
                              pszAuthorityName, pszAuthorityCode );

            rc = sqlite3_get_table( hDB, osCommand, &papszResult, 
                                    &nRowCount, &nColCount, &pszErrMsg );
            if( rc != SQLITE_OK )
            {
                CPLError( CE_Failure, CPLE_AppDefined,
                          "Search for existing SRS ID failed: %s", pszErrMsg );
                sqlite3_free( pszErrMsg );
            }
            else if( nRowCount == 1 )
            {
                nSRSId = atoi(papszResult[1]);
                sqlite3_free_table(papszResult);
                return nSRSId;
            }
            sqlite3_free_table(papszResult);
        }
    }

/* -------------------------------------------------------------------- */
/*      Search for existing record using either WKT definition or       */
/*      PROJ.4 string (SpatiaLite variant).                             */
/* -------------------------------------------------------------------- */
    CPLString   osSRS;

    if ( !bIsSpatiaLite )
    {
/* -------------------------------------------------------------------- */
/*      Translate SRS to WKT.                                           */
/* -------------------------------------------------------------------- */
        char    *pszWKT = NULL;

        if( poSRS->exportToWkt( &pszWKT ) != OGRERR_NONE )
            return -1;

        osSRS = pszWKT;
        CPLFree( pszWKT );
        pszWKT = NULL;

/* -------------------------------------------------------------------- */
/*      Try to find based on the WKT match.                             */
/* -------------------------------------------------------------------- */
        osCommand.Printf( "SELECT srid FROM spatial_ref_sys WHERE srtext = '%s'",
                          osSRS.c_str());
        
        rc = sqlite3_get_table( hDB, osCommand, &papszResult,
                                &nRowCount, &nColCount, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Search for existing SRS by WKT failed: %s", pszErrMsg );
            sqlite3_free( pszErrMsg );
        }
        else if( nRowCount == 1 )
        {
            nSRSId = atoi(papszResult[1]);
            sqlite3_free_table(papszResult);
            return nSRSId;
        }
        sqlite3_free_table(papszResult);
    }

/* -------------------------------------------------------------------- */
/*      Handle SpatiaLite flavour of the spatial_ref_sys.               */
/* -------------------------------------------------------------------- */
    else
    {
/* -------------------------------------------------------------------- */
/*      Translate SRS to PROJ.4 string.                                 */
/* -------------------------------------------------------------------- */
        char    *pszProj4 = NULL;

        if( poSRS->exportToProj4( &pszProj4 ) != OGRERR_NONE )
            return -1;

        osSRS = pszProj4;
        CPLFree( pszProj4 );
        pszProj4 = NULL;

/* -------------------------------------------------------------------- */
/*      Try to find based on the WKT match.                             */
/* -------------------------------------------------------------------- */
        osCommand.Printf(
            "SELECT srid FROM spatial_ref_sys WHERE proj4text = '%s'",
            osSRS.c_str());
        
        rc = sqlite3_get_table( hDB, osCommand, &papszResult,
                                &nRowCount, &nColCount, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Search for existing SRS by PROJ.4 string failed: %s",
                      pszErrMsg );
            sqlite3_free( pszErrMsg );
        }
        else if( nRowCount == 1 )
        {
            nSRSId = atoi(papszResult[1]);
            sqlite3_free_table(papszResult);
            return nSRSId;
        }
        sqlite3_free_table(papszResult);
    }

/* -------------------------------------------------------------------- */
/*      If the command actually failed, then the metadata table is      */
/*      likely missing, so we give up.                                  */
/* -------------------------------------------------------------------- */
    if( rc != SQLITE_OK )
        return -1;

/* -------------------------------------------------------------------- */
/*      If we have an authority code try to assign SRS ID the same      */
/*      as that code.                                                   */
/* -------------------------------------------------------------------- */
    if ( pszAuthorityCode != NULL && strlen(pszAuthorityCode) > 0 )
    {
        osCommand.Printf( "SELECT * FROM spatial_ref_sys WHERE auth_srid='%s'",
                          pszAuthorityCode );
        rc = sqlite3_get_table( hDB, osCommand, &papszResult,
                                &nRowCount, &nColCount, &pszErrMsg );
        
        if( rc != SQLITE_OK )
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "exec(SELECT '%s' FROM spatial_ref_sys) failed: %s",
                      pszAuthorityCode, pszErrMsg );
            sqlite3_free( pszErrMsg );
        }

/* -------------------------------------------------------------------- */
/*      If there is no SRS ID with such auth_srid, use it as SRS ID.    */
/* -------------------------------------------------------------------- */
        if ( nRowCount < 1 )
            nSRSId = atoi(pszAuthorityCode);
        sqlite3_free_table(papszResult);
    }

/* -------------------------------------------------------------------- */
/*      Otherwise get the current maximum srid in the srs table.        */
/* -------------------------------------------------------------------- */
    if ( nSRSId == -1 )
    {
        rc = sqlite3_get_table( hDB, "SELECT MAX(srid) FROM spatial_ref_sys", 
                                &papszResult, &nRowCount, &nColCount,
                                &pszErrMsg );
        
        if( rc != SQLITE_OK )
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "SELECT of the maximum SRS ID failed: %s", pszErrMsg );
            sqlite3_free( pszErrMsg );
            return -1;
        }

        if ( nRowCount < 1 || !papszResult[1] )
            nSRSId = 50000;
        else
            nSRSId = atoi(papszResult[1]) + 1;  // Insert as the next SRS ID
        sqlite3_free_table(papszResult);
    }

/* -------------------------------------------------------------------- */
/*      Try adding the SRS to the SRS table.                            */
/* -------------------------------------------------------------------- */
    if ( !bIsSpatiaLite )
    {
        if( pszAuthorityName != NULL )
        {
            osCommand.Printf(
                "INSERT INTO spatial_ref_sys (srid,srtext,auth_name,auth_srid) "
                "                     VALUES (%d, '%s', '%s', '%s')",
                nSRSId, osSRS.c_str(), 
                pszAuthorityName, poSRS->GetAuthorityCode(NULL) );
        }
        else
        {
            osCommand.Printf(
                "INSERT INTO spatial_ref_sys (srid,srtext) "
                "                     VALUES (%d, '%s')",
                nSRSId, osSRS.c_str() );
        }
    }
    else
    {
        const char  *pszProjCS = poSRS->GetAttrValue("PROJCS");

        if( pszAuthorityName != NULL )
        {
            if ( pszProjCS )
                osCommand.Printf(
                    "INSERT INTO spatial_ref_sys "
                    "(srid, auth_name, auth_srid, ref_sys_name, proj4text) "
                    "VALUES (%d, '%s', '%s', '%s', '%s')",
                    nSRSId, pszAuthorityName,
                    poSRS->GetAuthorityCode(NULL), pszProjCS, osSRS.c_str() );
            else
                osCommand.Printf(
                    "INSERT INTO spatial_ref_sys "
                    "(srid, auth_name, auth_srid, proj4text) "
                    "VALUES (%d, '%s', '%s', '%s')",
                    nSRSId, pszAuthorityName,
                    poSRS->GetAuthorityCode(NULL), osSRS.c_str() );
        }
        else
        {
            if ( pszProjCS )
                osCommand.Printf(
                    "INSERT INTO spatial_ref_sys "
                    "(srid, ref_sys_name, proj4text) VALUES (%d, '%s', '%s')",
                    nSRSId, pszProjCS, osSRS.c_str() );
            else
                osCommand.Printf(
                    "INSERT INTO spatial_ref_sys "
                    "(srid, proj4text) VALUES (%d, '%s')",
                    nSRSId, osSRS.c_str() );
        }
    }

    rc = sqlite3_exec( hDB, osCommand, NULL, NULL, &pszErrMsg );
    if( rc != SQLITE_OK )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Unable to insert SRID (%s): %s",
                  osCommand.c_str(), pszErrMsg );
        sqlite3_free( pszErrMsg );
        return FALSE;
    }

    return nSRSId;
}

/************************************************************************/
/*                              FetchSRS()                              */
/*                                                                      */
/*      Return a SRS corresponding to a particular id.  Note that       */
/*      reference counting should be honoured on the returned           */
/*      OGRSpatialReference, as handles may be cached.                  */
/************************************************************************/

OGRSpatialReference *OGRSQLiteDataSource::FetchSRS( int nId )

{
    if( nId <= 0 )
        return NULL;

/* -------------------------------------------------------------------- */
/*      First, we look through our SRID cache, is it there?             */
/* -------------------------------------------------------------------- */
    int  i;

    for( i = 0; i < nKnownSRID; i++ )
    {
        if( panSRID[i] == nId )
            return papoSRS[i];
    }

/* -------------------------------------------------------------------- */
/*      Try looking up in spatial_ref_sys table.                        */
/* -------------------------------------------------------------------- */
    char *pszErrMsg;
    int   rc;
    char **papszResult;
    int nRowCount, nColCount;
    CPLString osCommand;
    OGRSpatialReference *poSRS = NULL;

    osCommand.Printf( "SELECT srtext FROM spatial_ref_sys WHERE srid = %d",
                      nId );
    rc = sqlite3_get_table( hDB, osCommand, 
                            &papszResult, &nRowCount, &nColCount, &pszErrMsg );

    if ( rc == SQLITE_OK )
    {
        if( nRowCount < 1 )
        {
            sqlite3_free_table(papszResult);
            return NULL;
        }

        CPLString osWKT = papszResult[1];
        sqlite3_free_table(papszResult);

/* -------------------------------------------------------------------- */
/*      Translate into a spatial reference.                             */
/* -------------------------------------------------------------------- */
        char *pszWKT = (char *) osWKT.c_str();

        poSRS = new OGRSpatialReference();
        if( poSRS->importFromWkt( &pszWKT ) != OGRERR_NONE )
        {
            delete poSRS;
            poSRS = NULL;
        }
    }

/* -------------------------------------------------------------------- */
/*      Next try SpatiaLite flavour. SpatiaLite uses PROJ.4 strings     */
/*      in 'proj4text' column instead of WKT in 'srtext'.               */
/* -------------------------------------------------------------------- */
    else
    {
        sqlite3_free( pszErrMsg );
        pszErrMsg = NULL;

        osCommand.Printf(
            "SELECT proj4text FROM spatial_ref_sys WHERE srid = %d", nId );
        rc = sqlite3_get_table( hDB, osCommand, 
                                &papszResult, &nRowCount,
                                &nColCount, &pszErrMsg );
        if ( rc == SQLITE_OK )
        {
            if( nRowCount < 1 )
            {
                sqlite3_free_table(papszResult);
                return NULL;
            }

/* -------------------------------------------------------------------- */
/*      Translate into a spatial reference.                             */
/* -------------------------------------------------------------------- */
            poSRS = new OGRSpatialReference();
            if( poSRS->importFromProj4( papszResult[1] ) != OGRERR_NONE )
            {
                delete poSRS;
                poSRS = NULL;
            }

            sqlite3_free_table(papszResult);
        }

/* -------------------------------------------------------------------- */
/*      No success, report an error.                                    */
/* -------------------------------------------------------------------- */
        else
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "%s: %s", osCommand.c_str(), pszErrMsg );
            sqlite3_free( pszErrMsg );
            return NULL;
        }
    }

/* -------------------------------------------------------------------- */
/*      Add to the cache.                                               */
/* -------------------------------------------------------------------- */
    panSRID = (int *) CPLRealloc(panSRID,sizeof(int) * (nKnownSRID+1) );
    papoSRS = (OGRSpatialReference **)
        CPLRealloc(papoSRS, sizeof(void*) * (nKnownSRID + 1) );
    panSRID[nKnownSRID] = nId;
    papoSRS[nKnownSRID] = poSRS;
    nKnownSRID++;

    return poSRS;
}

