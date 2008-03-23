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
        "SELECT f_table_name, f_geometry_column, type, coord_dimension, f_geometry_format"
        " FROM geometry_columns",
        &papszResult, &nRowCount, &nColCount, &pszErrMsg );

    if( rc == SQLITE_OK )
    {
        bHaveGeometryColumns = TRUE;

        for( iRow = 0; iRow < nRowCount; iRow++ )
        {
            char **papszRow = papszResult + iRow * 5 + 5;
            OGRwkbGeometryType eGeomType = wkbUnknown;

            if( EQUAL(papszRow[2],"POINT") )
                eGeomType = wkbPoint;
            else if( EQUAL(papszRow[2],"LINESTRING") )
                eGeomType = wkbLineString;
            else if( EQUAL(papszRow[2],"POLYGON") )
                eGeomType = wkbPolygon;
            else if( EQUAL(papszRow[2],"MULTIPOINT") )
                eGeomType = wkbMultiPoint;
            else if( EQUAL(papszRow[2],"MULTILINESTRING") )
                eGeomType = wkbMultiLineString;
            else if( EQUAL(papszRow[2],"MULTIPOLYGON") )
                eGeomType = wkbMultiPolygon;
            else if( EQUAL(papszRow[2],"MULTIPOLYGON") )
                eGeomType = wkbMultiPolygon;
            else if( EQUAL(papszRow[2],"GEOMETRYCOLLECTION") )
                eGeomType = wkbGeometryCollection;
            else if( EQUAL(papszRow[2],"GEOMETRY") )
                eGeomType = wkbUnknown;
            else
            {
                CPLDebug( "SQLITE", "Unrecognised geometry type '%s'.", 
                          papszRow[2] );
            }

            if( atoi(papszRow[3]) > 2 )
                eGeomType = (OGRwkbGeometryType) (((int)eGeomType) | wkb25DBit);

            OpenTable( papszRow[0], papszRow[1], eGeomType, papszRow[4] );
        }

        sqlite3_free_table(papszResult);

        return TRUE;
    }

/* -------------------------------------------------------------------- */
/*      Otherwise our final resort is to return all tables and views    */
/*      as non-spatial tables.                                          */
/* -------------------------------------------------------------------- */
    else
    {
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
    }

    return TRUE;
}

/************************************************************************/
/*                             OpenTable()                              */
/************************************************************************/

int OGRSQLiteDataSource::OpenTable( const char *pszNewName, 
                                    const char *pszGeomCol,
                                    OGRwkbGeometryType eGeomType,
                                    const char *pszGeomFormat )

{
/* -------------------------------------------------------------------- */
/*      Create the layer object.                                        */
/* -------------------------------------------------------------------- */
    OGRSQLiteTableLayer  *poLayer;

    poLayer = new OGRSQLiteTableLayer( this );

    if( poLayer->Initialize( pszNewName, pszGeomCol, 
                             eGeomType, pszGeomFormat ) )
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
        pszGeomFormat = "WKT";

    if( !EQUAL(pszGeomFormat,"WKT") 
        && !EQUAL(pszGeomFormat,"WKB") )
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
/*      adding tot the srs table if needed.                             */
/* -------------------------------------------------------------------- */
#ifdef notdef
    int nSRSId = -1;

    if( poSRS != NULL )
        nSRSId = FetchSRSId( poSRS );
#endif

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

    CPLDebug( "OGR_SQLITE", "exec(%s)", osCommand.c_str() );

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
        const char *pszGeometryType;
        int nCoordDim;

        /* Sometimes there is an old cruft entry in the geometry_columns
         * table if things were not properly cleaned up before.  We make
         * an effort to clean out such cruft.
         */
        osCommand.Printf(
            "DELETE FROM geometry_columns WHERE f_table_name = '%s'", 
            pszLayerName );
                 
        CPLDebug( "OGR_SQLITE", "exec(%s)", osCommand.c_str() );
        rc = sqlite3_exec( hDB, osCommand, NULL, NULL, &pszErrMsg );
        if( rc != SQLITE_OK )
        {
            sqlite3_free( pszErrMsg );
            return FALSE;
        }
        
        switch( wkbFlatten(eType) )
        {
          case wkbPoint:
            pszGeometryType = "POINT";
            break;

          case wkbLineString:
            pszGeometryType = "LINESTRING";
            break;

          case wkbPolygon:
            pszGeometryType = "POLYGON";
            break;

          case wkbMultiPoint:
            pszGeometryType = "MULTIPOINT";
            break;

          case wkbMultiLineString:
            pszGeometryType = "MULTILINESTRING";
            break;

          case wkbMultiPolygon:
            pszGeometryType = "MULTIPOLYGON";
            break;

          case wkbGeometryCollection:
            pszGeometryType = "GEOMETRYCOLLECTION";
            break;

          default:
            pszGeometryType = "GEOMETRY";
            break;

        }

        if( eType == wkbFlatten(eType) )
            nCoordDim = 2;
        else
            nCoordDim = 3;

        osCommand.Printf(
            "INSERT INTO geometry_columns "
            "(f_table_name, f_geometry_column, f_geometry_format, type, "
            "coord_dimension, srid) VALUES "
            "('%s','%s','%s','%s', %d, %d)", 
            pszLayerName, pszGeomCol, pszGeomFormat, pszGeometryType, 
            nCoordDim, -1 );
        
        CPLDebug( "OGR_SQLITE", "exec(%s)", osCommand.c_str() );
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

    poLayer->Initialize( pszLayerName, pszGeomCol, eType, pszGeomFormat );

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
    CPLDebug( "OGR_PG", "DeleteLayer(%s)", pszLayerName );

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
        
        CPLDebug( "OGR_SQLITE", "BEGIN Transaction" );
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
        
        CPLDebug( "OGR_SQLITE", "COMMIT Transaction" );
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
    
    CPLDebug( "OGR_SQLITE", "ROLLBACK Transaction" );
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
