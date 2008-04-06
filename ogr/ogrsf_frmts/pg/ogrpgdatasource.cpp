/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements OGRPGDataSource class.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2000, Frank Warmerdam
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

#include "ogr_pg.h"
#include "ogrpgutility.h"
#include "cpl_conv.h"
#include "cpl_string.h"

CPL_CVSID("$Id$");

static void OGRPGNoticeProcessor( void *arg, const char * pszMessage );

/************************************************************************/
/*                          OGRPGDataSource()                           */
/************************************************************************/

OGRPGDataSource::OGRPGDataSource()

{
    pszName = NULL;
    pszDBName = NULL;
    papoLayers = NULL;
    nLayers = 0;
    hPGConn = NULL;
    bHavePostGIS = FALSE;
    bUseBinaryCursor = FALSE;
    nSoftTransactionLevel = 0;
    bBinaryTimeFormatIsInt8 = FALSE;

    nKnownSRID = 0;
    panSRID = NULL;
    papoSRS = NULL;

    poLayerInCopyMode = NULL;
}

/************************************************************************/
/*                          ~OGRPGDataSource()                          */
/************************************************************************/

OGRPGDataSource::~OGRPGDataSource()

{
    int         i;

    FlushSoftTransaction();

    CPLFree( pszName );
    CPLFree( pszDBName );

    for( i = 0; i < nLayers; i++ )
        delete papoLayers[i];

    CPLFree( papoLayers );

    if( hPGConn != NULL )
    {
        /* XXX - mloskot: After the connection is closed, valgrind still
         * reports 36 bytes definitely lost, somewhere in the libpq.
         */
        PQfinish( hPGConn );
        hPGConn = NULL;
    }

    for( i = 0; i < nKnownSRID; i++ )
    {
        if( papoSRS[i] != NULL )
            papoSRS[i]->Release();
    }
    CPLFree( panSRID );
    CPLFree( papoSRS );
}

/************************************************************************/
/*                      OGRPGDecodeVersionString()                      */
/************************************************************************/

void OGRPGDataSource::OGRPGDecodeVersionString(PGver* psVersion, char* pszVer)
{
    GUInt32 iLen;
    char* ptr;
    char szNum[25];
    char szVer[10];

    ptr = pszVer;
    // get Version string
    if ( *ptr == ' ' ) *ptr++;
    while (*ptr && *ptr != ' ') ptr++;
    iLen = ptr-pszVer;
    if ( iLen > sizeof(szVer) - 1 ) iLen = sizeof(szVer) - 1;
    strncpy(szVer,pszVer,iLen);
    szVer[iLen] = '\0';

    ptr = pszVer = szVer;

    // get Major number
    while (*ptr && *ptr != '.') ptr++;
    iLen = ptr-pszVer;
    if ( iLen > sizeof(szNum) - 1) iLen = sizeof(szNum) - 1;
    strncpy(szNum,pszVer,iLen);
    szNum[iLen] = '\0';
    psVersion->nMajor = atoi(szNum);

    pszVer = ++ptr;

    // get Minor number
    while (*ptr && *ptr != '.') ptr++;
    iLen = ptr-pszVer;
    if ( iLen > sizeof(szNum) - 1) iLen = sizeof(szNum) - 1;
    strncpy(szNum,pszVer,iLen);
    szNum[iLen] = '\0';
    psVersion->nMinor = atoi(szNum);


    if ( *ptr )
    {
        pszVer = ++ptr;

        // get Release number
        while (*ptr && *ptr != '.') ptr++;
        iLen = ptr-pszVer;
        if ( iLen > sizeof(szNum) - 1) iLen = sizeof(szNum) - 1;
        strncpy(szNum,pszVer,iLen);
        szNum[iLen] = '\0';
        psVersion->nRelease = atoi(szNum);
    }

}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

int OGRPGDataSource::Open( const char * pszNewName, int bUpdate,
                              int bTestOpen )

{
    CPLAssert( nLayers == 0 );

/* -------------------------------------------------------------------- */
/*      Verify postgresql prefix.                                       */
/* -------------------------------------------------------------------- */
    if( EQUALN(pszNewName,"PGB:",4) )
    {
        bUseBinaryCursor = TRUE;
        CPLDebug("OGR_PG","BINARY cursor is used for geometry fetching");
    }
    else
    if( !EQUALN(pszNewName,"PG:",3) )
    {
        if( !bTestOpen )
            CPLError( CE_Failure, CPLE_AppDefined,
                      "%s does not conform to PostgreSQL naming convention,"
                      " PG:*\n", pszNewName );
        return FALSE;
    }

    pszName = CPLStrdup( pszNewName );

/* -------------------------------------------------------------------- */
/*      Determine if the connection string contains an optional         */
/*      TABLES portion. If so, parse it out. The expected               */
/*      connection string in this case will be, e.g.:                   */
/*                                                                      */
/*        'PG:dbname=warmerda user=warmerda tables=s1.t1,[s2.t2,...]    */
/*              - where sN is schema and tN is table name               */
/*      We must also strip this information from the connection         */
/*      string; PQconnectdb() does not like unknown directives          */
/* -------------------------------------------------------------------- */
    char              **papszTableNames=NULL;
    char              **papszSchemaNames=NULL;

    char             *pszTableStart;
    pszTableStart = strstr(pszName, "tables=");
    if (pszTableStart == NULL)
        pszTableStart = strstr(pszName, "TABLES=");

    if( pszTableStart != NULL )
    {
        char          **papszTableList;
        char           *pszTableSpec;
        const char     *pszEnd = NULL;
        int             i;

        pszTableSpec = CPLStrdup( pszTableStart + 7 );

        for( i = 0; pszTableStart[i] != '\0'; i++ )
        {
            if( pszTableStart[i] == ' ' )
            {
                pszEnd = pszTableStart + i;
                break;
            }
        }

        if( pszEnd == NULL )
            pszEnd = pszName + strlen(pszName);

        // Remove TABLES=xxxxx from pszName string
        memmove( pszTableStart, pszEnd, strlen(pszEnd) + 1 );

        pszTableSpec[pszEnd - pszTableStart - 7] = '\0';
        papszTableList = CSLTokenizeString2( pszTableSpec, ",", 0 );

        for( i = 0; i < CSLCount(papszTableList); i++ )
        {
            char      **papszQualifiedParts;

            // Get schema and table name
            papszQualifiedParts = CSLTokenizeString2( papszTableList[i],
                                                      ".", 0 );

            if( CSLCount( papszQualifiedParts ) == 2 )
            {
                papszSchemaNames = CSLAddString( papszSchemaNames, 
                                                papszQualifiedParts[0] );
                papszTableNames = CSLAddString( papszTableNames,
                                                papszQualifiedParts[1] );
            }
            else if( CSLCount( papszQualifiedParts ) == 1 )
            {
                papszSchemaNames = CSLAddString( papszSchemaNames, "public");
                papszTableNames = CSLAddString( papszTableNames,
                                                papszQualifiedParts[0] );
            }

            CSLDestroy(papszQualifiedParts);
        }

        CSLDestroy(papszTableList);
        CPLFree(pszTableSpec);
    }

/* -------------------------------------------------------------------- */
/*      Try to establish connection.                                    */
/* -------------------------------------------------------------------- */
    hPGConn = PQconnectdb( pszName + (bUseBinaryCursor ? 4 : 3) );
    if( hPGConn == NULL || PQstatus(hPGConn) == CONNECTION_BAD )
    {
        CPLFree(pszName);
        pszName = NULL;

        CPLError( CE_Failure, CPLE_AppDefined,
                  "PQconnectdb failed.\n%s",
                  PQerrorMessage(hPGConn) );
        PQfinish(hPGConn);
        hPGConn = NULL;
        return FALSE;
    }

    bDSUpdate = bUpdate;

/* -------------------------------------------------------------------- */
/*      Set the encoding						*/
/* -------------------------------------------------------------------- */
#ifdef notdef
    char* encoding = "LATIN1";
    if (PQsetClientEncoding(hPGConn, encoding) == -1)
    {
        CPLError( CE_Warning, CPLE_AppDefined,
                  "PQsetClientEncoding(%s) failed.\n%s", 
		  encoding, PQerrorMessage( hPGConn ) );
    }
#endif

/* -------------------------------------------------------------------- */
/*      Install a notice processor.                                     */
/* -------------------------------------------------------------------- */
    PQsetNoticeProcessor( hPGConn, OGRPGNoticeProcessor, this );

/* -------------------------------------------------------------------- */
/*      Try to establish the database name from the connection          */
/*      string passed.                                                  */
/* -------------------------------------------------------------------- */
    if( strstr(pszNewName, "dbname=") != NULL )
    {
        pszDBName = CPLStrdup( strstr(pszNewName, "dbname=") + 7 );

        for( int i = 0; pszDBName[i] != '\0'; i++ )
        {
            if( pszDBName[i] == ' ' )
            {
                pszDBName[i] = '\0';
                break;
            }
        }
    }
    else if( getenv( "USER" ) != NULL )
        pszDBName = CPLStrdup( getenv("USER") );
    else
        pszDBName = CPLStrdup( "unknown_dbname" );

    CPLDebug( "OGR_PG", "DBName=\"%s\"", pszDBName );


/* -------------------------------------------------------------------- */
/*      Find out PostgreSQL version                                     */
/* -------------------------------------------------------------------- */
    PGresult    *hResult = NULL;

    sPostgreSQLVersion.nMajor = -1;
    sPostgreSQLVersion.nMinor = -1;
    sPostgreSQLVersion.nRelease = -1;

    hResult = PQexec(hPGConn, "SELECT version()" );
    if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK
        && PQntuples(hResult) > 0 )
    {
        char * pszVer = PQgetvalue(hResult,0,0);

        CPLDebug("OGR_PG","PostgreSQL version string : '%s'", pszVer);

        if (EQUALN(pszVer, "PostgreSQL ", 11))
        {
            OGRPGDecodeVersionString(&sPostgreSQLVersion, pszVer + 11);
            if (sPostgreSQLVersion.nMajor == 7 && sPostgreSQLVersion.nMinor < 4)
            {
                /* We don't support BINARY CURSOR for PostgreSQL < 7.4. */
                /* The binary protocol for arrays seems to be different from later versions */
                CPLDebug("OGR_PG","BINARY cursor will finally NOT be used because version < 7.4");
                bUseBinaryCursor = FALSE;
            }
        }
    }
    OGRPGClearResult(hResult);
    CPLAssert(NULL == hResult); /* Test if safe PQclear has not been broken */

/* -------------------------------------------------------------------- */
/*      Test if time binary format is int8 or float8                    */
/* -------------------------------------------------------------------- */
#if !defined(PG_PRE74)
    if (bUseBinaryCursor)
    {
        SoftStartTransaction();

        hResult = PQexec(hPGConn, "DECLARE gettimebinaryformat BINARY CURSOR FOR SELECT CAST ('00:00:01' AS time)");

        if( hResult && PQresultStatus(hResult) == PGRES_COMMAND_OK )
        {
            OGRPGClearResult( hResult );

            hResult = PQexec(hPGConn, "FETCH ALL IN gettimebinaryformat" );

            if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK  && PQntuples(hResult) == 1 )
            {
                if ( PQfformat( hResult, 0 ) == 1 ) // Binary data representation
                {
                    CPLAssert(PQgetlength(hResult, 0, 0) == 8);
                    double dVal;
                    unsigned int nVal[2];
                    memcpy( nVal, PQgetvalue( hResult, 0, 0 ), 8 );
                    CPL_MSBPTR32(&nVal[0]);
                    CPL_MSBPTR32(&nVal[1]);
                    memcpy( &dVal, PQgetvalue( hResult, 0, 0 ), 8 );
                    CPL_MSBPTR64(&dVal);
                    if (nVal[0] == 0 && nVal[1] == 1000000)
                    {
                        bBinaryTimeFormatIsInt8 = TRUE;
                        CPLDebug( "OGR_PG", "Time binary format is int8");
                    }
                    else if (dVal == 1.)
                    {
                        bBinaryTimeFormatIsInt8 = FALSE;
                        CPLDebug( "OGR_PG", "Time binary format is float8");
                    }
                    else
                    {
                        bBinaryTimeFormatIsInt8 = FALSE;
                        CPLDebug( "OGR_PG", "Time binary format is unknown");
                    }
                }
            }
        }

        OGRPGClearResult( hResult );

        hResult = PQexec(hPGConn, "CLOSE gettimebinaryformat");
        OGRPGClearResult( hResult );

        SoftCommit();
    }
#endif

/* -------------------------------------------------------------------- */
/*      Test to see if this database instance has support for the       */
/*      PostGIS Geometry type.  If so, disable sequential scanning      */
/*      so we will get the value of the gist indexes.                   */
/* -------------------------------------------------------------------- */
    hResult = PQexec(hPGConn, "BEGIN");

    if( hResult && PQresultStatus(hResult) == PGRES_COMMAND_OK )
    {
        OGRPGClearResult( hResult );
        CPLAssert(NULL == hResult);

        hResult = PQexec(hPGConn,
                         "SELECT oid FROM pg_type WHERE typname = 'geometry'" );
    }

    if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK
        && PQntuples(hResult) > 0 )
    {
        bHavePostGIS = TRUE;
        nGeometryOID = atoi(PQgetvalue(hResult,0,0));
    }
    else
    {
        nGeometryOID = (Oid) 0;
    }

    OGRPGClearResult( hResult );

/* -------------------------------------------------------------------- */
/*      Find out PostGIS version                                        */
/* -------------------------------------------------------------------- */
    // find out postgis version.
    sPostGISVersion.nMajor = -1;
    sPostGISVersion.nMinor = -1;
    sPostGISVersion.nRelease = -1;

    if( bHavePostGIS )
    {
        hResult = PQexec(hPGConn, "SELECT postgis_version()" );
        if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK
            && PQntuples(hResult) > 0 )
        {
            char * pszVer = PQgetvalue(hResult,0,0);

            CPLDebug("OGR_PG","PostGIS version string : '%s'", pszVer);

            OGRPGDecodeVersionString(&sPostGISVersion, pszVer);

        }
        OGRPGClearResult(hResult);


        if (sPostGISVersion.nMajor == 0 && sPostGISVersion.nMinor < 8)
        {
            // Turning off sequential scans for PostGIS < 0.8
            hResult = PQexec(hPGConn, "SET ENABLE_SEQSCAN = OFF");
            
            CPLDebug( "OGR_PG", "SET ENABLE_SEQSCAN=OFF" );
        }
        else
        {
            // PostGIS >=0.8 is correctly integrated with query planner,
            // thus PostgreSQL will use indexes whenever appropriate.
            hResult = PQexec(hPGConn, "SET ENABLE_SEQSCAN = ON");
        }
        OGRPGClearResult( hResult );
    }

    hResult = PQexec(hPGConn, "COMMIT");
    OGRPGClearResult( hResult );

/* -------------------------------------------------------------------- */
/*      Get a list of available tables if they have not been            */
/*      specified through the TABLES connection string param           */
/* -------------------------------------------------------------------- */

    if (papszTableNames == NULL)
    {
        hResult = PQexec(hPGConn, "BEGIN");

        if( hResult && PQresultStatus(hResult) == PGRES_COMMAND_OK )
        {
            OGRPGClearResult( hResult );

            if ( bHavePostGIS )
                hResult = PQexec(hPGConn,
                                "DECLARE mycursor CURSOR for "
                                "SELECT c.relname, n.nspname FROM pg_class c, pg_namespace n, geometry_columns g "
                                "WHERE (c.relkind in ('r','v') AND c.relname !~ '^pg' AND c.relnamespace=n.oid "
                                "AND c.relname::TEXT = g.f_table_name::TEXT AND n.nspname = g.f_table_schema)" );
            else
                hResult = PQexec(hPGConn,
                                "DECLARE mycursor CURSOR for "
                                "SELECT c.relname, n.nspname FROM pg_class c, pg_namespace n "
                                "WHERE (c.relkind in ('r','v') AND c.relname !~ '^pg' AND c.relnamespace=n.oid)" );
        }

        if( hResult && PQresultStatus(hResult) == PGRES_COMMAND_OK )
        {
            OGRPGClearResult( hResult );
            hResult = PQexec(hPGConn, "FETCH ALL in mycursor" );
        }

        if( !hResult || PQresultStatus(hResult) != PGRES_TUPLES_OK )
        {
            OGRPGClearResult( hResult );

            CPLError( CE_Failure, CPLE_AppDefined,
                    "%s", PQerrorMessage(hPGConn) );
            return FALSE;
        }

    /* -------------------------------------------------------------------- */
    /*      Parse the returned table list                                   */
    /* -------------------------------------------------------------------- */
        for( int iRecord = 0; iRecord < PQntuples(hResult); iRecord++ )
        {
            const char *pszTable = PQgetvalue(hResult, iRecord, 0);

            if( EQUAL(pszTable,"spatial_ref_sys")
                || EQUAL(pszTable,"geometry_columns") )
                continue;

            papszTableNames = CSLAddString(papszTableNames,
                                        PQgetvalue(hResult, iRecord, 0));
            papszSchemaNames = CSLAddString(papszSchemaNames,
                                        PQgetvalue(hResult, iRecord, 1));

        }

    /* -------------------------------------------------------------------- */
    /*      Cleanup                                                         */
    /* -------------------------------------------------------------------- */
        OGRPGClearResult( hResult );

        hResult = PQexec(hPGConn, "CLOSE mycursor");
        OGRPGClearResult( hResult );

        hResult = PQexec(hPGConn, "COMMIT");
        OGRPGClearResult( hResult );
    }

/* -------------------------------------------------------------------- */
/*      Register the available tables.                                  */
/* -------------------------------------------------------------------- */
    for( int iRecord = 0;
         papszTableNames != NULL && papszTableNames[iRecord] != NULL;
         iRecord++ )
    {
        OpenTable( papszTableNames[iRecord], papszSchemaNames[iRecord], bUpdate, FALSE );
    }

    CSLDestroy( papszSchemaNames );
    CSLDestroy( papszTableNames );

/* -------------------------------------------------------------------- */
    return nLayers > 0 || bUpdate;
}

/************************************************************************/
/*                             OpenTable()                              */
/************************************************************************/

int OGRPGDataSource::OpenTable( const char *pszNewName, const char *pszSchemaName, int bUpdate,
                                int bTestOpen )

{
/* -------------------------------------------------------------------- */
/*      Create the layer object.                                        */
/* -------------------------------------------------------------------- */
    OGRPGTableLayer  *poLayer;

    poLayer = new OGRPGTableLayer( this, pszNewName, pszSchemaName, bUpdate );
    if( poLayer->GetLayerDefn() == NULL )
    {
        delete poLayer;
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Add layer to data source layer list.                            */
/* -------------------------------------------------------------------- */
    papoLayers = (OGRPGTableLayer **)
        CPLRealloc( papoLayers,  sizeof(OGRPGTableLayer *) * (nLayers+1) );
    papoLayers[nLayers++] = poLayer;

    return TRUE;
}

/************************************************************************/
/*                            DeleteLayer()                             */
/************************************************************************/

int OGRPGDataSource::DeleteLayer( int iLayer )

{
    if( iLayer < 0 || iLayer >= nLayers )
        return OGRERR_FAILURE;

/* -------------------------------------------------------------------- */
/*      Blow away our OGR structures related to the layer.  This is     */
/*      pretty dangerous if anything has a reference to this layer!     */
/* -------------------------------------------------------------------- */
    CPLString osLayerName = papoLayers[iLayer]->GetLayerDefn()->GetName();
    CPLString osTableName = papoLayers[iLayer]->GetTableName();
    CPLString osSchemaName = papoLayers[iLayer]->GetSchemaName();

    CPLDebug( "OGR_PG", "DeleteLayer(%s)", osLayerName.c_str() );

    delete papoLayers[iLayer];
    memmove( papoLayers + iLayer, papoLayers + iLayer + 1,
             sizeof(void *) * (nLayers - iLayer - 1) );
    nLayers--;

/* -------------------------------------------------------------------- */
/*      Remove from the database.                                       */
/* -------------------------------------------------------------------- */
    PGresult            *hResult;
    char                szCommand[1024];

    hResult = PQexec(hPGConn, "BEGIN");
    OGRPGClearResult( hResult );

    if( bHavePostGIS )
    {
        sprintf( szCommand,
                 "SELECT DropGeometryColumn('%s','%s',(SELECT f_geometry_column from geometry_columns where f_table_name='%s' and f_table_schema='%s' order by f_geometry_column limit 1))",
                 osSchemaName.c_str(), osTableName.c_str(), osTableName.c_str(), osSchemaName.c_str() );

        CPLDebug( "OGR_PG", "PGexec(%s)", szCommand );

        hResult = PQexec( hPGConn, szCommand );
        OGRPGClearResult( hResult );
    }

    sprintf( szCommand, "DROP TABLE \"%s\".\"%s\" CASCADE", osSchemaName.c_str(), osTableName.c_str() );
    CPLDebug( "OGR_PG", "PGexec(%s)", szCommand );
    hResult = PQexec( hPGConn, szCommand );
    OGRPGClearResult( hResult );

    hResult = PQexec(hPGConn, "COMMIT");
    OGRPGClearResult( hResult );

    return OGRERR_NONE;
}

/************************************************************************/
/*                            CreateLayer()                             */
/************************************************************************/

OGRLayer *
OGRPGDataSource::CreateLayer( const char * pszLayerNameIn,
                              OGRSpatialReference *poSRS,
                              OGRwkbGeometryType eType,
                              char ** papszOptions )

{
    PGresult            *hResult = NULL;
    char                szCommand[1024];
    const char          *pszGeomType = NULL;
    char                *pszLayerName = NULL;
    const char          *pszTableName = NULL;
    char                *pszSchemaName = NULL;
    int                 nDimension = 3;

    if( CSLFetchBoolean(papszOptions,"LAUNDER", TRUE) )
    {
        pszLayerName = LaunderName( pszLayerNameIn );
        
    }
    else
        pszLayerName = CPLStrdup( pszLayerNameIn );

    if( wkbFlatten(eType) == eType )
        nDimension = 2;

    /* Postgres Schema handling:
       Extract schema name from input layer name or passed with -lco SCHEMA.
       Set layer name to "schema.table" or to "table" if schema == current_schema()
       Usage without schema name is backwards compatible
    */
    pszTableName = strstr(pszLayerName,".");
    if ( pszTableName != NULL )
    {
      int length = pszTableName - pszLayerName;
      pszSchemaName = (char*)CPLMalloc(length+1);
      strncpy(pszSchemaName, pszLayerName, length);
      pszSchemaName[length] = '\0';
      ++pszTableName; //skip "."
    }
    else
    {
      pszSchemaName = NULL;
      pszTableName = pszLayerName;
    }

/* -------------------------------------------------------------------- */
/*      Set the default schema for the layers.                          */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue( papszOptions, "SCHEMA" ) != NULL )
    {
        CPLFree(pszSchemaName);
        pszSchemaName = CPLStrdup(CSLFetchNameValue( papszOptions, "SCHEMA" ));
    }

    if ( pszSchemaName == NULL )
    {
      //pszSchemaName = current_schema()
      hResult = PQexec(hPGConn,"SELECT current_schema()");
      if ( hResult && PQntuples(hResult) == 1 && !PQgetisnull(hResult,0,0) )
      {
          pszSchemaName = CPLStrdup(PQgetvalue(hResult,0,0));
      }
      OGRPGClearResult( hResult );
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
                DeleteLayer( iLayer );
            }
            else
            {
                CPLError( CE_Failure, CPLE_AppDefined,
                          "Layer %s already exists, CreateLayer failed.\n"
                          "Use the layer creation option OVERWRITE=YES to "
                          "replace it.",
                          pszLayerName );
                CPLFree( pszLayerName );
                CPLFree( pszSchemaName );
                return NULL;
            }
        }
    }

/* -------------------------------------------------------------------- */
/*      Handle the GEOM_TYPE option.                                    */
/* -------------------------------------------------------------------- */
    pszGeomType = CSLFetchNameValue( papszOptions, "GEOM_TYPE" );
    if( pszGeomType == NULL )
    {
        if( bHavePostGIS )
            pszGeomType = "geometry";
        else
            pszGeomType = "bytea";
    }

    if( bHavePostGIS && !EQUAL(pszGeomType,"geometry") )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "Can't override GEOM_TYPE in PostGIS enabled databases.\n"
                  "Creation of layer %s with GEOM_TYPE %s has failed.",
                  pszLayerName, pszGeomType );
        CPLFree( pszLayerName );
        CPLFree( pszSchemaName );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Try to get the SRS Id of this spatial reference system,         */
/*      adding tot the srs table if needed.                             */
/* -------------------------------------------------------------------- */
    int nSRSId = -1;

    if( poSRS != NULL )
        nSRSId = FetchSRSId( poSRS );

/* -------------------------------------------------------------------- */
/*      Create a basic table with the FID.  Also include the            */
/*      geometry if this is not a PostGIS enabled table.                */
/* -------------------------------------------------------------------- */
    hResult = PQexec(hPGConn, "BEGIN");
    OGRPGClearResult( hResult );

    if( !bHavePostGIS )
    {
        sprintf( szCommand,
                 "CREATE TABLE \"%s\".\"%s\" ( "
                 "   OGC_FID SERIAL, "
                 "   WKB_GEOMETRY %s, "
                 "   CONSTRAINT \"%s_pk\" PRIMARY KEY (OGC_FID) )",
                 pszSchemaName, pszTableName, pszGeomType, pszTableName );
    }
    else
    {
        sprintf( szCommand,
                 "CREATE TABLE \"%s\".\"%s\" ( OGC_FID SERIAL, CONSTRAINT \"%s_pk\" PRIMARY KEY (OGC_FID) )",
                 pszSchemaName, pszTableName, pszTableName );
    }

    CPLDebug( "OGR_PG", "PQexec( %s )", szCommand );
    hResult = PQexec(hPGConn, szCommand);
    if( PQresultStatus(hResult) != PGRES_COMMAND_OK )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "%s\n%s", szCommand, PQerrorMessage(hPGConn) );
        CPLFree( pszLayerName );
        CPLFree( pszSchemaName );

        OGRPGClearResult( hResult );
        hResult = PQexec( hPGConn, "ROLLBACK" );
        OGRPGClearResult( hResult );
        return NULL;
    }

    OGRPGClearResult( hResult );

/* -------------------------------------------------------------------- */
/*      Eventually we should be adding this table to a table of         */
/*      "geometric layers", capturing the WKT projection, and           */
/*      perhaps some other housekeeping.                                */
/* -------------------------------------------------------------------- */
    if( bHavePostGIS )
    {
        const char *pszGeometryType;
        const char *pszGFldName;
 
        if( CSLFetchNameValue( papszOptions, "DIM") != NULL )
            nDimension = atoi(CSLFetchNameValue( papszOptions, "DIM"));

        if( CSLFetchNameValue( papszOptions, "GEOMETRY_NAME") != NULL )
            pszGFldName = CSLFetchNameValue( papszOptions, "GEOMETRY_NAME");
	else
	    pszGFldName = "wkb_geometry";

        /* Sometimes there is an old cruft entry in the geometry_columns
         * table if things were not properly cleaned up before.  We make
         * an effort to clean out such cruft.
         */
        sprintf( szCommand,
                 "DELETE FROM geometry_columns WHERE f_table_name = '%s' AND f_table_schema = '%s'",
                 pszTableName, pszSchemaName );

        CPLDebug( "OGR_PG", "PQexec(%s)", szCommand );
        hResult = PQexec(hPGConn, szCommand);
        OGRPGClearResult( hResult );

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

        sprintf( szCommand,
                 "select AddGeometryColumn('%s','%s','%s',%d,'%s',%d)",
                 pszSchemaName, pszTableName, pszGFldName, nSRSId, pszGeometryType,
                 nDimension );

        CPLDebug( "OGR_PG", "PQexec(%s)", szCommand );
        hResult = PQexec(hPGConn, szCommand);

        if( !hResult
            || PQresultStatus(hResult) != PGRES_TUPLES_OK )
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "AddGeometryColumn failed for layer %s, layer creation has failed.",
                      pszLayerName );

            CPLFree( pszLayerName );
            CPLFree( pszSchemaName );

            OGRPGClearResult( hResult );

            hResult = PQexec(hPGConn, "ROLLBACK");
            OGRPGClearResult( hResult );

            return NULL;
        }

        OGRPGClearResult( hResult );

/* -------------------------------------------------------------------- */
/*      Create the spatial index.                                       */
/*                                                                      */
/*      We're doing this before we add geometry and record to the table */
/*      so this may not be exactly the best way to do it.               */
/* -------------------------------------------------------------------- */
        const char *pszSI = CSLFetchNameValue( papszOptions, "SPATIAL_INDEX" );
        if( pszSI == NULL || CSLTestBoolean(pszSI) )
        {
            sprintf( szCommand, "CREATE INDEX %s_geom_idx ON \"%s\".\"%s\" USING GIST (\"%s\")",
                    pszTableName, pszSchemaName, pszTableName, pszGFldName);

            CPLDebug( "OGR_PG", "PQexec(%s)", szCommand );
            hResult = PQexec(hPGConn, szCommand);

            if( !hResult
                || PQresultStatus(hResult) != PGRES_COMMAND_OK )
            {
                CPLError( CE_Failure, CPLE_AppDefined,
                        "'%s' failed for layer %s, layer creation has failed.",
                        szCommand, pszLayerName );

                CPLFree( pszLayerName );
                CPLFree( pszSchemaName );

                OGRPGClearResult( hResult );

                hResult = PQexec(hPGConn, "ROLLBACK");
                OGRPGClearResult( hResult );

                return NULL;
            }
            OGRPGClearResult( hResult );
        }
    }

/* -------------------------------------------------------------------- */
/*      Complete, and commit the transaction.                           */
/* -------------------------------------------------------------------- */
    hResult = PQexec(hPGConn, "COMMIT");
    OGRPGClearResult( hResult );

/* -------------------------------------------------------------------- */
/*      Create the layer object.                                        */
/* -------------------------------------------------------------------- */
    OGRPGTableLayer     *poLayer;

    poLayer = new OGRPGTableLayer( this, pszTableName, pszSchemaName, TRUE, nSRSId );

    poLayer->SetLaunderFlag( CSLFetchBoolean(papszOptions,"LAUNDER",TRUE) );
    poLayer->SetPrecisionFlag( CSLFetchBoolean(papszOptions,"PRECISION",TRUE));

/* -------------------------------------------------------------------- */
/*      Add layer to data source layer list.                            */
/* -------------------------------------------------------------------- */
    papoLayers = (OGRPGTableLayer **)
        CPLRealloc( papoLayers,  sizeof(OGRPGTableLayer *) * (nLayers+1) );

    papoLayers[nLayers++] = poLayer;

    CPLFree( pszLayerName );
    CPLFree( pszSchemaName );

    return poLayer;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRPGDataSource::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,ODsCCreateLayer) 
        || EQUAL(pszCap,ODsCDeleteLayer) )
        return TRUE;
    else
        return FALSE;
}

/************************************************************************/
/*                              GetLayer()                              */
/************************************************************************/

OGRLayer *OGRPGDataSource::GetLayer( int iLayer )

{
    if( iLayer < 0 || iLayer >= nLayers )
        return NULL;
    else
        return papoLayers[iLayer];
}

/************************************************************************/
/*                           GetLayerByName()                           */
/************************************************************************/

OGRLayer *OGRPGDataSource::GetLayerByName( const char *pszName )

{
    if ( ! pszName )
        return NULL;

    int  i;
    
    int count = GetLayerCount();
    /* first a case sensitive check */
    for( i = 0; i < count; i++ )
    {
        OGRPGTableLayer *poLayer = papoLayers[i];

        if( strcmp( pszName, poLayer->GetLayerDefn()->GetName() ) == 0 )
            return poLayer;
    }
        
    /* then case insensitive */
    for( i = 0; i < count; i++ )
    {
        OGRPGTableLayer *poLayer = papoLayers[i];

        if( EQUAL( pszName, poLayer->GetLayerDefn()->GetName() ) )
            return poLayer;
    }
    
    if( OpenTable( pszName, NULL, TRUE, FALSE ) )
        return GetLayer(count);
    else
        return NULL;
}


/************************************************************************/
/*                        OGRPGNoticeProcessor()                        */
/************************************************************************/

static void OGRPGNoticeProcessor( void *arg, const char * pszMessage )

{
    CPLDebug( "OGR_PG_NOTICE", "%s", pszMessage );
}

/************************************************************************/
/*                      InitializeMetadataTables()                      */
/*                                                                      */
/*      Create the metadata tables (SPATIAL_REF_SYS and                 */
/*      GEOMETRY_COLUMNS).                                              */
/************************************************************************/

OGRErr OGRPGDataSource::InitializeMetadataTables()

{
    // implement later.
    return OGRERR_FAILURE;
}

/************************************************************************/
/*                              FetchSRS()                              */
/*                                                                      */
/*      Return a SRS corresponding to a particular id.  Note that       */
/*      reference counting should be honoured on the returned           */
/*      OGRSpatialReference, as handles may be cached.                  */
/************************************************************************/

OGRSpatialReference *OGRPGDataSource::FetchSRS( int nId )

{
    if( nId < 0 )
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
    PGresult        *hResult = NULL;
    char            szCommand[1024];
    OGRSpatialReference *poSRS = NULL;

    SoftStartTransaction();

    sprintf( szCommand,
             "SELECT srtext FROM spatial_ref_sys "
             "WHERE srid = %d",
             nId );
    hResult = PQexec(hPGConn, szCommand );

    if( hResult
        && PQresultStatus(hResult) == PGRES_TUPLES_OK
        && PQntuples(hResult) == 1 )
    {
        char *pszWKT;

        pszWKT = PQgetvalue(hResult,0,0);
        poSRS = new OGRSpatialReference();
        if( poSRS->importFromWkt( &pszWKT ) != OGRERR_NONE )
        {
            delete poSRS;
            poSRS = NULL;
        }
    }

    OGRPGClearResult( hResult );
    SoftCommit();

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

/************************************************************************/
/*                             FetchSRSId()                             */
/*                                                                      */
/*      Fetch the id corresponding to an SRS, and if not found, add     */
/*      it to the table.                                                */
/************************************************************************/

int OGRPGDataSource::FetchSRSId( OGRSpatialReference * poSRS )

{
    PGresult            *hResult = NULL;
    char                szCommand[10000];
    char                *pszWKT = NULL;
    int                 nSRSId = -1;
    const char*         pszAuthorityName;

    if( poSRS == NULL )
        return -1;

    pszAuthorityName = poSRS->GetAuthorityName(NULL);

/* -------------------------------------------------------------------- */
/*      Check whether the EPSG authority code is already mapped to a    */
/*      SRS ID.                                                         */
/* -------------------------------------------------------------------- */
    if( pszAuthorityName != NULL && EQUAL( pszAuthorityName, "EPSG" ) )
    {
        int             nAuthorityCode;

        /* For the root authority name 'EPSG', the authority code
         * should always be integral
         */
        nAuthorityCode = atoi( poSRS->GetAuthorityCode(NULL) );

        sprintf( szCommand, "SELECT srid FROM spatial_ref_sys WHERE "
                            "auth_name = '%s' AND auth_srid = %d",
                            pszAuthorityName,
                            nAuthorityCode );
        hResult = PQexec(hPGConn, szCommand);

        if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK
            && PQntuples(hResult) > 0 )
        {
            nSRSId = atoi(PQgetvalue( hResult, 0, 0 ));

            OGRPGClearResult( hResult );

            return nSRSId;
        }

        OGRPGClearResult( hResult );
    }

/* -------------------------------------------------------------------- */
/*      Translate SRS to WKT.                                           */
/* -------------------------------------------------------------------- */
    if( poSRS->exportToWkt( &pszWKT ) != OGRERR_NONE )
        return -1;

    CPLAssert( strlen(pszWKT) < sizeof(szCommand) - 500 );


/* -------------------------------------------------------------------- */
/*      Try to find in the existing table.                              */
/* -------------------------------------------------------------------- */
    hResult = PQexec(hPGConn, "BEGIN");
    OGRPGClearResult( hResult );

    sprintf( szCommand,
             "SELECT srid FROM spatial_ref_sys WHERE srtext = '%s'",
             pszWKT );
    hResult = PQexec(hPGConn, szCommand );
    CPLFree( pszWKT );  // CM:  Added to prevent mem leaks
    pszWKT = NULL;      // CM:  Added

/* -------------------------------------------------------------------- */
/*      We got it!  Return it.                                          */
/* -------------------------------------------------------------------- */
    if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK
        && PQntuples(hResult) > 0 )
    {
        nSRSId = atoi(PQgetvalue( hResult, 0, 0 ));

        OGRPGClearResult( hResult );

        hResult = PQexec(hPGConn, "COMMIT");
        OGRPGClearResult( hResult );

        return nSRSId;
    }

/* -------------------------------------------------------------------- */
/*      If the command actually failed, then the metadata table is      */
/*      likely missing. Try defining it.                                */
/* -------------------------------------------------------------------- */
    int         bTableMissing;

    bTableMissing =
        hResult == NULL || PQresultStatus(hResult) == PGRES_NONFATAL_ERROR;

    OGRPGClearResult( hResult );

    hResult = PQexec(hPGConn, "COMMIT");
    OGRPGClearResult( hResult );

    if( bTableMissing )
    {
        if( InitializeMetadataTables() != OGRERR_NONE )
            return -1;
    }

/* -------------------------------------------------------------------- */
/*      Get the current maximum srid in the srs table.                  */
/* -------------------------------------------------------------------- */
    hResult = PQexec(hPGConn, "BEGIN");
    OGRPGClearResult( hResult );

    hResult = PQexec(hPGConn, "SELECT MAX(srid) FROM spatial_ref_sys" );

    if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK )
    {
        nSRSId = atoi(PQgetvalue(hResult,0,0)) + 1;
        OGRPGClearResult( hResult );
    }
    else
    {
        nSRSId = 1;
    }

/* -------------------------------------------------------------------- */
/*      Try adding the SRS to the SRS table.                            */
/* -------------------------------------------------------------------- */
    if( poSRS->exportToWkt( &pszWKT ) != OGRERR_NONE )
    {
        return -1;
    }

    CPLAssert( strlen(pszWKT) < sizeof(szCommand) - 500 );

    char    *pszProj4 = NULL;
    if( poSRS->exportToProj4( &pszProj4 ) != OGRERR_NONE )
    {
        CPLFree( pszWKT );  // Prevent mem leaks
        pszWKT = NULL;
		
        return -1;
    }

    if( pszAuthorityName != NULL && EQUAL(pszAuthorityName, "EPSG") )
    {
        int             nAuthorityCode;

        nAuthorityCode = atoi( poSRS->GetAuthorityCode(NULL) );

        sprintf( szCommand,
                 "INSERT INTO spatial_ref_sys (srid,srtext,proj4text,auth_name,auth_srid) "
                 "VALUES (%d, '%s', '%s', '%s', %d)",
                 nSRSId, pszWKT, pszProj4, pszAuthorityName,
                 nAuthorityCode );
    }
    else
    {
        sprintf( szCommand,
                 "INSERT INTO spatial_ref_sys (srid,srtext,proj4text) VALUES (%d,'%s','%s')",
                 nSRSId, pszWKT, pszProj4 );
    }

    // Free everything that was allocated.
    CPLFree( pszProj4 );
    CPLFree( pszWKT);

    hResult = PQexec(hPGConn, szCommand );
    OGRPGClearResult( hResult );

    hResult = PQexec(hPGConn, "COMMIT");
    OGRPGClearResult( hResult );

    return nSRSId;
}

/************************************************************************/
/*                        SoftStartTransaction()                        */
/*                                                                      */
/*      Create a transaction scope.  If we already have a               */
/*      transaction active this isn't a real transaction, but just      */
/*      an increment to the scope count.                                */
/************************************************************************/

OGRErr OGRPGDataSource::SoftStartTransaction()

{
    nSoftTransactionLevel++;

    if( nSoftTransactionLevel == 1 )
    {
        PGresult    *hResult = NULL;
        PGconn      *hPGConn = GetPGConn();

        //CPLDebug( "OGR_PG", "BEGIN Transaction" );
        hResult = PQexec(hPGConn, "BEGIN");

        if( !hResult || PQresultStatus(hResult) != PGRES_COMMAND_OK )
        {
            OGRPGClearResult( hResult );

            CPLDebug( "OGR_PG", "BEGIN Transaction failed:\n%s",
                      PQerrorMessage( hPGConn ) );
            return OGRERR_FAILURE;
        }

        OGRPGClearResult( hResult );
    }

    return OGRERR_NONE;
}

/************************************************************************/
/*                             SoftCommit()                             */
/*                                                                      */
/*      Commit the current transaction if we are at the outer           */
/*      scope.                                                          */
/************************************************************************/

OGRErr OGRPGDataSource::SoftCommit()

{
    EndCopy();

    if( nSoftTransactionLevel <= 0 )
    {
        CPLDebug( "OGR_PG", "SoftCommit() with no transaction active." );
        return OGRERR_FAILURE;
    }

    nSoftTransactionLevel--;

    if( nSoftTransactionLevel == 0 )
    {
        PGresult    *hResult = NULL;
        PGconn      *hPGConn = GetPGConn();

        //CPLDebug( "OGR_PG", "COMMIT Transaction" );
        hResult = PQexec(hPGConn, "COMMIT");

        if( !hResult || PQresultStatus(hResult) != PGRES_COMMAND_OK )
        {
            OGRPGClearResult( hResult );

            CPLDebug( "OGR_PG", "COMMIT Transaction failed:\n%s",
                      PQerrorMessage( hPGConn ) );
            return OGRERR_FAILURE;
        }
        
        OGRPGClearResult( hResult );
    }

    return OGRERR_NONE;
}

/************************************************************************/
/*                            SoftRollback()                            */
/*                                                                      */
/*      Force a rollback of the current transaction if there is one,    */
/*      even if we are nested several levels deep.                      */
/************************************************************************/

OGRErr OGRPGDataSource::SoftRollback()

{
    if( nSoftTransactionLevel <= 0 )
    {
        CPLDebug( "OGR_PG", "SoftRollback() with no transaction active." );
        return OGRERR_FAILURE;
    }

    nSoftTransactionLevel = 0;

    PGresult    *hResult = NULL;
    PGconn      *hPGConn = GetPGConn();

    hResult = PQexec(hPGConn, "ROLLBACK");

    if( !hResult || PQresultStatus(hResult) != PGRES_COMMAND_OK )
    {
        OGRPGClearResult( hResult );

        return OGRERR_FAILURE;
    }

    OGRPGClearResult( hResult );

    return OGRERR_NONE;
}

/************************************************************************/
/*                        FlushSoftTransaction()                        */
/*                                                                      */
/*      Force the unwinding of any active transaction, and it's         */
/*      commit.                                                         */
/************************************************************************/

OGRErr OGRPGDataSource::FlushSoftTransaction()

{
    /* This must come first because of ogr2ogr.  If you want
       to use ogr2ogr with COPY support, then you must specify
       that ogr2ogr does not use transactions.  Thus, 
       nSoftTransactionLevel will always be zero, so this has
       to come first. */
    EndCopy(); 

    if( nSoftTransactionLevel <= 0 )
        return OGRERR_NONE;

    nSoftTransactionLevel = 1;

    return SoftCommit();
}

/************************************************************************/
/*                             ExecuteSQL()                             */
/************************************************************************/

OGRLayer * OGRPGDataSource::ExecuteSQL( const char *pszSQLCommand,
                                        OGRGeometry *poSpatialFilter,
                                        const char *pszDialect )

{
    if( poSpatialFilter != NULL )
    {
        CPLDebug( "OGR_PG",
          "Spatial filter ignored for now in OGRPGDataSource::ExecuteSQL()" );
    }

/* -------------------------------------------------------------------- */
/*      Use generic implementation for OGRSQL dialect.                  */
/* -------------------------------------------------------------------- */
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
        
        for( int iLayer = 0; iLayer < nLayers; iLayer++ )
        {
            if( EQUAL(papoLayers[iLayer]->GetLayerDefn()->GetName(), 
                      pszLayerName ))
            {
                DeleteLayer( iLayer );
                break;
            }
        }
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Execute the statement.                                          */
/* -------------------------------------------------------------------- */
    PGresult    *hResult = NULL;

    FlushSoftTransaction();

    if( SoftStartTransaction() == OGRERR_NONE  )
    {
        CPLDebug( "OGR_PG", "PQexec(%s)", pszSQLCommand );
        hResult = PQexec(hPGConn, pszSQLCommand );
        CPLDebug( "OGR_PG", "Command Results Tuples = %d", PQntuples(hResult) );
    }

/* -------------------------------------------------------------------- */
/*      Do we have a tuple result? If so, instantiate a results         */
/*      layer for it.                                                   */
/* -------------------------------------------------------------------- */

    if( hResult && PQresultStatus(hResult) == PGRES_TUPLES_OK
        && PQntuples(hResult) > 0 )
    {
        OGRPGResultLayer *poLayer = NULL;

        poLayer = new OGRPGResultLayer( this, pszSQLCommand, hResult );

        return poLayer;
    }

/* -------------------------------------------------------------------- */
/*      Generate an error report if an error occured.                   */
/* -------------------------------------------------------------------- */
    if( hResult &&
        (PQresultStatus(hResult) == PGRES_NONFATAL_ERROR
         || PQresultStatus(hResult) == PGRES_FATAL_ERROR ) )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "%s", PQresultErrorMessage( hResult ) );
    }

    OGRPGClearResult( hResult );

    FlushSoftTransaction();

    return NULL;
}

/************************************************************************/
/*                          ReleaseResultSet()                          */
/************************************************************************/

void OGRPGDataSource::ReleaseResultSet( OGRLayer * poLayer )

{
    delete poLayer;
}

/************************************************************************/
/*                            LaunderName()                             */
/************************************************************************/

char *OGRPGDataSource::LaunderName( const char *pszSrcName )

{
    char    *pszSafeName = CPLStrdup( pszSrcName );

    for( int i = 0; pszSafeName[i] != '\0'; i++ )
    {
        pszSafeName[i] = (char) tolower( pszSafeName[i] );
        if( pszSafeName[i] == '-' || pszSafeName[i] == '#' )
            pszSafeName[i] = '_';
    }

    CPLDebug("OGR_PG","LaunderName( %s ) result: %s", pszSrcName, pszSafeName);

    return pszSafeName;
}

/************************************************************************/
/*                             StartCopy()                              */
/************************************************************************/
void OGRPGDataSource::StartCopy( OGRPGTableLayer *poPGLayer )
{
    EndCopy();
    poLayerInCopyMode = poPGLayer;
}

/************************************************************************/
/*                              EndCopy()                               */
/************************************************************************/
OGRErr OGRPGDataSource::EndCopy( )
{
    if( poLayerInCopyMode != NULL )
    {
        OGRErr result = poLayerInCopyMode->EndCopy();
        poLayerInCopyMode = NULL;

        return result;
    }
    else
        return OGRERR_NONE;
}

/************************************************************************/
/*                           CopyInProgress()                           */
/************************************************************************/
int OGRPGDataSource::CopyInProgress( )
{
    return ( poLayerInCopyMode != NULL );
}
