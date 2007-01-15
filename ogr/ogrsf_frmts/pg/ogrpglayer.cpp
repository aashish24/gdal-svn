/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements OGRPGLayer class  which implements shared handling
 *           of feature geometry and so forth needed by OGRPGResultLayer and
 *           OGRPGTableLayer.
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
 ******************************************************************************
 *
 * $Log$
 * Revision 1.32  2006/11/30 05:01:43  fwarmerdam
 * Added support for EWKB (the default geometry format) in HEXToGeometry().
 *
 * Revision 1.31  2006/04/05 03:15:13  fwarmerdam
 * use CPLString for command buffer to handle large commands
 *
 * Revision 1.30  2006/04/02 18:47:39  fwarmerdam
 * added detailed date type support
 *
 * Revision 1.29  2006/02/15 04:26:17  fwarmerdam
 * added date support
 *
 * Revision 1.28  2006/01/27 00:10:32  fwarmerdam
 * added Get{FID,Geometry}Column() support
 *
 * Revision 1.27  2006/01/16 15:36:33  fwarmerdam
 * disable PQfformat calls in pre7.4 client libs
 *
 * Revision 1.26  2005/12/16 07:42:08  osemykin
 * Added regular data fields support for binary cursor
 *
 * Revision 1.25  2005/11/18 12:42:55  dron
 * Handle varchar arrays when reading feature definition fields.
 *
 * Revision 1.24  2005/10/24 23:50:16  fwarmerdam
 * fixed extraction of geomType in GeometryToHex()
 *
 * Revision 1.23  2005/10/16 01:38:34  cfis
 * Updates that add support for using COPY for inserting data to Postgresql.  COPY is less robust than INSERT, but signficantly faster.
 *
 * Revision 1.22  2005/09/30 19:11:16  fwarmerdam
 * Fixed Hextobinary conversion (a and A mistranslated).
 *
 * Revision 1.21  2005/09/21 00:55:42  fwarmerdam
 * fixup OGRFeatureDefn and OGRSpatialReference refcount handling
 *
 * Revision 1.20  2005/08/06 14:49:27  osemykin
 * Added BINARY CURSOR support
 * Use it with 'PGB:dbname=...' instead 'PG:dbname=...'
 *
 * Revision 1.19  2005/07/20 01:45:01  fwarmerdam
 * added PostGIS 8.0 hex geometry format support
 *
 * Revision 1.18  2005/05/05 20:47:52  dron
 * Override GetExtent() method for PostGIS layers with PostGIS standard function
 * extent() (Oleg Semykin <oleg.semykin@gmail.com>
 *
 * Revision 1.17  2005/02/22 12:54:05  fwarmerdam
 * use OGRLayer base spatial filter support
 *
 * Revision 1.16  2005/02/02 20:54:27  fwarmerdam
 * track m_nFeaturesRead
 *
 * Revision 1.15  2004/07/10 04:46:24  warmerda
 * initialize nResultOffset, use soft transactions
 *
 * Revision 1.14  2004/05/08 02:14:49  warmerda
 * added GetFeature() on table, generalize FID support a bit
 *
 * Revision 1.13  2003/05/21 03:59:42  warmerda
 * expand tabs
 *
 * Revision 1.12  2003/02/01 07:55:48  warmerda
 * avoid dependence on libpq-fs.h
 *
 * Revision 1.11  2003/01/08 22:07:14  warmerda
 * Added support for integer and real list field types
 *
 * Revision 1.10  2002/05/09 16:03:19  warmerda
 * major upgrade to support SRS better and add ExecuteSQL
 *
 * Revision 1.9  2001/11/15 21:19:47  warmerda
 * added soft transaction semantics, handle null fields properly
 *
 * Revision 1.8  2001/11/15 16:10:12  warmerda
 * fixed some escaping issues with string field values
 *
 * Revision 1.7  2001/09/28 04:03:52  warmerda
 * partially upraded to PostGIS 0.6
 *
 * Revision 1.6  2001/07/18 04:55:16  warmerda
 * added CPL_CSVID
 *
 * Revision 1.5  2001/06/26 20:59:13  warmerda
 * implement efficient spatial and attribute query support
 *
 * Revision 1.4  2001/06/19 22:29:12  warmerda
 * upgraded to include PostGIS support
 *
 * Revision 1.3  2001/06/19 15:50:23  warmerda
 * added feature attribute query support
 *
 * Revision 1.2  2000/11/23 06:03:35  warmerda
 * added Oid support
 *
 * Revision 1.1  2000/10/17 17:46:51  warmerda
 * New
 *
 */

#include "cpl_conv.h"
#include "ogr_pg.h"
#include "cpl_string.h"

CPL_CVSID("$Id$");

#define CURSOR_PAGE     1

// These originally are defined in libpq-fs.h.

#ifndef INV_WRITE
#define INV_WRITE               0x00020000
#define INV_READ                0x00040000
#endif

/* Flags for creating WKB format for PostGIS */
#define WKBZOFFSET 0x80000000
#define WKBMOFFSET 0x40000000
#define WKBSRIDFLAG 0x20000000
#define WKBBBOXFLAG 0x10000000

/************************************************************************/
/*                           OGRPGLayer()                               */
/************************************************************************/

OGRPGLayer::OGRPGLayer()

{
    poDS = NULL;

    bHasWkb = FALSE;
    bWkbAsOid = FALSE;
    bHasPostGISGeometry = FALSE;
    pszGeomColumn = NULL;
    pszQueryStatement = NULL;

    bHasFid = FALSE;
    pszFIDColumn = NULL;

    iNextShapeId = 0;
    nResultOffset = 0;

    poSRS = NULL;
    nSRSId = -2; // we haven't even queried the database for it yet.

    /* Eventually we may need to make these a unique name */
    pszCursorName = "OGRPGLayerReader";
    hCursorResult = NULL;
    bCursorActive = FALSE;

    poFeatureDefn = NULL;
}

/************************************************************************/
/*                            ~OGRPGLayer()                             */
/************************************************************************/

OGRPGLayer::~OGRPGLayer()

{
    if( m_nFeaturesRead > 0 && poFeatureDefn != NULL )
    {
        CPLDebug( "PG", "%d features read on layer '%s'.",
                  (int) m_nFeaturesRead,
                  poFeatureDefn->GetName() );
    }

    ResetReading();

    CPLFree( pszGeomColumn );
    CPLFree( pszFIDColumn );
    CPLFree( pszQueryStatement );

    if( poSRS != NULL )
        poSRS->Release();

    if( poFeatureDefn )
        poFeatureDefn->Release();
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRPGLayer::ResetReading()

{
    PGconn      *hPGConn = poDS->GetPGConn();
    char        szCommand[1024];

    iNextShapeId = 0;

    if( hCursorResult != NULL )
    {
        PQclear( hCursorResult );

        if( bCursorActive )
        {
            sprintf( szCommand, "CLOSE %s", pszCursorName );

            hCursorResult = PQexec(hPGConn, szCommand);
            PQclear( hCursorResult );
        }

        poDS->FlushSoftTransaction();

        hCursorResult = NULL;
    }
}

/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRPGLayer::GetNextFeature()

{

    for( ; TRUE; )
    {
        OGRFeature      *poFeature;

        poFeature = GetNextRawFeature();
        if( poFeature == NULL )
            return NULL;

        if( (m_poFilterGeom == NULL
            || bHasPostGISGeometry
            || !FilterGeometry( poFeature->GetGeometryRef() ) )
            && (m_poAttrQuery == NULL
                || m_poAttrQuery->Evaluate( poFeature )) )
            return poFeature;

        delete poFeature;
    }
}
/************************************************************************/
/*                          RecordToFeature()                           */
/*                                                                      */
/*      Convert the indicated record of the current result set into     */
/*      a feature.                                                      */
/************************************************************************/

OGRFeature *OGRPGLayer::RecordToFeature( int iRecord )

{
/* -------------------------------------------------------------------- */
/*      Create a feature from the current result.                       */
/* -------------------------------------------------------------------- */
    int         iField;
    OGRFeature *poFeature = new OGRFeature( poFeatureDefn );

    poFeature->SetFID( iNextShapeId );
    m_nFeaturesRead++;

/* ==================================================================== */
/*      Transfer all result fields we can.                              */
/* ==================================================================== */
    for( iField = 0;
         iField < PQnfields(hCursorResult);
         iField++ )
    {
        int     iOGRField;

/* -------------------------------------------------------------------- */
/*      Handle FID.                                                     */
/* -------------------------------------------------------------------- */
        if( bHasFid && EQUAL(PQfname(hCursorResult,iField),pszFIDColumn) )
            poFeature->SetFID( atoi(PQgetvalue(hCursorResult,iRecord,iField)));

/* -------------------------------------------------------------------- */
/*      Handle PostGIS geometry                                         */
/* -------------------------------------------------------------------- */
        if( bHasPostGISGeometry
            && (EQUAL(PQfname(hCursorResult,iField),pszGeomColumn)
                || EQUAL(PQfname(hCursorResult,iField),"asEWKT")
                || EQUAL(PQfname(hCursorResult,iField),"asText") ) )
        {
            char        *pszWKT;
            char        *pszPostSRID;
            OGRGeometry *poGeometry = NULL;

            pszWKT = PQgetvalue( hCursorResult, iRecord, iField );
            pszPostSRID = pszWKT;

            // optionally strip off PostGIS SRID identifier.  This
            // happens if we got a raw geometry field.
            if( EQUALN(pszPostSRID,"SRID=",5) )
            {
                while( *pszPostSRID != '\0' && *pszPostSRID != ';' )
                    pszPostSRID++;
                if( *pszPostSRID == ';' )
                    pszPostSRID++;
            }

            if( EQUALN(pszPostSRID,"00",2) || EQUALN(pszPostSRID,"01",2) )
            {
                poGeometry =
                    HEXToGeometry(
                        PQgetvalue( hCursorResult, iRecord, iField ) );
            }
            else
                OGRGeometryFactory::createFromWkt( &pszPostSRID, NULL,
                                                   &poGeometry );
            if( poGeometry != NULL )
                poFeature->SetGeometryDirectly( poGeometry );

            continue;
        }
/* -------------------------------------------------------------------- */
/*      Handle raw binary geometry ... this hasn't been tested in a     */
/*      while.                                                          */
/* -------------------------------------------------------------------- */
        else if( EQUAL(PQfname(hCursorResult,iField),"WKB_GEOMETRY") )
        {
            if( bWkbAsOid )
            {
                poFeature->SetGeometryDirectly(
                    OIDToGeometry( (Oid) atoi(
                        PQgetvalue( hCursorResult,
                                    iRecord, iField ) ) ) );
            }
            else
            {
                poFeature->SetGeometryDirectly(
                    BYTEAToGeometry(
                        PQgetvalue( hCursorResult,
                                    iRecord, iField ) ) );
            }
            continue;
        }
        /* Handle binary cursor result */
        else if ( EQUAL(PQfname(hCursorResult,iField),"AsBinary") )
        {
            GByte * pabyWkb = (GByte *)PQgetvalue( hCursorResult,
                                                   iRecord, iField);
            OGRGeometry * poGeom = NULL;
            OGRGeometryFactory::createFromWkb(pabyWkb,NULL,&poGeom);
            poFeature->SetGeometryDirectly( poGeom );
            continue;
        }

/* -------------------------------------------------------------------- */
/*      Transfer regular data fields.                                   */
/* -------------------------------------------------------------------- */
        iOGRField =
            poFeatureDefn->GetFieldIndex(PQfname(hCursorResult,iField));

        if( iOGRField < 0 )
            continue;

        if( PQgetisnull( hCursorResult, iRecord, iField ) )
            continue;

        OGRFieldType eOGRType = 
            poFeatureDefn->GetFieldDefn(iOGRField)->GetType();

        if( eOGRType == OFTIntegerList)
        {
            int *panList, nCount, i;

#if !defined(PG_PRE74)
            if ( PQfformat( hCursorResult, iField ) == 1 ) // Binary data representation
            {
                char * pData = PQgetvalue( hCursorResult, iRecord, iField );

                // goto number of array elements
                pData += 3 * sizeof(int);
                memcpy( &nCount, pData, sizeof(int) );
                CPL_MSBPTR32( &nCount );

                panList = (int *) CPLCalloc(sizeof(int),nCount);

                // goto first array element
                pData += 2 * sizeof(int);

                for( i = 0; i < nCount; i++ )
                {
                    // get element size
                    int nSize = *(int *)(pData);
                    CPL_MSBPTR32( &nSize );

                    CPLAssert( nSize == sizeof(int) );

                    pData += sizeof(int);

                    memcpy( &panList[i], pData, nSize );
                    CPL_MSBPTR32(&panList[i]);

                    pData += nSize;
                }
            }
            else
#endif /* notdef PG_PRE74 */
            {
                char **papszTokens;
                papszTokens = CSLTokenizeStringComplex(
                    PQgetvalue( hCursorResult, iRecord, iField ),
                    "{,}", FALSE, FALSE );

                nCount = CSLCount(papszTokens);
                panList = (int *) CPLCalloc(sizeof(int),nCount);

                for( i = 0; i < nCount; i++ )
                    panList[i] = atoi(papszTokens[i]);
                CSLDestroy( papszTokens );
            }
            poFeature->SetField( iOGRField, nCount, panList );
            CPLFree( panList );
        }

        else if( eOGRType == OFTRealList )
        {
            int nCount, i;
            double *padfList;

#if !defined(PG_PRE74)
            if ( PQfformat( hCursorResult, iField ) == 1 ) // Binary data representation
            {
                char * pData = PQgetvalue( hCursorResult, iRecord, iField );

                // goto number of array elements
                pData += 3 * sizeof(int);
                memcpy( &nCount, pData, sizeof(int) );
                CPL_MSBPTR32( &nCount );

                padfList = (double *) CPLCalloc(sizeof(double),nCount);

                // goto first array element
                pData += 2 * sizeof(int);

                for( i = 0; i < nCount; i++ )
                {
                    // get element size
                    int nSize = *(int *)(pData);
                    CPL_MSBPTR32( &nSize );

                    CPLAssert( nSize == sizeof(double) );

                    pData += sizeof(int);

                    memcpy( &padfList[i], pData, nSize );
                    CPL_MSBPTR64(&padfList[i]);

                    pData += nSize;
                }
            }
            else
#endif /* notdef PG_PRE74 */
            {
                char **papszTokens;
                papszTokens = CSLTokenizeStringComplex(
                    PQgetvalue( hCursorResult, iRecord, iField ),
                    "{,}", FALSE, FALSE );

                nCount = CSLCount(papszTokens);
                padfList = (double *) CPLCalloc(sizeof(double),nCount);

                for( i = 0; i < nCount; i++ )
                    padfList[i] = atof(papszTokens[i]);
                CSLDestroy( papszTokens );
            }

            poFeature->SetField( iOGRField, nCount, padfList );
            CPLFree( padfList );
        }

        else if( eOGRType == OFTStringList )
        {
            char **papszTokens = 0;

#if !defined(PG_PRE74)
            if ( PQfformat( hCursorResult, iField ) == 1 ) // Binary data representation
            {
                char * pData = PQgetvalue( hCursorResult, iRecord, iField );
                int nCount, i;

                // goto number of array elements
                pData += 3 * sizeof(int);
                memcpy( &nCount, pData, sizeof(int) );
                CPL_MSBPTR32( &nCount );

                // goto first array element
                pData += 2 * sizeof(int);

                for( i = 0; i < nCount; i++ )
                {
                    // get element size
                    int nSize = *(int *)(pData);
                    CPL_MSBPTR32( &nSize );

                    pData += sizeof(int);

                    papszTokens = CSLAddString(papszTokens, pData);

                    pData += nSize;
                }
            }
            else
#endif /* notdef PG_PRE74 */
            {
                papszTokens = CSLTokenizeStringComplex(
                        PQgetvalue( hCursorResult, iRecord, iField ),
                        "{,}", FALSE, FALSE );
            }

            if ( papszTokens )
            {
                poFeature->SetField( iOGRField, papszTokens );
                CSLDestroy( papszTokens );
            }
        }

        else if( eOGRType == OFTDate 
                 || eOGRType == OFTTime 
                 || eOGRType == OFTDateTime )
        {
#if !defined(PG_PRE74)
            if ( PQfformat( hCursorResult, iField ) == 1 ) // Binary data
            {
                CPLDebug( "PG", "Binary DATE format not yet implemented." );
            }
            else
#endif /* notdef PG_PRE74 */
            {
                OGRField  sFieldValue;

                if( OGRParseDate( PQgetvalue( hCursorResult, iRecord, iField ),
                                  &sFieldValue, 0 ) )
                {
                    poFeature->SetField( iOGRField, &sFieldValue );
                }
            }
        }
        else
        {
#if !defined(PG_PRE74)
            if ( PQfformat( hCursorResult, iField ) == 1 &&
                 eOGRType != OFTString ) // Binary data
            {
                if ( eOGRType == OFTInteger )
                {
                    int nVal;
                    memcpy( &nVal, PQgetvalue( hCursorResult, iRecord, iField ), sizeof(int) );
                    CPL_MSBPTR32(&nVal);
                    poFeature->SetField( iOGRField, nVal );
                }
                else if ( eOGRType == OFTReal )
                {
                    double dfVal;
                    memcpy( &dfVal, PQgetvalue( hCursorResult, iRecord, iField ), sizeof(double) );
                    CPL_MSBPTR64(&dfVal);
                    poFeature->SetField( iOGRField, dfVal );
                }
            }
            else
#endif /* notdef PG_PRE74 */
                poFeature->SetField( iOGRField,
                                     PQgetvalue( hCursorResult, iRecord, iField ) );
        }
    }

    return poFeature;
}

/************************************************************************/
/*                         GetNextRawFeature()                          */
/************************************************************************/

OGRFeature *OGRPGLayer::GetNextRawFeature()

{
    PGconn      *hPGConn = poDS->GetPGConn();
    CPLString   osCommand;

/* -------------------------------------------------------------------- */
/*      Do we need to establish an initial query?                       */
/* -------------------------------------------------------------------- */
    if( iNextShapeId == 0 && hCursorResult == NULL )
    {
        CPLAssert( pszQueryStatement != NULL );

        poDS->FlushSoftTransaction();
        poDS->SoftStartTransaction();

        if ( poDS->bUseBinaryCursor )
            osCommand.Printf( "DECLARE %s BINARY CURSOR for %s",
                              pszCursorName, pszQueryStatement );
        else
            osCommand.Printf( "DECLARE %s CURSOR for %s",
                              pszCursorName, pszQueryStatement );

        CPLDebug( "OGR_PG", "PQexec(%s)", osCommand.c_str() );

        hCursorResult = PQexec(hPGConn, osCommand );
        PQclear( hCursorResult );

        osCommand.Printf( "FETCH %d in %s", CURSOR_PAGE, pszCursorName );
        hCursorResult = PQexec(hPGConn, osCommand );

        bCursorActive = TRUE;

        nResultOffset = 0;
    }

/* -------------------------------------------------------------------- */
/*      Are we in some sort of error condition?                         */
/* -------------------------------------------------------------------- */
    if( hCursorResult == NULL
        || PQresultStatus(hCursorResult) != PGRES_TUPLES_OK )
    {
        iNextShapeId = MAX(1,iNextShapeId);
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Do we need to fetch more records?                               */
/* -------------------------------------------------------------------- */
    if( nResultOffset >= PQntuples(hCursorResult)
        && bCursorActive )
    {
        PQclear( hCursorResult );

        osCommand.Printf( "FETCH %d in %s", CURSOR_PAGE, pszCursorName );
        hCursorResult = PQexec(hPGConn, osCommand );

        nResultOffset = 0;
    }

/* -------------------------------------------------------------------- */
/*      Are we out of results?  If so complete the transaction, and     */
/*      cleanup, but don't reset the next shapeid.                      */
/* -------------------------------------------------------------------- */
    if( nResultOffset >= PQntuples(hCursorResult) )
    {
        PQclear( hCursorResult );

        if( bCursorActive )
        {
            osCommand.Printf( "CLOSE %s", pszCursorName );

            hCursorResult = PQexec(hPGConn, osCommand);
            PQclear( hCursorResult );
        }

        poDS->FlushSoftTransaction();

        hCursorResult = NULL;
        bCursorActive = FALSE;

        iNextShapeId = MAX(1,iNextShapeId);

        return NULL;
    }


/* -------------------------------------------------------------------- */
/*      Create a feature from the current result.                       */
/* -------------------------------------------------------------------- */
    OGRFeature *poFeature = RecordToFeature( nResultOffset );

    nResultOffset++;
    iNextShapeId++;

    return poFeature;
}

/************************************************************************/
/*                             GetFeature()                             */
/************************************************************************/

OGRFeature *OGRPGLayer::GetFeature( long nFeatureId )

{
    /* This should be implemented! */

    return NULL;
}

/************************************************************************/
/*                           HEXToGeometry()                            */
/************************************************************************/

OGRGeometry *OGRPGLayer::HEXToGeometry( const char *pszBytea )

{
    GByte       *pabyWKB;
    int iSrc=0, iDst=0;
    OGRGeometry *poGeometry;

    if( pszBytea == NULL )
        return NULL;

/* -------------------------------------------------------------------- */
/*      Convert hex to binary.                                          */
/* -------------------------------------------------------------------- */
    pabyWKB = (GByte *) CPLMalloc(strlen(pszBytea)+1);
    while( pszBytea[iSrc] != '\0' )
    {
        if( pszBytea[iSrc] >= '0' && pszBytea[iSrc] <= '9' )
            pabyWKB[iDst] = pszBytea[iSrc] - '0';
        else if( pszBytea[iSrc] >= 'A' && pszBytea[iSrc] <= 'F' )
            pabyWKB[iDst] = pszBytea[iSrc] - 'A' + 10;
        else if( pszBytea[iSrc] >= 'a' && pszBytea[iSrc] <= 'f' )
            pabyWKB[iDst] = pszBytea[iSrc] - 'a' + 10;
        else
            pabyWKB[iDst] = 0;

        pabyWKB[iDst] *= 16;

        iSrc++;

        if( pszBytea[iSrc] >= '0' && pszBytea[iSrc] <= '9' )
            pabyWKB[iDst] += pszBytea[iSrc] - '0';
        else if( pszBytea[iSrc] >= 'A' && pszBytea[iSrc] <= 'F' )
            pabyWKB[iDst] += pszBytea[iSrc] - 'A' + 10;
        else if( pszBytea[iSrc] >= 'a' && pszBytea[iSrc] <= 'f' )
            pabyWKB[iDst] += pszBytea[iSrc] - 'a' + 10;
        else
            pabyWKB[iDst] += 0;

        iSrc++;
        iDst++;
    }

/* -------------------------------------------------------------------- */
/*      PostGIS EWKB format includes an  SRID, but this won't be        */
/*      understood by OGR, so if the SRID flag is set, we remove the    */
/*      SRID (bytes at offset 5 to 8).                                  */
/* -------------------------------------------------------------------- */
    if( (pabyWKB[0] == 0 /* big endian */ && (pabyWKB[1] & 0x20) )
        || (pabyWKB[0] != 0 /* little endian */ && (pabyWKB[4] & 0x20)) )
    {
        memmove( pabyWKB+5, pabyWKB+9, iDst-9 );
        iDst -= 4;
        if( pabyWKB[0] == 0 )
            pabyWKB[1] &= (~0x20);
        else
            pabyWKB[4] &= (~0x20);
    }

/* -------------------------------------------------------------------- */
/*      Try to ingest the geometry.                                     */
/* -------------------------------------------------------------------- */
    poGeometry = NULL;
    OGRGeometryFactory::createFromWkb( pabyWKB, NULL, &poGeometry, iDst );

    CPLFree( pabyWKB );
    return poGeometry;
}

/************************************************************************/
/*                           GeometryToHex()                            */
/************************************************************************/
char *OGRPGLayer::GeometryToHex( OGRGeometry * poGeometry, int nSRSId )
{
    GByte       *pabyWKB;
    char        *pszTextBuf;
    char        *pszTextBufCurrent;
    char        *pszHex;

    int nWkbSize = poGeometry->WkbSize();
    pabyWKB = (GByte *) CPLMalloc(nWkbSize);

    if( poGeometry->exportToWkb( wkbNDR, pabyWKB ) != OGRERR_NONE )
    {
        CPLFree( pabyWKB );
        return CPLStrdup("");
    }

    /* When converting to hex, each byte takes 2 hex characters.  In addition
       we add in 8 characters to represent the SRID integer in hex, and
       one for a null terminator */

    int pszSize = nWkbSize*2 + 8 + 1;
    pszTextBuf = (char *) CPLMalloc(pszSize);
    pszTextBufCurrent = pszTextBuf;

    /* Convert the 1st byte, which is the endianess flag, to hex. */
    pszHex = CPLBinaryToHex( 1, pabyWKB );
    sprintf(pszTextBufCurrent, pszHex );
    CPLFree ( pszHex );
    pszTextBufCurrent += 2;

    /* Next, get the geom type which is bytes 2 through 5 */
    GUInt32 geomType;
    memcpy( &geomType, pabyWKB+1, 4 );

    /* Now add the SRID flag if an SRID is provided */
    if (nSRSId != -1)
    {
        /* Change the flag to wkbNDR (little) endianess */
        GUInt32 nGSrsFlag = CPL_LSBWORD32( WKBSRIDFLAG );
        /* Apply the flag */
        geomType = geomType | nGSrsFlag;
    }

    /* Now write the geom type which is 4 bytes */
    pszHex = CPLBinaryToHex( 4, (GByte*) &geomType );
    sprintf(pszTextBufCurrent, pszHex );
    CPLFree ( pszHex );
    pszTextBufCurrent += 8;

    /* Now include SRID if provided */
    if (nSRSId != -1)
    {
        /* Force the srsid to wkbNDR (little) endianess */
        GUInt32 nGSRSId = CPL_LSBWORD32( nSRSId );
        pszHex = CPLBinaryToHex( sizeof(nGSRSId),(GByte*) &nGSRSId );
        sprintf(pszTextBufCurrent, pszHex );
        CPLFree ( pszHex );
        pszTextBufCurrent += 8;
    }

    /* Copy the rest of the data over - subtract
       5 since we already copied 5 bytes above */
    pszHex = CPLBinaryToHex( nWkbSize - 5, pabyWKB + 5 );
    sprintf(pszTextBufCurrent, pszHex );
    CPLFree ( pszHex );

    CPLFree( pabyWKB );

    return pszTextBuf;
}


/************************************************************************/
/*                          BYTEAToGeometry()                           */
/************************************************************************/

OGRGeometry *OGRPGLayer::BYTEAToGeometry( const char *pszBytea )

{
    GByte       *pabyWKB;
    int iSrc=0, iDst=0;
    OGRGeometry *poGeometry;

    if( pszBytea == NULL )
        return NULL;

    pabyWKB = (GByte *) CPLMalloc(strlen(pszBytea));
    while( pszBytea[iSrc] != '\0' )
    {
        if( pszBytea[iSrc] == '\\' )
        {
            if( pszBytea[iSrc+1] >= '0' && pszBytea[iSrc+1] <= '9' )
            {
                pabyWKB[iDst++] =
                    (pszBytea[iSrc+1] - 48) * 64
                    + (pszBytea[iSrc+2] - 48) * 8
                    + (pszBytea[iSrc+3] - 48) * 1;
                iSrc += 4;
            }
            else
            {
                pabyWKB[iDst++] = pszBytea[iSrc+1];
                iSrc += 2;
            }
        }
        else
        {
            pabyWKB[iDst++] = pszBytea[iSrc++];
        }
    }

    poGeometry = NULL;
    OGRGeometryFactory::createFromWkb( pabyWKB, NULL, &poGeometry, iDst );

    CPLFree( pabyWKB );
    return poGeometry;
}

/************************************************************************/
/*                          GeometryToBYTEA()                           */
/************************************************************************/

char *OGRPGLayer::GeometryToBYTEA( OGRGeometry * poGeometry )

{
    int         nWkbSize = poGeometry->WkbSize();
    GByte       *pabyWKB;
    char        *pszTextBuf, *pszRetBuf;

    pabyWKB = (GByte *) CPLMalloc(nWkbSize);
    if( poGeometry->exportToWkb( wkbNDR, pabyWKB ) != OGRERR_NONE )
        return CPLStrdup("");

    pszTextBuf = (char *) CPLMalloc(nWkbSize*5+1);

    int  iSrc, iDst=0;

    for( iSrc = 0; iSrc < nWkbSize; iSrc++ )
    {
        if( pabyWKB[iSrc] < 40 || pabyWKB[iSrc] > 126
            || pabyWKB[iSrc] == '\\' )
        {
            sprintf( pszTextBuf+iDst, "\\\\%03o", pabyWKB[iSrc] );
            iDst += 5;
        }
        else
            pszTextBuf[iDst++] = pabyWKB[iSrc];
    }
    pszTextBuf[iDst] = '\0';

    pszRetBuf = CPLStrdup( pszTextBuf );
    CPLFree( pszTextBuf );

    return pszRetBuf;
}

/************************************************************************/
/*                          OIDToGeometry()                             */
/************************************************************************/

OGRGeometry *OGRPGLayer::OIDToGeometry( Oid oid )

{
    PGconn      *hPGConn = poDS->GetPGConn();
    GByte       *pabyWKB;
    int         fd, nBytes;
    OGRGeometry *poGeometry;

#define MAX_WKB 500000

    if( oid == 0 )
        return NULL;

    fd = lo_open( hPGConn, oid, INV_READ );
    if( fd < 0 )
        return NULL;

    pabyWKB = (GByte *) CPLMalloc(MAX_WKB);
    nBytes = lo_read( hPGConn, fd, (char *) pabyWKB, MAX_WKB );
    lo_close( hPGConn, fd );

    poGeometry = NULL;
    OGRGeometryFactory::createFromWkb( pabyWKB, NULL, &poGeometry, nBytes );

    CPLFree( pabyWKB );

    return poGeometry;
}

/************************************************************************/
/*                           GeometryToOID()                            */
/************************************************************************/

Oid OGRPGLayer::GeometryToOID( OGRGeometry * poGeometry )

{
    PGconn      *hPGConn = poDS->GetPGConn();
    int         nWkbSize = poGeometry->WkbSize();
    GByte       *pabyWKB;
    Oid         oid;
    int         fd, nBytesWritten;

    pabyWKB = (GByte *) CPLMalloc(nWkbSize);
    if( poGeometry->exportToWkb( wkbNDR, pabyWKB ) != OGRERR_NONE )
        return 0;

    oid = lo_creat( hPGConn, INV_READ|INV_WRITE );

    fd = lo_open( hPGConn, oid, INV_WRITE );
    nBytesWritten = lo_write( hPGConn, fd, (char *) pabyWKB, nWkbSize );
    lo_close( hPGConn, fd );

    if( nBytesWritten != nWkbSize )
    {
        CPLDebug( "OGR_PG",
                  "Only wrote %d bytes of %d intended for (fd=%d,oid=%d).\n",
                  nBytesWritten, nWkbSize, fd, oid );
    }

    CPLFree( pabyWKB );

    return oid;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRPGLayer::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,OLCRandomRead) )
        return FALSE;

    else if( EQUAL(pszCap,OLCFastFeatureCount) )
        return m_poFilterGeom == NULL || bHasPostGISGeometry;

    else if( EQUAL(pszCap,OLCFastSpatialFilter) )
        return TRUE;

    else if( EQUAL(pszCap,OLCTransactions) )
        return TRUE;

	else if( EQUAL(pszCap,OLCFastGetExtent) )
		return bHasPostGISGeometry;

    else
        return FALSE;
}

/************************************************************************/
/*                          StartTransaction()                          */
/************************************************************************/

OGRErr OGRPGLayer::StartTransaction()

{
    return poDS->SoftStartTransaction();
}

/************************************************************************/
/*                         CommitTransaction()                          */
/************************************************************************/

OGRErr OGRPGLayer::CommitTransaction()

{
    return poDS->SoftCommit();
}

/************************************************************************/
/*                        RollbackTransaction()                         */
/************************************************************************/

OGRErr OGRPGLayer::RollbackTransaction()

{
    return poDS->SoftRollback();
}

/************************************************************************/
/*                           GetSpatialRef()                            */
/************************************************************************/

OGRSpatialReference *OGRPGLayer::GetSpatialRef()

{
    if( poSRS == NULL && nSRSId > -1 )
    {
        poSRS = poDS->FetchSRS( nSRSId );
        if( poSRS != NULL )
            poSRS->Reference();
        else
            nSRSId = -1;
    }

    return poSRS;
}

/************************************************************************/
/*                            GetFIDColumn()                            */
/************************************************************************/

const char *OGRPGLayer::GetFIDColumn() 

{
    if( pszFIDColumn != NULL )
        return pszFIDColumn;
    else
        return "";
}

/************************************************************************/
/*                         GetGeometryColumn()                          */
/************************************************************************/

const char *OGRPGLayer::GetGeometryColumn() 

{
    if( pszGeomColumn != NULL )
        return pszGeomColumn;
    else
        return "";
}
