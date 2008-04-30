/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements OGRPGResultLayer class, access the resultset from
 *           a particular select query done via ExecuteSQL().
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2002, Frank Warmerdam
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

#include "cpl_conv.h"
#include "ogr_pg.h"
#include "ogrpgutility.h"

CPL_CVSID("$Id$");


/************************************************************************/
/*                          OGRPGResultLayer()                          */
/************************************************************************/

OGRPGResultLayer::OGRPGResultLayer( OGRPGDataSource *poDSIn, 
                                    const char * pszRawQueryIn,
                                    PGresult *hInitialResultIn )
{
    poDS = poDSIn;

    iNextShapeId = 0;

    pszRawStatement = CPLStrdup(pszRawQueryIn);

    /* Eventually we may need to make these a unique name */
    pszCursorName = "OGRPGResultLayerReader";

    osWHERE = "";

    BuildFullQueryStatement();

    poFeatureDefn = ReadResultDefinition(hInitialResultIn);

    /* We have to get the SRID of the geometry column, so to be able */
    /* to do spatial filtering */
    if (bHasPostGISGeometry)
    {
        CPLString osGetSRID;
        osGetSRID += "SELECT getsrid(\"";
        osGetSRID += pszGeomColumn;
        osGetSRID += "\") FROM (";
        osGetSRID += pszRawStatement;
        osGetSRID += ") AS ogrpggetsrid LIMIT 1";

        CPLDebug("PG", "PQexec(%s)\n", osGetSRID.c_str());

        PGresult* hSRSIdResult = PQexec(poDS->GetPGConn(), osGetSRID );

        if( hSRSIdResult && PQresultStatus(hSRSIdResult) == PGRES_TUPLES_OK)
        {
            if ( PQntuples(hSRSIdResult) > 0 )
                nSRSId = atoi(PQgetvalue(hSRSIdResult, 0, 0));
        }
        else
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                        "%s", PQerrorMessage(poDS->GetPGConn()) );
        }

        OGRPGClearResult(hSRSIdResult);
    }

    /* Now set the cursor that will fetch the first rows */
    /* This is usefull when used in situations like */
    /* ds->ReleaseResultSet(ds->ExecuteSQL("SELECT AddGeometryColumn(....)")) */
    /* when people don't actually try to get elements */
    SetInitialQueryCursor();
}

/************************************************************************/
/*                          ~OGRPGResultLayer()                          */
/************************************************************************/

OGRPGResultLayer::~OGRPGResultLayer()

{
    CPLFree( pszRawStatement );
}

/************************************************************************/
/*                        ReadResultDefinition()                        */
/*                                                                      */
/*      Build a schema from the current resultset.                      */
/************************************************************************/

OGRFeatureDefn *OGRPGResultLayer::ReadResultDefinition(PGresult *hInitialResultIn)

{
    PGresult            *hResult = hInitialResultIn;

/* -------------------------------------------------------------------- */
/*      Parse the returned table information.                           */
/* -------------------------------------------------------------------- */
    OGRFeatureDefn *poDefn = new OGRFeatureDefn( "sql_statement" );
    int            iRawField;

    poDefn->Reference();

    for( iRawField = 0; iRawField < PQnfields(hResult); iRawField++ )
    {
        OGRFieldDefn    oField( PQfname(hResult,iRawField), OFTString);
        Oid             nTypeOID;

        nTypeOID = PQftype(hResult,iRawField);
        
        if( EQUAL(oField.GetNameRef(),"ogc_fid") )
        {
            bHasFid = TRUE;
            pszFIDColumn = CPLStrdup(oField.GetNameRef());
            continue;
        }
        else if( nTypeOID == poDS->GetGeometryOID()  ||
                 EQUAL(oField.GetNameRef(),"asEWKT") ||
                 EQUAL(oField.GetNameRef(),"asText") )
        {
            bHasPostGISGeometry = TRUE;
            pszGeomColumn = CPLStrdup(oField.GetNameRef());
            continue;
        }
        else if( EQUAL(oField.GetNameRef(),"WKB_GEOMETRY") )
        {
            bHasWkb = TRUE;
            if( nTypeOID == OIDOID )
                bWkbAsOid = TRUE;
            continue;
        }

        if( nTypeOID == BYTEAOID )
        {
            oField.SetType( OFTBinary );
        }
        else if( nTypeOID == CHAROID ||
                 nTypeOID == TEXTOID ||
                 nTypeOID == BPCHAROID ||
                 nTypeOID == VARCHAROID )
        {
            oField.SetType( OFTString );
            oField.SetWidth( PQfsize(hResult, iRawField) );
        }
        else if( nTypeOID == BOOLOID )
        {
            oField.SetType( OFTInteger );
            oField.SetWidth( 1 );
        }
        else if (nTypeOID == INT2OID )
        {
            oField.SetType( OFTInteger );
            oField.SetWidth( 5 );
        }
        else if (nTypeOID == INT4OID )
        {
            oField.SetType( OFTInteger );
        }
        else if ( nTypeOID == INT8OID )
        {
            /* FIXME: OFTInteger can not handle 64bit integers */
            oField.SetType( OFTInteger );
        }
        else if( nTypeOID == FLOAT4OID ||
                 nTypeOID == FLOAT8OID ||
                 nTypeOID == NUMERICOID )
        {
            oField.SetType( OFTReal );
        }
        else if ( nTypeOID == INT4ARRAYOID )
        {
            oField.SetType ( OFTIntegerList );
        }
        else if ( nTypeOID == FLOAT4ARRAYOID ||
                  nTypeOID == FLOAT8ARRAYOID )
        {
            oField.SetType ( OFTRealList );
        }
        else if ( nTypeOID == TEXTARRAYOID ||
                  nTypeOID == BPCHARARRAYOID ||
                  nTypeOID == VARCHARARRAYOID )
        {
            oField.SetType ( OFTStringList );
        }
        else if ( nTypeOID == DATEOID )
        {
            oField.SetType( OFTDate );
        }
        else if ( nTypeOID == TIMEOID )
        {
            oField.SetType( OFTTime );
        }
        else if ( nTypeOID == TIMESTAMPOID ||
                  nTypeOID == TIMESTAMPTZOID )
        {
            /* We can't deserialize properly timestamp with time zone */
            /* with binary cursors */
            if (nTypeOID == TIMESTAMPTZOID)
                bCanUseBinaryCursor = FALSE;

            oField.SetType( OFTDateTime );
        }
        else /* unknown type */
        {
            CPLDebug("PG", "Unhandled OID (%d) for column %d. Defaulting to String.", nTypeOID, iRawField);
            oField.SetType( OFTString );
        }
        
        poDefn->AddFieldDefn( &oField );
    }

    return poDefn;
}

/************************************************************************/
/*                      BuildFullQueryStatement()                       */
/************************************************************************/

void OGRPGResultLayer::BuildFullQueryStatement()

{
    if( pszQueryStatement != NULL )
    {
        CPLFree( pszQueryStatement );
        pszQueryStatement = NULL;
    }

    pszQueryStatement = (char*) CPLMalloc(strlen(pszRawStatement) + strlen(osWHERE) + 40);

    if (strlen(osWHERE) == 0)
        strcpy(pszQueryStatement, pszRawStatement);
    else
        sprintf(pszQueryStatement, "SELECT * FROM (%s) AS ogrpgsubquery %s",
                pszRawStatement, osWHERE.c_str());
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRPGResultLayer::ResetReading()

{
    OGRPGLayer::ResetReading();
}

/************************************************************************/
/*                          GetFeatureCount()                           */
/************************************************************************/

int OGRPGResultLayer::GetFeatureCount( int bForce )

{
    if( TestCapability(OLCFastFeatureCount) == FALSE )
        return OGRPGLayer::GetFeatureCount( bForce );

    PGconn              *hPGConn = poDS->GetPGConn();
    PGresult            *hResult = NULL;
    CPLString           osCommand;
    int                 nCount = 0;

    osCommand.Printf(
        "SELECT count(*) FROM (%s) AS ogrpgcount",
        pszQueryStatement );

    CPLDebug( "PG", "PQexec(%s)\n",
              osCommand.c_str() );

    hResult = PQexec(hPGConn, osCommand);
    if( hResult != NULL && PQresultStatus(hResult) == PGRES_TUPLES_OK )
        nCount = atoi(PQgetvalue(hResult,0,0));
    else
        CPLDebug( "PG", "%s; failed.", osCommand.c_str() );
    OGRPGClearResult( hResult );

    return nCount;
}


/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRPGResultLayer::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,OLCFastFeatureCount) )
        return (m_poFilterGeom == NULL || 
                (bHasPostGISGeometry && nSRSId != -2)) && m_poAttrQuery == NULL;

    else if( EQUAL(pszCap,OLCFastSpatialFilter) || EQUAL(pszCap,OLCFastGetExtent) )
        return (bHasPostGISGeometry && nSRSId != -2) && m_poAttrQuery == NULL;

    else if( EQUAL(pszCap,OLCStringsAsUTF8) )
        return TRUE;

    else
        return FALSE;
}


/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRPGResultLayer::GetNextFeature()

{

    for( ; TRUE; )
    {
        OGRFeature      *poFeature;

        poFeature = GetNextRawFeature();
        if( poFeature == NULL )
            return NULL;

        if( (m_poFilterGeom == NULL
            || (bHasPostGISGeometry && nSRSId != -2)
            || FilterGeometry( poFeature->GetGeometryRef() ) )
            && (m_poAttrQuery == NULL
                || m_poAttrQuery->Evaluate( poFeature )) )
            return poFeature;

        delete poFeature;
    }
}

/************************************************************************/
/*                          SetSpatialFilter()                          */
/************************************************************************/

void OGRPGResultLayer::SetSpatialFilter( OGRGeometry * poGeomIn )

{
    if( InstallFilter( poGeomIn ) )
    {
        if (bHasPostGISGeometry && nSRSId != -2)
        {
            if( m_poFilterGeom != NULL)
            {
                OGREnvelope  sEnvelope;

                m_poFilterGeom->getEnvelope( &sEnvelope );
                osWHERE.Printf("WHERE %s && SetSRID('BOX3D(%.12f %.12f, %.12f %.12f)'::box3d,%d) ",
                            pszGeomColumn,
                            sEnvelope.MinX, sEnvelope.MinY,
                            sEnvelope.MaxX, sEnvelope.MaxY,
                            nSRSId );
            }
            else
            {
                osWHERE = "";
            }

            BuildFullQueryStatement();
        }

        ResetReading();
    }

}

/************************************************************************/
/*                             GetExtent()                              */
/*                                                                      */
/*      For PostGIS use internal Extend(geometry) function              */
/*      in other cases we use standard OGRLayer::GetExtent()            */ /************************************************************************/

OGRErr OGRPGResultLayer::GetExtent( OGREnvelope *psExtent, int bForce )
{
    CPLString   osCommand;

    if ( TestCapability(OLCFastGetExtent) )
    {
        /* Do not take the spatial filter into account */
        osCommand.Printf( "SELECT Extent(\"%s\") FROM (%s) AS ogrpgextent", 
                         pszGeomColumn, pszRawStatement );
    }

    return RunGetExtentRequest(psExtent, bForce, osCommand);
}
