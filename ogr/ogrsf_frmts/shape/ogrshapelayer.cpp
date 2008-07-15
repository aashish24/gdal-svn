/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements OGRShapeLayer class.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 1999,  Les Technologies SoftMap Inc.
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

#include "ogrshape.h"
#include "cpl_conv.h"
#include "cpl_string.h"

#if defined(_WIN32_WCE)
#  include <wce_errno.h>
#endif

CPL_CVSID("$Id$");

/************************************************************************/
/*                           OGRShapeLayer()                            */
/************************************************************************/

OGRShapeLayer::OGRShapeLayer( const char * pszName,
                              SHPHandle hSHPIn, DBFHandle hDBFIn, 
                              OGRSpatialReference *poSRSIn, int bUpdate,
                              OGRwkbGeometryType eReqType )

{
    poSRS = poSRSIn;

    pszFullName = CPLStrdup(pszName);
    
    hSHP = hSHPIn;
    hDBF = hDBFIn;
    bUpdateAccess = bUpdate;

    iNextShapeId = 0;
    panMatchingFIDs = NULL;

    bCheckedForQIX = FALSE;
    fpQIX = NULL;

    bHeaderDirty = FALSE;

    if( hSHP != NULL )
        nTotalShapeCount = hSHP->nRecords;
    else 
        nTotalShapeCount = hDBF->nRecords;
    
    poFeatureDefn = SHPReadOGRFeatureDefn( CPLGetBasename(pszName), 
                                           hSHP, hDBF );

    eRequestedGeomType = eReqType;
}

/************************************************************************/
/*                           ~OGRShapeLayer()                           */
/************************************************************************/

OGRShapeLayer::~OGRShapeLayer()

{
    if( m_nFeaturesRead > 0 && poFeatureDefn != NULL )
    {
        CPLDebug( "Shape", "%d features read on layer '%s'.",
                  (int) m_nFeaturesRead, 
                  poFeatureDefn->GetName() );
    }

    CPLFree( panMatchingFIDs );
    panMatchingFIDs = NULL;

    CPLFree( pszFullName );

    if( poFeatureDefn != NULL )
        poFeatureDefn->Release();

    if( poSRS != NULL )
        poSRS->Release();

    if( hDBF != NULL )
        DBFClose( hDBF );

    if( hSHP != NULL )
        SHPClose( hSHP );

    if( fpQIX != NULL )
        VSIFClose( fpQIX );
}

/************************************************************************/
/*                            CheckForQIX()                             */
/************************************************************************/

int OGRShapeLayer::CheckForQIX()

{
    const char *pszQIXFilename;

    if( bCheckedForQIX )
        return fpQIX != NULL;

    pszQIXFilename = CPLResetExtension( pszFullName, "qix" );

    fpQIX = VSIFOpen( pszQIXFilename, "rb" );

    bCheckedForQIX = TRUE;

    return fpQIX != NULL;
}

/************************************************************************/
/*                            ScanIndices()                             */
/*                                                                      */
/*      Utilize optional spatial and attribute indices if they are      */
/*      available.                                                      */
/************************************************************************/

int OGRShapeLayer::ScanIndices()

{
    iMatchingFID = 0;

/* -------------------------------------------------------------------- */
/*      Utilize attribute index if appropriate.                         */
/* -------------------------------------------------------------------- */
    if( m_poAttrQuery != NULL )
    {
        CPLAssert( panMatchingFIDs == NULL );
        panMatchingFIDs = m_poAttrQuery->EvaluateAgainstIndices( this,
                                                                 NULL );
    }

/* -------------------------------------------------------------------- */
/*      Check for spatial index if we have a spatial query.             */
/* -------------------------------------------------------------------- */
    if( m_poFilterGeom != NULL && !bCheckedForQIX )
        CheckForQIX();

/* -------------------------------------------------------------------- */
/*      Utilize spatial index if appropriate.                           */
/* -------------------------------------------------------------------- */
    if( m_poFilterGeom && fpQIX )
    {
        int nSpatialFIDCount, *panSpatialFIDs;
        double adfBoundsMin[4], adfBoundsMax[4];
        OGREnvelope oEnvelope;

        m_poFilterGeom->getEnvelope( &oEnvelope );

        adfBoundsMin[0] = oEnvelope.MinX;
        adfBoundsMin[1] = oEnvelope.MinY;
        adfBoundsMin[2] = 0.0;
        adfBoundsMin[3] = 0.0;
        adfBoundsMax[0] = oEnvelope.MaxX;
        adfBoundsMax[1] = oEnvelope.MaxY;
        adfBoundsMax[2] = 0.0;
        adfBoundsMax[3] = 0.0;

        panSpatialFIDs = SHPSearchDiskTree( fpQIX, 
                                            adfBoundsMin, adfBoundsMax, 
                                            &nSpatialFIDCount );
        CPLDebug( "SHAPE", "Used spatial index, got %d matches.", 
                  nSpatialFIDCount );

        // Use resulting list as matching FID list (but reallocate and
        // terminate with OGRNullFID).

        if( panMatchingFIDs == NULL )
        {
            int i;

            panMatchingFIDs = (long *) 
                CPLMalloc(sizeof(long) * (nSpatialFIDCount+1) );
            for( i = 0; i < nSpatialFIDCount; i++ )
                panMatchingFIDs[i] = (long) panSpatialFIDs[i];
            panMatchingFIDs[nSpatialFIDCount] = OGRNullFID;
        }

        // Cull attribute index matches based on those in the spatial index
        // result set.  We assume that the attribute results are in sorted
        // order.
        else
        {
            int iRead, iWrite=0, iSpatial=0;

            for( iRead = 0; panMatchingFIDs[iRead] != OGRNullFID; iRead++ )
            {
                while( iSpatial < nSpatialFIDCount
                       && panSpatialFIDs[iSpatial] < panMatchingFIDs[iRead] )
                    iSpatial++;

                if( iSpatial == nSpatialFIDCount )
                    continue;

                if( panSpatialFIDs[iSpatial] == panMatchingFIDs[iRead] )
                    panMatchingFIDs[iWrite++] = panMatchingFIDs[iRead];
            }
            panMatchingFIDs[iWrite] = OGRNullFID;
        }

        if ( panSpatialFIDs )
            free( panSpatialFIDs );
    }

    return TRUE;
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRShapeLayer::ResetReading()

{
/* -------------------------------------------------------------------- */
/*      Clear previous index search result, if any.                     */
/* -------------------------------------------------------------------- */
    CPLFree( panMatchingFIDs );
    panMatchingFIDs = NULL;
    iMatchingFID = 0;

    iNextShapeId = 0;

    if( bHeaderDirty && bUpdateAccess )
        SyncToDisk();
}

/************************************************************************/
/*                           SetNextByIndex()                           */
/*                                                                      */
/*      If we already have an FID list, we can easily resposition       */
/*      ourselves in it.                                                */
/************************************************************************/

OGRErr OGRShapeLayer::SetNextByIndex( long nIndex )

{
    // Eventually we should try to use panMatchingFIDs list 
    // if available and appropriate. 
    if( m_poFilterGeom != NULL || m_poAttrQuery != NULL )
        return OGRLayer::SetNextByIndex( nIndex );

    iNextShapeId = nIndex;

    return OGRERR_NONE;
}

/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRShapeLayer::GetNextFeature()

{
    OGRFeature  *poFeature = NULL;

/* -------------------------------------------------------------------- */
/*      Collect a matching list if we have attribute or spatial         */
/*      indices.  Only do this on the first request for a given pass    */
/*      of course.                                                      */
/* -------------------------------------------------------------------- */
    if( (m_poAttrQuery != NULL || m_poFilterGeom != NULL)
        && iNextShapeId == 0 && panMatchingFIDs == NULL )
    {
        ScanIndices();
    }
    
/* -------------------------------------------------------------------- */
/*      Loop till we find a feature matching our criteria.              */
/* -------------------------------------------------------------------- */
    while( TRUE )
    {
        if( panMatchingFIDs != NULL )
        {
            if( panMatchingFIDs[iMatchingFID] == OGRNullFID )
            {
                return NULL;
            }

            poFeature = SHPReadOGRFeature( hSHP, hDBF, poFeatureDefn, 
                                           panMatchingFIDs[iMatchingFID++] );
        }
        else
        {
            if( iNextShapeId >= nTotalShapeCount )
            {
                return NULL;
            }
    
            if( hDBF && DBFIsRecordDeleted( hDBF, iNextShapeId ) )
                poFeature = NULL;
            else
                poFeature = SHPReadOGRFeature( hSHP, hDBF, poFeatureDefn,
                                               iNextShapeId );

            iNextShapeId++;
        }

        if( poFeature != NULL )
        {
            if( poFeature->GetGeometryRef() != NULL )
            {
                poFeature->GetGeometryRef()->assignSpatialReference( poSRS );
            }

            m_nFeaturesRead++;

            if( (m_poFilterGeom == NULL || FilterGeometry( poFeature->GetGeometryRef() ) )
                && (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate( poFeature )) )
            {
                return poFeature;
            }

            delete poFeature;
        }
    }        

    /*
     * NEVER SHOULD GET HERE
     */
    CPLAssert(!"OGRShapeLayer::GetNextFeature(): Execution never should get here!");
}

/************************************************************************/
/*                             GetFeature()                             */
/************************************************************************/

OGRFeature *OGRShapeLayer::GetFeature( long nFeatureId )

{
    OGRFeature *poFeature = NULL;
    poFeature = SHPReadOGRFeature( hSHP, hDBF, poFeatureDefn, nFeatureId );

    if( poFeature != NULL )
    {
        if( poFeature->GetGeometryRef() != NULL )
            poFeature->GetGeometryRef()->assignSpatialReference( poSRS );

        m_nFeaturesRead++;
    
        return poFeature;
    }

    /*
     * Reading shape feature failed.
     */
    return NULL;
}

/************************************************************************/
/*                             SetFeature()                             */
/************************************************************************/

OGRErr OGRShapeLayer::SetFeature( OGRFeature *poFeature )

{
    bHeaderDirty = TRUE;

    return SHPWriteOGRFeature( hSHP, hDBF, poFeatureDefn, poFeature );
}

/************************************************************************/
/*                           DeleteFeature()                            */
/************************************************************************/

OGRErr OGRShapeLayer::DeleteFeature( long nFID )

{
    if( nFID < 0 
        || (hSHP != NULL && nFID >= hSHP->nRecords)
        || (hDBF != NULL && nFID >= hDBF->nRecords) )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Attempt to delete shape with feature id (%ld) which does "
                  "not exist.", nFID );
        return OGRERR_FAILURE;
    }

    if( !hDBF )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Attempt to delete shape in shapefile with no .dbf file.\n"
                  "Deletion is done by marking record deleted in dbf\n"
                  "and is not supported without a .dbf file." );
        return OGRERR_FAILURE;
    }

    if( DBFIsRecordDeleted( hDBF, nFID ) )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Attempt to delete shape with feature id (%ld), but it is marked deleted already.",
                  nFID );
        return OGRERR_FAILURE;
    }

    if( !DBFMarkRecordDeleted( hDBF, nFID, TRUE ) )
        return OGRERR_FAILURE;

    bHeaderDirty = TRUE;

    return OGRERR_NONE;
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/

OGRErr OGRShapeLayer::CreateFeature( OGRFeature *poFeature )

{
    OGRErr eErr;

    bHeaderDirty = TRUE;

    poFeature->SetFID( OGRNullFID );

    if( nTotalShapeCount == 0 
        && eRequestedGeomType == wkbUnknown 
        && poFeature->GetGeometryRef() != NULL )
    {
        OGRGeometry     *poGeom = poFeature->GetGeometryRef();
        int             nShapeType;
        
        switch( poGeom->getGeometryType() )
        {
          case wkbPoint:
            nShapeType = SHPT_POINT;
            eRequestedGeomType = wkbPoint;
            break;

          case wkbPoint25D:
            nShapeType = SHPT_POINTZ;
            eRequestedGeomType = wkbPoint25D;
            break;

          case wkbMultiPoint:
            nShapeType = SHPT_MULTIPOINT;
            eRequestedGeomType = wkbMultiPoint;
            break;

          case wkbMultiPoint25D:
            nShapeType = SHPT_MULTIPOINTZ;
            eRequestedGeomType = wkbMultiPoint25D;
            break;

          case wkbLineString:
          case wkbMultiLineString:
            nShapeType = SHPT_ARC;
            eRequestedGeomType = wkbLineString;
            break;

          case wkbLineString25D:
          case wkbMultiLineString25D:
            nShapeType = SHPT_ARCZ;
            eRequestedGeomType = wkbLineString25D;
            break;

          case wkbPolygon:
          case wkbMultiPolygon:
            nShapeType = SHPT_POLYGON;
            eRequestedGeomType = wkbPolygon;
            break;

          case wkbPolygon25D:
          case wkbMultiPolygon25D:
            nShapeType = SHPT_POLYGONZ;
            eRequestedGeomType = wkbPolygon25D;
            break;

          default:
            nShapeType = -1;
            break;
        }

        if( nShapeType != -1 )
        {
            ResetGeomType( nShapeType );
        }
    }
    
    eErr = SHPWriteOGRFeature( hSHP, hDBF, poFeatureDefn, poFeature );

    if( hSHP != NULL )
        nTotalShapeCount = hSHP->nRecords;
    else 
        nTotalShapeCount = hDBF->nRecords;
    
    return eErr;
}

/************************************************************************/
/*                          GetFeatureCount()                           */
/*                                                                      */
/*      If a spatial filter is in effect, we turn control over to       */
/*      the generic counter.  Otherwise we return the total count.      */
/*      Eventually we should consider implementing a more efficient     */
/*      way of counting features matching a spatial query.              */
/************************************************************************/

int OGRShapeLayer::GetFeatureCount( int bForce )

{
    if( m_poFilterGeom != NULL || m_poAttrQuery != NULL )
        return OGRLayer::GetFeatureCount( bForce );
    else
        return nTotalShapeCount;
}

/************************************************************************/
/*                             GetExtent()                              */
/*                                                                      */
/*      Fetch extent of the data currently stored in the dataset.       */
/*      The bForce flag has no effect on SHO files since that value     */
/*      is always in the header.                                        */
/*                                                                      */
/*      Returns OGRERR_NONE/OGRRERR_FAILURE.                            */
/************************************************************************/

OGRErr OGRShapeLayer::GetExtent (OGREnvelope *psExtent, int bForce)

{
    double adMin[4], adMax[4];

    if( hSHP == NULL )
        return OGRERR_FAILURE;

    SHPGetInfo(hSHP, NULL, NULL, adMin, adMax);

    psExtent->MinX = adMin[0];
    psExtent->MinY = adMin[1];
    psExtent->MaxX = adMax[0];
    psExtent->MaxY = adMax[1];

    return OGRERR_NONE;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRShapeLayer::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,OLCRandomRead) )
        return TRUE;

    else if( EQUAL(pszCap,OLCSequentialWrite) 
             || EQUAL(pszCap,OLCRandomWrite) )
        return bUpdateAccess;

    else if( EQUAL(pszCap,OLCFastFeatureCount) )
        return m_poFilterGeom == NULL;

    else if( EQUAL(pszCap,OLCDeleteFeature) )
        return TRUE;

    else if( EQUAL(pszCap,OLCFastSpatialFilter) )
        return FALSE;

    else if( EQUAL(pszCap,OLCFastGetExtent) )
        return TRUE;

    else if( EQUAL(pszCap,OLCFastSetNextByIndex) )
        return m_poFilterGeom == NULL && m_poAttrQuery == NULL;

    else if( EQUAL(pszCap,OLCCreateField) )
        return TRUE;

    else 
        return FALSE;
}

/************************************************************************/
/*                            CreateField()                             */
/************************************************************************/

OGRErr OGRShapeLayer::CreateField( OGRFieldDefn *poField, int bApproxOK )

{
    CPLAssert( NULL != poField );

    int         iNewField;
    if( GetFeatureCount(TRUE) != 0 )
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "Can't create fields on a Shapefile layer with features.\n");
        return OGRERR_FAILURE;
    }

    if( !bUpdateAccess )
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "Can't create fields on a read-only shapefile layer.\n");
        return OGRERR_FAILURE;

    }

/* -------------------------------------------------------------------- */
/*      Normalize field name                                            */
/* -------------------------------------------------------------------- */
        
    char * pszNewFieldName = NULL;

    size_t nNameSize = strlen( poField->GetNameRef() );
    pszNewFieldName = CPLScanString( poField->GetNameRef(),
                                     nNameSize, TRUE, TRUE);

    if( !EQUAL(poField->GetNameRef(),pszNewFieldName) )
        CPLDebug( "Shape", "Normalized field name: '%s' to '%s'", 
                  poField->GetNameRef(),
                  pszNewFieldName );

    // Set field name with normalized value
    poField->SetName( pszNewFieldName );

    CPLFree( pszNewFieldName );

/* -------------------------------------------------------------------- */
/*      Add field to layer                                              */
/* -------------------------------------------------------------------- */

    if( poField->GetType() == OFTInteger )
    {
        if( poField->GetWidth() == 0 )
            iNewField =
                DBFAddField( hDBF, poField->GetNameRef(), FTInteger, 11, 0 );
        else
            iNewField = DBFAddField( hDBF, poField->GetNameRef(), FTInteger,
                                     poField->GetWidth(), 0 );

        if( iNewField != -1 )
            poFeatureDefn->AddFieldDefn( poField );
    }
    else if( poField->GetType() == OFTReal )
    {
        if( poField->GetWidth() == 0 )
            iNewField =
                DBFAddField( hDBF, poField->GetNameRef(), FTDouble, 24, 15 );
        else
            iNewField =
                DBFAddField( hDBF, poField->GetNameRef(), FTDouble,
                             poField->GetWidth(), poField->GetPrecision() );

        if( iNewField != -1 )
            poFeatureDefn->AddFieldDefn( poField );
    }
    else if( poField->GetType() == OFTString )
    {
        if( poField->GetWidth() < 1 )
            iNewField =
                DBFAddField( hDBF, poField->GetNameRef(), FTString, 80, 0 );
        else
            iNewField = DBFAddField( hDBF, poField->GetNameRef(), FTString, 
                                     poField->GetWidth(), 0 );

        if( iNewField != -1 )
            poFeatureDefn->AddFieldDefn( poField );
    }
    else if( poField->GetType() == OFTDate )
    {
        iNewField =
            DBFAddNativeFieldType( hDBF, poField->GetNameRef(), 'D', 8, 0 );

        if( iNewField != -1 )
            poFeatureDefn->AddFieldDefn( poField );
    }
    else if( poField->GetType() == OFTDateTime )
    {
        CPLError( CE_Warning, CPLE_NotSupported,
                  "Field %s create as date field, though DateTime requested.\n",
                  poField->GetNameRef() );

        iNewField =
            DBFAddNativeFieldType( hDBF, poField->GetNameRef(), 'D', 8, 0 );

        if( iNewField != -1 )
        {
            OGRFieldDefn oModDefn( poField );
            
            oModDefn.SetType( OFTDate );
            poFeatureDefn->AddFieldDefn( &oModDefn );
        }
    }
    else
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "Can't create fields of type %s on shapefile layers.\n",
                  OGRFieldDefn::GetFieldTypeName(poField->GetType()) );

        return OGRERR_FAILURE;
    }

    if( iNewField != -1 )
    {
        return OGRERR_NONE;
    }
    else        
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "Can't create field %s in Shape DBF file, reason unknown.\n",
                  poField->GetNameRef() );

        return OGRERR_FAILURE;
    }
}

/************************************************************************/
/*                           GetSpatialRef()                            */
/************************************************************************/

OGRSpatialReference *OGRShapeLayer::GetSpatialRef()

{
    return poSRS;
}

/************************************************************************/
/*                           ResetGeomType()                            */
/*                                                                      */
/*      Modify the geometry type for this file.  Used to convert to     */
/*      a different geometry type when a layer was created with a       */
/*      type of unknown, and we get to the first feature to             */
/*      establish the type.                                             */
/************************************************************************/

int OGRShapeLayer::ResetGeomType( int nNewGeomType )

{
    char        abyHeader[100];
    int         nStartPos;

    if( nTotalShapeCount > 0 )
        return FALSE;

    if( hSHP->fpSHX == NULL)
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  " OGRShapeLayer::ResetGeomType failed : SHX file is closed");
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Update .shp header.                                             */
/* -------------------------------------------------------------------- */
    nStartPos = hSHP->sHooks.FTell( hSHP->fpSHP );

    if( hSHP->sHooks.FSeek( hSHP->fpSHP, 0, SEEK_SET ) != 0
        || hSHP->sHooks.FRead( abyHeader, 100, 1, hSHP->fpSHP ) != 1 )
        return FALSE;

    *((GInt32 *) (abyHeader + 32)) = CPL_LSBWORD32( nNewGeomType );

    if( hSHP->sHooks.FSeek( hSHP->fpSHP, 0, SEEK_SET ) != 0
        || hSHP->sHooks.FWrite( abyHeader, 100, 1, hSHP->fpSHP ) != 1 )
        return FALSE;

    if( hSHP->sHooks.FSeek( hSHP->fpSHP, nStartPos, SEEK_SET ) != 0 )
        return FALSE;

/* -------------------------------------------------------------------- */
/*      Update .shx header.                                             */
/* -------------------------------------------------------------------- */
    nStartPos = hSHP->sHooks.FTell( hSHP->fpSHX );
    
    if( hSHP->sHooks.FSeek( hSHP->fpSHX, 0, SEEK_SET ) != 0
        || hSHP->sHooks.FRead( abyHeader, 100, 1, hSHP->fpSHX ) != 1 )
        return FALSE;

    *((GInt32 *) (abyHeader + 32)) = CPL_LSBWORD32( nNewGeomType );

    if( hSHP->sHooks.FSeek( hSHP->fpSHX, 0, SEEK_SET ) != 0
        || hSHP->sHooks.FWrite( abyHeader, 100, 1, hSHP->fpSHX ) != 1 )
        return FALSE;

    if( hSHP->sHooks.FSeek( hSHP->fpSHX, nStartPos, SEEK_SET ) != 0 )
        return FALSE;

/* -------------------------------------------------------------------- */
/*      Update other information.                                       */
/* -------------------------------------------------------------------- */
    hSHP->nShapeType = nNewGeomType;

    return TRUE;
}

/************************************************************************/
/*                             SyncToDisk()                             */
/************************************************************************/

OGRErr OGRShapeLayer::SyncToDisk()

{
    if( bHeaderDirty )
    {
        if( hSHP != NULL )
            SHPWriteHeader( hSHP );

        if( hDBF != NULL )
            DBFUpdateHeader( hDBF );
        
        bHeaderDirty = FALSE;
    }

    if( hSHP != NULL )
    {
        hSHP->sHooks.FFlush( hSHP->fpSHP );
        if( hSHP->fpSHX != NULL )
            hSHP->sHooks.FFlush( hSHP->fpSHX );
    }

    if( hDBF != NULL )
        hDBF->sHooks.FFlush( hDBF->fp );

    return OGRERR_NONE;
}

/************************************************************************/
/*                          DropSpatialIndex()                          */
/************************************************************************/

OGRErr OGRShapeLayer::DropSpatialIndex()

{
    if( !CheckForQIX() )
    {
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "Layer %s has no spatial index, DROP SPATIAL INDEX failed.",
                  poFeatureDefn->GetName() );
        return OGRERR_FAILURE;
    }

    VSIFClose( fpQIX );
    fpQIX = NULL;
    bCheckedForQIX = FALSE;
    
    const char *pszQIXFilename;

    pszQIXFilename = CPLResetExtension( pszFullName, "qix" );
    CPLDebug( "SHAPE", "Unlinking index file %s", pszQIXFilename );

    if( VSIUnlink( pszQIXFilename ) != 0 )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "Failed to delete file %s.\n%s", 
                  pszQIXFilename, VSIStrerror( errno ) );
        return OGRERR_FAILURE;
    }
    else
        return OGRERR_NONE;
}

/************************************************************************/
/*                         CreateSpatialIndex()                         */
/************************************************************************/

OGRErr OGRShapeLayer::CreateSpatialIndex( int nMaxDepth )

{
/* -------------------------------------------------------------------- */
/*      If we have an existing spatial index, blow it away first.       */
/* -------------------------------------------------------------------- */
    if( CheckForQIX() )
        DropSpatialIndex();

    bCheckedForQIX = FALSE;

/* -------------------------------------------------------------------- */
/*      Build a quadtree structure for this file.                       */
/* -------------------------------------------------------------------- */
    SHPTree	*psTree;

    SyncToDisk();
    psTree = SHPCreateTree( hSHP, 2, nMaxDepth, NULL, NULL );

    if( NULL == psTree )
    {
        // TODO - mloskot: Is it better to return OGRERR_NOT_ENOUGH_MEMORY?

        CPLDebug( "SHAPE",
                  "Index creation failure. Likely, memory allocation error." );

        return OGRERR_FAILURE;
    }

/* -------------------------------------------------------------------- */
/*      Trim unused nodes from the tree.                                */
/* -------------------------------------------------------------------- */
    SHPTreeTrimExtraNodes( psTree );

/* -------------------------------------------------------------------- */
/*      Dump tree to .qix file.                                         */
/* -------------------------------------------------------------------- */
    char *pszQIXFilename;

    pszQIXFilename = CPLStrdup(CPLResetExtension( pszFullName, "qix" ));

    CPLDebug( "SHAPE", "Creating index file %s", pszQIXFilename );

    SHPWriteTree( psTree, pszQIXFilename );
    CPLFree( pszQIXFilename );


/* -------------------------------------------------------------------- */
/*      cleanup                                                         */
/* -------------------------------------------------------------------- */
    SHPDestroyTree( psTree );

    CheckForQIX();

    return OGRERR_NONE;
}

/************************************************************************/
/*                               Repack()                               */
/*                                                                      */
/*      Repack the shape and dbf file, dropping deleted records.        */
/*      FIDs may change.                                                */
/************************************************************************/

OGRErr OGRShapeLayer::Repack()

{
    if( !bUpdateAccess )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
            "The REPACK operation not permitted on a read-only shapefile." );
        return OGRERR_FAILURE;
    }
    
/* -------------------------------------------------------------------- */
/*      Build a list of records to be dropped.                          */
/* -------------------------------------------------------------------- */
    int *panRecordsToDelete = (int *) 
        CPLMalloc(sizeof(int)*(nTotalShapeCount+1));
    int nDeleteCount = 0;
    int iShape = 0;
    OGRErr eErr = OGRERR_NONE;

    for( iShape = 0; iShape < nTotalShapeCount; iShape++ )
    {
        if( DBFIsRecordDeleted( hDBF, iShape ) )
            panRecordsToDelete[nDeleteCount++] = iShape;
    }
    panRecordsToDelete[nDeleteCount] = -1;

/* -------------------------------------------------------------------- */
/*      If there are no records marked for deletion, we take no         */
/*      action.                                                         */
/* -------------------------------------------------------------------- */
    if( nDeleteCount == 0 )
        return OGRERR_NONE;

/* -------------------------------------------------------------------- */
/*      Cleanup any existing spatial index.  It will become             */
/*      meaningless when the fids change.                               */
/* -------------------------------------------------------------------- */
    if( CheckForQIX() )
        DropSpatialIndex();

/* -------------------------------------------------------------------- */
/*      Create a new dbf file, matching the old.                        */
/* -------------------------------------------------------------------- */
    DBFHandle hNewDBF = NULL;
    
    CPLString oTempFile(CPLGetPath(pszFullName));
    oTempFile += '\\';
    oTempFile += CPLGetBasename(pszFullName);
    oTempFile += "_packed.dbf";

    hNewDBF = DBFCloneEmpty( hDBF, oTempFile );
    if( hNewDBF == NULL )
    {
        CPLFree( panRecordsToDelete );

        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "Failed to create temp file %s.", 
                  oTempFile.c_str() );
        return OGRERR_FAILURE;
    }

/* -------------------------------------------------------------------- */
/*      Copy over all records that are not deleted.                     */
/* -------------------------------------------------------------------- */
    int iDestShape = 0;
    int iNextDeletedShape = 0;

    for( iShape = 0; 
         iShape < nTotalShapeCount && eErr == OGRERR_NONE; 
         iShape++ )
    {
        if( panRecordsToDelete[iNextDeletedShape] == iShape )
            iNextDeletedShape++;
        else
        {
            void *pTuple = (void *) DBFReadTuple( hDBF, iShape );
            if( pTuple == NULL )
                eErr = OGRERR_FAILURE;
            else if( !DBFWriteTuple( hNewDBF, iDestShape++, pTuple ) )
                eErr = OGRERR_FAILURE;
        }                           
    }

    if( eErr != OGRERR_NONE )
    {
        CPLFree( panRecordsToDelete );
        VSIUnlink( oTempFile );
        return eErr;
    }

/* -------------------------------------------------------------------- */
/*      Cleanup the old .dbf and rename the new one.                    */
/* -------------------------------------------------------------------- */
    DBFClose( hDBF );
    DBFClose( hNewDBF );
    hDBF = hNewDBF = NULL;

    VSIUnlink( CPLResetExtension( pszFullName, "dbf" ) );
    if( VSIRename( oTempFile, CPLResetExtension( pszFullName, "dbf" ) ) != 0 )
    {
        CPLDebug( "Shape", "Can not rename DBF file: %s", VSIStrerror( errno ) );
        return OGRERR_FAILURE;
    }
    
/* -------------------------------------------------------------------- */
/*      Now create a shapefile matching the old one.                    */
/* -------------------------------------------------------------------- */
    if( hSHP != NULL )
    {
        SHPHandle hNewSHP = NULL;

        oTempFile = CPLGetPath(pszFullName);
        oTempFile += '\\';
        oTempFile += CPLGetBasename(pszFullName);
        oTempFile += "_packed.shp";

        hNewSHP = SHPCreate( oTempFile, hSHP->nShapeType );
        if( hNewSHP == NULL )
            return OGRERR_FAILURE;

/* -------------------------------------------------------------------- */
/*      Copy over all records that are not deleted.                     */
/* -------------------------------------------------------------------- */
        iNextDeletedShape = 0;

        for( iShape = 0; 
             iShape < nTotalShapeCount && eErr == OGRERR_NONE; 
             iShape++ )
        {
            if( panRecordsToDelete[iNextDeletedShape] == iShape )
                iNextDeletedShape++;
            else
            {
                SHPObject *hObject;

                hObject = SHPReadObject( hSHP, iShape );
                if( hObject == NULL )
                    eErr = OGRERR_FAILURE;
                else if( SHPWriteObject( hNewSHP, -1, hObject ) == -1 )
                    eErr = OGRERR_FAILURE;

                if( hObject )
                    SHPDestroyObject( hObject );
            }
        }

        if( eErr != OGRERR_NONE )
        {
            CPLFree( panRecordsToDelete );
            VSIUnlink( CPLResetExtension( oTempFile, "shp" ) );
            VSIUnlink( CPLResetExtension( oTempFile, "shx" ) );
            return eErr;
        }

/* -------------------------------------------------------------------- */
/*      Cleanup the old .shp/.shx and rename the new one.               */
/* -------------------------------------------------------------------- */
        SHPClose( hSHP );
        SHPClose( hNewSHP );
        hSHP = hNewSHP = NULL;

        VSIUnlink( CPLResetExtension( pszFullName, "shp" ) );
        VSIUnlink( CPLResetExtension( pszFullName, "shx" ) );

        if( VSIRename( oTempFile, CPLResetExtension( pszFullName, "shp" ) ) != 0 )
        {
            CPLDebug( "Shape", "Can not rename SHP file: %s", VSIStrerror( errno ) );
            return OGRERR_FAILURE;
        }
    
        oTempFile = CPLResetExtension( oTempFile, "shx" );
        if( VSIRename( oTempFile, CPLResetExtension( pszFullName, "shx" ) ) != 0 )
        {
            CPLDebug( "Shape", "Can not rename SHX file: %s", VSIStrerror( errno ) );
            return OGRERR_FAILURE;
        }
    }

/* -------------------------------------------------------------------- */
/*      Reopen the shapefile                                            */
/*                                                                      */
/* We do not need to reimplement OGRShapeDataSource::OpenFile() here    */  
/* with the fully featured error checking.                              */
/* If all operations above succeeded, then all necessery files are      */
/* in the right place and accessible.                                   */
/* -------------------------------------------------------------------- */
    CPLAssert( NULL == hSHP );
    CPLAssert( NULL == hDBF && NULL == hNewDBF );
    
    CPLPushErrorHandler( CPLQuietErrorHandler );
    
    const char* pszAccess = NULL;
    if( bUpdateAccess )
        pszAccess = "r+";
    else
        pszAccess = "r";
    
    hSHP = SHPOpen ( CPLResetExtension( pszFullName, "shp" ) , pszAccess );
    hDBF = DBFOpen ( CPLResetExtension( pszFullName, "dbf" ) , pszAccess );
    
    CPLPopErrorHandler();
    
    if( NULL == hSHP || NULL == hDBF )
    {
        CPLString osMsg(CPLGetLastErrorMsg());
        CPLError( CE_Failure, CPLE_OpenFailed, "%s", osMsg.c_str() );

        return OGRERR_FAILURE;
    }

/* -------------------------------------------------------------------- */
/*      Update total shape count.                                       */
/* -------------------------------------------------------------------- */
    nTotalShapeCount = hDBF->nRecords;

    CPLFree( panRecordsToDelete );

    return OGRERR_NONE;
}
