/******************************************************************************
 * $Id$
 *
 * Project:  OGR
 * Purpose:  Implements OGRGMLLayer class.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2002, Frank Warmerdam <warmerdam@pobox.com>
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

#include "ogr_gml.h"
#include "cpl_conv.h"
#include "cpl_port.h"
#include "cpl_string.h"
#include "ogr_p.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*                           OGRGMLLayer()                              */
/************************************************************************/

OGRGMLLayer::OGRGMLLayer( const char * pszName,
                          OGRSpatialReference *poSRSIn, int bWriterIn,
                          OGRwkbGeometryType eReqType,
                          OGRGMLDataSource *poDSIn )

{
    if( poSRSIn == NULL )
        poSRS = NULL;
    else
        poSRS = poSRSIn->Clone();
    
    iNextGMLId = 0;
    nTotalGMLCount = -1;
    bInvalidFIDFound = FALSE;
    pszFIDPrefix = NULL;
        
    poDS = poDSIn;

    if ( EQUALN(pszName, "ogr:", 4) )
      poFeatureDefn = new OGRFeatureDefn( pszName+4 );
    else
      poFeatureDefn = new OGRFeatureDefn( pszName );
    poFeatureDefn->Reference();
    poFeatureDefn->SetGeomType( eReqType );

    bWriter = bWriterIn;

/* -------------------------------------------------------------------- */
/*      Reader's should get the corresponding GMLFeatureClass and       */
/*      cache it.                                                       */
/* -------------------------------------------------------------------- */
    if( !bWriter )
        poFClass = poDS->GetReader()->GetClass( pszName );
    else
        poFClass = NULL;
}

/************************************************************************/
/*                           ~OGRGMLLayer()                           */
/************************************************************************/

OGRGMLLayer::~OGRGMLLayer()

{
    CPLFree(pszFIDPrefix);

    if( poFeatureDefn )
        poFeatureDefn->Release();

    if( poSRS != NULL )
        poSRS->Release();
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRGMLLayer::ResetReading()

{
    iNextGMLId = 0;
    poDS->GetReader()->ResetReading();
}

/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRGMLLayer::GetNextFeature()

{
    GMLFeature  *poGMLFeature = NULL;
    OGRGeometry *poGeom = NULL;

    if (bWriter)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "Cannot read features when writing a GML file");
        return NULL;
    }

    if( iNextGMLId == 0 )
        ResetReading();

/* ==================================================================== */
/*      Loop till we find and translate a feature meeting all our       */
/*      requirements.                                                   */
/* ==================================================================== */
    while( TRUE )
    {
/* -------------------------------------------------------------------- */
/*      Cleanup last feature, and get a new raw gml feature.            */
/* -------------------------------------------------------------------- */
        if( poGMLFeature != NULL )
            delete poGMLFeature;

        if( poGeom != NULL )
        {
            delete poGeom;
            poGeom = NULL;
        }

        poGMLFeature = poDS->GetReader()->NextFeature();
        if( poGMLFeature == NULL )
            return NULL;

/* -------------------------------------------------------------------- */
/*      Is it of the proper feature class?                              */
/* -------------------------------------------------------------------- */

        // We count reading low level GML features as a feature read for
        // work checking purposes, though at least we didn't necessary
        // have to turn it into an OGRFeature.
        m_nFeaturesRead++;

        if( poGMLFeature->GetClass() != poFClass )
            continue;

/* -------------------------------------------------------------------- */
/*      Extract the fid:                                                */
/*      -Assumes the fids are non-negative integers with an optional    */
/*       prefix                                                         */
/*      -If a prefix differs from the prefix of the first feature from  */
/*       the poDS then the fids from the poDS are ignored and are       */
/*       assigned serially thereafter                                   */
/* -------------------------------------------------------------------- */
        int nFID;
        if( bInvalidFIDFound )
        {
            nFID = iNextGMLId++;
        }
        else if( poGMLFeature->GetFID() == NULL )
        {
            bInvalidFIDFound = TRUE;
            nFID = iNextGMLId++;
        }
        else if( iNextGMLId == 0 )
        {
            int i = strlen( poGMLFeature->GetFID() )-1, j = 0;
            while( i >= 0 && *(poGMLFeature->GetFID()+i) >= '0'
                          && *(poGMLFeature->GetFID()+i) <= '9' && j<8)
                i--, j++;
            /* i points the last character of the fid */
            if( i >= 0 && j < 8)
            {
                pszFIDPrefix = (char *) CPLMalloc(i+2);
                pszFIDPrefix[i+1] = '\0';
                strncpy(pszFIDPrefix, poGMLFeature->GetFID(), i+1);
            }
            /* pszFIDPrefix now contains the prefix or NULL if no prefix is found */
            if( j < 8 && sscanf(poGMLFeature->GetFID()+i+1, "%d", &nFID)==1)
            {
                if( iNextGMLId <= nFID )
                    iNextGMLId = nFID + 1;
            }
            else
            {
                bInvalidFIDFound = TRUE;
                nFID = iNextGMLId++;
            }
        }
        else if( iNextGMLId != 0 )
        {
            const char * pszGML_FID = poGMLFeature->GetFID();
            if( (pszFIDPrefix == NULL || strstr( pszGML_FID, pszFIDPrefix) == pszGML_FID) &&
                strlen(pszGML_FID) <= strlen(pszFIDPrefix) + 9 &&
                sscanf(poGMLFeature->GetFID()+(pszFIDPrefix==NULL?0:strlen(pszFIDPrefix)), "%d", &nFID) == 1 )
            { /* fid with the prefix. Using its numerical part */
                if( iNextGMLId < nFID )
                    iNextGMLId = nFID + 1;
            }
            else
            { /* fid without the aforementioned prefix or a valid numerical part */
                bInvalidFIDFound = TRUE;
                nFID = iNextGMLId++;
            }
        }

/* -------------------------------------------------------------------- */
/*      Does it satisfy the spatial query, if there is one?             */
/* -------------------------------------------------------------------- */

        if( poGMLFeature->GetGeometry() != NULL )
        {
            poGeom = OGRGeometryFactory::createFromGML( poGMLFeature->GetGeometry() );
            // We assume the createFromGML() function would have already
            // reported the error. 
            if( poGeom == NULL )
            {
                delete poGMLFeature;
                return NULL;
            }
            
            if( m_poFilterGeom != NULL && !FilterGeometry( poGeom ) )
                continue;
        }
        
/* -------------------------------------------------------------------- */
/*      Convert the whole feature into an OGRFeature.                   */
/* -------------------------------------------------------------------- */
        int iField;
        OGRFeature *poOGRFeature = new OGRFeature( GetLayerDefn() );

        poOGRFeature->SetFID( nFID );

        for( iField = 0; iField < poFClass->GetPropertyCount(); iField++ )
        {
            const char *pszProperty = poGMLFeature->GetProperty( iField );
            
            if( pszProperty != NULL )
                poOGRFeature->SetField( iField, pszProperty );
        }

/* -------------------------------------------------------------------- */
/*      Test against the attribute query.                               */
/* -------------------------------------------------------------------- */
        if( m_poAttrQuery != NULL
            && !m_poAttrQuery->Evaluate( poOGRFeature ) )
        {
            delete poOGRFeature;
            continue;
        }

/* -------------------------------------------------------------------- */
/*      Wow, we got our desired feature. Return it.                     */
/* -------------------------------------------------------------------- */
        delete poGMLFeature;

        poOGRFeature->SetGeometryDirectly( poGeom );

        return poOGRFeature;
    }

    return NULL;
}

/************************************************************************/
/*                          GetFeatureCount()                           */
/************************************************************************/

int OGRGMLLayer::GetFeatureCount( int bForce )

{
    if( poFClass == NULL )
        return 0;

    if( m_poFilterGeom != NULL || m_poAttrQuery != NULL )
        return OGRLayer::GetFeatureCount( bForce );
    else
    {
        /* If the schema is read from a .xsd file, we haven't read */
        /* the feature count, so compute it now */
        int nFeatureCount = poFClass->GetFeatureCount();
        if (nFeatureCount < 0)
        {
            nFeatureCount = OGRLayer::GetFeatureCount(bForce);
            poFClass->SetFeatureCount(nFeatureCount);
        }
        return nFeatureCount;
    }
}

/************************************************************************/
/*                             GetExtent()                              */
/************************************************************************/

OGRErr OGRGMLLayer::GetExtent(OGREnvelope *psExtent, int bForce )

{
    double dfXMin, dfXMax, dfYMin, dfYMax;

    if( poFClass != NULL && 
        poFClass->GetExtents( &dfXMin, &dfXMax, &dfYMin, &dfYMax ) )
    {
        psExtent->MinX = dfXMin;
        psExtent->MaxX = dfXMax;
        psExtent->MinY = dfYMin;
        psExtent->MaxY = dfYMax;

        return OGRERR_NONE;
    }
    else 
        return OGRLayer::GetExtent( psExtent, bForce );
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/

OGRErr OGRGMLLayer::CreateFeature( OGRFeature *poFeature )

{
    FILE        *fp = poDS->GetOutputFP();

    if( !bWriter )
        return OGRERR_FAILURE;

    VSIFPrintf( fp, "  <gml:featureMember>\n" );

    if( poFeature->GetFID() == OGRNullFID )
        poFeature->SetFID( iNextGMLId++ );

    VSIFPrintf( fp, "    <ogr:%s fid=\"F%ld\">\n", 
                poFeatureDefn->GetName(),
                poFeature->GetFID() );

    // Write out Geometry - for now it isn't indented properly.
    if( poFeature->GetGeometryRef() != NULL )
    {
        char    *pszGeometry;
        OGREnvelope sGeomBounds;

        pszGeometry = poFeature->GetGeometryRef()->exportToGML();
        VSIFPrintf( fp, "      <ogr:geometryProperty>%s</ogr:geometryProperty>\n",
                    pszGeometry );
        CPLFree( pszGeometry );

        poFeature->GetGeometryRef()->getEnvelope( &sGeomBounds );
        poDS->GrowExtents( &sGeomBounds );
    }

    // Write all "set" fields. 
    for( int iField = 0; iField < poFeatureDefn->GetFieldCount(); iField++ )
    {
        
        OGRFieldDefn *poField = poFeatureDefn->GetFieldDefn( iField );

        if( poFeature->IsFieldSet( iField ) )
        {
            const char *pszRaw = poFeature->GetFieldAsString( iField );

            while( *pszRaw == ' ' )
                pszRaw++;

            char *pszEscaped = OGRGetXML_UTF8_EscapedString( pszRaw );
            VSIFPrintf( fp, "      <ogr:%s>%s</ogr:%s>\n", 
                        poField->GetNameRef(), pszEscaped, 
                        poField->GetNameRef() );
            CPLFree( pszEscaped );
        }
    }

    VSIFPrintf( fp, "    </ogr:%s>\n", poFeatureDefn->GetName() );
    VSIFPrintf( fp, "  </gml:featureMember>\n" );

    return OGRERR_NONE;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRGMLLayer::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,OLCSequentialWrite) )
        return bWriter;

    else if( EQUAL(pszCap,OLCCreateField) )
        return bWriter && iNextGMLId == 0;

    else if( EQUAL(pszCap,OLCFastGetExtent) )
    {
        double  dfXMin, dfXMax, dfYMin, dfYMax;

        if( poFClass == NULL )
            return FALSE;

        return poFClass->GetExtents( &dfXMin, &dfXMax, &dfYMin, &dfYMax );
    }

    else if( EQUAL(pszCap,OLCFastFeatureCount) )
    {
        if( poFClass == NULL 
            || m_poFilterGeom != NULL 
            || m_poAttrQuery != NULL )
            return FALSE;

        return poFClass->GetFeatureCount() != -1;
    }

    else if( EQUAL(pszCap,OLCStringsAsUTF8) )
        return TRUE;

    else 
        return FALSE;
}

/************************************************************************/
/*                            CreateField()                             */
/************************************************************************/

OGRErr OGRGMLLayer::CreateField( OGRFieldDefn *poField, int bApproxOK )

{
    if( !bWriter || iNextGMLId != 0 )
        return OGRERR_FAILURE;

/* -------------------------------------------------------------------- */
/*      Enforce XML naming semantics on element name.                   */
/* -------------------------------------------------------------------- */
    OGRFieldDefn oCleanCopy( poField );
    char *pszName = CPLStrdup( poField->GetNameRef() );
    CPLCleanXMLElementName( pszName );
    
    if( strcmp(pszName,poField->GetNameRef()) != 0 )
    {
        if( !bApproxOK )
        {
            CPLFree( pszName );
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Unable to create field with name '%s', it would not\n"
                      "be valid as an XML element name.",
                      poField->GetNameRef() );
            return OGRERR_FAILURE;
        }

        oCleanCopy.SetName( pszName );
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "Field name '%s' adjusted to '%s' to be a valid\n"
                  "XML element name.",
                  poField->GetNameRef(), pszName );
    }

    CPLFree( pszName );

    
    poFeatureDefn->AddFieldDefn( &oCleanCopy );

    return OGRERR_NONE;
}

/************************************************************************/
/*                           GetSpatialRef()                            */
/************************************************************************/

OGRSpatialReference *OGRGMLLayer::GetSpatialRef()

{
    return poSRS;
}

