/******************************************************************************
 * $Id$
 *
 * Project:  KML Driver
 * Purpose:  Implementation of OGRKMLLayer class.
 * Author:   Christopher Condit, condit@sdsc.edu
 *           Jens Oberender, j.obi@troja.net
 *
 ******************************************************************************
 * Copyright (c) 2006, Christopher Condit
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

#include "ogr_kml.h"
#include "ogr_api.h"
#include "cpl_conv.h"
#include "cpl_string.h"

/* Function utility to dump OGRGeoemtry to KML text. */
char *OGR_G_ExportToKML( OGRGeometryH hGeometry, const char* pszAltitudeMode );

/************************************************************************/
/*                           OGRKMLLayer()                              */
/************************************************************************/

OGRKMLLayer::OGRKMLLayer( const char * pszName,
                          OGRSpatialReference *poSRSIn, int bWriterIn,
                          OGRwkbGeometryType eReqType,
                          OGRKMLDataSource *poDSIn )
{    	
	/* KML should be created as WGS84. */
	poSRS_ = new OGRSpatialReference(NULL);   
	poSRS_->SetWellKnownGeogCS( "WGS84" );
	poCT_ = OGRCreateCoordinateTransformation( poSRSIn, poSRS_ );
    if( poCT_ == NULL && poDSIn->IsFirstCTError() )
	{
		/* If we can't create a transformation, issue a warning - but continue the transformation*/
		char *pszWKT = NULL;
		
		printf("Failed to create coordinate transformation between the\n"
			   "following coordinate systems.  This may be because they\n"
			   "are not transformable, or because projection services\n"
			   "(PROJ.4 DLL/.so) could not be loaded.\n" 
			   "KML geometries may not render correctly.\n"
			   "This message will not be issued any more. \n");
		
		poSRSIn->exportToPrettyWkt( &pszWKT, FALSE );
		printf( "Source:\n%s\n", pszWKT );
		CPLFree( pszWKT );
		
		poSRS_->exportToPrettyWkt( &pszWKT, FALSE );
		printf( "Target:\n%s\n", pszWKT );
		CPLFree( pszWKT );

		poDSIn->IssuedFirstCTError(); 
	}

    iNextKMLId_ = 0;
    nTotalKMLCount_ = -1;
    
    poDS_ = poDSIn;
    
    poFeatureDefn_ = new OGRFeatureDefn( pszName );
    poFeatureDefn_->Reference();
    poFeatureDefn_->SetGeomType( eReqType );

    OGRFieldDefn oFieldName( "Name", OFTString );
    poFeatureDefn_->AddFieldDefn( &oFieldName );
    
    OGRFieldDefn oFieldDesc( "Description", OFTString );
    poFeatureDefn_->AddFieldDefn( &oFieldDesc );

    bWriter_ = bWriterIn;
    nWroteFeatureCount_ = 0;

    pszName_ = CPLStrdup(pszName);
}

/************************************************************************/
/*                           ~OGRKMLLayer()                             */
/************************************************************************/

OGRKMLLayer::~OGRKMLLayer()
{
    if( NULL != poFeatureDefn_ )
        poFeatureDefn_->Release();

    if( NULL != poSRS_ )
        poSRS_->Release();
	
	if( NULL != poCT_ )
		delete poCT_;
	
    CPLFree( pszName_ );
}

/************************************************************************/
/*                            GetLayerDefn()                            */
/************************************************************************/

OGRFeatureDefn* OGRKMLLayer::GetLayerDefn()
{
    return poFeatureDefn_;
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRKMLLayer::ResetReading()
{
    iNextKMLId_ = 0;    
}

/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRKMLLayer::GetNextFeature()
{
#ifndef HAVE_EXPAT
    return NULL;
#else
    /* -------------------------------------------------------------------- */
    /*      Loop till we find a feature matching our criteria.              */
    /* -------------------------------------------------------------------- */
    while(TRUE)
    {
        unsigned short nCount = 0;
        unsigned short nCount2 = 0;
        KML *poKMLFile = poDS_->GetKMLFile();
        poKMLFile->selectLayer(nLayerNumber_);
    
        Feature *poFeatureKML = NULL;
        poFeatureKML = poKMLFile->getFeature(iNextKMLId_++);
    
        if(poFeatureKML == NULL)
            return NULL;
    
        CPLAssert( poFeatureKML != NULL );
    
        OGRFeature *poFeature = new OGRFeature( poFeatureDefn_ );
        
        // Handle a Point
        if(poFeatureKML->eType == Point)
        {
            poFeature->SetGeometryDirectly(
                    new OGRPoint( (*poFeatureKML->pvpsCoordinates)[0]->dfLongitude, (*poFeatureKML->pvpsCoordinates)[0]->dfLatitude, (*poFeatureKML->pvpsCoordinates)[0]->dfAltitude)
                    );
        }
        // Handle a LineString
        else if(poFeatureKML->eType == LineString)
        {
            OGRLineString *poLS = new OGRLineString();
            for(nCount = 0; nCount < poFeatureKML->pvpsCoordinates->size(); nCount++)
            {
                poLS->addPoint( (*poFeatureKML->pvpsCoordinates)[nCount]->dfLongitude, (*poFeatureKML->pvpsCoordinates)[nCount]->dfLatitude, (*poFeatureKML->pvpsCoordinates)[nCount]->dfAltitude );
            }
            poFeature->SetGeometryDirectly(poLS);
        }
        // Handle a Polygon
        else if(poFeatureKML->eType == Polygon)
        {
            OGRPolygon *poPG = new OGRPolygon();
            OGRLinearRing *poLR = new OGRLinearRing();
            for(nCount = 0; nCount < poFeatureKML->pvpsCoordinates->size(); nCount++)
            {
                poLR->addPoint( (*poFeatureKML->pvpsCoordinates)[nCount]->dfLongitude, (*poFeatureKML->pvpsCoordinates)[nCount]->dfLatitude, (*poFeatureKML->pvpsCoordinates)[nCount]->dfAltitude );
            }
            poPG->addRingDirectly(poLR);
            for(nCount = 0; nCount < poFeatureKML->pvpsCoordinatesExtra->size(); nCount++)
            {
                poLR = new OGRLinearRing();
                for(nCount2 = 0; nCount2 < (*poFeatureKML->pvpsCoordinatesExtra)[nCount]->size(); nCount2++)
                {
                    poLR->addPoint( (*(*poFeatureKML->pvpsCoordinatesExtra)[nCount])[nCount2]->dfLongitude, 
                        (*(*poFeatureKML->pvpsCoordinatesExtra)[nCount])[nCount2]->dfLatitude, 
                        (*(*poFeatureKML->pvpsCoordinatesExtra)[nCount])[nCount2]->dfAltitude );
                }
                poPG->addRingDirectly(poLR);
            }
            poFeature->SetGeometryDirectly(poPG);
        }
    
        // Add fields
        poFeature->SetField( poFeatureDefn_->GetFieldIndex("Name"), poFeatureKML->sName.c_str() );
        poFeature->SetField( poFeatureDefn_->GetFieldIndex("Description"), poFeatureKML->sDescription.c_str() );
        poFeature->SetFID( iNextKMLId_ - 1 );
    
        // Clean up
        for(nCount = 0; nCount < poFeatureKML->pvpsCoordinates->size(); nCount++)
        {
            delete (*poFeatureKML->pvpsCoordinates)[nCount];
        }
    
        if(poFeatureKML->pvpsCoordinatesExtra != NULL)
        {
            for(nCount = 0; nCount < poFeatureKML->pvpsCoordinatesExtra->size(); nCount++)
            {
                for(nCount2 = 0; nCount2 < (*poFeatureKML->pvpsCoordinatesExtra)[nCount]->size(); nCount2++)
                    delete (*(*poFeatureKML->pvpsCoordinatesExtra)[nCount])[nCount2];
                delete (*poFeatureKML->pvpsCoordinatesExtra)[nCount];
            }
            delete poFeatureKML->pvpsCoordinatesExtra;
        }
    
        delete poFeatureKML->pvpsCoordinates;
        delete poFeatureKML;
    
        if( poFeature->GetGeometryRef() != NULL && poSRS_ != NULL)
        {
            poFeature->GetGeometryRef()->assignSpatialReference( poSRS_ );
        }
    
        /* Check spatial/attribute filters */
        if ((m_poFilterGeom == NULL || FilterGeometry( poFeature->GetGeometryRef() ) ) &&
            (m_poAttrQuery == NULL || m_poAttrQuery->Evaluate( poFeature )) )
        {
        // Return the feature
            return poFeature;
        }
        else
        {
            delete poFeature;
        }
    }

#endif /* HAVE_EXPAT */
}

/************************************************************************/
/*                          GetFeatureCount()                           */
/************************************************************************/

int OGRKMLLayer::GetFeatureCount( int bForce )
{
    int nCount = 0;

#ifdef HAVE_EXPAT
    if (m_poFilterGeom == NULL && m_poAttrQuery == NULL)
    {
        KML *poKMLFile = poDS_->GetKMLFile();
        if( NULL != poKMLFile )
        {
            poKMLFile->selectLayer(nLayerNumber_);
            nCount = poKMLFile->getNumFeatures();
        }
    }
    else
        return OGRLayer::GetFeatureCount(bForce);
#endif

    return nCount;
}

/************************************************************************/
/*                             GetExtent()                              */
/************************************************************************/

OGRErr OGRKMLLayer::GetExtent(OGREnvelope *psExtent, int bForce )
{
#ifdef HAVE_EXPAT
    CPLAssert( NULL != psExtent );

    KML *poKMLFile = poDS_->GetKMLFile();
    if( NULL != poKMLFile )
    {
        poKMLFile->selectLayer(nLayerNumber_);
        if( poKMLFile->getExtents( psExtent->MinX, psExtent->MaxX,
                                   psExtent->MinY, psExtent->MaxY ) )
        {
            return OGRERR_NONE;
        }
    }
#endif

    return OGRERR_FAILURE;
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/

OGRErr OGRKMLLayer::CreateFeature( OGRFeature* poFeature )
{
    CPLAssert( NULL != poFeature );
    CPLAssert( NULL != poDS_ );

    if( !bWriter_ )
        return OGRERR_FAILURE;

    FILE *fp = poDS_->GetOutputFP();
    CPLAssert( NULL != fp );

    // If we haven't writen any features yet, output the layer's schema
    if (0 == nWroteFeatureCount_)
    {
        VSIFPrintf( fp, "<Schema name=\"%s\" parent=\"Placemark\">\n", pszName_  );
        OGRFeatureDefn *featureDefinition = GetLayerDefn();
        for (int j=0; j < featureDefinition->GetFieldCount(); j++)
        {
            OGRFieldDefn *fieldDefinition = featureDefinition->GetFieldDefn(j);			
            const char* pszKMLType = NULL;
            const char* pszKMLEltName = NULL;
            // Match the OGR type to the GDAL type
            switch (fieldDefinition->GetType())
            {
            case OFTInteger:
                pszKMLType = "int";
                pszKMLEltName = "SimpleField";
                break;
            case OFTIntegerList:
                pszKMLType = "int";
                pszKMLEltName = "SimpleArrayField";
                break;
            case OFTReal:
                pszKMLType = "float";
                pszKMLEltName = "SimpleField";
                break;
            case OFTRealList:
                pszKMLType = "float";
                pszKMLEltName = "SimpleArrayField";
                break;
            case OFTString:
                pszKMLType = "string";
                pszKMLEltName = "SimpleField";
                break;
            case OFTStringList:
                pszKMLType = "string";
                pszKMLEltName = "SimpleArrayField";
                break;
            case OFTBinary:
                pszKMLType = "bool";
                pszKMLEltName = "SimpleField";
                break;
            //TODO: KML doesn't handle these data types yet...
            case OFTDate:                
            case OFTTime:                
            case OFTDateTime:
				pszKMLType = "string";
                pszKMLEltName = "SimpleField";                
                break;

              default:
                pszKMLType = "string";
                pszKMLEltName = "SimpleField";
                break;
            }
            VSIFPrintf( fp, "\t<%s name=\"%s\" type=\"%s\"></%s>\n", 
                    pszKMLEltName, fieldDefinition->GetNameRef() ,pszKMLType, pszKMLEltName );
        }
        VSIFPrintf( fp, "</Schema>\n" );
    }

    VSIFPrintf( fp, "  <Placemark>\n" );

    if( poFeature->GetFID() == OGRNullFID )
        poFeature->SetFID( iNextKMLId_++ );

    // Find and write the name element
    if (NULL != poDS_->GetNameField())
    {
        for( int iField = 0; iField < poFeatureDefn_->GetFieldCount(); iField++ )
        {        
            OGRFieldDefn *poField = poFeatureDefn_->GetFieldDefn( iField );

            if( poFeature->IsFieldSet( iField )
                && EQUAL(poField->GetNameRef(), poDS_->GetNameField()) )
            {           
                const char *pszRaw = poFeature->GetFieldAsString( iField );
                while( *pszRaw == ' ' )
                    pszRaw++;

                char *pszEscaped = CPLEscapeString( pszRaw, -1, CPLES_XML );

                VSIFPrintf( fp, "\t<name>%s</name>\n", pszEscaped);
                CPLFree( pszEscaped );
            }
        }
    }

    if (NULL != poDS_->GetDescriptionField())
    {
        for( int iField = 0; iField < poFeatureDefn_->GetFieldCount(); iField++ )
        {        
            OGRFieldDefn *poField = poFeatureDefn_->GetFieldDefn( iField );

            if( poFeature->IsFieldSet( iField )
                && EQUAL(poField->GetNameRef(), poDS_->GetDescriptionField()) )
            {           
                const char *pszRaw = poFeature->GetFieldAsString( iField );
                while( *pszRaw == ' ' )
                    pszRaw++;

                char *pszEscaped = CPLEscapeString( pszRaw, -1, CPLES_XML );

                VSIFPrintf( fp, "\t<description>%s</description>\n", pszEscaped);
                CPLFree( pszEscaped );
            }
        }
    }

    int bHasFoundOtherField = FALSE;

    // Write all fields as SchemaData
    for( int iField = 0; iField < poFeatureDefn_->GetFieldCount(); iField++ )
    {
        OGRFieldDefn *poField = poFeatureDefn_->GetFieldDefn( iField );

        if( poFeature->IsFieldSet( iField ))
        {
            if (!bHasFoundOtherField)
            {                
                VSIFPrintf( fp, "\t<ExtendedData><SchemaData schemaURL=\"#%s\">\n", pszName_ );
                bHasFoundOtherField = TRUE;
            }
            const char *pszRaw = poFeature->GetFieldAsString( iField );

            while( *pszRaw == ' ' )
                pszRaw++;

            char *pszEscaped = CPLEscapeString( pszRaw, -1, CPLES_XML );

            VSIFPrintf( fp, "\t\t<SimpleData name=\"%s\">%s</SimpleData>\n", 
                        poField->GetNameRef(), pszEscaped);

            CPLFree( pszEscaped );
        }
    }

    if (bHasFoundOtherField)
    {
        VSIFPrintf( fp, "\t</SchemaData></ExtendedData>\n" );
    }

    // Write out Geometry - for now it isn't indented properly.
    if( poFeature->GetGeometryRef() != NULL )
    {
        char* pszGeometry = NULL;
        OGREnvelope sGeomBounds;		
				
		if (NULL != poCT_)
			poFeature->GetGeometryRef()->transform( poCT_ );		
		
        // TODO - porting
        // pszGeometry = poFeature->GetGeometryRef()->exportToKML();
        pszGeometry = 
            OGR_G_ExportToKML( static_cast<OGRGeometryH>( poFeature->GetGeometryRef() ),
                               poDS_->GetAltitudeMode());
        
        VSIFPrintf( fp, "      %s\n", pszGeometry );
        CPLFree( pszGeometry );

        poFeature->GetGeometryRef()->getEnvelope( &sGeomBounds );
        poDS_->GrowExtents( &sGeomBounds );
    }
    
    if ( wkbPolygon == poFeatureDefn_->GetGeomType()
         || wkbMultiPolygon == poFeatureDefn_->GetGeomType()
         || wkbLineString == poFeatureDefn_->GetGeomType()
         || wkbMultiLineString == poFeatureDefn_->GetGeomType() )
    {
        //If we're dealing with a polygon, add a line style that will stand out a bit
        VSIFPrintf( fp, "  <Style><LineStyle><color>ff0000ff</color></LineStyle>");
        VSIFPrintf( fp, "  <PolyStyle><fill>0</fill></PolyStyle></Style>\n" );
    }

    VSIFPrintf( fp, "  </Placemark>\n" );
    nWroteFeatureCount_++;
    return OGRERR_NONE;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRKMLLayer::TestCapability( const char * pszCap )
{
    if( EQUAL(pszCap, OLCSequentialWrite) )
    {
        return bWriter_;
    }
    else if( EQUAL(pszCap, OLCCreateField) )
    {
        return bWriter_ && iNextKMLId_ == 0;
    }
    else if( EQUAL(pszCap, OLCFastGetExtent) )
    {
//        double  dfXMin, dfXMax, dfYMin, dfYMax;
//        if( poFClass == NULL )
            return FALSE;

//        return poFClass->GetExtents( &dfXMin, &dfXMax, &dfYMin, &dfYMax );
    }
    else if( EQUAL(pszCap,OLCFastFeatureCount) )
    {
//        if( poFClass == NULL 
//            || m_poFilterGeom != NULL 
//            || m_poAttrQuery != NULL )
            return FALSE;

//        return poFClass->GetFeatureCount() != -1;
    }

    return FALSE;
}

/************************************************************************/
/*                            CreateField()                             */
/************************************************************************/

OGRErr OGRKMLLayer::CreateField( OGRFieldDefn *poField, int bApproxOK )
{
    if( !bWriter_ || iNextKMLId_ != 0 )
        return OGRERR_FAILURE;
		  
	OGRFieldDefn oCleanCopy( poField );
    poFeatureDefn_->AddFieldDefn( &oCleanCopy );

    return OGRERR_NONE;
}

/************************************************************************/
/*                           GetSpatialRef()                            */
/************************************************************************/

OGRSpatialReference *OGRKMLLayer::GetSpatialRef()
{
    return poSRS_;
}

/************************************************************************/
/*                           SetLayerNumber()                           */
/************************************************************************/

void OGRKMLLayer::SetLayerNumber( int nLayer )
{
    nLayerNumber_ = nLayer;
}

