/******************************************************************************
 * $Id$
 *
 * Project:  KML Driver
 * Purpose:  Implementation of OGRKMLDataSource class.
 * Author:   Christopher Condit, condit@sdsc.edu;
 *           Jens Oberender, j.obi@troja.net
 *
 ******************************************************************************
 * Copyright (c) 2006, Christopher Condit
 *               2007, Jens Oberender
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
#include "cpl_conv.h"
#include "cpl_string.h"
#include "cpl_error.h"
#include "cpl_minixml.h"

/************************************************************************/
/*                         OGRKMLDataSource()                           */
/************************************************************************/
OGRKMLDataSource::OGRKMLDataSource()
{
    pszName = NULL;
    pszNameField = NULL;
    papoLayers = NULL;
    nLayers = 0;
    
    fpOutput = NULL;

    papszCreateOptions = NULL;
}

/************************************************************************/
/*                        ~OGRKMLDataSource()                           */
/************************************************************************/
OGRKMLDataSource::~OGRKMLDataSource()
{
    if( fpOutput != NULL )
    {
        VSIFPrintf( fpOutput, "%s", 
                    "</Folder></Document></kml>\n" );
        
        if( fpOutput != stdout )
            VSIFClose( fpOutput );
    }

    CSLDestroy( papszCreateOptions );
    CPLFree( pszName );
    CPLFree( pszNameField );

    for( int i = 0; i < nLayers; i++ )
    {
        delete papoLayers[i];
    }
    
    CPLFree( papoLayers );
    
    if(this->poKMLFile != NULL)
        delete this->poKMLFile;
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/
int OGRKMLDataSource::Open( const char * pszNewName, int bTestOpen )
{
    int nCount;
    OGRKMLLayer *poLayer;
    OGRwkbGeometryType poGeotype;

    CPLAssert( NULL != pszNewName );

/* -------------------------------------------------------------------- */
/*      Create a KML object and open the source file.                   */
/* -------------------------------------------------------------------- */
    this->poKMLFile = new KMLvector();
    if( !this->poKMLFile->open( pszNewName )) {
        delete this->poKMLFile;
        this->poKMLFile = NULL;
        return FALSE;
    }
    this->pszName = CPLStrdup( pszNewName );

/* -------------------------------------------------------------------- */
/*      If we aren't sure it is KML, validate it by start parsing       */
/* -------------------------------------------------------------------- */
    if( bTestOpen && !poKMLFile->isValid())
    {
        delete this->poKMLFile;
        this->poKMLFile = NULL;
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Prescan the KML file so we can later work with the structure    */
/* -------------------------------------------------------------------- */
    this->poKMLFile->parse();

/* -------------------------------------------------------------------- */
/*      Classify the nodes                                              */
/* -------------------------------------------------------------------- */
    this->poKMLFile->classifyNodes();

/* -------------------------------------------------------------------- */
/*      Eliminate the empty containers                                  */
/* -------------------------------------------------------------------- */
    this->poKMLFile->eliminateEmpty();

/* -------------------------------------------------------------------- */
/*      Find layers to use in the KML structure                         */
/* -------------------------------------------------------------------- */
    this->poKMLFile->findLayers(NULL);

/* -------------------------------------------------------------------- */
/*      Print the structure                                             */
/* -------------------------------------------------------------------- */
    this->poKMLFile->print(3);

    nLayers = this->poKMLFile->numLayers();

/* -------------------------------------------------------------------- */
/*      Allocate memory for the Layers                                  */
/* -------------------------------------------------------------------- */
    papoLayers = (OGRKMLLayer **)
        CPLMalloc( sizeof(OGRKMLLayer *) * nLayers );

    OGRSpatialReference *poSRS = new OGRSpatialReference("GEOGCS[\"WGS 84\", "
        "   DATUM[\"WGS_1984\","
        "       SPHEROID[\"WGS 84\",6378137,298.257223563,"
        "           AUTHORITY[\"EPSG\",\"7030\"]],"
        "           AUTHORITY[\"EPSG\",\"6326\"]],"
        "       PRIMEM[\"Greenwich\",0,"
        "           AUTHORITY[\"EPSG\",\"8901\"]],"
        "       UNIT[\"degree\",0.01745329251994328,"
        "           AUTHORITY[\"EPSG\",\"9122\"]],"
        "           AUTHORITY[\"EPSG\",\"4326\"]]");

/* -------------------------------------------------------------------- */
/*      Create the Layers and fill them                                 */
/* -------------------------------------------------------------------- */
    for(nCount = 0; nCount < nLayers; nCount++) {
        CPLDebug("KML", "Loading Layer #%d", nCount);
        if(!this->poKMLFile->selectLayer(nCount)) {
            CPLError(CE_Failure, CPLE_AppDefined,
                "There are no layers or a layer can not be found!");
            break;
        }

        if(this->poKMLFile->getCurrentType() == Point)
            poGeotype = wkbPoint;
        else if(this->poKMLFile->getCurrentType() == LineString)
            poGeotype = wkbLineString;
        else if(this->poKMLFile->getCurrentType() == Polygon)
            poGeotype = wkbPolygon;
        else
            poGeotype = wkbUnknown;

/* -------------------------------------------------------------------- */
/*      Create the layer object.                                        */
/* -------------------------------------------------------------------- */
        std::string sName = this->poKMLFile->getCurrentName();
        if(sName.compare("") == 0) {
            char *pszName = new char[10];
            snprintf(pszName, 10, "Layer #%d", nCount);
            sName = pszName;
            if(pszName != NULL)
                delete pszName;
        }

        poLayer = new OGRKMLLayer( sName.c_str(), poSRS, FALSE, poGeotype, this );

        poLayer->SetLayerNumber(nCount);

/* -------------------------------------------------------------------- */
/*      Add layer to data source layer list.                            */
/* -------------------------------------------------------------------- */
        papoLayers[nCount] = poLayer;
    }
    
    poSRS->Release();
    
    return TRUE;	
}

/************************************************************************/
/*                               Create()                               */
/************************************************************************/
int OGRKMLDataSource::Create( const char *pszFilename, 
                              char **papszOptions )
{
    CPLAssert( NULL != pszFilename );

    if( fpOutput != NULL )
    {
        CPLAssert( FALSE );
        return FALSE;
    }

    pszNameField = CPLStrdup(CSLFetchNameValue(papszOptions, "NameField"));
    CPLDebug("KML", "Using the field '%s' for name element", pszNameField);
    
/* -------------------------------------------------------------------- */
/*      Create the output file.                                         */
/* -------------------------------------------------------------------- */
    pszName = CPLStrdup( pszFilename );

    
    if( EQUAL(pszFilename,"stdout") )
        fpOutput = stdout;
    else
        fpOutput = VSIFOpen( pszFilename, "wt+" );

    if( fpOutput == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "Failed to create KML file %s.", 
                  pszFilename );
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Write out "standard" header.                                    */
/* -------------------------------------------------------------------- */
    VSIFPrintf( fpOutput, "<?xml version=\"1.0\" encoding=\"utf-8\" ?>\n" );	

    nSchemaInsertLocation = VSIFTell( fpOutput );
    
    VSIFPrintf( fpOutput, "<kml xmlns=\"http://earth.google.com/kml/2.0\">\n<Document>" );

    return TRUE;
}

/************************************************************************/
/*                            CreateLayer()                             */
/************************************************************************/

OGRLayer *
OGRKMLDataSource::CreateLayer( const char * pszLayerName,
                               OGRSpatialReference *poSRS,
                               OGRwkbGeometryType eType,
                               char ** papszOptions )
{
    CPLAssert( NULL != pszLayerName);

/* -------------------------------------------------------------------- */
/*      Verify we are in update mode.                                   */
/* -------------------------------------------------------------------- */
    if( fpOutput == NULL )
    {
        CPLError( CE_Failure, CPLE_NoWriteAccess,
                  "Data source %s opened for read access.\n"
                  "New layer %s cannot be created.\n",
                  pszName, pszLayerName );

        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Close the previous layer (if there is one open)                 */
/* -------------------------------------------------------------------- */
    if (GetLayerCount() > 0)
        VSIFPrintf( fpOutput, "</Folder>\n");

    
/* -------------------------------------------------------------------- */
/*      Ensure name is safe as an element name.                         */
/* -------------------------------------------------------------------- */
    char *pszCleanLayerName = CPLStrdup( pszLayerName );

    CPLCleanXMLElementName( pszCleanLayerName );
    if( strcmp(pszCleanLayerName, pszLayerName) != 0 )
    {
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "Layer name '%s' adjusted to '%s' for XML validity.",
                  pszLayerName, pszCleanLayerName );
    }
    VSIFPrintf( fpOutput, "<Folder><name>%s</name>\n", pszCleanLayerName);
    
/* -------------------------------------------------------------------- */
/*      Create the layer object.                                        */
/* -------------------------------------------------------------------- */
    OGRKMLLayer *poLayer;
    poLayer = new OGRKMLLayer( pszCleanLayerName, poSRS, TRUE, eType, this );
    CPLFree( pszCleanLayerName );

/* -------------------------------------------------------------------- */
/*      Add layer to data source layer list.                            */
/* -------------------------------------------------------------------- */
    papoLayers = (OGRKMLLayer **)
        CPLRealloc( papoLayers,  sizeof(OGRKMLLayer *) * (nLayers+1) );
    
    papoLayers[nLayers++] = poLayer;

    return poLayer;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/
int OGRKMLDataSource::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,ODsCCreateLayer) )
        return TRUE;
    else
        return FALSE;
}

/************************************************************************/
/*                              GetLayer()                              */
/************************************************************************/
OGRLayer *OGRKMLDataSource::GetLayer( int iLayer )
{
    CPLDebug("KML", "Get Layer #%d", iLayer);
    if( iLayer < 0 || iLayer >= nLayers )
        return NULL;
    else
        return papoLayers[iLayer];
}

/************************************************************************/
/*                            GrowExtents()                             */
/************************************************************************/
void OGRKMLDataSource::GrowExtents( OGREnvelope *psGeomBounds )
{
    CPLAssert( NULL != psGeomBounds );

    sBoundingRect.Merge( *psGeomBounds );
}

/************************************************************************/
/*                            InsertHeader()                            */
/*                                                                      */
/*      This method is used to update boundedby info for a              */
/*      dataset, and insert schema descriptions depending on            */
/*      selection options in effect.                                    */
/************************************************************************/
void OGRKMLDataSource::InsertHeader()
{    
        return;
}
