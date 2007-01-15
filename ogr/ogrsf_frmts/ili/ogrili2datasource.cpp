/******************************************************************************
 * $Id$
 *
 * Project:  Interlis 2 Translator
 * Purpose:  Implements OGRILI2DataSource class.
 * Author:   Markus Schnider, Sourcepole AG
 *
 ******************************************************************************
 * Copyright (c) 2004, Pirmin Kalberer, Sourcepole AG
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
 * Revision 1.8  2006/10/24 03:21:55  fwarmerdam
 * Fixed memory leak of filenames[] token array.
 *
 * Revision 1.7  2006/04/27 16:37:19  pka
 * Ili2 model reader fix
 * Support for multiple Ili2 models
 *
 * Revision 1.6  2006/04/24 16:49:48  pka
 * Fixed polyline feature with coordinate attribute
 * Float support for ARC_DEGREES
 *
 * Revision 1.5  2006/03/28 16:07:14  pka
 * Optional model file for Interlis 2 reader
 *
 * Revision 1.4  2005/12/20 16:47:04  pka
 * Interlis 1 output without model
 *
 * Revision 1.3  2005/12/19 17:33:21  pka
 * Interlis 1: Support for 100 columns (unlimited, if model given)
 * Interlis 1: Fixes for output
 * Interlis: Examples in driver documentation
 *
 * Revision 1.2  2005/08/06 22:21:53  pka
 * Area polygonizer added
 *
 * Revision 1.1  2005/07/08 22:10:57  pka
 * Initial import of OGR Interlis driver
 *
 */

#include "ogr_ili2.h"
#include "cpl_conv.h"
#include "cpl_string.h"

#include "ili2reader.h"
#include "iomhelper.h"

using namespace std;


CPL_CVSID("$Id$");

/************************************************************************/
/*                         OGRILI2DataSource()                         */
/************************************************************************/

OGRILI2DataSource::OGRILI2DataSource()

{
    pszName = NULL;
    poReader = NULL;
    fpTransfer = NULL;
    basket = NULL;
}

/************************************************************************/
/*                        ~OGRILI2DataSource()                         */
/************************************************************************/

OGRILI2DataSource::~OGRILI2DataSource()

{
    if (basket) iom_releasebasket(basket);
    if (fpTransfer)
    {  
      // write file
      iom_save(fpTransfer);
  
      // clean up
      iom_close(fpTransfer);
  
      iom_end();
  
    }
    CPLFree( pszName );
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

int OGRILI2DataSource::Open( const char * pszNewName, int bTestOpen )

{
    FILE        *fp;
    char        szHeader[1000];

    char **modelFilenames = NULL;
    char **filenames = CSLTokenizeString2( pszNewName, ",", 0 );

    pszName = CPLStrdup( filenames[0] );

    if( CSLCount(filenames) > 1 )
        modelFilenames = &filenames[1];

    CSLDestroy( filenames );

/* -------------------------------------------------------------------- */
/*      Open the source file.                                           */
/* -------------------------------------------------------------------- */
    fp = VSIFOpen( pszName, "r" );
    if( fp == NULL )
    {
        if( !bTestOpen )
            CPLError( CE_Failure, CPLE_OpenFailed, 
                      "Failed to open ILI2 file `%s'.", 
                      pszNewName );

        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      If we aren't sure it is ILI2, load a header chunk and check      */
/*      for signs it is ILI2                                             */
/* -------------------------------------------------------------------- */
    if( bTestOpen )
    {
        VSIFRead( szHeader, 1, sizeof(szHeader), fp );
        szHeader[sizeof(szHeader)-1] = '\0';

        if( szHeader[0] != '<' 
            && strstr(szHeader,"interlis.ch/INTERLIS2") == NULL )
        { // "www.interlis.ch/INTERLIS2.2"
            VSIFClose( fp );
            return FALSE;
        }
    }
    
/* -------------------------------------------------------------------- */
/*      We assume now that it is ILI2.  Close and instantiate a          */
/*      ILI2Reader on it.                                                */
/* -------------------------------------------------------------------- */
    VSIFClose( fp );
    
    poReader = CreateILI2Reader();
    if( poReader == NULL )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "File %s appears to be ILI2 but the ILI2 reader can't\n"
                  "be instantiated, likely because Xerces support wasn't\n"
                  "configured in.", 
                  pszNewName );
        return FALSE;
    }

    if (modelFilenames)
        poReader->ReadModel( modelFilenames );

    if( getenv( "ARC_DEGREES" ) != NULL ) {
      //No better way to pass arguments to the reader (it could even be an -lco arg)
      poReader->SetArcDegrees( atof( getenv("ARC_DEGREES") ) );
    }

    poReader->SetSourceFile( pszName );

    poReader->SaveClasses( pszName );

    listLayer = poReader->GetLayers();

    return TRUE;
}


/************************************************************************/
/*                               Create()                               */
/************************************************************************/

int OGRILI2DataSource::Create( const char *pszFilename, 
                              char **papszOptions )

{
    char **filenames = CSLTokenizeString2( pszFilename, ",", 0 );
    pszName = filenames[0];
    pszModelFilename = (CSLCount(filenames)>1) ? filenames[1] : NULL;

    if( pszModelFilename == NULL )
    {
        CPLError( CE_Warning, CPLE_OpenFailed, 
                  "Model file '%s' (%s) not found.", 
                  pszModelFilename, pszFilename, VSIStrerror( errno ) );
        return FALSE;
    }

	iom_init();

	// set error listener to a iom provided one, that just 
	// dumps all errors to stderr
	iom_seterrlistener(iom_stderrlistener);

	// compile ili model
    char *iliFiles[1] = {(char *)pszModelFilename};
	IOM_BASKET model=iom_compileIli(1,iliFiles);
	if(!model){
        CPLError( CE_Warning, CPLE_OpenFailed, 
                  "iom_compileIli .", 
                  pszName, VSIStrerror( errno ) );
		iom_end();
        return FALSE;
	}

	// open new file
	fpTransfer=iom_open(pszName,IOM_CREATE | IOM_DONTREAD,0);
	if(!fpTransfer){
        CPLError( CE_Warning, CPLE_OpenFailed, 
                  "Failed to open %s.", 
                  pszName );
        return FALSE;
	}

	// set model of new file
	iom_setmodel(fpTransfer,model);

	iom_setheadsender(fpTransfer, pszModelFilename);

	iom_setheadcomment(fpTransfer,"Created by OGR");

    // create new basket
    static char basketname[512];
    basketname[0] = '\0';
    const char* val = GetAttrObjName(model, "iom04.metamodel.DataModel");
    if (val)
    {
      strcat(basketname, val);
      strcat(basketname, ".");
      val = GetAttrObjName(model, "iom04.metamodel.Topic");
      if (val) strcat(basketname, val);
    }
    else
    {
      strcat(basketname, "Basket");
    }

    basket=iom_newbasket(fpTransfer);
    iom_setbaskettag(basket, basketname);
    iom_setbasketoid(basket, "0");
    return TRUE;
}

/************************************************************************/
/*                            CreateLayer()                             */
/************************************************************************/

OGRLayer *
OGRILI2DataSource::CreateLayer( const char * pszLayerName,
                               OGRSpatialReference *poSRS,
                               OGRwkbGeometryType eType,
                               char ** papszOptions )

{
    OGRILI2Layer *poLayer = new OGRILI2Layer(CPLStrdup(pszLayerName), poSRS, TRUE, eType, this);
    return poLayer;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRILI2DataSource::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,ODsCCreateLayer) )
        return TRUE;
    else
        return FALSE;
}

/************************************************************************/
/*                              GetLayer()                              */
/************************************************************************/

OGRLayer *OGRILI2DataSource::GetLayer( int iLayer )

{
  list<OGRLayer *>::const_iterator layerIt = listLayer.begin();
  int i = 0;
  while (i < iLayer && layerIt != listLayer.end()) {
    i++;
    layerIt++;
  }
  
  if (i == iLayer) {
    OGRILI2Layer *tmpLayer = (OGRILI2Layer *)*layerIt;
    return tmpLayer;
  } else
    return NULL;
}
