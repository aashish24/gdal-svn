/******************************************************************************
 * $Id$
 *
 * Project:  KML Driver
 * Purpose:  Declarations for OGR wrapper classes for KML, and OGR->KML
 *           translation of geometry.
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
#ifndef OGR_KML_H_INCLUDED
#define OGR_KML_H_INCLUDED

#include "ogrsf_frmts.h"
#include "kmlvector.h"

class OGRKMLDataSource;

/************************************************************************/
/*                            OGRKMLLayer                               */
/************************************************************************/

class OGRKMLLayer : public OGRLayer
{
private:
    OGRSpatialReference *poSRS;
    OGRFeatureDefn     *poFeatureDefn;

    unsigned short      iNextKMLId;
    unsigned short      nNextFID;
    int                 nTotalKMLCount;

    int                 bWriter;

    OGRKMLDataSource    *poDS;

    unsigned short      nLayerNumber;

  public:
                        OGRKMLLayer( const char * pszName, 
                                     OGRSpatialReference *poSRS, 
                                     int bWriter,
                                     OGRwkbGeometryType eType,
                                     OGRKMLDataSource *poDS );

                        ~OGRKMLLayer();

    void                ResetReading();
    OGRFeature *        GetNextFeature();

    int                 GetFeatureCount( int bForce = TRUE );
    OGRErr              GetExtent(OGREnvelope *psExtent, int bForce = TRUE);

    OGRErr              CreateFeature( OGRFeature *poFeature );

    OGRFeatureDefn *    GetLayerDefn() { return poFeatureDefn; }

    virtual OGRErr      CreateField( OGRFieldDefn *poField,
                                     int bApproxOK = TRUE );

    virtual OGRSpatialReference *GetSpatialRef();
    
    void                SetLayerNumber(unsigned short);
    
    int                 TestCapability( const char * );
};

/************************************************************************/
/*                           OGRKMLDataSource                           */
/************************************************************************/

class OGRKMLDataSource : public OGRDataSource
{
    OGRKMLLayer         **papoLayers;
    int                 nLayers;
    
    char                *pszName;
    
    //The name of the field to use for 
    char                *pszNameField;
    
    char               **papszCreateOptions;

    // output related parameters 
    FILE                *fpOutput;
    OGREnvelope         sBoundingRect;
    int                 nBoundedByLocation;
    
    int                 nSchemaInsertLocation;

    KML	                *poKMLFile;

    void                InsertHeader();

    
  public:
                        OGRKMLDataSource();
                        ~OGRKMLDataSource();

    int                 Open( const char *, int bTestOpen );
    int                 Create( const char *pszFile, char **papszOptions );

    const char          *GetName() { return pszName; }
    const char          *GetNameField() { return pszNameField; }
    int                 GetLayerCount() { return nLayers; }
    OGRLayer            *GetLayer( int );

    virtual OGRLayer    *CreateLayer( const char *, 
                                      OGRSpatialReference * = NULL,
                                      OGRwkbGeometryType = wkbUnknown,
                                      char ** = NULL );

    int                 TestCapability( const char * );

    FILE                *GetOutputFP() { return fpOutput; }

    void                GrowExtents( OGREnvelope *psGeomBounds );
    
    KML*                GetKMLFile() { return poKMLFile; };
    
};

/************************************************************************/
/*                             OGRKMLDriver                             */
/************************************************************************/

class OGRKMLDriver : public OGRSFDriver
{
  public:
                ~OGRKMLDriver();
                
    const char *GetName();
    OGRDataSource *Open( const char *, int );

    virtual OGRDataSource *CreateDataSource( const char *pszName,
                                             char ** = NULL );
    
    int                 TestCapability( const char * );
};

#endif /* OGR_KML_H_INCLUDED */

