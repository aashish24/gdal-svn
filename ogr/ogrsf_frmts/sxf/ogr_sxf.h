/******************************************************************************
 * $Id: ogr_sxf.h  $
 *
 * Project:  SXF Translator
 * Purpose:  Include file defining classes for OGR SXF driver, datasource and layers.
 * Author:   Ben Ahmed Daho Ali, bidandou(at)yahoo(dot)fr
 *
 ******************************************************************************
 * Copyright (c) 2011, Ben Ahmed Daho Ali
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

#ifndef _OGR_SXF_H_INCLUDED
#define _OGR_SXF_H_INCLUDED

#include <set>
#include <vector>

#include "ogrsf_frmts.h"
#include "org_sxf_defs.h"

/**
 * List of SXF file format geometry types.  
 */

typedef enum org_sxf_geometry_type
{
    sxfLine    = 0,      /* MultiLineString geometric object                  */
    sxfPolygon = 1,      /* Polygon geometric object                          */
    sxfPoint   = 2,      /* MultiPoint geometric object                       */
    sxfText    = 3,      /* LineString geometric object with associated label */
	sxfVector  = 4,      /* Vector geometric object with associated label */
	sxfTextTemplate    = 5,      /*  */
} OGRsxfGeometryType;

/**
 * Attribute header
 */
struct AttributeSXFOBJ
{
	GUInt16 attributeCode;
	AttributeTypeSXFOBJ	attributeType;
	GByte	attributeScale;

	bool operator<(const AttributeSXFOBJ &attr) const
	{
		if(attributeCode < attr.attributeCode)
			return true;
		else
			return false;
	}
} ;
typedef std::set<AttributeSXFOBJ> SetOfAttributesSXFOBJ;

/************************************************************************/
/*                         OGRSXFLayer                                */
/************************************************************************/
class OGRSXFLayer : public OGRLayer
{
protected:
	//OGRFeatureDefn*    poFeatureDefn;
	std::auto_ptr<OGRFeatureDefn> poFeatureDefn;
    std::auto_ptr<OGRSpatialReference> poSRS;

	VSILFILE*          fpSXF;
	int                bEOF;

	int                nNextFID;
	int                bIncorrectFType;
	int                bIncorrectFClassificator;

	const GInt32 objectsClassificator;
	const vsi_l_offset firstObjectOffset;
	vsi_l_offset lastObjectOffset;
	
	RecordSXFPSP  oSXFPSP;
    RecordSXFDSC  oSXFDSC;

	virtual OGRFeature *       GetNextRawFeature();

	GUInt32 TranslateXYH ( RecordSXFOBJ  oSXFObj,     char *psBuff, 
                          double *dfX, double *dfY, double *dfH = NULL);


	OGRFeature *TranslatePoint( RecordSXFOBJ  oSXFObj, char * psRecordBuf );
	OGRFeature *TranslateText ( RecordSXFOBJ  oSXFObj, char * psBuff );
	OGRFeature *TranslatePolygon ( RecordSXFOBJ  oSXFObj, char * psBuff );
	OGRFeature *TranslateLine ( RecordSXFOBJ  oSXFObj, char * psBuff );
public:
	OGRSXFLayer(VSILFILE* fp, const char* pszLayerName, OGRSpatialReference *sr, std::auto_ptr<OGRFeatureDefn> featureDefn,  RecordSXFPSP&  sxfP, RecordSXFDSC&  sxfD, GInt32 objCl, vsi_l_offset foo);
	~OGRSXFLayer();

	virtual void                ResetReading();
    virtual OGRFeature *        GetNextFeature();

	virtual OGRFeatureDefn *    GetLayerDefn() { return poFeatureDefn.get();}

    virtual int                 TestCapability( const char * );

    virtual OGRSpatialReference *GetSpatialRef() { return poSRS.get(); }
};


/************************************************************************/
/*                        OGRSXFDataSource                       */
/************************************************************************/

class OGRSXFDataSource : public OGRDataSource
{
    char*               pszName;

    OGRLayer**          papoLayers;
    int                 nLayers;

    VSILFILE* fpSXF;

  public:
                        OGRSXFDataSource();
                        ~OGRSXFDataSource();

    int                 Open( const char * pszFilename,
                              int bUpdate );

    virtual const char*     GetName() { return pszName; }

    virtual int             GetLayerCount() { return nLayers; }
    virtual OGRLayer*       GetLayer( int );

    virtual int             TestCapability( const char * );
    void                    CloseFile(); 
};

/************************************************************************/
/*                         OGRSXFDriver                          */
/************************************************************************/

class OGRSXFDriver : public OGRSFDriver
{
  public:
                ~OGRSXFDriver();

    virtual const char*         GetName();
    virtual OGRDataSource*      Open( const char *, int );
    virtual int                 TestCapability( const char * );
};



#endif 
