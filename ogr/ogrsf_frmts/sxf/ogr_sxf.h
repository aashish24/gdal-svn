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
#include <map>

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
 * Semantics attribute header
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
};

typedef std::set<AttributeSXFOBJ> SetOfAttributesSXFOBJ;

typedef std::map<GUInt32, std::string> RSCObjects;

struct RSCLayer
{
    GByte szLayerId;
    std::string szLayerName;
    RSCObjects rscObjects;
};

typedef std::map<GByte, RSCLayer> RSCLayers;


/************************************************************************/
/*                         OGRSXFLayer                                */
/************************************************************************/
class OGRSXFLayer : public OGRLayer
{
protected:
    OGRFeatureDefn*    poFeatureDefn;
    OGRSpatialReference* poSRS;

	VSILFILE*          fpSXF;
	int                bEOF;

	int                nNextFID;
	int                bIncorrectFType;
	int                bIncorrectFClassificator;

    std::set<GInt32> objectsClassificators;
    vsi_l_offset firstObjectOffset;
	vsi_l_offset lastObjectOffset;
	
    RSCLayer*  oRSCLayer;

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
    OGRSXFLayer(VSILFILE* fp, const char* pszLayerName, OGRSpatialReference *sr, RecordSXFPSP&  sxfP, RecordSXFDSC&  sxfD, std::set<GInt32> objCls, RSCLayer*  rscLayer);
    ~OGRSXFLayer();

	virtual void                ResetReading();
    virtual OGRFeature *        GetNextFeature();

    virtual OGRFeatureDefn *    GetLayerDefn() { return poFeatureDefn;}

    virtual int                 TestCapability( const char * );

    virtual OGRSpatialReference *GetSpatialRef() { return poSRS; }
};


/************************************************************************/
/*                        OGRSXFDataSource                       */
/************************************************************************/

class OGRSXFDataSource : public OGRDataSource
{
    CPLString               pszName;

    OGRLayer**          papoLayers;
    size_t              nLayers;

    VSILFILE* fpSXF;
    VSILFILE* fpRSC;

    RSCLayers   rscLayers;

    void CreateLayers(RecordSXFPSP  &oSXFPSP, RecordSXFDSC &oSXFDSC, OGRSpatialReference *poSRS);
    void ReadRSCLayers(RecordRSCHEAD &RSCFileHeader);
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

    const char*     GetName();
    OGRDataSource*  Open( const char *, int );
    OGRErr          DeleteDataSource(const char* pszName);
    int             TestCapability(const char *);
};

#endif 
