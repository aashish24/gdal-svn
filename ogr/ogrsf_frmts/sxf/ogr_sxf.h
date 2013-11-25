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

#ifdef WIN32
#else
    #include <tr1/memory>
#endif

#include "ogrsf_frmts.h"
#include "org_sxf_defs.h"

/*
 *  Record headr size in sxf v.4 and sxf v.3
 */
const static size_t recordHeaderSize = 32;

bool readSXFRecord(VSILFILE* fpSXF, const SXFPassport& passport, SXFRecordInfo& recordInfo);

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

    std::tr1::shared_ptr<SXFPassport> poSXFPassport;

	virtual OGRFeature *       GetNextRawFeature();

    GUInt32 TranslateXYH ( const SXFRecordCertifInfo& certifInfo,     char *psBuff,
                          double *dfX, double *dfY, double *dfH = NULL);


    OGRFeature *TranslatePoint( const SXFRecordCertifInfo& certifInfo, char * psRecordBuf );
    OGRFeature *TranslateText ( const SXFRecordCertifInfo& certifInfo, char * psBuff );
    OGRFeature *TranslatePolygon ( const SXFRecordCertifInfo& certifInfo, char * psBuff );
    OGRFeature *TranslateLine ( const SXFRecordCertifInfo& certifInfo, char * psBuff );
public:
    OGRSXFLayer(VSILFILE* fp, const char* pszLayerName, OGRSpatialReference *sr, SXFPassport&  sxfPassport, std::set<GInt32> objCls, RSCLayer*  rscLayer);
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

    void CreateLayers(SXFPassport& sxfPassport, OGRSpatialReference *poSRS);
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
