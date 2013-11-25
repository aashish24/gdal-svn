/******************************************************************************
 * $Id: ogr_sxflayer.cpp  $
 *
 * Project:  SXF Translator
 * Purpose:  Definition of classes for OGR SXF Layers.
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
#define  _USE_MATH_DEFINES

#include "ogr_sxf.h"
#include "cpl_conv.h"
#include "cpl_string.h"
#include "ogr_p.h"
#include "ogr_srs_api.h"

CPL_CVSID("$Id: ogrsxflayer.cpp $");

/************************************************************************/
/*                        OGRSXFLayer()                                 */
/************************************************************************/

OGRSXFLayer::OGRSXFLayer(VSILFILE* fp, const char* pszLayerName, OGRSpatialReference *sr, SXFPassport&  sxfPassport, std::set<GInt32> objCls, RSCLayer*  rscLayer)
    : poSRS(new OGRSpatialReference(*sr)),
      oRSCLayer(rscLayer),
      objectsClassificators(objCls),
      poSXFPassport(new SXFPassport(sxfPassport))
{
    fpSXF = fp;
    nNextFID = 0;
    bIncorrectFType = FALSE;
    bIncorrectFClassificator = FALSE;
    bEOF = FALSE;

    poFeatureDefn =new OGRFeatureDefn(pszLayerName);
    poFeatureDefn->Reference();

    OGRFieldDefn oClCodeField = OGRFieldDefn( "CLCODE", OFTInteger );
    oClCodeField.SetWidth(10);
    poFeatureDefn->AddFieldDefn( &oClCodeField );

    OGRFieldDefn oClNameField = OGRFieldDefn( "CLNAME", OFTString );
    oClNameField.SetWidth(32);
    poFeatureDefn->AddFieldDefn( &oClNameField );

    OGRFieldDefn oNumField = OGRFieldDefn( "OBJECTNUMB", OFTInteger );
    oNumField.SetWidth(10);
    poFeatureDefn->AddFieldDefn( &oNumField );

    OGRFieldDefn  oTextField( "TEXT", OFTString );
    oTextField.SetWidth(255);
    poFeatureDefn->AddFieldDefn( &oTextField );


    if(poSXFPassport->version == IDSXFVERSION4)
        firstObjectOffset = 452;
    else if(poSXFPassport->version == IDSXFVERSION3)
        firstObjectOffset = 300;
    lastObjectOffset = firstObjectOffset;
    VSIFSeekL( fpSXF, firstObjectOffset,  SEEK_SET);

    std::set<GUInt16> unicClassifiers;

    SXFRecordInfo recordInfo;
    while(readSXFRecord(fpSXF, sxfPassport, recordInfo))
    {
        VSIFSeekL(fpSXF, recordInfo.nCertifLength, SEEK_CUR);

        size_t nDataSize = recordInfo.nRecordLength - 32 - recordInfo.nCertifLength;

        char * psRecordBuf = (char *) CPLMalloc( nDataSize );
        VSIFReadL( psRecordBuf, nDataSize , 1, fpSXF );


        if(recordInfo.bHazSemantics == 0)
            continue;

        if(objectsClassificators.find(recordInfo.iCC) == objectsClassificators.end())
            continue;

        size_t attributeOffset = 0;

        while(attributeOffset < nDataSize)
        {
            GUInt16 code = *(GUInt16 *)(psRecordBuf + attributeOffset);
            attributeOffset += 2;

            GByte type = *(GByte *)(psRecordBuf + attributeOffset);
            attributeOffset += 1;

            char scale = *(char *)(psRecordBuf + attributeOffset);
            attributeOffset += 1;

            CPLString oFieldName;
            oFieldName.Printf("SC_%d", code);

            switch(type)
            {
                case sctASCIIZ_DOS:
                {
                    attributeOffset += scale + 1;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(scale + 1);

                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctOneByte:
                {
                    attributeOffset += 1;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctTwoByte:
                {
                    attributeOffset += 2;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctForeByte:
                {
                    attributeOffset += 4;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctEightByte:
                {
                    attributeOffset += 8;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctANSI_Windows:
                {
                    attributeOffset += scale + 1;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(scale + 1);
                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctUNICODE_UNIX:
                {
                    attributeOffset += scale + 1;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(scale + 1);
                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctBigString:
                {
                    GUInt32 scale2 = *(GUInt32 *)(psRecordBuf+attributeOffset);
                    attributeOffset += scale2;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(attributeOffset);
                    if(unicClassifiers.find(code) == unicClassifiers.end())
                        poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
            }

            unicClassifiers.insert(code);
        }
        /**/
    }
}

/************************************************************************/
/*                         ~OGRSXFLayer()                               */
/************************************************************************/

OGRSXFLayer::~OGRSXFLayer()
{
    poSRS->Release();
    poFeatureDefn->Release();
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRSXFLayer::ResetReading()

{
    nNextFID = 0;
    bIncorrectFType = FALSE;
	bIncorrectFClassificator = FALSE;
    bEOF = FALSE;
	VSIFSeekL( fpSXF, firstObjectOffset,  SEEK_SET);
	lastObjectOffset = firstObjectOffset;
}


/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRSXFLayer::GetNextFeature()
{
    OGRFeature  *poFeature;

    while(TRUE)
    {
        if (bEOF)
            return NULL;

		VSIFSeekL( fpSXF, lastObjectOffset,  SEEK_SET);
        poFeature = GetNextRawFeature();
        lastObjectOffset = VSIFTellL( fpSXF );

        if ( bIncorrectFType == TRUE) 
        {
            bIncorrectFType = FALSE ;
            continue;
        }
		if ( bIncorrectFClassificator == TRUE) 
        {         
            bIncorrectFClassificator = FALSE ;
            continue;
        }
        if (poFeature == NULL)
		{
			return NULL;
		}

        if((m_poFilterGeom == NULL
            || FilterGeometry( poFeature->GetGeometryRef() ) )
        && (m_poAttrQuery == NULL
            || m_poAttrQuery->Evaluate( poFeature )) )
        {
            return poFeature;
        }
        else
            delete poFeature;
    }    
  
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRSXFLayer::TestCapability( const char * pszCap )

{
    if (EQUAL(pszCap, OLCStringsAsUTF8))
        return TRUE;
        
    return FALSE;
}

/************************************************************************/
/*                                TranslateXYH()                        */
/************************************************************************/
/****
 * TODO : Take into account informations given in the passport 
 * like unit of mesurement, type and dimensions (integer, float, double) of coordinate,
 * the vector format ....
 */

GUInt32 OGRSXFLayer::TranslateXYH ( const SXFRecordCertifInfo& certifInfo, char *psBuff,
                          double *dfX, double *dfY, double *dfH)
{
	int offset = 0;

    if (certifInfo.bElemSize == 0)
	{
        if (certifInfo.bElemType == 0)
		{
			short y = *(short *)(psBuff);
			short x = *(short *)(psBuff + 2);

			*dfX = (double)x;
			*dfY = (double)y;

			offset += 4;

			if (dfH != NULL)
			{
                float h = *(float *)(psBuff + 8); // H always in float
				*dfH = (double)h; 

				offset += 4;
			}
		}
		else
		{
			float y = *(float *)(psBuff);
			float x = *(float *)(psBuff + 4);

			*dfX = (double)x;
			*dfY = (double)y;

			offset += 8;

			if (dfH != NULL)
			{
				float h = *(float *)(psBuff + 8); 
				*dfH = (double)h; 

				offset += 4;
			}
		}
	
	}
	else
	{
        if (certifInfo.bElemType == 0)
		{
			int y = *(int *)(psBuff);
			int x = *(int *)(psBuff + 4);

			*dfX = (double)x;
			*dfY = (double)y;

			offset += 8;

			if (dfH != NULL)
			{
				double h = *(double *)(psBuff + 8); // H always in float
				*dfH = (double)h; 

				offset += 4;
			}
		}
		else
		{
			*dfX = *(double *)(psBuff);
			*dfY = *(double *)(psBuff + 8);

			offset += 16;
			
			if (dfH != NULL)
			{
				*dfH = *(double *)(psBuff + 16); 
				offset += 8;
			}
		}
	}

    if ( poSXFPassport->informationFlags.realCoordinatesCompliance == false)
    {
        double scale = (double)poSXFPassport->nScale / (double)poSXFPassport->deviceInfo.iDeviceCapability;

        *dfX *= scale;
        *dfX -= scale * (double)poSXFPassport->deviceInfo.deviceFrameCoordinates.dfYsw;
        *dfX += poSXFPassport->sheetRectCoordinates.dfYsw;

        *dfY *= scale;
        *dfY -= scale * (double)poSXFPassport->deviceInfo.deviceFrameCoordinates.dfXsw;
        *dfY += poSXFPassport->sheetRectCoordinates.dfXsw;
    }

	return offset;
}

/************************************************************************/
/*                         GetNextRawFeature()                          */
/************************************************************************/

OGRFeature *OGRSXFLayer::GetNextRawFeature()
{


    /*
    GByte* buf = (GByte*)CPLMalloc(recordHeaderSize);
    if ( VSIFReadL( buf, recordHeaderSize, 1, fpSXF ) != 1)
    {
        bEOF = TRUE;
        return NULL;
    }
    */

    SXFRecordInfo recordInfo;
    if(readSXFRecord(fpSXF, *poSXFPassport.get(), recordInfo) == false)
    {
        bEOF = TRUE;
        return NULL;
    }


    if (recordInfo.bHazTyingVect == true)
        CPLError( CE_Failure, CPLE_NotSupported,
                  "SXF. Parsing the vector of the tying not support." );

    std::set<GInt32>::iterator classifierIt = objectsClassificators.find(recordInfo.iCC);
    if(classifierIt == objectsClassificators.end())
	{
		bIncorrectFClassificator = TRUE;
        VSIFSeekL( fpSXF, recordInfo.nRecordLength - recordHeaderSize, SEEK_CUR );
		return NULL;
	}

    OGRFeature *poFeature = NULL;
    char * recordCertifBuf = (char *) CPLMalloc( recordInfo.nCertifLength );
    VSIFReadL( recordCertifBuf, recordInfo.nCertifLength , 1, fpSXF );

    if (recordInfo.bGeomType == sxfPoint)
        poFeature = TranslatePoint( recordInfo.certifInfo, recordCertifBuf );
    else if (recordInfo.bGeomType == sxfLine)
        poFeature = TranslateLine ( recordInfo.certifInfo, recordCertifBuf );
    else if (recordInfo.bGeomType == sxfPolygon )
        poFeature = TranslatePolygon( recordInfo.certifInfo, recordCertifBuf );
    else if (recordInfo.bGeomType == sxfText )
        poFeature = TranslateText ( recordInfo.certifInfo, recordCertifBuf );
  /*else if (recordInfo.bGeomType == sxfVector ) // TODO realise this
    {
        CPLError( CE_Warning, CPLE_NotSupported,
                  "SXF. Geometry type Vector do not support." );
    }
    else if (recordInfo.bGeomType == sxfTextTemplate ) // TODO realise this
    {
        CPLError( CE_Warning, CPLE_NotSupported,
                  "SXF. Geometry type Text Template do not support." );
    }*/
    else
    {
        bIncorrectFType = TRUE ;
        return NULL;
    }

    poFeature->SetField("CLCODE", recordInfo.iCC);

    if (oRSCLayer != NULL)
        poFeature->SetField("CLNAME", oRSCLayer->rscObjects[*classifierIt].c_str());
    else
        poFeature->SetField("CLNAME", recordInfo.iCC);

    poFeature->SetField("OBJECTNUMB", recordInfo.nObjNumb);

    if(recordInfo.bHazSemantics == true)
    {
        size_t  nSemanticsSize = recordInfo.nRecordLength - recordHeaderSize - recordInfo.nCertifLength;
        char * psSemanticsdBuf = (char *) CPLMalloc( nSemanticsSize );
        VSIFReadL( psSemanticsdBuf, nSemanticsSize , 1, fpSXF );

        size_t offset = 0;
        while (offset < nSemanticsSize)
        {
            GUInt16 characterCode = *(GUInt16 *)(psSemanticsdBuf+offset);
            offset += 2;

            GByte characterType = *(GByte *)(psSemanticsdBuf+offset);
            offset += 1;

            char scale = *(char *)(psSemanticsdBuf+offset);
            offset += 1;

            CPLString oFieldName;
            oFieldName.Printf("SC_%d",characterCode);
            CPLString oFieldValue;

            switch(characterType)
            {
                case sctASCIIZ_DOS:
                {
                    char * value = (char*) CPLMalloc( scale + 1 );
                    memcpy(value, psSemanticsdBuf+offset,scale + 1);
                    poFeature->SetField(oFieldName, value );

                    offset += scale + 1;
                    break;
                }
                case sctOneByte:
                {
                    double d = *(GByte *)(psSemanticsdBuf+offset);
                    d *= std::pow(10.0, (double)scale);

                    oFieldValue.Printf("%f",d);
                    poFeature->SetField(oFieldName, d);

                    offset += 1;
                    break;
                }
                case sctTwoByte:
                {
                    double d = *(GInt16 *)(psSemanticsdBuf+offset);
                    d *= std::pow(10.0, (double)scale);

                    oFieldValue.Printf("%f",d);
                    poFeature->SetField(oFieldName, d);

                    offset += 2;
                    break;
                }
                case sctForeByte:
                {
                    double d = *(GInt32 *)(psSemanticsdBuf+offset);
                    d *= std::pow(10.0, (double)scale);

                    oFieldValue.Printf("%f",d);
                    poFeature->SetField(oFieldName, d);

                    offset += 4;
                    break;
                }
                case sctEightByte:
                {
                    double d = *(double *)(psSemanticsdBuf+offset);
                    oFieldValue.Printf("%f",d);
                    poFeature->SetField(oFieldName, d);

                    offset += 8;
                    break;
                }
                case sctANSI_Windows:
                {
                    char * value = (char*) CPLMalloc( scale + 1 );
                    memcpy(value, psSemanticsdBuf+offset,scale + 1);
                    poFeature->SetField(oFieldName, CPLRecode(value, "CP1251", CPL_ENC_UTF8));

                    offset += scale + 1;
                    break;
                }
                case sctUNICODE_UNIX:
                {
                    char * value = (char*) CPLMalloc( scale + 1 );
                    memcpy(value, psSemanticsdBuf+offset,scale + 1);
                    poFeature->SetField(oFieldName, value);

                    offset += scale + 1;
                    break;
                }
                case sctBigString:
                {
                    GUInt32 scale2 = *(GUInt32 *)(psSemanticsdBuf+offset);
                    char * value = (char*) CPLMalloc( scale2 + 1 );
                    memcpy(value, psSemanticsdBuf+offset,scale2 + 1);
                    poFeature->SetField(oFieldName, CPLRecode(value, CPL_ENC_UTF16, CPL_ENC_UTF8));

                    offset += scale2;
                    break;
                }
            }
        }

        CPLFree( psSemanticsdBuf );
    }

    poFeature->SetFID( nNextFID++ );

    CPLFree( recordCertifBuf );
	
    return poFeature;
}

/************************************************************************/
/*                         TranslatePoint   ()                          */
/************************************************************************/
OGRFeature *OGRSXFLayer::TranslatePoint( const SXFRecordCertifInfo& certifInfo, char * psRecordBuf )
{
        double dfX = 1.0;
        double dfY = 1.0;
        GUInt32 nOffset = 0;

		OGRFeatureDefn *fd = poFeatureDefn->Clone();
		fd->SetGeomType( wkbMultiPoint );
        OGRFeature *poFeature = new OGRFeature(fd);
        OGRMultiPoint* poMPt = new OGRMultiPoint();

        if (certifInfo.bDim == 1) // TODO realise this
		{
			 CPLError( CE_Fatal, CPLE_NotSupported, 
					  "SXF. 3D metrics do not support." );
		}

        nOffset += TranslateXYH( certifInfo, psRecordBuf , &dfX, &dfY ) ;

        poMPt->addGeometryDirectly( new OGRPoint( dfX, dfY ) );

/*---------------------- Reading SubObjects --------------------------------*/

    for(int count=0 ; count <  certifInfo.nSubObjCount ; count++)
    {
        char * psBuff = psRecordBuf + nOffset;

        GUInt16 nSubObj = *(GUInt16*) psBuff;
        GUInt16 nCoords = *(GUInt16*) (psBuff + 2);


        nOffset +=4;

        for (int i=0; i < nCoords ; i++)
        {
            char * psCoords = psRecordBuf + nOffset ;

            nOffset +=  TranslateXYH( certifInfo, psCoords , &dfX, &dfY ) ;

            poMPt->addGeometryDirectly( new OGRPoint( dfX, dfY ) );
        } 
    }

/*****
 * TODO : 
 *          - Translate graphics 
 *          - Translate 3D vector
 */

        poFeature->SetGeometryDirectly( poMPt );

        return poFeature;
}

/************************************************************************/
/*                         TranslateLine    ()                          */
/************************************************************************/

OGRFeature *OGRSXFLayer::TranslateLine( const SXFRecordCertifInfo& certifInfo, char * psRecordBuf )
{
        double dfX = 1.0;
        double dfY = 1.0;

        GUInt32 nOffset = 0;

		OGRFeatureDefn *fd = poFeatureDefn->Clone();
		fd->SetGeomType( wkbMultiLineString );
        OGRFeature *poFeature = new OGRFeature(fd);
        OGRMultiLineString *poMLS = new  OGRMultiLineString ();

/*---------------------- Reading Primary Line --------------------------------*/

    OGRLineString* poLS = new OGRLineString();

    if (certifInfo.bDim == 1) // TODO realise this
	{
		 CPLError( CE_Fatal, CPLE_NotSupported, 
                  "SXF. 3D metrics do not support." );
	}

    for(int count=0 ; count <  certifInfo.nPointsCount ; count++)
    {
        char * psCoords = psRecordBuf + nOffset ;
        nOffset += TranslateXYH( certifInfo, psCoords , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );
    }

    poMLS->addGeometry( poLS );

/*---------------------- Reading Sub Lines --------------------------------*/

    for(int count=0 ; count <  certifInfo.nSubObjCount ; count++)
    {
    poLS->empty();
    char * psBuff = psRecordBuf + nOffset;

    GUInt16 nSubObj = *(GUInt16*) psBuff; 
    GUInt16 nCoords = *(GUInt16*) (psBuff + 2); 

    nOffset +=4;

        for (int i=0; i < nCoords ; i++)
        {
        char * psCoords = psRecordBuf + nOffset ;

         nOffset +=  TranslateXYH( certifInfo, psCoords , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );
        }

    poMLS->addGeometry( poLS );
    }    // for

        delete poLS;
        poFeature->SetGeometryDirectly( poMLS );

/*****
 * TODO : 
 *          - Translate graphics 
 *          - Translate 3D vector
 */

        return poFeature;
}

/************************************************************************/
/*                         TranslatePolyg   ()                          */
/************************************************************************/
OGRFeature *OGRSXFLayer::TranslatePolygon( const SXFRecordCertifInfo& certifInfo, char * psRecordBuf )
{
    double dfX = 1.0;
    double dfY = 1.0;
    GUInt32 nOffset = 0;

		OGRFeatureDefn *fd = poFeatureDefn->Clone();
		fd->SetGeomType( wkbMultiPolygon );
        OGRFeature *poFeature = new OGRFeature(fd);
        OGRPolygon *poPoly = new OGRPolygon();
        OGRLineString* poLS = new OGRLineString();

/*---------------------- Reading Primary Polygon --------------------------------*/

        if (certifInfo.bDim == 1) // TODO realise this
		{
			 CPLError( CE_Fatal, CPLE_NotSupported, 
					  "SXF. 3D metrics do not support." );
		}

    for(int count=0 ; count <  certifInfo.nPointsCount ; count++)
    {
        char * psBuf = psRecordBuf + nOffset ;

        nOffset +=  TranslateXYH( certifInfo, psBuf , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );

    }    // for

        OGRLinearRing *poLR = new OGRLinearRing();
        poLR->addSubLineString( poLS, 0 );

        poPoly->addRingDirectly( poLR );

/*---------------------- Reading Sub Lines --------------------------------*/

    for(int count=0 ; count <  certifInfo.nSubObjCount ; count++)
    {
    poLS->empty();
    char * psBuff = psRecordBuf + nOffset;

    GUInt16 nSubObj = *(GUInt16*) psBuff; 
    GUInt16 nCoords = *(GUInt16*) (psBuff + 2); 

    nOffset +=4;

        for (int i=0; i < nCoords ; i++)
        {
        char * psCoords = psRecordBuf + nOffset ;

         nOffset +=  TranslateXYH( certifInfo, psCoords , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );
        }

        OGRLinearRing *poLR = new OGRLinearRing();
        poLR->addSubLineString( poLS, 0 );

        poPoly->addRingDirectly( poLR );
    }    // for

        poFeature->SetGeometryDirectly( poPoly );   //poLS);
        delete poLS;

/*****
 * TODO : 
 *          - Translate graphics 
 *          - Translate 3D vector
 */
        return poFeature;
}

/************************************************************************/
/*                         TranslateText    ()                          */
/************************************************************************/
OGRFeature *OGRSXFLayer::TranslateText( const SXFRecordCertifInfo& certifInfo, char * psRecordBuf )
{
    double dfX = 1.0;
    double dfY = 1.0;
    GUInt32 nOffset = 0;

		OGRFeatureDefn *fd = poFeatureDefn->Clone();
		fd->SetGeomType( wkbLineString );
        OGRFeature *poFeature = new OGRFeature(fd);
        OGRLineString* poLS = new OGRLineString();

        if (certifInfo.bDim == 1) // TODO realise this
		{
			 CPLError( CE_Fatal, CPLE_NotSupported, 
					  "SXF. 3D metrics do not support." );
		}

    for(int count=0 ; count <  certifInfo.nPointsCount ; count++)
    {
        char * psBuf = psRecordBuf + nOffset;

        nOffset += TranslateXYH( certifInfo, psBuf , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );
    }

    poFeature->SetGeometryDirectly( poLS );

/*------------------     READING TEXT VALUE   ---------------------------------------*/

    if ( certifInfo.nSubObjCount == 0 && certifInfo.bHasTextSign == true)
    {

        char * pszTxt = psRecordBuf + nOffset;
        GByte nTextL = (GByte) *pszTxt;

        char * pszTextBuf = (char *)CPLMalloc( nTextL+1 );

        strncpy(pszTextBuf, (pszTxt+1),    nTextL+1);

        //TODO: Check encoding from sxf
        poFeature->SetField("TEXT", pszTextBuf);
 
        CPLFree( pszTextBuf );
    }


/*****
 * TODO : 
 *          - Translate graphics 
 *          - Translate 3D vector
 */

    return poFeature;
}



