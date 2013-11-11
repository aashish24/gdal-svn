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

OGRSXFLayer::OGRSXFLayer(VSILFILE* fp, const char* pszLayerName, OGRSpatialReference *sr, RecordSXFPSP&  oSXFP, RecordSXFDSC&  oSXFD, std::set<GInt32> objCls, RSCLayer*  rscLayer)
    : poSRS(new OGRSpatialReference(*sr)),
      oRSCLayer(rscLayer),
      objectsClassificators(objCls)
{
    poFeatureDefn =new OGRFeatureDefn(pszLayerName);

    fpSXF = fp;
    oSXFPSP = oSXFP;
    oSXFDSC = oSXFD;
    nNextFID = 0;
    bIncorrectFType = FALSE;
    bIncorrectFClassificator = FALSE;
    bEOF = FALSE;

    firstObjectOffset = sizeof(RecordSXFHEAD) + sizeof(RecordSXFPSP) + sizeof(RecordSXFDSC);
    lastObjectOffset = firstObjectOffset;

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


    VSIFSeekL( fpSXF, firstObjectOffset,  SEEK_SET);

    std::set<GUInt16> unicClassifiers;
    RecordSXFOBJ oSXFObj;
    while(VSIFReadL( &oSXFObj, sizeof(RecordSXFOBJ), 1, fpSXF ))
    {

        VSIFSeekL(fpSXF, oSXFObj.nCertifLength, SEEK_CUR);

        size_t nDataSize = oSXFObj.nRecordLength - sizeof(RecordSXFOBJ) - oSXFObj.nCertifLength;
        char * psRecordBuf = (char *) CPLMalloc( nDataSize );
        VSIFReadL( psRecordBuf, nDataSize , 1, fpSXF );

        if(oSXFObj.bHazSemantics == 0)
            continue;

        if(objectsClassificators.find(oSXFObj.iCC) == objectsClassificators.end())
            continue;

        size_t attributeOffset = 0;

        while(attributeOffset < nDataSize)
        {
            GUInt16 code = *(GUInt16 *)(psRecordBuf + attributeOffset);
            attributeOffset += 2;

            if(unicClassifiers.find(code) != unicClassifiers.end())
            {
                break;
            }

            GByte type = *(GByte *)(psRecordBuf + attributeOffset);
            attributeOffset += 1;

            char scale = *(char *)(psRecordBuf + attributeOffset);
            attributeOffset += 1;

            unicClassifiers.insert(code);

            CPLString oFieldName;
            oFieldName.Printf("SEMCODE_%d", code);

            switch(type)
            {
                case sctASCIIZ_DOS:
                {
                    attributeOffset += scale + 1;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(scale + 1);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctOneByte:
                {
                    attributeOffset += 1;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctTwoByte:
                {
                    attributeOffset += 2;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctForeByte:
                {
                    attributeOffset += 4;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctEightByte:
                {
                    attributeOffset += 8;

                    OGRFieldDefn  oField( oFieldName, OFTReal );
                    oField.SetWidth(10);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctANSI_Windows:
                {
                    attributeOffset += scale + 1;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(scale + 1);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctUNICODE_UNIX:
                {
                    attributeOffset += scale + 1;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(scale + 1);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
                case sctBigString:
                {
                    GUInt32 scale2 = *(GUInt32 *)(psRecordBuf+attributeOffset);
                    attributeOffset += scale2;

                    OGRFieldDefn  oField( oFieldName, OFTString );
                    oField.SetWidth(attributeOffset);
                    poFeatureDefn->AddFieldDefn( &oField );

                    break;
                }
            }

        }
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

GUInt32 OGRSXFLayer::TranslateXYH ( RecordSXFOBJ  oSXFObj,     char *psBuff, 
                          double *dfX, double *dfY, double *dfH)
{
	int offset = 0;

	//TODO to improve
	if (oSXFObj.bCertifSize == 0)
	{
		if (oSXFObj.bElemType == 0)
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
		if (oSXFObj.bElemType == 0)
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

    if ( oSXFPSP.iPlanUoM == 64) // radians  - to enhance
    {
        *dfX  *= (180.0/M_PI);   
        *dfY  *= (180.0/M_PI);     
    }

	return offset;
}

/************************************************************************/
/*                         GetNextRawFeature()                          */
/************************************************************************/

OGRFeature *OGRSXFLayer::GetNextRawFeature()
{
    OGRFeature *poFeature = NULL;
    RecordSXFOBJ  oSXFObj;

    if ( VSIFReadL( &oSXFObj, sizeof(RecordSXFOBJ), 1, fpSXF ) < 1 
        || nNextFID == oSXFDSC.nObjCount )
    {
        bEOF = TRUE;
        return NULL;
    }

    std::set<GInt32>::iterator classifierIt = objectsClassificators.find(oSXFObj.iCC);
    if(classifierIt == objectsClassificators.end())
	{
		bIncorrectFClassificator = TRUE;
		VSIFSeekL( fpSXF, oSXFObj.nRecordLength - sizeof(RecordSXFOBJ), SEEK_CUR );
		return NULL;
	}

    char * psRecordBuf = (char *) CPLMalloc( oSXFObj.nCertifLength );
    VSIFReadL( psRecordBuf, oSXFObj.nCertifLength , 1, fpSXF );

    if (oSXFObj.bGeomType == sxfPoint) 
        poFeature = TranslatePoint( oSXFObj, psRecordBuf );
	else if (oSXFObj.bGeomType == sxfLine)       
        poFeature = TranslateLine ( oSXFObj, psRecordBuf );
	else if (oSXFObj.bGeomType == sxfPolygon ) 
        poFeature = TranslatePolygon( oSXFObj, psRecordBuf );
	else if (oSXFObj.bGeomType == sxfText ) 
        poFeature = TranslateText ( oSXFObj, psRecordBuf );
    /*
	else if (oSXFObj.bGeomType == sxfVector ) // TODO realise this
        CPLError( CE_Warning, CPLE_NotSupported,
                  "SXF. Geometry type Vector do not support." ); 
	else if (oSXFObj.bGeomType == sxfTextTemplate ) // TODO realise this
        CPLError( CE_Warning, CPLE_NotSupported,
                  "SXF. Geometry type Text Template do not support." );
    */
	if (oSXFObj.bHazTyingVect == 1)
        CPLError( CE_Fatal, CPLE_NotSupported,
                  "SXF. Parsing the vector of the tying not support." );

    if (poFeature != NULL)
    {
        poFeature->SetField("CLCODE", oSXFObj.iCC);

        poFeature->SetField("CLNAME", oSXFObj.iCC);

        if (oRSCLayer != NULL)
            poFeature->SetField("CLNAME", oRSCLayer->rscObjects[*classifierIt].c_str());
        else
            poFeature->SetField("CLNAME", oSXFObj.iCC);

        poFeature->SetField("OBJECTNUMB", oSXFObj.nObjNumb);

		if(oSXFObj.bHazSemantics == 1)
		{
			size_t  nSemanticsSize = oSXFObj.nRecordLength - sizeof(RecordSXFOBJ) - oSXFObj.nCertifLength;
			char * psSemanticsdBuf = (char *) CPLMalloc( nSemanticsSize );
			VSIFReadL( psSemanticsdBuf, nSemanticsSize , 1, fpSXF );

			size_t offset = 0;
			while (offset < nSemanticsSize)
			{
				GUInt16 characterCode = *(GUInt16 *)(psSemanticsdBuf+offset);
				offset += 2;

                //printf("cc: %d  ", characterCode);

				GByte characterType = *(GByte *)(psSemanticsdBuf+offset);
				offset += 1;

                char scale = *(char *)(psSemanticsdBuf+offset);
				offset += 1;
			
				CPLString oFieldName;
                oFieldName.Printf("SEMCODE_%d",characterCode);
				CPLString oFieldValue;

				switch(characterType)
				{
					case sctASCIIZ_DOS:
					{
						char * value = (char*) CPLMalloc( scale + 1 );
						memcpy(value, psSemanticsdBuf+offset,scale + 1);
                        poFeature->SetField(oFieldName, CPLRecode( value, CPL_ENC_ASCII, "UTF-8") );

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
                        poFeature->SetField(oFieldName, CPLRecode( value, "CP1251", "UTF-8") );

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
                        poFeature->SetField(oFieldName, CPLRecode( value, "UTF-16", "UTF-8"));

						offset += scale2;
						break;
					}
				}
			}

			CPLFree( psSemanticsdBuf );
		}

		poFeature->SetFID( nNextFID++ );		
    }
    else
	{
		bIncorrectFType = TRUE ;
	}

    CPLFree( psRecordBuf );
	
    return poFeature;
}

/************************************************************************/
/*                         TranslatePoint   ()                          */
/************************************************************************/
OGRFeature *OGRSXFLayer::TranslatePoint( RecordSXFOBJ  oSXFObj, char * psRecordBuf )
{
        double dfX = 1.0;
        double dfY = 1.0;
        GUInt32 nOffset = 0;

		OGRFeatureDefn *fd = poFeatureDefn->Clone();
		fd->SetGeomType( wkbMultiPoint );
        OGRFeature *poFeature = new OGRFeature(fd);
        OGRMultiPoint* poMPt = new OGRMultiPoint();

		if (oSXFObj.bDimIdea == 1) // TODO realise this
		{
			 CPLError( CE_Fatal, CPLE_NotSupported, 
					  "SXF. 3D metrics do not support." );
		}

        nOffset += TranslateXYH( oSXFObj, psRecordBuf , &dfX, &dfY ) ;

        poMPt->addGeometryDirectly( new OGRPoint( dfX, dfY ) );

/*---------------------- Reading SubObjects --------------------------------*/

    for(int count=0 ; count <  oSXFObj.nSubObjCount ; count++)
    {
    char * psBuff = psRecordBuf + nOffset;

    GUInt16 nSubObj = *(GUInt16*) psBuff; 
    GUInt16 nCoords = *(GUInt16*) (psBuff + 2); 


    nOffset +=4;

        for (int i=0; i < nCoords ; i++)
        {
        char * psCoords = psRecordBuf + nOffset ;

        nOffset +=  TranslateXYH( oSXFObj, psCoords , &dfX, &dfY ) ;

        poMPt->addGeometryDirectly( new OGRPoint( dfX, dfY ) );
        } 
    } // for count

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

OGRFeature *OGRSXFLayer::TranslateLine( RecordSXFOBJ  oSXFObj, char * psRecordBuf )
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

	if (oSXFObj.bDimIdea == 1) // TODO realise this
	{
		 CPLError( CE_Fatal, CPLE_NotSupported, 
                  "SXF. 3D metrics do not support." );
	}

    for(int count=0 ; count <  oSXFObj.nPointsCount ; count++)
    {
        char * psCoords = psRecordBuf + nOffset ;
        nOffset += TranslateXYH( oSXFObj, psCoords , &dfX, &dfY ) ;
        poLS->addPoint( dfX  , dfY );
    }

    poMLS->addGeometry( poLS );

/*---------------------- Reading Sub Lines --------------------------------*/

    for(int count=0 ; count <  oSXFObj.nSubObjCount ; count++)
    {
    poLS->empty();
    char * psBuff = psRecordBuf + nOffset;

    GUInt16 nSubObj = *(GUInt16*) psBuff; 
    GUInt16 nCoords = *(GUInt16*) (psBuff + 2); 

    nOffset +=4;

        for (int i=0; i < nCoords ; i++)
        {
        char * psCoords = psRecordBuf + nOffset ;

         nOffset +=  TranslateXYH( oSXFObj, psCoords , &dfX, &dfY ) ;

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
OGRFeature *OGRSXFLayer::TranslatePolygon( RecordSXFOBJ  oSXFObj, char * psRecordBuf )
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

		if (oSXFObj.bDimIdea == 1) // TODO realise this
		{
			 CPLError( CE_Fatal, CPLE_NotSupported, 
					  "SXF. 3D metrics do not support." );
		}

    for(int count=0 ; count <  oSXFObj.nPointsCount ; count++)
    {
        char * psBuf = psRecordBuf + nOffset ;

        nOffset +=  TranslateXYH( oSXFObj, psBuf , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );

    }    // for

        OGRLinearRing *poLR = new OGRLinearRing();
        poLR->addSubLineString( poLS, 0 );

        poPoly->addRingDirectly( poLR );

/*---------------------- Reading Sub Lines --------------------------------*/

    for(int count=0 ; count <  oSXFObj.nSubObjCount ; count++)
    {
    poLS->empty();
    char * psBuff = psRecordBuf + nOffset;

    GUInt16 nSubObj = *(GUInt16*) psBuff; 
    GUInt16 nCoords = *(GUInt16*) (psBuff + 2); 

    nOffset +=4;

        for (int i=0; i < nCoords ; i++)
        {
        char * psCoords = psRecordBuf + nOffset ;

         nOffset +=  TranslateXYH( oSXFObj, psCoords , &dfX, &dfY ) ;

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
OGRFeature *OGRSXFLayer::TranslateText( RecordSXFOBJ  oSXFObj, char * psRecordBuf )
{
    double dfX = 1.0;
    double dfY = 1.0;
    GUInt32 nOffset = 0;

		OGRFeatureDefn *fd = poFeatureDefn->Clone();
		fd->SetGeomType( wkbLineString );
        OGRFeature *poFeature = new OGRFeature(fd);
        OGRLineString* poLS = new OGRLineString();

		if (oSXFObj.bDimIdea == 1) // TODO realise this
		{
			 CPLError( CE_Fatal, CPLE_NotSupported, 
					  "SXF. 3D metrics do not support." );
		}

    for(int count=0 ; count <  oSXFObj.nPointsCount ; count++)
    {
        char * psBuf = psRecordBuf + nOffset;

        nOffset += TranslateXYH( oSXFObj, psBuf , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );

    }    // for


        poFeature->SetGeometryDirectly( poLS );

/*------------------     READING TEXT VALUE   ---------------------------------------*/

if ( oSXFObj.nSubObjCount == 0)
  {
      char * pszTxt = psRecordBuf +(16*oSXFObj.nPointsCount);

        GByte nTextL = (GByte) *pszTxt;

        char * pszTextBuf = (char *)CPLMalloc( nTextL+1 );

        strncpy(pszTextBuf, (pszTxt+1),    nTextL+1);

        poFeature->SetField( "TEXT", pszTextBuf );
 
//         poFeature->SetStyleString("LABEL(f:\"Arial\",t:\"Hello ...\",s:\"20\")");
//       printf("\n Text lenght = %#d text : %s ",nTextL, pszTextBuf);

        CPLFree( pszTextBuf );  
  }


/*****
 * TODO : 
 *          - Translate graphics 
 *          - Translate 3D vector
 */

        return poFeature;
}



