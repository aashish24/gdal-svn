/******************************************************************************
 * $Id: ogr_sxflayer.cpp  $
 *
 * Project:  SXF Translator
 * Purpose:  Definition of classes for OGR SXF Layers.
 * Author:   Ben Ahmed Daho Ali, bidandou(at)yahoo(dot)fr
 *           Dmitry Baryshnikov, polimax@mail.ru
 *           Alexandr Lisovenko
 *
 ******************************************************************************
 * Copyright (c) 2011, Ben Ahmed Daho Ali
 * Copyright (c) 2013, NextGIS
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

OGRSXFLayer::OGRSXFLayer(VSILFILE* fp, GByte nID, const char* pszLayerName, int nVer, const SXFMapDescription&  sxfMapDesc) : OGRLayer()
{
    fpSXF = fp;
    nLayerID = nID;
    stSXFMapDescription = sxfMapDesc;
    stSXFMapDescription.pSpatRef->Reference();
    m_nSXFFormatVer = nVer;
    oNextIt = mnRecordDesc.begin();

    poFeatureDefn = new OGRFeatureDefn(pszLayerName);
    poFeatureDefn->Reference();
    
    poFeatureDefn->SetGeomType(wkbGeometryCollection);
    //OGRGeomFieldDefn oGeomFieldDefn("Shape", wkbGeometryCollection);
    //oGeomFieldDefn.SetSpatialRef(stSXFMapDescription.pSpatRef);
    //poFeatureDefn->AddGeomFieldDefn(&oGeomFieldDefn);

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
}

/************************************************************************/
/*                         ~OGRSXFLayer()                               */
/************************************************************************/

OGRSXFLayer::~OGRSXFLayer()
{
    stSXFMapDescription.pSpatRef->Release();
    poFeatureDefn->Release();
}

/************************************************************************/
/*                AddClassifyCode(unsigned nClassCode)                  */
/* Add layer supported classify codes. Only records with this code can  */
/* be in layer                                                          */
/************************************************************************/

void OGRSXFLayer::AddClassifyCode(unsigned nClassCode, const char *szName)
{
    if (szName != NULL)
    {
        mnClassificators[nClassCode] = CPLString(szName);
    }
    else
    {
        CPLString szIdName;
        szIdName.Printf("%d", nClassCode);
        mnClassificators[nClassCode] = szIdName;
    }
}

/************************************************************************/
/*                         AddRecord()                               */
/************************************************************************/

int OGRSXFLayer::AddRecord(int nFID, unsigned nClassCode, vsi_l_offset nOffset, bool bHasSemantic)
{
    if (mnClassificators.empty() || mnClassificators.find(nClassCode) != mnClassificators.end())
    {
        mnRecordDesc[nFID] = nOffset;
        //add addtionals semantics (attribute fields)
        if (/*mnClassificators[nClassCode] != 1 && */bHasSemantic)
        {
            //mnClassificators[nClassCode] = 1;
            SXFRecordAttributeInfo stAttrInfo;
            int nReadObj = VSIFReadL(&stAttrInfo, sizeof(SXFRecordAttributeInfo), 1, fpSXF);
            if (nReadObj == 1)
            {
                if (snAttributeCodes.find(stAttrInfo.nCode) != snAttributeCodes.end())
                {
                    return TRUE;
                }

                snAttributeCodes.insert(stAttrInfo.nCode);
                CPLString oFieldName;
                oFieldName.Printf("SC_%d", stAttrInfo.nCode);

                SXFRecordAttributeType eType = (SXFRecordAttributeType)stAttrInfo.nType;

                switch (eType)
                {
                case SXF_RAT_ONEBYTE:
                case SXF_RAT_TWOBYTE:
                case SXF_RAT_FOURBYTE:
                    {
                        OGRFieldDefn  oField(oFieldName, OFTReal);
                        poFeatureDefn->AddFieldDefn( &oField );

                        break;
                    }
                case SXF_RAT_EIGHTBYTE:
                    {
                        OGRFieldDefn  oField( oFieldName, OFTReal );
                        poFeatureDefn->AddFieldDefn( &oField );

                        break;
                    }
                case SXF_RAT_ASCIIZ_DOS:
                case SXF_RAT_ANSI_WIN:
                case SXF_RAT_UNICODE:
                    {
                        OGRFieldDefn  oField( oFieldName, OFTString );
                        oField.SetWidth(255);
                        poFeatureDefn->AddFieldDefn( &oField );

                        break;
                    }
                case SXF_RAT_BIGTEXT:
                    {
                        OGRFieldDefn  oField( oFieldName, OFTString );
                        oField.SetWidth(1024);
                        poFeatureDefn->AddFieldDefn( &oField );

                        break;
                    }
                }
            }
        }
        return TRUE;
    }

    return FALSE;
}

/************************************************************************/
/*                           SetNextByIndex()                           */
/************************************************************************/

OGRErr OGRSXFLayer::SetNextByIndex(long nIndex)
{
    if (nIndex < 0 || nIndex > mnRecordDesc.size())
        return OGRERR_FAILURE;

    oNextIt = mnRecordDesc.begin();
    std::advance(oNextIt, nIndex);

    return OGRERR_NONE;
}

/************************************************************************/
/*                             GetFeature()                             */
/************************************************************************/

OGRFeature *OGRSXFLayer::GetFeature(long nFID)
{
    std::map<int, vsi_l_offset>::const_iterator IT = mnRecordDesc.find(nFID);
    if (IT != mnRecordDesc.end())
    {
        oNextIt = IT;
        return GetNextFeature();
    }

    return NULL;
}

/************************************************************************/
/*                           GetSpatialRef()                            */
/************************************************************************/

OGRSpatialReference *OGRSXFLayer::GetSpatialRef()
{
    return stSXFMapDescription.pSpatRef;
}

/************************************************************************/
/*                             GetExtent()                              */
/************************************************************************/

OGRErr OGRSXFLayer::GetExtent(OGREnvelope *psExtent, int bForce)
{
    if (bForce)
    {
        return OGRLayer::GetExtent(psExtent, bForce);
    }
    else
    {
        psExtent->MinX = stSXFMapDescription.Env.MinX;
        psExtent->MaxX = stSXFMapDescription.Env.MaxX;
        psExtent->MinY = stSXFMapDescription.Env.MinY;
        psExtent->MaxY = stSXFMapDescription.Env.MaxY;

        return OGRERR_NONE;
    }
}

/************************************************************************/
/*                          GetFeatureCount()                           */
/************************************************************************/

int OGRSXFLayer::GetFeatureCount(int bForce)
{
    return mnRecordDesc.size();
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRSXFLayer::ResetReading()

{
    oNextIt = mnRecordDesc.begin();
}


/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRSXFLayer::GetNextFeature()
{
    if (oNextIt == mnRecordDesc.end())
        return NULL;

    OGRFeature  *poFeature = NULL;

    while (TRUE)
    {
        VSIFSeekL(fpSXF, oNextIt->second, SEEK_SET);
        poFeature = GetNextRawFeature();

        oNextIt++;
        if (poFeature == NULL)
            break;

        if ((m_poFilterGeom == NULL
            || FilterGeometry(poFeature->GetGeometryRef()))
            && (m_poAttrQuery == NULL
            || m_poAttrQuery->Evaluate(poFeature)))
        {
            break;
        }

        delete poFeature;
    }

    return poFeature;
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRSXFLayer::TestCapability( const char * pszCap )

{
    if (EQUAL(pszCap, OLCStringsAsUTF8))
        return TRUE;
    else if (EQUAL(pszCap, OLCRandomRead))
        return TRUE;
    else if (EQUAL(pszCap, OLCFastFeatureCount))
        return TRUE;
    else if (EQUAL(pszCap, OLCFastGetExtent))
        return TRUE;
    else if (EQUAL(pszCap, OLCFastSetNextByIndex))
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

GUInt32 OGRSXFLayer::TranslateXYH(const SXFRecordDescription& certifInfo, char *psBuff,
                          double *dfX, double *dfY, double *dfH)
{
    //Xp, Yp(м) = Xo, Yo(м) + (Xd, Yd / R * S), (1)

    double dfCoeff = stSXFMapDescription.dfScale / stSXFMapDescription.nResolution;

	int offset = 0;
    switch (certifInfo.eValType)
    {
    case SXF_VT_SHORT:
    {
        short y = *(short *)(psBuff);
        short x = *(short *)(psBuff + 2);

        if (stSXFMapDescription.bIsRealCoordinates)
        {
            *dfX = (double)x;
            *dfY = (double)y;
        }
        else
        {
            *dfX = stSXFMapDescription.dfXOr + (double)x * dfCoeff;
            *dfY = stSXFMapDescription.dfYOr + (double)y * dfCoeff;
        }

        offset += 4;

        if (dfH != NULL)
        {
            float h = *(float *)(psBuff + 8); // H always in float
            *dfH = (double)h;

            offset += 4;
        }
    }
        break;
    case SXF_VT_FLOAT:
    {
        float y = *(float *)(psBuff);
        float x = *(float *)(psBuff + 4);

        if (stSXFMapDescription.bIsRealCoordinates)
        {
            *dfX = (double)x;
            *dfY = (double)y;
        }
        else
        {
            *dfX = stSXFMapDescription.dfXOr + (double)x * dfCoeff;
            *dfY = stSXFMapDescription.dfYOr + (double)y * dfCoeff;
        }

        offset += 8;

        if (dfH != NULL)
        {
            float h = *(float *)(psBuff + 8);  // H always in float
            *dfH = (double)h;

            offset += 4;
        }
    }
        break;
    case SXF_VT_INT:
    {
        int y = *(int *)(psBuff);
        int x = *(int *)(psBuff + 4);

        if (stSXFMapDescription.bIsRealCoordinates)
        {
            *dfX = (double)x;
            *dfY = (double)y;
        }
        else
        {
            *dfX = stSXFMapDescription.dfXOr + (double)x * dfCoeff;
            *dfY = stSXFMapDescription.dfYOr + (double)y * dfCoeff;
        }

        offset += 8;

        if (dfH != NULL)
        {
            float h = *(float *)(psBuff + 8); // H always in float
            *dfH = (double)h;

            offset += 4;
        }
    }
        break;
    case SXF_VT_DOUBLE:
    {
        if (stSXFMapDescription.bIsRealCoordinates)
        {
            *dfX = *(double *)(psBuff);
            *dfY = *(double *)(psBuff + 8);
        }
        else
        {
            *dfX = stSXFMapDescription.dfXOr + *(double *)(psBuff)* dfCoeff;
            *dfY = stSXFMapDescription.dfYOr + *(double *)(psBuff + 8) * dfCoeff;
        }

        offset += 16;

        if (dfH != NULL)
        {
            *dfH = *(double *)(psBuff + 16);
            offset += 8;
        }
    }
        break;
    };

    return offset;
}

/************************************************************************/
/*                         GetNextRawFeature()                          */
/************************************************************************/

OGRFeature *OGRSXFLayer::GetNextRawFeature()
{
    SXFRecordHeader stRecordHeader;
    int nObjectRead;

    nObjectRead = VSIFReadL(&stRecordHeader, sizeof(SXFRecordHeader), 1, fpSXF);

    if (nObjectRead != 1 || stRecordHeader.nID != IDSXFOBJ)
    {
        CPLError(CE_Fatal, CPLE_FileIO, "SXF. Read record failed.");
        return NULL;
    }

    SXFGeometryType eGeomType;
    GByte code;
    if (m_nSXFFormatVer == 3)
        code = stRecordHeader.nRef[0] & 0x0f;
    else if (m_nSXFFormatVer == 4)
        code = stRecordHeader.nRef[0];

    if (code == 0x00) // xxxx0000
        eGeomType = SXF_GT_Line;
    else if (code == 0x01) // xxxx0001
        eGeomType = SXF_GT_Polygon;
    else if (code == 0x02) // xxxx0010
        eGeomType = SXF_GT_Point;
    else if (code == 0x03) // xxxx0011
        eGeomType = SXF_GT_Text;
    else if (code == 0x04) // xxxx0100
        eGeomType = SXF_GT_Vector;
    else if (code == 0x05) // xxxx0101
        eGeomType = SXF_GT_TextTemplate;

    bool bHasAttributes = CHECK_BIT(stRecordHeader.nRef[1], 1);
    bool bHasRefVector = CHECK_BIT(stRecordHeader.nRef[1], 3);
    if (bHasRefVector == true)
        CPLError(CE_Failure, CPLE_NotSupported,
        "SXF. Parsing the vector of the tying not support.");

    SXFRecordDescription stCertInfo;
    if (stRecordHeader.nPointCountSmall == 65535)
    {
        stCertInfo.nPointCount = stRecordHeader.nPointCount;
    }
    else
    {
        stCertInfo.nPointCount = stRecordHeader.nPointCountSmall;
    }
    stCertInfo.nSubObjectCount = stRecordHeader.nSubObjectCount;

    bool bFloatType, bBigType;
    bool b3D(true);
    if (m_nSXFFormatVer == 3)
    {
        b3D = CHECK_BIT(stRecordHeader.nRef[2], 1);
        bFloatType = CHECK_BIT(stRecordHeader.nRef[2], 2);
        bBigType = CHECK_BIT(stRecordHeader.nRef[1], 2);
        stCertInfo.bHasTextSign = CHECK_BIT(stRecordHeader.nRef[2], 5);
    }
    else if (m_nSXFFormatVer == 4)
    {
        b3D = CHECK_BIT(stRecordHeader.nRef[2], 1);
        bFloatType = CHECK_BIT(stRecordHeader.nRef[2], 2);
        bBigType = CHECK_BIT(stRecordHeader.nRef[1], 2);
        stCertInfo.bHasTextSign = CHECK_BIT(stRecordHeader.nRef[2], 3);
    }

    if (b3D) //xххххх1х
        stCertInfo.bDim = 1;
    else
        stCertInfo.bDim = 0;

    if (bFloatType)
    {
        if (bBigType)
        {
            stCertInfo.eValType = SXF_VT_DOUBLE;
        }
        else
        {
            stCertInfo.eValType = SXF_VT_FLOAT;
        }
    }
    else
    {
        if (bBigType)
        {
            stCertInfo.eValType = SXF_VT_INT;
        }
        else
        {
            stCertInfo.eValType = SXF_VT_SHORT;
        }
    }


    stCertInfo.bFormat = CHECK_BIT(stRecordHeader.nRef[2], 0);

    OGRFeature *poFeature = NULL;
    char * recordCertifBuf = (char *)CPLMalloc(stRecordHeader.nGeometryLength);
    nObjectRead = VSIFReadL(recordCertifBuf, stRecordHeader.nGeometryLength, 1, fpSXF);
    if (nObjectRead != 1)
    {
        CPLError(CE_Failure, CPLE_FileIO,
            "SXF. Read geometry failed.");
        CPLFree(recordCertifBuf);
        return NULL;
    }

    if (eGeomType == SXF_GT_Point)
        poFeature = TranslatePoint(stCertInfo, recordCertifBuf);
    else if (eGeomType == SXF_GT_Line)
        poFeature = TranslateLine(stCertInfo, recordCertifBuf);
    else if (eGeomType == SXF_GT_Polygon)
        poFeature = TranslatePolygon(stCertInfo, recordCertifBuf);
    else if (eGeomType == SXF_GT_Text)
        poFeature = TranslateText(stCertInfo, recordCertifBuf);
    /*else if (eGeomType == SXF_GT_Vector ) // TODO realise this
      {
      CPLError( CE_Warning, CPLE_NotSupported,
      "SXF. Geometry type Vector do not support." );
      }
      else if (eGeomType == SXF_GT_TextTemplate ) // TODO realise this
      {
      CPLError( CE_Warning, CPLE_NotSupported,
      "SXF. Geometry type Text Template do not support." );
      }*/
    else
    {
        CPLError(CE_Failure, CPLE_NotSupported,
            "SXF. Unsupported geometry type.");
        CPLFree(recordCertifBuf);
        return NULL;
    }

    poFeature->SetField("CLCODE", (int)stRecordHeader.nClassifyCode);

    CPLString szName = mnClassificators[stRecordHeader.nClassifyCode];

    if (szName.empty())
    {
        szName.Printf("%d", stRecordHeader.nClassifyCode);
    }
    poFeature->SetField("CLNAME", szName);

    poFeature->SetField("OBJECTNUMB", stRecordHeader.nSubObjectCount);

    if (bHasAttributes)
    {
        size_t  nSemanticsSize = stRecordHeader.nFullLength - 32 - stRecordHeader.nGeometryLength;
        char * psSemanticsdBuf = (char *)CPLMalloc(nSemanticsSize);
        char * psSemanticsdBufOrig = psSemanticsdBuf;
        nObjectRead = VSIFReadL(psSemanticsdBuf, nSemanticsSize, 1, fpSXF);
        if (nObjectRead == 1)
        {
            size_t offset = 0;
            while (offset < nSemanticsSize)
            {
                char *psSemanticsdBufBeg = psSemanticsdBuf + offset;
                SXFRecordAttributeInfo stAttInfo = *(SXFRecordAttributeInfo*)psSemanticsdBufBeg;
                offset += 4;

                CPLString oFieldName;
                oFieldName.Printf("SC_%d", stAttInfo.nCode);

                CPLString oFieldValue;

                SXFRecordAttributeType eType = (SXFRecordAttributeType)stAttInfo.nType;

                switch (eType)
                {
                case SXF_RAT_ASCIIZ_DOS:
                {
                    char * value = (char*)CPLMalloc(stAttInfo.nScale + 1);
                    memcpy(value, psSemanticsdBuf + offset, stAttInfo.nScale + 1);
                    poFeature->SetField(oFieldName, value);//TODO: CPLRecode(value, "CP866", CPL_ENC_UTF8)

                    offset += stAttInfo.nScale + 1;
                    break;
                }
                case SXF_RAT_ONEBYTE:
                {
                    double nVal = *(GByte *)(psSemanticsdBuf + offset);
                    nVal *= pow(10.0, (double)stAttInfo.nScale);

                    poFeature->SetField(oFieldName, nVal);
                    offset += 1;
                    break;
                }
                case SXF_RAT_TWOBYTE:
                {
                    double nVal = *(GInt16 *)(psSemanticsdBuf + offset);
                    nVal *= pow(10.0, (double)stAttInfo.nScale);

                    poFeature->SetField(oFieldName, nVal);
                    offset += 2;
                    break;
                }
                case SXF_RAT_FOURBYTE:
                {
                    double nVal = *(GInt32 *)(psSemanticsdBuf + offset);
                    nVal *= pow(10.0, (double)stAttInfo.nScale);

                    poFeature->SetField(oFieldName, nVal);
                    offset += 4;
                    break;
                }
                case SXF_RAT_EIGHTBYTE:
                {
                    double d = *(double *)(psSemanticsdBuf + offset);
                    d *= pow(10.0, (double)stAttInfo.nScale);
                    poFeature->SetField(oFieldName, d);

                    offset += 8;
                    break;
                }
                case SXF_RAT_ANSI_WIN:
                {
                    char * value = (char*)CPLMalloc(stAttInfo.nScale + 1);
                    memcpy(value, psSemanticsdBuf + offset, stAttInfo.nScale + 1);
                    poFeature->SetField(oFieldName, CPLRecode(value, "CP1251", CPL_ENC_UTF8));

                    offset += stAttInfo.nScale + 1;
                    break;
                }
                case SXF_RAT_UNICODE:
                {
                    char * value = (char*)CPLMalloc(stAttInfo.nScale + 1);
                    memcpy(value, psSemanticsdBuf + offset, stAttInfo.nScale + 1);
                    poFeature->SetField(oFieldName, value);

                    offset += stAttInfo.nScale + 1;
                    break;
                }
                case SXF_RAT_BIGTEXT:
                {
                    GUInt32 scale2 = *(GUInt32 *)(psSemanticsdBuf + offset);
                    char * value = (char*)CPLMalloc(scale2 + 1);
                    memcpy(value, psSemanticsdBuf + offset, scale2 + 1);
                    poFeature->SetField(oFieldName, CPLRecode(value, CPL_ENC_UTF16, CPL_ENC_UTF8));

                    offset += scale2;
                    break;
                }
                default:
                    CPLFree(recordCertifBuf);
                    CPLFree(psSemanticsdBufOrig);
                    return NULL;
                }
            }
            CPLFree(psSemanticsdBufOrig);
        }
    }

    poFeature->SetFID(oNextIt->first);

    CPLFree(recordCertifBuf);

    return poFeature;
}

/************************************************************************/
/*                         TranslatePoint   ()                          */
/************************************************************************/

OGRFeature *OGRSXFLayer::TranslatePoint(const SXFRecordDescription& certifInfo, char * psRecordBuf)
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

    for(int count=0 ; count <  certifInfo.nSubObjectCount ; count++)
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

OGRFeature *OGRSXFLayer::TranslateLine(const SXFRecordDescription& certifInfo, char * psRecordBuf)
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

    for(int count=0 ; count <  certifInfo.nPointCount ; count++)
    {
        char * psCoords = psRecordBuf + nOffset ;
        nOffset += TranslateXYH( certifInfo, psCoords , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );
    }

    poMLS->addGeometry( poLS );

/*---------------------- Reading Sub Lines --------------------------------*/

    for(int count=0 ; count <  certifInfo.nSubObjectCount ; count++)
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

OGRFeature *OGRSXFLayer::TranslatePolygon(const SXFRecordDescription& certifInfo, char * psRecordBuf)
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

    for(int count=0 ; count <  certifInfo.nPointCount ; count++)
    {
        char * psBuf = psRecordBuf + nOffset ;

        nOffset +=  TranslateXYH( certifInfo, psBuf , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );

    }    // for

        OGRLinearRing *poLR = new OGRLinearRing();
        poLR->addSubLineString( poLS, 0 );

        poPoly->addRingDirectly( poLR );

/*---------------------- Reading Sub Lines --------------------------------*/

    for(int count=0 ; count <  certifInfo.nSubObjectCount ; count++)
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
OGRFeature *OGRSXFLayer::TranslateText(const SXFRecordDescription& certifInfo, char * psRecordBuf)
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

    for(int count=0 ; count <  certifInfo.nPointCount ; count++)
    {
        char * psBuf = psRecordBuf + nOffset;

        nOffset += TranslateXYH( certifInfo, psBuf , &dfX, &dfY ) ;

        poLS->addPoint( dfX  , dfY );
    }

    poFeature->SetGeometryDirectly( poLS );

/*------------------     READING TEXT VALUE   ---------------------------------------*/

    if ( certifInfo.nSubObjectCount == 0 && certifInfo.bHasTextSign == true)
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



