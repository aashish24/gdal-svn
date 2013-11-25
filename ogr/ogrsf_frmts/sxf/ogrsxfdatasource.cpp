/******************************************************************************
 * $Id: ogr_sxfdatasource.cpp  $
 *
 * Project:  SXF Translator
 * Purpose:  Definition of classes for OGR SXF Datasource.
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

#include "ogr_sxf.h"
#include "cpl_conv.h"
#include "cpl_string.h"

#include <math.h>
#include <map>
#include <string>

CPL_CVSID("$Id: ogrsxfdatasource.cpp  $");

namespace
{
    SXFVersion readSXFVersion(VSILFILE* fpSXF)
    {
        GByte *buf;

        VSIFSeekL(fpSXF, 8, SEEK_SET);
        buf = (GByte *)CPLMalloc(4);
        VSIFReadL( buf, 4 , 1, fpSXF );

        if (*(SXFVersion*)buf == IDSXFVERSION4)
        {
            return IDSXFVERSION4;
        }
        CPLFree(buf);

        VSIFSeekL(fpSXF, 8, SEEK_SET);
        buf = (GByte *)CPLMalloc(2);
        VSIFReadL( buf, 2 , 1, fpSXF );

        if ((SXFVersion)*(GInt16*)buf == IDSXFVERSION3)
        {
            return IDSXFVERSION3;
        }
        CPLFree(buf);

        return IDSXFVERSIONUNKNOWN;
    }
    SXFInformationFlags readSXFInformationFlags(VSILFILE* fpSXF, const SXFVersion& version)
    {
        SXFInformationFlags informationFlags;
        GByte *buf;

        if (version == IDSXFVERSION4)
        {
            VSIFSeekL(fpSXF, 96, SEEK_SET);
        }
        else if (version == IDSXFVERSION3)
        {
            VSIFSeekL(fpSXF, 78, SEEK_SET);
        }
        else
        {
            CPLError(CE_Fatal, CPLE_NotSupported , "Function readSXFInformationFlags does not support SXF File version %x", version);
        }

        buf = (GByte *)CPLMalloc(1);
        VSIFReadL( buf, 1 , 1, fpSXF );

        if ( (*buf & 0x03) == 0x03 ) // 00000011
            informationFlags.dataState = SXF_DS_EXCHANGE;
        else
            informationFlags.dataState = SXF_DS_UNKNOWN;

        if ( (*buf & 0x04) == 0x04 ) // 00000100
            informationFlags.projectionDataCompliance = true;
        else
            informationFlags.projectionDataCompliance = false;

        if ( (*buf & 0x18) == 0x18 ) // 00011000
            informationFlags.realCoordinatesCompliance = true;
        else
            informationFlags.realCoordinatesCompliance = false;


        CPLFree(buf);

        return informationFlags;
    }
    SheetCornersCoordinates readSXFSheetCornersCoordinates(VSILFILE* fpSXF, const SXFVersion& version)
    {
        SheetCornersCoordinates coordinates; // (in meters)

        if (version == IDSXFVERSION4)
        {
            VSIFSeekL(fpSXF, 104, SEEK_SET);
            VSIFReadL( &coordinates, 64 , 1, fpSXF );
        }
        else if (version == IDSXFVERSION3)
        {
            struct _Coordinates // coordinate in decimetre
            {
                GInt32   dfXsw  ; /* X the South Western angle (vertical line) */
                GInt32   dfYsw  ; /* Y  */
                GInt32   dfXnw  ; /* X the North Western angle */
                GInt32   dfYnw  ; /* Y  */
                GInt32   dfXne  ; /* X the North Eastern angle */
                GInt32   dfYne  ; /* Y  */
                GInt32   dfXse  ; /* X the South Eastern angle */
                GInt32   dfYse  ; /* Y  */
            };
            _Coordinates _coordinates;
            VSIFSeekL(fpSXF, 94, SEEK_SET);
            VSIFReadL( &_coordinates, sizeof(_coordinates) , 1, fpSXF );

            coordinates.dfXsw = (long double)_coordinates.dfXsw / 10;
            coordinates.dfYsw = (long double)_coordinates.dfYsw / 10;
            coordinates.dfXnw = (long double)_coordinates.dfXnw / 10;
            coordinates.dfYnw = (long double)_coordinates.dfYnw / 10;
            coordinates.dfXne = (long double)_coordinates.dfXne / 10;
            coordinates.dfYne = (long double)_coordinates.dfYne / 10;
            coordinates.dfXse = (long double)_coordinates.dfXse / 10;
            coordinates.dfYse = (long double)_coordinates.dfYse / 10;
        }
        else
        {
            CPLError(CE_Fatal, CPLE_NotSupported , "Function readSXFSheetCornersCoordinates does not support SXF File version %x", version);
        }

        return coordinates;
    }
    SXFMathBase readSXFMathBase(VSILFILE* fpSXF, const SXFVersion& version)
    {
        SXFMathBase mathBase;

        GByte *buf = (GByte *)CPLMalloc(5);
        if (version == IDSXFVERSION4)
            VSIFSeekL(fpSXF, 232, SEEK_SET);
        else if (version == IDSXFVERSION3)
            VSIFSeekL(fpSXF, 158, SEEK_SET);
        else
            CPLError(CE_Fatal, CPLE_NotSupported , "Function readSXFMathBase does not support SXF File version %x", version);

        VSIFReadL( buf, 5 , 1, fpSXF );

        switch(*buf)
        {
            case ELLIPS_42:
            {
               mathBase.iEllips =  ELLIPS_42;
               break;
            }
            case ELLIPS_WGS_76:
            {
               mathBase.iEllips =  ELLIPS_WGS_76;
               break;
            }
            default:
            {
                mathBase.iEllips =  ELLIPS_UNKNOWN;
                break;
            }
        }

        switch( *(buf+2) )
        {
            case PROJ_TM:
            {
               mathBase.iProjSys =  PROJ_TM;
               break;
            }
            case PROJ_LCC:
            {
               mathBase.iProjSys =  PROJ_LCC;
               break;
            }
            case PROJ_UTM:
            {
               mathBase.iProjSys =  PROJ_UTM;
               break;
            }
            default:
            {
                mathBase.iProjSys =  PROJ_UNKNOWN;
                break;
            }
        }

        switch( *(buf+3) )
        {
            case DATUM_PUL_42:
            {
               mathBase.iDatum =  DATUM_PUL_42;
               break;
            }
            default:
            {
                mathBase.iDatum =  DATUM_UNKNOWN;
                break;
            }
        }


        if (version == IDSXFVERSION4)
            switch( *(buf+4) )
            {
                case 0:
                {
                    mathBase.unitInPlan = CMU_METRE;
                    break;
                }
                case 64:
                {
                    mathBase.unitInPlan = CMU_RADIAN;
                    break;
                }
                case 65:
                {
                    mathBase.unitInPlan = CMU_DEGREE;
                    break;
                }
                default:
                {
                    mathBase.unitInPlan = CMU_METRE;
                    break;
                }
            }
        else if (version == IDSXFVERSION3)
            switch( *(buf+4) )
            {
                case 0:
                {
                    mathBase.unitInPlan = CMU_METRE;
                    break;
                }
                case 1:
                {
                    mathBase.unitInPlan = CMU_DECIMETRE;
                    break;
                }
                case 2:
                {
                    mathBase.unitInPlan = CMU_CENTIMETRE;
                    break;
                }
                case 3:
                {
                    mathBase.unitInPlan = CMU_MILLIMETRE;
                    break;
                }
                case 130:
                {
                    mathBase.unitInPlan = CMU_RADIAN;
                    break;
                }
                case 129:
                {
                    mathBase.unitInPlan = CMU_DEGREE;
                    break;
                }
                default:
                {
                    mathBase.unitInPlan = CMU_METRE;
                    break;
                }
            }
        else
            CPLError(CE_Fatal, CPLE_NotSupported , "Function readSXFMathBase does not support SXF File version %x", version);



        return mathBase;
    }
    SXFDeviceInfo readSXFDeviceInfo(VSILFILE* fpSXF, const SXFVersion& version)
    {
        SXFDeviceInfo deviceInfo;

        GByte *buf = (GByte *)CPLMalloc(4);
        if (version == IDSXFVERSION4)
            VSIFSeekL(fpSXF, 312, SEEK_SET);
        else if (version == IDSXFVERSION3)
            VSIFSeekL(fpSXF, 212, SEEK_SET);
        else
            CPLError(CE_Fatal, CPLE_NotSupported , "Function readSXFDeviceCapability does not support SXF File version %x", version);

        VSIFReadL( buf, 4 , 1, fpSXF );

        deviceInfo.iDeviceCapability = *(GInt32*)buf;

        if (version == IDSXFVERSION4)
        {
            struct _Coordinates_v4 // coordinate in decimetre
            {
                GInt32   dfXsw  ; /* X the South Western angle (vertical line) */
                GInt32   dfYsw  ; /* Y  */
                GInt32   dfXnw  ; /* X the North Western angle */
                GInt32   dfYnw  ; /* Y  */
                GInt32   dfXne  ; /* X the North Eastern angle */
                GInt32   dfYne  ; /* Y  */
                GInt32   dfXse  ; /* X the South Eastern angle */
                GInt32   dfYse  ; /* Y  */
            };
            _Coordinates_v4 _coordinates_v4;

            VSIFSeekL(fpSXF, 316, SEEK_SET);
            VSIFReadL( &_coordinates_v4, sizeof(_coordinates_v4) , 1, fpSXF );


            deviceInfo.deviceFrameCoordinates.dfXsw = (long double)_coordinates_v4.dfXsw;
            deviceInfo.deviceFrameCoordinates.dfYsw = (long double)_coordinates_v4.dfYsw;
            deviceInfo.deviceFrameCoordinates.dfXnw = (long double)_coordinates_v4.dfXnw;
            deviceInfo.deviceFrameCoordinates.dfYnw = (long double)_coordinates_v4.dfYnw;
            deviceInfo.deviceFrameCoordinates.dfXne = (long double)_coordinates_v4.dfXne;
            deviceInfo.deviceFrameCoordinates.dfYne = (long double)_coordinates_v4.dfYne;
            deviceInfo.deviceFrameCoordinates.dfXse = (long double)_coordinates_v4.dfXse;
            deviceInfo.deviceFrameCoordinates.dfYse = (long double)_coordinates_v4.dfYse;
        }
        else if (version == IDSXFVERSION3)
        {
            struct _Coordinates_v3 // coordinate in decimetre
            {
                GInt16   dfXsw  ; /* X the South Western angle (vertical line) */
                GInt16   dfYsw  ; /* Y  */
                GInt16   dfXnw  ; /* X the North Western angle */
                GInt16   dfYnw  ; /* Y  */
                GInt16   dfXne  ; /* X the North Eastern angle */
                GInt16   dfYne  ; /* Y  */
                GInt16   dfXse  ; /* X the South Eastern angle */
                GInt16   dfYse  ; /* Y  */
            };
            _Coordinates_v3 _coordinates_v3;
            VSIFSeekL(fpSXF, 216, SEEK_SET);
            VSIFReadL( &_coordinates_v3, sizeof(_coordinates_v3) , 1, fpSXF );

            deviceInfo.deviceFrameCoordinates.dfXsw = (long double)_coordinates_v3.dfXsw;
            deviceInfo.deviceFrameCoordinates.dfYsw = (long double)_coordinates_v3.dfYsw;
            deviceInfo.deviceFrameCoordinates.dfXnw = (long double)_coordinates_v3.dfXnw;
            deviceInfo.deviceFrameCoordinates.dfYnw = (long double)_coordinates_v3.dfYnw;
            deviceInfo.deviceFrameCoordinates.dfXne = (long double)_coordinates_v3.dfXne;
            deviceInfo.deviceFrameCoordinates.dfYne = (long double)_coordinates_v3.dfYne;
            deviceInfo.deviceFrameCoordinates.dfXse = (long double)_coordinates_v3.dfXse;
            deviceInfo.deviceFrameCoordinates.dfYse = (long double)_coordinates_v3.dfYse;
        }
        else
        {
            CPLError(CE_Fatal, CPLE_NotSupported , "Function readSXFSheetCornersCoordinates does not support SXF File version %x", version);
        }

        return deviceInfo;
    }

    GUInt32 readSXFMapScale(VSILFILE* fpSXF, const SXFVersion& version)
    {
        GByte *buf = (GByte *)CPLMalloc(4);

        if (version == IDSXFVERSION4)
            VSIFSeekL(fpSXF, 60, SEEK_SET);
        else if (version == IDSXFVERSION3)
            VSIFSeekL(fpSXF, 48, SEEK_SET);
        else
            CPLError(CE_Fatal, CPLE_NotSupported , "Function readSXFMapScale does not support SXF File version %x", version);

        VSIFReadL( buf, 4 , 1, fpSXF );

        GUInt32 scale = *(GUInt32*)buf;

        return scale;
    }

    OGRSpatialReference calculateOGRSpatialReference(
            VSILFILE* fpSXF,
            const SXFMathBase& mathBase,
            const SheetCornersCoordinates& sheetRectCoordinates,
            const SXFVersion& version)
    {
        OGRSpatialReference poSRS;

        ProjectionInfo projectionInfo;

        if (version == IDSXFVERSION4)
        {
            VSIFSeekL(fpSXF, 352, SEEK_SET);
            VSIFReadL( &projectionInfo, sizeof(projectionInfo) , 1, fpSXF );
        }
        else if (version == IDSXFVERSION3)
        {
            struct _ProjectionInfoSXFv3
            {
                GInt32   dfMainPar1;
                GInt32   dfMainPar2;
                GInt32   dfAxialMer;
                GInt32   dfMainPtPar;
            };
            _ProjectionInfoSXFv3 _projectionInfoSXFv3;
            VSIFSeekL(fpSXF, 236, SEEK_SET);
            VSIFReadL( &_projectionInfoSXFv3, sizeof(_projectionInfoSXFv3) , 1, fpSXF );

            projectionInfo.dfAxialMer = (long double)_projectionInfoSXFv3.dfAxialMer;
            projectionInfo.dfMainPar1 = (long double)_projectionInfoSXFv3.dfMainPar1;
            projectionInfo.dfMainPar2 = (long double)_projectionInfoSXFv3.dfMainPar2;
            projectionInfo.dfMainPtPar = (long double)_projectionInfoSXFv3.dfMainPtPar;

            if (projectionInfo.dfAxialMer == 0xffffff) projectionInfo.dfAxialMer *= pow(10.0,6);
            if (projectionInfo.dfMainPar1 == 0xffffff) projectionInfo.dfMainPar1 *= pow(10.0,6);
            if (projectionInfo.dfMainPar2 == 0xffffff) projectionInfo.dfMainPar2 *= pow(10.0,6);
            if (projectionInfo.dfMainPtPar == 0xffffff) projectionInfo.dfMainPtPar *= pow(10.0,6);

            projectionInfo.dfFalseEasting = 500000;
            projectionInfo.dfFalseNorthing = 0;
        }
        else
            CPLError(CE_Fatal, CPLE_NotSupported , "Function calculateOGRSpatialReference does not support SXF File version %x", version);

        double ScaleFactor = 1.0;
        size_t ZoneNumber = 0;

        if (mathBase.iProjSys == PROJ_TM)
        {
            if (mathBase.iDatum == DATUM_PUL_42 &&
                sheetRectCoordinates.dfYnw >= 1000000)
            {
                ZoneNumber = (size_t)(sheetRectCoordinates.dfYnw / 1000000);
                if (ZoneNumber >= 2 && ZoneNumber <=32)
                {
                    int EPSG = 28400 + ZoneNumber;
                    poSRS.importFromEPSG(EPSG);
                    return poSRS;
                }
            }

            double axialMerInDegrees = projectionInfo.dfAxialMer * 180 / M_PI;
            ZoneNumber = (size_t)(axialMerInDegrees / 6 + 1);

            if (version == IDSXFVERSION3)
            {
                projectionInfo.dfFalseEasting += ZoneNumber * pow(10.0, 6);
            }

            double padfPrjParams[8] = {projectionInfo.dfMainPar1, projectionInfo.dfMainPar2, projectionInfo.dfMainPtPar,
                               projectionInfo.dfAxialMer,      ScaleFactor,
                               projectionInfo.dfFalseEasting, projectionInfo.dfFalseNorthing, ZoneNumber };

            OGRErr err = poSRS.importFromPanorama( mathBase.iProjSys, mathBase.iDatum, mathBase.iEllips, padfPrjParams );
            return poSRS;
        }

        if (mathBase.iProjSys == PROJ_UTM)
        {
            double ScaleFactor = 0.9996;
            size_t ZoneNumber = 0;
            double axialMerInDegrees = projectionInfo.dfAxialMer * 180 / M_PI;

            if (axialMerInDegrees/6 + 1 > 30)
                ZoneNumber = (size_t)(axialMerInDegrees/6 + 1 - 30);
            else
                ZoneNumber = (size_t)(axialMerInDegrees/6 + 1 + 30);

            if (version == IDSXFVERSION3)
            {
                projectionInfo.dfFalseEasting += ZoneNumber * pow(10.0, 6);
            }

            double padfPrjParams[8] = {projectionInfo.dfMainPar1, projectionInfo.dfMainPar2, projectionInfo.dfMainPtPar,
                               projectionInfo.dfAxialMer,      ScaleFactor,
                               projectionInfo.dfFalseEasting, projectionInfo.dfFalseNorthing, ZoneNumber };

            OGRErr err = poSRS.importFromPanorama( mathBase.iProjSys, mathBase.iDatum, mathBase.iEllips, padfPrjParams );
            return poSRS;
        }


        double padfPrjParams[8] = {projectionInfo.dfMainPar1, projectionInfo.dfMainPar2, projectionInfo.dfMainPtPar,
                               projectionInfo.dfAxialMer,      ScaleFactor,
                               projectionInfo.dfFalseEasting, projectionInfo.dfFalseNorthing, ZoneNumber };

        OGRErr err = poSRS.importFromPanorama( mathBase.iProjSys, mathBase.iDatum, mathBase.iEllips, padfPrjParams );
        return poSRS;

    }

    SXFRecordCertifInfo getSXFRecordCertifInfo(GByte* buf, const SXFVersion& version)
    {
        SXFRecordCertifInfo certifInfo;

        if( (*(buf+22) & 0x02) == 0x02) //xххххх1х
            certifInfo.bDim = 1;
        else
            certifInfo.bDim = 0;

        if( (*(buf+21) & 0x04) == 0x04) //xхххх1xх
            certifInfo.bElemSize = 1;
        else
            certifInfo.bElemSize = 0;

        if( (*(buf+22) & 0x04) == 0x04) //xхххх1xх
            certifInfo.bElemType = 1;
        else
            certifInfo.bElemType = 0;

        if (version == IDSXFVERSION4)
        {
            if( (*(buf+22) & 0x08) == 0x08) //xхxх1xxх
                certifInfo.bHasTextSign = true;
            else
                certifInfo.bHasTextSign = false;
        }
        else if (version == IDSXFVERSION3)
        {
            if( (*(buf+22) & 0x20) == 0x20) //xх1ххxxх
                certifInfo.bHasTextSign = true;
            else
                certifInfo.bHasTextSign = false;
        }

        if( (*(buf+22) & 0x01) == 0x01) //xхххxxx1
            certifInfo.bRecordFormat = 1;
        else
            certifInfo.bRecordFormat = 0;

        certifInfo.nPointsCount = *(GUInt16*)(buf+30);

        certifInfo.nSubObjCount = *(GUInt16*)(buf+28);

        return certifInfo;
    }

}

bool readSXFRecord(VSILFILE* fpSXF, const SXFPassport& passport, SXFRecordInfo& recordInfo)
{
    GByte* buf = (GByte*)CPLMalloc(recordHeaderSize);

    size_t count = VSIFReadL( buf, 1, recordHeaderSize, fpSXF);

    if(count != recordHeaderSize)
    {
        return false;
    }

    recordInfo.certifInfo = getSXFRecordCertifInfo(buf, passport.version);

    recordInfo.nRecordLength = *(GInt32*)(buf+4);
    recordInfo.nCertifLength = *(GInt32*)(buf+8);

    recordInfo.iCC = *(GInt32*)(buf+12);

    recordInfo.nObjNumb = *(GUInt16*)(buf+16);

    if (passport.version == IDSXFVERSION4)
    {
        GByte code = *(buf+20) & 0x0f;
        if ( code == 0x00 ) // xxxx0000
            recordInfo.bGeomType = sxfLine;
        else if ( code == 0x01 ) // xxxx0001
            recordInfo.bGeomType = sxfPolygon;
        else if ( code == 0x02 ) // xxxx0010
            recordInfo.bGeomType = sxfPoint;
        else if ( code == 0x03 ) // xxxx0011
            recordInfo.bGeomType = sxfText;
        else if ( code == 0x04 ) // xxxx0100
            recordInfo.bGeomType = sxfVector;
        else if ( code == 0x05 ) // xxxx0101
            recordInfo.bGeomType = sxfTextTemplate;
    }
    else if (passport.version == IDSXFVERSION3)
    {
        GByte code = *(buf+20) & 0x0f;
        if ( code == 0x00 ) // xxxxxx00
            recordInfo.bGeomType = sxfLine;
        else if ( code == 0x01 ) // xxxxxx01
            recordInfo.bGeomType = sxfPolygon;
        else if ( code == 0x02 ) // xxxxxx10
            recordInfo.bGeomType = sxfPoint;
        else if ( code == 0x03 ) // xxxxxx11
            recordInfo.bGeomType = sxfText;
    }

    if( (*(buf+21) & 0x02) == 0x02) //xххххх1х
        recordInfo.bHazSemantics = true;
    else
        recordInfo.bHazSemantics = false;


    if (passport.version == IDSXFVERSION4)
    {
        if( (*(buf+21) & 0x08) == 0x08) //xххх1xxх
            recordInfo.bHazTyingVect = true;
        else
            recordInfo.bHazTyingVect = false;
    }
    else if (passport.version == IDSXFVERSION3)
    {
        recordInfo.bHazTyingVect = false;
    }

    return true;
}

/************************************************************************/
/*                      OGRSXFDataSource()                       */
/************************************************************************/

OGRSXFDataSource::OGRSXFDataSource()

{
    papoLayers = NULL;
    nLayers = 0;

    fpSXF = NULL;
    fpRSC = NULL;
}

/************************************************************************/
/*                     ~OGRSXFDataSource()                       */
/************************************************************************/

OGRSXFDataSource::~OGRSXFDataSource()

{
    for( size_t i = 0; i < nLayers; i++ )
        delete papoLayers[i];
    CPLFree( papoLayers );
}

/************************************************************************/
/*                     CloseFile()                                        */
/************************************************************************/
void  OGRSXFDataSource::CloseFile()
{ 
    if (NULL != fpSXF)
    {
        VSIFCloseL( fpSXF );
    }

    if (NULL != fpRSC)
    {
        VSIFCloseL( fpRSC );
    }
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRSXFDataSource::TestCapability( const char * pszCap )

{
    return FALSE;
}

/************************************************************************/
/*                              GetLayer()                              */
/************************************************************************/

OGRLayer *OGRSXFDataSource::GetLayer( int iLayer )

{
    if( iLayer < 0 || iLayer >= (int)nLayers )
        return NULL;
    else
        return papoLayers[iLayer];
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

int OGRSXFDataSource::Open( const char * pszFilename, int bUpdateIn)
{
    SXFPassport oSXFPassport;

    size_t nObjectsRead;

    if (bUpdateIn)
    {
        return FALSE;
    }

    pszName = pszFilename;

/* -------------------------------------------------------------------- */
/*      Determine what sort of object this is.                          */
/* -------------------------------------------------------------------- */

    VSIStatBufL sStatBuf;
    if (VSIStatL(pszName, &sStatBuf) != 0 ||
        !VSI_ISREG(sStatBuf.st_mode) ||
        !EQUAL(CPLGetExtension(pszName), "sxf"))
        return FALSE;


    fpSXF = VSIFOpenL(pszName, "rb");
    if ( fpSXF == NULL )
    {
        CPLError(CE_Warning, CPLE_OpenFailed, "SXF open file %s failed", pszFilename);
        return FALSE;
    }


    oSXFPassport.version = readSXFVersion(fpSXF);
    if ( oSXFPassport.version == IDSXFVERSIONUNKNOWN )
    {
        CPLError(CE_Failure, CPLE_NotSupported , "SXF File version not supported");
        CloseFile();
        return FALSE;
    }

    oSXFPassport.informationFlags = readSXFInformationFlags(fpSXF, oSXFPassport.version);
    if (oSXFPassport.informationFlags.dataState != SXF_DS_EXCHANGE)
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "SXF. Wrong state of the data." );
        //return FALSE;
    }
    if (oSXFPassport.informationFlags.projectionDataCompliance == false)
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "SXF. Data are not corresponde to the projection." );
        return FALSE;
    }

    oSXFPassport.mathBase = readSXFMathBase(fpSXF, oSXFPassport.version);

    if(oSXFPassport.informationFlags.realCoordinatesCompliance == false )
    {
        CPLError( CE_Warning, CPLE_NotSupported,
                  "SXF. Given material may be rotated in the conditional system of coordinates" );

        if(oSXFPassport.mathBase.unitInPlan != CMU_METRE)
        {
            CPLError( CE_Fatal, CPLE_None,
                      "SXF. Incorrect structure. The conditional system of coordinates, certificate mast be in the samples)" );
            return FALSE;
        }
    }

    oSXFPassport.nScale = readSXFMapScale(fpSXF, oSXFPassport.version);
    oSXFPassport.deviceInfo = readSXFDeviceInfo(fpSXF, oSXFPassport.version);
    oSXFPassport.sheetRectCoordinates = readSXFSheetCornersCoordinates(fpSXF, oSXFPassport.version);

/*---------------- TRY READ THE RSC FILE HEADER  -----------------------*/

    CPLString pszRSCRileName = CPLResetExtension(pszFilename, "rsc");
    if (CPLCheckForFile((char *)pszRSCRileName.c_str(), NULL) == FALSE)
    {
        pszRSCRileName = CPLGetConfigOption("RSC_FILENAME", "");
        if (CPLCheckForFile((char *)pszRSCRileName.c_str(), NULL) == FALSE)
        {
            CPLError(CE_Warning, CPLE_None, "RSC file %s not exist", pszRSCRileName.c_str());
            pszRSCRileName.Clear();
        }
    }

    RecordRSCHEAD RSCFileHeader;
    if (!pszRSCRileName.empty())
    {
        fpRSC = VSIFOpenL(pszRSCRileName, "rb");
        if (fpRSC == NULL)
        {
            CPLError(CE_Warning, CPLE_OpenFailed, "RSC open file %s failed", pszFilename);
        }
        else
        {
            int nRSCFileHeaderSize = sizeof(RSCFileHeader);
            nObjectsRead = VSIFReadL(&RSCFileHeader, nRSCFileHeaderSize, 1, fpRSC);

            if (nObjectsRead != 1)
            {
                CPLError(CE_Failure, CPLE_None, "RSC head read failed");
                CloseFile();
            }
        }
    }

    if (fpRSC != NULL)
    {
        ReadRSCLayers(RSCFileHeader);
    }

    OGRSpatialReference *poSRS = new OGRSpatialReference();
    *poSRS = calculateOGRSpatialReference(fpSXF, oSXFPassport.mathBase, oSXFPassport.sheetRectCoordinates, oSXFPassport.version);

    CreateLayers(oSXFPassport, poSRS);

    return TRUE;
}

void OGRSXFDataSource::CreateLayers(
        SXFPassport& sxfInfo,
        OGRSpatialReference *poSRS)
{
    CPLDebug("SXF","Create layers");
    CPLString osLayerName;

    if(rscLayers.size() != 0)
    {
        CPLDebug("SXF","Layers from RSC");

        RSCLayers::iterator layerIt;
        for (layerIt = rscLayers.begin(); layerIt != rscLayers.end(); ++layerIt)
        {
            osLayerName.Printf("%s", layerIt->second.szLayerName.c_str());

            RSCObjects& rscObjects = layerIt->second.rscObjects;

            std::set<GInt32> layerClassifiers;
            RSCObjects::iterator rscObjectIt;
            for (rscObjectIt = rscObjects.begin(); rscObjectIt != rscObjects.end(); ++rscObjectIt)
            {
                layerClassifiers.insert(rscObjectIt->first);
            }

            papoLayers = (OGRLayer**) CPLRealloc(papoLayers, sizeof(OGRLayer*) * (nLayers+1));
            papoLayers[nLayers] = new OGRSXFLayer(fpSXF, osLayerName, poSRS, sxfInfo, layerClassifiers, &layerIt->second);
            nLayers++;
        }
    }
    else
    {
        CPLDebug("SXF","Layers from SXF");

        std::set<GInt32> classifiersCodes;
        SXFRecordInfo recordInfo;

        vsi_l_offset objectOffset = VSIFTellL( fpSXF );

        while( readSXFRecord(fpSXF, sxfInfo, recordInfo) )
        {
            VSIFSeekL(fpSXF, recordInfo.nRecordLength - recordHeaderSize, SEEK_CUR);

            if ( classifiersCodes.find(recordInfo.iCC) == classifiersCodes.end())
            {
                classifiersCodes.insert(recordInfo.iCC);

                osLayerName.Printf("%d", recordInfo.iCC);

                std::set<GInt32> layerClassifiers;
                layerClassifiers.insert(recordInfo.iCC);

                papoLayers = (OGRLayer**) CPLRealloc(papoLayers, sizeof(OGRLayer*) * (nLayers+1));
                papoLayers[nLayers] = new OGRSXFLayer(fpSXF, osLayerName, poSRS, sxfInfo, layerClassifiers, NULL);



                nLayers++;
            }

            objectOffset += recordInfo.nRecordLength;
            VSIFSeekL(fpSXF, objectOffset, SEEK_SET);
        }
    }
}

void OGRSXFDataSource::ReadRSCLayers(RecordRSCHEAD &RSCFileHeader)
{
    CPLDebug("SXF","ReadRSCLayers");

    char szLayersID[4];
    struct _layer{
        unsigned nLength;
        char szName[32];
        char szShortName[16];
        GByte nNo;
        char nPos;
        short nSematicCount;
    };

    VSIFSeekL(fpRSC, RSCFileHeader.Layers.nOffset - sizeof(szLayersID), SEEK_SET);
    VSIFReadL(&szLayersID, 1, sizeof(szLayersID), fpRSC);
    vsi_l_offset nOffset = RSCFileHeader.Layers.nOffset;
    _layer LAYER;

    for (unsigned i = 0; i < RSCFileHeader.Layers.nRecordCount; ++i)
    {
        VSIFReadL(&LAYER, 1, sizeof(_layer), fpRSC);

        RSCLayer layer;
        layer.szLayerId = LAYER.nNo;
        layer.szLayerName = std::string( CPLRecode( LAYER.szName, "CP1251", "UTF-8") );
        rscLayers.insert(std::pair<GByte, RSCLayer>(LAYER.nNo, layer));

        nOffset += LAYER.nLength;
        VSIFSeekL(fpRSC, nOffset, SEEK_SET);
    }

    char szObjectsID[4];
    struct _object{
        unsigned nLength;
        unsigned szClassifyCode;
        unsigned szObjectNumber;
        unsigned szObjectCode;
        char szShortName[32];
        char szName[32];
        char szGeomType;
        char szLayernNo;
        char szUnimportantSeg[14];
    };

    VSIFSeekL(fpRSC, RSCFileHeader.Objects.nOffset - sizeof(szObjectsID), SEEK_SET);
    VSIFReadL(&szObjectsID, 1, sizeof(szObjectsID), fpRSC);
    nOffset = RSCFileHeader.Objects.nOffset;
    _object OBJECT;

    for (unsigned i = 0; i < RSCFileHeader.Objects.nRecordCount; ++i)
    {
        VSIFReadL(&OBJECT, 1, sizeof(_object), fpRSC);

        std::string rscobjName(OBJECT.szName);
        rscLayers[OBJECT.szLayernNo].rscObjects.insert(std::pair<GUInt32, std::string>(OBJECT.szClassifyCode, rscobjName));

        nOffset += OBJECT.nLength;
        VSIFSeekL(fpRSC, nOffset, SEEK_SET);
    }
}
