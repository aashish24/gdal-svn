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
            case PanaramaEllips::ELLIPS_42:
            {
               mathBase.iEllips =  PanaramaEllips::ELLIPS_42;
               break;
            }
            case PanaramaEllips::ELLIPS_WGS_76:
            {
               mathBase.iEllips =  PanaramaEllips::ELLIPS_WGS_76;
               break;
            }
            default:
            {
                mathBase.iEllips =  PanaramaEllips::ELLIPS_UNKNOWN;
                break;
            }
        }

        switch( *(buf+2) )
        {
            case PanaramaProjCode::PROJ_TM:
            {
               mathBase.iProjSys =  PanaramaProjCode::PROJ_TM;
               break;
            }
            case PanaramaProjCode::PROJ_LCC:
            {
               mathBase.iProjSys =  PanaramaProjCode::PROJ_LCC;
               break;
            }
            case PanaramaProjCode::PROJ_UTM:
            {
               mathBase.iProjSys =  PanaramaProjCode::PROJ_UTM;
               break;
            }
            default:
            {
                mathBase.iProjSys =  PanaramaProjCode::PROJ_UNKNOWN;
                break;
            }
        }

        switch( *(buf+3) )
        {
            case PanaramaDatumCode::DATUM_PUL_42:
            {
               mathBase.iDatum =  PanaramaDatumCode::DATUM_PUL_42;
               break;
            }
            default:
            {
                mathBase.iDatum =  PanaramaDatumCode::DATUM_UNKNOWN;
                break;
            }
        }


        if (version == IDSXFVERSION4)
            switch( *(buf+4) )
            {
                case 0:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_METRE;
                    break;
                }
                case 64:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_RADIAN;
                    break;
                }
                case 65:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_DEGREE;
                    break;
                }
                default:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_METRE;
                    break;
                }
            }
        else if (version == IDSXFVERSION3)
            switch( *(buf+4) )
            {
                case 0:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_METRE;
                    break;
                }
                case 1:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_DECIMETRE;
                    break;
                }
                case 2:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_CENTIMETRE;
                    break;
                }
                case 3:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_MILLIMETRE;
                    break;
                }
                case 130:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_RADIAN;
                    break;
                }
                case 129:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_DEGREE;
                    break;
                }
                default:
                {
                    mathBase.unitInPlan = CoordinateMeasUnit::CMU_METRE;
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

        /*
        printf("fr dfXsw: %f\n", deviceInfo.deviceFrameCoordinates.dfXsw);
        printf("fr dfYsw: %f\n", deviceInfo.deviceFrameCoordinates.dfYsw);
        printf("fr dfXnw: %f\n", deviceInfo.deviceFrameCoordinates.dfXnw);
        printf("fr dfYnw: %f\n", deviceInfo.deviceFrameCoordinates.dfYnw);
        printf("fr dfXne: %f\n", deviceInfo.deviceFrameCoordinates.dfXne);
        printf("fr dfYne: %f\n", deviceInfo.deviceFrameCoordinates.dfYne);
        printf("fr dfXse: %f\n", deviceInfo.deviceFrameCoordinates.dfXse);
        printf("fr dfYse: %f\n", deviceInfo.deviceFrameCoordinates.dfYse);
        */
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
        /*
        printf("RECTANGULAR COORDINATES OF THE ANGLES OF SHEET (in meters):\n");
        printf("	dfXsw: %f \n", sheetRectCoordinates.dfXsw);
        printf("	dfYsw: %f \n", sheetRectCoordinates.dfYsw);
        printf("	dfXnw: %f \n", sheetRectCoordinates.dfXnw);
        printf("	dfYnw: %f \n", sheetRectCoordinates.dfYnw);
        printf("	dfXne: %f \n", sheetRectCoordinates.dfXne);
        printf("	dfYne: %f \n", sheetRectCoordinates.dfYne);
        printf("	dfXse: %f \n", sheetRectCoordinates.dfXse);
        printf("	dfYse: %f \n", sheetRectCoordinates.dfYse);

        printf("MATH BASE:\n");
        printf("	iProjSys: %d \n", mathBase.iProjSys);
        printf("	iDatum: %d \n", mathBase.iDatum);
        printf("	iEllips: %d \n", mathBase.iEllips);
        */

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

            if (projectionInfo.dfAxialMer == 0xffffff) projectionInfo.dfAxialMer *= std::pow(10.0,6);
            if (projectionInfo.dfMainPar1 == 0xffffff) projectionInfo.dfMainPar1 *= std::pow(10.0,6);
            if (projectionInfo.dfMainPar2 == 0xffffff) projectionInfo.dfMainPar2 *= std::pow(10.0,6);
            if (projectionInfo.dfMainPtPar == 0xffffff) projectionInfo.dfMainPtPar *= std::pow(10.0,6);

            projectionInfo.dfFalseEasting = 500000;
            projectionInfo.dfFalseNorthing = 0;
        }
        else
            CPLError(CE_Fatal, CPLE_NotSupported , "Function calculateOGRSpatialReference does not support SXF File version %x", version);

        /*
        printf("ProjectionInfo:\n");
        printf("    dfMainPar1: %f \n", projectionInfo.dfMainPar1);
        printf("    dfMainPar2: %f \n", projectionInfo.dfMainPar2);
        printf("    dfAxialMer: %f \n", projectionInfo.dfAxialMer);
        printf("    dfMainPtPar: %f \n", projectionInfo.dfMainPtPar);
        printf("    dfFalseNorthing: %f \n", projectionInfo.dfFalseNorthing);
        printf("    dfFalseEasting: %f \n", projectionInfo.dfFalseEasting);
        */

        double ScaleFactor = 1.0;
        size_t ZoneNumber = 0;

        if (mathBase.iProjSys == PanaramaProjCode::PROJ_TM)
        {
            if (mathBase.iDatum == PanaramaDatumCode::DATUM_PUL_42 &&
                sheetRectCoordinates.dfYnw >= 1000000)
            {
                ZoneNumber = sheetRectCoordinates.dfYnw / 1000000;
                if (ZoneNumber >= 2 && ZoneNumber <=32)
                {
                    int EPSG = 28400 + ZoneNumber;
                    poSRS.importFromEPSG(EPSG);
                    return poSRS;
                }
            }

            double axialMerInDegrees = projectionInfo.dfAxialMer * 180 / M_PI;
            ZoneNumber = axialMerInDegrees / 6 + 1;

            if (version == IDSXFVERSION3)
            {
                projectionInfo.dfFalseEasting += ZoneNumber * std::pow(10.0, 6);
            }

            double padfPrjParams[8] = {projectionInfo.dfMainPar1, projectionInfo.dfMainPar2, projectionInfo.dfMainPtPar,
                               projectionInfo.dfAxialMer,      ScaleFactor,
                               projectionInfo.dfFalseEasting, projectionInfo.dfFalseNorthing, ZoneNumber };

            OGRErr err = poSRS.importFromPanorama( mathBase.iProjSys, mathBase.iDatum, mathBase.iEllips, padfPrjParams );
            return poSRS;
        }

        if (mathBase.iProjSys == PanaramaProjCode::PROJ_UTM)
        {
            double ScaleFactor = 0.9996;
            size_t ZoneNumber = 0;
            double axialMerInDegrees = projectionInfo.dfAxialMer * 180 / M_PI;

            if (axialMerInDegrees/6 + 1 > 30)
                ZoneNumber = axialMerInDegrees/6 + 1 - 30;
            else
                ZoneNumber = axialMerInDegrees/6 + 1 + 30;

            if (version == IDSXFVERSION3)
            {
                projectionInfo.dfFalseEasting += ZoneNumber * std::pow(10.0, 6);
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
    for( int i = 0; i < nLayers; i++ )
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
    if( iLayer < 0 || iLayer >= nLayers )
        return NULL;
    else
        return papoLayers[iLayer];
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

int OGRSXFDataSource::Open( const char * pszFilename, int bUpdateIn)
{
    SXFInfo oSXFInfo;

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


    oSXFInfo.version = readSXFVersion(fpSXF);
    if ( oSXFInfo.version == IDSXFVERSIONUNKNOWN )
    {
        CPLError(CE_Failure, CPLE_NotSupported , "SXF File version not supported");
        CloseFile();
        return FALSE;
    }

    oSXFInfo.informationFlags = readSXFInformationFlags(fpSXF, oSXFInfo.version);
    if (oSXFInfo.informationFlags.dataState != SXFDataState::SXF_DS_EXCHANGE)
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "SXF. Wrong state of the data." );
        //return FALSE;
    }
    if (oSXFInfo.informationFlags.projectionDataCompliance == false)
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "SXF. Data are not corresponde to the projection." );
        return FALSE;
    }

    oSXFInfo.mathBase = readSXFMathBase(fpSXF, oSXFInfo.version);
    oSXFInfo.nScale = readSXFMapScale(fpSXF, oSXFInfo.version);

    oSXFInfo.deviceInfo = readSXFDeviceInfo(fpSXF, oSXFInfo.version);

    oSXFInfo.sheetRectCoordinates = readSXFSheetCornersCoordinates(fpSXF, oSXFInfo.version);

/*---------------- TRY READ THE RSC FILE HEADER  -----------------------*/

    CPLString pszRSCRileName = CPLResetExtension(pszFilename, "rsc");
    if (CPLCheckForFile((char *)pszRSCRileName.c_str(), NULL) == FALSE)
    {
        pszRSCRileName = CPLGetConfigOption("RSC_FILENAME", "");
        if (CPLCheckForFile((char *)pszRSCRileName.c_str(), NULL) == FALSE)
        {
            CPLError(CE_Warning, CPLE_None, "RSC file %s not exist", pszRSCRileName);
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

/*---------------- Reda RSC file -----------------------------------*/

    if (fpRSC != NULL)
    {
        ReadRSCLayers(RSCFileHeader);
    }

/*---------------- Spatial index for all Layers --------------------*/

    OGRSpatialReference *poSRS = new OGRSpatialReference();
    *poSRS = calculateOGRSpatialReference(fpSXF, oSXFInfo.mathBase, oSXFInfo.sheetRectCoordinates, oSXFInfo.version);

/*---------------- Layers Creation ---------------------------------*/ 

    CreateLayers(oSXFInfo, poSRS);

    return TRUE;
}

void OGRSXFDataSource::CreateLayers(
        SXFInfo& sxfInfo,
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

        GByte* buf = (GByte*)CPLMalloc(32);
        std::set<GInt32> classifiersCodes;

        vsi_l_offset objectOffset = VSIFTellL( fpSXF );
        while(VSIFReadL( buf, 32, 1, fpSXF ))
        {
            GInt32 nRecordLength = *(GInt32*)(buf+4);
            GInt32 iCC = *(GInt32*)(buf+12);

            VSIFSeekL(fpSXF, nRecordLength - 32, SEEK_CUR);

            if ( classifiersCodes.find(iCC) == classifiersCodes.end())
            {
                classifiersCodes.insert(iCC);

                osLayerName.Printf("%d", iCC);

                std::set<GInt32> layerClassifiers;
                layerClassifiers.insert(iCC);

                papoLayers = (OGRLayer**) CPLRealloc(papoLayers, sizeof(OGRLayer*) * (nLayers+1));
                papoLayers[nLayers] = new OGRSXFLayer(fpSXF, osLayerName, poSRS, sxfInfo, layerClassifiers, NULL);
                nLayers++;
            }

            objectOffset += nRecordLength;
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

    for (int i = 0; i < RSCFileHeader.Layers.nRecordCount; ++i)
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

    for (int i = 0; i < RSCFileHeader.Objects.nRecordCount; ++i)
    {
        VSIFReadL(&OBJECT, 1, sizeof(_object), fpRSC);

        std::string rscobjName(OBJECT.szName);
        rscLayers[OBJECT.szLayernNo].rscObjects.insert(std::pair<GUInt32, std::string>(OBJECT.szClassifyCode, rscobjName));

        nOffset += OBJECT.nLength;
        VSIFSeekL(fpRSC, nOffset, SEEK_SET);
    }
}
