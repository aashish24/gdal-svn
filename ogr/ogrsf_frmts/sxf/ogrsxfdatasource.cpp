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
	OGRErr getOGRSpatialReference(RecordSXFPSP & oSXFSP, OGRSpatialReference *poSRS)
	{
		if (oSXFSP.iProjSys == PanaramaProjCode::PROJ_TM)
		{

			if (oSXFSP.iDatum == PanaramaDatumCode::DATUM_PUL_42 && 
				oSXFSP.dfYnw >= 1000000)
			{
				size_t ZoneNumber = oSXFSP.dfYnw / 1000000;
				if (ZoneNumber >= 2 && ZoneNumber <=32)
				{
					int EPSG = 28400 + ZoneNumber;
					poSRS->importFromEPSG(EPSG);
					return OGRERR_NONE;
				}
			}
			
			double ScaleFactor = 1.0;
			size_t ZoneNumber = 0;
				
			double axialMerInDegrees = oSXFSP.dfAxialMer * 180 / M_PI;
			ZoneNumber = axialMerInDegrees / 6 + 1;

			double padfPrjParams[8] = {oSXFSP.dfMainPar1, oSXFSP.dfMainPar2, oSXFSP.dfMainPtPar, 
                               oSXFSP.dfAxialMer,      ScaleFactor,  
                               oSXFSP.dfFalseEasting, oSXFSP.dfFalseNorthing, ZoneNumber };

			OGRErr err = poSRS->importFromPanorama( oSXFSP.iProjSys, oSXFSP.iDatum, oSXFSP.iEllips, padfPrjParams );
			if ( err != OGRERR_NONE )
			{
				return err;
			}
		}
		
		if (oSXFSP.iProjSys == PanaramaProjCode::PROJ_UTM)
		{
			double ScaleFactor = 0.9996;
			size_t ZoneNumber = 0;
			double axialMerInDegrees = oSXFSP.dfAxialMer * 180 / M_PI;
			
			if (axialMerInDegrees/6 + 1 > 30)
				ZoneNumber = axialMerInDegrees/6 + 1 - 30;
			else
				ZoneNumber = axialMerInDegrees/6 + 1 + 30;

			double padfPrjParams[8] = {oSXFSP.dfMainPar1, oSXFSP.dfMainPar2, oSXFSP.dfMainPtPar, 
                               oSXFSP.dfAxialMer,      ScaleFactor,  
                               oSXFSP.dfFalseEasting, oSXFSP.dfFalseNorthing, ZoneNumber };

			OGRErr err = poSRS->importFromPanorama( oSXFSP.iProjSys, oSXFSP.iDatum, oSXFSP.iEllips, padfPrjParams );
			if ( err != OGRERR_NONE )
			{
				return err;
			}
		}
		
		double ScaleFactor = 1;
		double ZoneNumber = 0;
	
		double padfPrjParams[8] = {oSXFSP.dfMainPar1, oSXFSP.dfMainPar2, oSXFSP.dfMainPtPar, 
                               oSXFSP.dfAxialMer,      ScaleFactor,  
                               oSXFSP.dfFalseEasting, oSXFSP.dfFalseNorthing, ZoneNumber };

		OGRErr err = poSRS->importFromPanorama( oSXFSP.iProjSys, oSXFSP.iDatum, oSXFSP.iEllips, padfPrjParams );
		return err;
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

// -------------------------------------------------------------------- 
//      Does this appear to be a .sxf file?
// --------------------------------------------------------------------

    RecordSXFHEAD FileHeader;
    RecordSXFPSP  oSXFSP;
    RecordSXFDSC  oSXFDSC;

    fpSXF = VSIFOpenL(pszName, "rb");
    if ( fpSXF == NULL )
        return FALSE;


/*---------------- READ THE SXF FILE HEADER  ---------------------------------*/

    size_t nSXFFileHeaderSize = sizeof(RecordSXFHEAD);
    nObjectsRead = VSIFReadL( &FileHeader, nSXFFileHeaderSize, 1, fpSXF );

    if (nObjectsRead != 1)
    {
        CPLError(CE_Failure, CPLE_None , "SXF Read of head failed");
        CloseFile();
        return FALSE;
    }

	if ( FileHeader.nVersion != IDSXFVERSION )    
	{
		CPLError(CE_Failure, CPLE_NotSupported , "SXF File version not supported");

		CloseFile();
		return FALSE;
	}


    size_t nSXFFileSPSize = sizeof(oSXFSP);
    nObjectsRead = VSIFReadL( &oSXFSP, nSXFFileSPSize, 1, fpSXF );

    if (nObjectsRead != 1)
    {
        CPLError(CE_Failure, CPLE_None , "SXF Read of passport failed");
        CloseFile();
        return FALSE;
    }

    size_t nSXFFileDSCSize = sizeof(oSXFDSC);
    nObjectsRead = VSIFReadL( &oSXFDSC, nSXFFileDSCSize, 1, fpSXF );
  
    if (nObjectsRead != 1)
    {
        CPLError(CE_Failure, CPLE_None , "SXF Read of descriptor failed");
        CloseFile();
        return FALSE;
    }
    /*
	printf("NOMENCLATURE OF THE SHEET (ANSI): %s \n", oSXFSP.szName);
	printf("SCALE OF SHEET (DENOMINATOR): %d \n", oSXFSP.nScale);

	printf("RECTANGULAR COORDINATES OF THE ANGLES OF SHEET (in meters):\n");
	printf("	dfXsw: %f \n", oSXFSP.dfXsw);
	printf("	dfYsw: %f \n", oSXFSP.dfYsw);
	printf("	dfXnw: %f \n", oSXFSP.dfXnw);
	printf("	dfYnw: %f \n", oSXFSP.dfYnw);
	printf("	dfXne: %f \n", oSXFSP.dfXne);
	printf("	dfYne: %f \n", oSXFSP.dfYne);
	printf("	dfXse: %f \n", oSXFSP.dfXse);
	printf("	dfYse: %f \n", oSXFSP.dfYse);

	printf("GEODETIC COORDINATES OF THE ANGLES OF THE SHEET (in radians):\n");
	printf("	dfBsw: %f \n", oSXFSP.dfBsw);
	printf("	dfLsw: %f \n", oSXFSP.dfLsw);
	printf("	dfBnw: %f \n", oSXFSP.dfBnw);
	printf("	dfLnw: %f \n", oSXFSP.dfLnw);
	printf("	dfBne: %f \n", oSXFSP.dfBne);
	printf("	dfLne: %f \n", oSXFSP.dfLne);
	printf("	dfBse: %f \n", oSXFSP.dfBse);
	printf("	dfLse: %f \n", oSXFSP.dfLse);

	
	printf("Flag of the state of the data (passport): %d \n", oSXFSP.bDataFlag);
	printf("Flag of the state of the data (desc): %d \n", oSXFDSC.bDataFlag);

	printf("Flag of the correspondence to the projection (passport): %d \n", oSXFSP.bProjCorres);
	printf("Flag of the correspondence to the projection (desc): %d \n", oSXFDSC.bProjCorres);

	printf("Flag of the presence of the real coordinates (passport): %d \n", oSXFSP.bpnalrealk);
	printf("Flag of the presence of the real coordinates (desc): %d \n", oSXFDSC.bpnalrealk);
	
	printf("\n");

	printf("Classifier of the map objects: %d \n", oSXFSP.iMapCC);
	printf("CLASSIFICATION CODE OF THE FRAMEWORK OF THE OBJECT: %d \n", oSXFSP.iFrameCC);

	printf("\n");

	printf("CHARACTERISTICS OF INSTRUMENT: %d \n", oSXFSP.iDeviceCapability); 
	printf("Arrangement of the framework on the instrument: (in the system of instrument):\n");
	printf("	dfXswp: %d \n", oSXFSP.dfXswp);
	printf("	dfYswp: %d \n", oSXFSP.dfYswp);
	printf("	dfXnwp: %d \n", oSXFSP.dfXnwp);
	printf("	dfYnwp: %d \n", oSXFSP.dfYnwp);
	printf("	dfXnep: %d \n", oSXFSP.dfXnep);
	printf("	dfYnep: %d \n", oSXFSP.dfYnep);
	printf("	dfXsep: %d \n", oSXFSP.dfXsep);
	printf("	dfYsep: %d \n", oSXFSP.dfYsep);

    printf("\n");

    printf(" Number of recordings of the data: %d \n", oSXFDSC.nObjCount);

	printf("bDataFlag: %d \n", oSXFSP.bDataFlag);
	printf("bpnalrealk: %d \n", oSXFSP.bpnalrealk);

	
	
   printf("iEllips: %d \n", oSXFSP.iEllips);
   printf("iHeightSys: %d \n", oSXFSP.iHeightSys);
   printf("iProjSys: %d \n", oSXFSP.iProjSys);
   printf("iDatum: %d \n", oSXFSP.iDatum);
   printf("iPlanUoM: %d \n", oSXFSP.iPlanUoM);
   printf("iFrameForm: %d \n", oSXFSP.iFrameForm);
   

	printf("dfMainPar1: %f \n", oSXFSP.dfMainPar1);
	printf("dfMainPar2: %f \n", oSXFSP.dfMainPar2);
	printf("dfAxialMer: %f \n", oSXFSP.dfAxialMer);
	printf("dfMainPtPar: %f \n", oSXFSP.dfMainPtPar);
	printf("dfFalseNorthing: %f \n", oSXFSP.dfFalseNorthing);
	printf("dfFalseEasting: %f \n", oSXFSP.dfFalseEasting);


/*---------------- READ THE RSC FILE HEADER  ---------------------------*/

    //try find RSC file
    CPLString pszRSCRileName = CPLResetExtension(pszFilename, "rsc");
    if (CPLCheckForFile((char *)pszRSCRileName.c_str(), NULL) == FALSE)
    {
        //try get rsc file from config
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
        if (fpSXF == NULL)
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

/*----------------- SOME CHECKS -----------------------------------------*/
    /*
	if (oSXFSP.bDataFlag != FLAGSXFDATACOMMUNICATION)
	{
        CPLError( CE_Fatal, CPLE_NotSupported,
                  "SXF. Wrong state of the data." );
		return FALSE;
	}
    */

	if (oSXFSP.bProjCorres != FLAGSXFDATAINPROJECTION)
	{
		CPLError( CE_Fatal, CPLE_NotSupported, 
                  "SXF. Data are not corresponde to the projection." );
		return FALSE;
	}

	
	if (oSXFSP.bpnalrealk != FLAGSXFREALCOORDINATES)
	{
		CPLError( CE_Warning, CPLE_NotSupported, 
                  "SXF. Data are not of the real coordinates." );
	}

/*---------------- Reda RSC file -------------------------------------------*/

    if (fpRSC != NULL)
    {
        ReadRSCLayers(RSCFileHeader);
    }

/*---------------- Spatial index for all Layers --------------------*/

    OGRSpatialReference *poSRS = new OGRSpatialReference();
    getOGRSpatialReference(oSXFSP, poSRS);

/*---------------- Layers Creation ---------------------------------*/ 

    CreateLayers(oSXFSP, oSXFDSC, poSRS);

    return TRUE;
}

void OGRSXFDataSource::CreateLayers(
        RecordSXFPSP  &oSXFPSP,
        RecordSXFDSC &oSXFDSC,
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
            papoLayers[nLayers] = new OGRSXFLayer(fpSXF, osLayerName, poSRS, oSXFPSP, oSXFDSC, layerClassifiers, &layerIt->second);
            nLayers++;
        }
    }
    else
    {
        CPLDebug("SXF","Layers from SXF");

        RecordSXFOBJ oSXFObj;
        std::set<GInt32> classifiersCodes;

        vsi_l_offset objectOffset = VSIFTellL( fpSXF );
        while(VSIFReadL( &oSXFObj, sizeof(RecordSXFOBJ), 1, fpSXF ))
        {
            VSIFSeekL(fpSXF, oSXFObj.nRecordLength - sizeof(RecordSXFOBJ), SEEK_CUR);

            if ( classifiersCodes.find(oSXFObj.iCC) == classifiersCodes.end())
            {
                classifiersCodes.insert(oSXFObj.iCC);

                osLayerName.Printf("%d", oSXFObj.iCC);

                std::set<GInt32> layerClassifiers;
                layerClassifiers.insert(oSXFObj.iCC);

                papoLayers = (OGRLayer**) CPLRealloc(papoLayers, sizeof(OGRLayer*) * (nLayers+1));
                papoLayers[nLayers] = new OGRSXFLayer(fpSXF, osLayerName, poSRS, oSXFPSP, oSXFDSC, layerClassifiers, NULL);
                nLayers++;
            }

            objectOffset += oSXFObj.nRecordLength;
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
