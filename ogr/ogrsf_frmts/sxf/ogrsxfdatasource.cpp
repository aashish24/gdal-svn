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
		CPLSetConfigOption("GDAL_DATA", "d:\\Development\\NextGIS\\GDAL\\gdal-1.10.1-cmake\\data\\");

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
		if ( err != OGRERR_NONE )
		{
			return err;
		}
	}
}
/************************************************************************/
/*                      OGRSXFDataSource()                       */
/************************************************************************/

OGRSXFDataSource::OGRSXFDataSource()

{
    papoLayers = NULL;
    nLayers = 0;

    pszName = NULL;
}

/************************************************************************/
/*                     ~OGRSXFDataSource()                       */
/************************************************************************/

OGRSXFDataSource::~OGRSXFDataSource()

{
    for( int i = 0; i < nLayers; i++ )
        delete papoLayers[i];

    CPLFree( papoLayers );
    CPLFree( pszName );

}

/************************************************************************/
/*                     CloseFile()                                        */
/************************************************************************/
void  OGRSXFDataSource::CloseFile()
{ 
    VSIFCloseL( fpSXF ); 
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

   if (bUpdateIn)
    {
        return FALSE;
    }

    pszName = CPLStrdup( pszFilename );
        
/* -------------------------------------------------------------------- */
/*      Determine what sort of object this is.                          */
/* -------------------------------------------------------------------- */
    VSIStatBufL sStatBuf;

    if( VSIStatL( pszFilename, &sStatBuf ) != 0 ||
        !VSI_ISREG(sStatBuf.st_mode) ||
        !EQUAL(CPLGetExtension(pszFilename), "sxf") )
        return FALSE;

// -------------------------------------------------------------------- 
//      Does this appear to be a .sxf file?
// --------------------------------------------------------------------

    RecordSXFHEAD FileHeader;
    RecordSXFPSP  oSXFSP;
    RecordSXFDSC  oSXFDSC;

    fpSXF = VSIFOpenL(pszFilename, "rb");
        if ( fpSXF == NULL )    
            return FALSE;

/*---------------- READ THE FILE HEADER  ---------------------------------*/ 

	VSIFReadL( &FileHeader, sizeof(FileHeader), 1, fpSXF );

	if ( FileHeader.nVersion != IDSXFVERSION )    
	{
		CPLError(CE_Failure, CPLE_NotSupported , "SXF File version not supported");

		CloseFile();
		return FALSE;
	}

	VSIFReadL( &oSXFSP, sizeof(oSXFSP), 1, fpSXF );
	VSIFReadL( &oSXFDSC, sizeof(oSXFDSC), 1, fpSXF );
  
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
	*/
	
/*----------------------------------------------------------------------*/
	
	if (oSXFSP.bDataFlag != FLAGSXFDATACOMMUNICATION)
	{
		CPLError( CE_Fatal, CPLE_NotSupported, 
                  "SXF. Wrong state of the data." );
		return FALSE;
	}

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
	
/*---------------- Read data from file ---------------------------------*/ 
	
	RecordSXFOBJ oSXFObj;

	std::map<GInt32, SetOfAttributesSXFOBJ> attributesTypesInLayers;
	std::map<GInt32, vsi_l_offset> firstObjectInLayers;

	vsi_l_offset objectOffset = VSIFTellL( fpSXF );

	while(VSIFReadL( &oSXFObj, sizeof(RecordSXFOBJ), 1, fpSXF ))
	{
		VSIFSeekL(fpSXF, oSXFObj.nCertifLength, SEEK_CUR);

		size_t nDataSize = oSXFObj.nRecordLength - sizeof(RecordSXFOBJ) - oSXFObj.nCertifLength;
        char * psRecordBuf = (char *) CPLMalloc( nDataSize );
        VSIFReadL( psRecordBuf, nDataSize , 1, fpSXF );

		SetOfAttributesSXFOBJ attributes;
		
		size_t attributeOffset = 0;
		
		while(attributeOffset < nDataSize)
		{
			
			AttributeSXFOBJ attr;

			GUInt16 code = *(GUInt16 *)(psRecordBuf + attributeOffset);
			attributeOffset += 2;
			attr.attributeCode = code;
			
			GByte type = *(GByte *)(psRecordBuf + attributeOffset);
			attributeOffset += 1;
			attr.attributeType = (AttributeTypeSXFOBJ) type;
			
			GByte scale = *(GByte *)(psRecordBuf + attributeOffset);
			attributeOffset += 1;
			attr.attributeScale = scale;
			
			attributes.insert(attr);
			
			switch(attr.attributeType)
			{
				case sctASCIIZ_DOS:
				{
					attributeOffset += scale + 1;
					break;
				}
				case sctOneByte:
				{
					attributeOffset += 1;
					break;
				}
				case sctTwoByte:
				{
					attributeOffset += 2;
					break;
				}
				case sctForeByte:
				{
					attributeOffset += 4;
					break;
				}
				case sctEightByte:
				{
					attributeOffset += 8;
					break;
				}
				case sctANSI_Windows:
				{
					attributeOffset += scale + 1;
					break;
				}
				case sctUNICODE_UNIX:
				{
					attributeOffset += scale + 1;
					break;
				}
				case sctBigString:
				{
					GUInt32 scale2 = *(GUInt32 *)(psRecordBuf+attributeOffset);
					attributeOffset += scale2;
					break;
				}
			}
			
		}
		
		std::map<GInt32, SetOfAttributesSXFOBJ>::iterator itLayer;
		itLayer = attributesTypesInLayers.find(oSXFObj.iCC);
		
		if ( itLayer != attributesTypesInLayers.end())
		{
			itLayer->second.insert(attributes.begin(),attributes.end());
		}
		else
		{
			attributesTypesInLayers.insert(std::pair<GInt32, SetOfAttributesSXFOBJ>(oSXFObj.iCC, attributes));

			objectOffset = VSIFTellL( fpSXF ) - oSXFObj.nRecordLength;

			firstObjectInLayers.insert(std::pair<GInt32, vsi_l_offset>(oSXFObj.iCC, objectOffset));			
		}
		
	}

/*---------------- Spatial index for all Layers --------------------*/
	OGRSpatialReference *poSRS(new OGRSpatialReference);
	OGRErr err = getOGRSpatialReference(oSXFSP, poSRS);

	/*
	char* pData(NULL);
	poSRS->exportToPrettyWkt(&pData);
	std::string message(pData);
	printf("OGRSpatialReference:\n %s\n",message);
	*/

/*---------------- Layers Creation ---------------------------------*/ 
	CPLError( CE_Warning, CPLE_None, 
                "SXF. Semantic attribute add as OFTString type.");

	nLayers = attributesTypesInLayers.size();

	papoLayers = (OGRLayer**) CPLMalloc(sizeof(OGRLayer*) * (nLayers));
	
	CPLString osLayerName;

	std::map<GInt32, SetOfAttributesSXFOBJ>::iterator itLayer;
	itLayer = attributesTypesInLayers.begin();

	for(size_t layerIndex = 0; layerIndex < nLayers; layerIndex++)
	{
		osLayerName.Printf("%d", itLayer->first);

		std::auto_ptr<OGRFeatureDefn> poFeatureDefn(new OGRFeatureDefn( osLayerName ));
		//poFeatureDefn->SetGeomType( wkbGeometryCollection );
		poFeatureDefn->Reference();
		
		OGRFieldDefn oCodeField = OGRFieldDefn( "OBJECTCODE", OFTInteger );
		oCodeField.SetWidth(10);
		poFeatureDefn->AddFieldDefn( &oCodeField );

		OGRFieldDefn oNumField = OGRFieldDefn( "OBJECTNUMB", OFTInteger );
		oNumField.SetWidth(10);
		poFeatureDefn->AddFieldDefn( &oNumField );

		OGRFieldDefn  oTextField( "TEXT", OFTString );
		oTextField.SetWidth(255);
		poFeatureDefn->AddFieldDefn( &oTextField );
		
		SetOfAttributesSXFOBJ attrs = itLayer->second;
		SetOfAttributesSXFOBJ::iterator itAttribute;

		for(itAttribute = attrs.begin(); itAttribute != attrs.end(); itAttribute++)
		{		
			CPLString oFieldName;
			oFieldName.Printf("%d", (*itAttribute).attributeCode);

			OGRFieldDefn oField = OGRFieldDefn(oFieldName, OFTString);
			oField.SetWidth(10);
			poFeatureDefn->AddFieldDefn(&oField);
		}
		
		papoLayers[layerIndex] = new OGRSXFLayer(fpSXF,osLayerName, poSRS, poFeatureDefn,
										oSXFSP, oSXFDSC, itLayer->first, 
										firstObjectInLayers[itLayer->first]);
		
		osLayerName.Clear();

		itLayer++;
	}

    return TRUE;
}