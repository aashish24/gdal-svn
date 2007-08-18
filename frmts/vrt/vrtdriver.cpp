/******************************************************************************
 * $Id$
 *
 * Project:  Virtual GDAL Datasets
 * Purpose:  Implementation of VRTDriver
 * Author:   Frank Warmerdam <warmerdam@pobox.com>
 *
 ******************************************************************************
 * Copyright (c) 2003, Frank Warmerdam <warmerdam@pobox.com>
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

#include "vrtdataset.h"
#include "cpl_minixml.h"
#include "cpl_string.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*                             VRTDriver()                              */
/************************************************************************/

VRTDriver::VRTDriver()

{
    papszSourceParsers = NULL;
}

/************************************************************************/
/*                             ~VRTDriver()                             */
/************************************************************************/

VRTDriver::~VRTDriver()

{
    CSLDestroy( papszSourceParsers );
}

/************************************************************************/
/*                            GetMetadata()                             */
/************************************************************************/

char **VRTDriver::GetMetadata( const char *pszDomain )

{
    if( pszDomain && EQUAL(pszDomain,"SourceParsers") )
        return papszSourceParsers;
    else
        return GDALDriver::GetMetadata( pszDomain );
}

/************************************************************************/
/*                            SetMetadata()                             */
/************************************************************************/

CPLErr VRTDriver::SetMetadata( char **papszMetadata, const char *pszDomain )

{
    if( pszDomain && EQUAL(pszDomain,"SourceParsers") )
    {
        CSLDestroy( papszSourceParsers );
        papszSourceParsers = CSLDuplicate( papszMetadata );
        return CE_None;
    }
    else
        return GDALDriver::SetMetadata( papszMetadata, pszDomain );
}

/************************************************************************/
/*                          AddSourceParser()                           */
/************************************************************************/

void VRTDriver::AddSourceParser( const char *pszElementName, 
                                 VRTSourceParser pfnParser )

{
    char szPtrValue[128];

    sprintf( szPtrValue, "%p", pfnParser );
    papszSourceParsers = CSLSetNameValue( papszSourceParsers, 
                                          pszElementName, szPtrValue );
}

/************************************************************************/
/*                            ParseSource()                             */
/************************************************************************/

VRTSource *VRTDriver::ParseSource( CPLXMLNode *psSrc, const char *pszVRTPath )

{
    const char *pszParserFunc;

    if( psSrc == NULL || psSrc->eType != CXT_Element )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Corrupt or empty VRT source XML document." );
        return NULL;
    }

    pszParserFunc = CSLFetchNameValue( papszSourceParsers, psSrc->pszValue );
    if( pszParserFunc == NULL )
        return NULL;

    VRTSourceParser pfnParser = NULL;

    sscanf( pszParserFunc, "%p", &pfnParser );

    if( pfnParser == NULL )
        return NULL;
    else
        return pfnParser( psSrc, pszVRTPath );
}

/************************************************************************/
/*                           VRTCreateCopy()                            */
/************************************************************************/

static GDALDataset *
VRTCreateCopy( const char * pszFilename, GDALDataset *poSrcDS, 
               int bStrict, char ** papszOptions, 
               GDALProgressFunc pfnProgress, void * pProgressData )

{
    VRTDataset *poVRTDS = NULL;

    (void) bStrict;
    (void) papszOptions;

    CPLAssert( NULL != poSrcDS );

/* -------------------------------------------------------------------- */
/*      If the source dataset is a virtual dataset then just write      */
/*      it to disk as a special case to avoid extra layers of           */
/*      indirection.                                                    */
/* -------------------------------------------------------------------- */
    if( EQUAL(poSrcDS->GetDriver()->GetDescription(),"VRT") )
    {

    /* -------------------------------------------------------------------- */
    /*      Convert tree to a single block of XML text.                     */
    /* -------------------------------------------------------------------- */
        char *pszVRTPath = CPLStrdup(CPLGetPath(pszFilename));
        CPLXMLNode *psDSTree = ((VRTDataset *) poSrcDS)->SerializeToXML( pszVRTPath );

        char *pszXML = CPLSerializeXMLTree( psDSTree );
        
        CPLDestroyXMLNode( psDSTree );

        CPLFree( pszVRTPath );
        
    /* -------------------------------------------------------------------- */
    /*      Write to disk.                                                  */
    /* -------------------------------------------------------------------- */
        GDALDataset* pCopyDS = NULL;

        if( 0 != strlen( pszFilename ) )
        {
            FILE *fpVRT = NULL;

            fpVRT = VSIFOpen( pszFilename, "w" );
            CPLAssert( NULL != fpVRT );

            VSIFWrite( pszXML, 1, strlen(pszXML), fpVRT );
            VSIFClose( fpVRT );

            pCopyDS = (GDALDataset *) GDALOpen( pszFilename, GA_Update );
        }
        else
        {
            /* No destination file is given, so pass serialized XML directly. */

            pCopyDS = (GDALDataset *) GDALOpen( pszXML, GA_Update );
        }

        CPLFree( pszXML );

        return pCopyDS;
    }

/* -------------------------------------------------------------------- */
/*      Create the virtual dataset.                                     */
/* -------------------------------------------------------------------- */
    poVRTDS = (VRTDataset *) 
        VRTDataset::Create( pszFilename, 
                            poSrcDS->GetRasterXSize(),
                            poSrcDS->GetRasterYSize(),
                            0, GDT_Byte, NULL );

/* -------------------------------------------------------------------- */
/*      Do we have a geotransform?                                      */
/* -------------------------------------------------------------------- */
    double adfGeoTransform[6];

    if( poSrcDS->GetGeoTransform( adfGeoTransform ) == CE_None )
    {
        poVRTDS->SetGeoTransform( adfGeoTransform );
    }

/* -------------------------------------------------------------------- */
/*      Copy projection                                                 */
/* -------------------------------------------------------------------- */
    poVRTDS->SetProjection( poSrcDS->GetProjectionRef() );

/* -------------------------------------------------------------------- */
/*      Emit dataset level metadata.                                    */
/* -------------------------------------------------------------------- */
    poVRTDS->SetMetadata( poSrcDS->GetMetadata() );

/* -------------------------------------------------------------------- */
/*      GCPs                                                            */
/* -------------------------------------------------------------------- */
    if( poSrcDS->GetGCPCount() > 0 )
    {
        poVRTDS->SetGCPs( poSrcDS->GetGCPCount(), 
                          poSrcDS->GetGCPs(),
                          poSrcDS->GetGCPProjection() );
    }

/* -------------------------------------------------------------------- */
/*      Loop over all the bands.					*/
/* -------------------------------------------------------------------- */
    for( int iBand = 0; iBand < poSrcDS->GetRasterCount(); iBand++ )
    {
        GDALRasterBand *poSrcBand = poSrcDS->GetRasterBand( iBand+1 );

/* -------------------------------------------------------------------- */
/*      Create the band with the appropriate band type.                 */
/* -------------------------------------------------------------------- */
        poVRTDS->AddBand( poSrcBand->GetRasterDataType(), NULL );

        VRTSourcedRasterBand *poVRTBand = 
            (VRTSourcedRasterBand *) poVRTDS->GetRasterBand( iBand+1 );

/* -------------------------------------------------------------------- */
/*      Setup source mapping.                                           */
/* -------------------------------------------------------------------- */
        poVRTBand->AddSimpleSource( poSrcBand );

/* -------------------------------------------------------------------- */
/*      Emit various band level metadata.                               */
/* -------------------------------------------------------------------- */
        poVRTBand->CopyCommonInfoFrom( poSrcBand );
    }

    poVRTDS->FlushCache();

    return poVRTDS;
}

/************************************************************************/
/*                          GDALRegister_VRT()                          */
/************************************************************************/

void GDALRegister_VRT()

{
    VRTDriver	*poDriver;

    if( GDALGetDriverByName( "VRT" ) == NULL )
    {
        poDriver = new VRTDriver();
        
        poDriver->SetDescription( "VRT" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "Virtual Raster" );
        poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "vrt" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, "gdal_vrttut.html" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONDATATYPES, 
                                   "Byte Int16 UInt16 Int32 UInt32 Float32 Float64 CInt16 CInt32 CFloat32 CFloat64" );
        
        poDriver->pfnOpen = VRTDataset::Open;
        poDriver->pfnCreateCopy = VRTCreateCopy;
        poDriver->pfnCreate = VRTDataset::Create;
        poDriver->pfnIdentify = VRTDataset::Identify;

        poDriver->AddSourceParser( "SimpleSource", 
                                   VRTParseCoreSources );
        poDriver->AddSourceParser( "ComplexSource", 
                                   VRTParseCoreSources );
        poDriver->AddSourceParser( "AveragedSource", 
                                   VRTParseCoreSources );

        poDriver->AddSourceParser( "KernelFilteredSource", 
                                   VRTParseFilterSources );

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}

