/******************************************************************************
 * $Id$
 *
 * Project:  ENVI .hdr Driver
 * Purpose:  Implementation of ENVI .hdr labelled raw raster support.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2002, Frank Warmerdam
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

#include "rawdataset.h"
#include "ogr_spatialref.h"
#include "cpl_string.h"

CPL_CVSID("$Id$");

CPL_C_START
void	GDALRegister_ENVI(void);
CPL_C_END

static int anUsgsEsriZones[] =
{
  101, 3101,
  102, 3126,
  201, 3151,
  202, 3176,
  203, 3201,
  301, 3226,
  302, 3251,
  401, 3276,
  402, 3301,
  403, 3326,
  404, 3351,
  405, 3376,
  406, 3401,
  407, 3426,
  501, 3451,
  502, 3476,
  503, 3501,
  600, 3526,
  700, 3551,
  901, 3601,
  902, 3626,
  903, 3576,
 1001, 3651,
 1002, 3676,
 1101, 3701,
 1102, 3726,
 1103, 3751,
 1201, 3776,
 1202, 3801,
 1301, 3826,
 1302, 3851,
 1401, 3876,
 1402, 3901,
 1501, 3926,
 1502, 3951,
 1601, 3976,
 1602, 4001,
 1701, 4026,
 1702, 4051,
 1703, 6426,
 1801, 4076,
 1802, 4101,
 1900, 4126,
 2001, 4151,
 2002, 4176,
 2101, 4201,
 2102, 4226,
 2103, 4251,
 2111, 6351,
 2112, 6376,
 2113, 6401,
 2201, 4276,
 2202, 4301,
 2203, 4326,
 2301, 4351,
 2302, 4376,
 2401, 4401,
 2402, 4426,
 2403, 4451,
 2500,    0,
 2501, 4476,
 2502, 4501,
 2503, 4526,
 2600,    0,
 2601, 4551,
 2602, 4576,
 2701, 4601,
 2702, 4626,
 2703, 4651,
 2800, 4676,
 2900, 4701,
 3001, 4726,
 3002, 4751,
 3003, 4776,
 3101, 4801,
 3102, 4826,
 3103, 4851,
 3104, 4876,
 3200, 4901,
 3301, 4926,
 3302, 4951,
 3401, 4976,
 3402, 5001,
 3501, 5026,
 3502, 5051,
 3601, 5076,
 3602, 5101,
 3701, 5126,
 3702, 5151,
 3800, 5176,
 3900,    0,
 3901, 5201,
 3902, 5226,
 4001, 5251,
 4002, 5276,
 4100, 5301,
 4201, 5326,
 4202, 5351,
 4203, 5376,
 4204, 5401,
 4205, 5426,
 4301, 5451,
 4302, 5476,
 4303, 5501,
 4400, 5526,
 4501, 5551,
 4502, 5576,
 4601, 5601,
 4602, 5626,
 4701, 5651,
 4702, 5676,
 4801, 5701,
 4802, 5726,
 4803, 5751,
 4901, 5776,
 4902, 5801,
 4903, 5826,
 4904, 5851,
 5001, 6101,
 5002, 6126,
 5003, 6151,
 5004, 6176,
 5005, 6201,
 5006, 6226,
 5007, 6251,
 5008, 6276,
 5009, 6301,
 5010, 6326,
 5101, 5876,
 5102, 5901,
 5103, 5926,
 5104, 5951,
 5105, 5976,
 5201, 6001,
 5200, 6026,
 5200, 6076,
 5201, 6051,
 5202, 6051,
 5300,    0, 
 5400,    0
};

/************************************************************************/
/*                           ESRIToUSGSZone()                           */
/*                                                                      */
/*      Convert ESRI style state plane zones to USGS style state        */
/*      plane zones.                                                    */
/************************************************************************/

static int ESRIToUSGSZone( int nESRIZone )

{
    int		nPairs = sizeof(anUsgsEsriZones) / (2*sizeof(int));
    int		i;
    
    for( i = 0; i < nPairs; i++ )
    {
        if( anUsgsEsriZones[i*2+1] == nESRIZone )
            return anUsgsEsriZones[i*2];
    }

    return nESRIZone; // perhaps it *is* the USGS zone?
}

/************************************************************************/
/* ==================================================================== */
/*				ENVIDataset				*/
/* ==================================================================== */
/************************************************************************/

class ENVIDataset : public RawDataset
{
    FILE	*fpImage;	// image data file.
    FILE	*fp;		// header file
    const char	*pszHDRFilename;

    int		bFoundMapinfo;

    int         bHeaderDirty;

    double      adfGeoTransform[6];

    char	*pszProjection;

    char        **papszHeader;

    int         ReadHeader( FILE * );
    int         ProcessMapinfo( const char * );
    void        SetENVIDatum( OGRSpatialReference *, const char * );
    void        SetENVIEllipse( OGRSpatialReference *, char ** );
    void        WriteProjectionInfo();
    
    char        **SplitList( const char * );

    enum Interleave { BSQ, BIL, BIP } interleave;
    static int GetEnviType(GDALDataType eType);

  public:
    		ENVIDataset();
    	        ~ENVIDataset();

    virtual void    FlushCache( void );
    virtual CPLErr  GetGeoTransform( double * padfTransform );
    virtual CPLErr  SetGeoTransform( double * );
    virtual const char *GetProjectionRef(void);
    virtual CPLErr  SetProjection( const char * );
    virtual char  **GetFileList(void);

    static GDALDataset *Open( GDALOpenInfo * );
    static GDALDataset *Create( const char * pszFilename,
                                int nXSize, int nYSize, int nBands,
                                GDALDataType eType, char ** papszOptions );
};

/************************************************************************/
/*                            ENVIDataset()                             */
/************************************************************************/

ENVIDataset::ENVIDataset()
{
    fpImage = NULL;
    fp = NULL;
    pszHDRFilename = NULL;
    pszProjection = CPLStrdup("");

    papszHeader = NULL;

    bFoundMapinfo = FALSE;

    bHeaderDirty = FALSE;

    adfGeoTransform[0] = 0.0;
    adfGeoTransform[1] = 1.0;
    adfGeoTransform[2] = 0.0;
    adfGeoTransform[3] = 0.0;
    adfGeoTransform[4] = 0.0;
    adfGeoTransform[5] = 1.0;
}

/************************************************************************/
/*                            ~ENVIDataset()                            */
/************************************************************************/

ENVIDataset::~ENVIDataset()

{
    FlushCache();
    if( fpImage )
        VSIFCloseL( fpImage );
    if( fp )
        VSIFClose( fp );
    if ( pszProjection )
	CPLFree( pszProjection );
    if ( papszHeader )
	CSLDestroy( papszHeader );
}

/************************************************************************/
/*                             FlushCache()                             */
/************************************************************************/

void ENVIDataset::FlushCache()

{
    RawDataset::FlushCache();

    if ( !bHeaderDirty )
        return;

    VSIFSeek( fp, 0, SEEK_SET );

/* -------------------------------------------------------------------- */
/*      Rewrite out the header.                                           */
/* -------------------------------------------------------------------- */
    int		iBigEndian;

    const char	*pszInterleaving;
    char** catNames;
    
#ifdef CPL_LSB
    iBigEndian = 0;
#else
    iBigEndian = 1;
#endif

    VSIFPrintf( fp, "ENVI\n" );
    if ("" != sDescription)
        VSIFPrintf( fp, "description = {\n%s}\n", sDescription.c_str());
    VSIFPrintf( fp, "samples = %d\nlines   = %d\nbands   = %d\n",
		nRasterXSize, nRasterYSize, nBands );

    GDALRasterBand* band = GetRasterBand(1);
    catNames = band->GetCategoryNames();
    VSIFPrintf( fp, "header offset = 0\n");
    if (0 == catNames)
        VSIFPrintf( fp, "file type = ENVI Standard\n" );
    else
        VSIFPrintf( fp, "file type = ENVI Classification\n" );
    int iENVIType = GetEnviType(band->GetRasterDataType());
    VSIFPrintf( fp, "data type = %d\n", iENVIType );
    switch (interleave)
    {
      case BIP:
        pszInterleaving = "bip";		    // interleaved by pixel
        break;
      case BIL:
        pszInterleaving = "bil";		    // interleaved by line
        break;
      case BSQ:
        pszInterleaving = "bsq";		// band sequental by default
        break;
      default:
    	pszInterleaving = "bsq";
        break;
    }
    VSIFPrintf( fp, "interleave = %s\n", pszInterleaving);
    VSIFPrintf( fp, "byte order = %d\n", iBigEndian );


/* -------------------------------------------------------------------- */
/*      Write class and color information                               */
/* -------------------------------------------------------------------- */
    catNames = band->GetCategoryNames();
    if (0 != catNames)
    {
        int nrClasses = 0;
        while (*catNames++)
            ++nrClasses;

        if (nrClasses > 0)
        {
            VSIFPrintf( fp, "classes = %d\n", nrClasses );

            GDALColorTable* colorTable = band->GetColorTable();
            if (0 != colorTable)
            {
                int nrColors = colorTable->GetColorEntryCount();
                if (nrColors > nrClasses)
                    nrColors = nrClasses;
                VSIFPrintf( fp, "class lookup = {\n");
                for (int i = 0; i < nrColors; ++i)
                {
                    const GDALColorEntry* color = colorTable->GetColorEntry(i);
                    VSIFPrintf(fp, "%d, %d, %d", color->c1, color->c2, color->c3);
                    if (i < nrColors - 1)
                    {
                        VSIFPrintf(fp, ", ");
                        if (0 == (i+1) % 5)
                            VSIFPrintf(fp, "\n");
                    }
                }
                VSIFPrintf(fp, "}\n");
            }

            catNames = band->GetCategoryNames();
            if (0 != *catNames)
            {
                VSIFPrintf( fp, "class names = {\n%s", *catNames++);
                int i = 0;
                while (*catNames) {
                    VSIFPrintf( fp, ",");
                    if (0 == (++i) % 5)
                        VSIFPrintf(fp, "\n");
                    VSIFPrintf( fp, " %s", *catNames++);
                }
                VSIFPrintf( fp, "}\n");
            }
        }
    }

/* -------------------------------------------------------------------- */
/*      Write the rest of header.                                       */
/* -------------------------------------------------------------------- */
    WriteProjectionInfo();
    VSIFPrintf( fp, "band names = {\n" );
    for ( int i = 1; i <= nBands; i++ )
    {
        std::string sBandDesc = GetRasterBand( i )->GetDescription();

        if ( sBandDesc == "" )
            sBandDesc = CPLSPrintf( "Band %d", i );
        VSIFPrintf( fp, "%s", sBandDesc.c_str() );
        if ( i != nBands )
            VSIFPrintf( fp, ",\n" );
    }
    VSIFPrintf( fp, "}\n" );
}

/************************************************************************/
/*                            GetFileList()                             */
/************************************************************************/

char **ENVIDataset::GetFileList()

{
    char **papszFileList = NULL;

    // Main data file, etc. 
    papszFileList = GDALPamDataset::GetFileList();

    // Header file.
    papszFileList = CSLAddString( papszFileList, pszHDRFilename );

    return papszFileList;
}

/************************************************************************/
/*                        WriteProjectionInfo()                         */
/************************************************************************/

void ENVIDataset::WriteProjectionInfo()

{
/* -------------------------------------------------------------------- */
/*      Format the location (geotransform) portion of the map info      */
/*      line.                                                           */
/* -------------------------------------------------------------------- */
    CPLString   osLocation;

    osLocation.Printf( "1, 1, %f, %f, %f, %f", 
                       adfGeoTransform[0], adfGeoTransform[3], 
                       adfGeoTransform[1], fabs(adfGeoTransform[5]) );
                       
/* -------------------------------------------------------------------- */
/*      Minimal case - write out simple geotransform if we have a       */
/*      non-default geotransform.                                       */
/* -------------------------------------------------------------------- */
    if( pszProjection == NULL || strlen(pszProjection) == 0 )
    {
        if( adfGeoTransform[0] != 0.0 || adfGeoTransform[1] != 1.0
            || adfGeoTransform[2] != 0.0 || adfGeoTransform[3] != 0.0
            || adfGeoTransform[4] != 0.0 || adfGeoTransform[5] != 1.0 )
        {
            const char* pszHemisphere = "North";
            VSIFPrintf( fp, "map info = {Unknown, %s, %d, %s}\n",
                        osLocation.c_str(), 0, pszHemisphere);
        }
        return;
    }

/* -------------------------------------------------------------------- */
/*      Ingest WKT.                                                     */
/* -------------------------------------------------------------------- */
    OGRSpatialReference oSRS;
    
    char	*pszProj = pszProjection;
    
    if( oSRS.importFromWkt( &pszProj ) != OGRERR_NONE )
        return;

/* -------------------------------------------------------------------- */
/*      Try to translate the datum and get major/minor ellipsoid        */
/*      values.                                                         */
/* -------------------------------------------------------------------- */
    int nEPSG_GCS = oSRS.GetEPSGGeogCS();
    CPLString osDatum, osCommaDatum;
    double dfA, dfB;

    if( nEPSG_GCS == 4326 )
        osDatum = "WGS-84";
    else if( nEPSG_GCS == 4322 )
        osDatum = "WGS-72";
    else if( nEPSG_GCS == 4269 )
        osDatum = "North America 1983";
    else if( nEPSG_GCS == 4267 )
        osDatum = "North America 1927";
    else if( nEPSG_GCS == 4230 )
        osDatum = "European 1950";
    else if( nEPSG_GCS == 4277 )
        osDatum = "Ordnance Survey of Great Britain '36";
    else if( nEPSG_GCS == 4291 )
        osDatum = "SAD-69/Brazil";
    else if( nEPSG_GCS == 4283 )
        osDatum = "Geocentric Datum of Australia 1994";
    else if( nEPSG_GCS == 4275 )
        osDatum = "Nouvelle Triangulation Francaise IGN";

    if( osDatum != "" )
        osCommaDatum.Printf( ",%s", osDatum.c_str() );

    dfA = oSRS.GetSemiMajor();
    dfB = oSRS.GetSemiMinor();

/* -------------------------------------------------------------------- */
/*      Do we have unusual linear units?                                */
/* -------------------------------------------------------------------- */
    CPLString osOptionalUnits;
    if( fabs(oSRS.GetLinearUnits()-0.3048) < 0.0001 )
        osOptionalUnits = ", units=Feet";

/* -------------------------------------------------------------------- */
/*      Handle UTM case.                                                */
/* -------------------------------------------------------------------- */
    const char	*pszHemisphere;
    const char  *pszProjName = oSRS.GetAttrValue("PROJECTION");
    int		bNorth;
    int		iUTMZone;

    iUTMZone = oSRS.GetUTMZone( &bNorth );
    if ( iUTMZone )
    {
        if ( bNorth )
            pszHemisphere = "North";
        else
            pszHemisphere = "South";

        VSIFPrintf( fp, "map info = {UTM, %s, %d, %s%s%s}\n",
                    osLocation.c_str(), iUTMZone, pszHemisphere,
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );
    }
    else if( oSRS.IsGeographic() )
    {
        VSIFPrintf( fp, "map info = {Geographic Lat/Lon, %s%s}\n",
                    osLocation.c_str(), osCommaDatum.c_str());
    }
    else if( pszProjName == NULL )
    {
        // what to do? 
    }
    else if( EQUAL(pszProjName,SRS_PT_NEW_ZEALAND_MAP_GRID) )
    {
        VSIFPrintf( fp, "map info = {New Zealand Map Grid, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {39, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, New Zealand Map Grid}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_TRANSVERSE_MERCATOR) )
    {
        VSIFPrintf( fp, "map info = {Transverse Mercator, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {3, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Transverse Mercator}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_SCALE_FACTOR,1.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_LAMBERT_CONFORMAL_CONIC_2SP)
         || EQUAL(pszProjName,SRS_PT_LAMBERT_CONFORMAL_CONIC_2SP_BELGIUM) )
    {
        VSIFPrintf( fp, "map info = {Lambert Conformal Conic, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {4, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Lambert Conformal Conic}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_STANDARD_PARALLEL_1,0.0),
                    oSRS.GetNormProjParm(SRS_PP_STANDARD_PARALLEL_2,0.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,
                   SRS_PT_HOTINE_OBLIQUE_MERCATOR_TWO_POINT_NATURAL_ORIGIN) )
    {
        VSIFPrintf( fp, "map info = {Hotine Oblique Mercator A, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {5, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Hotine Oblique Mercator A}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_POINT_1,0.0),
                    oSRS.GetNormProjParm(SRS_PP_LONGITUDE_OF_POINT_1,0.0),
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_POINT_2,0.0),
                    oSRS.GetNormProjParm(SRS_PP_LONGITUDE_OF_POINT_2,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_SCALE_FACTOR,1.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_HOTINE_OBLIQUE_MERCATOR) )
    {
        VSIFPrintf( fp, "map info = {Hotine Oblique Mercator B, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {6, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Hotine Oblique Mercator B}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_AZIMUTH,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_SCALE_FACTOR,1.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_STEREOGRAPHIC) 
             || EQUAL(pszProjName,SRS_PT_OBLIQUE_STEREOGRAPHIC) )
    {
        VSIFPrintf( fp, "map info = {Stereographic (ellipsoid), %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {7, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Stereographic (ellipsoid)}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_SCALE_FACTOR,1.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_ALBERS_CONIC_EQUAL_AREA) )
    {
        VSIFPrintf( fp, "map info = {Albers Conical Equal Area, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {9, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Albers Conical Equal Area}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_STANDARD_PARALLEL_1,0.0),
                    oSRS.GetNormProjParm(SRS_PP_STANDARD_PARALLEL_2,0.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_POLYCONIC) )
    {
        VSIFPrintf( fp, "map info = {Polyconic, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {10, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Polyconic}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_LAMBERT_AZIMUTHAL_EQUAL_AREA) )
    {
        VSIFPrintf( fp, "map info = {Lambert Azimuthal Equal Area, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {11, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Lambert Azimuthal Equal Area}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_AZIMUTHAL_EQUIDISTANT) )
    {
        VSIFPrintf( fp, "map info = {Azimuthal Equadistant, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {12, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Azimuthal Equadistant}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    osCommaDatum.c_str() );
    }
    else if( EQUAL(pszProjName,SRS_PT_POLAR_STEREOGRAPHIC) )
    {
        VSIFPrintf( fp, "map info = {Polar Stereographic, %s%s%s}\n",
                    osLocation.c_str(), 
                    osCommaDatum.c_str(), osOptionalUnits.c_str() );

        VSIFPrintf( fp, "projection info = {31, %.16g, %.16g, %.16g, %.16g, %.16g, %.16g%s, Polar Stereographic}\n",
                    dfA, dfB, 
                    oSRS.GetNormProjParm(SRS_PP_LATITUDE_OF_ORIGIN,90.0),
                    oSRS.GetNormProjParm(SRS_PP_CENTRAL_MERIDIAN,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_EASTING,0.0),
                    oSRS.GetNormProjParm(SRS_PP_FALSE_NORTHING,0.0),
                    osCommaDatum.c_str() );
    }
    else
    {
        VSIFPrintf( fp, "map info = {%s, %s}\n",
                    pszProjName, osLocation.c_str());
    }
}


/************************************************************************/
/*                          GetProjectionRef()                          */
/************************************************************************/

const char *ENVIDataset::GetProjectionRef()

{
    return pszProjection;
}

/************************************************************************/
/*                          SetProjection()                             */
/************************************************************************/

CPLErr ENVIDataset::SetProjection( const char *pszNewProjection )

{
    if ( pszProjection )
	CPLFree( pszProjection );
    pszProjection = CPLStrdup( pszNewProjection );

    bHeaderDirty = TRUE;

    return CE_None;
}

/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr ENVIDataset::GetGeoTransform( double * padfTransform )

{
    memcpy( padfTransform, adfGeoTransform, sizeof(double) * 6 );
    
    if( bFoundMapinfo )
        return CE_None;
    else
        return CE_Failure;
}

/************************************************************************/
/*                          SetGeoTransform()                           */
/************************************************************************/

CPLErr ENVIDataset::SetGeoTransform( double * padfTransform )
{
    memcpy( adfGeoTransform, padfTransform, sizeof(double) * 6 );

    bHeaderDirty = TRUE;
    
    return CE_None;
}

/************************************************************************/
/*                             SplitList()                              */
/*                                                                      */
/*      Split an ENVI value list into component fields, and strip       */
/*      white space.                                                    */
/************************************************************************/

char **ENVIDataset::SplitList( const char *pszCleanInput )

{
    char	**papszReturn = NULL;
    char	*pszInput = CPLStrdup(pszCleanInput);

    if( pszInput[0] != '{' )
        return NULL;

    int iChar=1;


    while( pszInput[iChar] != '}' && pszInput[iChar] != '\0' )
    {
        int iFStart=-1, iFEnd=-1;

        // Find start of token.
        iFStart = iChar;
        while( pszInput[iFStart] == ' ' )
            iFStart++;

        iFEnd = iFStart;
        while( pszInput[iFEnd] != ',' 
               && pszInput[iFEnd] != '}'
               && pszInput[iFEnd] != '\0' )
            iFEnd++;

        if( pszInput[iFEnd] == '\0' )
            break;

        iChar = iFEnd + 1;
        iFEnd = iFEnd - 1;

        while( iFEnd > iFStart && pszInput[iFEnd] == ' ' )
            iFEnd--;

        pszInput[iFEnd + 1] = '\0';
        papszReturn = CSLAddString( papszReturn, pszInput + iFStart );
    }

    CPLFree( pszInput );

    return papszReturn;
}

/************************************************************************/
/*                            SetENVIDatum()                            */
/************************************************************************/

void ENVIDataset::SetENVIDatum( OGRSpatialReference *poSRS, 
                                const char *pszENVIDatumName )

{
    // datums
    if( EQUAL(pszENVIDatumName, "WGS-84") )
        poSRS->SetWellKnownGeogCS( "WGS84" );
    else if( EQUAL(pszENVIDatumName, "WGS-72") )
        poSRS->SetWellKnownGeogCS( "WGS72" );
    else if( EQUAL(pszENVIDatumName, "North America 1983") )
        poSRS->SetWellKnownGeogCS( "NAD83" );
    else if( EQUAL(pszENVIDatumName, "North America 1927") 
             || strstr(pszENVIDatumName,"NAD27") 
             || strstr(pszENVIDatumName,"NAD-27") )
        poSRS->SetWellKnownGeogCS( "NAD27" );
    else if( EQUALN(pszENVIDatumName, "European 1950",13) )
        poSRS->SetWellKnownGeogCS( "EPSG:4230" );
    else if( EQUAL(pszENVIDatumName, "Ordnance Survey of Great Britain '36") )
        poSRS->SetWellKnownGeogCS( "EPSG:4277" );
    else if( EQUAL(pszENVIDatumName, "SAD-69/Brazil") )
        poSRS->SetWellKnownGeogCS( "EPSG:4291" );
    else if( EQUAL(pszENVIDatumName, "Geocentric Datum of Australia 1994") )
        poSRS->SetWellKnownGeogCS( "EPSG:4283" );
    else if( EQUAL(pszENVIDatumName, "Nouvelle Triangulation Francaise IGN") )
        poSRS->SetWellKnownGeogCS( "EPSG:4275" );

    // Ellipsoids
    else if( EQUAL(pszENVIDatumName, "GRS 80") )
        poSRS->SetWellKnownGeogCS( "NAD83" );
    else if( EQUAL(pszENVIDatumName, "Airy") )
        poSRS->SetWellKnownGeogCS( "EPSG:4001" );
    else if( EQUAL(pszENVIDatumName, "Australian National") )
        poSRS->SetWellKnownGeogCS( "EPSG:4003" );
    else if( EQUAL(pszENVIDatumName, "Bessel 1841") )
        poSRS->SetWellKnownGeogCS( "EPSG:4004" );
    else if( EQUAL(pszENVIDatumName, "Clark 1866") )
        poSRS->SetWellKnownGeogCS( "EPSG:4008" );
    else 
    {
        CPLError( CE_Warning, CPLE_AppDefined,
                  "Unrecognised datum '%s', defaulting to WGS84." );
        poSRS->SetWellKnownGeogCS( "WGS84" );
    }
}

/************************************************************************/
/*                           SetENVIEllipse()                           */
/************************************************************************/

void ENVIDataset::SetENVIEllipse( OGRSpatialReference *poSRS, 
                                  char **papszPI_EI )

{
    double dfA = CPLAtofM(papszPI_EI[0]);
    double dfB = CPLAtofM(papszPI_EI[1]);
    double dfInvF;

    if( fabs(dfA-dfB) < 0.1 )
        dfInvF = 0.0; // sphere
    else
        dfInvF = dfA / (dfA - dfB);
    
    
    poSRS->SetGeogCS( "Ellipse Based", "Ellipse Based", "Unnamed", 
                      dfA, dfInvF );
}

/************************************************************************/
/*                           ProcessMapinfo()                           */
/*                                                                      */
/*      Extract projection, and geotransform from a mapinfo value in    */
/*      the header.                                                     */
/************************************************************************/

int ENVIDataset::ProcessMapinfo( const char *pszMapinfo )

{
    char	**papszFields;		
    int         nCount;
    OGRSpatialReference oSRS;

    papszFields = SplitList( pszMapinfo );
    nCount = CSLCount(papszFields);

    if( nCount < 7 )
    {
        CSLDestroy( papszFields );
        return FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Check if we have projection info, and if so parse it.           */
/* -------------------------------------------------------------------- */
    char        **papszPI = NULL;
    int         nPICount = 0;
    if( CSLFetchNameValue( papszHeader, "projection_info" ) != NULL )
    {
        papszPI = SplitList( 
            CSLFetchNameValue( papszHeader, "projection_info" ) );
        nPICount = CSLCount(papszPI);
    }

/* -------------------------------------------------------------------- */
/*      Capture geotransform.                                           */
/* -------------------------------------------------------------------- */
    adfGeoTransform[1] = atof(papszFields[5]);	    // Pixel width
    adfGeoTransform[5] = -atof(papszFields[6]);	    // Pixel height
    adfGeoTransform[0] =			    // Upper left X coordinate
	atof(papszFields[3]) - (atof(papszFields[1]) - 1) * adfGeoTransform[1];
    adfGeoTransform[3] =			    // Upper left Y coordinate
	atof(papszFields[4]) - (atof(papszFields[2]) - 1) * adfGeoTransform[5];
    adfGeoTransform[2] = 0.0;
    adfGeoTransform[4] = 0.0;

/* -------------------------------------------------------------------- */
/*      Capture projection.                                             */
/* -------------------------------------------------------------------- */
    if( EQUALN(papszFields[0],"UTM",3) && nCount >= 9 )
    {
        oSRS.SetUTM( atoi(papszFields[7]), 
                     !EQUAL(papszFields[8],"South") );
        if( nCount >= 10 && strstr(papszFields[9],"=") == NULL )
            SetENVIDatum( &oSRS, papszFields[9] );
        else
            oSRS.SetWellKnownGeogCS( "WGS84" );
    }
    else if( EQUALN(papszFields[0],"State Plane (NAD 27)",19)
             && nCount >= 7 )
    {
        oSRS.SetStatePlane( ESRIToUSGSZone(atoi(papszFields[7])), FALSE );
    }
    else if( EQUALN(papszFields[0],"State Plane (NAD 83)",19)
             && nCount >= 7 )
    {
        oSRS.SetStatePlane( ESRIToUSGSZone(atoi(papszFields[7])), TRUE );
    }
    else if( EQUALN(papszFields[0],"Geographic Lat",14) 
             && nCount >= 8 )
    {
        if( nCount >= 8 && strstr(papszFields[7],"=") == NULL )
            SetENVIDatum( &oSRS, papszFields[7] );
        else
            oSRS.SetWellKnownGeogCS( "WGS84" );
    }
    else if( nPICount > 8 && atoi(papszPI[0]) == 3 ) // TM
    {
        oSRS.SetTM( CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]),
                    CPLAtofM(papszPI[7]),
                    CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }
    else if( nPICount > 8 && atoi(papszPI[0]) == 4 ) // Lambert Conformal Conic
    {
        oSRS.SetLCC( CPLAtofM(papszPI[7]), CPLAtofM(papszPI[8]),
                     CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]),
                     CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }
    else if( nPICount > 10 && atoi(papszPI[0]) == 5 ) // Oblique Merc (2 point)
    {
        oSRS.SetHOM2PNO( CPLAtofM(papszPI[3]), 
                         CPLAtofM(papszPI[4]), CPLAtofM(papszPI[5]), 
                         CPLAtofM(papszPI[6]), CPLAtofM(papszPI[7]), 
                         CPLAtofM(papszPI[10]), 
                         CPLAtofM(papszPI[8]), CPLAtofM(papszPI[9]) );
    }
    else if( nPICount > 8 && atoi(papszPI[0]) == 6 ) // Oblique Merc 
    {
        oSRS.SetHOM(CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]), 
                    CPLAtofM(papszPI[5]), 0.0,
                    CPLAtofM(papszPI[8]), 
                    CPLAtofM(papszPI[6]), CPLAtofM(papszPI[7]) );
    }
    else if( nPICount > 8 && atoi(papszPI[0]) == 7 ) // Stereographic
    {
        oSRS.SetStereographic( CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]),
                               CPLAtofM(papszPI[7]),
                               CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }
    else if( nPICount > 8 && atoi(papszPI[0]) == 9 ) // Albers Equal Area
    {
        oSRS.SetACEA( CPLAtofM(papszPI[7]), CPLAtofM(papszPI[8]),
                      CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]),
                      CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }
    else if( nPICount > 6 && atoi(papszPI[0]) == 10 ) // Polyconic
    {
        oSRS.SetPolyconic(CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]), 
                          CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }
    else if( nPICount > 6 && atoi(papszPI[0]) == 11 ) // LAEA
    {
        oSRS.SetLAEA(CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]), 
                     CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }
    else if( nPICount > 6 && atoi(papszPI[0]) == 12 ) // Azimuthal Equid.
    {
        oSRS.SetAE(CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]), 
                   CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }
    else if( nPICount > 6 && atoi(papszPI[0]) == 31 ) // Polar Stereographic
    {
        oSRS.SetPS(CPLAtofM(papszPI[3]), CPLAtofM(papszPI[4]), 
                   1.0,
                   CPLAtofM(papszPI[5]), CPLAtofM(papszPI[6]) );
    }

    // Still lots more that could be added for someone with the patience.

/* -------------------------------------------------------------------- */
/*      fallback to localcs if we don't recognise things.               */
/* -------------------------------------------------------------------- */
    if( oSRS.GetRoot() == NULL )
        oSRS.SetLocalCS( papszFields[0] );

/* -------------------------------------------------------------------- */
/*      Try to set datum from projection info line if we have a         */
/*      projected coordinate system without a GEOGCS.                   */
/* -------------------------------------------------------------------- */
    if( oSRS.IsProjected() && oSRS.GetAttrNode("GEOGCS") == NULL 
        && nPICount > 3 )
    {
        // Do we have a datum on the projection info line?
        int iDatum = nPICount-1;

        // Ignore units= items.
        if( strstr(papszPI[iDatum],"=") != NULL )
            iDatum--;

        // Skip past the name. 
        iDatum--;
    
        CPLString osDatumName = papszPI[iDatum];
        if( osDatumName.find_first_of("abcdefghijklmnopqrstuvwxyz"
                                      "ABCDEFGHIJKLMNOPQRSTUVWXYZ") 
                                      != std::string::npos )
        {
            SetENVIDatum( &oSRS, osDatumName );
        }
        else
        {
            SetENVIEllipse( &oSRS, papszPI + 1 );
        }
    }


/* -------------------------------------------------------------------- */
/*      Try to process specialized units.                               */
/* -------------------------------------------------------------------- */
    if( EQUAL(papszFields[nCount-1],"units=Feet") )
    {
        oSRS.SetLinearUnits( SRS_UL_US_FOOT, atof(SRS_UL_US_FOOT_CONV) );
    }
    else if( EQUAL(papszFields[nCount-1],"units=Seconds") 
             && oSRS.IsGeographic() )
    {
        /* convert geographic coordinate systems in seconds to degrees */
        adfGeoTransform[0] /= 3600.0;
        adfGeoTransform[1] /= 3600.0;
        adfGeoTransform[2] /= 3600.0;
        adfGeoTransform[3] /= 3600.0;
        adfGeoTransform[4] /= 3600.0;
        adfGeoTransform[5] /= 3600.0;
    }

/* -------------------------------------------------------------------- */
/*      Turn back into WKT.                                             */
/* -------------------------------------------------------------------- */
    if( oSRS.GetRoot() != NULL )
    {
        oSRS.Fixup();
	if ( pszProjection )
	{
	    CPLFree( pszProjection );
	    pszProjection = NULL;
	}
        oSRS.exportToWkt( &pszProjection );
    }

    CSLDestroy( papszFields );
    CSLDestroy( papszPI );
    return TRUE;
}

/************************************************************************/
/*                             ReadHeader()                             */
/************************************************************************/

int ENVIDataset::ReadHeader( FILE * fpHdr )

{

    CPLReadLine( fpHdr );

/* -------------------------------------------------------------------- */
/*      Now start forming sets of name/value pairs.                     */
/* -------------------------------------------------------------------- */
    while( TRUE )
    {
        const char *pszNewLine;
        char       *pszWorkingLine;

        pszNewLine = CPLReadLine( fpHdr );
        if( pszNewLine == NULL )
            break;

        if( strstr(pszNewLine,"=") == NULL )
            continue;

        pszWorkingLine = CPLStrdup(pszNewLine);

        // Collect additional lines if we have open sqiggly bracket.
        if( strstr(pszWorkingLine,"{") != NULL 
            && strstr(pszWorkingLine,"}") == NULL )
        {
            do { 
                pszNewLine = CPLReadLine( fpHdr );
                if( pszNewLine )
                {
                    pszWorkingLine = (char *) 
                        CPLRealloc(pszWorkingLine, 
                                 strlen(pszWorkingLine)+strlen(pszNewLine)+1);
                    strcat( pszWorkingLine, pszNewLine );
                }
            } while( pszNewLine != NULL && strstr(pszNewLine,"}") == NULL );
        }

        // Try to break input into name and value portions.  Trim whitespace.
        const char *pszValue;
        int         iEqual;

        for( iEqual = 0; 
             pszWorkingLine[iEqual] != '\0' && pszWorkingLine[iEqual] != '=';
             iEqual++ ) {}

        if( pszWorkingLine[iEqual] == '=' )
        {
            int		i;

            pszValue = pszWorkingLine + iEqual + 1;
            while( *pszValue == ' ' )
                pszValue++;
            
            pszWorkingLine[iEqual--] = '\0';
            while( iEqual > 0 && pszWorkingLine[iEqual] == ' ' )
                pszWorkingLine[iEqual--] = '\0';

            // Convert spaces in the name to underscores.
            for( i = 0; pszWorkingLine[i] != '\0'; i++ )
            {
                if( pszWorkingLine[i] == ' ' )
                    pszWorkingLine[i] = '_';
            }

            papszHeader = CSLSetNameValue( papszHeader, 
                                           pszWorkingLine, pszValue );
        }

        CPLFree( pszWorkingLine );
    }

    return TRUE;
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *ENVIDataset::Open( GDALOpenInfo * poOpenInfo )

{
    int		i;
    
/* -------------------------------------------------------------------- */
/*	We assume the user is pointing to the binary (ie. .bil) file.	*/
/* -------------------------------------------------------------------- */
    if( poOpenInfo->nHeaderBytes < 2 )
        return NULL;

/* -------------------------------------------------------------------- */
/*      Do we have a .hdr file?  Try upper and lower case, and          */
/*      replacing the extension as well as appending the extension      */
/*      to whatever we currently have.                                  */
/* -------------------------------------------------------------------- */
    const char	*pszMode;
    CPLString   osHdrFilename;
    FILE	*fpHeader;

    if( poOpenInfo->eAccess == GA_Update )
	pszMode = "r+";
    else
	pszMode = "r";

    osHdrFilename = CPLResetExtension( poOpenInfo->pszFilename, "hdr" );
    fpHeader = VSIFOpen( osHdrFilename, pszMode );

#ifndef WIN32
    if( fpHeader == NULL )
    {
        osHdrFilename = CPLResetExtension( poOpenInfo->pszFilename, "HDR" );
        fpHeader = VSIFOpen( osHdrFilename, pszMode );
    }
#endif
    if( fpHeader == NULL )
    {
        osHdrFilename = CPLFormFilename( NULL, poOpenInfo->pszFilename, 
                                          "hdr" );
        fpHeader = VSIFOpen( osHdrFilename, pszMode );
    }
#ifndef WIN32
    if( fpHeader == NULL )
    {
        osHdrFilename = CPLFormFilename( NULL, poOpenInfo->pszFilename, 
                                          "HDR" );
        fpHeader = VSIFOpen( osHdrFilename, pszMode );
    }
#endif

    if( fpHeader == NULL )
        return NULL;
    
/* -------------------------------------------------------------------- */
/*      Check that the first line says "ENVI".                          */
/* -------------------------------------------------------------------- */
    char	szTestHdr[4];

    if( VSIFRead( szTestHdr, 4, 1, fpHeader ) != 1 )
    {
        VSIFClose( fpHeader );
        return NULL;
    }
    if( strncmp(szTestHdr,"ENVI",4) != 0 )
    {
        VSIFClose( fpHeader );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */
    ENVIDataset 	*poDS;

    poDS = new ENVIDataset();
    poDS->pszHDRFilename = CPLStrdup(osHdrFilename);
    poDS->fp = fpHeader;

/* -------------------------------------------------------------------- */
/*      Read the header.                                                */
/* -------------------------------------------------------------------- */
    if( !poDS->ReadHeader( fpHeader ) )
    {
        delete poDS;
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Has the user selected the .hdr file to open?                    */
/* -------------------------------------------------------------------- */
    if( EQUAL(CPLGetExtension(poOpenInfo->pszFilename), "hdr") )
    {
        delete poDS;
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "The selected file is an ENVI header file, but to\n"
                  "open ENVI datasets, the data file should be selected\n"
                  "instead of the .hdr file.  Please try again selecting\n"
                  "the data file corresponding to the header file:\n"
                  "  %s\n", 
                  poOpenInfo->pszFilename );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Extract required values from the .hdr.                          */
/* -------------------------------------------------------------------- */
    int	nLines = 0, nSamples = 0, nBands = 0, nHeaderSize = 0;
    const char   *pszInterleave = NULL;

    if( CSLFetchNameValue(poDS->papszHeader,"lines") )
        nLines = atoi(CSLFetchNameValue(poDS->papszHeader,"lines"));

    if( CSLFetchNameValue(poDS->papszHeader,"samples") )
        nSamples = atoi(CSLFetchNameValue(poDS->papszHeader,"samples"));

    if( CSLFetchNameValue(poDS->papszHeader,"bands") )
        nBands = atoi(CSLFetchNameValue(poDS->papszHeader,"bands"));

    pszInterleave = CSLFetchNameValue(poDS->papszHeader,"interleave");

    if( nLines == 0 || nSamples == 0 || nBands == 0 || pszInterleave == NULL )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "The file appears to have an associated ENVI header, but\n"
                  "one or more of the samples, lines, bands and interleave\n"
                  "keywords appears to be missing." );
        return NULL;
    }

    if( CSLFetchNameValue(poDS->papszHeader,"header_offset") )
        nHeaderSize = atoi(CSLFetchNameValue(poDS->papszHeader,"header_offset"));

/* -------------------------------------------------------------------- */
/*      Translate the datatype.                                         */
/* -------------------------------------------------------------------- */
    GDALDataType	eType = GDT_Byte;

    if( CSLFetchNameValue(poDS->papszHeader,"data_type" ) != NULL )
    {
        switch( atoi(CSLFetchNameValue(poDS->papszHeader,"data_type" )) )
        {
          case 1:
            eType = GDT_Byte;
            break;

          case 2:
            eType = GDT_Int16;
            break;

          case 3:
            eType = GDT_Int32;
            break;

          case 4:
            eType = GDT_Float32;
            break;

          case 5:
            eType = GDT_Float64;
            break;

          case 6:
            eType = GDT_CFloat32;
            break;

          case 9:
            eType = GDT_CFloat64;
            break;

          case 12:
            eType = GDT_UInt16;
            break;

          case 13:
            eType = GDT_UInt32;
            break;

            /* 14=Int64, 15=UInt64 */

          default:
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "The file has a 'data type' value of '%s'.  This value\n"
                      "isn't recognised by the GDAL ENVI driver.",
                      CSLFetchNameValue(poDS->papszHeader,"data_type" ) );
            return NULL;
        }
    }

/* -------------------------------------------------------------------- */
/*      Translate the byte order.					*/
/* -------------------------------------------------------------------- */
    int		bNativeOrder = TRUE;

    if( CSLFetchNameValue(poDS->papszHeader,"byte_order" ) != NULL )
    {
#ifdef CPL_LSB                               
        bNativeOrder = atoi(CSLFetchNameValue(poDS->papszHeader,
                                              "byte_order" )) == 0;
#else
        bNativeOrder = atoi(CSLFetchNameValue(poDS->papszHeader,
                                              "byte_order" )) != 0;
#endif
    }

/* -------------------------------------------------------------------- */
/*      Warn about compressed datasets.                                 */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue(poDS->papszHeader,"file_compression" ) != NULL )
    {
        if( atoi(CSLFetchNameValue(poDS->papszHeader,"file_compression" )) 
            != 0 )
        {
            CPLError( CE_Warning, CPLE_AppDefined, 
                      "File %s is marked as compressed in the ENVI .hdr\n"
                      "GDAL does not support auto-decompression of ENVI data\n"
                      "files.  If the data appears corrupt please decompress\n"
                      "manually and then retry.",
                      poOpenInfo->pszFilename );
        }
    }

/* -------------------------------------------------------------------- */
/*      Capture some information from the file that is of interest.     */
/* -------------------------------------------------------------------- */
    poDS->nRasterXSize = nSamples;
    poDS->nRasterYSize = nLines;
    poDS->eAccess = poOpenInfo->eAccess;

/* -------------------------------------------------------------------- */
/*      Reopen file in update mode if necessary.                        */
/* -------------------------------------------------------------------- */
    if( poOpenInfo->eAccess == GA_Update )
        poDS->fpImage = VSIFOpenL( poOpenInfo->pszFilename, "rb+" );
    else
        poDS->fpImage = VSIFOpenL( poOpenInfo->pszFilename, "rb" );

    if( poDS->fpImage == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "Failed to re-open %s within ENVI driver.\n", 
                  poOpenInfo->pszFilename );
	delete poDS;
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Compute the line offset.                                        */
/* -------------------------------------------------------------------- */
    int	nDataSize = GDALGetDataTypeSize(eType)/8;
    int nPixelOffset, nLineOffset;
    vsi_l_offset nBandOffset;
    
    if( EQUALN(pszInterleave, "bsq", 3) )
    {
        poDS->interleave = BSQ;
        nLineOffset = nDataSize * nSamples;
        nPixelOffset = nDataSize;
        nBandOffset = (vsi_l_offset)nLineOffset * nLines;
    }
    else if( EQUALN(pszInterleave, "bil", 3) )
    {
        poDS->interleave = BIL;
        nLineOffset = nDataSize * nSamples * nBands;
        nPixelOffset = nDataSize;
        nBandOffset = (vsi_l_offset)nDataSize * nSamples;
    }
    else if( EQUALN(pszInterleave, "bip", 3) )
    {
        poDS->interleave = BIP;
        nLineOffset = nDataSize * nSamples * nBands;
        nPixelOffset = nDataSize * nBands;
        nBandOffset = nDataSize;
    }
    else
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "The interleaving type of the file (%s) is not supported.",
                  pszInterleave );
        return NULL;
    }
    
/* -------------------------------------------------------------------- */
/*      Create band information objects.                                */
/* -------------------------------------------------------------------- */
    poDS->nBands = nBands;
    for( i = 0; i < poDS->nBands; i++ )
    {
        poDS->SetBand( i + 1,
            new RawRasterBand(poDS, i + 1, poDS->fpImage,
                              nHeaderSize + nBandOffset * i,
                              nPixelOffset, nLineOffset, eType,
			      bNativeOrder, TRUE) );
    }

/* -------------------------------------------------------------------- */
/*      Apply band names if we have them.                               */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue( poDS->papszHeader, "band_names" ) != NULL )
    {
        char	**papszBandNames = 
            poDS->SplitList( CSLFetchNameValue( poDS->papszHeader, 
                                                "band_names" ) );

        for( i = 0; i < MIN(CSLCount(papszBandNames),nBands); i++ )
            poDS->GetRasterBand(i + 1)->SetDescription( papszBandNames[i] );

        CSLDestroy( papszBandNames );
    }
    
/* -------------------------------------------------------------------- */
/*      Apply class names if we have them.                              */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue( poDS->papszHeader, "class_names" ) != NULL )
    {
        char	**papszClassNames = 
            poDS->SplitList( CSLFetchNameValue( poDS->papszHeader, 
                                                "class_names" ) );

        poDS->GetRasterBand(1)->SetCategoryNames( papszClassNames );
        CSLDestroy( papszClassNames );
    }
    
/* -------------------------------------------------------------------- */
/*      Apply colormap if we have one.					*/
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue( poDS->papszHeader, "class_lookup" ) != NULL )
    {
        char	**papszClassColors = 
            poDS->SplitList( CSLFetchNameValue( poDS->papszHeader, 
                                                "class_lookup" ) );
        int nColorValueCount = CSLCount(papszClassColors);
        GDALColorTable oCT;

        for( i = 0; i*3 < nColorValueCount; i++ )
        {
            GDALColorEntry sEntry;

            sEntry.c1 = atoi(papszClassColors[i*3+0]);
            sEntry.c2 = atoi(papszClassColors[i*3+1]);
            sEntry.c3 = atoi(papszClassColors[i*3+2]);
            sEntry.c4 = 255;
            oCT.SetColorEntry( i, &sEntry );
        }

        CSLDestroy( papszClassColors );

        poDS->GetRasterBand(1)->SetColorTable( &oCT );
        poDS->GetRasterBand(1)->SetColorInterpretation( GCI_PaletteIndex );
    }
    
/* -------------------------------------------------------------------- */
/*      Look for mapinfo						*/
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue( poDS->papszHeader, "map_info" ) != NULL )
    {
        poDS->bFoundMapinfo = 
            poDS->ProcessMapinfo( 
                CSLFetchNameValue(poDS->papszHeader,"map_info") );
    }
    
/* -------------------------------------------------------------------- */
/*      Check for overviews.                                            */
/* -------------------------------------------------------------------- */
    poDS->oOvManager.Initialize( poDS, poOpenInfo->pszFilename );

/* -------------------------------------------------------------------- */
/*      Initialize any PAM information.                                 */
/* -------------------------------------------------------------------- */
    poDS->SetDescription( poOpenInfo->pszFilename );
    poDS->TryLoadXML();
    
    return( poDS );
}

int ENVIDataset::GetEnviType(GDALDataType eType)
{
  int iENVIType;
  switch( eType )
  {
      case GDT_Byte:
	      iENVIType = 1;
	      break;
      case GDT_Int16:
	      iENVIType = 2;
	      break;
      case GDT_Int32:
	      iENVIType = 3;
	      break;
      case GDT_Float32:
	      iENVIType = 4;
	      break;
      case GDT_Float64:
	      iENVIType = 5;
	      break;
      case GDT_CFloat32:
	      iENVIType = 6;
	      break;
      case GDT_CFloat64:
	      iENVIType = 9;
	      break;
      case GDT_UInt16:
	      iENVIType = 12;
	      break;
      case GDT_UInt32:
	      iENVIType = 13;
	      break;

	/* 14=Int64, 15=UInt64 */

      default:
        CPLError( CE_Failure, CPLE_AppDefined,
              "Attempt to create ENVI .hdr labelled dataset with an illegal\n"
              "data type (%s).\n",
              GDALGetDataTypeName(eType) );
      	return 1;
  }
  return iENVIType;
}

/************************************************************************/
/*                               Create()                               */
/************************************************************************/

GDALDataset *ENVIDataset::Create( const char * pszFilename,
                                  int nXSize, int nYSize, int nBands,
                                  GDALDataType eType,
                                  char ** papszOptions )

{
/* -------------------------------------------------------------------- */
/*      Verify input options.                                           */
/* -------------------------------------------------------------------- */
    int	iENVIType = GetEnviType(eType);
    if (0 == iENVIType)
      return 0;

/* -------------------------------------------------------------------- */
/*      Try to create the file.                                         */
/* -------------------------------------------------------------------- */
    FILE	*fp;

    fp = VSIFOpen( pszFilename, "wb" );

    if( fp == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed,
                  "Attempt to create file `%s' failed.\n",
                  pszFilename );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Just write out a couple of bytes to establish the binary        */
/*      file, and then close it.                                        */
/* -------------------------------------------------------------------- */
    VSIFWrite( (void *) "\0\0", 2, 1, fp );
    VSIFClose( fp );

/* -------------------------------------------------------------------- */
/*      Create the .hdr filename.                                       */
/* -------------------------------------------------------------------- */
    const char	*pszHDRFilename;
    const char	*pszSuffix;

    pszSuffix = CSLFetchNameValue( papszOptions, "SUFFIX" );
    if ( pszSuffix && EQUALN( pszSuffix, "ADD", 3 ))
	pszHDRFilename = CPLFormFilename( NULL, pszFilename, "hdr" );
    else
	pszHDRFilename = CPLResetExtension(pszFilename, "hdr" );

/* -------------------------------------------------------------------- */
/*      Open the file.                                                  */
/* -------------------------------------------------------------------- */
    fp = VSIFOpen( pszHDRFilename, "wt" );
    if( fp == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed,
                  "Attempt to create file `%s' failed.\n",
                  pszHDRFilename );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Write out the header.                                           */
/* -------------------------------------------------------------------- */
    int		iBigEndian;
    const char	*pszInterleaving;
    
#ifdef CPL_LSB
    iBigEndian = 0;
#else
    iBigEndian = 1;
#endif

    VSIFPrintf( fp, "ENVI\n" );
    VSIFPrintf( fp, "samples = %d\nlines   = %d\nbands   = %d\n",
		nXSize, nYSize, nBands );
    VSIFPrintf( fp, "header offset = 0\nfile type = ENVI Standard\n" );
    VSIFPrintf( fp, "data type = %d\n", iENVIType );
    pszInterleaving = CSLFetchNameValue( papszOptions, "INTERLEAVE" );
    if ( pszInterleaving )
    {
	if ( EQUALN( pszInterleaving, "bip", 3 ) )
	    pszInterleaving = "bip";		    // interleaved by pixel
	else if ( EQUALN( pszInterleaving, "bil", 3 ) )
	    pszInterleaving = "bil";		    // interleaved by line
	else
	    pszInterleaving = "bsq";		// band sequental by default
    }
    else
	pszInterleaving = "bsq";
    VSIFPrintf( fp, "interleave = %s\n", pszInterleaving);
    VSIFPrintf( fp, "byte order = %d\n", iBigEndian );

    VSIFClose( fp );

    return (GDALDataset *) GDALOpen( pszFilename, GA_Update );
}

/************************************************************************/
/*                         GDALRegister_ENVI()                          */
/************************************************************************/

void GDALRegister_ENVI()

{
    GDALDriver	*poDriver;

    if( GDALGetDriverByName( "ENVI" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "ENVI" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "ENVI .hdr Labelled" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, 
                                   "frmt_various.html#ENVI" );
        poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONDATATYPES, 
                                   "Byte Int16 UInt16 Int32 UInt32 "
                                   "Float32 Float64 CFloat32 CFloat64" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONOPTIONLIST,
"<CreationOptionList>"
"   <Option name='SUFFIX' type='string-select'>"
"       <Value>ADD</Value>"
"   </Option>"
"   <Option name='INTERLEAVE' type='string-select'>"
"       <Value>BIP</Value>"
"       <Value>BIL</Value>"
"       <Value>BSQ</Value>"
"   </Option>"
"</CreationOptionList>" );

        poDriver->pfnOpen = ENVIDataset::Open;
        poDriver->pfnCreate = ENVIDataset::Create;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}

