/******************************************************************************
 * $Id$
 *
 * Project:  E00 grid driver
 * Purpose:  GDALDataset driver for E00 grid dataset.
 * Author:   Even Rouault, <even dot rouault at mines dash paris dot org>
 *
 ******************************************************************************
 * Copyright (c) 2011, Even Rouault
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

#include "cpl_vsi_virtual.h"
#include "cpl_string.h"
#include "ogr_spatialref.h"
#include "gdal_pam.h"

/* Private import of e00read.c */
#define E00ReadOpen         GDALE00GRIDReadOpen
#define E00ReadCallbackOpen GDALE00GRIDReadCallbackOpen
#define E00ReadClose        GDALE00GRIDReadClose
#define E00ReadNextLine     GDALE00GRIDReadNextLine
#define E00ReadRewind       GDALE00GRIDReadRewind
#include "e00read.c"

#define E00_INT_SIZE    10
#define E00_INT14_SIZE  14
#define E00_FLOAT_SIZE  14
#define E00_DOUBLE_SIZE 21
#define VALS_PER_LINE   5

CPL_CVSID("$Id$");

CPL_C_START
void    GDALRegister_E00GRID(void);
CPL_C_END

/* g++ -fPIC -Wall -g frmts/e00grid/e00griddataset.cpp -shared -o gdal_E00GRID.so -Iport -Igcore -Iogr -L. -lgdal */

/* Test data ; (google for "EXP  0" "GRD  2")

ftp://msdis.missouri.edu/pub/dem/24k/county/
http://dusk.geo.orst.edu/djl/samoa/data/samoa_bathy.e00
http://dusk.geo.orst.edu/djl/samoa/FBNMS/RasterGrids-Metadata/ntae02_3m_utm.e00
http://www.navdat.org/coverages/elevation/iddem1.e00        (int32)
http://delta-vision.projects.atlas.ca.gov/lidar/bare_earth.grids/sac0165.e00
http://ag.arizona.edu/SRER/maps_e00/srer_dem.e00
http://ok.water.usgs.gov/projects/norlan/spatial/ntopo0408-10.e00 (compressed)
http://wrri.nmsu.edu/publish/techrpt/tr322/GIS/dem.e00 (compressed)
*/

/************************************************************************/
/* ==================================================================== */
/*                            E00GRIDDataset                            */
/* ==================================================================== */
/************************************************************************/

class E00GRIDRasterBand;

class E00GRIDDataset : public GDALPamDataset
{
    friend class E00GRIDRasterBand;

    E00ReadPtr  e00ReadPtr;
    VSILFILE   *fp;
    int         nDataStart;
    int         nBytesEOL;

    int         nLastLine;

    double      adfGeoTransform[6];
    CPLString   osProjection;

    double      dfNoData;

    char**      papszPrj;

    const char* ReadLine();

    int         bHasReadMetadata;
    void        ReadMetadata();

    int         bHasStats;
    double      dfMin, dfMax, dfMean, dfStddev;

  public:
                 E00GRIDDataset();
    virtual     ~E00GRIDDataset();
    
    virtual CPLErr GetGeoTransform( double * );
    virtual const char* GetProjectionRef();
    
    static GDALDataset *Open( GDALOpenInfo * );
    static int          Identify( GDALOpenInfo * );
};

/************************************************************************/
/* ==================================================================== */
/*                          E00GRIDRasterBand                           */
/* ==================================================================== */
/************************************************************************/

class E00GRIDRasterBand : public GDALPamRasterBand
{
    friend class E00GRIDDataset;

  public:

                E00GRIDRasterBand( E00GRIDDataset *, int, GDALDataType );

    virtual CPLErr      IReadBlock( int, int, void * );

    virtual double      GetNoDataValue( int *pbSuccess = NULL );
    virtual const char *GetUnitType();
    virtual double      GetMinimum( int *pbSuccess = NULL );
    virtual double      GetMaximum( int *pbSuccess = NULL );
    virtual CPLErr      GetStatistics( int bApproxOK, int bForce,
                                       double *pdfMin, double *pdfMax,
                                       double *pdfMean, double *padfStdDev );
};


/************************************************************************/
/*                         E00GRIDRasterBand()                          */
/************************************************************************/

E00GRIDRasterBand::E00GRIDRasterBand( E00GRIDDataset *poDS, int nBand, GDALDataType eDT )

{
    this->poDS = poDS;
    this->nBand = nBand;

    eDataType = eDT;

    nBlockXSize = poDS->GetRasterXSize();
    nBlockYSize = 1;
}

/************************************************************************/
/*                             IReadBlock()                             */
/************************************************************************/

CPLErr E00GRIDRasterBand::IReadBlock( int nBlockXOff, int nBlockYOff,
                                      void * pImage )

{
    E00GRIDDataset *poGDS = (E00GRIDDataset *) poDS;

    char szVal[E00_FLOAT_SIZE+1];
    szVal[E00_FLOAT_SIZE] = 0;

    int i;
    float* pafImage = (float*)pImage;
    int* panImage = (int*)pImage;
    const float fNoData = (const float)poGDS->dfNoData;

    /* A new data line begins on a new text line. So if the xsize */
    /* is not a multiple of VALS_PER_LINE, there are padding values */
    /* that must be ignored */
    const int nRoundedBlockXSize = ((nBlockXSize + VALS_PER_LINE - 1) / VALS_PER_LINE) * VALS_PER_LINE;

    if (poGDS->e00ReadPtr)
    {
        if (nBlockYOff <= poGDS->nLastLine || poGDS->nLastLine < 0)
        {
            E00ReadRewind(poGDS->e00ReadPtr);
            for(i=0;i<6;i++)
                E00ReadNextLine(poGDS->e00ReadPtr);
            poGDS->nLastLine = -1;
        }

        if (nBlockYOff > poGDS->nLastLine + 1)
        {
            int nValsToSkip = (nBlockYOff - poGDS->nLastLine - 1) * nRoundedBlockXSize;
            int nLinesToSkip = nValsToSkip / VALS_PER_LINE;
            for(i=0;i<nLinesToSkip;i++)
                E00ReadNextLine(poGDS->e00ReadPtr);
        }

        const char* pszLine = NULL;
        for(i=0;i<nBlockXSize;i++)
        {
            if ((i % VALS_PER_LINE) == 0)
            {
                pszLine = E00ReadNextLine(poGDS->e00ReadPtr);
                if (pszLine == NULL || strlen(pszLine) < 5 * E00_FLOAT_SIZE)
                    return CE_Failure;
            }
            if (eDataType == GDT_Float32)
            {
                pafImage[i] = atof(pszLine + (i%VALS_PER_LINE) * E00_FLOAT_SIZE);
                /* Workaround single vs double precision problems */
                if (fNoData != 0 && fabs((pafImage[i] - fNoData)/fNoData) < 1e-6)
                    pafImage[i] = fNoData;
            }
            else
            {
                panImage[i] = atoi(pszLine + (i%VALS_PER_LINE) * E00_FLOAT_SIZE);
            }
        }

        poGDS->nLastLine = nBlockYOff;

        return CE_None;
    }

    int nValsToSkip = nBlockYOff * nRoundedBlockXSize;
    int nLinesToSkip = nValsToSkip / VALS_PER_LINE;
    int nBytesPerLine = VALS_PER_LINE * E00_FLOAT_SIZE + poGDS->nBytesEOL;
    int nPos = poGDS->nDataStart + nLinesToSkip * nBytesPerLine;
    VSIFSeekL(poGDS->fp, nPos, SEEK_SET);

    for(i=0;i<nBlockXSize;i++)
    {
        if (VSIFReadL(szVal, E00_FLOAT_SIZE, 1, poGDS->fp) != 1)
            return CE_Failure;

        if (eDataType == GDT_Float32)
        {
            pafImage[i] = atof(szVal);
            /* Workaround single vs double precision problems */
            if (fNoData != 0 && fabs((pafImage[i] - fNoData)/fNoData) < 1e-6)
                pafImage[i] = fNoData;
        }
        else
        {
            panImage[i] = atoi(szVal);
        }

        if (((i+1) % VALS_PER_LINE) == 0)
            VSIFReadL(szVal, poGDS->nBytesEOL, 1, poGDS->fp);
    }

    return CE_None;
}

/************************************************************************/
/*                           GetNoDataValue()                           */
/************************************************************************/

double E00GRIDRasterBand::GetNoDataValue( int *pbSuccess )
{
    E00GRIDDataset *poGDS = (E00GRIDDataset *) poDS;

    if (pbSuccess)
        *pbSuccess = TRUE;

    if (eDataType == GDT_Float32)
        return (double)(float) poGDS->dfNoData;
    else
        return (double)(int)poGDS->dfNoData;
}

/************************************************************************/
/*                             GetUnitType()                            */
/************************************************************************/

const char * E00GRIDRasterBand::GetUnitType()
{
    E00GRIDDataset *poGDS = (E00GRIDDataset *) poDS;

    poGDS->ReadMetadata();

    if (poGDS->papszPrj == NULL)
        return GDALPamRasterBand::GetUnitType();

    char** papszIter = poGDS->papszPrj;
    const char* pszRet = "";
    while(*papszIter)
    {
        if (EQUALN(*papszIter, "Zunits", 6))
        {
            char** papszTokens = CSLTokenizeString(*papszIter);
            if (CSLCount(papszTokens) == 2)
            {
                if (EQUAL(papszTokens[1], "FEET"))
                    pszRet = "ft";
                else if (EQUAL(papszTokens[1], "METERS"))
                    pszRet = "m";
            }
            CSLDestroy(papszTokens);
            break;
        }
        papszIter ++;
    }

    return pszRet;
}

/************************************************************************/
/*                           GetMinimum()                               */
/************************************************************************/

double E00GRIDRasterBand::GetMinimum( int *pbSuccess )
{
    E00GRIDDataset *poGDS = (E00GRIDDataset *) poDS;

    if (poGDS->e00ReadPtr == NULL)
        poGDS->ReadMetadata();

    if (poGDS->bHasStats)
    {
        if( pbSuccess != NULL )
            *pbSuccess = TRUE;

        return poGDS->dfMin;
    }

    return GDALPamRasterBand::GetMinimum( pbSuccess );
}

/************************************************************************/
/*                           GetMaximum()                               */
/************************************************************************/

double E00GRIDRasterBand::GetMaximum( int *pbSuccess )
{
    E00GRIDDataset *poGDS = (E00GRIDDataset *) poDS;

    if (poGDS->e00ReadPtr == NULL)
        poGDS->ReadMetadata();

    if (poGDS->bHasStats)
    {
        if( pbSuccess != NULL )
            *pbSuccess = TRUE;

        return poGDS->dfMax;
    }

    return GDALPamRasterBand::GetMaximum( pbSuccess );
}

/************************************************************************/
/*                            GetStatistics()                           */
/************************************************************************/

CPLErr E00GRIDRasterBand::GetStatistics( int bApproxOK, int bForce,
                                         double *pdfMin, double *pdfMax,
                                         double *pdfMean, double *pdfStdDev )
{
    E00GRIDDataset *poGDS = (E00GRIDDataset *) poDS;

    if (poGDS->e00ReadPtr == NULL || bForce)
        poGDS->ReadMetadata();

    if (poGDS->bHasStats)
    {
        if (pdfMin)
            *pdfMin = poGDS->dfMin;
        if (pdfMax)
            *pdfMax = poGDS->dfMax;
        if (pdfMean)
            *pdfMean = poGDS->dfMean;
        if (pdfStdDev)
            *pdfStdDev = poGDS->dfStddev;
        return CE_None;
    }

    return GDALPamRasterBand::GetStatistics(bApproxOK, bForce,
                                            pdfMin, pdfMax,
                                            pdfMean, pdfStdDev);
}

/************************************************************************/
/*                           E00GRIDDataset()                           */
/************************************************************************/

E00GRIDDataset::E00GRIDDataset()
{
    fp = NULL;
    adfGeoTransform[0] = 0;
    adfGeoTransform[1] = 1;
    adfGeoTransform[2] = 0;
    adfGeoTransform[3] = 0;
    adfGeoTransform[4] = 0;
    adfGeoTransform[5] = 1;
    bHasReadMetadata = FALSE;
    dfNoData = 0;
    nDataStart = 0;
    nBytesEOL = 1;
    papszPrj = NULL;
    e00ReadPtr = NULL;
    nLastLine = -1;
    bHasStats = FALSE;
    dfMin = 0;
    dfMax = 0;
    dfMean = 0;
    dfStddev = 0;
}

/************************************************************************/
/*                           ~E00GRIDDataset()                          */
/************************************************************************/

E00GRIDDataset::~E00GRIDDataset()

{
    FlushCache();
    if (fp)
        VSIFCloseL(fp);
    CSLDestroy(papszPrj);
    E00ReadClose(e00ReadPtr);
}

/************************************************************************/
/*                             Identify()                               */
/************************************************************************/

int E00GRIDDataset::Identify( GDALOpenInfo * poOpenInfo )
{
    if (poOpenInfo->nHeaderBytes == 0)
        return FALSE;

    if (!(EQUALN((const char*)poOpenInfo->pabyHeader, "EXP  0", 6) ||
          EQUALN((const char*)poOpenInfo->pabyHeader, "EXP  1", 6)))
        return FALSE;

    /* FIXME: handle GRD  3 if that ever exists ? */
    if (strstr((const char*)poOpenInfo->pabyHeader, "GRD  2") == NULL)
        return FALSE;

    return TRUE;
}

/************************************************************************/
/*                    E00GRIDDatasetReadNextLine()                      */
/************************************************************************/

static const char* E00GRIDDatasetReadNextLine(void * fp)
{
    return CPLReadLine2L((VSILFILE*)fp, 256, NULL);
}

/************************************************************************/
/*                        E00GRIDDatasetRewind()                       */
/************************************************************************/

static void E00GRIDDatasetRewind(void * fp)
{
    VSIRewindL((VSILFILE*)fp);
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *E00GRIDDataset::Open( GDALOpenInfo * poOpenInfo )

{
    int         i;
    GDALDataType eDT = GDT_Float32;

    if (!Identify(poOpenInfo))
        return NULL;

/* -------------------------------------------------------------------- */
/*      Find dataset characteristics                                    */
/* -------------------------------------------------------------------- */
    VSILFILE* fp = VSIFOpenL(poOpenInfo->pszFilename, "rb");
    if (fp == NULL)
        return NULL;

    if (poOpenInfo->eAccess == GA_Update)
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "The E00GRID driver does not support update access to existing"
                  " datasets.\n" );
        VSIFCloseL(fp);
        return NULL;
    }

    const char* pszLine;
    /* read EXP  0 or EXP  1 line */
    pszLine = CPLReadLine2L(fp, 81, NULL);
    int bCompressed = EQUALN(pszLine, "EXP  1", 6);
    if (pszLine == NULL)
    {
        CPLDebug("E00GRID", "Bad 1st line");
        VSIFCloseL(fp);
        return NULL;
    }

    E00ReadPtr e00ReadPtr = NULL;
    if (bCompressed)
    {
        VSIRewindL(fp);
        e00ReadPtr = E00ReadCallbackOpen(fp,
                                         E00GRIDDatasetReadNextLine,
                                         E00GRIDDatasetRewind);
        if (e00ReadPtr == NULL)
        {
            VSIFCloseL(fp);
            return NULL;
        }
        E00ReadNextLine(e00ReadPtr);
    }

    /* skip GRD  2 line */
    if (e00ReadPtr)
        pszLine = E00ReadNextLine(e00ReadPtr);
    else
        pszLine = CPLReadLine2L(fp, 81, NULL);
    if (pszLine == NULL || !EQUALN(pszLine, "GRD  2", 6))
    {
        CPLDebug("E00GRID", "Bad 2nd line");
        VSIFCloseL(fp);
        E00ReadClose(e00ReadPtr);
        return NULL;
    }

    /* read ncols, nrows and nodata value */
    if (e00ReadPtr)
        pszLine = E00ReadNextLine(e00ReadPtr);
    else
        pszLine = CPLReadLine2L(fp, 81, NULL);
    if (pszLine == NULL || strlen(pszLine) <
                E00_INT_SIZE+E00_INT_SIZE+2+E00_DOUBLE_SIZE)
    {
        CPLDebug("E00GRID", "Bad 3rd line");
        VSIFCloseL(fp);
        E00ReadClose(e00ReadPtr);
        return NULL;
    }

    int nRasterXSize = atoi(pszLine);
    int nRasterYSize = atoi(pszLine + E00_INT_SIZE);
    
    if (EQUALN(pszLine + E00_INT_SIZE + E00_INT_SIZE, " 1", 2))
        eDT = GDT_Int32;
    else if (EQUALN(pszLine + E00_INT_SIZE + E00_INT_SIZE, " 2", 2))
        eDT = GDT_Float32;
    else
    {
        CPLDebug("E00GRID", "Unknown data type : %s", pszLine);
    }

    double dfNoData = atof(pszLine + E00_INT_SIZE + E00_INT_SIZE + 2);

    /* read pixel size */
    if (e00ReadPtr)
        pszLine = E00ReadNextLine(e00ReadPtr);
    else
        pszLine = CPLReadLine2L(fp, 81, NULL);
    if (pszLine == NULL || strlen(pszLine) < 2*E00_DOUBLE_SIZE)
    {
        CPLDebug("E00GRID", "Bad 4th line");
        VSIFCloseL(fp);
        E00ReadClose(e00ReadPtr);
        return NULL;
    }
/*
    double dfPixelX = atof(pszLine);
    double dfPixelY = atof(pszLine + E00_DOUBLE_SIZE);
*/

    /* read xmin, ymin */
    if (e00ReadPtr)
        pszLine = E00ReadNextLine(e00ReadPtr);
    else
        pszLine = CPLReadLine2L(fp, 81, NULL);
    if (pszLine == NULL || strlen(pszLine) < 2*E00_DOUBLE_SIZE)
    {
        CPLDebug("E00GRID", "Bad 5th line");
        VSIFCloseL(fp);
        E00ReadClose(e00ReadPtr);
        return NULL;
    }
    double dfMinX = atof(pszLine);
    double dfMinY = atof(pszLine + E00_DOUBLE_SIZE);

    /* read xmax, ymax */
    if (e00ReadPtr)
        pszLine = E00ReadNextLine(e00ReadPtr);
    else
        pszLine = CPLReadLine2L(fp, 81, NULL);
    if (pszLine == NULL || strlen(pszLine) < 2*E00_DOUBLE_SIZE)
    {
        CPLDebug("E00GRID", "Bad 6th line");
        VSIFCloseL(fp);
        E00ReadClose(e00ReadPtr);
        return NULL;
    }
    double dfMaxX = atof(pszLine);
    double dfMaxY = atof(pszLine + E00_DOUBLE_SIZE);

/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */
    E00GRIDDataset         *poDS;

    poDS = new E00GRIDDataset();
    if (strstr((const char*)poOpenInfo->pabyHeader, "\r\n") != NULL)
        poDS->nBytesEOL = 2;

    poDS->nRasterXSize = nRasterXSize;
    poDS->nRasterYSize = nRasterYSize;
    poDS->dfNoData = dfNoData;
    poDS->adfGeoTransform[0] = dfMinX;
    poDS->adfGeoTransform[1] = (dfMaxX - dfMinX) / nRasterXSize;
    poDS->adfGeoTransform[2] = 0;
    poDS->adfGeoTransform[3] = dfMaxY;
    poDS->adfGeoTransform[4] = 0;
    poDS->adfGeoTransform[5] = - (dfMaxY - dfMinY) / nRasterYSize;
    poDS->fp = fp;
    poDS->e00ReadPtr = e00ReadPtr;
    poDS->nDataStart = VSIFTellL(fp);

    if (!GDALCheckDatasetDimensions(poDS->nRasterXSize, poDS->nRasterYSize))
    {
        delete poDS;
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Create band information objects.                                */
/* -------------------------------------------------------------------- */
    poDS->nBands = 1;
    for( i = 0; i < poDS->nBands; i++ )
        poDS->SetBand( i+1, new E00GRIDRasterBand( poDS, i+1, eDT ) );

/* -------------------------------------------------------------------- */
/*      Initialize any PAM information.                                 */
/* -------------------------------------------------------------------- */
    poDS->SetDescription( poOpenInfo->pszFilename );
    poDS->TryLoadXML();

/* -------------------------------------------------------------------- */
/*      Support overviews.                                              */
/* -------------------------------------------------------------------- */
    poDS->oOvManager.Initialize( poDS, poOpenInfo->pszFilename );
    return( poDS );
}

/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr E00GRIDDataset::GetGeoTransform( double * padfTransform )

{
    memcpy(padfTransform, adfGeoTransform, 6 * sizeof(double));

    return( CE_None );
}


/************************************************************************/
/*                             ReadLine()                               */
/************************************************************************/

const char* E00GRIDDataset::ReadLine()
{
    if (e00ReadPtr)
        return E00ReadNextLine(e00ReadPtr);
    else
        return CPLReadLine2L(fp, 81, NULL);
}

/************************************************************************/
/*                         GetProjectionRef()                           */
/************************************************************************/

const char* E00GRIDDataset::GetProjectionRef()

{
    ReadMetadata();
    return osProjection.c_str();
}

/************************************************************************/
/*                          ReadMetadata()                              */
/************************************************************************/

void E00GRIDDataset::ReadMetadata()

{
    if (bHasReadMetadata)
        return;

    bHasReadMetadata = TRUE;

    if (e00ReadPtr == NULL)
    {
        int nRoundedBlockXSize = ((nRasterXSize + VALS_PER_LINE - 1) / VALS_PER_LINE) * VALS_PER_LINE;
        int nValsToSkip = nRasterYSize * nRoundedBlockXSize;
        int nLinesToSkip = nValsToSkip / VALS_PER_LINE;
        int nBytesPerLine = VALS_PER_LINE * E00_FLOAT_SIZE + nBytesEOL;
        int nPos = nDataStart + nLinesToSkip * nBytesPerLine;
        VSIFSeekL(fp, nPos, SEEK_SET);
    }
    else
    {
        nLastLine = -1;
    }

    const char* pszLine;
    int bPRJFound = FALSE;
    int bStatsFound = FALSE;
    while((pszLine = ReadLine()) != NULL)
    {
        if (EQUALN(pszLine, "PRJ  2", 6))
        {
            bPRJFound = TRUE;
            while((pszLine = ReadLine()) != NULL)
            {
                if (EQUAL(pszLine, "EOP"))
                {
                    break;
                }
                papszPrj = CSLAddString(papszPrj, pszLine);
            }

            OGRSpatialReference oSRS;
            if( oSRS.importFromESRI( papszPrj ) != OGRERR_NONE )
            {
                CPLError( CE_Warning, CPLE_AppDefined,
                            "Failed to parse PRJ section, ignoring." );
            }
            else
            {
                char* pszWKT = NULL;
                if (oSRS.exportToWkt(&pszWKT) == OGRERR_NONE && pszWKT != NULL)
                    osProjection = pszWKT;
                CPLFree(pszWKT);
            }
            if (bStatsFound)
                break;
        }
        else if (strcmp(pszLine, "STDV              8-1  254-1  15 3 60-1  -1  -1-1                   4-") == 0)
        {
            bStatsFound = TRUE;
            pszLine = ReadLine();
            if (pszLine)
            {
                CPLString osStats = pszLine;
                pszLine = ReadLine();
                if (pszLine)
                {
                    osStats += pszLine;
                    char** papszTokens = CSLTokenizeString(osStats);
                    if (CSLCount(papszTokens) == 4)
                    {
                        dfMin = atof(papszTokens[0]);
                        dfMax = atof(papszTokens[1]);
                        dfMean = atof(papszTokens[2]);
                        dfStddev = atof(papszTokens[3]);
                        bHasStats = TRUE;
                    }
                    CSLDestroy(papszTokens);
                }
            }
            if (bPRJFound)
                break;
        }
    }
}

/************************************************************************/
/*                       GDALRegister_E00GRID()                         */
/************************************************************************/

void GDALRegister_E00GRID()

{
    GDALDriver  *poDriver;

    if( GDALGetDriverByName( "E00GRID" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "E00GRID" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "Arc/Info Export E00 GRID" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, 
                                   "frmt_various.html#E00GRID" );
        poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "e00" );


        poDriver->SetMetadataItem( GDAL_DCAP_VIRTUALIO, "YES" );

        poDriver->pfnOpen = E00GRIDDataset::Open;
        poDriver->pfnIdentify = E00GRIDDataset::Identify;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}

