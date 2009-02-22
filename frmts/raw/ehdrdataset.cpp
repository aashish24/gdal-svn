/******************************************************************************
 * $Id$
 *
 * Project:  ESRI .hdr Driver
 * Purpose:  Implementation of EHdrDataset
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 1999, Frank Warmerdam <warmerdam@pobox.com>
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
void	GDALRegister_EHdr(void);
CPL_C_END

/************************************************************************/
/* ==================================================================== */
/*				EHdrDataset				*/
/* ==================================================================== */
/************************************************************************/

class EHdrRasterBand;

class EHdrDataset : public RawDataset
{
    friend class EHdrRasterBand;

    FILE	*fpImage;	// image data file.

    int         bGotTransform;
    double      adfGeoTransform[6];
    char       *pszProjection;

    int         bHDRDirty;
    char      **papszHDR;

    int         bSTXDirty;

    CPLErr      ReadSTX();
    CPLErr      RewriteSTX();
    CPLErr      RewriteHDR();
    void        ResetKeyValue( const char *pszKey, const char *pszValue );
    const char *GetKeyValue( const char *pszKey, const char *pszDefault = "" );
    void        RewriteColorTable( GDALColorTable * );

  public:
    EHdrDataset();
    ~EHdrDataset();
    
    virtual CPLErr GetGeoTransform( double * padfTransform );
    virtual CPLErr SetGeoTransform( double *padfTransform );
    virtual const char *GetProjectionRef(void);
    virtual CPLErr SetProjection( const char * );
    
    virtual char **GetFileList();

    static GDALDataset *Open( GDALOpenInfo * );
    static GDALDataset *Create( const char * pszFilename,
                                int nXSize, int nYSize, int nBands,
                                GDALDataType eType, char ** papszParmList );
    static GDALDataset *CreateCopy( const char * pszFilename, 
                                    GDALDataset * poSrcDS, 
                                    int bStrict, char ** papszOptions,
                                    GDALProgressFunc pfnProgress,
                                    void * pProgressData );
    static CPLString GetImageRepFilename(const char* pszFilename);
};

/************************************************************************/
/* ==================================================================== */
/*                          EHdrRasterBand                              */
/* ==================================================================== */
/************************************************************************/

class EHdrRasterBand : public RawRasterBand
{
   friend class EHdrDataset;

    int            nBits;
    long           nStartBit;
    int            nPixelOffsetBits;
    int            nLineOffsetBits;

    double         dfMin;
    double         dfMax;
    double         dfMean;
    double         dfStdDev;

    int            minmaxmeanstddev;

    virtual CPLErr IRasterIO( GDALRWFlag, int, int, int, int,
                              void *, int, int, GDALDataType,
                              int, int );

  public:
    EHdrRasterBand( GDALDataset *poDS, int nBand, FILE * fpRaw, 
                    vsi_l_offset nImgOffset, int nPixelOffset,
                    int nLineOffset,
                    GDALDataType eDataType, int bNativeOrder,
                    int nBits);

    virtual CPLErr IReadBlock( int, int, void * );
    virtual CPLErr IWriteBlock( int, int, void * );

    virtual double GetMinimum( int *pbSuccess = NULL );
    virtual double GetMaximum(int *pbSuccess = NULL );
    virtual CPLErr GetStatistics( int bApproxOK, int bForce,
                                  double *pdfMin, double *pdfMax, 
                                  double *pdfMean, double *pdfStdDev );
    virtual CPLErr SetStatistics( double dfMin, double dfMax, 
                                  double dfMean, double dfStdDev );

};

/************************************************************************/
/*                           EHdrRasterBand()                           */
/************************************************************************/

EHdrRasterBand::EHdrRasterBand( GDALDataset *poDS,
                                int nBand, FILE * fpRaw, 
                                vsi_l_offset nImgOffset, int nPixelOffset,
                                int nLineOffset,
                                GDALDataType eDataType, int bNativeOrder,
                                int nBits)
: RawRasterBand( poDS, nBand, fpRaw, nImgOffset, nPixelOffset, nLineOffset, 
                         eDataType, bNativeOrder, TRUE ),
  nBits(nBits),
  dfMin(0),
  dfMax(0),
  minmaxmeanstddev(0)
{
    if (nBits < 8)
    {
        EHdrDataset* poEDS = (EHdrDataset*)poDS;
        nStartBit = atoi(poEDS->GetKeyValue("SKIPBYTES")) * 8;
        if (nBand >= 2)
        {
            long nRowBytes = atoi(poEDS->GetKeyValue("BANDROWBYTES"));
            if (nRowBytes == 0)
                nRowBytes = (nBits * poDS->GetRasterXSize() + 7) / 8;

            nStartBit += nRowBytes * (nBand-1) * 8;
        }

        nPixelOffsetBits = nBits;
        nLineOffsetBits = atoi(poEDS->GetKeyValue("TOTALROWBYTES")) * 8;
        if( nLineOffsetBits == 0 )
            nLineOffsetBits = nPixelOffsetBits * poDS->GetRasterXSize();

        nBlockXSize = poDS->GetRasterXSize();
        nBlockYSize = 1;

        SetMetadataItem( "NBITS", 
                         CPLString().Printf( "%d", nBits ),
                         "IMAGE_STRUCTURE" );
    }
}

/************************************************************************/
/*                             IReadBlock()                             */
/************************************************************************/

CPLErr EHdrRasterBand::IReadBlock( int nBlockXOff, int nBlockYOff,
                                   void * pImage )

{
    if (nBits >= 8)
      return RawRasterBand::IReadBlock(nBlockXOff, nBlockYOff, pImage);

    vsi_l_offset   nLineStart;
    unsigned int   nLineBytes;
    int            iBitOffset;
    GByte         *pabyBuffer;

/* -------------------------------------------------------------------- */
/*      Establish desired position.                                     */
/* -------------------------------------------------------------------- */
    nLineBytes = (nPixelOffsetBits*nBlockXSize + 7)/8;
    nLineStart = (nStartBit + ((vsi_l_offset)nLineOffsetBits) * nBlockYOff) / 8;
    iBitOffset = 
        (nStartBit + ((vsi_l_offset)nLineOffsetBits) * nBlockYOff) % 8;

/* -------------------------------------------------------------------- */
/*      Read data into buffer.                                          */
/* -------------------------------------------------------------------- */
    pabyBuffer = (GByte *) CPLCalloc(nLineBytes,1);

    if( VSIFSeekL( GetFP(), nLineStart, SEEK_SET ) != 0 
        || VSIFReadL( pabyBuffer, 1, nLineBytes, GetFP() ) != nLineBytes )
    {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Failed to read %u bytes at offset %lu.\n%s",
                  nLineBytes, (unsigned long)nLineStart, 
                  VSIStrerror( errno ) );
        return CE_Failure;
    }

/* -------------------------------------------------------------------- */
/*      Copy data, promoting to 8bit.                                   */
/* -------------------------------------------------------------------- */
    int iPixel = 0, iX;

    for( iX = 0; iX < nBlockXSize; iX++ )
    {
        int  nOutWord = 0, iBit;

        for( iBit = 0; iBit < nBits; iBit++ )
        {
            if( pabyBuffer[iBitOffset>>3]  & (0x80 >>(iBitOffset & 7)) )
                nOutWord |= (1 << (nBits - 1 - iBit));
            iBitOffset++;
        } 

        iBitOffset = iBitOffset + nPixelOffsetBits - nBits;
                
        ((GByte *) pImage)[iPixel++] = nOutWord;
    }

    CPLFree( pabyBuffer );

    return CE_None;
}

/************************************************************************/
/*                            IWriteBlock()                             */
/************************************************************************/

CPLErr EHdrRasterBand::IWriteBlock( int nBlockXOff, int nBlockYOff,
                                   void * pImage )

{
    if (nBits >= 8)
      return RawRasterBand::IWriteBlock(nBlockXOff, nBlockYOff, pImage);

    vsi_l_offset   nLineStart;
    unsigned int   nLineBytes;
    int            iBitOffset;
    GByte         *pabyBuffer;

/* -------------------------------------------------------------------- */
/*      Establish desired position.                                     */
/* -------------------------------------------------------------------- */
    nLineBytes = (nPixelOffsetBits*nBlockXSize + 7)/8;
    nLineStart = (nStartBit + ((vsi_l_offset)nLineOffsetBits) * nBlockYOff) / 8;
    iBitOffset = 
        (nStartBit + ((vsi_l_offset)nLineOffsetBits) * nBlockYOff) % 8;

/* -------------------------------------------------------------------- */
/*      Read data into buffer.                                          */
/* -------------------------------------------------------------------- */
    pabyBuffer = (GByte *) CPLCalloc(nLineBytes,1);

    if( VSIFSeekL( GetFP(), nLineStart, SEEK_SET ) != 0 )
    {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Failed to read %u bytes at offset %lu.\n%s",
                  nLineBytes, (unsigned long)nLineStart, 
                  VSIStrerror( errno ) );
        return CE_Failure;
    }

    VSIFReadL( pabyBuffer, 1, nLineBytes, GetFP() );

/* -------------------------------------------------------------------- */
/*      Copy data, promoting to 8bit.                                   */
/* -------------------------------------------------------------------- */
    int iPixel = 0, iX;

    for( iX = 0; iX < nBlockXSize; iX++ )
    {
        int iBit;
        int  nOutWord = ((GByte *) pImage)[iPixel++];

        for( iBit = 0; iBit < nBits; iBit++ )
        {
            if( nOutWord & (1 << (nBits - 1 - iBit)) )
                pabyBuffer[iBitOffset>>3] |= (0x80 >>(iBitOffset & 7));
            else
                pabyBuffer[iBitOffset>>3] &= ~((0x80 >>(iBitOffset & 7)));

            iBitOffset++;
        } 

        iBitOffset = iBitOffset + nPixelOffsetBits - nBits;
    }

/* -------------------------------------------------------------------- */
/*      Write the data back out.                                        */
/* -------------------------------------------------------------------- */
    if( VSIFSeekL( GetFP(), nLineStart, SEEK_SET ) != 0 
        || VSIFWriteL( pabyBuffer, 1, nLineBytes, GetFP() ) != nLineBytes )
    {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Failed to write %u bytes at offset %lu.\n%s",
                  nLineBytes, (unsigned long)nLineStart, 
                  VSIStrerror( errno ) );
        return CE_Failure;
    }

    CPLFree( pabyBuffer );

    return CE_None;
}

/************************************************************************/
/*                             IRasterIO()                              */
/************************************************************************/

CPLErr EHdrRasterBand::IRasterIO( GDALRWFlag eRWFlag,
                                  int nXOff, int nYOff, int nXSize, int nYSize,
                                  void * pData, int nBufXSize, int nBufYSize,
                                  GDALDataType eBufType,
                                  int nPixelSpace, int nLineSpace )

{
    // Defer to RawRasterBand
    if (nBits >= 8)
        return RawRasterBand::IRasterIO( eRWFlag, 
                                         nXOff, nYOff, nXSize, nYSize,
                                         pData, nBufXSize, nBufYSize, 
                                         eBufType, nPixelSpace, nLineSpace );

    // Force use of IReadBlock() and IWriteBlock()
    else
        return GDALRasterBand::IRasterIO( eRWFlag, 
                                          nXOff, nYOff, nXSize, nYSize,
                                          pData, nBufXSize, nBufYSize, 
                                          eBufType, nPixelSpace, nLineSpace );
}

/************************************************************************/
/*                              OSR_GDS()                               */
/************************************************************************/

static const char*OSR_GDS( char **papszNV, const char * pszField, 
                           const char *pszDefaultValue )

{
    int         iLine;

    if( papszNV == NULL || papszNV[0] == NULL )
        return pszDefaultValue;

    for( iLine = 0; 
         papszNV[iLine] != NULL && 
             !EQUALN(papszNV[iLine],pszField,strlen(pszField));
         iLine++ ) {}

    if( papszNV[iLine] == NULL )
        return pszDefaultValue;
    else
    {
        static char     szResult[80];
        char    **papszTokens;
        
        papszTokens = CSLTokenizeString(papszNV[iLine]);

        if( CSLCount(papszTokens) > 1 )
            strncpy( szResult, papszTokens[1], sizeof(szResult));
        else
            strncpy( szResult, pszDefaultValue, sizeof(szResult));
        
        CSLDestroy( papszTokens );
        return szResult;
    }
}


/************************************************************************/
/* ==================================================================== */
/*				EHdrDataset				*/
/* ==================================================================== */
/************************************************************************/

/************************************************************************/
/*                            EHdrDataset()                             */
/************************************************************************/

EHdrDataset::EHdrDataset()
{
    fpImage = NULL;
    pszProjection = CPLStrdup("");
    bGotTransform = FALSE;
    adfGeoTransform[0] = 0.0;
    adfGeoTransform[1] = 1.0;
    adfGeoTransform[2] = 0.0;
    adfGeoTransform[3] = 0.0;
    adfGeoTransform[4] = 0.0;
    adfGeoTransform[5] = 1.0;
    papszHDR = NULL;
    bHDRDirty = FALSE;
    bSTXDirty = FALSE;
}

/************************************************************************/
/*                            ~EHdrDataset()                            */
/************************************************************************/

EHdrDataset::~EHdrDataset()

{
    FlushCache();

    if( nBands > 0 && GetAccess() == GA_Update )
    {
        int bNoDataSet;
        double dfNoData;
        RawRasterBand *poBand = (RawRasterBand *) GetRasterBand( 1 );

        dfNoData = poBand->GetNoDataValue(&bNoDataSet);
        if( bNoDataSet )
        {
            ResetKeyValue( "NODATA", 
                           CPLString().Printf( "%.8g", dfNoData ) );
        }

        if( poBand->GetColorTable() != NULL )
            RewriteColorTable( poBand->GetColorTable() );

        if( bHDRDirty )
            RewriteHDR();

        if( bSTXDirty )
            RewriteSTX();
    }

    if( fpImage != NULL )
        VSIFCloseL( fpImage );

    CPLFree( pszProjection );
    CSLDestroy( papszHDR );
}

/************************************************************************/
/*                            GetKeyValue()                             */
/************************************************************************/

const char *EHdrDataset::GetKeyValue( const char *pszKey, 
                                      const char *pszDefault )

{
    int i;

    for( i = 0; papszHDR[i] != NULL; i++ )
    {
        if( EQUALN(pszKey,papszHDR[i],strlen(pszKey))
            && isspace(papszHDR[i][strlen(pszKey)]) )
        {
            const char *pszValue = papszHDR[i] + strlen(pszKey);
            while( isspace(*pszValue) )
                pszValue++;
            
            return pszValue;
        }
    }

    return pszDefault;
}

/************************************************************************/
/*                           ResetKeyValue()                            */
/*                                                                      */
/*      Replace or add the keyword with the indicated value in the      */
/*      papszHDR list.                                                  */
/************************************************************************/

void EHdrDataset::ResetKeyValue( const char *pszKey, const char *pszValue )

{
    int i;
    char szNewLine[82];

    if( strlen(pszValue) > 65 )
    {
        CPLAssert( strlen(pszValue) <= 65 );
        return;
    }

    sprintf( szNewLine, "%-15s%s", pszKey, pszValue );

    for( i = CSLCount(papszHDR)-1; i >= 0; i-- )
    {
        if( EQUALN(papszHDR[i],szNewLine,strlen(pszKey)+1 ) )
        {
            if( strcmp(papszHDR[i],szNewLine) != 0 )
            {
                CPLFree( papszHDR[i] );
                papszHDR[i] = CPLStrdup( szNewLine );
                bHDRDirty = TRUE;
            }
            return;
        }
    }

    bHDRDirty = TRUE;
    papszHDR = CSLAddString( papszHDR, szNewLine );
}

/************************************************************************/
/*                         RewriteColorTable()                          */
/************************************************************************/

void EHdrDataset::RewriteColorTable( GDALColorTable *poTable )

{
    CPLString osCLRFilename = CPLResetExtension( GetDescription(), "clr" );
    FILE *fp;

    fp = VSIFOpenL( osCLRFilename, "wt" );
    if( fp != NULL )
    {
        int iColor;

        for( iColor = 0; iColor < poTable->GetColorEntryCount(); iColor++ )
        {
            CPLString oLine;
            GDALColorEntry sEntry;

            poTable->GetColorEntryAsRGB( iColor, &sEntry );

            // I wish we had a way to mark transparency.
            oLine.Printf( "%3d %3d %3d %3d\n",
                          iColor, sEntry.c1, sEntry.c2, sEntry.c3 );
            VSIFWriteL( (void *) oLine.c_str(), 1, strlen(oLine), fp );
        }
        VSIFCloseL( fp );
    }
    else
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "Unable to create color file %s.", 
                  osCLRFilename.c_str() );
    }
}

/************************************************************************/
/*                          GetProjectionRef()                          */
/************************************************************************/

const char *EHdrDataset::GetProjectionRef()

{
    if (pszProjection && strlen(pszProjection) > 0)
        return pszProjection;

    return GDALPamDataset::GetProjectionRef();
}

/************************************************************************/
/*                           SetProjection()                            */
/************************************************************************/

CPLErr EHdrDataset::SetProjection( const char *pszSRS )

{
/* -------------------------------------------------------------------- */
/*      Reset coordinate system on the dataset.                         */
/* -------------------------------------------------------------------- */
    CPLFree( pszProjection );
    pszProjection = CPLStrdup( pszSRS );

    if( strlen(pszSRS) == 0 )
        return CE_None;

/* -------------------------------------------------------------------- */
/*      Convert to ESRI WKT.                                            */
/* -------------------------------------------------------------------- */
    OGRSpatialReference oSRS( pszSRS );
    char *pszESRI_SRS = NULL;

    oSRS.morphToESRI();
    oSRS.exportToWkt( &pszESRI_SRS );

/* -------------------------------------------------------------------- */
/*      Write to .prj file.                                             */
/* -------------------------------------------------------------------- */
    CPLString osPrjFilename = CPLResetExtension( GetDescription(), "prj" );
    FILE *fp;

    fp = VSIFOpenL( osPrjFilename.c_str(), "wt" );
    if( fp != NULL )
    {
        VSIFWriteL( pszESRI_SRS, 1, strlen(pszESRI_SRS), fp );
        VSIFWriteL( (void *) "\n", 1, 1, fp );
        VSIFCloseL( fp );
    }

    CPLFree( pszESRI_SRS );

    return CE_None;
}

/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr EHdrDataset::GetGeoTransform( double * padfTransform )

{
    if( bGotTransform )
    {
        memcpy( padfTransform, adfGeoTransform, sizeof(double) * 6 );
        return CE_None;
    }
    else
    {
        return GDALPamDataset::GetGeoTransform( padfTransform );
    }
}

/************************************************************************/
/*                          SetGeoTransform()                           */
/************************************************************************/

CPLErr EHdrDataset::SetGeoTransform( double *padfGeoTransform )

{
/* -------------------------------------------------------------------- */
/*      We only support non-rotated images with info in the .HDR file.  */
/* -------------------------------------------------------------------- */
    if( padfGeoTransform[2] != 0.0 
        || padfGeoTransform[4] != 0.0 )
    {
        return GDALPamDataset::SetGeoTransform( padfGeoTransform );
    }

/* -------------------------------------------------------------------- */
/*      Record new geotransform.                                        */
/* -------------------------------------------------------------------- */
    bGotTransform = TRUE;
    memcpy( adfGeoTransform, padfGeoTransform, sizeof(double) * 6 );

/* -------------------------------------------------------------------- */
/*      Strip out all old geotransform keywords from HDR records.       */
/* -------------------------------------------------------------------- */
    int i;
    for( i = CSLCount(papszHDR)-1; i >= 0; i-- )
    {
        if( EQUALN(papszHDR[i],"ul",2)
            || EQUALN(papszHDR[i]+1,"ll",2)
            || EQUALN(papszHDR[i],"cell",4)
            || EQUALN(papszHDR[i]+1,"dim",3) )
        {
            papszHDR = CSLRemoveStrings( papszHDR, i, 1, NULL );
        }
    }

/* -------------------------------------------------------------------- */
/*      Set the transformation information.                             */
/* -------------------------------------------------------------------- */
    CPLString  oValue;

    oValue.Printf( "%.15g", adfGeoTransform[0] + adfGeoTransform[1] * 0.5 );
    ResetKeyValue( "ULXMAP", oValue );
                   
    oValue.Printf( "%.15g", adfGeoTransform[3] + adfGeoTransform[5] * 0.5 );
    ResetKeyValue( "ULYMAP", oValue );

    oValue.Printf( "%.15g", adfGeoTransform[1] );
    ResetKeyValue( "XDIM", oValue );

    oValue.Printf( "%.15g", fabs(adfGeoTransform[5]) );
    ResetKeyValue( "YDIM", oValue );

    return CE_None;
}

/************************************************************************/
/*                             RewriteHDR()                             */
/************************************************************************/

CPLErr EHdrDataset::RewriteHDR()

{
    CPLString osPath = CPLGetPath( GetDescription() );
    CPLString osName = CPLGetBasename( GetDescription() );
    CPLString osHDRFilename = CPLFormCIFilename( osPath, osName, "hdr" );

/* -------------------------------------------------------------------- */
/*      Write .hdr file.                                                */
/* -------------------------------------------------------------------- */
    FILE	*fp;
    int i;

    fp = VSIFOpenL( osHDRFilename, "wt" );

    if( fp == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "Failed to rewrite .hdr file %s.", 
                  osHDRFilename.c_str() );
        return CE_Failure;
    }

    for( i = 0; papszHDR[i] != NULL; i++ )
    {
        VSIFWriteL( papszHDR[i], 1, strlen(papszHDR[i]), fp );
        VSIFWriteL( (void *) "\n", 1, 1, fp );
    }

    VSIFCloseL( fp );

    bHDRDirty = FALSE;

    return CE_None;
}

/************************************************************************/
/*                             RewriteSTX()                             */
/************************************************************************/
    
CPLErr EHdrDataset::RewriteSTX()
{
    CPLString osPath = CPLGetPath( GetDescription() );
    CPLString osName = CPLGetBasename( GetDescription() );
    CPLString osSTXFilename = CPLFormCIFilename( osPath, osName, "stx" );

/* -------------------------------------------------------------------- */
/*      Write .stx file.                                                */
/* -------------------------------------------------------------------- */
    FILE	*fp;
    fp = VSIFOpenL( osSTXFilename, "wt" );
    if( fp == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "Failed to rewrite .stx file %s.", 
                  osSTXFilename.c_str() );
        return CE_Failure;
    }

    for (int i = 0; i < nBands; ++i)
    {
        EHdrRasterBand* poBand = (EHdrRasterBand*)papoBands[i];
        VSIFPrintfL( fp, "%d %.10f %.10f ", i+1, poBand->dfMin, poBand->dfMax );
        if ( poBand->minmaxmeanstddev & 0x4 )
            VSIFPrintfL( fp, "%.10f ", poBand->dfMean);
        else
            VSIFPrintfL( fp, "# ");

        if ( poBand->minmaxmeanstddev & 0x8 )
            VSIFPrintfL( fp, "%.10f\n", poBand->dfStdDev);
        else
            VSIFPrintfL( fp, "#\n");
    }

    VSIFCloseL( fp );

    bSTXDirty = FALSE;

    return CE_None;
}

/************************************************************************/
/*                              ReadSTX()                               */
/************************************************************************/

CPLErr EHdrDataset::ReadSTX()
{
    CPLString osPath = CPLGetPath( GetDescription() );
    CPLString osName = CPLGetBasename( GetDescription() );
    CPLString osSTXFilename = CPLFormCIFilename( osPath, osName, "stx" );

/* -------------------------------------------------------------------- */
/*      Read .stx file.                                                 */
/* -------------------------------------------------------------------- */
    FILE	*fp;
    if ((fp = VSIFOpenL( osSTXFilename, "rt" )))
    {
      const char *	pszLine;
      while( (pszLine = CPLReadLineL( fp )) )
      {
          char	**papszTokens;
          papszTokens = CSLTokenizeStringComplex( pszLine, " \t", TRUE, FALSE );
          if( CSLCount( papszTokens ) >= 5 )
          {
            int i = atoi(papszTokens[0]);
            if (i > 0 && i <= nBands)
            {
              EHdrRasterBand* poBand = (EHdrRasterBand*)papoBands[i-1];
              poBand->dfMin = atof(papszTokens[1]);
              poBand->dfMax = atof(papszTokens[2]);
              poBand->minmaxmeanstddev = 0x3;
              // reads optional mean and stddev
              if ( !EQUAL(papszTokens[3], "#") )
              {
                poBand->dfMean   = atof(papszTokens[3]);
                poBand->minmaxmeanstddev |= 0x4;
              }
              if ( !EQUAL(papszTokens[4], "#") )
              {
                poBand->dfStdDev = atof(papszTokens[4]);
                poBand->minmaxmeanstddev |= 0x8;
              }
            }
          }

          CSLDestroy( papszTokens );
      }

      VSIFCloseL( fp );
    }

    return CE_None;
}


/************************************************************************/
/*                      GetImageRepFilename()                           */
/************************************************************************/

/* -------------------------------------------------------------------- */
/*  Check for IMAGE.REP (Spatiocarte Defense 1.0) or name_of_image.rep  */
/*  if it's a GIS-GeoSPOT image                                         */
/*  For the specification of SPDF (in French),                          */
/*   see http://eden.ign.fr/download/pub/doc/emabgi/spdf10.pdf/download */
/* -------------------------------------------------------------------- */

CPLString EHdrDataset::GetImageRepFilename(const char* pszFilename)
{
    VSIStatBufL sStatBuf;

    CPLString osPath = CPLGetPath( pszFilename );
    CPLString osName = CPLGetBasename( pszFilename );
    CPLString osREPFilename =
        CPLFormCIFilename( osPath, osName, "rep" );
    if( VSIStatL( (const char*)osREPFilename, &sStatBuf ) == 0 )
        return osREPFilename;

    if (EQUAL(CPLGetFilename(pszFilename), "imspatio.bil") ||
        EQUAL(CPLGetFilename(pszFilename), "haspatio.bil"))
    {
        CPLString pszImageRepFilename(CPLFormCIFilename( osPath, "image", "rep" ));
        if( VSIStatL( (const char*)pszImageRepFilename, &sStatBuf ) == 0 )
            return pszImageRepFilename;

        /* Try in the upper directories if not found in the BIL image directory */
        CPLString dirName(CPLGetDirname(osPath));
        if (CPLIsFilenameRelative((const char*)osPath))
        {
            char* cwd = CPLGetCurrentDir();
            if (cwd)
            {
                dirName = CPLFormFilename(cwd, (const char*)dirName, NULL);
                CPLFree(cwd);
            }
        }
        while (dirName[0] != 0 && EQUAL(dirName, ".") == FALSE && EQUAL(dirName, "/") == FALSE)
        {
            pszImageRepFilename = CPLFormCIFilename( (const char*)dirName, "image", "rep" );
            if( VSIStatL( (const char*)pszImageRepFilename, &sStatBuf ) == 0 )
                return pszImageRepFilename;

            /* Don't try to recurse above the 'image' subdirectory */
            if (EQUAL(dirName, "image"))
            {
                break;
            }
            dirName = CPLString(CPLGetDirname(dirName));
        }
    }
    return "";
}

/************************************************************************/
/*                            GetFileList()                             */
/************************************************************************/

char **EHdrDataset::GetFileList()

{
    VSIStatBufL sStatBuf;
    CPLString osPath = CPLGetPath( GetDescription() );
    CPLString osName = CPLGetBasename( GetDescription() );
    char **papszFileList = NULL;

    // Main data file, etc. 
    papszFileList = GDALPamDataset::GetFileList();

    // Header file.
    CPLString osFilename = CPLFormCIFilename( osPath, osName, "hdr" );
    papszFileList = CSLAddString( papszFileList, osFilename );

    // Statistics file
    osFilename = CPLFormCIFilename( osPath, osName, "stx" );
    if( VSIStatL( osFilename, &sStatBuf ) == 0 )
        papszFileList = CSLAddString( papszFileList, osFilename );
    
    // color table file.
    osFilename = CPLFormCIFilename( osPath, osName, "clr" );
    if( VSIStatL( osFilename, &sStatBuf ) == 0 )
        papszFileList = CSLAddString( papszFileList, osFilename );
    
    // projections file.
    osFilename = CPLFormCIFilename( osPath, osName, "prj" );
    if( VSIStatL( osFilename, &sStatBuf ) == 0 )
        papszFileList = CSLAddString( papszFileList, osFilename );
    
    CPLString imageRepFilename = GetImageRepFilename( GetDescription() );
    if (imageRepFilename[0])
        papszFileList = CSLAddString( papszFileList, (const char*)imageRepFilename );
    
    return papszFileList;
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *EHdrDataset::Open( GDALOpenInfo * poOpenInfo )

{
    int		i, bSelectedHDR;
    
/* -------------------------------------------------------------------- */
/*	We assume the user is pointing to the binary (ie. .bil) file.	*/
/* -------------------------------------------------------------------- */
    if( poOpenInfo->nHeaderBytes < 2 )
        return NULL;

/* -------------------------------------------------------------------- */
/*      Now we need to tear apart the filename to form a .HDR           */
/*      filename.                                                       */
/* -------------------------------------------------------------------- */
    CPLString osPath = CPLGetPath( poOpenInfo->pszFilename );
    CPLString osName = CPLGetBasename( poOpenInfo->pszFilename );
    CPLString osHDRFilename;

    if( poOpenInfo->papszSiblingFiles )
    {
        int iFile = CSLFindString(poOpenInfo->papszSiblingFiles, 
                              CPLFormFilename( NULL, osName, "hdr" ) );
        if( iFile < 0 ) // return if there is no corresponding .hdr file
            return NULL;
        
        osHDRFilename = 
            CPLFormFilename( osPath, poOpenInfo->papszSiblingFiles[iFile], 
                             NULL );
    }
    else
    {
        osHDRFilename = CPLFormCIFilename( osPath, osName, "hdr" );
    }

    bSelectedHDR = EQUAL( osHDRFilename, poOpenInfo->pszFilename );

/* -------------------------------------------------------------------- */
/*      Do we have a .hdr file?                                         */
/* -------------------------------------------------------------------- */
    FILE	*fp;

    fp = VSIFOpenL( osHDRFilename, "r" );
    
    if( fp == NULL )
    {
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Is this file an ESRI header file?  Read a few lines of text     */
/*      searching for something starting with nrows or ncols.           */
/* -------------------------------------------------------------------- */
    const char *	pszLine;
    int			nRows = -1, nCols = -1, nBands = 1;
    int			nSkipBytes = 0;
    double		dfULXMap=0.5, dfULYMap = 0.5, dfYLLCorner = -123.456;
    int                 bCenter = TRUE;
    double		dfXDim = 1.0, dfYDim = 1.0, dfNoData = 0.0;
    int			nLineCount = 0, bNoDataSet = FALSE;
    GDALDataType	eDataType = GDT_Byte;
    int                 nBits = -1;
    char		chByteOrder = 'M';
    char                chPixelType = 'N'; // not defined
    char                szLayout[10] = "BIL";
    char              **papszHDR = NULL;
    
    while( (pszLine = CPLReadLineL( fp )) )    
    {
        char	**papszTokens;

        nLineCount++;

        if( nLineCount > 50 || strlen(pszLine) > 1000 )
            break;

        papszHDR = CSLAddString( papszHDR, pszLine );

        papszTokens = CSLTokenizeStringComplex( pszLine, " \t", TRUE, FALSE );
        if( CSLCount( papszTokens ) < 2 )
        {
            CSLDestroy( papszTokens );
            continue;
        }
        
        if( EQUAL(papszTokens[0],"ncols") )
        {
            nCols = atoi(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"nrows") )
        {
            nRows = atoi(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"skipbytes") )
        {
            nSkipBytes = atoi(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"ulxmap") 
                 || EQUAL(papszTokens[0],"xllcorner") 
                 || EQUAL(papszTokens[0],"xllcenter") )
        {
            dfULXMap = atof(papszTokens[1]);
            if( EQUAL(papszTokens[0],"xllcorner") )
                bCenter = FALSE;
        }
        else if( EQUAL(papszTokens[0],"ulymap") )
        {
            dfULYMap = atof(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"yllcorner") 
                 || EQUAL(papszTokens[0],"yllcenter") )
        {
            dfYLLCorner = atof(papszTokens[1]);
            if( EQUAL(papszTokens[0],"yllcorner") )
                bCenter = FALSE;
        }
        else if( EQUAL(papszTokens[0],"xdim") )
        {
            dfXDim = atof(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"ydim") )
        {
            dfYDim = atof(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"cellsize") )
        {
            dfXDim = dfYDim = atof(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"nbands") )
        {
            nBands = atoi(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"layout") )
        {
            strncpy( szLayout, papszTokens[1], sizeof(szLayout)-1 );
        }
        else if( EQUAL(papszTokens[0],"NODATA_value") 
                 || EQUAL(papszTokens[0],"NODATA") )
        {
            dfNoData = atof(papszTokens[1]);
            bNoDataSet = TRUE;
        }
        else if( EQUAL(papszTokens[0],"NBITS") )
        {
            nBits = atoi(papszTokens[1]);
        }
        else if( EQUAL(papszTokens[0],"PIXELTYPE") )
        {
            chPixelType = toupper(papszTokens[1][0]);
        }
        else if( EQUAL(papszTokens[0],"byteorder") )
        {
            chByteOrder = toupper(papszTokens[1][0]);
        }

        CSLDestroy( papszTokens );
    }

    VSIFCloseL( fp );

/* -------------------------------------------------------------------- */
/*      Did we get the required keywords?  If not we return with        */
/*      this never having been considered to be a match. This isn't     */
/*      an error!                                                       */
/* -------------------------------------------------------------------- */
    if( nRows == -1 || nCols == -1 )
    {
        CSLDestroy( papszHDR );
        return NULL;
    }

    if (!GDALCheckDatasetDimensions(nCols, nRows) ||
        !GDALCheckBandCount(nBands, FALSE))
    {
        CSLDestroy( papszHDR );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Has the user selected the .hdr file to open?                    */
/* -------------------------------------------------------------------- */
    if( bSelectedHDR )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "The selected file is an ESRI BIL header file, but to\n"
                  "open ESRI BIL datasets, the data file should be selected\n"
                  "instead of the .hdr file.  Please try again selecting\n"
                  "the data file (often with the extension .bil) corresponding\n"
                  "to the header file: %s\n", 
                  poOpenInfo->pszFilename );
        CSLDestroy( papszHDR );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */
    EHdrDataset     *poDS;

    poDS = new EHdrDataset();

/* -------------------------------------------------------------------- */
/*      Capture some information from the file that is of interest.     */
/* -------------------------------------------------------------------- */
    poDS->nRasterXSize = nCols;
    poDS->nRasterYSize = nRows;
    poDS->papszHDR = papszHDR;

/* -------------------------------------------------------------------- */
/*      Open target binary file.                                        */
/* -------------------------------------------------------------------- */
    if( poOpenInfo->eAccess == GA_ReadOnly )
        poDS->fpImage = VSIFOpenL( poOpenInfo->pszFilename, "rb" );
    else
        poDS->fpImage = VSIFOpenL( poOpenInfo->pszFilename, "r+b" );

    if( poDS->fpImage == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed, 
                  "Failed to open %s with write permission.\n%s", 
                  osName.c_str(), VSIStrerror( errno ) );
        delete poDS;
        return NULL;
    }

    poDS->eAccess = poOpenInfo->eAccess;

/* -------------------------------------------------------------------- */
/*      Figure out the data type.                                       */
/* -------------------------------------------------------------------- */
    if( nBits == 16 )
    {
        if ( chPixelType == 'S' )
            eDataType = GDT_Int16;
        else
            eDataType = GDT_UInt16; // default
    }
    else if( nBits == 32 )
    {
        if( chPixelType == 'S' )
            eDataType = GDT_Int32;
        else if( chPixelType == 'F' )
            eDataType = GDT_Float32;
        else
            eDataType = GDT_UInt32; // default 
    }
    else if( nBits == 8 || nBits == -1 )
    {
        eDataType = GDT_Byte;
        nBits = 8;
    }
    else if( nBits < 8 && nBits >= 1 )
        eDataType = GDT_Byte;
    
    else
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "EHdr driver does not support %d NBITS value.", 
                  nBits );
        delete poDS;
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Compute the line offset.                                        */
/* -------------------------------------------------------------------- */
    int             nItemSize = GDALGetDataTypeSize(eDataType)/8;
    int             nPixelOffset;
    vsi_l_offset    nLineOffset, nBandOffset;

    if( EQUAL(szLayout,"BIP") )
    {
        nPixelOffset = nItemSize * nBands;
        nLineOffset = (vsi_l_offset)nPixelOffset * nCols;
        nBandOffset = (vsi_l_offset)nItemSize;
    }
    else if( EQUAL(szLayout,"BSQ") )
    {
        nPixelOffset = nItemSize;
        nLineOffset = (vsi_l_offset)nPixelOffset * nCols;
        nBandOffset = (vsi_l_offset)nLineOffset * nRows;
    }
    else /* assume BIL */
    {
        nPixelOffset = nItemSize;
        nLineOffset = (vsi_l_offset)nItemSize * nBands * nCols;
        nBandOffset = (vsi_l_offset)nItemSize * nCols;
    }
    
    poDS->SetDescription( poOpenInfo->pszFilename );
    poDS->PamInitialize();

/* -------------------------------------------------------------------- */
/*      Create band information objects.                                */
/* -------------------------------------------------------------------- */
    poDS->nBands = nBands;
    for( i = 0; i < poDS->nBands; i++ )
    {
        EHdrRasterBand	*poBand;
            
        poBand = 
            new EHdrRasterBand( poDS, i+1, poDS->fpImage,
                                nSkipBytes + nBandOffset * i, 
                                nPixelOffset, nLineOffset, eDataType,
#ifdef CPL_LSB                               
                                chByteOrder == 'I' || chByteOrder == 'L',
#else
                                chByteOrder == 'M',
#endif        
                                nBits);

        if( bNoDataSet )
            poBand->SetNoDataValue( dfNoData );
            
        poDS->SetBand( i+1, poBand );
    }

/* -------------------------------------------------------------------- */
/*      If we didn't get bounds in the .hdr, look for a worldfile.      */
/* -------------------------------------------------------------------- */
    if( dfYLLCorner != -123.456 )
    {
        if( bCenter )
            dfULYMap = dfYLLCorner + (nRows-1) * dfYDim;
        else
            dfULYMap = dfYLLCorner + nRows * dfYDim;
    }

    if( dfULYMap != 0.5 || dfULYMap != 0.5 || dfXDim != 1.0 || dfYDim != 1.0 )
    {
        poDS->bGotTransform = TRUE;

        if( bCenter )
        {
            poDS->adfGeoTransform[0] = dfULXMap - dfXDim * 0.5;
            poDS->adfGeoTransform[1] = dfXDim;
            poDS->adfGeoTransform[2] = 0.0;
            poDS->adfGeoTransform[3] = dfULYMap + dfYDim * 0.5;
            poDS->adfGeoTransform[4] = 0.0;
            poDS->adfGeoTransform[5] = - dfYDim;
        }
        else
        {
            poDS->adfGeoTransform[0] = dfULXMap;
            poDS->adfGeoTransform[1] = dfXDim;
            poDS->adfGeoTransform[2] = 0.0;
            poDS->adfGeoTransform[3] = dfULYMap;
            poDS->adfGeoTransform[4] = 0.0;
            poDS->adfGeoTransform[5] = - dfYDim;
        }
    }
    
    if( !poDS->bGotTransform )
        poDS->bGotTransform = 
            GDALReadWorldFile( poOpenInfo->pszFilename, 0, 
                               poDS->adfGeoTransform );

    if( !poDS->bGotTransform )
        poDS->bGotTransform = 
            GDALReadWorldFile( poOpenInfo->pszFilename, "wld", 
                               poDS->adfGeoTransform );

/* -------------------------------------------------------------------- */
/*      Check for a .prj file.                                          */
/* -------------------------------------------------------------------- */
    const char  *pszPrjFilename = CPLFormCIFilename( osPath, osName, "prj" );

    fp = VSIFOpenL( pszPrjFilename, "r" );
    if( fp != NULL )
    {
        char	**papszLines;
        OGRSpatialReference oSRS;

        VSIFCloseL( fp );
        
        papszLines = CSLLoad( pszPrjFilename );

        if( oSRS.importFromESRI( papszLines ) == OGRERR_NONE )
        {
            // If geographic values are in seconds, we must transform. 
            // Is there a code for minutes too? 
            if( oSRS.IsGeographic() 
                && EQUAL(OSR_GDS( papszLines, "Units", ""), "DS") )
            {
                poDS->adfGeoTransform[0] /= 3600.0;
                poDS->adfGeoTransform[1] /= 3600.0;
                poDS->adfGeoTransform[2] /= 3600.0;
                poDS->adfGeoTransform[3] /= 3600.0;
                poDS->adfGeoTransform[4] /= 3600.0;
                poDS->adfGeoTransform[5] /= 3600.0;
            }

            CPLFree( poDS->pszProjection );
            oSRS.exportToWkt( &(poDS->pszProjection) );
        }

        CSLDestroy( papszLines );
    }
    else
    {
/* -------------------------------------------------------------------- */
/*  Check for IMAGE.REP (Spatiocarte Defense 1.0) or name_of_image.rep  */
/*  if it's a GIS-GeoSPOT image                                         */
/*  For the specification of SPDF (in French),                          */
/*   see http://eden.ign.fr/download/pub/doc/emabgi/spdf10.pdf/download */
/* -------------------------------------------------------------------- */
        CPLString pszImageRepFilename = GetImageRepFilename(poOpenInfo->pszFilename );
        if (pszImageRepFilename[0])
        {
            fp = VSIFOpenL( (const char*)pszImageRepFilename, "r" );
        }
        if (fp != NULL)
        {
            char	**papszLines;
            char  **iter;
            int bUTM = FALSE;
            int bWGS84 = FALSE;
            int bNorth = FALSE;
            int bSouth = FALSE;
            int utmZone = 0;

            VSIFCloseL( fp );

            iter = papszLines = CSLLoad( pszPrjFilename );
            while (iter && *iter)
            {
                if (strncmp(*iter, "PROJ_ID", strlen("PROJ_ID")) == 0 &&
                    strstr(*iter, "UTM"))
                {
                    bUTM = TRUE;
                }
                else if (strncmp(*iter, "PROJ_ZONE", strlen("PROJ_ZONE")) == 0)
                {
                    char* c = strchr(*iter, '"');
                    if (c)
                    {
                        c++;
                        if (*c >= '0' && *c <= '9')
                        {
                            utmZone = atoi(c);
                            if (utmZone >= 1 && utmZone <= 60)
                            {
                                if (strstr(*iter, "Nord") || strstr(*iter, "NORD"))
                                {
                                    bNorth = TRUE;
                                }
                                else if (strstr(*iter, "Sud") || strstr(*iter, "SUD"))
                                {
                                    bSouth = TRUE;
                                }
                            }
                        }
                    }
                }
                else if (strncmp(*iter, "PROJ_CODE", strlen("PROJ_CODE")) == 0 &&
                         strstr(*iter, "FR-MINDEF"))
                {
                    char* c = strchr(*iter, 'A');
                    if (c)
                    {
                        c++;
                        if (*c >= '0' && *c <= '9')
                        {
                            utmZone = atoi(c);
                            if (utmZone >= 1 && utmZone <= 60)
                            {
                                if (c[1] == 'N' || c[2] == 'N')
                                {
                                    bNorth = TRUE;
                                }
                                else if (c[1] == 'S' || c[2] == 'S')
                                {
                                    bSouth = TRUE;
                                }
                            }
                        }
                    }
                }
                else if (strncmp(*iter, "HORIZ_DATUM", strlen("HORIZ_DATUM")) == 0 &&
                         (strstr(*iter, "WGS 84") || strstr(*iter, "WGS84")))
                {
                    bWGS84 = TRUE;
                }
                else if (strncmp(*iter, "MAP_NUMBER", strlen("MAP_NUMBER")) == 0)
                {
                    char* c = strchr(*iter, '"');
                    if (c)
                    {
                        char* c2 = strchr(c+1, '"');
                        if (c2) *c2 = 0;
                        poDS->SetMetadataItem("SPDF_MAP_NUMBER", c + 1);
                    }
                }
                else if (strncmp(*iter, "PRODUCTION_DATE", strlen("PRODUCTION_DATE")) == 0)
                {
                    char* c = *iter + strlen("PRODUCTION_DATE");
                    while(*c == ' ')
                        c++;
                    if (*c)
                    {
                        poDS->SetMetadataItem("SPDF_PRODUCTION_DATE", c );
                    }
                }
                iter++;
            }

            if (utmZone != 0 && bUTM && bWGS84 && (bNorth || bSouth))
            {
                char projCSStr[64];
                OGRSpatialReference oSRS;

                sprintf(projCSStr, "WGS 84 / UTM zone %d%c",
                        utmZone, (bNorth) ? 'N' : 'S');
                oSRS.SetProjCS(projCSStr);
                oSRS.SetWellKnownGeogCS( "WGS84" );
                oSRS.SetUTM(utmZone, bNorth);
                oSRS.SetAuthority("PROJCS", "EPSG", ((bNorth) ? 32600 : 32700) + utmZone);
                oSRS.AutoIdentifyEPSG();

                CPLFree( poDS->pszProjection );
                oSRS.exportToWkt( &(poDS->pszProjection) );
            }
            else
            {
                CPLError( CE_Warning, CPLE_NotSupported, "Cannot retrive projection from IMAGE.REP");
            }

            CSLDestroy( papszLines );
        }
    }

/* -------------------------------------------------------------------- */
/*      Check for a color table.                                        */
/* -------------------------------------------------------------------- */
    const char  *pszCLRFilename = CPLFormCIFilename( osPath, osName, "clr" );
    
    fp = VSIFOpenL( pszCLRFilename, "r" );
    if( fp != NULL )
    {
        GDALColorTable oColorTable;

        for(i = 0;;)
        {
            const char  *pszLine =  CPLReadLineL(fp);
            if ( !pszLine )
                break;

            if( *pszLine == '#' )
                continue;

            char	**papszValues = CSLTokenizeString2(pszLine, "\t ",
                                                           CSLT_HONOURSTRINGS);
            GDALColorEntry oEntry;

            if ( CSLCount(papszValues) == 4 )
            {
                oEntry.c1 = atoi( papszValues[1] ); // Red
                oEntry.c2 = atoi( papszValues[2] ); // Green
                oEntry.c3 = atoi( papszValues[3] ); // Blue
                oEntry.c4 = 255;

                oColorTable.SetColorEntry( i++, &oEntry );
            }

            CSLDestroy( papszValues );
        }
    
        VSIFCloseL( fp );

        for( i = 1; i <= poDS->nBands; i++ )
        {
            GDALRasterBand *poBand = poDS->GetRasterBand( i );
            poBand->SetColorTable( &oColorTable );
            poBand->SetColorInterpretation( GCI_PaletteIndex );
        }
    }

/* -------------------------------------------------------------------- */
/*      Read statistics (.STX)                                          */
/* -------------------------------------------------------------------- */
    poDS->ReadSTX();

/* -------------------------------------------------------------------- */
/*      Check for overviews.                                            */
/* -------------------------------------------------------------------- */
    poDS->oOvManager.Initialize( poDS, poOpenInfo->pszFilename );

/* -------------------------------------------------------------------- */
/*      Initialize any PAM information.                                 */
/* -------------------------------------------------------------------- */
    poDS->TryLoadXML();
    
    return( poDS );
}

/************************************************************************/
/*                               Create()                               */
/************************************************************************/

GDALDataset *EHdrDataset::Create( const char * pszFilename,
                                  int nXSize, int nYSize, int nBands,
                                  GDALDataType eType,
                                  char **papszParmList )

{
/* -------------------------------------------------------------------- */
/*      Verify input options.                                           */
/* -------------------------------------------------------------------- */
    if (nBands <= 0)
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "EHdr driver does not support %d bands.\n", nBands);
        return NULL;
    }

    if( eType != GDT_Byte && eType != GDT_Float32 && eType != GDT_UInt16
        && eType != GDT_Int16 && eType != GDT_Int32 && eType != GDT_UInt32 )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
              "Attempt to create ESRI .hdr labelled dataset with an illegal\n"
              "data type (%s).\n",
              GDALGetDataTypeName(eType) );

        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Try to create the file.                                         */
/* -------------------------------------------------------------------- */
    FILE	*fp;

    fp = VSIFOpenL( pszFilename, "wb" );

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
    VSIFWriteL( (void *) "\0\0", 2, 1, fp );
    VSIFCloseL( fp );

/* -------------------------------------------------------------------- */
/*      Create the hdr filename.                                        */
/* -------------------------------------------------------------------- */
    char *pszHdrFilename;

    pszHdrFilename = 
        CPLStrdup( CPLResetExtension( pszFilename, "hdr" ) );

/* -------------------------------------------------------------------- */
/*      Open the file.                                                  */
/* -------------------------------------------------------------------- */
    fp = VSIFOpenL( pszHdrFilename, "wt" );
    if( fp == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed,
                  "Attempt to create file `%s' failed.\n",
                  pszHdrFilename );
        CPLFree( pszHdrFilename );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Decide how many bits the file should have.                      */
/* -------------------------------------------------------------------- */
    int nRowBytes;
    int nBits = GDALGetDataTypeSize(eType);

    if( CSLFetchNameValue( papszParmList, "NBITS" ) != NULL )
        nBits = atoi(CSLFetchNameValue( papszParmList, "NBITS" ));

    nRowBytes = (nBits * nXSize + 7) / 8;
    
/* -------------------------------------------------------------------- */
/*      Write out the raw definition for the dataset as a whole.        */
/* -------------------------------------------------------------------- */
    VSIFPrintfL( fp, "BYTEORDER      I\n" );
    VSIFPrintfL( fp, "LAYOUT         BIL\n" );
    VSIFPrintfL( fp, "NROWS          %d\n", nYSize );
    VSIFPrintfL( fp, "NCOLS          %d\n", nXSize );
    VSIFPrintfL( fp, "NBANDS         %d\n", nBands );
    VSIFPrintfL( fp, "NBITS          %d\n", nBits );
    VSIFPrintfL( fp, "BANDROWBYTES   %d\n", nRowBytes );
    VSIFPrintfL( fp, "TOTALROWBYTES  %d\n", nRowBytes * nBands );
    
    if( eType == GDT_Float32 )
        VSIFPrintfL( fp, "PIXELTYPE      FLOAT\n");
    else if( eType == GDT_Int16 || eType == GDT_Int32 )
        VSIFPrintfL( fp, "PIXELTYPE      SIGNEDINT\n");
    else
        VSIFPrintfL( fp, "PIXELTYPE      UNSIGNEDINT\n");

/* -------------------------------------------------------------------- */
/*      Cleanup                                                         */
/* -------------------------------------------------------------------- */
    VSIFCloseL( fp );

    CPLFree( pszHdrFilename );

    return (GDALDataset *) GDALOpen( pszFilename, GA_Update );
}

/************************************************************************/
/*                             CreateCopy()                             */
/************************************************************************/

GDALDataset *EHdrDataset::CreateCopy( const char * pszFilename, 
                                      GDALDataset * poSrcDS, 
                                      int bStrict, char ** papszOptions,
                                      GDALProgressFunc pfnProgress,
                                      void * pProgressData )

{
    char **papszAdjustedOptions = CSLDuplicate( papszOptions );
    GDALDataset *poOutDS;

    int nBands = poSrcDS->GetRasterCount();
    if (nBands == 0)
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "EHdr driver does not support source dataset with zero band.\n");
        return NULL;
    }

    if( poSrcDS->GetRasterBand(1)->GetMetadataItem( "NBITS", 
                                                    "IMAGE_STRUCTURE" ) !=NULL
        && CSLFetchNameValue( papszOptions, "NBITS" ) == NULL )
    {
        papszAdjustedOptions = 
            CSLSetNameValue( papszAdjustedOptions, 
                             "NBITS", 
                             poSrcDS->GetRasterBand(1)->GetMetadataItem("NBITS","IMAGE_STRUCTURE") );
    }
    
    GDALDriver	*poDriver = (GDALDriver *) GDALGetDriverByName( "EHdr" );

    poOutDS = poDriver->DefaultCreateCopy( pszFilename, poSrcDS, bStrict,
                                           papszAdjustedOptions, 
                                           pfnProgress, pProgressData );
    CSLDestroy( papszAdjustedOptions );

    return poOutDS;
}
    
/************************************************************************/
/*                           GetMinimum()                               */
/************************************************************************/

double EHdrRasterBand::GetMinimum( int *pbSuccess )
{
    if( pbSuccess != NULL )
        *pbSuccess = (minmaxmeanstddev & 0x1) != 0;

    if( minmaxmeanstddev & 0x1 )
      return dfMin;

    return RawRasterBand::GetMinimum( pbSuccess );
}

/************************************************************************/
/*                           GetMaximum()                               */
/************************************************************************/

double EHdrRasterBand::GetMaximum( int *pbSuccess )
{
    if( pbSuccess != NULL )
        *pbSuccess = (minmaxmeanstddev & 0x2) != 0;

    if( minmaxmeanstddev & 0x2 )
      return dfMax;

    return RawRasterBand::GetMaximum( pbSuccess );
}

/************************************************************************/
/*                           GetStatistics()                            */
/************************************************************************/

CPLErr EHdrRasterBand::GetStatistics( int bApproxOK, int bForce, double *pdfMin, double *pdfMax, double *pdfMean, double *pdfStdDev )
{
    if( (minmaxmeanstddev & 0xf) == 0xf)
    {
        if ( pdfMin ) *pdfMin = dfMin;
        if ( pdfMax ) *pdfMax = dfMax;
        if ( pdfMean ) *pdfMean = dfMean;
        if ( pdfStdDev ) *pdfStdDev = dfStdDev;
        return CE_None;
    }

    CPLErr eErr = RawRasterBand::GetStatistics( bApproxOK, bForce, 
                                                pdfMin, pdfMax, 
                                                pdfMean, pdfStdDev );

    if( eErr == CE_None )
    {
        EHdrDataset* poEDS = (EHdrDataset *) poDS;

        if( pdfMin && pdfMax )
        {
            dfMin = *pdfMin;
            dfMax = *pdfMax;
            
            minmaxmeanstddev |= 0x3;
            poEDS->bSTXDirty = TRUE;
        }

        if( *pdfMean )
        {
            dfMean = *pdfMean;
            minmaxmeanstddev |= 0x4;
            poEDS->bSTXDirty = TRUE;
        }

        if( *pdfStdDev )
        {
            dfStdDev = *pdfStdDev;
            minmaxmeanstddev |= 0x8;
            poEDS->bSTXDirty = TRUE;
        }
    }

    return eErr;
}

/************************************************************************/
/*                           SetStatistics()                            */
/************************************************************************/

CPLErr EHdrRasterBand::SetStatistics( double dfMin, double dfMax, double dfMean, double dfStdDev )
{
    this->dfMin = dfMin;
    this->dfMax = dfMax;
    this->dfMean = dfMean;
    this->dfStdDev = dfStdDev;
    // marks stats valid
    minmaxmeanstddev = 0xf;
    // marks dataset stats dirty
    ((EHdrDataset*)poDS)->bSTXDirty = TRUE;
   
    return CE_None;
}

/************************************************************************/
/*                         GDALRegister_EHdr()                          */
/************************************************************************/

void GDALRegister_EHdr()

{
    GDALDriver	*poDriver;

    if( GDALGetDriverByName( "EHdr" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "EHdr" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "ESRI .hdr Labelled" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, 
                                   "frmt_various.html#EHdr" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONDATATYPES, 
                                   "Byte Int16 UInt16 Int32 UInt32 Float32" );

        poDriver->SetMetadataItem( GDAL_DMD_CREATIONOPTIONLIST, 
"<CreationOptionList>"
"   <Option name='NBITS' type='int' description='Special pixel bits (1-7)'/>"
"</CreationOptionList>" );

        poDriver->SetMetadataItem( GDAL_DCAP_VIRTUALIO, "YES" );
        poDriver->pfnOpen = EHdrDataset::Open;
        poDriver->pfnCreate = EHdrDataset::Create;
        poDriver->pfnCreateCopy = EHdrDataset::CreateCopy;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}
