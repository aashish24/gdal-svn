/******************************************************************************
 * $Id$
 *
 * Project:  NITF Read/Write Translator
 * Purpose:  GDALDataset/GDALRasterBand implementation on top of "nitflib".
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2002, Frank Warmerdam
 *
 * Portions Copyright (c) Her majesty the Queen in right of Canada as
 * represented by the Minister of National Defence, 2006.
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

#include "gdal_pam.h"
#include "nitflib.h"
#include "ogr_spatialref.h"
#include "cpl_string.h"
#include "cpl_csv.h"

CPL_CVSID("$Id$");

static void NITFPatchImageLength( const char *pszFilename,
                                  long nImageOffset, 
                                  GIntBig nPixelCount, const char *pszIC );
static void NITFWriteTextSegments( const char *pszFilename, char **papszList );

static GDALDataset *poWritableJ2KDataset = NULL;
static CPLErr NITFSetColorInterpretation( NITFImage *psImage, 
                                          int nBand,
                                          GDALColorInterp eInterp );
#ifdef JPEG_SUPPORTED
static int NITFWriteJPEGImage( GDALDataset *, FILE *, int, char **,
                               GDALProgressFunc pfnProgress, 
                               void * pProgressData );
#endif

/************************************************************************/
/* ==================================================================== */
/*				NITFDataset				*/
/* ==================================================================== */
/************************************************************************/

class NITFRasterBand;

class NITFDataset : public GDALPamDataset
{
    friend class NITFRasterBand;

    NITFFile    *psFile;
    NITFImage   *psImage;

    GDALDataset *poJ2KDataset;
    int         bJP2Writing;

    GDALDataset *poJPEGDataset;

    int         bGotGeoTransform;
    double      adfGeoTransform[6];

    char        *pszProjection;

    int         nGCPCount;
    GDAL_GCP    *pasGCPList;
    char        *pszGCPProjection;

    GDALMultiDomainMetadata oSpecialMD;

    void         InitializeCGMMetadata();
    void         InitializeTextMetadata();
    void         InitializeTREMetadata();

    int         *panJPEGBlockOffset;
    GByte       *pabyJPEGBlock;
    int          nQLevel;

    int          ScanJPEGQLevel( GUInt32 *pnDataStart );
    CPLErr       ScanJPEGBlocks( void );
    CPLErr       ReadJPEGBlock( int, int );
    void         CheckGeoSDEInfo();

    int          nIMIndex;
    CPLString    osNITFFilename;

  public:
                 NITFDataset();
                 ~NITFDataset();

    virtual CPLErr AdviseRead( int nXOff, int nYOff, int nXSize, int nYSize,
                               int nBufXSize, int nBufYSize, 
                               GDALDataType eDT, 
                               int nBandCount, int *panBandList,
                               char **papszOptions );

    virtual CPLErr IRasterIO( GDALRWFlag, int, int, int, int,
                              void *, int, int, GDALDataType,
                              int, int *, int, int, int );

    virtual const char *GetProjectionRef(void);
    virtual CPLErr SetProjection( const char * );
    virtual CPLErr GetGeoTransform( double * );
    virtual CPLErr SetGeoTransform( double * );

    virtual int    GetGCPCount();
    virtual const char *GetGCPProjection();
    virtual const GDAL_GCP *GetGCPs();

    virtual char      **GetMetadata( const char * pszDomain = "" );
    virtual const char *GetMetadataItem( const char * pszName,
                                         const char * pszDomain = "" );
    virtual void   FlushCache();

    static int          Identify( GDALOpenInfo * );
    static GDALDataset *Open( GDALOpenInfo * );
    static GDALDataset *
    NITFCreateCopy( const char *pszFilename, GDALDataset *poSrcDS,
                    int bStrict, char **papszOptions, 
                    GDALProgressFunc pfnProgress, void * pProgressData );

};

/************************************************************************/
/* ==================================================================== */
/*                            NITFRasterBand                             */
/* ==================================================================== */
/************************************************************************/

class NITFRasterBand : public GDALPamRasterBand
{
    friend class NITFDataset;

    NITFImage   *psImage;

    GDALColorTable *poColorTable;

  public:
                   NITFRasterBand( NITFDataset *, int );
                  ~NITFRasterBand();

    virtual CPLErr IReadBlock( int, int, void * );
    virtual CPLErr IWriteBlock( int, int, void * );

    virtual GDALColorInterp GetColorInterpretation();
    virtual CPLErr SetColorInterpretation( GDALColorInterp );
    virtual GDALColorTable *GetColorTable();
    virtual CPLErr SetColorTable( GDALColorTable * ); 
    virtual double GetNoDataValue( int *pbSuccess = NULL );
};

/************************************************************************/
/*                           NITFRasterBand()                           */
/************************************************************************/

NITFRasterBand::NITFRasterBand( NITFDataset *poDS, int nBand )

{
    NITFBandInfo *psBandInfo = poDS->psImage->pasBandInfo + nBand - 1;

    this->poDS = poDS;
    this->nBand = nBand;

    this->eAccess = poDS->eAccess;
    this->psImage = poDS->psImage;

/* -------------------------------------------------------------------- */
/*      Translate data type(s).                                         */
/* -------------------------------------------------------------------- */
    if( psImage->nBitsPerSample <= 8 )
        eDataType = GDT_Byte;
    else if( psImage->nBitsPerSample == 16 
             && EQUAL(psImage->szPVType,"SI") )
        eDataType = GDT_Int16;
    else if( psImage->nBitsPerSample == 16 )
        eDataType = GDT_UInt16;
    else if( psImage->nBitsPerSample == 12 )
        eDataType = GDT_UInt16;
    else if( psImage->nBitsPerSample == 32 
             && EQUAL(psImage->szPVType,"SI") )
        eDataType = GDT_Int32;
    else if( psImage->nBitsPerSample == 32 
             && EQUAL(psImage->szPVType,"R") )
        eDataType = GDT_Float32;
    else if( psImage->nBitsPerSample == 32 )
        eDataType = GDT_UInt32;
    else if( psImage->nBitsPerSample == 64 
             && EQUAL(psImage->szPVType,"R") )
        eDataType = GDT_Float64;
    else
    {
        eDataType = GDT_Unknown;
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "Unsupported combination of PVTYPE(%s) and NBPP(%d).",
                  psImage->szPVType, psImage->nBitsPerSample );
    }

/* -------------------------------------------------------------------- */
/*      Work out block size. If the image is all one big block we       */
/*      handle via the scanline access API.                             */
/* -------------------------------------------------------------------- */
    if( psImage->nBlocksPerRow == 1 
        && psImage->nBlocksPerColumn == 1
        && psImage->nBitsPerSample != 1  
        && EQUAL(psImage->szIC,"NC") )
    {
        nBlockXSize = psImage->nBlockWidth;
        nBlockYSize = 1;
    }
    else
    {
        nBlockXSize = psImage->nBlockWidth;
        nBlockYSize = psImage->nBlockHeight;
    }

/* -------------------------------------------------------------------- */
/*      Do we have a color table?                                       */
/* -------------------------------------------------------------------- */
    poColorTable = NULL;

    if( psBandInfo->nSignificantLUTEntries > 0 )
    {
        int  iColor;

        poColorTable = new GDALColorTable();

        for( iColor = 0; iColor < psBandInfo->nSignificantLUTEntries; iColor++)
        {
            GDALColorEntry sEntry;

            sEntry.c1 = psBandInfo->pabyLUT[  0 + iColor];
            sEntry.c2 = psBandInfo->pabyLUT[256 + iColor];
            sEntry.c3 = psBandInfo->pabyLUT[512 + iColor];
            sEntry.c4 = 255;

            poColorTable->SetColorEntry( iColor, &sEntry );
        }

        if (psImage->bNoDataSet)
        {
            GDALColorEntry sEntry;
            sEntry.c1 = sEntry.c2 = sEntry.c3 = sEntry.c4 = 0;
            poColorTable->SetColorEntry( psImage->nNoDataValue, &sEntry );
        }
    }

/* -------------------------------------------------------------------- */
/*      We create a color table for 1 bit data too...                   */
/* -------------------------------------------------------------------- */
    if( poColorTable == NULL && psImage->nBitsPerSample == 1 )
    {
        GDALColorEntry sEntry;

        poColorTable = new GDALColorTable();

        sEntry.c1 = 0;
        sEntry.c2 = 0;
        sEntry.c3 = 0;
        sEntry.c4 = 255;
        poColorTable->SetColorEntry( 0, &sEntry );

        sEntry.c1 = 255;
        sEntry.c2 = 255;
        sEntry.c3 = 255;
        sEntry.c4 = 255;
        poColorTable->SetColorEntry( 1, &sEntry );
    }

    if( psImage->nBitsPerSample == 1 )
        SetMetadataItem( "NBITS", "1", "IMAGE_STRUCTURE" );
    if( psImage->nBitsPerSample == 12 )
        SetMetadataItem( "NBITS", "12", "IMAGE_STRUCTURE" );
}

/************************************************************************/
/*                          ~NITFRasterBand()                           */
/************************************************************************/

NITFRasterBand::~NITFRasterBand()

{
    if( poColorTable != NULL )
        delete poColorTable;
}

/************************************************************************/
/*                             IReadBlock()                             */
/************************************************************************/

CPLErr NITFRasterBand::IReadBlock( int nBlockXOff, int nBlockYOff,
                                   void * pImage )

{
    int  nBlockResult;
    NITFDataset *poGDS = (NITFDataset *) poDS;

/* -------------------------------------------------------------------- */
/*      Special case for JPEG blocks.                                   */
/* -------------------------------------------------------------------- */
    if( EQUAL(psImage->szIC,"C3") || EQUAL(psImage->szIC,"M3") )
    {
        CPLErr eErr = poGDS->ReadJPEGBlock( nBlockXOff, nBlockYOff );
        int nBlockBandSize = psImage->nBlockWidth*psImage->nBlockHeight;

        if( eErr != CE_None )
            return eErr;

        memcpy( pImage, 
                poGDS->pabyJPEGBlock + (nBand - 1) * nBlockBandSize, 
                nBlockBandSize );

        return eErr;
    }

/* -------------------------------------------------------------------- */
/*      Read the line/block                                             */
/* -------------------------------------------------------------------- */
    if( nBlockYSize == 1 )
    {
        nBlockResult = 
            NITFReadImageLine(psImage, nBlockYOff, nBand, pImage);
    }
    else
    {
        nBlockResult = 
            NITFReadImageBlock(psImage, nBlockXOff, nBlockYOff, nBand, pImage);
    }

/* -------------------------------------------------------------------- */
/*      If the image is 1 bit, expand it now to 8bit.                   */
/* -------------------------------------------------------------------- */
    if( nBlockResult == BLKREAD_OK && psImage->nBitsPerSample == 1 )
    {
        int nPixelCount = psImage->nBlockWidth * psImage->nBlockHeight;
        int i;
        GByte *pabyImage = (GByte *) pImage;

        for( i = nPixelCount-1; i >= 0; i-- )
        {
            if( pabyImage[i>>3] & (0x80 >> (i&7)) )
                pabyImage[i] = 1;
            else
                pabyImage[i] = 0;
        }
        
        return CE_None;
    }

/* -------------------------------------------------------------------- */
/*      if data type is 12 bit, we expand to 16bit.                     */
/* -------------------------------------------------------------------- */
    if( nBlockResult == BLKREAD_OK && psImage->nBitsPerSample == 12 )
    {
        int nPixelCount = psImage->nBlockWidth * psImage->nBlockHeight;
        int i;
        GByte *pabyImage = (GByte *) pImage;
        GUInt16 *panImage = (GUInt16 *) pImage;

        for( i = nPixelCount-1; i >= 0; i-- )
        {
            if( i % 2 == 0 )
            {
                int iOffset = i*3 / 2;

                panImage[i] = pabyImage[iOffset] 
                    + (pabyImage[iOffset+1] & 0xf0) * 16;
            }
            else
            {
                int iOffset = i*3 / 2;

                panImage[i] = (pabyImage[iOffset] & 0x0f) * 16
                    + (pabyImage[iOffset+1] & 0xf0) / 16
                    + (pabyImage[iOffset+1] & 0x0f) * 256;
            }
        }
        
        return CE_None;
    }


/* -------------------------------------------------------------------- */
/*      Return result.                                                  */
/* -------------------------------------------------------------------- */
    if( nBlockResult == BLKREAD_OK )
        return CE_None;
    else if( nBlockResult == BLKREAD_FAIL )
        return CE_Failure;
    else /* nBlockResult == BLKREAD_NULL */ 
    {
        if( psImage->bNoDataSet && eDataType == GDT_Byte )
            memset( pImage, psImage->nNoDataValue, 
                    psImage->nBlockWidth*psImage->nBlockHeight);
        else
            memset( pImage, 0, 
                    psImage->nBlockWidth*psImage->nBlockHeight*(GDALGetDataTypeSize(eDataType)/8));

        return CE_None;
    }
}

/************************************************************************/
/*                            IWriteBlock()                             */
/************************************************************************/

CPLErr NITFRasterBand::IWriteBlock( int nBlockXOff, int nBlockYOff,
                                    void * pImage )
    
{
    int  nBlockResult;

/* -------------------------------------------------------------------- */
/*      Write the line/block                                            */
/* -------------------------------------------------------------------- */
    if( nBlockYSize == 1 )
    {
        nBlockResult = 
            NITFWriteImageLine(psImage, nBlockYOff, nBand, pImage);
    }
    else
    {
        nBlockResult = 
            NITFWriteImageBlock(psImage, nBlockXOff, nBlockYOff, nBand,pImage);
    }

    if( nBlockResult == BLKREAD_OK )
        return CE_None;
    else
        return CE_Failure;
}

/************************************************************************/
/*                           GetNoDataValue()                           */
/************************************************************************/

double NITFRasterBand::GetNoDataValue( int *pbSuccess )

{
    if( pbSuccess != NULL )
        *pbSuccess = psImage->bNoDataSet;

    if( psImage->bNoDataSet )
        return psImage->nNoDataValue;
    else
        return GDALPamRasterBand::GetNoDataValue( pbSuccess );
}

/************************************************************************/
/*                       GetColorInterpretation()                       */
/************************************************************************/

GDALColorInterp NITFRasterBand::GetColorInterpretation()

{
    NITFBandInfo *psBandInfo = psImage->pasBandInfo + nBand - 1;

    if( poColorTable != NULL )
        return GCI_PaletteIndex;
    
    if( EQUAL(psBandInfo->szIREPBAND,"R") )
        return GCI_RedBand;
    if( EQUAL(psBandInfo->szIREPBAND,"G") )
        return GCI_GreenBand;
    if( EQUAL(psBandInfo->szIREPBAND,"B") )
        return GCI_BlueBand;
    if( EQUAL(psBandInfo->szIREPBAND,"M") )
        return GCI_GrayIndex;
    if( EQUAL(psBandInfo->szIREPBAND,"Y") )
        return GCI_YCbCr_YBand;
    if( EQUAL(psBandInfo->szIREPBAND,"Cb") )
        return GCI_YCbCr_CbBand;
    if( EQUAL(psBandInfo->szIREPBAND,"Cr") )
        return GCI_YCbCr_CrBand;

    return GCI_Undefined;
}

/************************************************************************/
/*                     NITFSetColorInterpretation()                     */
/************************************************************************/

static CPLErr NITFSetColorInterpretation( NITFImage *psImage, 
                                          int nBand,
                                          GDALColorInterp eInterp )

{
    NITFBandInfo *psBandInfo = psImage->pasBandInfo + nBand - 1;
    const char *pszREP = NULL;
    GUInt32 nOffset;

    if( eInterp == GCI_RedBand )
        pszREP = "R";
    else if( eInterp == GCI_GreenBand )
        pszREP = "G";
    else if( eInterp == GCI_BlueBand )
        pszREP = "B";
    else if( eInterp == GCI_GrayIndex )
        pszREP = "M";
    else if( eInterp == GCI_YCbCr_YBand )
        pszREP = "Y";
    else if( eInterp == GCI_YCbCr_CbBand )
        pszREP = "Cb";
    else if( eInterp == GCI_YCbCr_CrBand )
        pszREP = "Cr";
    else if( eInterp == GCI_Undefined )
        return CE_None;

    if( pszREP == NULL )
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "Requested color interpretation (%s) not supported in NITF.",
                  GDALGetColorInterpretationName( eInterp ) );
        return CE_Failure;
    }

/* -------------------------------------------------------------------- */
/*      Where does this go in the file?                                 */
/* -------------------------------------------------------------------- */
    strcpy( psBandInfo->szIREPBAND, pszREP );
    nOffset = NITFIHFieldOffset( psImage, "IREPBAND" );

    if( nOffset != 0 )
        nOffset += (nBand - 1) * 13;
    
/* -------------------------------------------------------------------- */
/*      write it (space padded).                                        */
/* -------------------------------------------------------------------- */
    char szPadded[4];
    strcpy( szPadded, pszREP );
    strcat( szPadded, " " );
    
    if( nOffset != 0 )
    {
        if( VSIFSeekL( psImage->psFile->fp, nOffset, SEEK_SET ) != 0 
            || VSIFWriteL( (void *) szPadded, 1, 2, psImage->psFile->fp ) != 2 )
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "IO failure writing new IREPBAND value to NITF file." );
            return CE_Failure;
        }
    }
    
    return CE_None;
}

/************************************************************************/
/*                       SetColorInterpretation()                       */
/************************************************************************/

CPLErr NITFRasterBand::SetColorInterpretation( GDALColorInterp eInterp )

{
    return NITFSetColorInterpretation( psImage, nBand, eInterp );
}

/************************************************************************/
/*                           GetColorTable()                            */
/************************************************************************/

GDALColorTable *NITFRasterBand::GetColorTable()

{
    return poColorTable;
}

/************************************************************************/
/*                           SetColorTable()                            */
/************************************************************************/

CPLErr NITFRasterBand::SetColorTable( GDALColorTable *poNewCT )

{
    if( poNewCT == NULL )
        return CE_Failure;

    GByte abyNITFLUT[768];
    int   i;
    int   nCount = MIN(256,poNewCT->GetColorEntryCount());

    memset( abyNITFLUT, 0, 768 );
    for( i = 0; i < nCount; i++ )
    {
        GDALColorEntry sEntry;

        poNewCT->GetColorEntryAsRGB( i, &sEntry );
        abyNITFLUT[i    ] = (GByte) sEntry.c1;
        abyNITFLUT[i+256] = (GByte) sEntry.c2;
        abyNITFLUT[i+512] = (GByte) sEntry.c3;
    }

    if( NITFWriteLUT( psImage, nBand, nCount, abyNITFLUT ) )
        return CE_None;
    else
        return CE_Failure;
}

/************************************************************************/
/* ==================================================================== */
/*                             NITFDataset                              */
/* ==================================================================== */
/************************************************************************/

/************************************************************************/
/*                            NITFDataset()                             */
/************************************************************************/

NITFDataset::NITFDataset()

{
    psFile = NULL;
    psImage = NULL;
    bGotGeoTransform = FALSE;
    pszProjection = CPLStrdup("");
    poJ2KDataset = NULL;
    bJP2Writing = FALSE;
    poJPEGDataset = NULL;

    panJPEGBlockOffset = NULL;
    pabyJPEGBlock = NULL;
    nQLevel = 0;

    nGCPCount = 0;
    pasGCPList = NULL;
    pszGCPProjection = NULL;

    adfGeoTransform[0] = 0.0;
    adfGeoTransform[1] = 1.0;
    adfGeoTransform[2] = 0.0;
    adfGeoTransform[3] = 0.0;
    adfGeoTransform[4] = 0.0;
    adfGeoTransform[5] = 1.0;
}

/************************************************************************/
/*                            ~NITFDataset()                            */
/************************************************************************/

NITFDataset::~NITFDataset()

{
    FlushCache();

/* -------------------------------------------------------------------- */
/*      If we have been writing to a JPEG2000 file, check if the        */
/*      color interpretations were set.  If so, apply the settings      */
/*      to the NITF file.                                               */
/* -------------------------------------------------------------------- */
    if( poJ2KDataset != NULL && bJP2Writing )
    {
        int i;

        for( i = 0; i < nBands && papoBands != NULL; i++ )
        {
            if( papoBands[i]->GetColorInterpretation() != GCI_Undefined )
                NITFSetColorInterpretation( psImage, i+1, 
                                papoBands[i]->GetColorInterpretation() );
        }
    }

/* -------------------------------------------------------------------- */
/*      Close the underlying NITF file.                                 */
/* -------------------------------------------------------------------- */
    int nImageStart = -1;
    if( psFile != NULL )
    {
        if (psFile->nSegmentCount > 0)
            nImageStart = psFile->pasSegmentInfo[0].nSegmentStart;

        NITFClose( psFile );
        psFile = NULL;
    }

/* -------------------------------------------------------------------- */
/*      Free datastructures.                                            */
/* -------------------------------------------------------------------- */
    CPLFree( pszProjection );

    GDALDeinitGCPs( nGCPCount, pasGCPList );
    CPLFree( pasGCPList );

/* -------------------------------------------------------------------- */
/*      If we have a jpeg2000 output file, make sure it gets closed     */
/*      and flushed out.                                                */
/* -------------------------------------------------------------------- */
    if( poJ2KDataset != NULL )
    {
        int i;

        GDALClose( (GDALDatasetH) poJ2KDataset );
        
        // the bands are really jpeg2000 bands ... remove them 
        // from the NITF list so they won't get destroyed twice.
        for( i = 0; i < nBands && papoBands != NULL; i++ )
            papoBands[i] = NULL;
    }

/* -------------------------------------------------------------------- */
/*      Update file length, and COMRAT for JPEG2000 files we are        */
/*      writing to.                                                     */
/* -------------------------------------------------------------------- */
    if( bJP2Writing )
    {
        GIntBig nPixelCount = nRasterXSize * ((GIntBig) nRasterYSize) * 
            nBands;

        NITFPatchImageLength( GetDescription(), nImageStart, nPixelCount, 
                              "C8" );
    }

/* -------------------------------------------------------------------- */
/*      If we have a jpeg output file, make sure it gets closed         */
/*      and flushed out.                                                */
/* -------------------------------------------------------------------- */
    if( poJPEGDataset != NULL )
    {
        int i;

        GDALClose( (GDALDatasetH) poJPEGDataset );
        
        // the bands are really jpeg bands ... remove them 
        // from the NITF list so they won't get destroyed twice.
        for( i = 0; i < nBands && papoBands != NULL; i++ )
            papoBands[i] = NULL;
    }

    CPLFree( panJPEGBlockOffset );
    CPLFree( pabyJPEGBlock );
}

/************************************************************************/
/*                             FlushCache()                             */
/************************************************************************/

void NITFDataset::FlushCache()

{
    if( poJ2KDataset != NULL && bJP2Writing)
        poJ2KDataset->FlushCache();

    // If the JPEG dataset has dirty pam info, then we should consider 
    // ourselves to as well.
    if( poJPEGDataset != NULL 
        && (((GDALPamDataset *) poJPEGDataset)->GetPamFlags() & GPF_DIRTY) )
        MarkPamDirty();

    GDALPamDataset::FlushCache();
}

/************************************************************************/
/*                              Identify()                              */
/************************************************************************/

int NITFDataset::Identify( GDALOpenInfo * poOpenInfo )

{
    const char *pszFilename = poOpenInfo->pszFilename;

/* -------------------------------------------------------------------- */
/*      Is this a dataset selector? If so, it is obviously NITF.        */
/* -------------------------------------------------------------------- */
    if( EQUALN(pszFilename, "NITF_IM:",8) )
        return TRUE;

/* -------------------------------------------------------------------- */
/*	First we check to see if the file has the expected header	*/
/*	bytes.								*/    
/* -------------------------------------------------------------------- */
    if( poOpenInfo->nHeaderBytes < 4 )
        return FALSE;
    
    if( !EQUALN((char *) poOpenInfo->pabyHeader,"NITF",4) 
        && !EQUALN((char *) poOpenInfo->pabyHeader,"NSIF",4)
        && !EQUALN((char *) poOpenInfo->pabyHeader,"NITF",4) )
        return FALSE;

    int i;
    /* Check that it's not in fact a NITF A.TOC file, which is handled by the RPFTOC driver */
    for(i=0;i<(int)poOpenInfo->nHeaderBytes-(int)strlen("A.TOC");i++)
    {
        if (EQUALN((const char*)poOpenInfo->pabyHeader + i, "A.TOC", strlen("A.TOC")))
            return FALSE;
    }

    return TRUE;
}
        
/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *NITFDataset::Open( GDALOpenInfo * poOpenInfo )

{
    int nIMIndex = -1;
    const char *pszFilename = poOpenInfo->pszFilename;

    if( !Identify( poOpenInfo ) )
        return NULL;
        
/* -------------------------------------------------------------------- */
/*      Select a specific subdataset.                                   */
/* -------------------------------------------------------------------- */
    if( EQUALN(pszFilename, "NITF_IM:",8) )
    {
        pszFilename += 8;
        nIMIndex = atoi(pszFilename);
        
        while( *pszFilename != '\0' && *pszFilename != ':' )
            pszFilename++;

        if( *pszFilename == ':' )
            pszFilename++;
    }

/* -------------------------------------------------------------------- */
/*      Open the file with library.                                     */
/* -------------------------------------------------------------------- */
    NITFFile *psFile;

    psFile = NITFOpen( pszFilename, poOpenInfo->eAccess == GA_Update );
    if( psFile == NULL )
    {
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Is there an image to operate on?                                */
/* -------------------------------------------------------------------- */
    int iSegment, nThisIM = 0;
    NITFImage *psImage = NULL;

    for( iSegment = 0; iSegment < psFile->nSegmentCount; iSegment++ )
    {
        if( EQUAL(psFile->pasSegmentInfo[iSegment].szSegmentType,"IM") 
            && (nThisIM++ == nIMIndex || nIMIndex == -1) )
        {
            psImage = NITFImageAccess( psFile, iSegment );
            if( psImage == NULL )
            {
                NITFClose( psFile );
                return NULL;
            }
            break;
        }
    }

/* -------------------------------------------------------------------- */
/*      If no image segments found report this to the user.             */
/* -------------------------------------------------------------------- */
    if( psImage == NULL )
    {
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "The file %s appears to be an NITF file, but no image\n"
                  "blocks were found on it.", 
                  poOpenInfo->pszFilename );
    }
    
/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */
    NITFDataset 	*poDS;

    poDS = new NITFDataset();

    poDS->psFile = psFile;
    poDS->psImage = psImage;
    poDS->eAccess = poOpenInfo->eAccess;
    poDS->osNITFFilename = pszFilename;
    poDS->nIMIndex = nIMIndex;

    if( psImage )
    {
        if (psImage->nCols <= 0 || psImage->nRows <= 0 ||
            psImage->nBlockWidth <= 0 || psImage->nBlockHeight <= 0)
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Bad values in NITF image : nCols=%d, nRows=%d, nBlockWidth=%d, nBlockHeight=%d",
                      psImage->nCols, psImage->nRows, psImage->nBlockWidth, psImage->nBlockHeight);
            delete poDS;
            return NULL;
        }

        poDS->nRasterXSize = psImage->nCols;
        poDS->nRasterYSize = psImage->nRows;
    }
    else
    {
        poDS->nRasterXSize = 1;
        poDS->nRasterYSize = 1;
    }

/* -------------------------------------------------------------------- */
/*      If the image is JPEG2000 (C8) compressed, we will need to       */
/*      open the image data as a JPEG2000 dataset.                      */
/* -------------------------------------------------------------------- */
    int nUsableBands = 0;
    int		iBand;

    if( psImage )
        nUsableBands = psImage->nBands;

    if( psImage != NULL && EQUAL(psImage->szIC,"C8") )
    {
        CPLString osDSName;

        osDSName.Printf( "/vsisubfile/%d_%d,%s", 
                         psFile->pasSegmentInfo[iSegment].nSegmentStart,
                         psFile->pasSegmentInfo[iSegment].nSegmentSize,
                         pszFilename );
    
        if( poWritableJ2KDataset != NULL )
        {
            poDS->poJ2KDataset = poWritableJ2KDataset; 
            poDS->bJP2Writing = TRUE;
            poWritableJ2KDataset = NULL;
        }
        else
        {
            poDS->poJ2KDataset = (GDALDataset *) 
                GDALOpen( osDSName, GA_ReadOnly );
        }

        if( poDS->poJ2KDataset == NULL )
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Unable to open JPEG2000 image within NITF file.\n"
                      "Is the JP2KAK driver available?" );
            delete poDS;
            return NULL;
        }

        if( poDS->poJ2KDataset->GetRasterCount() < nUsableBands )
        {
            CPLError( CE_Warning, CPLE_AppDefined, 
                      "JPEG2000 data stream has less useful bands than expected, likely\n"
                      "because some channels have differing resolutions." );
            
            nUsableBands = poDS->poJ2KDataset->GetRasterCount();
        }

        // Force NITF derived color space info 
        for( iBand = 0; iBand < nUsableBands; iBand++ )
        {
            NITFBandInfo *psBandInfo = psImage->pasBandInfo + iBand;
            GDALRasterBand *poBand=poDS->poJ2KDataset->GetRasterBand(iBand+1);
            
            CPLPushErrorHandler( CPLQuietErrorHandler );
            if( EQUAL(psBandInfo->szIREPBAND,"R") )
                poBand->SetColorInterpretation( GCI_RedBand );
            if( EQUAL(psBandInfo->szIREPBAND,"G") )
                poBand->SetColorInterpretation( GCI_GreenBand );
            if( EQUAL(psBandInfo->szIREPBAND,"B") )
                poBand->SetColorInterpretation( GCI_BlueBand );
            if( EQUAL(psBandInfo->szIREPBAND,"M") )
                poBand->SetColorInterpretation( GCI_GrayIndex );
            if( EQUAL(psBandInfo->szIREPBAND,"Y") )
                poBand->SetColorInterpretation( GCI_YCbCr_YBand );
            if( EQUAL(psBandInfo->szIREPBAND,"Cb") )
                poBand->SetColorInterpretation( GCI_YCbCr_CbBand );
            if( EQUAL(psBandInfo->szIREPBAND,"Cr") )
                poBand->SetColorInterpretation( GCI_YCbCr_CrBand );
            CPLPopErrorHandler();
            CPLErrorReset();
        }
    }

/* -------------------------------------------------------------------- */
/*      If the image is JPEG (C3) compressed, we will need to open      */
/*      the image data as a JPEG dataset.                               */
/* -------------------------------------------------------------------- */
    else if( psImage != NULL
             && EQUAL(psImage->szIC,"C3") 
             && psImage->nBlocksPerRow == 1
             && psImage->nBlocksPerColumn == 1 )
    {
        GUInt32 nJPEGStart = psFile->pasSegmentInfo[iSegment].nSegmentStart;

        poDS->nQLevel = poDS->ScanJPEGQLevel( &nJPEGStart );

        CPLString osDSName;

        osDSName.Printf( "JPEG_SUBFILE:Q%d,%d,%d,%s", 
                         poDS->nQLevel, nJPEGStart,
                         psFile->pasSegmentInfo[iSegment].nSegmentSize
                         - (nJPEGStart - psFile->pasSegmentInfo[iSegment].nSegmentStart),
                         pszFilename );

        CPLDebug( "GDAL", 
                  "NITFDataset::Open() as IC=C3 (JPEG compressed)\n");

        poDS->poJPEGDataset = (GDALDataset *) GDALOpen( osDSName, GA_ReadOnly);
        if( poDS->poJPEGDataset == NULL )
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Unable to open JPEG image within NITF file.\n"
                      "Is the JPEG driver available?" );
            delete poDS;
            return NULL;
        }

        if( poDS->poJPEGDataset->GetRasterCount() < nUsableBands )
        {
            CPLError( CE_Warning, CPLE_AppDefined, 
                      "JPEG data stream has less useful bands than expected, likely\n"
                      "because some channels have differing resolutions." );
            
            nUsableBands = poDS->poJPEGDataset->GetRasterCount();
        }

        // Force NITF derived color space info 
        for( iBand = 0; iBand < nUsableBands; iBand++ )
        {
            NITFBandInfo *psBandInfo = psImage->pasBandInfo + iBand;
            GDALRasterBand *poBand=poDS->poJPEGDataset->GetRasterBand(iBand+1);

            CPLPushErrorHandler( CPLQuietErrorHandler );
            if( EQUAL(psBandInfo->szIREPBAND,"R") )
                poBand->SetColorInterpretation( GCI_RedBand );
            if( EQUAL(psBandInfo->szIREPBAND,"G") )
                poBand->SetColorInterpretation( GCI_GreenBand );
            if( EQUAL(psBandInfo->szIREPBAND,"B") )
                poBand->SetColorInterpretation( GCI_BlueBand );
            if( EQUAL(psBandInfo->szIREPBAND,"M") )
                poBand->SetColorInterpretation( GCI_GrayIndex );
            if( EQUAL(psBandInfo->szIREPBAND,"Y") )
                poBand->SetColorInterpretation( GCI_YCbCr_YBand );
            if( EQUAL(psBandInfo->szIREPBAND,"Cb") )
                poBand->SetColorInterpretation( GCI_YCbCr_CbBand );
            if( EQUAL(psBandInfo->szIREPBAND,"Cr") )
                poBand->SetColorInterpretation( GCI_YCbCr_CrBand );
            CPLPopErrorHandler();
            CPLErrorReset();
        }
    }

/* -------------------------------------------------------------------- */
/*      Create band information objects.                                */
/* -------------------------------------------------------------------- */
    for( iBand = 0; iBand < nUsableBands; iBand++ )
    {
        if( poDS->poJ2KDataset != NULL )
            poDS->SetBand( iBand+1, 
                           poDS->poJ2KDataset->GetRasterBand(iBand+1) );
        else if( poDS->poJPEGDataset != NULL )
            poDS->SetBand( iBand+1, 
                           poDS->poJPEGDataset->GetRasterBand(iBand+1) );
        else
        {
            GDALRasterBand* poBand = new NITFRasterBand( poDS, iBand+1 );
            if (poBand->GetRasterDataType() == GDT_Unknown)
            {
                delete poBand;
                delete poDS;
                return NULL;
            }
            poDS->SetBand( iBand+1, poBand );
        }
    }

/* -------------------------------------------------------------------- */
/*      Report problems with odd bit sizes.                             */
/* -------------------------------------------------------------------- */
    if( psImage != NULL 
        && psImage->nBitsPerSample != 1
        && psImage->nBitsPerSample != 12
        && (psImage->nBitsPerSample < 8 || psImage->nBitsPerSample % 8 != 0) )
    {
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "Image with %d bits per sample will not be interpreted properly.", 
                  psImage->nBitsPerSample );
    }

/* -------------------------------------------------------------------- */
/*      Process the projection from the ICORDS.                         */
/* -------------------------------------------------------------------- */
    OGRSpatialReference oSRSWork;

    if( psImage == NULL )
    {
        /* nothing */
    }
    else if( psImage->chICORDS == 'G' || psImage->chICORDS == 'D' )
    {
        CPLFree( poDS->pszProjection );
        poDS->pszProjection = NULL;
        
        oSRSWork.SetWellKnownGeogCS( "WGS84" );
        oSRSWork.exportToWkt( &(poDS->pszProjection) );
    }
    else if( psImage->chICORDS == 'C' )
    {
        CPLFree( poDS->pszProjection );
        poDS->pszProjection = NULL;
        
        oSRSWork.SetWellKnownGeogCS( "WGS84" );
        oSRSWork.exportToWkt( &(poDS->pszProjection) );

        /* convert latitudes from geocentric to geodetic form. */
        
        psImage->dfULY = 
            NITF_WGS84_Geocentric_Latitude_To_Geodetic_Latitude( 
                psImage->dfULY );
        psImage->dfLLY = 
            NITF_WGS84_Geocentric_Latitude_To_Geodetic_Latitude( 
                psImage->dfLLY );
        psImage->dfURY = 
            NITF_WGS84_Geocentric_Latitude_To_Geodetic_Latitude( 
                psImage->dfURY );
        psImage->dfLRY = 
            NITF_WGS84_Geocentric_Latitude_To_Geodetic_Latitude( 
                psImage->dfLRY );
    }
    else if( psImage->chICORDS == 'S' || psImage->chICORDS == 'N' )
    {
        CPLFree( poDS->pszProjection );
        poDS->pszProjection = NULL;

        oSRSWork.SetUTM( psImage->nZone, psImage->chICORDS == 'N' );
        oSRSWork.SetWellKnownGeogCS( "WGS84" );
        oSRSWork.exportToWkt( &(poDS->pszProjection) );
    }
    else if( psImage->chICORDS == 'U' && psImage->nZone != 0 )
    {
        CPLFree( poDS->pszProjection );
        poDS->pszProjection = NULL;

        oSRSWork.SetUTM( ABS(psImage->nZone), psImage->nZone > 0 );
        oSRSWork.SetWellKnownGeogCS( "WGS84" );
        oSRSWork.exportToWkt( &(poDS->pszProjection) );
    }


/* -------------------------------------------------------------------- */
/*      Try looking for a .nfw file.                                    */
/* -------------------------------------------------------------------- */
    if( psImage
        && GDALReadWorldFile( pszFilename, "nfw", 
                              poDS->adfGeoTransform ) )
    {
        const char *pszHDR;
        FILE *fpHDR;
        char **papszLines;
        int isNorth;
        int zone;
        
        poDS->bGotGeoTransform = TRUE;

        /* If nfw found, try looking for a header with projection info */
        /* in space imaging style format                               */
        pszHDR = CPLResetExtension( pszFilename, "hdr" );
        
        fpHDR = VSIFOpenL( pszHDR, "rt" );

#ifndef WIN32
        if( fpHDR == NULL )
        {
            pszHDR = CPLResetExtension( pszFilename, "HDR" );
            fpHDR = VSIFOpenL( pszHDR, "rt" );
        }
#endif
    
        if( fpHDR != NULL )
        {
            VSIFCloseL( fpHDR );
            papszLines=CSLLoad(pszHDR);
            if (CSLCount(papszLines) == 16)
            {

                if (psImage->chICORDS == 'N')
                    isNorth=1;
                else if (psImage->chICORDS =='S')
                    isNorth=0;
                else
                {
                    if (psImage->dfLLY+psImage->dfLRY+psImage->dfULY+psImage->dfURY < 0)
                        isNorth=0;
                    else
                        isNorth=1;
                }
                if( (EQUALN(papszLines[7],
                            "Selected Projection: Universal Transverse Mercator",50)) &&
                    (EQUALN(papszLines[8],"Zone: ",6)) &&
                    (strlen(papszLines[8]) >= 7))
                {
                    CPLFree( poDS->pszProjection );
                    poDS->pszProjection = NULL;
                    zone=atoi(&(papszLines[8][6]));
                    oSRSWork.SetUTM( zone, isNorth );
                    oSRSWork.SetWellKnownGeogCS( "WGS84" );
                    oSRSWork.exportToWkt( &(poDS->pszProjection) );
                }
                else
                {
                    /* Couldn't find associated projection info.
                       Go back to original file for geotransform.
                    */
                    poDS->bGotGeoTransform = FALSE;
                }
            }
            else
                poDS->bGotGeoTransform = FALSE;
            CSLDestroy(papszLines);
        }
        else
            poDS->bGotGeoTransform = FALSE;
    }

/* -------------------------------------------------------------------- */
/*      Do we have IGEOLO data that can be treated as a                 */
/*      geotransform?  Our approach should support images in an         */
/*      affine rotated frame of reference.                              */
/* -------------------------------------------------------------------- */
    int nGCPCount = 0;
    GDAL_GCP    *psGCPs = NULL;

    if( psImage && !poDS->bGotGeoTransform && psImage->chICORDS != ' ' )
    {
        nGCPCount = 4;

        psGCPs = (GDAL_GCP *) CPLMalloc(sizeof(GDAL_GCP) * nGCPCount);
        GDALInitGCPs( nGCPCount, psGCPs );

        psGCPs[0].dfGCPPixel	= 0.0;
        psGCPs[0].dfGCPLine		= 0.0;
        psGCPs[0].dfGCPX		= psImage->dfULX;
        psGCPs[0].dfGCPY		= psImage->dfULY;

        psGCPs[1].dfGCPPixel = poDS->nRasterXSize;
        psGCPs[1].dfGCPLine = 0.0;
        psGCPs[1].dfGCPX		= psImage->dfURX;
        psGCPs[1].dfGCPY		= psImage->dfURY;

        psGCPs[2].dfGCPPixel = poDS->nRasterXSize;
        psGCPs[2].dfGCPLine = poDS->nRasterYSize;
        psGCPs[2].dfGCPX		= psImage->dfLRX;
        psGCPs[2].dfGCPY		= psImage->dfLRY;

        psGCPs[3].dfGCPPixel = 0.0;
        psGCPs[3].dfGCPLine = poDS->nRasterYSize;
        psGCPs[3].dfGCPX		= psImage->dfLLX;
        psGCPs[3].dfGCPY		= psImage->dfLLY;
    }

/* -------------------------------------------------------------------- */
/*      Convert the GCPs into a geotransform definition, if possible.   */
/* -------------------------------------------------------------------- */
    if( !psImage )
    {
        /* nothing */
    }
    else if( poDS->bGotGeoTransform == FALSE 
             && nGCPCount > 0 
             && GDALGCPsToGeoTransform( nGCPCount, psGCPs, 
                                        poDS->adfGeoTransform, TRUE ) )
    {	
        poDS->bGotGeoTransform = TRUE;
    } 

/* -------------------------------------------------------------------- */
/*      If we have IGEOLO that isn't north up, return it as GCPs.       */
/* -------------------------------------------------------------------- */
    else if( (psImage->dfULX != 0 || psImage->dfURX != 0 
              || psImage->dfLRX != 0 || psImage->dfLLX != 0)
             && psImage->chICORDS != ' ' && 
             ( poDS->bGotGeoTransform == FALSE ) )
    {
        CPLDebug( "GDAL", 
                  "NITFDataset::Open() wasn't able to derive a first order\n"
                  "geotransform.  It will be returned as GCPs.");

        poDS->nGCPCount = 4;
        poDS->pasGCPList = (GDAL_GCP *) CPLCalloc(sizeof(GDAL_GCP),
                                                  poDS->nGCPCount);
        GDALInitGCPs( 4, poDS->pasGCPList );

        poDS->pasGCPList[0].dfGCPX = psImage->dfULX;
        poDS->pasGCPList[0].dfGCPY = psImage->dfULY;
        poDS->pasGCPList[0].dfGCPPixel = 0;
        poDS->pasGCPList[0].dfGCPLine = 0;
        CPLFree( poDS->pasGCPList[0].pszId );
        poDS->pasGCPList[0].pszId = CPLStrdup( "UpperLeft" );

        poDS->pasGCPList[1].dfGCPX = psImage->dfURX;
        poDS->pasGCPList[1].dfGCPY = psImage->dfURY;
        poDS->pasGCPList[1].dfGCPPixel = poDS->nRasterXSize;
        poDS->pasGCPList[1].dfGCPLine = 0;
        CPLFree( poDS->pasGCPList[1].pszId );
        poDS->pasGCPList[1].pszId = CPLStrdup( "UpperRight" );

        poDS->pasGCPList[2].dfGCPX = psImage->dfLLX;
        poDS->pasGCPList[2].dfGCPY = psImage->dfLLY;
        poDS->pasGCPList[2].dfGCPPixel = 0;
        poDS->pasGCPList[2].dfGCPLine = poDS->nRasterYSize;
        CPLFree( poDS->pasGCPList[2].pszId );
        poDS->pasGCPList[2].pszId = CPLStrdup( "LowerLeft" );

        poDS->pasGCPList[3].dfGCPX = psImage->dfLRX;
        poDS->pasGCPList[3].dfGCPY = psImage->dfLRY;
        poDS->pasGCPList[3].dfGCPPixel = poDS->nRasterXSize;
        poDS->pasGCPList[3].dfGCPLine = poDS->nRasterYSize;
        CPLFree( poDS->pasGCPList[3].pszId );
        poDS->pasGCPList[3].pszId = CPLStrdup( "LowerRight" );

        poDS->pszGCPProjection = CPLStrdup( poDS->pszProjection );
    }

    // This cleans up the original copy of the GCPs used to test if 
    // this IGEOLO could be used for a geotransform.
    if( nGCPCount > 0 )
    {
        GDALDeinitGCPs( nGCPCount, psGCPs );
        CPLFree( psGCPs );
    }

/* -------------------------------------------------------------------- */
/*      Do we have PRJPSB and MAPLOB TREs to get better                 */
/*      georeferencing from?                                            */
/* -------------------------------------------------------------------- */
    if (psImage)
        poDS->CheckGeoSDEInfo();

/* -------------------------------------------------------------------- */
/*      Do we have metadata.                                            */
/* -------------------------------------------------------------------- */
    char **papszMergedMD;
    char **papszUSE00A_MD;

    // File and Image level metadata.
    papszMergedMD = CSLDuplicate( poDS->psFile->papszMetadata );

    if( psImage )
    {
        papszMergedMD = CSLInsertStrings( papszMergedMD, 
                                          CSLCount( papszMergedMD ),
                                          psImage->papszMetadata );

        // Comments.
        if( psImage->pszComments != NULL && strlen(psImage->pszComments) != 0 )
            papszMergedMD = CSLSetNameValue( 
                papszMergedMD, "NITF_IMAGE_COMMENTS", psImage->pszComments );
        
        // Compression code. 
        papszMergedMD = CSLSetNameValue( papszMergedMD, "NITF_IC", 
                                         psImage->szIC );
        
        // IMODE
        char szIMODE[2];
        szIMODE[0] = psImage->chIMODE;
        szIMODE[1] = '\0';
        papszMergedMD = CSLSetNameValue( papszMergedMD, "NITF_IMODE", szIMODE );

        // ILOC/Attachment info
        if( psImage->nILOCRow != 0 )
        {
            papszMergedMD = 
                CSLSetNameValue( papszMergedMD, "NITF_IDLVL", 
                                 CPLString().Printf("%d",psImage->nIDLVL) );
            papszMergedMD = 
                CSLSetNameValue( papszMergedMD, "NITF_IALVL", 
                                 CPLString().Printf("%d",psImage->nIALVL) );
            papszMergedMD = 
                CSLSetNameValue( papszMergedMD, "NITF_ILOC_ROW", 
                                 CPLString().Printf("%d",psImage->nILOCRow) );
            papszMergedMD = 
                CSLSetNameValue( papszMergedMD, "NITF_ILOC_COLUMN", 
                                CPLString().Printf("%d",psImage->nILOCColumn));
            papszMergedMD = 
                CSLSetNameValue( papszMergedMD, "NITF_IMAG", 
                                 psImage->szIMAG );
        }

        // USE00A 
        papszUSE00A_MD = NITFReadUSE00A( psImage );
        if( papszUSE00A_MD != NULL )
        {
            papszMergedMD = CSLInsertStrings( papszMergedMD, 
                                              CSLCount( papszUSE00A_MD ),
                                              papszUSE00A_MD );
            CSLDestroy( papszUSE00A_MD );
        }
        
        // BLOCKA 
        papszUSE00A_MD = NITFReadBLOCKA( psImage );
        if( papszUSE00A_MD != NULL )
        {
            papszMergedMD = CSLInsertStrings( papszMergedMD, 
                                              CSLCount( papszUSE00A_MD ),
                                              papszUSE00A_MD );
            CSLDestroy( papszUSE00A_MD );
        }
        
        papszUSE00A_MD = NITFReadSTDIDC( psImage );
        if( papszUSE00A_MD != NULL )
        {
            papszMergedMD = CSLInsertStrings( papszMergedMD, 
                                              CSLCount( papszUSE00A_MD ),
                                              papszUSE00A_MD );
            CSLDestroy( papszUSE00A_MD );
        }
    }
        
    poDS->SetMetadata( papszMergedMD );
    CSLDestroy( papszMergedMD );

/* -------------------------------------------------------------------- */
/*      Image structure metadata.                                       */
/* -------------------------------------------------------------------- */
    if( psImage == NULL )
        /* do nothing */;
    else if( psImage->szIC[1] == '1' )
        poDS->SetMetadataItem( "COMPRESSION", "BILEVEL", 
                               "IMAGE_STRUCTURE" );
    else if( psImage->szIC[1] == '2' )
        poDS->SetMetadataItem( "COMPRESSION", "ARIDPCM", 
                               "IMAGE_STRUCTURE" );
    else if( psImage->szIC[1] == '3' )
        poDS->SetMetadataItem( "COMPRESSION", "JPEG", 
                               "IMAGE_STRUCTURE" );
    else if( psImage->szIC[1] == '4' )
        poDS->SetMetadataItem( "COMPRESSION", "VECTOR QUANTIZATION", 
                               "IMAGE_STRUCTURE" );
    else if( psImage->szIC[1] == '5' )
        poDS->SetMetadataItem( "COMPRESSION", "LOSSLESS JPEG", 
                               "IMAGE_STRUCTURE" );
    else if( psImage->szIC[1] == '8' )
        poDS->SetMetadataItem( "COMPRESSION", "JPEG2000", 
                               "IMAGE_STRUCTURE" );
    
/* -------------------------------------------------------------------- */
/*      Do we have RPC info.                                            */
/* -------------------------------------------------------------------- */
    NITFRPC00BInfo sRPCInfo;

    if( psImage
        && NITFReadRPC00B( psImage, &sRPCInfo ) && sRPCInfo.SUCCESS )
    {
        char szValue[1280];
        int  i;

        sprintf( szValue, "%.16g", sRPCInfo.LINE_OFF );
        poDS->SetMetadataItem( "LINE_OFF", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.LINE_SCALE );
        poDS->SetMetadataItem( "LINE_SCALE", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.SAMP_OFF );
        poDS->SetMetadataItem( "SAMP_OFF", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.SAMP_SCALE );
        poDS->SetMetadataItem( "SAMP_SCALE", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.LONG_OFF );
        poDS->SetMetadataItem( "LONG_OFF", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.LONG_SCALE );
        poDS->SetMetadataItem( "LONG_SCALE", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.LAT_OFF );
        poDS->SetMetadataItem( "LAT_OFF", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.LAT_SCALE );
        poDS->SetMetadataItem( "LAT_SCALE", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.HEIGHT_OFF );
        poDS->SetMetadataItem( "HEIGHT_OFF", szValue, "RPC" );

        sprintf( szValue, "%.16g", sRPCInfo.HEIGHT_SCALE );
        poDS->SetMetadataItem( "HEIGHT_SCALE", szValue, "RPC" );

        szValue[0] = '\0'; 
        for( i = 0; i < 20; i++ )
            sprintf( szValue+strlen(szValue), "%.16g ",  
                     sRPCInfo.LINE_NUM_COEFF[i] );
        poDS->SetMetadataItem( "LINE_NUM_COEFF", szValue, "RPC" );

        szValue[0] = '\0'; 
        for( i = 0; i < 20; i++ )
            sprintf( szValue+strlen(szValue), "%.16g ",  
                     sRPCInfo.LINE_DEN_COEFF[i] );
        poDS->SetMetadataItem( "LINE_DEN_COEFF", szValue, "RPC" );
        
        szValue[0] = '\0'; 
        for( i = 0; i < 20; i++ )
            sprintf( szValue+strlen(szValue), "%.16g ",  
                     sRPCInfo.SAMP_NUM_COEFF[i] );
        poDS->SetMetadataItem( "SAMP_NUM_COEFF", szValue, "RPC" );
        
        szValue[0] = '\0'; 
        for( i = 0; i < 20; i++ )
            sprintf( szValue+strlen(szValue), "%.16g ",  
                     sRPCInfo.SAMP_DEN_COEFF[i] );
        poDS->SetMetadataItem( "SAMP_DEN_COEFF", szValue, "RPC" );

        sprintf( szValue, "%.16g", 
                 sRPCInfo.LONG_OFF - ( sRPCInfo.LONG_SCALE / 2.0 ) );
        poDS->SetMetadataItem( "MIN_LONG", szValue, "RPC" );

        sprintf( szValue, "%.16g",
                 sRPCInfo.LONG_OFF + ( sRPCInfo.LONG_SCALE / 2.0 ) );
        poDS->SetMetadataItem( "MAX_LONG", szValue, "RPC" );

        sprintf( szValue, "%.16g", 
                 sRPCInfo.LAT_OFF - ( sRPCInfo.LAT_SCALE / 2.0 ) );
        poDS->SetMetadataItem( "MIN_LAT", szValue, "RPC" );

        sprintf( szValue, "%.16g", 
                 sRPCInfo.LAT_OFF + ( sRPCInfo.LAT_SCALE / 2.0 ) );
        poDS->SetMetadataItem( "MAX_LAT", szValue, "RPC" );
    }

/* -------------------------------------------------------------------- */
/*      Do we have Chip info?                                            */
/* -------------------------------------------------------------------- */
    NITFICHIPBInfo sChipInfo;

    if( psImage
        && NITFReadICHIPB( psImage, &sChipInfo ) && sChipInfo.XFRM_FLAG == 0 )
    {
        char szValue[1280];

        sprintf( szValue, "%.16g", sChipInfo.SCALE_FACTOR );
        poDS->SetMetadataItem( "ICHIP_SCALE_FACTOR", szValue );

        sprintf( szValue, "%d", sChipInfo.ANAMORPH_CORR );
        poDS->SetMetadataItem( "ICHIP_ANAMORPH_CORR", szValue );

        sprintf( szValue, "%d", sChipInfo.SCANBLK_NUM );
        poDS->SetMetadataItem( "ICHIP_SCANBLK_NUM", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_ROW_11 );
        poDS->SetMetadataItem( "ICHIP_OP_ROW_11", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_COL_11 );
        poDS->SetMetadataItem( "ICHIP_OP_COL_11", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_ROW_12 );
        poDS->SetMetadataItem( "ICHIP_OP_ROW_12", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_COL_12 );
        poDS->SetMetadataItem( "ICHIP_OP_COL_12", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_ROW_21 );
        poDS->SetMetadataItem( "ICHIP_OP_ROW_21", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_COL_21 );
        poDS->SetMetadataItem( "ICHIP_OP_COL_21", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_ROW_22 );
        poDS->SetMetadataItem( "ICHIP_OP_ROW_22", szValue );

        sprintf( szValue, "%.16g", sChipInfo.OP_COL_22 );
        poDS->SetMetadataItem( "ICHIP_OP_COL_22", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_ROW_11 );
        poDS->SetMetadataItem( "ICHIP_FI_ROW_11", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_COL_11 );
        poDS->SetMetadataItem( "ICHIP_FI_COL_11", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_ROW_12 );
        poDS->SetMetadataItem( "ICHIP_FI_ROW_12", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_COL_12 );
        poDS->SetMetadataItem( "ICHIP_FI_COL_12", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_ROW_21 );
        poDS->SetMetadataItem( "ICHIP_FI_ROW_21", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_COL_21 );
        poDS->SetMetadataItem( "ICHIP_FI_COL_21", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_ROW_22 );
        poDS->SetMetadataItem( "ICHIP_FI_ROW_22", szValue );

        sprintf( szValue, "%.16g", sChipInfo.FI_COL_22 );
        poDS->SetMetadataItem( "ICHIP_FI_COL_22", szValue );

        sprintf( szValue, "%d", sChipInfo.FI_ROW );
        poDS->SetMetadataItem( "ICHIP_FI_ROW", szValue );

        sprintf( szValue, "%d", sChipInfo.FI_COL );
        poDS->SetMetadataItem( "ICHIP_FI_COL", szValue );

    }
    
    const NITFSeries* series = NITFGetSeriesInfo(pszFilename);
    if (series)
    {
        poDS->SetMetadataItem("NITF_SERIES_ABBREVIATION",
                            (series->abbreviation) ? series->abbreviation : "Unknown");
        poDS->SetMetadataItem("NITF_SERIES_NAME",
                            (series->name) ? series->name : "Unknown");
    }

/* -------------------------------------------------------------------- */
/*      If there are multiple image segments, and we are the zeroth,    */
/*      then setup the subdataset metadata.                             */
/* -------------------------------------------------------------------- */
    int nSubDSCount = 0;

    if( nIMIndex == -1 )
    {
        char **papszSubdatasets = NULL;
        int nIMCounter = 0;

        for( iSegment = 0; iSegment < psFile->nSegmentCount; iSegment++ )
        {
            if( EQUAL(psFile->pasSegmentInfo[iSegment].szSegmentType,"IM") )
            {
                CPLString oName;
                CPLString oValue;

                oName.Printf( "SUBDATASET_%d_NAME", nIMCounter+1 );
                oValue.Printf( "NITF_IM:%d:%s", nIMCounter, pszFilename );
                papszSubdatasets = CSLSetNameValue( papszSubdatasets, 
                                                    oName, oValue );

                oName.Printf( "SUBDATASET_%d_DESC", nIMCounter+1 );
                oValue.Printf( "Image %d of %s", nIMCounter+1, pszFilename );
                papszSubdatasets = CSLSetNameValue( papszSubdatasets, 
                                                    oName, oValue );

                nIMCounter++;
            }
        }

        nSubDSCount = CSLCount(papszSubdatasets) / 2;
        if( nSubDSCount > 1 )
            poDS->GDALMajorObject::SetMetadata( papszSubdatasets, 
                                                "SUBDATASETS" );
        
        CSLDestroy( papszSubdatasets );
    }

/* -------------------------------------------------------------------- */
/*      Check for overviews.                                            */
/* -------------------------------------------------------------------- */
    poDS->oOvManager.Initialize( poDS, pszFilename );

/* -------------------------------------------------------------------- */
/*      Initialize any PAM information.                                 */
/* -------------------------------------------------------------------- */
    poDS->SetDescription( poOpenInfo->pszFilename );
    poDS->SetPhysicalFilename( pszFilename );
    
    if( nSubDSCount > 1 || nIMIndex != -1 )
    {
        if( nIMIndex == -1 )
            nIMIndex = 0;

        poDS->SetSubdatasetName( CPLString().Printf("%d",nIMIndex) );
    }

    poDS->TryLoadXML();

    return( poDS );
}

/************************************************************************/
/*                            LoadDODDatum()                            */
/*                                                                      */
/*      Try to turn a US military datum name into a datum definition.   */
/************************************************************************/

static OGRErr LoadDODDatum( OGRSpatialReference *poSRS,
                            const char *pszDatumName )

{
/* -------------------------------------------------------------------- */
/*      The most common case...                                         */
/* -------------------------------------------------------------------- */
    if( EQUALN(pszDatumName,"WGE ",4) )
    {
        poSRS->SetWellKnownGeogCS( "WGS84" );
        return OGRERR_NONE;
    }

/* -------------------------------------------------------------------- */
/*      All the rest we will try and load from gt_datum.csv             */
/*      (Geotrans datum file).                                          */
/* -------------------------------------------------------------------- */
    char szExpanded[5];
    const char *pszGTDatum = CSVFilename( "gt_datum.csv" );

    strncpy( szExpanded, pszDatumName, 3 );
    szExpanded[3] = '\0';
    if( pszDatumName[3] != ' ' )
    {
        strcat( szExpanded, "-" );
        strncat( szExpanded, pszDatumName + 3, 1 );
    }

    CPLString osDName = CSVGetField( pszGTDatum, "CODE", szExpanded, 
                                     CC_ApproxString, "NAME" );
    if( strlen(osDName) == 0 )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "Failed to find datum %s/%s in gt_datum.csv.",
                  pszDatumName, szExpanded );
        return OGRERR_FAILURE;
    }
        
    CPLString osEllipseCode = CSVGetField( pszGTDatum, "CODE", szExpanded, 
                                           CC_ApproxString, "ELLIPSOID" );
    double dfDeltaX = CPLAtof(CSVGetField( pszGTDatum, "CODE", szExpanded, 
                                           CC_ApproxString, "DELTAX" ) );
    double dfDeltaY = CPLAtof(CSVGetField( pszGTDatum, "CODE", szExpanded, 
                                           CC_ApproxString, "DELTAY" ) );
    double dfDeltaZ = CPLAtof(CSVGetField( pszGTDatum, "CODE", szExpanded, 
                                           CC_ApproxString, "DELTAZ" ) );

/* -------------------------------------------------------------------- */
/*      Lookup the ellipse code.                                        */
/* -------------------------------------------------------------------- */
    const char *pszGTEllipse = CSVFilename( "gt_ellips.csv" );
    
    CPLString osEName = CSVGetField( pszGTEllipse, "CODE", osEllipseCode,
                                     CC_ApproxString, "NAME" );
    if( strlen(osEName) == 0 )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "Failed to find datum %s in gt_ellips.csv.",
                  osEllipseCode.c_str() );
        return OGRERR_FAILURE;
    }    
    
    double dfA = CPLAtof(CSVGetField( pszGTEllipse, "CODE", osEllipseCode,
                                      CC_ApproxString, "A" ));
    double dfInvF = CPLAtof(CSVGetField( pszGTEllipse, "CODE", osEllipseCode,
                                         CC_ApproxString, "RF" ));

/* -------------------------------------------------------------------- */
/*      Create geographic coordinate system.                            */
/* -------------------------------------------------------------------- */
    poSRS->SetGeogCS( osDName, osDName, osEName, dfA, dfInvF );

    poSRS->SetTOWGS84( dfDeltaX, dfDeltaY, dfDeltaZ );

    return OGRERR_NONE;
}

/************************************************************************/
/*                          CheckGeoSDEInfo()                           */
/*                                                                      */
/*      Check for GeoSDE TREs (GEOPSB/PRJPSB and MAPLOB).  If we        */
/*      have them, use them to override our coordinate system and       */
/*      geotransform info.                                              */
/************************************************************************/

void NITFDataset::CheckGeoSDEInfo()

{
/* -------------------------------------------------------------------- */
/*      Do we have the required TREs?                                   */
/* -------------------------------------------------------------------- */
    const char *pszGEOPSB , *pszPRJPSB, *pszMAPLOB;
    OGRSpatialReference oSRS;
    char szName[81];

    pszGEOPSB = NITFFindTRE( psFile->pachTRE, psFile->nTREBytes,"GEOPSB",NULL);
    pszPRJPSB = NITFFindTRE( psFile->pachTRE, psFile->nTREBytes,"PRJPSB",NULL);
    pszMAPLOB = NITFFindTRE(psImage->pachTRE,psImage->nTREBytes,"MAPLOB",NULL);

    if( pszGEOPSB == NULL || pszPRJPSB == NULL || pszMAPLOB == NULL )
        return;

/* -------------------------------------------------------------------- */
/*      Collect projection parameters.                                  */
/* -------------------------------------------------------------------- */
    int nRemainingBytesPRJPSB = psFile->nTREBytes - (pszPRJPSB - psFile->pachTRE);

    char szParm[16];
    if (nRemainingBytesPRJPSB < 82 + 1)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Cannot read PRJPSB TRE. Not enough bytes");
    }
    int nParmCount = atoi(NITFGetField(szParm,pszPRJPSB,82,1));
    int i;
    double adfParm[8], dfFN, dfFE;

    if (nRemainingBytesPRJPSB < 83+15*nParmCount+15+15)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Cannot read PRJPSB TRE. Not enough bytes");
    }
    for( i = 0; i < nParmCount; i++ )
        adfParm[i] = atof(NITFGetField(szParm,pszPRJPSB,83+15*i,15));

    dfFE = atof(NITFGetField(szParm,pszPRJPSB,83+15*nParmCount,15));
    dfFN = atof(NITFGetField(szParm,pszPRJPSB,83+15*nParmCount+15,15));

/* -------------------------------------------------------------------- */
/*      Try to handle the projection.                                   */
/* -------------------------------------------------------------------- */
    if( EQUALN(pszPRJPSB+80,"AC",2) )
        oSRS.SetACEA( adfParm[1], adfParm[2], adfParm[3], adfParm[0], 
                      dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"AK",2) )
        oSRS.SetLAEA( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"AL",2) )
        oSRS.SetAE( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"BF",2) )
        oSRS.SetBonne( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"CP",2) )
        oSRS.SetEquirectangular( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"CS",2) )
        oSRS.SetCS( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"EF",2) )
        oSRS.SetEckertIV( adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"ED",2) )
        oSRS.SetEckertVI( adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"GN",2) )
        oSRS.SetGnomonic( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"HX",2) )
        oSRS.SetHOM2PNO( adfParm[1], 
                         adfParm[3], adfParm[2],
                         adfParm[5], adfParm[4],
                         adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"KA",2) )
        oSRS.SetEC( adfParm[1], adfParm[2], adfParm[3], adfParm[0], 
                    dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"LE",2) )
        oSRS.SetLCC( adfParm[1], adfParm[2], adfParm[3], adfParm[0], 
                     dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"LI",2) )
        oSRS.SetCEA( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"MC",2) )
        oSRS.SetMercator( adfParm[2], adfParm[1], 1.0, dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"MH",2) )
        oSRS.SetMC( 0.0, adfParm[1], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"MP",2) )
        oSRS.SetMollweide( adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"NT",2) )
        oSRS.SetNZMG( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"OD",2) )
        oSRS.SetOrthographic( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"PC",2) )
        oSRS.SetPolyconic( adfParm[1], adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"PG",2) )
        oSRS.SetPS( adfParm[1], adfParm[0], 1.0, dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"RX",2) )
        oSRS.SetRobinson( adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"SA",2) )
        oSRS.SetSinusoidal( adfParm[0], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"TC",2) )
        oSRS.SetTM( adfParm[1], adfParm[0], adfParm[1], dfFE, dfFN );

    else if( EQUALN(pszPRJPSB+80,"VA",2) )
        oSRS.SetVDG( adfParm[0], dfFE, dfFN );

    else
        oSRS.SetLocalCS( NITFGetField(szName,pszPRJPSB,0,80) );

/* -------------------------------------------------------------------- */
/*      Try to apply the datum.                                         */
/* -------------------------------------------------------------------- */
    int nRemainingBytesGEOPSB = psFile->nTREBytes - (pszGEOPSB - psFile->pachTRE);
    if (nRemainingBytesGEOPSB < 86 + 4)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Cannot read GEOPSB TRE. Not enough bytes");
    }
    LoadDODDatum( &oSRS, NITFGetField(szParm,pszGEOPSB,86,4) );

/* -------------------------------------------------------------------- */
/*      Get the geotransform                                            */
/* -------------------------------------------------------------------- */
    double adfGT[6];

    int nRemainingBytesMAPLOB = psImage->nTREBytes - (pszMAPLOB - psImage->pachTRE);
    if (nRemainingBytesMAPLOB < 28 + 15)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Cannot read MAPLOB TRE. Not enough bytes");
    }
    adfGT[0] = atof(NITFGetField(szParm,pszMAPLOB,13,15));
    adfGT[1] = atof(NITFGetField(szParm,pszMAPLOB,3,5));
    adfGT[2] = 0.0;
    adfGT[3] = atof(NITFGetField(szParm,pszMAPLOB,28,15));
    adfGT[4] = 0.0;
    adfGT[5] = -atof(NITFGetField(szParm,pszMAPLOB,8,5));

/* -------------------------------------------------------------------- */
/*      Apply back to dataset.                                          */
/* -------------------------------------------------------------------- */
    CPLFree( pszProjection );
    pszProjection = NULL;

    oSRS.exportToWkt( &pszProjection );

    memcpy( adfGeoTransform, adfGT, sizeof(double)*6 );
}

/************************************************************************/
/*                             AdviseRead()                             */
/************************************************************************/

CPLErr NITFDataset::AdviseRead( int nXOff, int nYOff, int nXSize, int nYSize,
                                int nBufXSize, int nBufYSize, 
                                GDALDataType eDT, 
                                int nBandCount, int *panBandList,
                                char **papszOptions )
    
{
    if( poJ2KDataset == NULL )
        return GDALDataset::AdviseRead( nXOff, nYOff, nXSize, nYSize, 
                                        nBufXSize, nBufYSize, eDT, 
                                        nBandCount, panBandList, 
                                        papszOptions);
    else if( poJPEGDataset != NULL )
        return poJPEGDataset->AdviseRead( nXOff, nYOff, nXSize, nYSize, 
                                          nBufXSize, nBufYSize, eDT, 
                                          nBandCount, panBandList, 
                                          papszOptions);
    else
        return poJ2KDataset->AdviseRead( nXOff, nYOff, nXSize, nYSize, 
                                         nBufXSize, nBufYSize, eDT, 
                                         nBandCount, panBandList, 
                                         papszOptions);
}

/************************************************************************/
/*                             IRasterIO()                              */
/************************************************************************/

CPLErr NITFDataset::IRasterIO( GDALRWFlag eRWFlag,
                               int nXOff, int nYOff, int nXSize, int nYSize,
                               void * pData, int nBufXSize, int nBufYSize,
                               GDALDataType eBufType, 
                               int nBandCount, int *panBandMap,
                               int nPixelSpace, int nLineSpace, int nBandSpace)
    
{
    if( poJ2KDataset != NULL )
        return poJ2KDataset->RasterIO( eRWFlag, nXOff, nYOff, nXSize, nYSize,
                                       pData, nBufXSize, nBufYSize, eBufType,
                                       nBandCount, panBandMap, 
                                       nPixelSpace, nLineSpace, nBandSpace );
    else if( poJPEGDataset != NULL )
        return poJPEGDataset->RasterIO( eRWFlag, nXOff, nYOff, nXSize, nYSize,
                                        pData, nBufXSize, nBufYSize, eBufType,
                                        nBandCount, panBandMap, 
                                        nPixelSpace, nLineSpace, nBandSpace );
    else 
        return GDALDataset::IRasterIO( eRWFlag, nXOff, nYOff, nXSize, nYSize,
                                       pData, nBufXSize, nBufYSize, eBufType,
                                       nBandCount, panBandMap, 
                                       nPixelSpace, nLineSpace, nBandSpace );
}


/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr NITFDataset::GetGeoTransform( double *padfGeoTransform )

{
    memcpy( padfGeoTransform, adfGeoTransform, sizeof(double) * 6 );

    if( bGotGeoTransform )
        return CE_None;
    else
        return GDALPamDataset::GetGeoTransform( padfGeoTransform );
}

/************************************************************************/
/*                          SetGeoTransform()                           */
/************************************************************************/

CPLErr NITFDataset::SetGeoTransform( double *padfGeoTransform )

{
    double dfIGEOLOULX, dfIGEOLOULY, dfIGEOLOURX, dfIGEOLOURY, 
           dfIGEOLOLRX, dfIGEOLOLRY, dfIGEOLOLLX, dfIGEOLOLLY;

    bGotGeoTransform = TRUE;
    memcpy( adfGeoTransform, padfGeoTransform, sizeof(double) * 6 );

    dfIGEOLOULX = padfGeoTransform[0] + 0.5 * padfGeoTransform[1] 
                                      + 0.5 * padfGeoTransform[2];
    dfIGEOLOULY = padfGeoTransform[3] + 0.5 * padfGeoTransform[4] 
                                      + 0.5 * padfGeoTransform[5];
    dfIGEOLOURX = dfIGEOLOULX + padfGeoTransform[1] * (nRasterXSize - 1);
    dfIGEOLOURY = dfIGEOLOULY + padfGeoTransform[4] * (nRasterXSize - 1);
    dfIGEOLOLRX = dfIGEOLOULX + padfGeoTransform[1] * (nRasterXSize - 1)
                              + padfGeoTransform[2] * (nRasterYSize - 1);
    dfIGEOLOLRY = dfIGEOLOULY + padfGeoTransform[4] * (nRasterXSize - 1)
                              + padfGeoTransform[5] * (nRasterYSize - 1);
    dfIGEOLOLLX = dfIGEOLOULX + padfGeoTransform[2] * (nRasterYSize - 1);
    dfIGEOLOLLY = dfIGEOLOULY + padfGeoTransform[5] * (nRasterYSize - 1);

    if( NITFWriteIGEOLO( psImage, psImage->chICORDS, 
                         psImage->nZone, 
                         dfIGEOLOULX, dfIGEOLOULY, dfIGEOLOURX, dfIGEOLOURY, 
                         dfIGEOLOLRX, dfIGEOLOLRY, dfIGEOLOLLX, dfIGEOLOLLY ) )
        return CE_None;
    else
        return GDALPamDataset::SetGeoTransform( padfGeoTransform );
}

/************************************************************************/
/*                          GetProjectionRef()                          */
/************************************************************************/

const char *NITFDataset::GetProjectionRef()

{
    if( bGotGeoTransform )
        return pszProjection;
    else
        return GDALPamDataset::GetProjectionRef();
}

/************************************************************************/
/*                            SetProjection()                           */
/************************************************************************/

CPLErr NITFDataset::SetProjection(const char* _pszProjection)

{
    int    bNorth;
    OGRSpatialReference oSRS, oSRS_WGS84;
    char *pszWKT = (char *) _pszProjection;

    if( pszWKT != NULL )
        oSRS.importFromWkt( &pszWKT );
    else
        return CE_Failure;

    oSRS_WGS84.SetWellKnownGeogCS( "WGS84" );
    if ( oSRS.IsSameGeogCS(&oSRS_WGS84) == FALSE)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "NITF only supports WGS84 geographic and UTM projections.\n");
        return CE_Failure;
    }

    if( oSRS.IsGeographic() && oSRS.GetPrimeMeridian() == 0.0)
    {
        if (psImage->chICORDS != 'G')
        {
            CPLError(CE_Failure, CPLE_NotSupported,
                     "NITF file should have been created with creation option 'ICOORDS=G'.\n");
            return CE_Failure;
        }
    }
    else if( oSRS.GetUTMZone( &bNorth ) > 0)
    {
        if (bNorth && psImage->chICORDS != 'N')
        {
            CPLError(CE_Failure, CPLE_NotSupported,
                     "NITF file should have been created with creation option 'ICOORDS=N'.\n");
            return CE_Failure;
        }
        else if (!bNorth && psImage->chICORDS != 'S')
        {
            CPLError(CE_Failure, CPLE_NotSupported,
                     "NITF file should have been created with creation option 'ICOORDS=S'.\n");
            return CE_Failure;
        }

        psImage->nZone = oSRS.GetUTMZone( NULL );
    }
    else
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "NITF only supports WGS84 geographic and UTM projections.\n");
        return CE_Failure;
    }

    CPLFree(pszProjection);
    pszProjection = CPLStrdup(_pszProjection);

    SetGeoTransform(adfGeoTransform);

    return CE_None;
}

/************************************************************************/
/*                       InitializeCGMMetadata()                        */
/************************************************************************/

void NITFDataset::InitializeCGMMetadata()

{
    if( oSpecialMD.GetMetadataItem( "SEGMENT_COUNT", "CGM" ) != NULL )
        return;

    int iSegment;
    int iCGM = 0;
    char **papszCGMMetadata = NULL;

    papszCGMMetadata = 
        CSLSetNameValue( papszCGMMetadata, "SEGMENT_COUNT", "0" );

/* ==================================================================== */
/*      Process all graphics segments.                                  */
/* ==================================================================== */
    for( iSegment = 0; iSegment < psFile->nSegmentCount; iSegment++ )
    {
        NITFSegmentInfo *psSegment = psFile->pasSegmentInfo + iSegment;

        if( !EQUAL(psSegment->szSegmentType,"GR") 
            && !EQUAL(psSegment->szSegmentType,"SY") )
            continue;

/* -------------------------------------------------------------------- */
/*      Load the graphic subheader.                                     */
/* -------------------------------------------------------------------- */
        char achSubheader[298];
        int  nSTYPEOffset;

        if( VSIFSeekL( psFile->fp, psSegment->nSegmentHeaderStart, 
                       SEEK_SET ) != 0 
            || VSIFReadL( achSubheader, 1, sizeof(achSubheader), 
                          psFile->fp ) < 258 )
        {
            CPLError( CE_Warning, CPLE_FileIO, 
                      "Failed to read graphic subheader at %d.", 
                      psSegment->nSegmentHeaderStart );
            return;
        }

        // NITF 2.0. (also works for NITF 2.1)
        nSTYPEOffset = 200;
        if( EQUALN(achSubheader+193,"999998",6) )
            nSTYPEOffset += 40;

        // We don't want bitmaps or anything other than cgm for now.
        if( achSubheader[nSTYPEOffset] != 'C' )
            continue;

        char szSLOC_R[6], szSLOC_C[6];

        strncpy( szSLOC_R, achSubheader + nSTYPEOffset + 20, 5 );
        strncpy( szSLOC_C, achSubheader + nSTYPEOffset + 20 + 5, 5 );
        
        szSLOC_R[5] = '\0';
        szSLOC_C[5] = '\0';

        papszCGMMetadata = 
            CSLSetNameValue( papszCGMMetadata, 
                             CPLString().Printf("SEGMENT_%d_SLOC_ROW", iCGM), 
                             szSLOC_R );
        papszCGMMetadata = 
            CSLSetNameValue( papszCGMMetadata, 
                             CPLString().Printf("SEGMENT_%d_SLOC_COL", iCGM), 
                             szSLOC_C );

/* -------------------------------------------------------------------- */
/*      Load the raw CGM data itself.                                   */
/* -------------------------------------------------------------------- */
        char *pabyCGMData, *pszEscapedCGMData;

        pabyCGMData = (char *) CPLCalloc(1,psSegment->nSegmentSize);
        if( VSIFSeekL( psFile->fp, psSegment->nSegmentStart, 
                       SEEK_SET ) != 0 
            || VSIFReadL( pabyCGMData, 1, psSegment->nSegmentSize, 
                          psFile->fp ) != psSegment->nSegmentSize )
        {
            CPLError( CE_Warning, CPLE_FileIO, 
                      "Failed to read %d bytes of graphic data at %d.", 
                      psSegment->nSegmentSize,
                      psSegment->nSegmentStart );
            return;
        }

        pszEscapedCGMData = CPLEscapeString( pabyCGMData, 
                                             psSegment->nSegmentSize, 
                                             CPLES_BackslashQuotable );

        papszCGMMetadata = 
            CSLSetNameValue( papszCGMMetadata, 
                             CPLString().Printf("SEGMENT_%d_DATA", iCGM), 
                             pszEscapedCGMData );
        CPLFree( pszEscapedCGMData );
        CPLFree( pabyCGMData );

        iCGM++;
    }

/* -------------------------------------------------------------------- */
/*      Record the CGM segment count.                                   */
/* -------------------------------------------------------------------- */
    papszCGMMetadata = 
        CSLSetNameValue( papszCGMMetadata, 
                         "SEGMENT_COUNT", 
                         CPLString().Printf( "%d", iCGM ) );

    oSpecialMD.SetMetadata( papszCGMMetadata, "CGM" );

    CSLDestroy( papszCGMMetadata );
}

/************************************************************************/
/*                       InitializeTextMetadata()                       */
/************************************************************************/

void NITFDataset::InitializeTextMetadata()

{
    if( oSpecialMD.GetMetadata( "TEXT" ) != NULL )
        return;

    int iSegment;
    int iText = 0;

/* ==================================================================== */
/*      Process all graphics segments.                                  */
/* ==================================================================== */
    for( iSegment = 0; iSegment < psFile->nSegmentCount; iSegment++ )
    {
        NITFSegmentInfo *psSegment = psFile->pasSegmentInfo + iSegment;

        if( !EQUAL(psSegment->szSegmentType,"TX") )
            continue;

/* -------------------------------------------------------------------- */
/*      Load the raw TEXT data itself.                                  */
/* -------------------------------------------------------------------- */
        char *pabyTextData;

        /* Allocate one extra byte for the NULL terminating character */
        pabyTextData = (char *) CPLCalloc(1,psSegment->nSegmentSize+1);
        if( VSIFSeekL( psFile->fp, psSegment->nSegmentStart, 
                       SEEK_SET ) != 0 
            || VSIFReadL( pabyTextData, 1, psSegment->nSegmentSize, 
                          psFile->fp ) != psSegment->nSegmentSize )
        {
            CPLError( CE_Warning, CPLE_FileIO, 
                      "Failed to read %d bytes of text data at %d.", 
                      psSegment->nSegmentSize,
                      psSegment->nSegmentStart );
            return;
        }

        oSpecialMD.SetMetadataItem( CPLString().Printf( "DATA_%d", iText),
                                    pabyTextData, "TEXT" );
        CPLFree( pabyTextData );

        iText++;
    }
}

/************************************************************************/
/*                       InitializeTREMetadata()                        */
/************************************************************************/

void NITFDataset::InitializeTREMetadata()

{
    if( oSpecialMD.GetMetadata( "TRE" ) != NULL )
        return;

/* -------------------------------------------------------------------- */
/*      Loop over TRE sources (file and image).                         */
/* -------------------------------------------------------------------- */
    int nTRESrc;

    for( nTRESrc = 0; nTRESrc < 2; nTRESrc++ )
    {
        int nTREBytes;
        char *pszTREData;

        if( nTRESrc == 0 )
        {
            nTREBytes = psFile->nTREBytes;
            pszTREData = psFile->pachTRE;
        }
        else
        {
            if( psImage ) 
            {
                nTREBytes = psImage->nTREBytes;
                pszTREData = psImage->pachTRE;
            }
            else
            {
                nTREBytes = 0;
                pszTREData = NULL;
            }
        }

/* -------------------------------------------------------------------- */
/*      Loop over TREs.                                                 */
/* -------------------------------------------------------------------- */

        while( nTREBytes >= 11 )
        {
            char szTemp[100];
            char szTag[7];
            char *pszEscapedData;
            int nThisTRESize = atoi(NITFGetField(szTemp, pszTREData, 6, 5 ));

            if (nThisTRESize > nTREBytes - 11)
            {
                CPLError(CE_Failure, CPLE_AppDefined, "Not enough bytes in TRE");
                return;
            }

            strncpy( szTag, pszTREData, 6 );
            szTag[6] = '\0';

            // trim white off tag. 
            while( strlen(szTag) > 0 && szTag[strlen(szTag)-1] == ' ' )
                szTag[strlen(szTag)-1] = '\0';
            
            // escape data. 
            pszEscapedData = CPLEscapeString( pszTREData + 11,
                                              nThisTRESize,
                                              CPLES_BackslashQuotable );

            oSpecialMD.SetMetadataItem( szTag, pszEscapedData, "TRE" );
            CPLFree( pszEscapedData );
            
            nTREBytes -= (nThisTRESize + 11);
            pszTREData += (nThisTRESize + 11);
        }
    }
}

/************************************************************************/
/*                            GetMetadata()                             */
/************************************************************************/

char **NITFDataset::GetMetadata( const char * pszDomain )

{
    if( pszDomain != NULL && EQUAL(pszDomain,"CGM") )
    {
        InitializeCGMMetadata();
        return oSpecialMD.GetMetadata( pszDomain );
    }

    if( pszDomain != NULL && EQUAL(pszDomain,"TEXT") )
    {
        InitializeTextMetadata();
        return oSpecialMD.GetMetadata( pszDomain );
    }

    if( pszDomain != NULL && EQUAL(pszDomain,"TRE") )
    {
        InitializeTREMetadata();
        return oSpecialMD.GetMetadata( pszDomain );
    }

    return GDALPamDataset::GetMetadata( pszDomain );
}

/************************************************************************/
/*                          GetMetadataItem()                           */
/************************************************************************/

const char *NITFDataset::GetMetadataItem(const char * pszName,
                                         const char * pszDomain )

{
    if( pszDomain != NULL && EQUAL(pszDomain,"CGM") )
    {
        InitializeCGMMetadata();
        return oSpecialMD.GetMetadataItem( pszName, pszDomain );
    }

    if( pszDomain != NULL && EQUAL(pszDomain,"TEXT") )
    {
        InitializeTextMetadata();
        return oSpecialMD.GetMetadataItem( pszName, pszDomain );
    }

    if( pszDomain != NULL && EQUAL(pszDomain,"TRE") )
    {
        InitializeTREMetadata();
        return oSpecialMD.GetMetadataItem( pszName, pszDomain );
    }

    return GDALPamDataset::GetMetadataItem( pszName, pszDomain );
}


/************************************************************************/
/*                            GetGCPCount()                             */
/************************************************************************/

int NITFDataset::GetGCPCount()

{
    return nGCPCount;
}

/************************************************************************/
/*                          GetGCPProjection()                          */
/************************************************************************/

const char *NITFDataset::GetGCPProjection()

{
    if( nGCPCount > 0 && pszGCPProjection != NULL )
        return pszGCPProjection;
    else
        return "";
}

/************************************************************************/
/*                               GetGCP()                               */
/************************************************************************/

const GDAL_GCP *NITFDataset::GetGCPs()

{
    return pasGCPList;
}

/************************************************************************/
/*                           ScanJPEGQLevel()                           */
/*                                                                      */
/*      Search the NITF APP header in the jpeg data stream to find      */
/*      out what predefined Q level tables should be used (or -1 if     */
/*      they are inline).                                               */
/************************************************************************/

int NITFDataset::ScanJPEGQLevel( GUInt32 *pnDataStart )

{
    GByte abyHeader[100];

    if( VSIFSeekL( psFile->fp, *pnDataStart,
                   SEEK_SET ) != 0 )
    {
        CPLError( CE_Failure, CPLE_FileIO, 
                  "Seek error to jpeg data stream." );
        return 0;
    }
        
    if( VSIFReadL( abyHeader, 1, sizeof(abyHeader), psFile->fp ) 
        < sizeof(abyHeader) )
    {
        CPLError( CE_Failure, CPLE_FileIO, 
                  "Read error to jpeg data stream." );
        return 0;
    }

/* -------------------------------------------------------------------- */
/*      Scan ahead for jpeg magic code.  In some files (eg. NSIF)       */
/*      there seems to be some extra junk before the image data stream. */
/* -------------------------------------------------------------------- */
    GUInt32 nOffset = 0;
    while( nOffset < sizeof(abyHeader) - 23 
           && (abyHeader[nOffset+0] != 0xff
               || abyHeader[nOffset+1] != 0xd8
               || abyHeader[nOffset+2] != 0xff) )
        nOffset++;

    if( nOffset >= sizeof(abyHeader) - 23 )
        return 0;

    *pnDataStart += nOffset;

    if( nOffset > 0 )
        CPLDebug( "NITF", 
                  "JPEG data stream at offset %d from start of data segement, NSIF?", 
                  nOffset );

/* -------------------------------------------------------------------- */
/*      Do we have an NITF app tag?  If so, pull out the Q level.       */
/* -------------------------------------------------------------------- */
    if( !EQUAL((char *)abyHeader+nOffset+6,"NITF") )
        return 0;

    return abyHeader[22+nOffset];
}

/************************************************************************/
/*                           ScanJPEGBlocks()                           */
/************************************************************************/

CPLErr NITFDataset::ScanJPEGBlocks()

{
    int iBlock;
    GUInt32 nJPEGStart = 
        psFile->pasSegmentInfo[psImage->iSegment].nSegmentStart;

    nQLevel = ScanJPEGQLevel( &nJPEGStart );

/* -------------------------------------------------------------------- */
/*      Allocate offset array                                           */
/* -------------------------------------------------------------------- */
    panJPEGBlockOffset = (int *) 
        CPLCalloc(sizeof(int),
                  psImage->nBlocksPerRow*psImage->nBlocksPerColumn);
    panJPEGBlockOffset[0] = nJPEGStart;

    if ( psImage->nBlocksPerRow * psImage->nBlocksPerColumn == 1)
        return CE_None;

    for( iBlock = psImage->nBlocksPerRow * psImage->nBlocksPerColumn - 1;
         iBlock > 0; iBlock-- )
        panJPEGBlockOffset[iBlock] = -1;
    
/* -------------------------------------------------------------------- */
/*      Scan through the whole image data stream identifying all        */
/*      block boundaries.  Each block starts with 0xFFD8 (SOI).         */
/*      They also end with 0xFFD9, but we don't currently look for      */
/*      that.                                                           */
/* -------------------------------------------------------------------- */
    int iNextBlock = 1;
    int iSegOffset = 2;
    int iSegSize = psFile->pasSegmentInfo[psImage->iSegment].nSegmentSize
        - (nJPEGStart - psFile->pasSegmentInfo[psImage->iSegment].nSegmentStart);
    GByte abyBlock[512];

    while( iSegOffset < iSegSize-1 )
    {
        int nReadSize = MIN((int)sizeof(abyBlock),iSegSize - iSegOffset);
        int i;

        if( VSIFSeekL( psFile->fp, panJPEGBlockOffset[0] + iSegOffset, 
                       SEEK_SET ) != 0 )
        {
            CPLError( CE_Failure, CPLE_FileIO, 
                      "Seek error to jpeg data stream." );
            return CE_Failure;
        }
        
        if( VSIFReadL( abyBlock, 1, nReadSize, psFile->fp ) < (size_t)nReadSize)
        {
            CPLError( CE_Failure, CPLE_FileIO, 
                      "Read error to jpeg data stream." );
            return CE_Failure;
        }
        
        for( i = 0; i < nReadSize-1; i++ )
        {
            if( abyBlock[i] == 0xff && abyBlock[i+1] == 0xd8 )
            {
                panJPEGBlockOffset[iNextBlock++] 
                    = panJPEGBlockOffset[0] + iSegOffset + i; 

                if( iNextBlock == psImage->nBlocksPerRow*psImage->nBlocksPerColumn)
                {
                    return CE_None;
                }
            }
        }

        iSegOffset += nReadSize - 1;
    }

    return CE_None;
}

/************************************************************************/
/*                           ReadJPEGBlock()                            */
/************************************************************************/

CPLErr NITFDataset::ReadJPEGBlock( int iBlockX, int iBlockY )

{
    CPLErr eErr;

/* -------------------------------------------------------------------- */
/*      If this is our first request, do a scan for block boundaries.   */
/* -------------------------------------------------------------------- */
    if( panJPEGBlockOffset == NULL )
    {
        if (EQUAL(psImage->szIC,"M3"))
        {
/* -------------------------------------------------------------------- */
/*      When a data mask subheader is present, we don't need to scan    */
/*      the whole file. We just use the psImage->panBlockStart table    */
/* -------------------------------------------------------------------- */
            panJPEGBlockOffset = (int *) 
                CPLCalloc(sizeof(int),
                        psImage->nBlocksPerRow*psImage->nBlocksPerColumn);
            int i;
            for (i=0;i< psImage->nBlocksPerRow*psImage->nBlocksPerColumn;i++)
            {
                panJPEGBlockOffset[i] = psImage->panBlockStart[i];
                if (panJPEGBlockOffset[i] != -1)
                {
                    GUInt32 nOffset = panJPEGBlockOffset[i];
                    nQLevel = ScanJPEGQLevel(&nOffset);
                    /* The beginning of the JPEG stream should be the offset */
                    /* from the panBlockStart table */
                    if (nOffset != (GUInt32)panJPEGBlockOffset[i])
                    {
                        CPLError(CE_Failure, CPLE_AppDefined,
                                 "JPEG block doesn't start at expected offset");
                        return CE_Failure;
                    }
                }
            }
        }
        else /* 'C3' case */
        {
/* -------------------------------------------------------------------- */
/*      Scan through the whole image data stream identifying all        */
/*      block boundaries.                                               */
/* -------------------------------------------------------------------- */
            eErr = ScanJPEGBlocks();
            if( eErr != CE_None )
                return eErr;
        }
    }
    
/* -------------------------------------------------------------------- */
/*    Allocate image data block (where the uncompressed image will go)  */
/* -------------------------------------------------------------------- */
    if( pabyJPEGBlock == NULL )
    {
        // this is really 8bit only for now. 
        pabyJPEGBlock = (GByte *) 
            CPLCalloc(psImage->nBands,
                      psImage->nBlockWidth * psImage->nBlockHeight);
    }


/* -------------------------------------------------------------------- */
/*      Read JPEG Chunk.                                                */
/* -------------------------------------------------------------------- */
    CPLString osFilename;
    int iBlock = iBlockX + iBlockY * psImage->nBlocksPerRow;
    GDALDataset *poDS;
    int anBands[3] = { 1, 2, 3 };

    if (panJPEGBlockOffset[iBlock] == -1)
    {
        memset(pabyJPEGBlock, 0, psImage->nBlockWidth*psImage->nBlockHeight);
        return CE_None;
    }

    osFilename.Printf( "JPEG_SUBFILE:Q%d,%d,%d,%s", 
                       nQLevel,
                       panJPEGBlockOffset[iBlock], 0, 
                       osNITFFilename.c_str() );

    poDS = (GDALDataset *) GDALOpen( osFilename, GA_ReadOnly );
    if( poDS == NULL )
        return CE_Failure;

    if( poDS->GetRasterXSize() != psImage->nBlockWidth
        || poDS->GetRasterYSize() != psImage->nBlockHeight )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "JPEG block %d not same size as NITF blocksize.", 
                  iBlock );
        delete poDS;
        return CE_Failure;
    }
                       
    eErr = poDS->RasterIO( GF_Read, 
                           0, 0, 
                           psImage->nBlockWidth, psImage->nBlockHeight,
                           pabyJPEGBlock, 
                           psImage->nBlockWidth, psImage->nBlockHeight,
                           GDT_Byte, psImage->nBands, anBands, 0, 0, 0 );

    delete poDS;

    return eErr;
}

/************************************************************************/
/*                         GDALToNITFDataType()                         */
/************************************************************************/

static const char *GDALToNITFDataType( GDALDataType eType )

{
    const char *pszPVType;

    switch( eType )
    {
      case GDT_Byte:
      case GDT_UInt16:
      case GDT_UInt32:
        pszPVType = "INT";
        break;

      case GDT_Int16:
      case GDT_Int32:
        pszPVType = "SI";
        break;

      case GDT_Float32:
      case GDT_Float64:
        pszPVType = "R";
        break;

      case GDT_CInt16:
      case GDT_CInt32:
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "NITF format does not support complex integer data." );
        return NULL;

      case GDT_CFloat32:
      case GDT_CFloat64:
        pszPVType = "C";
        break;

      default:
        CPLError( CE_Failure, CPLE_AppDefined,
                  "Unsupported raster pixel type (%d).", 
                  (int) eType );
        return NULL;
    }

    return pszPVType;
}

/************************************************************************/
/*                           NITFJP2Options()                           */
/*                                                                      */
/*      Prepare JP2-in-NITF creation options based in part of the       */
/*      NITF creation options.                                          */
/************************************************************************/

static const char **NITFJP2Options( char **papszOptions )

{
    int i;
    static const char *apszOptions[] = { 
        "PROFILE=NPJE", 
        "CODESTREAM_ONLY=TRUE", 
        NULL,
        NULL };
    
    for( i = 0; papszOptions != NULL && papszOptions[i] != NULL; i++ )
    {
        if( EQUALN(papszOptions[i],"PROFILE=",8) )
            apszOptions[0] = papszOptions[i];
        if( EQUALN(papszOptions[i],"TARGET=",7) )
            apszOptions[2] = papszOptions[i];
    }

    return apszOptions;
}

/************************************************************************/
/*                         NITFDatasetCreate()                          */
/************************************************************************/

static GDALDataset *
NITFDatasetCreate( const char *pszFilename, int nXSize, int nYSize, int nBands,
                   GDALDataType eType, char **papszOptions )

{
    const char *pszPVType = GDALToNITFDataType( eType );
    const char *pszIC = CSLFetchNameValue( papszOptions, "IC" );

    if( pszPVType == NULL )
        return NULL;

/* -------------------------------------------------------------------- */
/*      We disallow any IC value except NC when creating this way.      */
/* -------------------------------------------------------------------- */
    GDALDriver *poJ2KDriver = NULL;

    if( pszIC != NULL && EQUAL(pszIC,"C8") )
    {
        int bHasCreate = FALSE;

        poJ2KDriver = GetGDALDriverManager()->GetDriverByName( "JP2ECW" );
        if( poJ2KDriver != NULL )
            bHasCreate = poJ2KDriver->GetMetadataItem( GDAL_DCAP_CREATE, 
                                                       NULL ) != NULL;
        if( !bHasCreate )
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Unable to create JPEG2000 encoded NITF files.  The\n"
                      "JP2ECW driver is unavailable, or missing Create support." );
            return NULL;
        }
    }

    else if( pszIC != NULL && !EQUAL(pszIC,"NC") )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Unsupported compression (IC=%s) used in direct\n"
                  "NITF File creation", 
                  pszIC );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Create the file.                                                */
/* -------------------------------------------------------------------- */

    if( !NITFCreate( pszFilename, nXSize, nYSize, nBands, 
                     GDALGetDataTypeSize( eType ), pszPVType, 
                     papszOptions ) )
        return NULL;

/* -------------------------------------------------------------------- */
/*      Various special hacks related to JPEG2000 encoded files.        */
/* -------------------------------------------------------------------- */
    if( poJ2KDriver )
    {
        NITFFile *psFile = NITFOpen( pszFilename, TRUE );
        int nImageOffset = psFile->pasSegmentInfo[0].nSegmentStart;

        CPLString osDSName;

        osDSName.Printf("J2K_SUBFILE:%d,%d,%s", nImageOffset, -1, pszFilename);

        NITFClose( psFile );

        poWritableJ2KDataset = 
            poJ2KDriver->Create( osDSName, nXSize, nYSize, nBands, eType, 
                                 (char **)NITFJP2Options( papszOptions ) );

        if( poWritableJ2KDataset == NULL )
            return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Open the dataset in update mode.                                */
/* -------------------------------------------------------------------- */
    return (GDALDataset *) GDALOpen( pszFilename, GA_Update );
}

/************************************************************************/
/*                           NITFCreateCopy()                           */
/************************************************************************/

GDALDataset *
NITFDataset::NITFCreateCopy( 
    const char *pszFilename, GDALDataset *poSrcDS,
    int bStrict, char **papszOptions, 
    GDALProgressFunc pfnProgress, void * pProgressData )

{
    GDALDataType eType;
    GDALRasterBand *poBand1;
    char  **papszFullOptions = CSLDuplicate( papszOptions );
    int   bJPEG2000 = FALSE;
    int   bJPEG = FALSE;
    NITFDataset *poDstDS = NULL;
    GDALDriver *poJ2KDriver = NULL;

    int  nBands = poSrcDS->GetRasterCount();
    if( nBands == 0 )
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "Unable to export files with zero bands." );
        CSLDestroy(papszFullOptions);
        return NULL;
    }

    poBand1 = poSrcDS->GetRasterBand(1);
    if( poBand1 == NULL )
    {
        CSLDestroy(papszFullOptions);
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Only allow supported compression values.                        */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue( papszOptions, "IC" ) != NULL )
    {
        if( EQUAL(CSLFetchNameValue( papszOptions, "IC" ),"NC") )
            /* ok */;
        else if( EQUAL(CSLFetchNameValue( papszOptions, "IC" ),"C8") )
        {
            poJ2KDriver = 
                GetGDALDriverManager()->GetDriverByName( "JP2ECW" );
            if( poJ2KDriver == NULL )
            {
                CPLError( 
                    CE_Failure, CPLE_AppDefined, 
                    "Unable to write JPEG2000 compressed NITF file.\n"
                    "No 'subfile' JPEG2000 write supporting drivers are\n"
                    "configured." );
                CSLDestroy(papszFullOptions);
                return NULL;
            }
            bJPEG2000 = TRUE;
        }
        else if( EQUAL(CSLFetchNameValue( papszOptions, "IC" ),"C3") )
        {
            bJPEG = TRUE;
#ifndef JPEG_SUPPORTED
            CPLError( 
                CE_Failure, CPLE_AppDefined, 
                "Unable to write JPEG compressed NITF file.\n"
                "Libjpeg is not configured into build." );
            CSLDestroy(papszFullOptions);
            return NULL;
#endif
        }
        else
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Only IC=NC (uncompressed), IC=C3 (JPEG) and IC=C8 (JPEG2000)\n"
                      "allowed with NITF CreateCopy method." );
            CSLDestroy(papszFullOptions);
            return NULL;
        }
    }

/* -------------------------------------------------------------------- */
/*      Get the data type.  Complex integers isn't supported by         */
/*      NITF, so map that to complex float if we aren't in strict       */
/*      mode.                                                           */
/* -------------------------------------------------------------------- */
    eType = poBand1->GetRasterDataType();
    if( !bStrict && (eType == GDT_CInt16 || eType == GDT_CInt32) )
        eType = GDT_CFloat32;

/* -------------------------------------------------------------------- */
/*      Copy over other source metadata items as creation options       */
/*      that seem useful.                                               */
/* -------------------------------------------------------------------- */
    char **papszSrcMD = poSrcDS->GetMetadata();
    int iMD;

    for( iMD = 0; papszSrcMD && papszSrcMD[iMD]; iMD++ )
    {
        if( EQUALN(papszSrcMD[iMD],"NITF_BLOCKA",11) 
            || EQUALN(papszSrcMD[iMD],"NITF_FHDR",9) )
        {
            char *pszName = NULL;
            const char *pszValue = CPLParseNameValue( papszSrcMD[iMD], 
                                                      &pszName );
            if( CSLFetchNameValue( papszFullOptions, pszName+5 ) == NULL )
                papszFullOptions = 
                    CSLSetNameValue( papszFullOptions, pszName+5, pszValue );
            CPLFree(pszName);
        }
    }

/* -------------------------------------------------------------------- */
/*      Copy TRE definitions as creation options.                       */
/* -------------------------------------------------------------------- */
    papszSrcMD = poSrcDS->GetMetadata( "TRE" );

    for( iMD = 0; papszSrcMD && papszSrcMD[iMD]; iMD++ )
    {
        CPLString osTRE;

        osTRE = "TRE=";
        osTRE += papszSrcMD[iMD];

        papszFullOptions = CSLAddString( papszFullOptions, osTRE );
    }

/* -------------------------------------------------------------------- */
/*      Prepare for text segments.                                      */
/* -------------------------------------------------------------------- */
    int iOpt, nNUMT = 0;
    char **papszTextMD = poSrcDS->GetMetadata( "TEXT" );

    for( iOpt = 0; 
         papszTextMD != NULL && papszTextMD[iOpt] != NULL; 
         iOpt++ )
    {
        if( !EQUALN(papszTextMD[iOpt],"DATA_",5) )
            continue;

        nNUMT++;
    }

    if( nNUMT > 0 )
    {
        papszFullOptions = CSLAddString( papszFullOptions, 
                                         CPLString().Printf( "NUMT=%d", 
                                                             nNUMT ) );
    }

/* -------------------------------------------------------------------- */
/*      Set if we can set IREP.                                         */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue(papszFullOptions,"IREP") == NULL )
    {
        if ( ((poSrcDS->GetRasterCount() == 3 && bJPEG) ||
              (poSrcDS->GetRasterCount() >= 3 && !bJPEG)) && eType == GDT_Byte &&
             poSrcDS->GetRasterBand(1)->GetColorInterpretation() == GCI_RedBand &&
             poSrcDS->GetRasterBand(2)->GetColorInterpretation() == GCI_GreenBand &&
             poSrcDS->GetRasterBand(3)->GetColorInterpretation() == GCI_BlueBand)
        {
            if( bJPEG )
                papszFullOptions = 
                    CSLSetNameValue( papszFullOptions, "IREP", "YCbCr601" );
            else
                papszFullOptions = 
                    CSLSetNameValue( papszFullOptions, "IREP", "RGB" );
        }
        else if( poSrcDS->GetRasterCount() == 1 && eType == GDT_Byte
                 && poBand1->GetColorTable() != NULL )
        {
            papszFullOptions = 
                CSLSetNameValue( papszFullOptions, "IREP", "RGB/LUT" );
            papszFullOptions = 
                CSLSetNameValue( papszFullOptions, "LUT_SIZE", 
                                 CPLString().Printf(
                                     "%d", poBand1->GetColorTable()->GetColorEntryCount()) );
        }
        else if( GDALDataTypeIsComplex(eType) )
            papszFullOptions = 
                CSLSetNameValue( papszFullOptions, "IREP", "NODISPLY" );
        
        else
            papszFullOptions = 
                CSLSetNameValue( papszFullOptions, "IREP", "MONO" );
    }

/* -------------------------------------------------------------------- */
/*      Do we have lat/long georeferencing information?                 */
/* -------------------------------------------------------------------- */
    double adfGeoTransform[6];
    int    bWriteGeoTransform = FALSE;
    int    bNorth, nZone = 0;
    OGRSpatialReference oSRS, oSRS_WGS84;
    char *pszWKT = (char *) poSrcDS->GetProjectionRef();

    if( pszWKT != NULL && pszWKT[0] != '\0' )
    {
        oSRS.importFromWkt( &pszWKT );

        /* NITF is only WGS84 */
        oSRS_WGS84.SetWellKnownGeogCS( "WGS84" );
        if ( oSRS.IsSameGeogCS(&oSRS_WGS84) == FALSE)
        {
            CPLError((bStrict) ? CE_Failure : CE_Warning, CPLE_NotSupported,
                    "NITF only supports WGS84 geographic and UTM projections.\n");
            if (bStrict)
            {
                CSLDestroy(papszFullOptions);
                return NULL;
            }
        }

        if( oSRS.IsGeographic() && oSRS.GetPrimeMeridian() == 0.0 
            && poSrcDS->GetGeoTransform( adfGeoTransform ) == CE_None )
        {
            papszFullOptions = 
                CSLSetNameValue( papszFullOptions, "ICORDS", "G" );
            bWriteGeoTransform = TRUE;
        }

        else if( oSRS.GetUTMZone( &bNorth ) > 0 
            && poSrcDS->GetGeoTransform( adfGeoTransform ) == CE_None )
        {
            if( bNorth )
                papszFullOptions = 
                    CSLSetNameValue( papszFullOptions, "ICORDS", "N" );
            else
                papszFullOptions = 
                    CSLSetNameValue( papszFullOptions, "ICORDS", "S" );

            nZone = oSRS.GetUTMZone( NULL );
            bWriteGeoTransform = TRUE;
        }
        else
        {
            CPLError((bStrict) ? CE_Failure : CE_Warning, CPLE_NotSupported,
                    "NITF only supports WGS84 geographic and UTM projections.\n");
            if (bStrict)
            {
                CSLDestroy(papszFullOptions);
                return NULL;
            }
        }
    }

/* -------------------------------------------------------------------- */
/*      Create the output file.                                         */
/* -------------------------------------------------------------------- */
    int nXSize = poSrcDS->GetRasterXSize();
    int nYSize = poSrcDS->GetRasterYSize();
    const char *pszPVType = GDALToNITFDataType( eType );

    if( pszPVType == NULL )
    {
        CSLDestroy(papszFullOptions);
        return NULL;
    }

    if (!NITFCreate( pszFilename, nXSize, nYSize, poSrcDS->GetRasterCount(),
                GDALGetDataTypeSize( eType ), pszPVType, 
                papszFullOptions ))
    {
        CSLDestroy( papszFullOptions );
        return NULL;
    }

    CSLDestroy( papszFullOptions );
    papszFullOptions = NULL;

/* ==================================================================== */
/*      JPEG2000 case.  We need to write the data through a J2K         */
/*      driver in pixel interleaved form.                               */
/* ==================================================================== */
    if( bJPEG2000 )
    {
        NITFFile *psFile = NITFOpen( pszFilename, TRUE );
        GDALDataset *poJ2KDataset = NULL;
        int nImageOffset = psFile->pasSegmentInfo[0].nSegmentStart;
        CPLString osDSName;

        osDSName.Printf( "J2K_SUBFILE:%d,%d,%s", nImageOffset, -1,
                         pszFilename );

        NITFClose( psFile );

        poJ2KDataset = 
            poJ2KDriver->CreateCopy( osDSName, poSrcDS, FALSE,
                                     (char **)NITFJP2Options(papszOptions),
                                     pfnProgress, pProgressData );
        if( poJ2KDataset == NULL )
            return NULL;

        delete poJ2KDataset;

        // Now we need to figure out the actual length of the file
        // and correct the image segment size information.
        GIntBig nPixelCount = nXSize * ((GIntBig) nYSize) * 
            poSrcDS->GetRasterCount();

        NITFPatchImageLength( pszFilename, nImageOffset, nPixelCount, "C8" );
        NITFWriteTextSegments( pszFilename, papszTextMD );

        poDstDS = (NITFDataset *) GDALOpen( pszFilename, GA_Update );

        if( poDstDS == NULL )
            return NULL;
    }

/* ==================================================================== */
/*      Loop copying bands to an uncompressed file.                     */
/* ==================================================================== */
    else if( bJPEG )
    {
#ifdef JPEG_SUPPORTED
        NITFFile *psFile = NITFOpen( pszFilename, TRUE );
        int nImageOffset = psFile->pasSegmentInfo[0].nSegmentStart;
        int bSuccess;
        
        bSuccess = 
            NITFWriteJPEGImage( poSrcDS, psFile->fp, nImageOffset,
                                papszOptions,
                                pfnProgress, pProgressData );
        
        if( !bSuccess )
        {
            NITFClose( psFile );
            return NULL;
        }

        // Now we need to figure out the actual length of the file
        // and correct the image segment size information.
        GIntBig nPixelCount = nXSize * ((GIntBig) nYSize) * 
            poSrcDS->GetRasterCount();

        NITFClose( psFile );

        NITFPatchImageLength( pszFilename, nImageOffset,
                              nPixelCount, "C3" );
        NITFWriteTextSegments( pszFilename, papszTextMD );
        
        poDstDS = (NITFDataset *) GDALOpen( pszFilename, GA_Update );

        if( poDstDS == NULL )
            return NULL;
#endif /* def JPEG_SUPPORTED */
    }

/* ==================================================================== */
/*      Loop copying bands to an uncompressed file.                     */
/* ==================================================================== */
    else
    {
        NITFWriteTextSegments( pszFilename, papszTextMD );

        poDstDS = (NITFDataset *) GDALOpen( pszFilename, GA_Update );
        if( poDstDS == NULL )
            return NULL;
        
        for( int iBand = 0; iBand < poSrcDS->GetRasterCount(); iBand++ )
        {
            GDALRasterBand *poSrcBand = poSrcDS->GetRasterBand( iBand+1 );
            GDALRasterBand *poDstBand = poDstDS->GetRasterBand( iBand+1 );

/* -------------------------------------------------------------------- */
/*      Do we need to copy a colortable or other metadata?              */
/* -------------------------------------------------------------------- */
            GDALColorTable *poCT;

            poCT = poSrcBand->GetColorTable();
            if( poCT != NULL )
                poDstBand->SetColorTable( poCT );

/* -------------------------------------------------------------------- */
/*      Copy image data.                                                */
/* -------------------------------------------------------------------- */
            void           *pData;
            CPLErr         eErr;

            pData = CPLMalloc(nXSize * GDALGetDataTypeSize(eType) / 8);

            for( int iLine = 0; iLine < nYSize; iLine++ )
            {
                eErr = poSrcBand->RasterIO( GF_Read, 0, iLine, nXSize, 1, 
                                            pData, nXSize, 1, eType, 0, 0 );
                if( eErr != CE_None )
                {
                    return NULL;
                }
            
                eErr = poDstBand->RasterIO( GF_Write, 0, iLine, nXSize, 1, 
                                            pData, nXSize, 1, eType, 0, 0 );

                if( eErr != CE_None )
                {
                    return NULL;
                }

                if( !pfnProgress( (iBand + (iLine+1) / (double) nYSize)
                                  / (double) poSrcDS->GetRasterCount(), 
                                  NULL, pProgressData ) )
                {
                    CPLError( CE_Failure, CPLE_UserInterrupt, "User terminated" );
                    delete poDstDS;
                    return NULL;
                }
            }

            CPLFree( pData );
        }
    }

/* -------------------------------------------------------------------- */
/*      Set the georeferencing.                                         */
/* -------------------------------------------------------------------- */
    if( bWriteGeoTransform )
    {
        poDstDS->psImage->nZone = nZone;
        poDstDS->SetGeoTransform( adfGeoTransform );
    }

    poDstDS->CloneInfo( poSrcDS, GCIF_PAM_DEFAULT );

    return poDstDS;
}

/************************************************************************/
/*                        NITFPatchImageLength()                        */
/*                                                                      */
/*      Fixup various stuff we don't know till we have written the      */
/*      imagery.  In particular the file length, image data length      */
/*      and the compression ratio achieved.                             */
/************************************************************************/

static void NITFPatchImageLength( const char *pszFilename,
                                  long nImageOffset,
                                  GIntBig nPixelCount,
                                  const char *pszIC )

{
    FILE *fpVSIL = VSIFOpenL( pszFilename, "r+b" );
    if( fpVSIL == NULL )
        return;
    
    VSIFSeekL( fpVSIL, 0, SEEK_END );
    GIntBig nFileLen = VSIFTellL( fpVSIL );

/* -------------------------------------------------------------------- */
/*      Update total file length.                                       */
/* -------------------------------------------------------------------- */
    VSIFSeekL( fpVSIL, 342, SEEK_SET );
    VSIFWriteL( (void *) CPLString().Printf("%012ld",(long)nFileLen).c_str(),
                1, 12, fpVSIL );
    
/* -------------------------------------------------------------------- */
/*      Update the image data length.                                   */
/* -------------------------------------------------------------------- */
    VSIFSeekL( fpVSIL, 369, SEEK_SET );
    VSIFWriteL( (void *) 
        CPLString().Printf("%010ld",(long)(nFileLen-nImageOffset)).c_str(), 
        1, 10, fpVSIL );

/* -------------------------------------------------------------------- */
/*      Update COMRAT, the compression rate variable.  It is a bit      */
/*      hard to know right here whether we have an IGEOLO segment,      */
/*      so the COMRAT will either be at offset 778 or 838.              */
/* -------------------------------------------------------------------- */
    char szICBuf[2];
    VSIFSeekL( fpVSIL, 779-2, SEEK_SET );
    VSIFReadL( szICBuf, 2, 1, fpVSIL );
    if( !EQUALN(szICBuf,pszIC,2) )
    {
        VSIFSeekL( fpVSIL, 839-2, SEEK_SET );
        VSIFReadL( szICBuf, 2, 1, fpVSIL );
    }
    
    if( !EQUALN(szICBuf,pszIC,2) )
    {
        CPLError( CE_Warning, CPLE_AppDefined, 
                  "Unable to locate COMRAT to update in NITF header." );
    }
    else
    {
        char szCOMRAT[5];

        if( EQUAL(pszIC,"C8") ) /* jpeg2000 */
        {
            double dfRate = (nFileLen-nImageOffset) * 8 / (double) nPixelCount;
            dfRate = MAX(0.01,MIN(99.99,dfRate));
        
            // We emit in wxyz format with an implicit decimal place
            // between wx and yz as per spec for lossy compression. 
            // We really should have a special case for lossless compression.
            sprintf( szCOMRAT, "%04d", (int) (dfRate * 100));
        }
        else if( EQUAL(pszIC, "C3") ) /* jpeg */
        {
            strcpy( szCOMRAT, "00.0" );
        }

        VSIFWriteL( szCOMRAT, 4, 1, fpVSIL );
    }
    
    VSIFCloseL( fpVSIL );
}
        
/************************************************************************/
/*                       NITFWriteTextSegments()                        */
/************************************************************************/

static void NITFWriteTextSegments( const char *pszFilename,
                                   char **papszList )

{
/* -------------------------------------------------------------------- */
/*      Count the number of apparent text segments to write.  There     */
/*      is nothing at all to do if there are none to write.             */
/* -------------------------------------------------------------------- */
    int iOpt, nNUMT = 0;

    for( iOpt = 0; papszList != NULL && papszList[iOpt] != NULL; iOpt++ )
    {
        if( EQUALN(papszList[iOpt],"DATA_",5) )
            nNUMT++;
    }

    if( nNUMT == 0 )
        return;

/* -------------------------------------------------------------------- */
/*      Open the target file.                                           */
/* -------------------------------------------------------------------- */
    FILE *fpVSIL = VSIFOpenL( pszFilename, "r+b" );

    if( fpVSIL == NULL )
        return;
    
/* -------------------------------------------------------------------- */
/*      Confirm that the NUMT in the file header already matches the    */
/*      number of text segements we want to write, and that the         */
/*      segment header/data size info is blank.                         */
/* -------------------------------------------------------------------- */
    char achNUMT[4];
    char *pachLT = (char *) CPLCalloc(nNUMT * 9 + 1, 1);

    VSIFSeekL( fpVSIL, 385, SEEK_SET );
    VSIFReadL( achNUMT, 1, 3, fpVSIL );
    achNUMT[3] = '\0';

    VSIFReadL( pachLT, 1, nNUMT * 9, fpVSIL );

    if( atoi(achNUMT) != nNUMT )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "It appears an attempt was made to add or update text\n"
                  "segments on an NITF file with existing segments.  This\n"
                  "is not currently supported by the GDAL NITF driver." );

        VSIFCloseL( fpVSIL );
        return;
    }

    if( !EQUALN(pachLT,"         ",9) )
    {
        CPLFree( pachLT );
        // presumably the text segments are already written, do nothing.
        VSIFCloseL( fpVSIL );
        return;
    }

/* -------------------------------------------------------------------- */
/*      At this point we likely ought to confirm NUMDES, NUMRES,        */
/*      UDHDL and XHDL are zero.  Consider adding later...              */
/* -------------------------------------------------------------------- */

/* ==================================================================== */
/*      Write the text segments at the end of the file.                 */
/* ==================================================================== */
#define PLACE(location,name,text)  strncpy(location,text,strlen(text))
    int iTextSeg = 0;
    
    for( iOpt = 0; papszList != NULL && papszList[iOpt] != NULL; iOpt++ )
    {
        const char *pszTextToWrite;

        if( !EQUALN(papszList[iOpt],"DATA_",5) )
            continue;

/* -------------------------------------------------------------------- */
/*      Prepare and write text header.                                  */
/* -------------------------------------------------------------------- */
        VSIFSeekL( fpVSIL, 0, SEEK_END );

        char achTSH[282];

        memset( achTSH, ' ', sizeof(achTSH) );

        PLACE( achTSH+  0, TE            , "TE"                              );
        PLACE( achTSH+  9, TXTALVL       , "000"                             );
        PLACE( achTSH+ 12, TXTDT         , "00000000000000"                  );
        PLACE( achTSH+106, TSCLAS        , "U"                               );
        PLACE( achTSH+273, ENCRYP        , "0"                               );
        PLACE( achTSH+274, TXTFMT        , "STA"                             );
        PLACE( achTSH+277, TXSHDL        , "00000"                           );

        VSIFWriteL( achTSH, 1, sizeof(achTSH), fpVSIL );

/* -------------------------------------------------------------------- */
/*      Prepare and write text segment data.                            */
/* -------------------------------------------------------------------- */
        pszTextToWrite = CPLParseNameValue( papszList[iOpt], NULL );

        VSIFWriteL( pszTextToWrite, 1, strlen(pszTextToWrite), fpVSIL );
        
/* -------------------------------------------------------------------- */
/*      Update the subheader and data size info in the file header.     */
/* -------------------------------------------------------------------- */
        sprintf( pachLT + 9*iTextSeg+0, "%04d%05d",
                 (int) sizeof(achTSH), (int) strlen(pszTextToWrite) );

        iTextSeg++;
    }

/* -------------------------------------------------------------------- */
/*      Write out the text segment info.                                */
/* -------------------------------------------------------------------- */
    VSIFSeekL( fpVSIL, 388, SEEK_SET );
    VSIFWriteL( pachLT, 1, nNUMT * 9, fpVSIL );

/* -------------------------------------------------------------------- */
/*      Update total file length.                                       */
/* -------------------------------------------------------------------- */
    VSIFSeekL( fpVSIL, 0, SEEK_END );
    GIntBig nFileLen = VSIFTellL( fpVSIL );

    VSIFSeekL( fpVSIL, 342, SEEK_SET );
    VSIFWriteL( (void *) CPLString().Printf("%012ld",(long)nFileLen).c_str(),
                1, 12, fpVSIL );
    
    VSIFCloseL( fpVSIL );
}
        
/************************************************************************/
/*                         NITFWriteJPEGImage()                         */
/************************************************************************/

#ifdef JPEG_SUPPORTED

CPL_C_START
#include "jpeglib.h"
CPL_C_END

void jpeg_vsiio_src (j_decompress_ptr cinfo, FILE * infile);
void jpeg_vsiio_dest (j_compress_ptr cinfo, FILE * outfile);

static int 
NITFWriteJPEGBlock( GDALDataset *poSrcDS, FILE *fp,
                    int nBlockXOff, int nBlockYOff,
                    int nBlockXSize, int nBlockYSize,
                    int bProgressive, int nQuality,
                    GDALProgressFunc pfnProgress, void * pProgressData );

static int 
NITFWriteJPEGImage( GDALDataset *poSrcDS, FILE *fp, int nStartOffset, 
                    char **papszOptions,
                    GDALProgressFunc pfnProgress, void * pProgressData )
{
    int  nBands = poSrcDS->GetRasterCount();
    int  nXSize = poSrcDS->GetRasterXSize();
    int  nYSize = poSrcDS->GetRasterYSize();
    int  nQuality = 75;
    int  bProgressive = FALSE;

    if( !pfnProgress( 0.0, NULL, pProgressData ) )
        return FALSE;

/* -------------------------------------------------------------------- */
/*      Some some rudimentary checks                                    */
/* -------------------------------------------------------------------- */
    if( nBands != 1 && nBands != 3 )
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "JPEG driver doesn't support %d bands.  Must be 1 (grey) "
                  "or 3 (RGB) bands.\n", nBands );

        return FALSE;
    }

    GDALDataType eDT = poSrcDS->GetRasterBand(1)->GetRasterDataType();

#ifdef JPEG_LIB_MK1
    if( eDT != GDT_Byte && eDT != GDT_UInt16 && bStrict )
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "JPEG driver doesn't support data type %s. "
                  "Only eight and twelve bit bands supported (Mk1 libjpeg).\n",
                  GDALGetDataTypeName( 
                      poSrcDS->GetRasterBand(1)->GetRasterDataType()) );

        return FALSE;
    }

    if( eDT == GDT_UInt16 || eDT == GDT_Int16 )
        eDT = GDT_UInt16;
    else
        eDT = GDT_Byte;

#else
    if( eDT != GDT_Byte )
    {
        CPLError( CE_Failure, CPLE_NotSupported, 
                  "JPEG driver doesn't support data type %s. "
                  "Only eight bit byte bands supported.\n", 
                  GDALGetDataTypeName( 
                      poSrcDS->GetRasterBand(1)->GetRasterDataType()) );

        return FALSE;
    }
    
    eDT = GDT_Byte; // force to 8bit. 
#endif

/* -------------------------------------------------------------------- */
/*      What options has the user selected?                             */
/* -------------------------------------------------------------------- */
    if( CSLFetchNameValue(papszOptions,"QUALITY") != NULL )
    {
        nQuality = atoi(CSLFetchNameValue(papszOptions,"QUALITY"));
        if( nQuality < 10 || nQuality > 100 )
        {
            CPLError( CE_Failure, CPLE_IllegalArg,
                      "QUALITY=%s is not a legal value in the range 10-100.",
                      CSLFetchNameValue(papszOptions,"QUALITY") );
            return FALSE;
        }
    }

    bProgressive = CSLFetchBoolean( papszOptions, "PROGRESSIVE", FALSE );

/* -------------------------------------------------------------------- */
/*      Compute blocking factors                                        */
/* -------------------------------------------------------------------- */
    int nNPPBH = nXSize;
    int nNPPBV = nYSize;

    if( CSLFetchNameValue( papszOptions, "BLOCKSIZE" ) != NULL )
        nNPPBH = nNPPBV = atoi(CSLFetchNameValue( papszOptions, "BLOCKSIZE" ));

    if( CSLFetchNameValue( papszOptions, "BLOCKXSIZE" ) != NULL )
        nNPPBH = atoi(CSLFetchNameValue( papszOptions, "BLOCKXSIZE" ));

    if( CSLFetchNameValue( papszOptions, "BLOCKYSIZE" ) != NULL )
        nNPPBV = atoi(CSLFetchNameValue( papszOptions, "BLOCKYSIZE" ));
    
    if( CSLFetchNameValue( papszOptions, "NPPBH" ) != NULL )
        nNPPBH = atoi(CSLFetchNameValue( papszOptions, "NPPBH" ));
    
    if( CSLFetchNameValue( papszOptions, "NPPBV" ) != NULL )
        nNPPBV = atoi(CSLFetchNameValue( papszOptions, "NPPBV" ));
    
    if( nNPPBH <= 0 || nNPPBV <= 0 ||
        nNPPBH > 9999 || nNPPBV > 9999  )
        nNPPBH = nNPPBV = 256;

    int nNBPR = (nXSize + nNPPBH - 1) / nNPPBH;
    int nNBPC = (nYSize + nNPPBV - 1) / nNPPBV;

    VSIFSeekL( fp, nStartOffset, SEEK_SET );

/* -------------------------------------------------------------------- */
/*      Copy each block                                                 */
/* -------------------------------------------------------------------- */
    int nBlockXOff, nBlockYOff;
    for(nBlockYOff=0;nBlockYOff<nNBPC;nBlockYOff++)
    {
        for(nBlockXOff=0;nBlockXOff<nNBPR;nBlockXOff++)
        {
            /*CPLDebug("NITF", "nBlockXOff=%d/%d, nBlockYOff=%d/%d",
                     nBlockXOff, nNBPR, nBlockYOff, nNBPC);*/
            if (!NITFWriteJPEGBlock(poSrcDS, fp,
                                    nBlockXOff, nBlockYOff,
                                    nNPPBH, nNPPBV,
                                    bProgressive, nQuality,
                                    pfnProgress, pProgressData))
            {
                return FALSE;
            }
        }
    }
    return TRUE;
}

/************************************************************************/
/*                         NITFWriteJPEGBlock()                         */
/************************************************************************/

static int 
NITFWriteJPEGBlock( GDALDataset *poSrcDS, FILE *fp,
                    int nBlockXOff, int nBlockYOff,
                    int nBlockXSize, int nBlockYSize,
                    int bProgressive, int nQuality,
                    GDALProgressFunc pfnProgress, void * pProgressData )
{
    int  nBands = poSrcDS->GetRasterCount();
    int  nXSize = poSrcDS->GetRasterXSize();
    int  nYSize = poSrcDS->GetRasterYSize();
    int  anBandList[3] = {1,2,3};

/* -------------------------------------------------------------------- */
/*      Initialize JPG access to the file.                              */
/* -------------------------------------------------------------------- */
    struct jpeg_compress_struct sCInfo;
    struct jpeg_error_mgr sJErr;
    
    sCInfo.err = jpeg_std_error( &sJErr );
    jpeg_create_compress( &sCInfo );

    jpeg_vsiio_dest( &sCInfo, fp );
    
    sCInfo.image_width = nBlockXSize;
    sCInfo.image_height = nBlockYSize;
    sCInfo.input_components = nBands;

    if( nBands == 1 )
    {
        sCInfo.in_color_space = JCS_GRAYSCALE;
    }
    else
    {
        sCInfo.in_color_space = JCS_RGB;
    }

    jpeg_set_defaults( &sCInfo );
    
#ifdef JPEG_LIB_MK1
    if( eDT == GDT_UInt16 )
    {
        sCInfo.data_precision = 12;
        sCInfo.bits_in_jsample = 12;
    }
    else
    {
        sCInfo.data_precision = 8;
        sCInfo.bits_in_jsample = 8;
    }
#endif

    sCInfo.write_JFIF_header = FALSE;
    jpeg_set_quality( &sCInfo, nQuality, TRUE );

    if( bProgressive )
        jpeg_simple_progression( &sCInfo );

    jpeg_start_compress( &sCInfo, TRUE );

/* -------------------------------------------------------------------- */
/*      Loop over image, copying image data.                            */
/* -------------------------------------------------------------------- */
    GByte 	*pabyScanline;
    CPLErr      eErr = CE_None;

    pabyScanline = (GByte *) CPLMalloc( nBands * nBlockXSize * 2 );

    double nTotalPixels = (double)nXSize * nYSize;

    int nBlockXSizeToRead = nBlockXSize;
    if (nBlockXSize * nBlockXOff + nBlockXSize > nXSize)
    {
        nBlockXSizeToRead = nXSize - nBlockXSize * nBlockXOff;
    }
    int nBlockYSizeToRead = nBlockYSize;
    if (nBlockYSize * nBlockYOff + nBlockYSize > nYSize)
    {
        nBlockYSizeToRead = nYSize - nBlockYSize * nBlockYOff;
    }

    for( int iLine = 0; iLine < nBlockYSize && eErr == CE_None; iLine++ )
    {
        JSAMPLE      *ppSamples;

        if (iLine < nBlockYSizeToRead)
        {
#ifdef JPEG_LIB_MK1
            eErr = poSrcDS->RasterIO( GF_Read, nBlockXSize * nBlockXOff, iLine + nBlockYSize * nBlockYOff, nBlockXSizeToRead, 1, 
                                    pabyScanline, nBlockXSizeToRead, 1, GDT_UInt16,
                                    nBands, anBandList, 
                                    nBands*2, nBands * nBlockXSize * 2, 2 );
#else
            eErr = poSrcDS->RasterIO( GF_Read, nBlockXSize * nBlockXOff, iLine + nBlockYSize * nBlockYOff, nBlockXSizeToRead, 1, 
                                    pabyScanline, nBlockXSizeToRead, 1, GDT_Byte,
                                    nBands, anBandList, 
                                    nBands, nBands * nBlockXSize, 1 );

            /* Repeat the last pixel till the end of the line */
            /* to minimize discontinuity */
            if (nBlockXSizeToRead < nBlockXSize)
            {
                for (int iBand = 0; iBand < nBands; iBand++)
                {
                    GByte bVal = pabyScanline[nBands * (nBlockXSizeToRead - 1) + iBand];
                    for(int iX = nBlockXSizeToRead; iX < nBlockXSize; iX ++)
                    {
                        pabyScanline[nBands * iX + iBand ] = bVal;
                    }
                }
            }
#endif
        }

        // Should we clip values over 4095 (12bit)? 

        ppSamples = (JSAMPLE *) pabyScanline;

        if( eErr == CE_None )
            jpeg_write_scanlines( &sCInfo, &ppSamples, 1 );

        double nCurPixels = (double)nBlockYOff * nBlockYSize * nXSize +
                            (double)nBlockXOff * nBlockYSize * nBlockXSize + (iLine + 1) * nBlockXSizeToRead;
        if( eErr == CE_None 
            && !pfnProgress( nCurPixels / nTotalPixels, NULL, pProgressData ) )
        {
            eErr = CE_Failure;
            CPLError( CE_Failure, CPLE_UserInterrupt, 
                      "User terminated CreateCopy()" );
        }
    }

/* -------------------------------------------------------------------- */
/*      Cleanup and close.                                              */
/* -------------------------------------------------------------------- */
    CPLFree( pabyScanline );

    if( eErr == CE_None )
        jpeg_finish_compress( &sCInfo );
    jpeg_destroy_compress( &sCInfo );

    return eErr == CE_None;
}
#endif /* def JPEG_SUPPORTED */

/************************************************************************/
/*                          GDALRegister_NITF()                         */
/************************************************************************/

typedef struct
{
    int         nMaxLen;
    const char* pszName;
} NITFFieldDescription;

/* Keep in sync with NITFCreate */
static const NITFFieldDescription asFieldDescription [] =
{
    { 2, "CLEVEL" } ,
    { 10, "OSTAID" } ,
    { 14, "FDT" } ,
    { 80, "FTITLE" } ,
    { 1, "FSCLAS" } ,
    { 2, "FSCLSY" } ,
    { 11, "FSCODE" } ,
    { 2, "FSCTLH" } ,
    { 20, "FSREL" } ,
    { 2, "FSDCTP" } ,
    { 8, "FSDCDT" } ,
    { 4, "FSDCXM" } ,
    { 1, "FSDG" } ,
    { 8, "FSDGDT" } ,
    { 43, "FSCLTX" } ,
    { 1, "FSCATP" } ,
    { 40, "FSCAUT" } ,
    { 1, "FSCRSN" } ,
    { 8, "FSSRDT" } ,
    { 15, "FSCTLN" } ,
    { 5, "FSCOP" } ,
    { 5, "FSCPYS" } ,
    { 24, "ONAME" } ,
    { 18, "OPHONE" } ,
    { 10, "IID1" } ,
    { 14, "IDATIM" } ,
    { 17, "TGTID" } ,
    { 80, "IID2" } ,
    {  1, "ISCLAS" } ,
    {  2, "ISCLSY" } ,
    { 11, "ISCODE" } ,
    {  2, "ISCTLH" } ,
    { 20, "ISREL" } ,
    {  2, "ISDCTP" } ,
    {  8, "ISDCDT" } ,
    {  4, "ISDCXM" } ,
    {  1, "ISDG" } ,
    {  8, "ISDGDT" } ,
    { 43, "ISCLTX" } ,
    {  1, "ISCATP" } ,
    { 40, "ISCAUT" } ,
    {  1, "ISCRSN" } ,
    {  8, "ISSRDT" } ,
    { 15, "ISCTLN" } ,
    { 42, "ISORCE" } ,
    {  8, "ICAT" } ,
    {  2, "ABPP" } ,
    {  1, "PJUST" } ,
};

/* Keep in sync with NITFWriteBLOCKA */
static const char *apszFieldsBLOCKA[] = { 
        "BLOCK_INSTANCE", "0", "2",
        "N_GRAY",         "2", "5",
        "L_LINES",        "7", "5",
        "LAYOVER_ANGLE",  "12", "3",
        "SHADOW_ANGLE",   "15", "3",
        "BLANKS",         "18", "16",
        "FRLC_LOC",       "34", "21",
        "LRLC_LOC",       "55", "21",
        "LRFC_LOC",       "76", "21",
        "FRFC_LOC",       "97", "21",
        NULL,             NULL, NULL };

void GDALRegister_NITF()

{
    GDALDriver	*poDriver;

    if( GDALGetDriverByName( "NITF" ) == NULL )
    {
        unsigned int i;
        CPLString osCreationOptions;

        osCreationOptions =
"<CreationOptionList>"
"   <Option name='IC' type='string-select' default='NC' description='Compression mode. NC=no compression. C3=JPEG compression. C8=JP2 compression through the JP2ECW driver'>"
"       <Value>NC</Value>"
#ifdef JPEG_SUPPORTED
"       <Value>C3</Value>"
#endif
"       <Value>C8</Value>"
"   </Option>"
#ifdef JPEG_SUPPORTED
"   <Option name='QUALITY' type='int' description='JPEG quality 10-100' default='75'/>"
"   <Option name='PROGRESSIVE' type='boolean' description='JPEG progressive mode'/>"
#endif
"   <Option name='TARGET' type='float' description='For JP2 only. Compression Percentage'/>"
"   <Option name='PROFILE' type='string-select' description='For JP2 only.'>"
"       <Value>BASELINE_0</Value>"
"       <Value>BASELINE_1</Value>"
"       <Value>BASELINE_2</Value>"
"       <Value>NPJE</Value>"
"       <Value>EPJE</Value>"
"   </Option>"
"   <Option name='ICORDS' type='string-select' description='To ensure that space will be reserved for geographic corner coordinates in DMS (G), in decimal degrees (D), UTM North (N) or UTM South'>"
"       <Value>G</Value>"
"       <Value>D</Value>"
"       <Value>N</Value>"
"       <Value>S</Value>"
"   </Option>"
"   <Option name='FHDR' type='string-select' description='File version' default='NITF02.10'>"
"       <Value>NITF02.10</Value>"
"       <Value>NSIF01.00</Value>"
"   </Option>"
"   <Option name='IREP' type='string' description='Set to RGB/LUT to reserve space for a color table for each output band. (Only needed for Create() method, not CreateCopy())'/>"
"   <Option name='LUT_SIZE' type='integer' description='Set to control the size of pseudocolor tables for RGB/LUT bands' default='256'/>"
"   <Option name='BLOCKXSIZE' type='int' description='Set the block width'/>"
"   <Option name='BLOCKYSIZE' type='int' description='Set the block height'/>"
"   <Option name='BLOCKSIZE' type='int' description='Set the block with and height. Overridden by BLOCKXSIZE and BLOCKYSIZE'/>";

        for(i=0;i<sizeof(asFieldDescription) / sizeof(asFieldDescription[0]); i++)
        {
            char szFieldDescription[128];
            sprintf(szFieldDescription, "   <Option name='%s' type='string' maxsize='%d'/>",
                    asFieldDescription[i].pszName, asFieldDescription[i].nMaxLen);
            osCreationOptions += szFieldDescription;
        }

        osCreationOptions +=
"   <Option name='TRE' type='string' description='Under the format TRE=tre-name,tre-contents'/>"
"   <Option name='BLOCKA_BLOCK_COUNT' type='int'/>";

        for(i=0; apszFieldsBLOCKA[i] != NULL; i+=3)
        {
            char szFieldDescription[128];
            sprintf(szFieldDescription, "   <Option name='BLOCKA_%s_*' type='string' maxsize='%d'/>",
                    apszFieldsBLOCKA[i], atoi(apszFieldsBLOCKA[i+2]));
            osCreationOptions += szFieldDescription;
        }

        osCreationOptions += "</CreationOptionList>";

        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "NITF" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "National Imagery Transmission Format" );
        
        poDriver->pfnIdentify = NITFDataset::Identify;
        poDriver->pfnOpen = NITFDataset::Open;
        poDriver->pfnCreate = NITFDatasetCreate;
        poDriver->pfnCreateCopy = NITFDataset::NITFCreateCopy;

        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, "frmt_nitf.html" );
        poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "ntf" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONDATATYPES, 
                                   "Byte UInt16 Int16 UInt32 Int32 Float32" );

        poDriver->SetMetadataItem( GDAL_DMD_CREATIONOPTIONLIST, osCreationOptions);
        poDriver->SetMetadataItem( GDAL_DCAP_VIRTUALIO, "YES" );

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}
