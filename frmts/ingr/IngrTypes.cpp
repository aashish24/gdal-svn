/*****************************************************************************
 * $Id$
 *
 * Project:  Intergraph Raster Format support
 * Purpose:  Types support function
 * Author:   Ivan Lucena, ivan.lucena@pmldnet.com
 *
 ******************************************************************************
 * Copyright (c) 2007, Ivan Lucena
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files ( the "Software" ),
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
 *****************************************************************************/

#ifdef DEBUG
#include "stdio.h"
#endif

#include "IngrTypes.h"
#include "JpegHelper.h"

static const INGR_FormatDescription INGR_FormatTable[] = {
    {PackedBinary,            "Packed Binary",               GDT_Byte},
    {ByteInteger,             "Byte Integer",                GDT_Byte},
    {WordIntegers,            "Word Integers",               GDT_Int16},
    {Integers32Bit,           "Integers 32Bit",              GDT_Int32},
    {FloatingPoint32Bit,      "Floating Point 32Bit",        GDT_Float32},
    {FloatingPoint64Bit,      "Floating Point 64Bit",        GDT_Float64},
    {Complex,                 "Complex",                     GDT_CFloat32},
    {DoublePrecisionComplex,  "Double Precision Complex",    GDT_CFloat64},
    {RunLengthEncoded,        "Run Length Encoded Bitonal",  GDT_Byte},
    {RunLengthEncodedC,       "Run Length Encoded Color",    GDT_Byte},
    {FigureOfMerit,           "Figure of Merit",             GDT_Byte},
    {DTMFlags,                "DTMFlags",                    GDT_Byte},
    {RLEVariableValuesWithZS, "RLE Variable Values With ZS", GDT_Byte},
    {RLEBinaryValues,         "RLE Binary Values",           GDT_Byte},
    {RLEVariableValues,       "RLE Variable Values",         GDT_Byte},
    {RLEVariableValuesWithZ,  "RLE Variable Values With Z",  GDT_Byte},
    {RLEVariableValuesC,      "RLE Variable Values C",       GDT_Byte},
    {RLEVariableValuesN,      "RLE Variable Values N",       GDT_Byte},
    {QuadTreeEncoded,         "Quad Tree Encoded",           GDT_Byte},
    {CCITTGroup4,             "CCITT Group 4",               GDT_Byte},
    {RunLengthEncodedRGB,     "Run Length Encoded RGB",      GDT_Byte},
    {VariableRunLength,       "Variable Run Length",         GDT_Byte},
    {AdaptiveRGB,             "Adaptive RGB",                GDT_Byte},
    {Uncompressed24bit,       "Uncompressed 24bit",          GDT_Byte},
    {AdaptiveGrayScale,       "Adaptive Gray Scale",         GDT_Byte},
    {JPEGGRAY,                "JPEG GRAY",                   GDT_Byte},
    {JPEGRGB,                 "JPEG RGB",                    GDT_Byte},
    {JPEGCYMK,                "JPEG CYMK",                   GDT_Byte},
    {TiledRasterData,         "Tiled Raste Data",            GDT_Byte},
    {NotUsedReserved,         "Not Used( Reserved )",        GDT_Byte},
    {ContinuousTone,          "Continuous Tone",             GDT_Byte},
    {LineArt,                 "LineArt",                     GDT_Byte}
};

static const char *IngrOrientation[] = {
    "Upper Left Vertical",
    "Upper Right Vertical",
    "Lower Left Vertical",
    "Lower Right Vertical",
    "Upper Left Horizontal",
    "Upper Right Horizontal",
    "Lower Left Horizontal",
    "Lower Right Horizontal"};

static const GByte BitReverseTable[256] =
{
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
};

// -----------------------------------------------------------------------------
//                                              Scanline Orientation Flip Matrix
// -----------------------------------------------------------------------------

static const double INGR_URV_Flip[16] =
    {
        1.0,  0.0,  0.0,  0.0,
        0.0, -1.0,  0.0,  0.0,
        0.0,  0.0,  1.0,  0.0,
        0.0,  0.0,  0.0,  1.0
    };
static const double INGR_LLV_Flip[16] =
    {
       -1.0,  0.0,  0.0,  0.0,
        0.0,  1.0,  0.0,  0.0,
        0.0,  0.0,  1.0,  0.0,
        0.0,  0.0,  0.0,  1.0
    };
static const double INGR_LRV_Flip[16] =
    {
       -1.0,  0.0,  0.0,  0.0,
        0.0, -1.0,  0.0,  0.0,
        0.0,  0.0,  1.0,  0.0,
        0.0,  0.0,  0.0,  1.0
    };
static const double INGR_ULH_Flip[16] =
    {
        1.0,  0.0,  0.0,  0.0,
        0.0,  1.0,  0.0,  0.0,
        0.0,  0.0, -1.0,  0.0,
        0.0,  0.0,  0.0,  1.0
    };
static const double INGR_URH_Flip[16] =
    {
        1.0,  0.0,  0.0,  0.0,
        0.0, -1.0,  0.0,  0.0,
        0.0,  0.0, -1.0,  0.0,
        0.0,  0.0,  0.0,  1.0
    };
static const double INGR_LLH_Flip[16] =
    {
       -1.0,  0.0,  0.0,  0.0,
        0.0,  1.0,  0.0,  0.0,
        0.0,  0.0, -1.0,  0.0,
        0.0,  0.0,  0.0,  1.0
    };
static const double INGR_LRH_Flip[16] =
    {
       -1.0,  0.0,  0.0,  0.0,
        0.0, -1.0,  0.0,  0.0,
        0.0,  0.0, -1.0,  0.0,
        0.0,  0.0,  0.0,  1.0
    };

void INGR_MultiplyMatrix( double *padfA, double *padfB, const double *padfC )
{
    int i;
    int j;

    for( i = 0; i < 4; i++ )
    {
        for( j = 0; j < 4; j++ )
        {
            padfA[(i * 4) + j] = 
                padfB[(i * 4) + 0] * padfC[(0 * 4) + j] +
                padfB[(i * 4) + 1] * padfC[(1 * 4) + j] +
                padfB[(i * 4) + 2] * padfC[(2 * 4) + j] +
                padfB[(i * 4) + 3] * padfC[(3 * 4) + j];
        }
    }
}

// -----------------------------------------------------------------------------
//                                                            INGR_GetDataType()
// -----------------------------------------------------------------------------

const GDALDataType CPL_STDCALL INGR_GetDataType( uint16 eCode )
{
    unsigned int i;

    for( i = 0; i < FORMAT_TAB_COUNT; i++ )
    {
		if( eCode == INGR_FormatTable[i].eFormatCode )
        {
            return INGR_FormatTable[i].eDataType;
        }
    }

    return GDT_Unknown;
}

// -----------------------------------------------------------------------------
//                                                          INGR_GetFormatName()
// -----------------------------------------------------------------------------

const char * CPL_STDCALL INGR_GetFormatName( uint16 eCode )
{
    unsigned int i;

    for( i = 0; i < FORMAT_TAB_COUNT; i++ )
    {
        if( eCode == INGR_FormatTable[i].eFormatCode )
        {
            return INGR_FormatTable[i].pszName;
        }
    }

    return "Not Identified";
}

// -----------------------------------------------------------------------------
//                                                         INGR_GetOrientation()
// -----------------------------------------------------------------------------

const char * CPL_STDCALL INGR_GetOrientation( uint8 nIndex )
{
    return IngrOrientation[nIndex];
}

// -----------------------------------------------------------------------------
//                                                              INGR_GetFormat()
// -----------------------------------------------------------------------------

const INGR_Format CPL_STDCALL INGR_GetFormat( GDALDataType eType, 
                                              const char *pszCompression )
{
    if( EQUAL( pszCompression, "None" ) ||
        EQUAL( pszCompression, "" ) )
    {
        switch ( eType )
        {
        case GDT_Byte:      return ByteInteger;
        case GDT_Int16:     return WordIntegers;
        case GDT_UInt16:    return WordIntegers;
        case GDT_Int32:     return Integers32Bit;
        case GDT_UInt32:    return Integers32Bit;
        case GDT_Float32:   return FloatingPoint32Bit;
        case GDT_Float64:   return FloatingPoint64Bit;
        default:            return ByteInteger;
        }
    }

    unsigned int i;

    for( i = 0; i < FORMAT_TAB_COUNT; i++ )
    {
        if( EQUAL( pszCompression, INGR_FormatTable[i].pszName ) )
        {
            return (INGR_Format) INGR_FormatTable[i].eFormatCode;
        }
    }

    return ByteInteger;
}

// -----------------------------------------------------------------------------
//                                                         INGR_GetTransMatrix()
// -----------------------------------------------------------------------------

void CPL_STDCALL INGR_GetTransMatrix( INGR_HeaderOne *pHeaderOne, 
                                      double *padfGeoTransform )
{
    // -------------------------------------------------------------
    // Check for empty transformation matrix
    // -------------------------------------------------------------

    if( pHeaderOne->TransformationMatrix[0]  == 0.0 &&
        pHeaderOne->TransformationMatrix[2]  == 0.0 &&
        pHeaderOne->TransformationMatrix[3]  == 0.0 &&
        pHeaderOne->TransformationMatrix[4]  == 0.0 &&
        pHeaderOne->TransformationMatrix[5]  == 0.0 &&
        pHeaderOne->TransformationMatrix[7]  == 0.0 )
    {
        padfGeoTransform[0] = 0.0;
        padfGeoTransform[1] = 1.0;
        padfGeoTransform[2] = 0.0; 
        padfGeoTransform[3] = pHeaderOne->PixelsPerLine;
        padfGeoTransform[4] = 0.0;
        padfGeoTransform[5] = -1.0;
        return;
    }

    // -------------------------------------------------------------
    // Calculate Concatened Tranformation Matrix based on Orientation
    // -------------------------------------------------------------

    double adfConcat[16];
   
    switch( (INGR_Orientation ) pHeaderOne->ScanlineOrientation )
    {
        case UpperLeftVertical:
            memcpy( adfConcat, pHeaderOne->TransformationMatrix, 16 ); //?? sizeof(double) ??
            break;
        case UpperRightVertical:
            INGR_MultiplyMatrix( adfConcat, pHeaderOne->TransformationMatrix, INGR_URV_Flip ); 
            break;
        case LowerLeftVertical:
            INGR_MultiplyMatrix( adfConcat, pHeaderOne->TransformationMatrix, INGR_LLV_Flip ); 
            break;
        case LowerRightVertical:
            INGR_MultiplyMatrix( adfConcat, pHeaderOne->TransformationMatrix, INGR_LRV_Flip ); 
            break;
        case UpperLeftHorizontal:
            INGR_MultiplyMatrix( adfConcat, pHeaderOne->TransformationMatrix, INGR_ULH_Flip ); 
            break;
        case UpperRightHorizontal:
            INGR_MultiplyMatrix( adfConcat, pHeaderOne->TransformationMatrix, INGR_URH_Flip ); 
            break;
        case LowerLeftHorizontal:
            INGR_MultiplyMatrix( adfConcat, pHeaderOne->TransformationMatrix, INGR_LLH_Flip ); 
            break;
        case LowerRightHorizontal:
            INGR_MultiplyMatrix( adfConcat, pHeaderOne->TransformationMatrix, INGR_LRH_Flip ); 
            break;
    }

    // -------------------------------------------------------------
    // Convert to GDAL GeoTransformation Matrix
    // -------------------------------------------------------------

    padfGeoTransform[0] = adfConcat[3] - adfConcat[0] / 2;
    padfGeoTransform[1] = adfConcat[0];
    padfGeoTransform[2] = adfConcat[1];
    padfGeoTransform[3] = adfConcat[7] + adfConcat[5] / 2;
    padfGeoTransform[4] = adfConcat[4];
    padfGeoTransform[5] = - adfConcat[5];
}

// -----------------------------------------------------------------------------
//                                                         INGR_SetTransMatrix()
// -----------------------------------------------------------------------------

void CPL_STDCALL INGR_SetTransMatrix( real64 *padfMatrix, double *padfGeoTransform )
{
    unsigned int i;

    for( i = 0; i < 15; i++ )
    {
        padfMatrix[i] = 0.0;
    }

    padfMatrix[10] = 1.0;
    padfMatrix[15] = 1.0;

    padfMatrix[3] = padfGeoTransform[0] + padfGeoTransform[1] / 2;
    padfMatrix[0] = padfGeoTransform[1];
    padfMatrix[1] = padfGeoTransform[2];
    padfMatrix[7] = padfGeoTransform[3] + padfGeoTransform[5] / 2;
    padfMatrix[4] = padfGeoTransform[4];
    padfMatrix[5] = - padfGeoTransform[5];
}

// -----------------------------------------------------------------------------
//                                                          INGR_SetIGDSColors()
// -----------------------------------------------------------------------------

uint32 CPL_STDCALL INGR_SetIGDSColors( GDALColorTable *poColorTable,
                                      INGR_ColorTable256 *pColorTableIGDS )
{
    GDALColorEntry oEntry;
    int i;

    for( i = 0; i < poColorTable->GetColorEntryCount(); i++ )
    {
        poColorTable->GetColorEntryAsRGB( i, &oEntry );
        pColorTableIGDS->Entry[i].v_red   = (uint8) oEntry.c1;
        pColorTableIGDS->Entry[i].v_green = (uint8) oEntry.c2;
        pColorTableIGDS->Entry[i].v_blue  = (uint8) oEntry.c3;
    }

    return i;
}

// -----------------------------------------------------------------------------
//                                                       INGR_GetTileDirectory()
// -----------------------------------------------------------------------------

uint32 CPL_STDCALL INGR_GetTileDirectory( FILE *fp,
                                          uint32 nOffset,
                                          int nBandXSize,
                                          int nBandYSize,
                                          INGR_TileHeader *pTileDir,
                                          INGR_TileItem **pahTiles)
{
    if( fp == NULL ||
        nBandXSize < 1 ||
        nBandYSize < 1 ||
        pTileDir == NULL )
    {
        return 0;
    }

    // -------------------------------------------------------------
    // Read it from the begging of the data segment
    // -------------------------------------------------------------

    GByte abyBuf[SIZEOF_TDIR];

    if( ( VSIFSeekL( fp, nOffset, SEEK_SET ) == -1 ) ||
        ( VSIFReadL( abyBuf, 1, SIZEOF_TDIR, fp ) == 0 ) )
    {
        CPLDebug("INGR", "Error reading tiles header");
        return 0;
    }

    INGR_TileHeaderDiskToMem( pTileDir, abyBuf );

    // ----------------------------------------------------------------
    // Calculate the number of tiles
    // ----------------------------------------------------------------

    int nTilesPerCol = (int) ceil( (float) nBandXSize / pTileDir->TileSize );
    int nTilesPerRow = (int) ceil( (float) nBandYSize / pTileDir->TileSize );

    uint32 nTiles = nTilesPerCol * nTilesPerRow;

    // ----------------------------------------------------------------
    // Load the tile table (first tile s already read)
    // ----------------------------------------------------------------

    *pahTiles  = (INGR_TileItem*) CPLCalloc( nTiles, SIZEOF_TILE );
    GByte *pabyBuf  = (GByte*) CPLCalloc( ( nTiles - 1 ), SIZEOF_TILE );

    (*pahTiles)[0].Start      = pTileDir->First.Start;
    (*pahTiles)[0].Allocated  = pTileDir->First.Allocated;
    (*pahTiles)[0].Used       = pTileDir->First.Used;

    if( nTiles > 1 &&
      ( VSIFReadL( pabyBuf, ( nTiles - 1 ), SIZEOF_TILE, fp ) == 0 ) )
    {
        CPLDebug("INGR", "Error reading tiles table");
        return 1;
    }

    unsigned int i;

    for( i = 1; i < nTiles; i++ )
    {
        INGR_TileItemDiskToMem( &((*pahTiles)[i]), 
                                &pabyBuf[ (i - 1) * SIZEOF_TILE] );
    }

    CPLFree( pabyBuf );
    return nTiles;
}

// -----------------------------------------------------------------------------
//                                                          INGR_GetIGDSColors()
// -----------------------------------------------------------------------------

void CPL_STDCALL INGR_GetIGDSColors( FILE *fp,
                                     uint32 nOffset,
                                     uint32 nEntries,
                                     GDALColorTable *poColorTable )
{
    if( fp == NULL ||
        nEntries == 0 ||
        poColorTable == NULL )
    {
        return;
    }

    // -------------------------------------------------------------
    // Read it from the middle of the second block
    // -------------------------------------------------------------

    uint32 nStart = nOffset + SIZEOF_HDR1 + SIZEOF_HDR2_A;

    INGR_ColorTable256 hIGDSColors;

    GByte *pabyBuf = (GByte*) CPLCalloc( nEntries, SIZEOF_IGDS );

    if( ( VSIFSeekL( fp, nStart, SEEK_SET ) == -1 ) ||
        ( VSIFReadL( pabyBuf, nEntries, SIZEOF_IGDS, fp ) == 0 ) )
    {
        return;
    }

    unsigned int i = 0;
    unsigned int n = 0;

    for( i = 0; i < nEntries; i++ )
    {
        BUF2STRC( pabyBuf, n, hIGDSColors.Entry[i].v_red );
        BUF2STRC( pabyBuf, n, hIGDSColors.Entry[i].v_green );
        BUF2STRC( pabyBuf, n, hIGDSColors.Entry[i].v_blue );
    }

    CPLFree( pabyBuf );

    // -------------------------------------------------------------
    // Read it to poColorTable
    // -------------------------------------------------------------

    GDALColorEntry oEntry;

    oEntry.c4 = 255;

    for( i = 0; i < nEntries; i++ )
    {
        oEntry.c1 = hIGDSColors.Entry[i].v_red;
        oEntry.c2 = hIGDSColors.Entry[i].v_green;
        oEntry.c3 = hIGDSColors.Entry[i].v_blue;
        poColorTable->SetColorEntry( i, &oEntry );
    }
}

// -----------------------------------------------------------------------------
//                                                       INGR_SetEnvironColors()
// -----------------------------------------------------------------------------

uint32 CPL_STDCALL INGR_SetEnvironColors( GDALColorTable *poColorTable,
                                          INGR_ColorTableVar *pEnvironTable )
{
    GDALColorEntry oEntry;
    real32 fNormFactor = 0xfff / 255;
    int i;

    for( i = 0; i < poColorTable->GetColorEntryCount(); i++ )
    {
        poColorTable->GetColorEntryAsRGB( i, &oEntry );
        pEnvironTable->Entry[i].v_slot  = (uint16) i;
        pEnvironTable->Entry[i].v_red   = (uint16) ( oEntry.c1 * fNormFactor );
        pEnvironTable->Entry[i].v_green = (uint16) ( oEntry.c2 * fNormFactor );
        pEnvironTable->Entry[i].v_blue  = (uint16) ( oEntry.c3 * fNormFactor );
    }

    return i;
}

// -----------------------------------------------------------------------------
//                                                      INGR_GetEnvironVColors()
// -----------------------------------------------------------------------------

void CPL_STDCALL INGR_GetEnvironVColors( FILE *fp,
                             uint32 nOffset,
                             uint32 nEntries,
                             GDALColorTable *poColorTable )
{
    if( fp == NULL ||
        nEntries == 0 ||
        poColorTable == NULL )
    {
        return;
    }

    // -------------------------------------------------------------
    // Read it from the third block
    // -------------------------------------------------------------

    uint32 nStart = nOffset + SIZEOF_HDR1 + SIZEOF_HDR2;

    INGR_ColorTableVar hVLTColors;

    hVLTColors.Entry = (vlt_slot*) CPLCalloc( nEntries, SIZEOF_VLTS );

    GByte *pabyBuf = (GByte*) CPLCalloc( nEntries, SIZEOF_VLTS );

    if( ( VSIFSeekL( fp, nStart, SEEK_SET ) == -1 ) ||
        ( VSIFReadL( pabyBuf, nEntries, SIZEOF_VLTS, fp ) == 0 ) )
    {
        CPLFree( hVLTColors.Entry );
        return;
    }

    unsigned int i = 0;
    unsigned int n = 0;

    for( i = 0; i < nEntries; i++ )
    {
        BUF2STRC( pabyBuf, n, hVLTColors.Entry[i].v_slot );
        BUF2STRC( pabyBuf, n, hVLTColors.Entry[i].v_red );
        BUF2STRC( pabyBuf, n, hVLTColors.Entry[i].v_green );
        BUF2STRC( pabyBuf, n, hVLTColors.Entry[i].v_blue );
    }

    // -------------------------------------------------------------
    // Sort records by crescent order of "slot" 
    // -------------------------------------------------------------

    vlt_slot hSwapSlot;
    bool bContinue = true; // Inproved bubble sort.
    unsigned j = 0;        // This is the best sort techique when you
                           // suspect that the data is already sorted

#if defined(CPL_MSB)
    for (i = 0; i < nEntries; i++)
    {
        CPL_LSBPTR16(&hVLTColors.Entry[i].v_slot);
        CPL_LSBPTR16(&hVLTColors.Entry[i].v_red);
        CPL_LSBPTR16(&hVLTColors.Entry[i].v_green);
        CPL_LSBPTR16(&hVLTColors.Entry[i].v_blue);
    }
#endif

    for( j = 1; j < nEntries && bContinue; j++ )
    {
        bContinue = false;
        for( i = 0; i < nEntries - j; i++ )
        {
            if( hVLTColors.Entry[i].v_slot > 
                hVLTColors.Entry[i + 1].v_slot )
            {
                hSwapSlot = hVLTColors.Entry[i];
                hVLTColors.Entry[i]   = hVLTColors.Entry[i+1];
                hVLTColors.Entry[i+1] = hSwapSlot;
                bContinue = true;
            }
        }
    }

    // -------------------------------------------------------------
    // Get Maximum Intensity and Index Values
    // -------------------------------------------------------------

    uint32 nMaxIndex    = 0;
    real32 fMaxRed      = 0.0;
    real32 fMaxGreen    = 0.0;
    real32 fMaxBlues    = 0.0;

    for( i = 0; i < nEntries; i++ )
    {
        nMaxIndex = MAX( nMaxIndex, hVLTColors.Entry[i].v_slot );
        fMaxRed   = MAX( fMaxRed  , hVLTColors.Entry[i].v_red );
        fMaxGreen = MAX( fMaxGreen, hVLTColors.Entry[i].v_green );
        fMaxBlues = MAX( fMaxBlues, hVLTColors.Entry[i].v_blue );
    }

    // -------------------------------------------------------------
    // Calculate Normalization Factor
    // -------------------------------------------------------------

    real32 fNormFactor  = 0.0;

    fNormFactor  = ( fMaxRed > fMaxGreen ? fMaxRed : fMaxGreen );
    fNormFactor  = ( fNormFactor > fMaxBlues ? fNormFactor : fMaxBlues );
    fNormFactor  = 255 / fNormFactor;

    // -------------------------------------------------------------
    // Loads GDAL Color Table ( filling the wholes )
    // -------------------------------------------------------------

    GDALColorEntry oEntry;
    GDALColorEntry oNullEntry;

    oNullEntry.c1 = (short) 0;
    oNullEntry.c2 = (short) 0;
    oNullEntry.c3 = (short) 0;
    oNullEntry.c4 = (short) 255;

    for( i = 0, j = 0; i <= nMaxIndex; i++ )
    {
        if( hVLTColors.Entry[i].v_slot == i )
        {
            oEntry.c1 = (short) ( hVLTColors.Entry[i].v_red   * fNormFactor );
            oEntry.c2 = (short) ( hVLTColors.Entry[i].v_green * fNormFactor );
            oEntry.c3 = (short) ( hVLTColors.Entry[i].v_blue  * fNormFactor );
            oEntry.c4 = (short) 255;
            poColorTable->SetColorEntry( i, &oEntry );
            j++;
        }
        else
        {
            poColorTable->SetColorEntry( i, &oNullEntry );
        }
    }

    CPLFree( hVLTColors.Entry );
}

// -----------------------------------------------------------------------------
//                                                                  SetMiniMax()
// -----------------------------------------------------------------------------

INGR_MinMax CPL_STDCALL INGR_SetMinMax( GDALDataType eType, double dValue )
{
    INGR_MinMax uResult;

    switch ( eType )
    {
    case GDT_Byte:
        uResult.AsUint8   = (uint8) dValue;
        break;
    case GDT_Int16:
        uResult.AsUint16  = (int16) dValue;
        break;
    case GDT_UInt16:
        uResult.AsUint16  = (uint16) dValue;
        break;
    case GDT_Int32:
        uResult.AsUint32  = (int32) dValue;
        break;
    case GDT_UInt32:
        uResult.AsUint32  = (uint32) dValue;
        break;
    case GDT_Float32:
        uResult.AsReal32  = (real32) dValue;
        break;
    case GDT_Float64:
        uResult.AsReal64  = (real64) dValue;
    default:
        uResult.AsUint8   = (uint8) 0;
    }

    return uResult;
}

// -----------------------------------------------------------------------------
//                                                              INGR_GetMinMax()
// -----------------------------------------------------------------------------

double CPL_STDCALL INGR_GetMinMax( GDALDataType eType, INGR_MinMax hValue )
{
    switch ( eType )
    {
    case GDT_Byte:    return (double) hValue.AsUint8;
    case GDT_Int16:   return (double) hValue.AsUint16;
    case GDT_UInt16:  return (double) hValue.AsUint16;
    case GDT_Int32:   return (double) hValue.AsUint32;
    case GDT_UInt32:  return (double) hValue.AsUint32;
    case GDT_Float32: return (double) hValue.AsReal32;
    case GDT_Float64: return (double) hValue.AsReal64;
    default:          return (double) 0.0;
    }
}

// -----------------------------------------------------------------------------
//                                                       INGR_GetDataBlockSize()
// -----------------------------------------------------------------------------

uint32 CPL_STDCALL INGR_GetDataBlockSize( const char *pszFilename,
                                          uint32 nBandOffset,
                                          uint32 nDataOffset )
{
    if( nBandOffset == 0 )
    {
        // -------------------------------------------------------------
        // Until the end of the file
        // -------------------------------------------------------------

        VSIStatBufL  sStat;
        VSIStatL( pszFilename, &sStat );
        return sStat.st_size - nDataOffset;
    }
    else
    {
        // -------------------------------------------------------------
        // Until the end of the band
        // -------------------------------------------------------------

        return nBandOffset - nDataOffset;
    }
}

// -----------------------------------------------------------------------------
//                                                      INGR_CreateVirtualFile()
// -----------------------------------------------------------------------------

INGR_VirtualFile CPL_STDCALL INGR_CreateVirtualFile( const char *pszFilename,
                                                     INGR_Format eFormat,
                                                     int nXSize, 
                                                     int nYSize,
                                                     int nTileSize,
                                                     int nQuality,
                                                     GByte *pabyBuffer,
                                                     int nBufferSize,
                                                     int nBand )
{
    INGR_VirtualFile hVirtual;

    hVirtual.pszFileName = CPLSPrintf( "/vsimem/%s.virtual",
        CPLGetBasename( pszFilename ) );

    int nJPGComponents = 1;

    switch( eFormat )
    {
    case JPEGRGB: 
        nJPGComponents = 3;
    case JPEGGRAY:
        {
            GByte *pabyHeader = (GByte*) CPLCalloc( 1, 2048 );
            int nHeaderSize   = JPGHLP_HeaderMaker( pabyHeader,
                                                    nTileSize,
                                                    nTileSize,
                                                    nJPGComponents,
                                                    0,
                                                    nQuality );
            FILE *fp = VSIFOpenL( hVirtual.pszFileName, "w+" );
            VSIFWriteL( pabyHeader, 1, nHeaderSize, fp );
            VSIFWriteL( pabyBuffer, 1, nBufferSize, fp );
            VSIFCloseL( fp );
            CPLFree( pabyHeader );
            break;
        }
    case CCITTGroup4:
        {
            REVERSEBITSBUFFER( pabyBuffer, nBufferSize );
            TIFF *hTIFF = VSI_TIFFOpen( hVirtual.pszFileName, "w+" );
            TIFFSetField( hTIFF, TIFFTAG_IMAGEWIDTH,      nXSize );
            TIFFSetField( hTIFF, TIFFTAG_IMAGELENGTH,     nYSize );
            TIFFSetField( hTIFF, TIFFTAG_BITSPERSAMPLE,   1 );
            TIFFSetField( hTIFF, TIFFTAG_SAMPLEFORMAT,    SAMPLEFORMAT_UINT );
            TIFFSetField( hTIFF, TIFFTAG_PLANARCONFIG,    PLANARCONFIG_CONTIG );
            TIFFSetField( hTIFF, TIFFTAG_FILLORDER,       FILLORDER_MSB2LSB );
            TIFFSetField( hTIFF, TIFFTAG_ROWSPERSTRIP,    -1 );
            TIFFSetField( hTIFF, TIFFTAG_SAMPLESPERPIXEL, 1 );
            TIFFSetField( hTIFF, TIFFTAG_PHOTOMETRIC,     PHOTOMETRIC_MINISWHITE );
            TIFFSetField( hTIFF, TIFFTAG_COMPRESSION,     COMPRESSION_CCITTFAX4 );
            TIFFWriteRawStrip( hTIFF, 0, pabyBuffer, nBufferSize );
            TIFFWriteDirectory( hTIFF );
            TIFFClose( hTIFF );
            break;
        }
    default:
        return hVirtual;
    }

    hVirtual.poDS   = (GDALDataset*) GDALOpen( hVirtual.pszFileName, GA_ReadOnly );

    if( hVirtual.poDS )
    {
        hVirtual.poBand = (GDALRasterBand*) GDALGetRasterBand( hVirtual.poDS, nBand );
    }

    return hVirtual;
}

// -----------------------------------------------------------------------------
//                                                            INGR_ReleaseVirtual()
// -----------------------------------------------------------------------------

void CPL_STDCALL INGR_ReleaseVirtual( INGR_VirtualFile *poTiffMem )
{
    delete poTiffMem->poDS;
    VSIUnlink( poTiffMem->pszFileName );
}

// -----------------------------------------------------------------------------
//                                                            INGR_ReleaseVirtual()
// -----------------------------------------------------------------------------

int CPL_STDCALL INGR_ReadJpegQuality( FILE *fp, uint32 nAppDataOfseet,
                                      uint32 nSeekLimit )
{
    if( nAppDataOfseet == 0  )
    {
        return INGR_JPEGQDEFAULT;
    }

    INGR_JPEGAppData hJpegData;
    uint32 nNext = nAppDataOfseet;

    GByte abyBuf[SIZEOF_JPGAD];

    do
    {
        if( ( VSIFSeekL( fp, nNext, SEEK_SET ) == -1 ) ||
            ( VSIFReadL( abyBuf, 1, SIZEOF_JPGAD, fp ) == 0 ) )
        {
            return INGR_JPEGQDEFAULT;
        }

        INGR_JPEGAppDataDiskToMem(&hJpegData, abyBuf);

        nNext += hJpegData.RemainingLength;

        if( nNext > ( nSeekLimit - SIZEOF_JPGAD ) )
        {
            return INGR_JPEGQDEFAULT;
        }
    } 
    while( ! ( hJpegData.ApplicationType == 2 &&
        hJpegData.SubTypeCode == 12 ) );

    return hJpegData.JpegQuality;
}

// -----------------------------------------------------------------------------
//                                                        INGR_Decode()
//
//	Decode the various RLE compression options.
// -----------------------------------------------------------------------------

int CPL_STDCALL 
INGR_Decode( INGR_Format eFormat, GByte *pabySrcData, GByte *pabyDstData,
             uint32 nSrcBytes, uint32 nBlockSize, uint32 *pnBytesConsumed )

{
    switch( eFormat )
    {
      case RunLengthEncoded:
        return INGR_DecodeRunLengthBitonal( pabySrcData,  pabyDstData,
                                            nSrcBytes, nBlockSize,
                                            pnBytesConsumed );

      case RunLengthEncodedC:
        return INGR_DecodeRunLengthPaletted( pabySrcData,  pabyDstData,
                                             nSrcBytes, nBlockSize, 
                                             pnBytesConsumed );

      default:
        return INGR_DecodeRunLength( pabySrcData,  pabyDstData,
                                     nSrcBytes, nBlockSize,
                                     pnBytesConsumed );
    }
}

// -----------------------------------------------------------------------------
//                                                        INGR_DecodeRunLength()
// -----------------------------------------------------------------------------

int CPL_STDCALL INGR_DecodeRunLength( GByte *pabySrcData, GByte *pabyDstData,
                                      uint32 nSrcBytes, uint32 nBlockSize,
                                      uint32 *pnBytesConsumed )
{
    signed char cAtomHead;

    unsigned int nRun;
    unsigned int i; 
    unsigned int iInput;
    unsigned int iOutput;

    iInput = 0;
    iOutput = 0;

    do
    {
        cAtomHead = (char) pabySrcData[iInput++];

        if( cAtomHead > 0 )
        {
            nRun = cAtomHead;

            for( i = 0; i < nRun && iOutput < nBlockSize; i++ )
            {
                pabyDstData[iOutput++] = pabySrcData[iInput++];
            }
        }
        else if( cAtomHead < 0 )
        {
            nRun = abs( cAtomHead );

            for( i = 0; i < nRun && iOutput < nBlockSize; i++ )
            {
                pabyDstData[iOutput++] = pabySrcData[iInput];
            }
            iInput++;
        }
    }
    while( ( iInput < nSrcBytes ) && ( iOutput < nBlockSize ) );

    if( pnBytesConsumed != NULL )
        *pnBytesConsumed = iInput * 2;

    return iOutput;
}

// -----------------------------------------------------------------------------
//                                                INGR_DecodeRunLengthPaletted()
// -----------------------------------------------------------------------------

int CPL_STDCALL 
INGR_DecodeRunLengthPaletted( GByte *pabySrcData, GByte *pabyDstData,
                              uint32 nSrcBytes, uint32 nBlockSize, 
                              uint32 *pnBytesConsumed )
{
    unsigned short nColor;
    unsigned short nCount;

    unsigned int i; 
    unsigned int iInput;
    unsigned int iOutput;

    unsigned short *pauiSrc = (unsigned short *) pabySrcData;
    unsigned int nSrcShorts = nSrcBytes / 2;
    unsigned short nSize;
    unsigned short nLine;
    unsigned short nRun;

    iInput = 0;
    iOutput = 0;

    do
    {
        nColor = CPL_LSBWORD16(pauiSrc[ iInput ]);
        iInput++;

        if( nColor == 0x5900 )
        {
            nSize = CPL_LSBWORD16(pauiSrc[ iInput ]);
            iInput++;
            nLine = CPL_LSBWORD16(pauiSrc[ iInput ]);
            iInput++;
            nRun  = CPL_LSBWORD16(pauiSrc[ iInput ]);
            iInput++;
            continue;
        }

        nCount = CPL_LSBWORD16(pauiSrc[iInput]);
        iInput++;

        for( i = 0; i < nCount && iOutput < nBlockSize; i++ )
        {
            pabyDstData[iOutput++] = (unsigned char) nColor;
        }
    }
    while( ( iInput < nSrcShorts ) && ( iOutput < nBlockSize) );

    if( pnBytesConsumed != NULL )
        *pnBytesConsumed = iInput * 2;

    return iOutput;
}

// -----------------------------------------------------------------------------
//                                                 INGR_DecodeRunLengthBitonal()
// -----------------------------------------------------------------------------

int CPL_STDCALL 
INGR_DecodeRunLengthBitonal( GByte *pabySrcData, GByte *pabyDstData,
                             uint32 nSrcBytes, uint32 nBlockSize,
                             uint32 *pnBytesConsumed )
{
    unsigned short i; 
    unsigned int   iInput = 0;
    unsigned int   iOutput = 0;
    unsigned short *pauiSrc = (unsigned short *) pabySrcData;
    unsigned int   nSrcShorts = nSrcBytes / 2;
    unsigned short nRun;
    unsigned char  nValue = 0;

    if( CPL_LSBWORD16(pauiSrc[0]) != 0x5900 )
        nValue = 1;

    do
    {
        nRun = CPL_LSBWORD16(pauiSrc[ iInput ]);
        iInput++;
        
        if( nRun == 0x5900 )
        {
            iInput++; // line id
            iInput++; // line data size
            continue;
        }
        
        for( i = 0; i < nRun && iOutput < nBlockSize; i++ )
        {
            pabyDstData[ iOutput++ ] = nValue;
        }
        
        nValue = ( nValue == 1 ? 0 : 1 );
    }
    while( ( iInput < nSrcShorts ) && ( iOutput < nBlockSize ) );

    if( pnBytesConsumed != NULL )
        *pnBytesConsumed = iInput * 2;

    return iOutput;
}

void CPL_STDCALL INGR_HeaderOneDiskToMem(INGR_HeaderOne* pHeaderOne, GByte *pabyBuf)
{
    unsigned int n = 0;

    BUF2STRC( pabyBuf, n, pHeaderOne->HeaderType );
    BUF2STRC( pabyBuf, n, pHeaderOne->WordsToFollow );
    BUF2STRC( pabyBuf, n, pHeaderOne->DataTypeCode );
    BUF2STRC( pabyBuf, n, pHeaderOne->ApplicationType );
    BUF2STRC( pabyBuf, n, pHeaderOne->XViewOrigin );
    BUF2STRC( pabyBuf, n, pHeaderOne->YViewOrigin );
    BUF2STRC( pabyBuf, n, pHeaderOne->ZViewOrigin );
    BUF2STRC( pabyBuf, n, pHeaderOne->XViewExtent );
    BUF2STRC( pabyBuf, n, pHeaderOne->YViewExtent );
    BUF2STRC( pabyBuf, n, pHeaderOne->ZViewExtent );
    BUF2STRC( pabyBuf, n, pHeaderOne->TransformationMatrix );
    BUF2STRC( pabyBuf, n, pHeaderOne->PixelsPerLine );
    BUF2STRC( pabyBuf, n, pHeaderOne->NumberOfLines );
    BUF2STRC( pabyBuf, n, pHeaderOne->DeviceResolution );
    BUF2STRC( pabyBuf, n, pHeaderOne->ScanlineOrientation );
    BUF2STRC( pabyBuf, n, pHeaderOne->ScannableFlag );
    BUF2STRC( pabyBuf, n, pHeaderOne->RotationAngle );
    BUF2STRC( pabyBuf, n, pHeaderOne->SkewAngle );
    BUF2STRC( pabyBuf, n, pHeaderOne->DataTypeModifier );
    BUF2STRC( pabyBuf, n, pHeaderOne->DesignFileName );
    BUF2STRC( pabyBuf, n, pHeaderOne->DataBaseFileName );
    BUF2STRC( pabyBuf, n, pHeaderOne->ParentGridFileName );
    BUF2STRC( pabyBuf, n, pHeaderOne->FileDescription );
    BUF2STRC( pabyBuf, n, pHeaderOne->Minimum );
    BUF2STRC( pabyBuf, n, pHeaderOne->Maximum );
    BUF2STRC( pabyBuf, n, pHeaderOne->Reserved );
    BUF2STRC( pabyBuf, n, pHeaderOne->GridFileVersion );

#if defined(CPL_MSB)
    CPL_LSBPTR16(&pHeaderOne->WordsToFollow);
    CPL_LSBPTR16(&pHeaderOne->DataTypeCode);
    CPL_LSBPTR16(&pHeaderOne->ApplicationType);
    CPL_LSBPTR32(&pHeaderOne->PixelsPerLine);
    CPL_LSBPTR32(&pHeaderOne->NumberOfLines);
    CPL_LSBPTR16(&pHeaderOne->DeviceResolution);
    CPL_LSBPTR16(&pHeaderOne->DataTypeModifier);
    switch (INGR_GetDataType(pHeaderOne->DataTypeCode))
    {
    case GDT_Byte:    
        pHeaderOne->Minimum.AsUint8 = *(uint8*)&(pHeaderOne->Minimum);
        pHeaderOne->Maximum.AsUint8 = *(uint8*)&(pHeaderOne->Maximum); 
        break;
    case GDT_Int16:   
        pHeaderOne->Minimum.AsUint16 = CPL_LSBWORD16(*(uint16*)&(pHeaderOne->Minimum));
        pHeaderOne->Maximum.AsUint16 = CPL_LSBWORD16(*(uint16*)&(pHeaderOne->Maximum)); 
        break;
    case GDT_UInt16:  
        pHeaderOne->Minimum.AsUint16 = CPL_LSBWORD16(*(uint16*)&(pHeaderOne->Minimum));
        pHeaderOne->Maximum.AsUint16 = CPL_LSBWORD16(*(uint16*)&(pHeaderOne->Maximum)); 
        break;
    case GDT_Int32:   
        pHeaderOne->Minimum.AsUint32 = CPL_LSBWORD32(*(uint32*)&(pHeaderOne->Minimum));
        pHeaderOne->Maximum.AsUint32 = CPL_LSBWORD32(*(uint32*)&(pHeaderOne->Maximum)); 
        break;
    case GDT_UInt32:  
        pHeaderOne->Minimum.AsUint32 = CPL_LSBWORD32(*(uint32*)&(pHeaderOne->Minimum));
        pHeaderOne->Maximum.AsUint32 = CPL_LSBWORD32(*(uint32*)&(pHeaderOne->Maximum)); 
        break;
        /* FIXME ? I'm not sure this is correct for floats */
    case GDT_Float32: 
        pHeaderOne->Minimum.AsUint32 = CPL_LSBWORD32(*(uint32*)&(pHeaderOne->Minimum));
        pHeaderOne->Maximum.AsUint32 = CPL_LSBWORD32(*(uint32*)&(pHeaderOne->Maximum)); 
        break;
    case GDT_Float64: 
        CPL_LSBPTR64(&pHeaderOne->Minimum); CPL_LSBPTR64(&pHeaderOne->Maximum); 
        break;
    default: break;
    }
#endif

    // -------------------------------------------------------------------- 
    // Convert WAX REAL*8 to IEEE double
    // -------------------------------------------------------------------- 

    if( pHeaderOne->GridFileVersion == 1 ||
      ( pHeaderOne->GridFileVersion == 2 && 
        ( pHeaderOne->TransformationMatrix[10] != 1.0 && 
          pHeaderOne->TransformationMatrix[15] != 1.0 ) ) )
    {
        INGR_DGN2IEEEDouble( &pHeaderOne->XViewOrigin );
        INGR_DGN2IEEEDouble( &pHeaderOne->YViewOrigin );
        INGR_DGN2IEEEDouble( &pHeaderOne->ZViewOrigin );
        INGR_DGN2IEEEDouble( &pHeaderOne->XViewExtent );
        INGR_DGN2IEEEDouble( &pHeaderOne->YViewExtent );
        INGR_DGN2IEEEDouble( &pHeaderOne->ZViewExtent );
        INGR_DGN2IEEEDouble( &pHeaderOne->RotationAngle );
        INGR_DGN2IEEEDouble( &pHeaderOne->SkewAngle );

        uint8 i;

        for( i = 0; i < 16; i++ )
        {
            INGR_DGN2IEEEDouble( &pHeaderOne->TransformationMatrix[i]);
        }
    }
    else if (pHeaderOne->GridFileVersion == 3)
    {
#ifdef CPL_MSB
        CPL_LSBPTR64( &pHeaderOne->XViewOrigin );
        CPL_LSBPTR64( &pHeaderOne->YViewOrigin );
        CPL_LSBPTR64( &pHeaderOne->ZViewOrigin );
        CPL_LSBPTR64( &pHeaderOne->XViewExtent );
        CPL_LSBPTR64( &pHeaderOne->YViewExtent );
        CPL_LSBPTR64( &pHeaderOne->ZViewExtent );
        CPL_LSBPTR64( &pHeaderOne->RotationAngle );
        CPL_LSBPTR64( &pHeaderOne->SkewAngle );

        uint8 i;

        for( i = 0; i < 16; i++ )
        {
            CPL_LSBPTR64( &pHeaderOne->TransformationMatrix[i]);
        }
#endif
    }
}

void CPL_STDCALL INGR_HeaderOneMemToDisk(INGR_HeaderOne* pHeaderOne, GByte *pabyBuf)
{
    unsigned int n = 0;

    STRC2BUF( pabyBuf, n, pHeaderOne->HeaderType );
    STRC2BUF( pabyBuf, n, pHeaderOne->WordsToFollow );
    STRC2BUF( pabyBuf, n, pHeaderOne->DataTypeCode );
    STRC2BUF( pabyBuf, n, pHeaderOne->ApplicationType );
    STRC2BUF( pabyBuf, n, pHeaderOne->XViewOrigin );
    STRC2BUF( pabyBuf, n, pHeaderOne->YViewOrigin );
    STRC2BUF( pabyBuf, n, pHeaderOne->ZViewOrigin );
    STRC2BUF( pabyBuf, n, pHeaderOne->XViewExtent );
    STRC2BUF( pabyBuf, n, pHeaderOne->YViewExtent );
    STRC2BUF( pabyBuf, n, pHeaderOne->ZViewExtent );
    STRC2BUF( pabyBuf, n, pHeaderOne->TransformationMatrix );
    STRC2BUF( pabyBuf, n, pHeaderOne->PixelsPerLine );
    STRC2BUF( pabyBuf, n, pHeaderOne->NumberOfLines );
    STRC2BUF( pabyBuf, n, pHeaderOne->DeviceResolution );
    STRC2BUF( pabyBuf, n, pHeaderOne->ScanlineOrientation );
    STRC2BUF( pabyBuf, n, pHeaderOne->ScannableFlag );
    STRC2BUF( pabyBuf, n, pHeaderOne->RotationAngle );
    STRC2BUF( pabyBuf, n, pHeaderOne->SkewAngle );
    STRC2BUF( pabyBuf, n, pHeaderOne->DataTypeModifier );
    STRC2BUF( pabyBuf, n, pHeaderOne->DesignFileName );
    STRC2BUF( pabyBuf, n, pHeaderOne->DataBaseFileName );
    STRC2BUF( pabyBuf, n, pHeaderOne->ParentGridFileName );
    STRC2BUF( pabyBuf, n, pHeaderOne->FileDescription );
    STRC2BUF( pabyBuf, n, pHeaderOne->Minimum );
    STRC2BUF( pabyBuf, n, pHeaderOne->Maximum );
    STRC2BUF( pabyBuf, n, pHeaderOne->Reserved );
    STRC2BUF( pabyBuf, n, pHeaderOne->GridFileVersion );

#if defined(CPL_MSB)
    switch (INGR_GetDataType(pHeaderOne->DataTypeCode))
    {
        case GDT_Byte:    *(uint8*)&(pHeaderOne->Minimum) = pHeaderOne->Minimum.AsUint8;
                          *(uint8*)&(pHeaderOne->Maximum) = pHeaderOne->Maximum.AsUint8; break;
        case GDT_Int16:   *(uint16*)&(pHeaderOne->Minimum) = CPL_LSBWORD16(pHeaderOne->Minimum.AsUint16);
                          *(uint16*)&(pHeaderOne->Maximum) = CPL_LSBWORD16(pHeaderOne->Maximum.AsUint16); break;
        case GDT_UInt16:  *(uint16*)&(pHeaderOne->Minimum) = CPL_LSBWORD16(pHeaderOne->Minimum.AsUint16);
                          *(uint16*)&(pHeaderOne->Maximum) = CPL_LSBWORD16(pHeaderOne->Maximum.AsUint16); break;
        case GDT_Int32:   *(uint32*)&(pHeaderOne->Minimum) = CPL_LSBWORD32(pHeaderOne->Minimum.AsUint32);
                          *(uint32*)&(pHeaderOne->Maximum) = CPL_LSBWORD32(pHeaderOne->Maximum.AsUint32); break;
        case GDT_UInt32:  *(uint32*)&(pHeaderOne->Minimum) = CPL_LSBWORD32(pHeaderOne->Minimum.AsUint32);
                          *(uint32*)&(pHeaderOne->Maximum) = CPL_LSBWORD32(pHeaderOne->Maximum.AsUint32); break;
        /* FIXME ? I'm not sure this is correct for floats */
        case GDT_Float32: *(uint32*)&(pHeaderOne->Minimum) = CPL_LSBWORD32(pHeaderOne->Minimum.AsUint32);
                          *(uint32*)&(pHeaderOne->Maximum) = CPL_LSBWORD32(pHeaderOne->Maximum.AsUint32); break;
        case GDT_Float64: CPL_LSBPTR64(&pHeaderOne->Minimum); CPL_LSBPTR64(&pHeaderOne->Maximum); break;
        default: break;
    }

    CPL_LSBPTR16(&pHeaderOne->WordsToFollow);
    CPL_LSBPTR16(&pHeaderOne->DataTypeCode);
    CPL_LSBPTR16(&pHeaderOne->ApplicationType);
    CPL_LSBPTR32(&pHeaderOne->PixelsPerLine);
    CPL_LSBPTR32(&pHeaderOne->NumberOfLines);
    CPL_LSBPTR16(&pHeaderOne->DeviceResolution);
    CPL_LSBPTR16(&pHeaderOne->DataTypeModifier);

    if (pHeaderOne->GridFileVersion == 3)
    {
        CPL_LSBPTR64( &pHeaderOne->XViewOrigin );
        CPL_LSBPTR64( &pHeaderOne->YViewOrigin );
        CPL_LSBPTR64( &pHeaderOne->ZViewOrigin );
        CPL_LSBPTR64( &pHeaderOne->XViewExtent );
        CPL_LSBPTR64( &pHeaderOne->YViewExtent );
        CPL_LSBPTR64( &pHeaderOne->ZViewExtent );
        CPL_LSBPTR64( &pHeaderOne->RotationAngle );
        CPL_LSBPTR64( &pHeaderOne->SkewAngle );

        uint8 i;

        for( i = 0; i < 16; i++ )
        {
            CPL_LSBPTR64( &pHeaderOne->TransformationMatrix[i]);
        }
    }
#endif
}

void CPL_STDCALL INGR_HeaderTwoADiskToMem(INGR_HeaderTwoA* pHeaderTwo, GByte *pabyBuf)
{
    unsigned int n = 0;

    BUF2STRC( pabyBuf, n, pHeaderTwo->Gain );                    
    BUF2STRC( pabyBuf, n, pHeaderTwo->OffsetThreshold );         
    BUF2STRC( pabyBuf, n, pHeaderTwo->View1 );                   
    BUF2STRC( pabyBuf, n, pHeaderTwo->View2 );                   
    BUF2STRC( pabyBuf, n, pHeaderTwo->ViewNumber );              
    BUF2STRC( pabyBuf, n, pHeaderTwo->Reserved2 );               
    BUF2STRC( pabyBuf, n, pHeaderTwo->Reserved3 );               
    BUF2STRC( pabyBuf, n, pHeaderTwo->AspectRatio );             
    BUF2STRC( pabyBuf, n, pHeaderTwo->CatenatedFilePointer );    
    BUF2STRC( pabyBuf, n, pHeaderTwo->ColorTableType );          
    BUF2STRC( pabyBuf, n, pHeaderTwo->Reserved8 );               
    BUF2STRC( pabyBuf, n, pHeaderTwo->NumberOfCTEntries );       
    BUF2STRC( pabyBuf, n, pHeaderTwo->ApplicationPacketPointer );
    BUF2STRC( pabyBuf, n, pHeaderTwo->ApplicationPacketLength ); 
    BUF2STRC( pabyBuf, n, pHeaderTwo->Reserved );           

#if defined(CPL_MSB)
    CPL_LSBPTR64(&pHeaderTwo->AspectRatio);
    CPL_LSBPTR32(&pHeaderTwo->CatenatedFilePointer);
    CPL_LSBPTR16(&pHeaderTwo->ColorTableType);
    CPL_LSBPTR32(&pHeaderTwo->NumberOfCTEntries);
    CPL_LSBPTR32(&pHeaderTwo->ApplicationPacketPointer);
    CPL_LSBPTR32(&pHeaderTwo->ApplicationPacketLength);
#endif
}

void CPL_STDCALL INGR_HeaderTwoAMemToDisk(INGR_HeaderTwoA* pHeaderTwo, GByte *pabyBuf)
{
    unsigned int n = 0;

    STRC2BUF( pabyBuf, n, pHeaderTwo->Gain );                    
    STRC2BUF( pabyBuf, n, pHeaderTwo->OffsetThreshold );         
    STRC2BUF( pabyBuf, n, pHeaderTwo->View1 );                   
    STRC2BUF( pabyBuf, n, pHeaderTwo->View2 );                   
    STRC2BUF( pabyBuf, n, pHeaderTwo->ViewNumber );              
    STRC2BUF( pabyBuf, n, pHeaderTwo->Reserved2 );               
    STRC2BUF( pabyBuf, n, pHeaderTwo->Reserved3 );               
    STRC2BUF( pabyBuf, n, pHeaderTwo->AspectRatio );             
    STRC2BUF( pabyBuf, n, pHeaderTwo->CatenatedFilePointer );    
    STRC2BUF( pabyBuf, n, pHeaderTwo->ColorTableType );          
    STRC2BUF( pabyBuf, n, pHeaderTwo->Reserved8 );               
    STRC2BUF( pabyBuf, n, pHeaderTwo->NumberOfCTEntries );       
    STRC2BUF( pabyBuf, n, pHeaderTwo->ApplicationPacketPointer );
    STRC2BUF( pabyBuf, n, pHeaderTwo->ApplicationPacketLength ); 
    STRC2BUF( pabyBuf, n, pHeaderTwo->Reserved );           

#if defined(CPL_MSB)
    CPL_LSBPTR64(&pHeaderTwo->AspectRatio);
    CPL_LSBPTR32(&pHeaderTwo->CatenatedFilePointer);
    CPL_LSBPTR16(&pHeaderTwo->ColorTableType);
    CPL_LSBPTR32(&pHeaderTwo->NumberOfCTEntries);
    CPL_LSBPTR32(&pHeaderTwo->ApplicationPacketPointer);
    CPL_LSBPTR32(&pHeaderTwo->ApplicationPacketLength);
#endif
}

void CPL_STDCALL INGR_TileHeaderDiskToMem(INGR_TileHeader* pTileHeader, GByte *pabyBuf)
{
    unsigned int n = 0;

    BUF2STRC( pabyBuf, n, pTileHeader->ApplicationType );
    BUF2STRC( pabyBuf, n, pTileHeader->SubTypeCode );
    BUF2STRC( pabyBuf, n, pTileHeader->WordsToFollow );
    BUF2STRC( pabyBuf, n, pTileHeader->PacketVersion );
    BUF2STRC( pabyBuf, n, pTileHeader->Identifier );
    BUF2STRC( pabyBuf, n, pTileHeader->Reserved );
    BUF2STRC( pabyBuf, n, pTileHeader->Properties );
    BUF2STRC( pabyBuf, n, pTileHeader->DataTypeCode );
    BUF2STRC( pabyBuf, n, pTileHeader->Reserved2 );
    BUF2STRC( pabyBuf, n, pTileHeader->TileSize ); 
    BUF2STRC( pabyBuf, n, pTileHeader->Reserved3 );
    BUF2STRC( pabyBuf, n, pTileHeader->First );

#if defined(CPL_MSB)
    CPL_LSBPTR16(&pTileHeader->ApplicationType);
    CPL_LSBPTR16(&pTileHeader->SubTypeCode);
    CPL_LSBPTR32(&pTileHeader->WordsToFollow);
    CPL_LSBPTR16(&pTileHeader->PacketVersion);
    CPL_LSBPTR16(&pTileHeader->Identifier);
    CPL_LSBPTR16(&pTileHeader->Properties);
    CPL_LSBPTR16(&pTileHeader->DataTypeCode);
    CPL_LSBPTR32(&pTileHeader->TileSize);
    CPL_LSBPTR32(&pTileHeader->First->Start);
    CPL_LSBPTR32(&pTileHeader->First->Allocated);
    CPL_LSBPTR32(&pTileHeader->First->Used);
#endif
}

void CPL_STDCALL INGR_TileItemDiskToMem(INGR_TileItem* pTileItem, GByte *pabyBuf)
{
    unsigned int n = 0;

    BUF2STRC( pabyBuf, n, pTileItem->Start );
    BUF2STRC( pabyBuf, n, pTileItem->Allocated );
    BUF2STRC( pabyBuf, n, pTileItem->Used );

#if defined(CPL_MSB)
    CPL_LSBPTR32(&pTileItem->Start);
    CPL_LSBPTR32(&pTileItem->Allocated);
    CPL_LSBPTR32(&pTileItem->Used);
#endif
}

void CPL_STDCALL INGR_JPEGAppDataDiskToMem(INGR_JPEGAppData* pJPEGAppData, GByte *pabyBuf)
{
    unsigned int n = 0;

    BUF2STRC( pabyBuf, n, pJPEGAppData->ApplicationType );
    BUF2STRC( pabyBuf, n, pJPEGAppData->SubTypeCode );
    BUF2STRC( pabyBuf, n, pJPEGAppData->RemainingLength );
    BUF2STRC( pabyBuf, n, pJPEGAppData->PacketVersion );
    BUF2STRC( pabyBuf, n, pJPEGAppData->JpegQuality );

#if defined(CPL_MSB)
    CPL_LSBPTR16(&pJPEGAppData->ApplicationType);
    CPL_LSBPTR16(&pJPEGAppData->SubTypeCode);
    CPL_LSBPTR32(&pJPEGAppData->RemainingLength);
    CPL_LSBPTR16(&pJPEGAppData->PacketVersion);
    CPL_LSBPTR16(&pJPEGAppData->JpegQuality);
#endif
}


//  ------------------------------------------------------------------
//    Pasted from the DNG OGR Driver to avoid dependency on OGR
//  ------------------------------------------------------------------

typedef struct dbl {
    GUInt32 hi;
    GUInt32 lo;
} double64_t;

/************************************************************************/
/*                           INGR_DGN2IEEEDouble()                      */
/************************************************************************/

void    INGR_DGN2IEEEDouble(void * dbl)

{
    double64_t  dt;
    GUInt32     sign;
    GUInt32     exponent;
    GUInt32     rndbits;
    unsigned char       *src;
    unsigned char       *dest;

/* -------------------------------------------------------------------- */
/*      Arrange the VAX double so that it may be accessed by a          */
/*      double64_t structure, (two GUInt32s).                           */
/* -------------------------------------------------------------------- */
    src =  (unsigned char *) dbl;
    dest = (unsigned char *) &dt;
#ifdef CPL_LSB
    dest[2] = src[0];
    dest[3] = src[1];
    dest[0] = src[2];
    dest[1] = src[3];
    dest[6] = src[4];
    dest[7] = src[5];
    dest[4] = src[6];
    dest[5] = src[7];
#else
    dest[1] = src[0];
    dest[0] = src[1];
    dest[3] = src[2];
    dest[2] = src[3];
    dest[5] = src[4];
    dest[4] = src[5];
    dest[7] = src[6];
    dest[6] = src[7];
#endif

/* -------------------------------------------------------------------- */
/*      Save the sign of the double                                     */
/* -------------------------------------------------------------------- */
    sign         = dt.hi & 0x80000000;

/* -------------------------------------------------------------------- */
/*      Adjust the exponent so that we may work with it                 */      
/* -------------------------------------------------------------------- */
    exponent = dt.hi >> 23;
    exponent = exponent & 0x000000ff;

    if (exponent)
        exponent = exponent -129 + 1023;

/* -------------------------------------------------------------------- */
/*      Save the bits that we are discarding so we can round properly   */
/* -------------------------------------------------------------------- */
    rndbits = dt.lo & 0x00000007;
        
    dt.lo = dt.lo >> 3;
    dt.lo = (dt.lo & 0x1fffffff) | (dt.hi << 29);

    if (rndbits)
        dt.lo = dt.lo | 0x00000001;

/* -------------------------------------------------------------------- */
/*      Shift the hi-order int over 3 and insert the exponent and sign  */
/* -------------------------------------------------------------------- */
    dt.hi = dt.hi >> 3;
    dt.hi = dt.hi & 0x000fffff;
    dt.hi = dt.hi | (exponent << 20) | sign;



#ifdef CPL_LSB
/* -------------------------------------------------------------------- */
/*      Change the number to a byte swapped format                      */
/* -------------------------------------------------------------------- */
    src = (unsigned char *) &dt;
    dest = (unsigned char *) dbl;

    dest[0] = src[4];
    dest[1] = src[5];
    dest[2] = src[6];
    dest[3] = src[7];
    dest[4] = src[0];
    dest[5] = src[1];
    dest[6] = src[2];
    dest[7] = src[3];
#else
    memcpy( dbl, &dt, 8 );
#endif
}
