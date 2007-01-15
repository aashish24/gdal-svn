/******************************************************************************
 * $Id$
 *
 * Project:  Arc/Info Binary Grid Translator
 * Purpose:  Grid file access cover API for non-GDAL use.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 1999, Frank Warmerdam
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
 ******************************************************************************
 *
 * $Log$
 * Revision 1.19  2006/12/08 04:03:14  fwarmerdam
 * Some datasets (ie. "ChicagoProblem") have less blocks in the block map
 * than are required to cover the dataset region.  It seems the missing
 * blocks should be treated as nodata.  The code now treats it this way,
 * and not as an error, though a debug message is emitted.
 *
 * Revision 1.18  2005/10/31 04:51:55  fwarmerdam
 * upgraded to use large file API and GUInt32 for block offsets
 *
 * Revision 1.17  2004/01/29 20:48:10  warmerda
 * Make sure that if a bare "w001001.adf" is given to AIGOpen() it will
 * work.  The name is changed to ".".
 *
 * Revision 1.16  2003/07/08 15:36:10  warmerda
 * avoid warnings
 *
 * Revision 1.15  2002/11/11 18:29:03  warmerda
 * added AIGLLOpen() to support upper case names too
 *
 * Revision 1.14  2002/11/05 03:19:08  warmerda
 * avoid nodata remapping in gridlib, use GInt32 not GUInt for image data
 *
 * Revision 1.13  2002/10/31 03:08:47  warmerda
 * fixed email
 *
 * Revision 1.12  2002/05/14 21:38:57  warmerda
 * fixed up nPixel and nLines calculation to avoid roundoff error
 *
 * Revision 1.11  2001/07/18 04:51:56  warmerda
 * added CPL_CVSID
 *
 * Revision 1.10  2000/11/09 06:22:40  warmerda
 * save cover name
 *
 * Revision 1.9  2000/09/13 14:42:38  warmerda
 * Improved casting thanks for Ian Macleod (Geosoft).
 *
 * Revision 1.8  2000/02/18 14:47:12  warmerda
 * Avoid warnings.
 *
 * Revision 1.7  2000/02/18 04:55:02  warmerda
 * Set bHasWarned flag.
 *
 * Revision 1.6  1999/07/23 14:04:34  warmerda
 * Removed extra printf.
 *
 * Revision 1.5  1999/06/28 01:21:39  warmerda
 * added support for selecting .adf files as well as coverage dir
 *
 * Revision 1.4  1999/05/17 01:52:35  warmerda
 * Fixed argument type problem.
 *
 * Revision 1.3  1999/04/21 16:51:30  warmerda
 * fixed up floating point support
 *
 * Revision 1.2  1999/02/04 22:15:33  warmerda
 * fleshed out implementation
 *
 * Revision 1.1  1999/02/03 14:12:56  warmerda
 * New
 */

#include "aigrid.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*                              AIGOpen()                               */
/************************************************************************/

AIGInfo_t *AIGOpen( const char * pszInputName, const char * pszAccess )

{
    AIGInfo_t	*psInfo;
    char	*pszHDRFilename;
    char        *pszCoverName;

    (void) pszAccess;

/* -------------------------------------------------------------------- */
/*      If the pass name ends in .adf assume a file within the          */
/*      coverage has been selected, and strip that off the coverage     */
/*      name.                                                           */
/* -------------------------------------------------------------------- */
    pszCoverName = CPLStrdup( pszInputName );
    if( EQUAL(pszCoverName+strlen(pszCoverName)-4, ".adf") )
    {
        int      i;

        for( i = strlen(pszCoverName)-1; i > 0; i-- )
        {
            if( pszCoverName[i] == '\\' || pszCoverName[i] == '/' )
            {
                pszCoverName[i] = '\0';
                break;
            }
        }

        if( i == 0 )
            strcpy(pszCoverName,".");
    }

/* -------------------------------------------------------------------- */
/*      Allocate info structure.                                        */
/* -------------------------------------------------------------------- */
    psInfo = (AIGInfo_t *) CPLCalloc(sizeof(AIGInfo_t),1);
    psInfo->bHasWarned = FALSE;
    psInfo->pszCoverName = pszCoverName;

/* -------------------------------------------------------------------- */
/*      Read the header file.                                           */
/* -------------------------------------------------------------------- */
    if( AIGReadHeader( pszCoverName, psInfo ) != CE_None )
    {
        CPLFree( pszCoverName );
        CPLFree( psInfo );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Open the file w001001.adf file itself.                          */
/* -------------------------------------------------------------------- */
    pszHDRFilename = (char *) CPLMalloc(strlen(pszCoverName)+40);
    sprintf( pszHDRFilename, "%s/w001001.adf", pszCoverName );

    psInfo->fpGrid = AIGLLOpen( pszHDRFilename, "rb" );
    
    if( psInfo->fpGrid == NULL )
    {
        CPLError( CE_Failure, CPLE_OpenFailed,
                  "Failed to open grid file:\n%s\n",
                  pszHDRFilename );

        CPLFree( psInfo );
        CPLFree( pszHDRFilename );
        CPLFree( pszCoverName );
        return( NULL );
    }

    CPLFree( pszHDRFilename );
    pszHDRFilename = NULL;
    
/* -------------------------------------------------------------------- */
/*      Read the block index file.                                      */
/* -------------------------------------------------------------------- */
    if( AIGReadBlockIndex( pszCoverName, psInfo ) != CE_None )
    {
        VSIFCloseL( psInfo->fpGrid );
        
        CPLFree( psInfo );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Read the extents.                                               */
/* -------------------------------------------------------------------- */
    if( AIGReadBounds( pszCoverName, psInfo ) != CE_None )
    {
        VSIFCloseL( psInfo->fpGrid );
        
        CPLFree( psInfo );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Read the statistics.                                            */
/* -------------------------------------------------------------------- */
    if( AIGReadStatistics( pszCoverName, psInfo ) != CE_None )
    {
        VSIFCloseL( psInfo->fpGrid );
        
        CPLFree( psInfo );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Compute the number of pixels and lines.                         */
/* -------------------------------------------------------------------- */
    psInfo->nPixels = (int)
        ((psInfo->dfURX - psInfo->dfLLX + 0.5 * psInfo->dfCellSizeX) 
		/ psInfo->dfCellSizeX);
    psInfo->nLines = (int)
        ((psInfo->dfURY - psInfo->dfLLY + 0.5 * psInfo->dfCellSizeY) 
		/ psInfo->dfCellSizeY);
    
    return( psInfo );
}

/************************************************************************/
/*                            AIGReadTile()                             */
/************************************************************************/

CPLErr AIGReadTile( AIGInfo_t * psInfo, int nBlockXOff, int nBlockYOff,
                    GInt32 *panData )

{
    int		nBlockID;
    CPLErr	eErr;

/* -------------------------------------------------------------------- */
/*      validate block id.                                              */
/* -------------------------------------------------------------------- */
    nBlockID = nBlockXOff + nBlockYOff * psInfo->nBlocksPerRow;
    if( nBlockID < 0 
        || nBlockID >= psInfo->nBlocksPerRow * psInfo->nBlocksPerColumn )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Illegal block requested." );
        return CE_Failure;
    }

    if( nBlockID >= psInfo->nBlocks )
    {
        int i;
        CPLDebug( "AIG", 
                  "Request legal block, but from beyond end of block map.\n"
                  "Assuming all nodata." );
        for( i = psInfo->nBlockXSize * psInfo->nBlockYSize - 1; i >= 0; i-- )
            panData[i] = ESRI_GRID_NO_DATA;
        return CE_None;
    }
    
/* -------------------------------------------------------------------- */
/*      Read block.                                                     */
/* -------------------------------------------------------------------- */
    eErr = AIGReadBlock( psInfo->fpGrid,
                         psInfo->panBlockOffset[nBlockID],
                         psInfo->panBlockSize[nBlockID],
                         psInfo->nBlockXSize, psInfo->nBlockYSize,
                         panData, psInfo->nCellType );

/* -------------------------------------------------------------------- */
/*      Apply floating point post-processing.                           */
/* -------------------------------------------------------------------- */
    if( eErr == CE_None && psInfo->nCellType == AIG_CELLTYPE_FLOAT )
    {
        float	*pafData = (float *) panData;
        int	i, nPixels = psInfo->nBlockXSize * psInfo->nBlockYSize;

        for( i = 0; i < nPixels; i++ )
        {
            panData[i] = (int) pafData[i];
        }
    }

    return( eErr );
}

/************************************************************************/
/*                          AIGReadFloatTile()                          */
/************************************************************************/

CPLErr AIGReadFloatTile( AIGInfo_t * psInfo, int nBlockXOff, int nBlockYOff,
                         float *pafData )

{
    int		nBlockID;
    CPLErr	eErr;

/* -------------------------------------------------------------------- */
/*      validate block id.                                              */
/* -------------------------------------------------------------------- */
    nBlockID = nBlockXOff + nBlockYOff * psInfo->nBlocksPerRow;
    if( nBlockID < 0 
        || nBlockID >= psInfo->nBlocksPerRow * psInfo->nBlocksPerColumn )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Illegal block requested." );
        return CE_Failure;
    }

    if( nBlockID >= psInfo->nBlocks )
    {
        int i;
        CPLDebug( "AIG", 
                  "Request legal block, but from beyond end of block map.\n"
                  "Assuming all nodata." );
        for( i = psInfo->nBlockXSize * psInfo->nBlockYSize - 1; i >= 0; i-- )
            pafData[i] = ESRI_GRID_FLOAT_NO_DATA;
        return CE_None;
    }
    
/* -------------------------------------------------------------------- */
/*      Read block.                                                     */
/* -------------------------------------------------------------------- */
    eErr = AIGReadBlock( psInfo->fpGrid,
                         psInfo->panBlockOffset[nBlockID],
                         psInfo->panBlockSize[nBlockID],
                         psInfo->nBlockXSize, psInfo->nBlockYSize,
                         (GInt32 *) pafData, psInfo->nCellType );

/* -------------------------------------------------------------------- */
/*      Perform integer post processing.                                */
/* -------------------------------------------------------------------- */
    if( eErr == CE_None && psInfo->nCellType == AIG_CELLTYPE_INT )
    {
        GUInt32	*panData = (GUInt32 *) pafData;
        int	i, nPixels = psInfo->nBlockXSize * psInfo->nBlockYSize;

        for( i = 0; i < nPixels; i++ )
        {
            pafData[i] = (float) panData[i];
        }
    }

    return( eErr );
}

/************************************************************************/
/*                              AIGClose()                              */
/************************************************************************/

void AIGClose( AIGInfo_t * psInfo )

{
    VSIFCloseL( psInfo->fpGrid );

    CPLFree( psInfo->panBlockOffset );
    CPLFree( psInfo->panBlockSize );
    CPLFree( psInfo->pszCoverName );
    CPLFree( psInfo );
}

/************************************************************************/
/*                             AIGLLOpen()                              */
/*                                                                      */
/*      Low level fopen() replacement that will try provided, and       */
/*      upper cased versions of file names.                             */
/************************************************************************/

FILE *AIGLLOpen( const char *pszFilename, const char *pszAccess )

{
    FILE	*fp;

    fp = VSIFOpenL( pszFilename, pszAccess );
    if( fp == NULL )
    {
        char *pszUCFilename = CPLStrdup(pszFilename);
        int  i;

        for( i = strlen(pszUCFilename)-1; 
             pszUCFilename[i] != '/' && pszUCFilename[i] != '\\';
             i-- )
        {
            pszUCFilename[i] = (char) toupper(pszUCFilename[i]);
        }
        
        fp = VSIFOpenL( pszUCFilename, pszAccess );

        CPLFree( pszUCFilename );
    }

    return fp;
}

