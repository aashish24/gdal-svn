/******************************************************************************
 * $Id$
 *
 * Project:  CEOS Translator
 * Purpose:  GDALDataset driver for CEOS translator.
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
 * Revision 1.11  2006/04/04 00:34:39  fwarmerdam
 * updated contact info
 *
 * Revision 1.10  2005/05/05 14:01:36  fwarmerdam
 * PAM Enable
 *
 * Revision 1.9  2002/09/04 06:50:36  warmerda
 * avoid static driver pointers
 *
 * Revision 1.8  2002/06/12 21:12:24  warmerda
 * update to metadata based driver info
 *
 * Revision 1.7  2001/11/11 23:50:59  warmerda
 * added required class keyword to friend declarations
 *
 * Revision 1.6  2001/07/18 04:51:56  warmerda
 * added CPL_CVSID
 *
 * Revision 1.5  2000/08/15 19:28:26  warmerda
 * added help topic
 *
 * Revision 1.4  2000/06/20 17:35:58  warmerda
 * added overview support
 *
 * Revision 1.3  2000/02/28 16:32:20  warmerda
 * use SetBand method
 *
 * Revision 1.2  1999/05/17 01:52:55  warmerda
 * Removed unused variable.
 *
 * Revision 1.1  1999/05/05 17:32:38  warmerda
 * New
 *
 */

#include "ceosopen.h"
#include "gdal_pam.h"

CPL_CVSID("$Id$");

CPL_C_START
void	GDALRegister_CEOS(void);
CPL_C_END

/************************************************************************/
/* ==================================================================== */
/*				CEOSDataset				*/
/* ==================================================================== */
/************************************************************************/

class CEOSRasterBand;

class CEOSDataset : public GDALPamDataset
{
    friend class CEOSRasterBand;
    
    CEOSImage	*psCEOS;

  public:
                 CEOSDataset();
                ~CEOSDataset();
    static GDALDataset *Open( GDALOpenInfo * );
};

/************************************************************************/
/* ==================================================================== */
/*                            CEOSRasterBand                             */
/* ==================================================================== */
/************************************************************************/

class CEOSRasterBand : public GDALPamRasterBand
{
    friend class CEOSDataset;
    
  public:

    		CEOSRasterBand( CEOSDataset *, int );
    
    virtual CPLErr IReadBlock( int, int, void * );
};


/************************************************************************/
/*                           CEOSRasterBand()                            */
/************************************************************************/

CEOSRasterBand::CEOSRasterBand( CEOSDataset *poDS, int nBand )

{
    this->poDS = poDS;
    this->nBand = nBand;
    
    eDataType = GDT_Byte;

    nBlockXSize = poDS->GetRasterXSize();
    nBlockYSize = 1;
}

/************************************************************************/
/*                             IReadBlock()                             */
/************************************************************************/

CPLErr CEOSRasterBand::IReadBlock( int nBlockXOff, int nBlockYOff,
                                  void * pImage )

{
    CEOSDataset	*poCEOS_DS = (CEOSDataset *) poDS;

    CPLAssert( nBlockXOff == 0 );

    return( CEOSReadScanline(poCEOS_DS->psCEOS, nBand, nBlockYOff+1, pImage) );
}

/************************************************************************/
/* ==================================================================== */
/*                             CEOSDataset                              */
/* ==================================================================== */
/************************************************************************/

/************************************************************************/
/*                            CEOSDataset()                             */
/************************************************************************/

CEOSDataset::CEOSDataset()

{
    psCEOS = NULL;
}

/************************************************************************/
/*                            ~CEOSDataset()                            */
/************************************************************************/

CEOSDataset::~CEOSDataset()

{
    FlushCache();
    if( psCEOS )
        CEOSClose( psCEOS );
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *CEOSDataset::Open( GDALOpenInfo * poOpenInfo )

{
    CEOSImage	*psCEOS;
    int		i;
    
/* -------------------------------------------------------------------- */
/*      Before trying CEOSOpen() we first verify that the first         */
/*      record is in fact a CEOS file descriptor record.                */
/* -------------------------------------------------------------------- */
    if( poOpenInfo->fp == NULL || poOpenInfo->nHeaderBytes < 100 )
        return NULL;

    if( poOpenInfo->pabyHeader[4] != 0x3f
        || poOpenInfo->pabyHeader[5] != 0xc0
        || poOpenInfo->pabyHeader[6] != 0x12
        || poOpenInfo->pabyHeader[7] != 0x12 )
        return NULL;

/* -------------------------------------------------------------------- */
/*      Try opening the dataset.                                        */
/* -------------------------------------------------------------------- */
    psCEOS = CEOSOpen( poOpenInfo->pszFilename, "rb" );
    
    if( psCEOS == NULL )
        return( NULL );

/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */
    CEOSDataset 	*poDS;

    poDS = new CEOSDataset();

    poDS->psCEOS = psCEOS;
    
/* -------------------------------------------------------------------- */
/*      Capture some information from the file that is of interest.     */
/* -------------------------------------------------------------------- */
    poDS->nRasterXSize = psCEOS->nPixels;
    poDS->nRasterYSize = psCEOS->nLines;
    
/* -------------------------------------------------------------------- */
/*      Create band information objects.                                */
/* -------------------------------------------------------------------- */
    poDS->nBands = psCEOS->nBands;;

    for( i = 0; i < poDS->nBands; i++ )
        poDS->SetBand( i+1, new CEOSRasterBand( poDS, i+1 ) );

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

/************************************************************************/
/*                          GDALRegister_GTiff()                        */
/************************************************************************/

void GDALRegister_CEOS()

{
    GDALDriver	*poDriver;

    if( GDALGetDriverByName( "CEOS" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "CEOS" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "CEOS Image" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, 
                                   "frmt_various.html#CEOS" );
        
        poDriver->pfnOpen = CEOSDataset::Open;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}

