/******************************************************************************
 * $Id$
 *
 * Project:  GDAL Python Bindings
 * Purpose:  Implementation of NumPy arrays as a GDALDataset.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2000, Frank Warmerdam
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
 ****************************************************************************/

#include "gdal_priv.h"
#include "../frmts/mem/memdataset.h"
#include "gdal_py.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*				NUMPYDataset				*/
/************************************************************************/

class NUMPYDataset : public GDALDataset
{
    PyArrayObject *psArray;

    double	  adfGeoTransform[6];
    char	  *pszProjection;

    int           nGCPCount;
    GDAL_GCP      *pasGCPList;
    char          *pszGCPProjection;

  public:
                 NUMPYDataset();
                 ~NUMPYDataset();

    virtual const char *GetProjectionRef(void);
    virtual CPLErr SetProjection( const char * );
    virtual CPLErr GetGeoTransform( double * );
    virtual CPLErr SetGeoTransform( double * );

    virtual int    GetGCPCount();
    virtual const char *GetGCPProjection();
    virtual const GDAL_GCP *GetGCPs();
    virtual CPLErr SetGCPs( int nGCPCount, const GDAL_GCP *pasGCPList,
                            const char *pszGCPProjection );

    static GDALDataset *Open( GDALOpenInfo * );
};

/************************************************************************/
/*                            NUMPYDataset()                            */
/************************************************************************/

NUMPYDataset::NUMPYDataset()

{
    pszProjection = CPLStrdup("");
    adfGeoTransform[0] = 0.0;
    adfGeoTransform[1] = 1.0;
    adfGeoTransform[2] = 0.0;
    adfGeoTransform[3] = 0.0;
    adfGeoTransform[4] = 0.0;
    adfGeoTransform[5] = 1.0;

    nGCPCount = 0;
    pasGCPList = NULL;
    pszGCPProjection = CPLStrdup("");
}

/************************************************************************/
/*                            ~NUMPYDataset()                            */
/************************************************************************/

NUMPYDataset::~NUMPYDataset()

{
    CPLFree( pszProjection );

    CPLFree( pszGCPProjection );
    if( nGCPCount > 0 )
    {
        GDALDeinitGCPs( nGCPCount, pasGCPList );
        CPLFree( pasGCPList );
    }

    FlushCache();
    Py_DECREF( psArray );
}

/************************************************************************/
/*                          GetProjectionRef()                          */
/************************************************************************/

const char *NUMPYDataset::GetProjectionRef()

{
    return( pszProjection );
}

/************************************************************************/
/*                           SetProjection()                            */
/************************************************************************/

CPLErr NUMPYDataset::SetProjection( const char * pszNewProjection )

{
    CPLFree( pszProjection );
    pszProjection = CPLStrdup( pszNewProjection );

    return CE_None;
}

/************************************************************************/
/*                          GetGeoTransform()                           */
/************************************************************************/

CPLErr NUMPYDataset::GetGeoTransform( double * padfTransform )

{
    memcpy( padfTransform, adfGeoTransform, sizeof(double)*6 );
    return CE_None;
}

/************************************************************************/
/*                          SetGeoTransform()                           */
/************************************************************************/

CPLErr NUMPYDataset::SetGeoTransform( double * padfTransform )

{
    memcpy( adfGeoTransform, padfTransform, sizeof(double)*6 );
    return( CE_None );
}

/************************************************************************/
/*                            GetGCPCount()                             */
/************************************************************************/

int NUMPYDataset::GetGCPCount()

{
    return nGCPCount;
}

/************************************************************************/
/*                          GetGCPProjection()                          */
/************************************************************************/

const char *NUMPYDataset::GetGCPProjection()

{
    return pszGCPProjection;
}

/************************************************************************/
/*                               GetGCPs()                              */
/************************************************************************/

const GDAL_GCP *NUMPYDataset::GetGCPs()

{
    return pasGCPList;
}

/************************************************************************/
/*                              SetGCPs()                               */
/************************************************************************/

CPLErr NUMPYDataset::SetGCPs( int nGCPCount, const GDAL_GCP *pasGCPList,
                              const char *pszGCPProjection )

{
    CPLFree( this->pszGCPProjection );
    if( this->nGCPCount > 0 )
    {
        GDALDeinitGCPs( this->nGCPCount, this->pasGCPList );
        CPLFree( this->pasGCPList );
    }

    this->pszGCPProjection = CPLStrdup(pszGCPProjection);

    this->nGCPCount = nGCPCount;

    this->pasGCPList = GDALDuplicateGCPs( nGCPCount, pasGCPList );

    return CE_None;
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *NUMPYDataset::Open( GDALOpenInfo * poOpenInfo )

{
    PyArrayObject *psArray;
    GDALDataType  eType;
    int     nBands;

/* -------------------------------------------------------------------- */
/*      Is this a numpy dataset name?                                   */
/* -------------------------------------------------------------------- */
    if( !EQUALN(poOpenInfo->pszFilename,"NUMPY:::",8) 
        || poOpenInfo->fp != NULL )
        return NULL;

    psArray = NULL;
    sscanf( poOpenInfo->pszFilename+8, "%p", &psArray );
    if( psArray == NULL )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Failed to parse meaningful pointer value from NUMPY name\n"
                  "string: %s\n", 
                  poOpenInfo->pszFilename );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      If we likely have corrupt definitions of the NUMPY stuff,       */
/*      then warn now.                                                  */
/* -------------------------------------------------------------------- */
#ifdef NUMPY_DEFS_WRONG
    CPLError( CE_Warning, CPLE_AppDefined, 
              "It would appear you have built GDAL without having it use\n"
              "the Numeric python include files.  Old definitions have\n"
              "been used instead at build time, and it is quite possible that\n"
              "the things will shortly fail or crash if they are wrong.\n"
              "Consider installing Numeric, and rebuilding with HAVE_NUMPY\n"
              "enabled in gdal\nmake.opt." );
#endif

/* -------------------------------------------------------------------- */
/*      Is this a directly mappable Python array?  Verify rank, and     */
/*      data type.                                                      */
/* -------------------------------------------------------------------- */
    if( psArray->nd < 2 || psArray->nd > 3 )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Illegal numpy array rank %d.\n", 
                  psArray->nd );
        return NULL;
    }

    switch( psArray->descr->type )
    {
      case 'D':
        eType = GDT_CFloat64;
        break;

      case 'F':
        eType = GDT_CFloat32;
        break;

      case 'd':
        eType = GDT_Float64;
        break;

      case 'f':
        eType = GDT_Float32;
        break;

      case 'l':
      case 'i':
        eType = GDT_Int32;
        break;

      case 'u':
        eType = GDT_UInt32;
        break;

      case 's':
        eType = GDT_Int16;
        break;

      case 'w':
        eType = GDT_UInt16;
        break;

      case 'b':
        eType = GDT_Byte;
        break;

      default:
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Unable to access numpy arrays of typecode `%c'.\n", 
                  psArray->descr->type );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Create the new NUMPYDataset object.                             */
/* -------------------------------------------------------------------- */
    NUMPYDataset *poDS;

    poDS = new NUMPYDataset();

    poDS->psArray = psArray;

    poDS->eAccess = GA_ReadOnly;

/* -------------------------------------------------------------------- */
/*      Add a reference to the array.                                   */
/* -------------------------------------------------------------------- */
    Py_INCREF( psArray );

/* -------------------------------------------------------------------- */
/*      Workout the data layout.                                        */
/* -------------------------------------------------------------------- */
    int    nBandOffset;
    int    nPixelOffset;
    int    nLineOffset;

    if( psArray->nd == 3 )
    {
        nBands = psArray->dimensions[0];
        nBandOffset = psArray->strides[0];
        poDS->nRasterXSize = psArray->dimensions[2];
        nPixelOffset = psArray->strides[2];
        poDS->nRasterYSize = psArray->dimensions[1];
        nLineOffset = psArray->strides[1];
    }
    else
    {
        nBands = 1;
        nBandOffset = 0;
        poDS->nRasterXSize = psArray->dimensions[1];
        nPixelOffset = psArray->strides[1];
        poDS->nRasterYSize = psArray->dimensions[0];
        nLineOffset = psArray->strides[0];
    }

/* -------------------------------------------------------------------- */
/*      Create band information objects.                                */
/* -------------------------------------------------------------------- */
    for( int iBand = 0; iBand < nBands; iBand++ )
    {
        poDS->SetBand( iBand+1, 
                       (GDALRasterBand *) 
                       MEMCreateRasterBand( poDS, iBand+1, 
                                (GByte *) psArray->data + nBandOffset*iBand,
                                          eType, nPixelOffset, nLineOffset,
                                          FALSE ) );
    }

/* -------------------------------------------------------------------- */
/*      Try to return a regular handle on the file.                     */
/* -------------------------------------------------------------------- */
    return poDS;
}

/************************************************************************/
/*                          GDALRegister_NUMPY()                        */
/************************************************************************/

void GDALRegister_NUMPY()

{
    GDALDriver	*poDriver;

    if( GDALGetDriverByName( "NUMPY" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "NUMPY" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "Numeric Python Array" );
        
        poDriver->pfnOpen = NUMPYDataset::Open;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}
