/******************************************************************************
 * $Id: ogrshapedatasource.cpp 11928 2007-08-22 20:43:30Z mloskot $
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  IO Redirection via VSI services for shp/dbf io.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2007,  Frank Warmerdam
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

#include "shapefil.h"
#include "cpl_vsi.h"
#include "cpl_error.h"

CPL_CVSID("$Id: ogrshapelayer.cpp 12930 2007-11-21 22:42:17Z warmerdam $");

/************************************************************************/
/*                            VSI_SHP_Open()                            */
/************************************************************************/

SAFile VSI_SHP_Open( const char *pszFilename, const char *pszAccess )

{
    return (SAFile) VSIFOpenL( pszFilename, pszAccess );
}

/************************************************************************/
/*                            VSI_SHP_Read()                            */
/************************************************************************/

SAOffset VSI_SHP_Read( void *p, SAOffset size, SAOffset nmemb, SAFile file )

{
    return (SAOffset) VSIFReadL( p, (size_t) size, (size_t) nmemb, 
                                 (FILE *) file );
}

/************************************************************************/
/*                           VSI_SHP_Write()                            */
/************************************************************************/

SAOffset VSI_SHP_Write( void *p, SAOffset size, SAOffset nmemb, SAFile file )

{
    return (SAOffset) VSIFWriteL( p, (size_t) size, (size_t) nmemb, 
                                  (FILE *) file );
}

/************************************************************************/
/*                            VSI_SHP_Seek()                            */
/************************************************************************/

SAOffset VSI_SHP_Seek( SAFile file, SAOffset offset, int whence )

{
    return (SAOffset) VSIFSeekL( (FILE *) file, (long) offset, whence );
}

/************************************************************************/
/*                            VSI_SHP_Tell()                            */
/************************************************************************/

SAOffset VSI_SHP_Tell( SAFile file )

{
    return (SAOffset) VSIFTellL( (FILE *) file );
}

/************************************************************************/
/*                           VSI_SHP_Flush()                            */
/************************************************************************/

int VSI_SHP_Flush( SAFile file )

{
    return VSIFFlushL( (FILE *) file );
}

/************************************************************************/
/*                           VSI_SHP_Close()                            */
/************************************************************************/

int VSI_SHP_Close( SAFile file )

{
    return VSIFCloseL( (FILE *) file );
}

/************************************************************************/
/*                              SADError()                              */
/************************************************************************/

void VSI_SHP_Error( const char *message )

{
    CPLError( CE_Failure, CPLE_AppDefined, "%s", message );
}

/************************************************************************/
/*                        SASetupDefaultHooks()                         */
/************************************************************************/

void SASetupDefaultHooks( SAHooks *psHooks )

{
    psHooks->FOpen   = VSI_SHP_Open;
    psHooks->FRead   = VSI_SHP_Read;
    psHooks->FWrite  = VSI_SHP_Write;
    psHooks->FSeek   = VSI_SHP_Seek;
    psHooks->FTell   = VSI_SHP_Tell;
    psHooks->FFlush  = VSI_SHP_Flush;
    psHooks->FClose  = VSI_SHP_Close;

    psHooks->Error   = VSI_SHP_Error;
}
