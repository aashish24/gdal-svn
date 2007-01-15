/******************************************************************************
 * $Id$
 *
 * Project:  TIGER/Line Translator
 * Purpose:  Implements TigerPoint class.
 * Author:   Mark Phillips, mbp@geomtech.com
 *
 ******************************************************************************
 * Copyright (c) 2002, Mark Phillips
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
 * Revision 1.5  2003/09/11 22:47:53  aamici
 * add class constructors and destructors where needed in order to
 * let the mingw/cygwin binutils produce sensible partially linked objet files
 * with 'ld -r'.
 *
 * Revision 1.4  2003/01/11 15:29:55  warmerda
 * expanded tabs
 *
 * Revision 1.3  2003/01/09 18:27:40  warmerda
 * added headers/function headers
 *
 */

#include "ogr_tiger.h"
#include "cpl_conv.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*                             TigerPoint()                             */
/************************************************************************/
TigerPoint::TigerPoint( int bRequireGeom )
{
    this->bRequireGeom = bRequireGeom;
}

TigerPoint::~TigerPoint()
{
}

/************************************************************************/
/*                             SetModule()                              */
/************************************************************************/
int TigerPoint::SetModule( const char * pszModule, char *pszFileCode )
{
    if( !OpenFile( pszModule, pszFileCode ) )
        return FALSE;
    EstablishFeatureCount();
    return TRUE;
}

/************************************************************************/
/*                             GetFeature()                             */
/************************************************************************/
OGRFeature *TigerPoint::GetFeature( int nRecordId,
                                    TigerRecordInfo *psRTInfo,
                                    int nX0, int nX1,
                                    int nY0, int nY1 )
{
    char        achRecord[OGR_TIGER_RECBUF_LEN];

    if( nRecordId < 0 || nRecordId >= nFeatures ) {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Request for out-of-range feature %d of %sP",
                  nRecordId, pszModule );
        return NULL;
    }

    /* -------------------------------------------------------------------- */
    /*      Read the raw record data from the file.                         */
    /* -------------------------------------------------------------------- */

    if( fpPrimary == NULL )
        return NULL;

    if( VSIFSeek( fpPrimary, nRecordId * nRecordLength, SEEK_SET ) != 0 ) {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Failed to seek to %d of %sP",
                  nRecordId * nRecordLength, pszModule );
        return NULL;
    }

    if( VSIFRead( achRecord, psRTInfo->nRecordLength, 1, fpPrimary ) != 1 ) {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Failed to read record %d of %sP",
                  nRecordId, pszModule );
        return NULL;
    }

    /* -------------------------------------------------------------------- */
    /*      Set fields.                                                     */
    /* -------------------------------------------------------------------- */

    OGRFeature  *poFeature = new OGRFeature( poFeatureDefn );

    SetFields( psRTInfo, poFeature, achRecord);

    /* -------------------------------------------------------------------- */
    /*      Set geometry                                                    */
    /* -------------------------------------------------------------------- */

    double      dfX, dfY;

    dfX = atoi(GetField(achRecord, nX0, nX1)) / 1000000.0;
    dfY = atoi(GetField(achRecord, nY0, nY1)) / 1000000.0;

    if( dfX != 0.0 || dfY != 0.0 ) {
        poFeature->SetGeometryDirectly( new OGRPoint( dfX, dfY ) );
    }
        
    return poFeature;
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/
OGRErr TigerPoint::CreateFeature( OGRFeature *poFeature, 
                                  TigerRecordInfo *psRTInfo,
                                  int pointIndex,
                                  char *pszFileCode)

{
    char        szRecord[OGR_TIGER_RECBUF_LEN];
    OGRPoint    *poPoint = (OGRPoint *) poFeature->GetGeometryRef();

    if( !SetWriteModule( pszFileCode, psRTInfo->nRecordLength+2, poFeature ) )
        return OGRERR_FAILURE;

    memset( szRecord, ' ', psRTInfo->nRecordLength );

    WriteFields( psRTInfo, poFeature, szRecord );

    if( poPoint != NULL 
        && (poPoint->getGeometryType() == wkbPoint
            || poPoint->getGeometryType() == wkbPoint25D) ) {
        WritePoint( szRecord, pointIndex, poPoint->getX(), poPoint->getY() );
    } else {
        if (bRequireGeom) {
            return OGRERR_FAILURE;
        }
    }

    WriteRecord( szRecord, psRTInfo->nRecordLength, pszFileCode );

    return OGRERR_NONE;
}
