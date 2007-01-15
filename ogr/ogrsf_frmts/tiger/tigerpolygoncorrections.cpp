/******************************************************************************
 * $Id$
 *
 * Project:  TIGER/Line Translator
 * Purpose:  Implements TigerPolygonCorrections, providing access to .RTB files.
 * Author:   Mark Phillips, mbp@geomtech.com
 *
 ******************************************************************************
 * Copyright (c) 2002, Frank Warmerdam, Mark Phillips
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
 * Revision 1.3  2005/09/21 00:53:19  fwarmerdam
 * fixup OGRFeatureDefn and OGRSpatialReference refcount handling
 *
 * Revision 1.2  2003/01/04 23:21:56  mbp
 * Minor bug fixes and field definition changes.  Cleaned
 * up and commented code written for TIGER 2002 support.
 *
 * Revision 1.1  2002/12/26 00:20:19  mbp
 * re-organized code to hold TIGER-version details in TigerRecordInfo structs;
 * first round implementation of TIGER_2002 support
 *
 */

#include "ogr_tiger.h"
#include "cpl_conv.h"

CPL_CVSID("$Id$");

#define FILE_CODE       "B"

static TigerFieldInfo rtB_fields[] = {
  // fieldname    fmt  type OFTType      beg  end  len  bDefine bSet bWrite
  { "MODULE",     ' ', ' ', OFTString,     0,   0,   8,       1,   0,     0 },
  { "FILE",       'L', 'N', OFTInteger,    6,  10,   5,       1,   1,     1 },
  { "CENID",      'L', 'A', OFTString,    11,  15,   5,       1,   1,     1 },
  { "POLYID",     'R', 'N', OFTInteger,   16,  25,  10,       1,   1,     1 },
  { "STATECQ",    'L', 'N', OFTInteger,   26,  27,   2,       1,   1,     1 },
  { "COUNTYCQ",   'L', 'N', OFTInteger,   28,  30,   3,       1,   1,     1 },
  { "TRACTCQ",    'L', 'N', OFTInteger,   31,  36,   6,       1,   1,     1 },
  { "BLOCKCQ",    'L', 'A', OFTString,    37,  41,   5,       1,   1,     1 },
  { "AIANHHFPCQ", 'L', 'N', OFTInteger,   42,  46,   5,       1,   1,     1 },
  { "AIANHHCQ",   'L', 'N', OFTInteger,   47,  50,   4,       1,   1,     1 },
  { "AIHHTLICQ",  'L', 'A', OFTString,    51,  51,   1,       1,   1,     1 },
  { "AITSCECQ",   'L', 'N', OFTInteger,   52,  54,   3,       1,   1,     1 },
  { "AITSCQ",     'L', 'N', OFTInteger,   55,  59,   5,       1,   1,     1 },
  { "ANRCCQ",     'L', 'N', OFTInteger,   60,  64,   5,       1,   1,     1 },
  { "CONCITCQ",   'L', 'N', OFTInteger,   65,  69,   5,       1,   1,     1 },
  { "COUSUBCQ",   'L', 'N', OFTInteger,   70,  74,   5,       1,   1,     1 },
  { "SUBMCDCQ",   'L', 'N', OFTInteger,   75,  79,   5,       1,   1,     1 },
  { "PLACECQ",    'L', 'N', OFTInteger,   80,  84,   5,       1,   1,     1 },
  { "UACC",       'L', 'N', OFTInteger,   85,  89,   5,       1,   1,     1 },
  { "URCC",       'L', 'A', OFTString,    90,  90,   1,       1,   1,     1 },
  { "RS-B1",      'L', 'A', OFTString,    91,  98,  12,       1,   1,     1 },
};
static TigerRecordInfo rtB_info =
  {
    rtB_fields,
    sizeof(rtB_fields) / sizeof(TigerFieldInfo),
    98
  };

/************************************************************************/
/*                     TigerPolygonCorrections()                        */
/************************************************************************/

TigerPolygonCorrections::TigerPolygonCorrections( OGRTigerDataSource * poDSIn,
                              const char * pszPrototypeModule )

{
    OGRFieldDefn        oField("",OFTInteger);

    poDS = poDSIn;
    poFeatureDefn = new OGRFeatureDefn( "PolygonCorrections" );
    poFeatureDefn->Reference();
    poFeatureDefn->SetGeomType( wkbNone );

    psRTBInfo = &rtB_info;

    /* -------------------------------------------------------------------- */
    /*      Fields from type B record.                                      */
    /* -------------------------------------------------------------------- */

    AddFieldDefns( psRTBInfo, poFeatureDefn );
}

/************************************************************************/
/*                           ~TigerPolygonCorrections()                           */
/************************************************************************/

TigerPolygonCorrections::~TigerPolygonCorrections()

{
}

/************************************************************************/
/*                             SetModule()                              */
/************************************************************************/

int TigerPolygonCorrections::SetModule( const char * pszModule )

{
    if( !OpenFile( pszModule, FILE_CODE ) )
        return FALSE;

    EstablishFeatureCount();
    
    return TRUE;
}

/************************************************************************/
/*                             GetFeature()                             */
/************************************************************************/

OGRFeature *TigerPolygonCorrections::GetFeature( int nRecordId )

{
    char        achRecord[OGR_TIGER_RECBUF_LEN];

    if( nRecordId < 0 || nRecordId >= nFeatures )
    {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Request for out-of-range feature %d of %sZ",
                  nRecordId, pszModule );
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Read the raw record data from the file.                         */
/* -------------------------------------------------------------------- */
    if( fpPrimary == NULL )
        return NULL;

    if( VSIFSeek( fpPrimary, nRecordId * nRecordLength, SEEK_SET ) != 0 )
    {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Failed to seek to %d of %sZ",
                  nRecordId * nRecordLength, pszModule );
        return NULL;
    }

    if( VSIFRead( achRecord, psRTBInfo->nRecordLength, 1, fpPrimary ) != 1 )
    {
        CPLError( CE_Failure, CPLE_FileIO,
                  "Failed to read record %d of %sZ",
                  nRecordId, pszModule );
        return NULL;
    }

    /* -------------------------------------------------------------------- */
    /*      Set fields.                                                     */
    /* -------------------------------------------------------------------- */

    OGRFeature  *poFeature = new OGRFeature( poFeatureDefn );

    SetFields( psRTBInfo, poFeature, achRecord );

    return poFeature;
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/

OGRErr TigerPolygonCorrections::CreateFeature( OGRFeature *poFeature )

{
    char        szRecord[OGR_TIGER_RECBUF_LEN];

    if( !SetWriteModule( FILE_CODE, psRTBInfo->nRecordLength+2, poFeature ) )
        return OGRERR_FAILURE;

    memset( szRecord, ' ', psRTBInfo->nRecordLength );

    WriteFields( psRTBInfo, poFeature, szRecord);

    WriteRecord( szRecord, psRTBInfo->nRecordLength, FILE_CODE );

    return OGRERR_NONE;
}
