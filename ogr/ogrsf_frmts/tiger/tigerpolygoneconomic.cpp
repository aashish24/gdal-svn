/******************************************************************************
 * $Id$
 *
 * Project:  TIGER/Line Translator
 * Purpose:  Implements TigerPolygonEconomic, providing access to .RTE files.
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
 * Revision 1.5  2005/09/21 00:53:19  fwarmerdam
 * fixup OGRFeatureDefn and OGRSpatialReference refcount handling
 *
 * Revision 1.4  2003/12/10 04:26:10  warmerda
 * updated RTE format based on 2003 spec and sample data
 *
 * Revision 1.3  2003/01/11 15:29:55  warmerda
 * expanded tabs
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

#define FILE_CODE       "E"

/* I think this was the expected RTE format, but was never deployed, leaving
   it in the code in case I am missing something. 
   
static TigerFieldInfo rtE_fields[] = {
  // fieldname    fmt  type OFTType      beg  end  len  bDefine bSet bWrite
  { "MODULE",     ' ', ' ', OFTString,     0,   0,   8,       1,   0,     0 },
  { "FILE",       'L', 'N', OFTInteger,    6,  10,   5,       1,   1,     1 },
  { "CENID",      'L', 'A', OFTString,    11,  15,   5,       1,   1,     1 },
  { "POLYID",     'R', 'N', OFTInteger,   16,  25,  10,       1,   1,     1 },
  { "STATEEC",    'L', 'N', OFTInteger,   26,  27,   2,       1,   1,     1 },
  { "COUNTYEC",   'L', 'N', OFTInteger,   28,  30,   3,       1,   1,     1 },
  { "CONCITEC",   'L', 'N', OFTInteger,   31,  35,   5,       1,   1,     1 },
  { "COUSUBEC",   'L', 'N', OFTInteger,   36,  40,   5,       1,   1,     1 },
  { "PLACEEC",    'L', 'N', OFTInteger,   41,  45,   5,       1,   1,     1 },
  { "AIANHHFPEC", 'L', 'N', OFTInteger,   46,  50,   5,       1,   1,     1 },
  { "AIANHHEC",   'L', 'N', OFTInteger,   51,  54,   4,       1,   1,     1 },
  { "AIAHHTLIEC", 'L', 'A', OFTString,    55,  55,   1,       1,   1,     1 },
  { "RS_E1",      'L', 'A', OFTString,    56,  73,  18,       1,   1,     1 }
};
*/

static TigerFieldInfo rtE_fields[] = {
  // fieldname    fmt  type OFTType      beg  end  len  bDefine bSet bWrite
  { "MODULE",     ' ', ' ', OFTString,     0,   0,   8,       1,   0,     0 },
  { "FILE",       'L', 'N', OFTInteger,    6,  10,   5,       1,   1,     1 },
  { "CENID",      'L', 'A', OFTString,    11,  15,   5,       1,   1,     1 },
  { "POLYID",     'R', 'N', OFTInteger,   16,  25,  10,       1,   1,     1 },
  { "STATEEC",    'L', 'N', OFTInteger,   26,  27,   2,       1,   1,     1 },
  { "COUNTYEC",   'L', 'N', OFTInteger,   28,  30,   3,       1,   1,     1 },
  { "RS_E1",      'L', 'A', OFTString,    31,  35,   5,       1,   1,     1 },
  { "RS_E2",      'L', 'A', OFTString,    36,  40,   5,       1,   1,     1 },
  { "PLACEEC",    'L', 'N', OFTInteger,   41,  45,   5,       1,   1,     1 },
  { "RS-E3",      'L', 'A', OFTString,    46,  50,   5,       1,   1,     1 },
  { "RS-E4",      'L', 'A', OFTString,    51,  54,   4,       1,   1,     1 },
  { "RS-E5",      'L', 'A', OFTString,    55,  55,   1,       1,   1,     1 },
  { "COMMREGEC",  'L', 'N', OFTInteger,   56,  56,   1,       1,   1,     1 },
  { "RS_E6",      'L', 'A', OFTString,    57,  73,  17,       1,   1,     1 }
};
static TigerRecordInfo rtE_info =
  {
    rtE_fields,
    sizeof(rtE_fields) / sizeof(TigerFieldInfo),
    73
  };

/************************************************************************/
/*                           TigerPolygonEconomic()                           */
/************************************************************************/

TigerPolygonEconomic::TigerPolygonEconomic( OGRTigerDataSource * poDSIn,
                              const char * pszPrototypeModule )

{
    OGRFieldDefn        oField("",OFTInteger);

    poDS = poDSIn;
    poFeatureDefn = new OGRFeatureDefn( "PolygonEconomic" );
    poFeatureDefn->Reference();
    poFeatureDefn->SetGeomType( wkbNone );

    psRTEInfo = &rtE_info;

    /* -------------------------------------------------------------------- */
    /*      Fields from type E record.                                      */
    /* -------------------------------------------------------------------- */

    AddFieldDefns( psRTEInfo, poFeatureDefn );
}

/************************************************************************/
/*                           ~TigerPolygonEconomic()                           */
/************************************************************************/

TigerPolygonEconomic::~TigerPolygonEconomic()

{
}

/************************************************************************/
/*                             SetModule()                              */
/************************************************************************/

int TigerPolygonEconomic::SetModule( const char * pszModule )

{
    if( !OpenFile( pszModule, FILE_CODE ) )
        return FALSE;

    EstablishFeatureCount();
    
    return TRUE;
}

/************************************************************************/
/*                             GetFeature()                             */
/************************************************************************/

OGRFeature *TigerPolygonEconomic::GetFeature( int nRecordId )

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

    if( VSIFRead( achRecord, psRTEInfo->nRecordLength, 1, fpPrimary ) != 1 )
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

    SetFields( psRTEInfo, poFeature, achRecord );

    return poFeature;
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/

OGRErr TigerPolygonEconomic::CreateFeature( OGRFeature *poFeature )

{
    char        szRecord[OGR_TIGER_RECBUF_LEN];

    if( !SetWriteModule( FILE_CODE, psRTEInfo->nRecordLength+2, poFeature ) )
        return OGRERR_FAILURE;

    memset( szRecord, ' ', psRTEInfo->nRecordLength );

    WriteFields( psRTEInfo, poFeature, szRecord);

    WriteRecord( szRecord, psRTEInfo->nRecordLength, FILE_CODE );

    return OGRERR_NONE;
}
