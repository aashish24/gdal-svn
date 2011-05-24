/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Private definitions for OGR/SQLite driver.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2004, Frank Warmerdam <warmerdam@pobox.com>
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

#ifndef _OGR_SQLITE_H_INCLUDED
#define _OGR_SQLITE_H_INCLUDED

#include "ogrsf_frmts.h"
#include "cpl_error.h"

/* When used with Spatialite amalgation, there might be no sqlite3 headers */
/* in other places than /include/spatialite/ subdir */
#ifdef HAVE_SPATIALITE
#include <spatialite/sqlite3.h>
#else
#include "sqlite3.h"
#endif

/************************************************************************/
/*      Format used to store geometry data in the database.             */
/************************************************************************/

enum OGRSQLiteGeomFormat
{
    OSGF_None = 0,
    OSGF_WKT = 1,
    OSGF_WKB = 2,
    OSGF_FGF = 3,
    OSGF_SpatiaLite = 4
};

/************************************************************************/
/*      SpatiaLite's own Geometry type IDs.                             */
/************************************************************************/

enum OGRSpatialiteGeomType
{
// 2D [XY]
    OGRSplitePointXY                     = 1,
    OGRSpliteLineStringXY                = 2,
    OGRSplitePolygonXY                   = 3,
    OGRSpliteMultiPointXY                = 4,
    OGRSpliteMultiLineStringXY           = 5,
    OGRSpliteMultiPolygonXY              = 6,
    OGRSpliteGeometryCollectionXY        = 7,
// 3D [XYZ]
    OGRSplitePointXYZ                    = 1001,
    OGRSpliteLineStringXYZ               = 1002,
    OGRSplitePolygonXYZ                  = 1003,
    OGRSpliteMultiPointXYZ               = 1004,
    OGRSpliteMultiLineStringXYZ          = 1005,
    OGRSpliteMultiPolygonXYZ             = 1006,
    OGRSpliteGeometryCollectionXYZ       = 1007,
// 2D with Measure [XYM] 
    OGRSplitePointXYM                    = 2001,
    OGRSpliteLineStringXYM               = 2002,
    OGRSplitePolygonXYM                  = 2003,
    OGRSpliteMultiPointXYM               = 2004,
    OGRSpliteMultiLineStringXYM          = 2005,
    OGRSpliteMultiPolygonXYM             = 2006,
    OGRSpliteGeometryCollectionXYM       = 2007,
// 3D with Measure [XYZM]
    OGRSplitePointXYZM                   = 3001,
    OGRSpliteLineStringXYZM              = 3002,
    OGRSplitePolygonXYZM                 = 3003,
    OGRSpliteMultiPointXYZM              = 3004,
    OGRSpliteMultiLineStringXYZM         = 3005,
    OGRSpliteMultiPolygonXYZM            = 3006,
    OGRSpliteGeometryCollectionXYZM      = 3007,
// COMPRESSED: 2D [XY]
    OGRSpliteComprLineStringXY           = 1000002,
    OGRSpliteComprPolygonXY              = 1000003,
    OGRSpliteComprMultiPointXY           = 1000004,
    OGRSpliteComprMultiLineStringXY      = 1000005,
    OGRSpliteComprMultiPolygonXY         = 1000006,
    OGRSpliteComprGeometryCollectionXY   = 1000007,
// COMPRESSED: 3D [XYZ]
    OGRSpliteComprLineStringXYZ          = 1001002,
    OGRSpliteComprPolygonXYZ             = 1001003,
    OGRSpliteComprMultiPointXYZ          = 1001004,
    OGRSpliteComprMultiLineStringXYZ     = 1001005,
    OGRSpliteComprMultiPolygonXYZ        = 1001006,
    OGRSpliteComprGeometryCollectionXYZ  = 1001007,
// COMPRESSED: 2D with Measure [XYM]
    OGRSpliteComprLineStringXYM          = 1002002,
    OGRSpliteComprPolygonXYM             = 1002003,
    OGRSpliteComprMultiPointXYM          = 1002004,
    OGRSpliteComprMultiLineStringXYM     = 1002005,
    OGRSpliteComprMultiPolygonXYM        = 1002006,
    OGRSpliteComprGeometryCollectionXYM  = 1002007,
// COMPRESSED: 3D with Measure [XYZM]
    OGRSpliteComprLineStringXYZM         = 1003002,
    OGRSpliteComprPolygonXYZM            = 1003003,
    OGRSpliteComprMultiPointXYZM         = 1003004,
    OGRSpliteComprMultiLineStringXYZM    = 1003005,
    OGRSpliteComprMultiPolygonXYZM       = 1003006,
    OGRSpliteComprGeometryCollectionXYZM = 1003007
};

/************************************************************************/
/*                            OGRSQLiteLayer                            */
/************************************************************************/

class OGRSQLiteDataSource;
    
class OGRSQLiteLayer : public OGRLayer
{
  private:
    static OGRErr       createFromSpatialiteInternal(const GByte *pabyData,
                                                     OGRGeometry **ppoReturn,
                                                     int nBytes,
                                                     OGRwkbByteOrder eByteOrder,
                                                     int* pnBytesConsumed);

    static int          ComputeSpatiaLiteGeometrySize(const OGRGeometry *poGeometry,
                                                      int bHasM );
    static int          ExportSpatiaLiteGeometryInternal(const OGRGeometry *poGeometry,
                                                        OGRwkbByteOrder eByteOrder,
                                                        int bHasM,
                                                        GByte* pabyData );

  protected:
    OGRFeatureDefn     *poFeatureDefn;

    // Layer spatial reference system, and srid.
    OGRSpatialReference *poSRS;
    int                 nSRSId;

    int                 iNextShapeId;

    sqlite3_stmt        *hStmt;

    OGRSQLiteDataSource *poDS;

    int                 bTriedAsSpatiaLite;
    CPLString           osGeomColumn;
    OGRSQLiteGeomFormat eGeomFormat;

    char                *pszFIDColumn;

    int                *panFieldOrdinals;
    int                 bHasSpatialIndex;
    int                 bHasM;
    int                 bSpatialiteReadOnly;
    int                 bSpatialiteLoaded;
    int                 iSpatialiteVersion;

    CPLErr              BuildFeatureDefn( const char *pszLayerName, 
                                          sqlite3_stmt *hStmt );

    void                ClearStatement();
    virtual OGRErr      ResetStatement() = 0;

    static OGRErr       ImportSpatiaLiteGeometry( const GByte *, int,
                                                  OGRGeometry ** );
    static OGRErr       ExportSpatiaLiteGeometry( const OGRGeometry *,
                                                  GInt32, OGRwkbByteOrder,
                                                  int, GByte **, int * );

  public:
                        OGRSQLiteLayer();
    virtual             ~OGRSQLiteLayer();

    virtual void        ResetReading();
    virtual OGRFeature *GetNextRawFeature();
    virtual OGRFeature *GetNextFeature();

    virtual OGRFeature *GetFeature( long nFeatureId );
    
    OGRFeatureDefn *    GetLayerDefn() { return poFeatureDefn; }

    virtual OGRSpatialReference *GetSpatialRef();

    virtual const char *GetFIDColumn();
    virtual const char *GetGeometryColumn();

    virtual int         TestCapability( const char * );

    virtual OGRErr       StartTransaction();
    virtual OGRErr       CommitTransaction();
    virtual OGRErr       RollbackTransaction();
};

/************************************************************************/
/*                         OGRSQLiteTableLayer                          */
/************************************************************************/

class OGRSQLiteTableLayer : public OGRSQLiteLayer
{
    int                 bUpdateAccess;
    int                 bLaunderColumnNames;

    CPLString           osWHERE;
    CPLString           osQuery;

    void                BuildWhere(void);

    OGRErr              ResetStatement();

  public:
                        OGRSQLiteTableLayer( OGRSQLiteDataSource * );
                        ~OGRSQLiteTableLayer();

    CPLErr              Initialize( const char *pszTableName, 
                                    const char *pszGeomCol,
                                    OGRwkbGeometryType eGeomType,
                                    const char *pszGeomFormat,
                                    OGRSpatialReference *poSRS,
                                    int nSRSId = -1,
                                    int bHasSpatialIndex = FALSE,
                                    int bHasM = FALSE,
                                    int bSpatialiteReadOnly = FALSE,
                                    int bSpatialiteLoaded = FALSE,
                                    int iSpatialiteVersion = -1 );

    virtual int         GetFeatureCount( int );

    virtual void        SetSpatialFilter( OGRGeometry * );
    virtual OGRErr      SetAttributeFilter( const char * );
    virtual OGRErr      SetFeature( OGRFeature *poFeature );
    virtual OGRErr      CreateFeature( OGRFeature *poFeature );

    virtual OGRErr      CreateField( OGRFieldDefn *poField,
                                     int bApproxOK = TRUE );
    virtual OGRFeature *GetFeature( long nFeatureId );
    
    virtual OGRSpatialReference *GetSpatialRef();

    virtual int         TestCapability( const char * );

    // follow methods are not base class overrides
    void                SetLaunderFlag( int bFlag ) 
                                { bLaunderColumnNames = bFlag; }
};

/************************************************************************/
/*                         OGRSQLiteSelectLayer                         */
/************************************************************************/

class OGRSQLiteSelectLayer : public OGRSQLiteLayer
{
    CPLString           osSQL;

    OGRErr              ResetStatement();

  public:
                        OGRSQLiteSelectLayer( OGRSQLiteDataSource *, 
                                              CPLString osSQL,
                                              sqlite3_stmt * );
                        ~OGRSQLiteSelectLayer();
};

/************************************************************************/
/*                   OGRSQLiteSingleFeatureLayer                        */
/************************************************************************/

class OGRSQLiteSingleFeatureLayer : public OGRLayer
{
  private:
    int                 nVal;
    OGRFeatureDefn     *poFeatureDefn;
    int                 iNextShapeId;

  public:
                        OGRSQLiteSingleFeatureLayer( const char* pszLayerName,
                                                     int nVal );
                        ~OGRSQLiteSingleFeatureLayer();

    virtual void        ResetReading();
    virtual OGRFeature *GetNextFeature();
    virtual OGRFeatureDefn *GetLayerDefn();
    virtual int         TestCapability( const char * );
};

/************************************************************************/
/*                         OGRSQLiteDataSource                          */
/************************************************************************/

class OGRSQLiteDataSource : public OGRDataSource
{
    OGRSQLiteLayer    **papoLayers;
    int                 nLayers;
    
    char               *pszName;

    sqlite3             *hDB;

    int                 nSoftTransactionLevel;

    // We maintain a list of known SRID to reduce the number of trips to
    // the database to get SRSes. 
    int                 nKnownSRID;
    int                *panSRID;
    OGRSpatialReference **papoSRS;

    int                 bHaveGeometryColumns;
    int                 bIsSpatiaLite;
    
    virtual void        DeleteLayer( const char *pszLayer );

  public:
                        OGRSQLiteDataSource();
                        ~OGRSQLiteDataSource();

    int                 Open( const char * );
    int                 OpenTable( const char *pszTableName, 
                                   const char *pszGeomCol = NULL,
                                   OGRwkbGeometryType eGeomType = wkbUnknown,
                                   const char *pszGeomFormat = NULL,
                                   OGRSpatialReference *poSRS = NULL,
                                   int nSRID = -1,
                                   int bHasSpatialIndex = FALSE,
                                   int bHasM = FALSE,
                                   int bSpatialiteReadOnly = FALSE,
                                   int bSpatialiteLoaded = FALSE,
                                   int iSpatialiteVersion = -1 );

    const char          *GetName() { return pszName; }
    int                 GetLayerCount() { return nLayers; }
    OGRLayer            *GetLayer( int );
    
    virtual OGRLayer    *CreateLayer( const char *pszLayerName, 
                                      OGRSpatialReference *poSRS, 
                                      OGRwkbGeometryType eType, 
                                      char **papszOptions );
    virtual OGRErr      DeleteLayer(int);

    int                 TestCapability( const char * );

    virtual OGRLayer *  ExecuteSQL( const char *pszSQLCommand,
                                    OGRGeometry *poSpatialFilter,
                                    const char *pszDialect );
    virtual void        ReleaseResultSet( OGRLayer * poLayer );

    OGRErr              SoftStartTransaction();
    OGRErr              SoftCommit();
    OGRErr              SoftRollback();
    
    OGRErr              FlushSoftTransaction();

    sqlite3            *GetDB() { return hDB; }

    char               *LaunderName( const char * );
    int                 FetchSRSId( OGRSpatialReference * poSRS );
    OGRSpatialReference*FetchSRS( int nSRID );
};

/************************************************************************/
/*                           OGRSQLiteDriver                            */
/************************************************************************/

class OGRSQLiteDriver : public OGRSFDriver
{
  private:
    static int          InitWithEPSG(sqlite3* hDB, int bSpatialite);

  public:
                ~OGRSQLiteDriver();
                
    const char *GetName();
    OGRDataSource *Open( const char *, int );

    virtual OGRDataSource *CreateDataSource( const char *pszName,
                                             char ** = NULL );
    
    int                 TestCapability( const char * );
};


#endif /* ndef _OGR_SQLITE_H_INCLUDED */


