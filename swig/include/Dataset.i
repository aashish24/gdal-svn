/******************************************************************************
 * $Id$
 *
 * Name:     Dataset.i
 * Project:  GDAL Python Interface
 * Purpose:  GDAL Core SWIG Interface declarations.
 * Author:   Kevin Ruland, kruland@ku.edu
 *
 ******************************************************************************
 * Copyright (c) 2005, Kevin Ruland
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
 *****************************************************************************/

#if !defined(SWIGJAVA)
%{

static
CPLErr DSReadRaster_internal( GDALDatasetShadow *obj, 
                            int xoff, int yoff, int xsize, int ysize,
                            int buf_xsize, int buf_ysize,
                            GDALDataType buf_type,
                            int *buf_size, char **buf,
                            int band_list, int *pband_list )
{
  CPLErr result;
  *buf_size = (size_t)buf_xsize * buf_ysize * (GDALGetDataTypeSize( buf_type ) / 8) * band_list;
  *buf = (char*) VSIMalloc3( buf_xsize, buf_ysize, (GDALGetDataTypeSize( buf_type ) / 8) * band_list );
  if (*buf)
  {
    result = GDALDatasetRasterIO(obj, GF_Read, xoff, yoff, xsize, ysize,
                                    (void*) *buf, buf_xsize, buf_ysize, buf_type,
                                    band_list, pband_list, 0, 0, 0 );
    if ( result != CE_None ) {
        free( *buf );
        *buf = 0;
        *buf_size = 0;
    }
  }
  else
  {
    result = CE_Failure;
    *buf = 0;
    *buf_size = 0;
  }
  return result;
}
%}
#endif

//************************************************************************
//
// Define the extensions for Dataset (nee GDALDatasetShadow)
//
//************************************************************************

%rename (Dataset) GDALDatasetShadow;

class GDALDatasetShadow : public GDALMajorObjectShadow {
private:
  GDALDatasetShadow();
public:
%extend {

%immutable;
  int RasterXSize;
  int RasterYSize;
  int RasterCount;
//
// Needed
// _band list?
%mutable;

  ~GDALDatasetShadow() {
    if ( GDALDereferenceDataset( self ) <= 0 ) {
      GDALClose(self);
    }
  }

  GDALDriverShadow* GetDriver() {
    return (GDALDriverShadow*) GDALGetDatasetDriver( self );
  }

  GDALRasterBandShadow* GetRasterBand(int nBand ) {
    return (GDALRasterBandShadow*) GDALGetRasterBand( self, nBand );
  }

  char const *GetProjection() {
    return GDALGetProjectionRef( self );
  }

  char const *GetProjectionRef() {
    return GDALGetProjectionRef( self );
  }

  CPLErr SetProjection( char const *prj ) {
    return GDALSetProjection( self, prj );
  }

  void GetGeoTransform( double argout[6] ) {
    if ( GDALGetGeoTransform( self, argout ) != 0 ) {
      argout[0] = 0.0;
      argout[1] = 1.0;
      argout[2] = 0.0;
      argout[3] = 0.0;
      argout[4] = 0.0;
      argout[5] = 1.0;
    }
  }

  CPLErr SetGeoTransform( double argin[6] ) {
    return GDALSetGeoTransform( self, argin );
  }

  // The (int,int*) arguments are typemapped.  The name of the first argument
  // becomes the kwarg name for it.
#ifndef SWIGCSHARP  
#ifndef SWIGJAVA
%feature("kwargs") BuildOverviews;
#endif
%apply (int nList, int* pList) { (int overviewlist, int *pOverviews) };
#else
%apply (void *buffer_ptr) {int *pOverviews};
#endif
#ifdef SWIGJAVA
%apply (const char* stringWithDefaultValue) {const char* resampling};
  int BuildOverviews( const char *resampling,
                      int overviewlist, int *pOverviews,
                      GDALProgressFunc callback = NULL,
                      void* callback_data=NULL ) {
#else
  int BuildOverviews( const char *resampling = "NEAREST",
                      int overviewlist = 0 , int *pOverviews = 0,
                      GDALProgressFunc callback = NULL,
                      void* callback_data=NULL ) {
#endif
    return GDALBuildOverviews(  self, 
                                resampling ? resampling : "NEAREST", 
                                overviewlist, 
                                pOverviews, 
                                0, 
                                0, 
                                callback, 
                                callback_data);
  }
#ifndef SWIGCSHARP
%clear (int overviewlist, int *pOverviews);
#else
%clear (int *pOverviews);
#endif
#ifdef SWIGJAVA
%clear (const char *resampling);
#endif

  int GetGCPCount() {
    return GDALGetGCPCount( self );
  }

  const char *GetGCPProjection() {
    return GDALGetGCPProjection( self );
  }
  
#ifndef SWIGCSHARP
  void GetGCPs( int *nGCPs, GDAL_GCP const **pGCPs ) {
    *nGCPs = GDALGetGCPCount( self );
    *pGCPs = GDALGetGCPs( self );
  }

  CPLErr SetGCPs( int nGCPs, GDAL_GCP const *pGCPs, const char *pszGCPProjection ) {
    return GDALSetGCPs( self, nGCPs, pGCPs, pszGCPProjection );
  }

#endif

  void FlushCache() {
    GDALFlushCache( self );
  }

#ifndef SWIGJAVA
%feature ("kwargs") AddBand;
#endif
/* uses the defined char **options typemap */
  CPLErr AddBand( GDALDataType datatype = GDT_Byte, char **options = 0 ) {
    return GDALAddBand( self, datatype, options );
  }

  CPLErr CreateMaskBand( int nFlags ) {
      return GDALCreateDatasetMaskBand( self, nFlags );
  }

#if defined(SWIGPYTHON) || defined (SWIGJAVA)
%apply (char **out_ppsz_and_free) {char **};
#else
/*  this is a required typemap (hi, Python and Java guys!) returned list is copied and CSLDestroy'ed */
%apply (char **CSL) {char **};
#endif
  char **GetFileList() {
    return GDALGetFileList( self );
  }
%clear char **;

#if !defined(SWIGCSHARP) && !defined(SWIGJAVA)
%feature("kwargs") WriteRaster;
%apply (int nLen, char *pBuf) { (int buf_len, char *buf_string) };
%apply (int *optional_int) { (int*) };
%apply (int *optional_int) { (GDALDataType *buf_type) };
%apply (int nList, int *pList ) { (int band_list, int *pband_list ) };
  CPLErr WriteRaster( int xoff, int yoff, int xsize, int ysize,
	              int buf_len, char *buf_string,
                      int *buf_xsize = 0, int *buf_ysize = 0,
                      GDALDataType *buf_type = 0,
                      int band_list = 0, int *pband_list = 0 ) {
    int nxsize = (buf_xsize==0) ? xsize : *buf_xsize;
    int nysize = (buf_ysize==0) ? ysize : *buf_ysize;
    GDALDataType ntype;
    if ( buf_type != 0 ) {
      ntype = (GDALDataType) *buf_type;
    } else {
      int lastband = GDALGetRasterCount( self ) - 1;
      ntype = GDALGetRasterDataType( GDALGetRasterBand( self, lastband ) );
    }
    bool myBandList = false;
    int nBandCount;
    int *pBandList;
    if ( band_list != 0 ) {
      myBandList = false;
      nBandCount = band_list;
      pBandList = pband_list;
    }
    else {
      myBandList = true;
      nBandCount = GDALGetRasterCount( self );
      pBandList = (int*) CPLMalloc( sizeof(int) * nBandCount );
      for( int i = 0; i< nBandCount; ++i ) {
        pBandList[i] = i;
      }
    }
    return GDALDatasetRasterIO( self, GF_Write, xoff, yoff, xsize, ysize,
                                (void*) buf_string, nxsize, nysize, ntype,
                                band_list, pband_list, 0, 0, 0 );
    if ( myBandList ) {
       CPLFree( pBandList );
    }
  }
%clear (int band_list, int *pband_list );
%clear (GDALDataType *buf_type);
%clear (int*);
%clear (int buf_len, char *buf_string);
#endif

#if !defined(SWIGCSHARP) && !defined(SWIGJAVA)
%feature("kwargs") ReadRaster;
%apply (int *optional_int) { (GDALDataType *buf_type) };
%apply (int nList, int *pList ) { (int band_list, int *pband_list ) };
%apply ( int *nLen, char **pBuf ) { (int *buf_len, char **buf ) };
%apply ( int *optional_int ) {(int*)};                            
CPLErr ReadRaster(  int xoff, int yoff, int xsize, int ysize,
	              int *buf_len, char **buf,
                      int *buf_xsize = 0, int *buf_ysize = 0,
                      GDALDataType *buf_type = 0,
                      int band_list = 0, int *pband_list = 0  )
{

    int nxsize = (buf_xsize==0) ? xsize : *buf_xsize;
    int nysize = (buf_ysize==0) ? ysize : *buf_ysize;
    GDALDataType ntype;
    if ( buf_type != 0 ) {
      ntype = (GDALDataType) *buf_type;
    } else {
      int lastband = GDALGetRasterCount( self ) - 1;
      ntype = GDALGetRasterDataType( GDALGetRasterBand( self, lastband ) );
    }
    bool myBandList = false;
    int nBandCount;
    int *pBandList;
    if ( band_list != 0 ) {
      myBandList = false;
      nBandCount = band_list;
      pBandList = pband_list;
    }
    else {
      myBandList = true;
      nBandCount = GDALGetRasterCount( self );
      pBandList = (int*) CPLMalloc( sizeof(int) * nBandCount );
      for( int i = 0; i< nBandCount; ++i ) {
        pBandList[i] = i;
      }
    }
                            
    return DSReadRaster_internal( self, xoff, yoff, xsize, ysize,
                                nxsize, nysize, ntype,
                                buf_len, buf, 
                                nBandCount, pBandList);
    if ( myBandList ) {
       CPLFree( pBandList );
    }

}
  
%clear (int *buf_len, char **buf );
%clear (int*);
#endif


/* NEEDED */
/* GetSubDatasets */
/* ReadAsArray */
/* AddBand */
/* AdviseRead */
/* ReadRaster */
  
} /* extend */
}; /* GDALDatasetShadow */

%{
int GDALDatasetShadow_RasterXSize_get( GDALDatasetShadow *h ) {
  return GDALGetRasterXSize( h );
}
int GDALDatasetShadow_RasterYSize_get( GDALDatasetShadow *h ) {
  return GDALGetRasterYSize( h );
}
int GDALDatasetShadow_RasterCount_get( GDALDatasetShadow *h ) {
  return GDALGetRasterCount( h );
}
%}
