/******************************************************************************
 * $Id$
 *
 * Name:     Operations.i
 * Project:  GDAL Python Interface
 * Purpose:  GDAL Raster Operations SWIG Interface declarations.
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2007, Howard Butler
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

/************************************************************************/
/*                            TermProgress()                            */
/************************************************************************/

#ifndef SWIGCSHARP
%rename (TermProgress_nocb) GDALTermProgress_nocb;
%feature( "kwargs" ) GDALTermProgress_nocb;
%inline %{
int GDALTermProgress_nocb( double dfProgress, const char * pszMessage=NULL, void *pData=NULL ) {
  return GDALTermProgress( dfProgress, pszMessage, pData);
}
%}

%rename (TermProgress) GDALTermProgress;
%callback("%s");
int GDALTermProgress( double, const char *, void * );
%nocallback;
#endif


/************************************************************************/
/*                        ComputeMedianCutPCT()                         */
/************************************************************************/
%feature( "kwargs" ) ComputeMedianCutPCT;
%inline %{
int  ComputeMedianCutPCT ( GDALRasterBandShadow *red,
                              GDALRasterBandShadow *green,
                              GDALRasterBandShadow *blue,
                              int num_colors,
                              GDALColorTableShadow* colors,
                              GDALProgressFunc callback = NULL,
                              void* callback_data=NULL) {

    CPLErrorReset();

    int err = GDALComputeMedianCutPCT( red,
                                          green,
                                          blue,
                                          NULL,
                                          num_colors,
                                          colors,
                                          callback,
                                          callback_data);
    
    return err;
}
%} 

/************************************************************************/
/*                           DitherRGB2PCT()                            */
/************************************************************************/
%feature( "kwargs" ) DitherRGB2PCT;
%inline %{
int  DitherRGB2PCT ( GDALRasterBandShadow *red,
                     GDALRasterBandShadow *green,
                     GDALRasterBandShadow *blue,
                     GDALRasterBandShadow *target,
                     GDALColorTableShadow *colors,
                     GDALProgressFunc callback = NULL,
                     void* callback_data=NULL) {

    CPLErrorReset();
    int err;
    err = GDALDitherRGB2PCT(  red,
                                  green,
                                  blue,
                                  target,
                                  colors,
                                  callback,
                                  callback_data);
    
    return err;
}
%}

%newobject ReprojectImage;
%inline %{
CPLErr  ReprojectImage ( GDALDatasetShadow *src_ds,
                         GDALDatasetShadow *dst_ds,
                         const char *src_wkt=NULL,
                         const char *dst_wkt=NULL,
                         GDALResampleAlg eResampleAlg=GRA_NearestNeighbour,
                         double WarpMemoryLimit=0.0,
                         double maxerror = 0.0) {

    CPLErrorReset();

    CPLErr err = GDALReprojectImage( src_ds,
                                     src_wkt,
                                     dst_ds,
                                     dst_wkt,
                                     eResampleAlg,
                                     WarpMemoryLimit,
                                     maxerror,
                                     NULL,
                                     NULL,
                                     NULL);
    
    return err;
}
%} 

/************************************************************************/
/*                          ComputeProximity()                          */
/************************************************************************/

%feature( "kwargs" ) ComputeProximity;
%inline %{
int  ComputeProximity( GDALRasterBandShadow *srcBand,
                       GDALRasterBandShadow *proximityBand,
                       char **options = NULL,
                       GDALProgressFunc callback=NULL,
                       void* callback_data=NULL) {

    CPLErrorReset();

    return GDALComputeProximity( srcBand, proximityBand, options,
                                 callback, callback_data );
}
%} 

/************************************************************************/
/*                        RegenerateOverviews()                         */
/************************************************************************/

%feature( "kwargs" ) RegenerateOverviews;
%apply (int object_list_count, GDALRasterBandShadow **poObjects) {(int overviewBandCount, GDALRasterBandShadow **overviewBands)};
%inline %{
int  RegenerateOverviews( GDALRasterBandShadow *srcBand,
     			  int overviewBandCount,
                          GDALRasterBandShadow **overviewBands,
                          char *resampling = "average",
                          GDALProgressFunc callback=NULL,
                          void* callback_data=NULL) {

    CPLErrorReset();

    return GDALRegenerateOverviews( srcBand, overviewBandCount, overviewBands,
    	   			    resampling, callback, callback_data );
}
%} 

/************************************************************************/
/*                         RegenerateOverview()                         */
/************************************************************************/

%feature( "kwargs" ) RegenerateOverview;
%inline %{
int  RegenerateOverview( GDALRasterBandShadow *srcBand,
                          GDALRasterBandShadow *overviewBand,
                          char *resampling,
                          GDALProgressFunc callback=NULL,
                          void* callback_data=NULL) {

    CPLErrorReset();

    return GDALRegenerateOverviews( srcBand, 1, &overviewBand,
    	   			    resampling, callback, callback_data );
}
%} 

/************************************************************************/
/*                        AutoCreateWarpedVRT()                         */
/************************************************************************/

%newobject AutoCreateWarpedVRT;
%inline %{
GDALDatasetShadow *AutoCreateWarpedVRT( GDALDatasetShadow *src_ds,
                                        const char *src_wkt = 0,
                                        const char *dst_wkt = 0,
                                        GDALResampleAlg eResampleAlg = GRA_NearestNeighbour,
                                        double maxerror = 0.0 ) {
  GDALDatasetShadow *ds = GDALAutoCreateWarpedVRT( src_ds, src_wkt,
                                                   dst_wkt,
                                                   eResampleAlg,
                                                   maxerror,
                                                   0 );
  if (ds == 0) {
    /*throw CPLGetLastErrorMsg(); causes a SWIG_exception later*/
  }
  return ds;
  
}
%}

/************************************************************************/
/*                             Transformer                              */
/************************************************************************/

%rename (Transformer) GDALTransformerInfoShadow;
class GDALTransformerInfoShadow {
private:
  GDALTransformerInfoShadow();
public:
%extend {

  GDALTransformerInfoShadow( GDALDatasetShadow *src, GDALDatasetShadow *dst,
                             char **options ) {
    GDALTransformerInfoShadow *obj = (GDALTransformerInfoShadow*) 
       GDALCreateGenImgProjTransformer2( (GDALDatasetH)src, (GDALDatasetH)dst, 
                                         options );
    return obj;
  }

  ~GDALTransformerInfoShadow() {
    GDALDestroyTransformer( self );
  }

// Need to apply argin typemap second so the numinputs=1 version gets applied
// instead of the numinputs=0 version from argout.
%apply (double argout[ANY]) {(double inout[3])};
%apply (double argin[ANY]) {(double inout[3])};
  int TransformPoint( int bDstToSrc, double inout[3] ) {
    int nRet, nSuccess = TRUE;

    nRet = GDALUseTransformer( self, bDstToSrc, 
                               1, &inout[0], &inout[1], &inout[2], 
                               &nSuccess );

    return nRet && nSuccess;
  }
%clear (double inout[3]);

  int TransformPoint( double argout[3], int bDstToSrc, 
                      double x, double y, double z = 0.0 ) {
    int nRet, nSuccess = TRUE;
    
    argout[0] = x;
    argout[1] = y;
    argout[2] = z;
    nRet = GDALUseTransformer( self, bDstToSrc, 
                               1, &argout[0], &argout[1], &argout[2], 
                               &nSuccess );

    return nRet && nSuccess;
  }
  
#ifdef SWIGCSHARP
  %apply (double *inout) {(double*)};
  %apply (double *inout) {(int*)};
#endif
  int TransformPoints( int bDstToSrc, 
                       int nCount, double *x, double *y, double *z,
                       int *panSuccess ) {
    int nRet;

    nRet = GDALUseTransformer( self, bDstToSrc, nCount, x, y, z, panSuccess );

    return nRet;
  }
#ifdef SWIGCSHARP
  %clear (double*);
  %clear (int*);
#endif

} /*extend */
};
