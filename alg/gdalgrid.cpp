/******************************************************************************
 * $Id$
 *
 * Project:  GDAL Gridding API.
 * Purpose:  Implementation of GDAL scattered data gridder.
 * Author:   Andrey Kiselev, dron@ak4719.spb.edu
 *
 ******************************************************************************
 * Copyright (c) 2007, Andrey Kiselev <dron@ak4719.spb.edu>
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

#include "gdalgrid.h"

CPL_CVSID("$Id$");

#define TO_RADIANS (3.14159265358979323846 / 180.0)

/************************************************************************/
/*                   GDALGridInverseDistanceToAPower()                  */
/************************************************************************/

/**
 * Inverse distance to a power.
 *
 * The Inverse Distance to a Power gridding method is a weighted average
 * interpolator. You should supply the input arrays with the scattered data
 * values including coordinates of every data point and output grid geometry.
 * The function will compute interpolated value for the given position in
 * output grid.
 *
 * For every grid node the resulting value \f$Z\f$ will be calculated using
 * formula:
 *
 * \f[
 *      Z=\frac{\sum_{i=1}^n{\frac{Z_i}{r_i^p}}}{\sum_{i=1}^n{\frac{1}{r_i^p}}}
 * \f]
 *
 *  where 
 *  <ul>
 *      <li> \f$Z_i\f$ is a known value at point \f$i\f$,
 *      <li> \f$r\f$ is a distance from the grid node to point \f$i\f$,
 *      <li> \f$p\f$ is a weighting power,
 *      <li> \f$n\f$ is a number of points in search ellipse.
 *  </ul>
 *
 *  In this method the weighting factor \f$w\f$ is
 *
 *  \f[
 *      w=\frac{1}{r^p}
 *  \f]
 *
 * @param poOptions Algorithm parameters. This should point to
 * GDALGridInverseDistanceToAPowerOptions object. 
 * @param nPoints Number of elements in input arrays.
 * @param padfX Input array of X coordinates. 
 * @param padfY Input array of Y coordinates. 
 * @param padfZ Input array of Z values. 
 * @param dfXPoint X coordinate of the point to compute.
 * @param dfYPoint Y coordinate of the point to compute.
 * @param nXPoint X position of the point to compute.  
 * @param nYPoint Y position of the point to compute.
 * @param pdfValue Pointer to variable where the computed grid node value
 * will be returned.
 *
 * @return CE_None on success or CE_Failure if something goes wrong.
 */

CPLErr
GDALGridInverseDistanceToAPower( const void *poOptions, GUInt32 nPoints,
                                 const double *padfX, const double *padfY,
                                 const double *padfZ,
                                 double dfXPoint, double dfYPoint,
                                 double *pdfValue )
{
    // TODO: For optimization purposes pre-computed parameters should be moved
    // out of this routine to the calling function.

    // Pre-compute search ellipse parameters
    double      dfRadius1 =
        ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->dfRadius1;
    double      dfRadius2 =
        ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->dfRadius2;
    double  dfR12;

    dfRadius1 *= dfRadius1;
    dfRadius2 *= dfRadius2;
    dfR12 = dfRadius1 * dfRadius2;

    // Compute coefficients for coordinate system rotation.
    double      dfCoeff1 = 0.0, dfCoeff2 = 0.0;
    const double dfAngle = TO_RADIANS
        * ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->dfAngle;
    const bool  bRotated = ( dfAngle == 0.0 ) ? false : true;
    if ( bRotated )
    {
        dfCoeff1 = cos(dfAngle);
        dfCoeff2 = sin(dfAngle);
    }

    const double    dfPower =
        ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->dfPower;
    const double    dfSmoothing =
        ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->dfSmoothing;
    const GUInt32   nMaxPoints = 
        ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->nMaxPoints;
    double  dfNominator = 0.0, dfDenominator = 0.0;
    GUInt32 i, n = 0;

    for ( i = 0; i < nPoints; i++ )
    {
        double  dfRX = padfX[i] - dfXPoint;
        double  dfRY = padfY[i] - dfYPoint;
        const double dfR2 =
            dfRX * dfRX + dfRY * dfRY + dfSmoothing * dfSmoothing;

        if ( bRotated )
        {
            double dfRXRotated = dfRX * dfCoeff1 + dfRY * dfCoeff2;
            double dfRYRotated = dfRY * dfCoeff1 - dfRX * dfCoeff2;

            dfRX = dfRXRotated;
            dfRY = dfRYRotated;
        }

        // Is this point located inside the search ellipse?
        if ( dfRadius2 * dfRX * dfRX + dfRadius1 * dfRY * dfRY <= dfR12 )
        {
            if ( CPLIsEqual(dfR2, 0.0) )
            {
                (*pdfValue) = padfZ[i];
                return CE_None;
            }
            else
            {
                const double  dfW = pow( sqrt(dfR2), dfPower );
                dfNominator += padfZ[i] / dfW;
                dfDenominator += 1.0 / dfW;
                n++;
                if ( nMaxPoints > 0 && n > nMaxPoints )
                    break;
            }
        }
    }

    if ( n < ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->nMinPoints
         || dfDenominator == 0.0 )
    {
        (*pdfValue) =
            ((GDALGridInverseDistanceToAPowerOptions*)poOptions)->dfNoDataValue;
    }
    else
        (*pdfValue) = dfNominator / dfDenominator;

    return CE_None;
}

/************************************************************************/
/*              GDALGridInverseDistanceToAPowerNoSearch()               */
/************************************************************************/

/**
 * Inverse distance to a power for whole data set.
 *
 * This is somewhat optimized version of the Inverse Distance to a Power
 * method. It is used when the search ellips is not set. The algorithm and
 * parameters are the same as in GDALGridInverseDistanceToAPower(), but this
 * implementation works faster, because of no search.
 *
 * @see GDALGridInverseDistanceToAPower()
 */

CPLErr
GDALGridInverseDistanceToAPowerNoSearch( const void *poOptions, GUInt32 nPoints,
                                         const double *padfX, const double *padfY,
                                         const double *padfZ,
                                         double dfXPoint, double dfYPoint,
                                         double *pdfValue )
{
    const double    dfPower =
        ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->dfPower;
    const double    dfSmoothing =
        ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->dfSmoothing;
    double  dfNominator = 0.0, dfDenominator = 0.0;
    GUInt32 i;

    for ( i = 0; i < nPoints; i++ )
    {
        const double dfRX = padfX[i] - dfXPoint;
        const double dfRY = padfY[i] - dfYPoint;
        const double dfR2 =
            dfRX * dfRX + dfRY * dfRY + dfSmoothing * dfSmoothing;

        if ( CPLIsEqual(dfR2, 0.0) )
        {
            (*pdfValue) = padfZ[i];
            return CE_None;
        }
        else
        {
            const double dfW = pow( sqrt(dfR2), dfPower );
            dfNominator += padfZ[i] / dfW;
            dfDenominator += 1.0 / dfW;
        }
    }

    if ( dfDenominator == 0.0 )
    {
        (*pdfValue) =
            ((GDALGridInverseDistanceToAPowerOptions*)poOptions)->dfNoDataValue;
    }
    else
        (*pdfValue) = dfNominator / dfDenominator;

    return CE_None;
}
/************************************************************************/
/*                        GDALGridMovingAverage()                       */
/************************************************************************/

/**
 * Moving average.
 *
 * The Moving Average is a simple data averaging algorithm. It uses a moving
 * window of elliptic form to search values and averages all data points
 * within the window. Search ellipse can be rotated by specified angle, the
 * center of ellipse located at the grid node. Also the minimum number of data
 * points to average can be set, if there are not enough points in window, the
 * grid node considered empty and will be filled with specified NODATA value.
 *
 * Mathematically it can be expressed with the formula:
 *
 * \f[
 *      Z=\frac{\sum_{i=1}^n{Z_i}}{n}
 * \f]
 *
 *  where 
 *  <ul>
 *      <li> \f$Z\f$ is a resulting value at the grid node,
 *      <li> \f$Z_i\f$ is a known value at point \f$i\f$,
 *      <li> \f$n\f$ is a number of points in search ellipse.
 *  </ul>
 *
 * @param poOptions Algorithm parameters. This should point to
 * GDALGridMovingAverageOptions object. 
 * @param nPoints Number of elements in input arrays.
 * @param padfX Input array of X coordinates. 
 * @param padfY Input array of Y coordinates. 
 * @param padfZ Input array of Z values. 
 * @param dfXPoint X coordinate of the point to compute.
 * @param dfYPoint Y coordinate of the point to compute.
 * @param pdfValue Pointer to variable where the computed grid node value
 * will be returned.
 *
 * @return CE_None on success or CE_Failure if something goes wrong.
 */

CPLErr
GDALGridMovingAverage( const void *poOptions, GUInt32 nPoints,
                       const double *padfX, const double *padfY,
                       const double *padfZ,
                       double dfXPoint, double dfYPoint, double *pdfValue )
{
    // TODO: For optimization purposes pre-computed parameters should be moved
    // out of this routine to the calling function.

    // Pre-compute search ellipse parameters
    double  dfRadius1 = ((GDALGridMovingAverageOptions *)poOptions)->dfRadius1;
    double  dfRadius2 = ((GDALGridMovingAverageOptions *)poOptions)->dfRadius2;
    double  dfR12;

    dfRadius1 *= dfRadius1;
    dfRadius2 *= dfRadius2;
    dfR12 = dfRadius1 * dfRadius2;

    // Compute coefficients for coordinate system rotation.
    double      dfCoeff1 = 0.0, dfCoeff2 = 0.0;
    const double    dfAngle =
        TO_RADIANS * ((GDALGridMovingAverageOptions *)poOptions)->dfAngle;
    const bool  bRotated = ( dfAngle == 0.0 ) ? false : true;
    if ( bRotated )
    {
        dfCoeff1 = cos(dfAngle);
        dfCoeff2 = sin(dfAngle);
    }

    double  dfAccumulator = 0.0;
    GUInt32 i = 0, n = 0;

    while ( i < nPoints )
    {
        double  dfRX = padfX[i] - dfXPoint;
        double  dfRY = padfY[i] - dfYPoint;

        if ( bRotated )
        {
            double dfRXRotated = dfRX * dfCoeff1 + dfRY * dfCoeff2;
            double dfRYRotated = dfRY * dfCoeff1 - dfRX * dfCoeff2;

            dfRX = dfRXRotated;
            dfRY = dfRYRotated;
        }

        // Is this point located inside the search ellipse?
        if ( dfRadius2 * dfRX * dfRX + dfRadius1 * dfRY * dfRY <= dfR12 )
        {
            dfAccumulator += padfZ[i];
            n++;
        }

        i++;
    }

    if ( n < ((GDALGridMovingAverageOptions *)poOptions)->nMinPoints
         || n == 0 )
    {
        (*pdfValue) =
            ((GDALGridMovingAverageOptions *)poOptions)->dfNoDataValue;
    }
    else
        (*pdfValue) = dfAccumulator / n;

    return CE_None;
}

/************************************************************************/
/*                        GDALGridNearestNeighbor()                     */
/************************************************************************/

/**
 * Nearest neighbor.
 *
 * The Nearest Neighbor method doesn't perform any interpolation or smoothing,
 * it just takes the value of nearest point found in grid node search ellipse
 * and returns it as a result. If there are no points found, the specified
 * NODATA value will be returned.
 *
 * @param poOptions Algorithm parameters. This should point to
 * GDALGridNearestNeighborOptions object. 
 * @param nPoints Number of elements in input arrays.
 * @param padfX Input array of X coordinates. 
 * @param padfY Input array of Y coordinates. 
 * @param padfZ Input array of Z values. 
 * @param dfXPoint X coordinate of the point to compute.
 * @param dfYPoint Y coordinate of the point to compute.
 * @param pdfValue Pointer to variable where the computed grid node value
 * will be returned.
 *
 * @return CE_None on success or CE_Failure if something goes wrong.
 */

CPLErr
GDALGridNearestNeighbor( const void *poOptions, GUInt32 nPoints,
                         const double *padfX, const double *padfY,
                         const double *padfZ,
                         double dfXPoint, double dfYPoint, double *pdfValue )
{
    // TODO: For optimization purposes pre-computed parameters should be moved
    // out of this routine to the calling function.

    // Pre-compute search ellipse parameters
    double  dfRadius1 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius1;
    double  dfRadius2 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius2;
    double  dfR12;

    dfRadius1 *= dfRadius1;
    dfRadius2 *= dfRadius2;
    dfR12 = dfRadius1 * dfRadius2;

    // Compute coefficients for coordinate system rotation.
    double      dfCoeff1 = 0.0, dfCoeff2 = 0.0;
    const double    dfAngle =
        TO_RADIANS * ((GDALGridNearestNeighborOptions *)poOptions)->dfAngle;
    const bool  bRotated = ( dfAngle == 0.0 ) ? false : true;
    if ( bRotated )
    {
        dfCoeff1 = cos(dfAngle);
        dfCoeff2 = sin(dfAngle);
    }

    // If the nearest point will not be found, its value remains as NODATA.
    double      dfNearestValue =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfNoDataValue;
    // Nearest distance will be initialized with a largest ellipse semi-axis.
    // All nearest points should be located in this range.
    double      dfNearestR = MAX(dfRadius1, dfRadius2);
    GUInt32 i = 0;

    while ( i < nPoints )
    {
        double  dfRX = padfX[i] - dfXPoint;
        double  dfRY = padfY[i] - dfYPoint;

        if ( bRotated )
        {
            double dfRXRotated = dfRX * dfCoeff1 + dfRY * dfCoeff2;
            double dfRYRotated = dfRY * dfCoeff1 - dfRX * dfCoeff2;

            dfRX = dfRXRotated;
            dfRY = dfRYRotated;
        }

        // Is this point located inside the search ellipse?
        if ( dfRadius2 * dfRX * dfRX + dfRadius1 * dfRY * dfRY <= dfR12 )
        {
            const double    dfR2 = dfRX * dfRX + dfRY * dfRY;
            if ( dfNearestR == 0.0 || dfR2 < dfNearestR )
            {
                dfNearestR = dfR2;
                dfNearestValue = padfZ[i];
            }
        }

        i++;
    }

    (*pdfValue) = dfNearestValue;

    return CE_None;
}

/************************************************************************/
/*                      GDALGridDataMetricMinimum()                     */
/************************************************************************/

/**
 * Minimum data value (data metric).
 *
 * Minimum value found in grid node search ellipse. If there are no points
 * found, the specified NODATA value will be returned.
 *
 * \f[
 *      Z=\min{(Z_1,Z_2,\ldots,Z_n)}
 * \f]
 *
 *  where 
 *  <ul>
 *      <li> \f$Z\f$ is a resulting value at the grid node,
 *      <li> \f$Z_i\f$ is a known value at point \f$i\f$,
 *      <li> \f$n\f$ is a number of points in search ellipse.
 *  </ul>
 *
 * @param poOptions Algorithm parameters. This should point to
 * GDALGridDataMetricsOptions object. 
 * @param nPoints Number of elements in input arrays.
 * @param padfX Input array of X coordinates. 
 * @param padfY Input array of Y coordinates. 
 * @param padfZ Input array of Z values. 
 * @param dfXPoint X coordinate of the point to compute.
 * @param dfYPoint Y coordinate of the point to compute.
 * @param pdfValue Pointer to variable where the computed grid node value
 * will be returned.
 *
 * @return CE_None on success or CE_Failure if something goes wrong.
 */

CPLErr
GDALGridDataMetricMinimum( const void *poOptions, GUInt32 nPoints,
                           const double *padfX, const double *padfY,
                           const double *padfZ,
                           double dfXPoint, double dfYPoint, double *pdfValue )
{
    // TODO: For optimization purposes pre-computed parameters should be moved
    // out of this routine to the calling function.

    // Pre-compute search ellipse parameters
    double  dfRadius1 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius1;
    double  dfRadius2 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius2;
    double  dfR12;

    dfRadius1 *= dfRadius1;
    dfRadius2 *= dfRadius2;
    dfR12 = dfRadius1 * dfRadius2;

    // Compute coefficients for coordinate system rotation.
    double      dfCoeff1 = 0.0, dfCoeff2 = 0.0;
    const double dfAngle =
        TO_RADIANS * ((GDALGridNearestNeighborOptions *)poOptions)->dfAngle;
    const bool  bRotated = ( dfAngle == 0.0 ) ? false : true;
    if ( bRotated )
    {
        dfCoeff1 = cos(dfAngle);
        dfCoeff2 = sin(dfAngle);
    }

    double      dfMinimumValue=0.0;
    GUInt32     i = 0, n = 0;

    while ( i < nPoints )
    {
        double  dfRX = padfX[i] - dfXPoint;
        double  dfRY = padfY[i] - dfYPoint;

        if ( bRotated )
        {
            double dfRXRotated = dfRX * dfCoeff1 + dfRY * dfCoeff2;
            double dfRYRotated = dfRY * dfCoeff1 - dfRX * dfCoeff2;

            dfRX = dfRXRotated;
            dfRY = dfRYRotated;
        }

        // Is this point located inside the search ellipse?
        if ( dfRadius2 * dfRX * dfRX + dfRadius1 * dfRY * dfRY <= dfR12 )
        {
            if ( n )
            {
                if ( dfMinimumValue > padfZ[i] )
                    dfMinimumValue = padfZ[i];
            }
            else
                dfMinimumValue = padfZ[i];
            n++;
        }

        i++;
    }

    if ( n < ((GDALGridMovingAverageOptions *)poOptions)->nMinPoints
         || n == 0 )
    {
        (*pdfValue) =
            ((GDALGridMovingAverageOptions *)poOptions)->dfNoDataValue;
    }
    else
        (*pdfValue) = dfMinimumValue;

    return CE_None;
}

/************************************************************************/
/*                      GDALGridDataMetricMaximum()                     */
/************************************************************************/

/**
 * Maximum data value (data metric).
 *
 * Maximum value found in grid node search ellipse. If there are no points
 * found, the specified NODATA value will be returned.
 *
 * \f[
 *      Z=\max{(Z_1,Z_2,\ldots,Z_n)}
 * \f]
 *
 *  where 
 *  <ul>
 *      <li> \f$Z\f$ is a resulting value at the grid node,
 *      <li> \f$Z_i\f$ is a known value at point \f$i\f$,
 *      <li> \f$n\f$ is a number of points in search ellipse.
 *  </ul>
 *
 * @param poOptions Algorithm parameters. This should point to
 * GDALGridDataMetricsOptions object. 
 * @param nPoints Number of elements in input arrays.
 * @param padfX Input array of X coordinates. 
 * @param padfY Input array of Y coordinates. 
 * @param padfZ Input array of Z values. 
 * @param dfXPoint X coordinate of the point to compute.
 * @param dfYPoint Y coordinate of the point to compute.
 * @param pdfValue Pointer to variable where the computed grid node value
 * will be returned.
 *
 * @return CE_None on success or CE_Failure if something goes wrong.
 */

CPLErr
GDALGridDataMetricMaximum( const void *poOptions, GUInt32 nPoints,
                           const double *padfX, const double *padfY,
                           const double *padfZ,
                           double dfXPoint, double dfYPoint, double *pdfValue )
{
    // TODO: For optimization purposes pre-computed parameters should be moved
    // out of this routine to the calling function.

    // Pre-compute search ellipse parameters
    double  dfRadius1 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius1;
    double  dfRadius2 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius2;
    double  dfR12;

    dfRadius1 *= dfRadius1;
    dfRadius2 *= dfRadius2;
    dfR12 = dfRadius1 * dfRadius2;

    // Compute coefficients for coordinate system rotation.
    double      dfCoeff1 = 0.0, dfCoeff2 = 0.0;
    const double    dfAngle =
        TO_RADIANS * ((GDALGridNearestNeighborOptions *)poOptions)->dfAngle;
    const bool  bRotated = ( dfAngle == 0.0 ) ? false : true;
    if ( bRotated )
    {
        dfCoeff1 = cos(dfAngle);
        dfCoeff2 = sin(dfAngle);
    }

    double      dfMaximumValue=0.0;
    GUInt32     i = 0, n = 0;

    while ( i < nPoints )
    {
        double  dfRX = padfX[i] - dfXPoint;
        double  dfRY = padfY[i] - dfYPoint;

        if ( bRotated )
        {
            double dfRXRotated = dfRX * dfCoeff1 + dfRY * dfCoeff2;
            double dfRYRotated = dfRY * dfCoeff1 - dfRX * dfCoeff2;

            dfRX = dfRXRotated;
            dfRY = dfRYRotated;
        }

        // Is this point located inside the search ellipse?
        if ( dfRadius2 * dfRX * dfRX + dfRadius1 * dfRY * dfRY <= dfR12 )
        {
            if ( n )
            {
                if ( dfMaximumValue < padfZ[i] )
                    dfMaximumValue = padfZ[i];
            }
            else
                dfMaximumValue = padfZ[i];
            n++;
        }

        i++;
    }

    if ( n < ((GDALGridMovingAverageOptions *)poOptions)->nMinPoints
         || n == 0 )
    {
        (*pdfValue) =
            ((GDALGridMovingAverageOptions *)poOptions)->dfNoDataValue;
    }
    else
        (*pdfValue) = dfMaximumValue;

    return CE_None;
}

/************************************************************************/
/*                       GDALGridDataMetricRange()                      */
/************************************************************************/

/**
 * Data range (data metric).
 *
 * A difference between the minimum and maximum values found in grid node
 * search ellipse. If there are no points found, the specified NODATA
 * value will be returned.
 *
 * \f[
 *      Z=\max{(Z_1,Z_2,\ldots,Z_n)}-\min{(Z_1,Z_2,\ldots,Z_n)}
 * \f]
 *
 *  where 
 *  <ul>
 *      <li> \f$Z\f$ is a resulting value at the grid node,
 *      <li> \f$Z_i\f$ is a known value at point \f$i\f$,
 *      <li> \f$n\f$ is a number of points in search ellipse.
 *  </ul>
 *
 * @param poOptions Algorithm parameters. This should point to
 * GDALGridDataMetricsOptions object. 
 * @param nPoints Number of elements in input arrays.
 * @param padfX Input array of X coordinates. 
 * @param padfY Input array of Y coordinates. 
 * @param padfZ Input array of Z values. 
 * @param dfXPoint X coordinate of the point to compute.
 * @param dfYPoint Y coordinate of the point to compute.
 * @param pdfValue Pointer to variable where the computed grid node value
 * will be returned.
 *
 * @return CE_None on success or CE_Failure if something goes wrong.
 */

CPLErr
GDALGridDataMetricRange( const void *poOptions, GUInt32 nPoints,
                         const double *padfX, const double *padfY,
                         const double *padfZ,
                         double dfXPoint, double dfYPoint, double *pdfValue )
{
    // TODO: For optimization purposes pre-computed parameters should be moved
    // out of this routine to the calling function.

    // Pre-compute search ellipse parameters
    double  dfRadius1 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius1;
    double  dfRadius2 =
        ((GDALGridNearestNeighborOptions *)poOptions)->dfRadius2;
    double  dfR12;

    dfRadius1 *= dfRadius1;
    dfRadius2 *= dfRadius2;
    dfR12 = dfRadius1 * dfRadius2;

    // Compute coefficients for coordinate system rotation.
    double      dfCoeff1 = 0.0, dfCoeff2 = 0.0;
    const double    dfAngle =
        TO_RADIANS * ((GDALGridNearestNeighborOptions *)poOptions)->dfAngle;
    const bool  bRotated = ( dfAngle == 0.0 ) ? false : true;
    if ( bRotated )
    {
        dfCoeff1 = cos(dfAngle);
        dfCoeff2 = sin(dfAngle);
    }

    double      dfMaximumValue=0.0, dfMinimumValue=0.0;
    GUInt32     i = 0, n = 0;

    while ( i < nPoints )
    {
        double  dfRX = padfX[i] - dfXPoint;
        double  dfRY = padfY[i] - dfYPoint;

        if ( bRotated )
        {
            double dfRXRotated = dfRX * dfCoeff1 + dfRY * dfCoeff2;
            double dfRYRotated = dfRY * dfCoeff1 - dfRX * dfCoeff2;

            dfRX = dfRXRotated;
            dfRY = dfRYRotated;
        }

        // Is this point located inside the search ellipse?
        if ( dfRadius2 * dfRX * dfRX + dfRadius1 * dfRY * dfRY <= dfR12 )
        {
            if ( n )
            {
                if ( dfMinimumValue > padfZ[i] )
                    dfMinimumValue = padfZ[i];
                if ( dfMaximumValue < padfZ[i] )
                    dfMaximumValue = padfZ[i];
            }
            else
                dfMinimumValue = dfMaximumValue = padfZ[i];
            n++;
        }

        i++;
    }

    if ( n < ((GDALGridMovingAverageOptions *)poOptions)->nMinPoints
         || n == 0 )
    {
        (*pdfValue) =
            ((GDALGridMovingAverageOptions *)poOptions)->dfNoDataValue;
    }
    else
        (*pdfValue) = dfMaximumValue - dfMinimumValue;

    return CE_None;
}

/************************************************************************/
/*                            GDALGridCreate()                          */
/************************************************************************/

/**
 * Create regular grid from the scattered data.
 *
 * This fucntion takes the arrays of X and Y coordinates and corresponding Z
 * values as input and computes regular grid (or call it a raster) from these
 * scattered data. You should supply geometry and extent of the output grid
 * and allocate array sufficient to hold such a grid.
 *
 * @param eAlgorithm Gridding method. 
 * @param poOptions Options to control choosen gridding method.
 * @param nPoints Number of elements in input arrays.
 * @param padfX Input array of X coordinates. 
 * @param padfY Input array of Y coordinates. 
 * @param padfZ Input array of Z values. 
 * @param dfXMin Lowest X border of output grid.
 * @param dfXMax Highest X border of output grid.
 * @param dfYMin Lowest Y border of output grid.
 * @param dfYMax Highest Y border of output grid.
 * @param nXSize Number of columns in output grid.
 * @param nYSize Number of rows in output grid.
 * @param eType Data type of output array.  
 * @param pData Pointer to array where the computed grid will be stored.
 * @param pfnProgress a GDALProgressFunc() compatible callback function for
 * reporting progress or NULL.
 * @param pProgressArg argument to be passed to pfnProgress.  May be NULL.
 *
 * @return CE_None on success or CE_Failure if something goes wrong.
 */

CPLErr
GDALGridCreate( GDALGridAlgorithm eAlgorithm, const void *poOptions,
                GUInt32 nPoints,
                const double *padfX, const double *padfY, const double *padfZ,
                double dfXMin, double dfXMax, double dfYMin, double dfYMax,
                GUInt32 nXSize, GUInt32 nYSize, GDALDataType eType, void *pData,
                GDALProgressFunc pfnProgress, void *pProgressArg )
{
    CPLAssert( poOptions );
    CPLAssert( padfX );
    CPLAssert( padfY );
    CPLAssert( padfZ );
    CPLAssert( pData );
    
    if ( pfnProgress == NULL )
        pfnProgress = GDALDummyProgress;

    if ( nXSize == 0 || nYSize == 0 )
    {
        CPLError( CE_Failure, CPLE_IllegalArg,
                  "Output raster dimesions should have non-zero size." );
        return CE_Failure;
    }

    GDALGridFunction    pfnGDALGridMethod;

    switch ( eAlgorithm )
    {
        case GGA_InverseDistanceToAPower:
            if ( ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->
                 dfRadius1 == 0.0 &&
                 ((GDALGridInverseDistanceToAPowerOptions *)poOptions)->
                 dfRadius2 == 0.0 )
                pfnGDALGridMethod = GDALGridInverseDistanceToAPowerNoSearch;
            else
                pfnGDALGridMethod = GDALGridInverseDistanceToAPower;
            break;

        case GGA_MovingAverage:
            pfnGDALGridMethod = GDALGridMovingAverage;
            break;

        case GGA_NearestNeighbor:
            pfnGDALGridMethod = GDALGridNearestNeighbor;
            break;

        case GGA_MetricMinimum:
            pfnGDALGridMethod = GDALGridDataMetricMinimum;
            break;

        case GGA_MetricMaximum:
            pfnGDALGridMethod = GDALGridDataMetricMaximum;
            break;

        case GGA_MetricRange:
            pfnGDALGridMethod = GDALGridDataMetricRange;
            break;

        default:
            CPLError( CE_Failure, CPLE_IllegalArg,
                      "GDAL does not support gridding method %d", eAlgorithm );
	    return CE_Failure;
    }

    GUInt32 nXPoint, nYPoint;
    const double    dfDeltaX = ( dfXMax - dfXMin ) / nXSize;
    const double    dfDeltaY = ( dfYMax - dfYMin ) / nYSize;

    for ( nYPoint = 0; nYPoint < nYSize; nYPoint++ )
    {
        const double    dfYPoint = dfYMin + ( nYPoint + 0.5 ) * dfDeltaY;
        for ( nXPoint = 0; nXPoint < nXSize; nXPoint++ )
        {
            const double    dfXPoint = dfXMin + ( nXPoint + 0.5 ) * dfDeltaX;
            double          dfValue = 0.0;
            if ( (*pfnGDALGridMethod)( poOptions, nPoints, padfX, padfY, padfZ,
                                       dfXPoint, dfYPoint,
                                       &dfValue ) != CE_None )
            {
                CPLError( CE_Failure, CPLE_AppDefined,
                          "Gridding failed at X position %lu, Y position %lu",
                          (long unsigned int)nXPoint,
                          (long unsigned int)nYPoint );
                return CE_Failure;
            }

            if ( eType == GDT_Byte )
                ((GByte *)pData)[nYPoint * nXSize + nXPoint] = (GByte)dfValue;
            else if ( eType == GDT_UInt16 )
                ((GUInt16 *)pData)[nYPoint * nXSize + nXPoint] = (GUInt16)dfValue;
            else if ( eType == GDT_Int16 )
                ((GInt16 *)pData)[nYPoint * nXSize + nXPoint] = (GInt16)dfValue;
            else if ( eType == GDT_UInt32 )
                ((GUInt32 *)pData)[nYPoint * nXSize + nXPoint] = (GUInt32)dfValue;
            else if ( eType == GDT_Int32 )
                ((GInt32 *)pData)[nYPoint * nXSize + nXPoint] = (GInt32)dfValue;
            else if ( eType == GDT_Float32 )
                ((float *)pData)[nYPoint * nXSize + nXPoint] = (float)dfValue;
            else if ( eType == GDT_Float64 )
                ((double *)pData)[nYPoint * nXSize + nXPoint] = dfValue;
        }

	if( !pfnProgress( (double)(nYPoint + 1) / nYSize, NULL, pProgressArg ) )
	{
	    CPLError( CE_Failure, CPLE_UserInterrupt, "User terminated" );
	    return CE_Failure;
	}
    }

    return CE_None;
}

