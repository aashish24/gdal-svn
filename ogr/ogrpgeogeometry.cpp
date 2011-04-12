/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements decoder of shapebin geometry for PGeo
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2005, Frank Warmerdam <warmerdam@pobox.com>
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

#include "ogrpgeogeometry.h"
#include "cpl_string.h"

CPL_CVSID("$Id$");


/************************************************************************/
/*                      OGRCreateFromShapeBin()                         */
/*                                                                      */
/*      Translate shapefile binary representation to an OGR             */
/*      geometry.                                                       */
/************************************************************************/

OGRErr OGRCreateFromShapeBin( GByte *pabyShape,
                              OGRGeometry **ppoGeom,
                              int nBytes )

{
    *ppoGeom = NULL;

    if( nBytes < 1 )
        return OGRERR_FAILURE;

    int nSHPType = pabyShape[0];


//    CPLDebug( "PGeo",
//              "Shape type read from PGeo data is nSHPType = %d",
//              nSHPType );

/* -------------------------------------------------------------------- */
/*      type 50 appears to just be an alias for normal line             */
/*      strings. (#1484)                                                */
/*      Type 51 appears to just be an alias for normal polygon. (#3100) */
/*      TODO: These types include additional attributes including       */
/*      non-linear segments and such. They should be handled.           */
/* -------------------------------------------------------------------- */
    switch( nSHPType )
    {
      case 50:
        nSHPType = SHPT_ARC;
        break;
      case 51:
        nSHPType = SHPT_POLYGON;
        break;
      case 52:
        nSHPType = SHPT_POINT;
        break;
      case 53:
        nSHPType = SHPT_MULTIPOINT;
        break;
      case 54:
        nSHPType = SHPT_MULTIPATCH;
    }

/* ==================================================================== */
/*  Extract vertices for a Polygon or Arc.              */
/* ==================================================================== */
    if(    nSHPType == SHPT_ARC
        || nSHPType == SHPT_ARCZ
        || nSHPType == SHPT_ARCM
        || nSHPType == SHPT_ARCZM
        || nSHPType == SHPT_POLYGON
        || nSHPType == SHPT_POLYGONZ
        || nSHPType == SHPT_POLYGONM
        || nSHPType == SHPT_POLYGONZM
        || nSHPType == SHPT_MULTIPATCH
        || nSHPType == SHPT_MULTIPATCHM)
    {
        GInt32         nPoints, nParts;
        int            i, nOffset;
        GInt32         *panPartStart;

        if (nBytes < 44)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Corrupted Shape : nBytes=%d, nSHPType=%d", nBytes, nSHPType);
            return OGRERR_FAILURE;
        }

/* -------------------------------------------------------------------- */
/*      Extract part/point count, and build vertex and part arrays      */
/*      to proper size.                                                 */
/* -------------------------------------------------------------------- */
    memcpy( &nPoints, pabyShape + 40, 4 );
    memcpy( &nParts, pabyShape + 36, 4 );

    CPL_LSBPTR32( &nPoints );
    CPL_LSBPTR32( &nParts );

        if (nPoints < 0 || nParts < 0 ||
            nPoints > 50 * 1000 * 1000 || nParts > 10 * 1000 * 1000)
        {
            CPLError(CE_Failure, CPLE_AppDefined, "Corrupted Shape : nPoints=%d, nParts=%d.",
                     nPoints, nParts);
            return OGRERR_FAILURE;
        }

        int bHasZ = (  nSHPType == SHPT_POLYGONZ
                    || nSHPType == SHPT_POLYGONZM
                    || nSHPType == SHPT_ARCZ
                    || nSHPType == SHPT_ARCZM
                    || nSHPType == SHPT_MULTIPATCH
                    || nSHPType == SHPT_MULTIPATCHM );

        int bIsMultiPatch = ( nSHPType == SHPT_MULTIPATCH || nSHPType == SHPT_MULTIPATCHM );

        /* With the previous checks on nPoints and nParts, */
        /* we should not overflow here and after */
        /* since 50 M * (16 + 8 + 8) = 1 600 MB */
        int nRequiredSize = 44 + 4 * nParts + 16 * nPoints;
        if ( bHasZ )
        {
            nRequiredSize += 16 + 8 * nPoints;
        }
        if( bIsMultiPatch )
        {
            nRequiredSize += 4 * nParts;
        }
        if (nRequiredSize > nBytes)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Corrupted Shape : nPoints=%d, nParts=%d, nBytes=%d, nSHPType=%d",
                     nPoints, nParts, nBytes, nSHPType);
            return OGRERR_FAILURE;
        }

        panPartStart = (GInt32 *) VSICalloc(nParts,sizeof(GInt32));
        if (panPartStart == NULL)
        {
            CPLError(CE_Failure, CPLE_OutOfMemory,
                     "Not enough memory for shape (nPoints=%d, nParts=%d)", nPoints, nParts);
            return OGRERR_FAILURE;
        }

/* -------------------------------------------------------------------- */
/*      Copy out the part array from the record.                        */
/* -------------------------------------------------------------------- */
    memcpy( panPartStart, pabyShape + 44, 4 * nParts );
    for( i = 0; i < nParts; i++ )
    {
            CPL_LSBPTR32( panPartStart + i );

            /* We check that the offset is inside the vertex array */
            if (panPartStart[i] < 0 ||
                panPartStart[i] >= nPoints)
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Corrupted Shape : panPartStart[%d] = %d, nPoints = %d",
                         i, panPartStart[i], nPoints);
                CPLFree(panPartStart);
                return OGRERR_FAILURE;
            }
            if (i > 0 && panPartStart[i] <= panPartStart[i-1])
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "Corrupted Shape : panPartStart[%d] = %d, panPartStart[%d] = %d",
                         i, panPartStart[i], i - 1, panPartStart[i - 1]);
                CPLFree(panPartStart);
                return OGRERR_FAILURE;
            }
    }

    nOffset = 44 + 4*nParts;

/* -------------------------------------------------------------------- */
/*      If this is a multipatch, we will also have parts types.  For    */
/*      now we ignore and skip past them.                               */
/* -------------------------------------------------------------------- */
        if( bIsMultiPatch )
            nOffset += 4*nParts;

/* -------------------------------------------------------------------- */
/*      Copy out the vertices from the record.                          */
/* -------------------------------------------------------------------- */
        double *padfX = (double *) VSIMalloc(sizeof(double)*nPoints);
        double *padfY = (double *) VSIMalloc(sizeof(double)*nPoints);
        double *padfZ = (double *) VSICalloc(sizeof(double),nPoints);
        if (padfX == NULL || padfY == NULL || padfZ == NULL)
        {
            CPLFree( panPartStart );
            CPLFree( padfX );
            CPLFree( padfY );
            CPLFree( padfZ );
            CPLError(CE_Failure, CPLE_OutOfMemory,
                     "Not enough memory for shape (nPoints=%d, nParts=%d)", nPoints, nParts);
            return OGRERR_FAILURE;
        }

    for( i = 0; i < nPoints; i++ )
    {
        memcpy(padfX + i, pabyShape + nOffset + i * 16, 8 );
        memcpy(padfY + i, pabyShape + nOffset + i * 16 + 8, 8 );
            CPL_LSBPTR64( padfX + i );
            CPL_LSBPTR64( padfY + i );
    }

        nOffset += 16*nPoints;

/* -------------------------------------------------------------------- */
/*      If we have a Z coordinate, collect that now.                    */
/* -------------------------------------------------------------------- */
        if( bHasZ )
        {
            for( i = 0; i < nPoints; i++ )
            {
                memcpy( padfZ + i, pabyShape + nOffset + 16 + i*8, 8 );
                CPL_LSBPTR64( padfZ + i );
            }

            nOffset += 16 + 8*nPoints;
        }

/* -------------------------------------------------------------------- */
/*      Build corresponding OGR objects.                                */
/* -------------------------------------------------------------------- */
        if(    nSHPType == SHPT_ARC
            || nSHPType == SHPT_ARCZ
            || nSHPType == SHPT_ARCM
            || nSHPType == SHPT_ARCZM )
        {
/* -------------------------------------------------------------------- */
/*      Arc - As LineString                                             */
/* -------------------------------------------------------------------- */
            if( nParts == 1 )
            {
                OGRLineString *poLine = new OGRLineString();
                *ppoGeom = poLine;

                poLine->setPoints( nPoints, padfX, padfY, padfZ );
            }

/* -------------------------------------------------------------------- */
/*      Arc - As MultiLineString                                        */
/* -------------------------------------------------------------------- */
            else
            {
                OGRMultiLineString *poMulti = new OGRMultiLineString;
                *ppoGeom = poMulti;

                for( i = 0; i < nParts; i++ )
                {
                    OGRLineString *poLine = new OGRLineString;
                    int nVerticesInThisPart;

                    if( i == nParts-1 )
                        nVerticesInThisPart = nPoints - panPartStart[i];
                    else
                        nVerticesInThisPart =
                            panPartStart[i+1] - panPartStart[i];

                    poLine->setPoints( nVerticesInThisPart,
                                       padfX + panPartStart[i],
                                       padfY + panPartStart[i],
                                       padfZ + panPartStart[i] );

                    poMulti->addGeometryDirectly( poLine );
                }
            }
        } /* ARC */

/* -------------------------------------------------------------------- */
/*      Polygon                                                         */
/* -------------------------------------------------------------------- */
        else if(    nSHPType == SHPT_POLYGON
                 || nSHPType == SHPT_POLYGONZ
                 || nSHPType == SHPT_POLYGONM
                 || nSHPType == SHPT_POLYGONZM )
        {
            if (nParts != 0)
            {
                if (nParts == 1)
                {
                    OGRPolygon *poOGRPoly = new OGRPolygon;
                    *ppoGeom = poOGRPoly;
                    OGRLinearRing *poRing = new OGRLinearRing;
                    int nVerticesInThisPart = nPoints - panPartStart[0];

                    poRing->setPoints( nVerticesInThisPart,
                                       padfX + panPartStart[0],
                                       padfY + panPartStart[0],
                                       padfZ + panPartStart[0] );

                    poOGRPoly->addRingDirectly( poRing );
                }
                else
                {
                    OGRGeometry *poOGR = NULL;
                    OGRPolygon** tabPolygons = new OGRPolygon*[nParts];

                    for( i = 0; i < nParts; i++ )
                    {
                        tabPolygons[i] = new OGRPolygon();
                        OGRLinearRing *poRing = new OGRLinearRing;
                        int nVerticesInThisPart;

                        if( i == nParts-1 )
                            nVerticesInThisPart = nPoints - panPartStart[i];
                        else
                            nVerticesInThisPart =
                                panPartStart[i+1] - panPartStart[i];

                        poRing->setPoints( nVerticesInThisPart,
                                           padfX + panPartStart[i],
                                           padfY + panPartStart[i],
                                           padfZ + panPartStart[i] );
                        tabPolygons[i]->addRingDirectly(poRing);
                    }

                    int isValidGeometry;
                    const char* papszOptions[] = { "METHOD=ONLY_CCW", NULL };
                    poOGR = OGRGeometryFactory::organizePolygons(
                        (OGRGeometry**)tabPolygons, nParts, &isValidGeometry, papszOptions );

                    if (!isValidGeometry)
                    {
                        CPLError(CE_Warning, CPLE_AppDefined,
                                 "Geometry of polygon cannot be translated to Simple Geometry. "
                                 "All polygons will be contained in a multipolygon.\n");
                    }

                    *ppoGeom = poOGR;
                    delete[] tabPolygons;
                }
            }
        } /* polygon */

/* -------------------------------------------------------------------- */
/*      Multipatch                                                      */
/* -------------------------------------------------------------------- */
        else if( bIsMultiPatch )
        {
            /* return to this later */
        }

        CPLFree( panPartStart );
        CPLFree( padfX );
        CPLFree( padfY );
        CPLFree( padfZ );

        if( !bHasZ )
            (*ppoGeom)->setCoordinateDimension( 2 );

        return OGRERR_NONE;
    }

/* ==================================================================== */
/*  Extract vertices for a MultiPoint.                  */
/* ==================================================================== */
    else if(    nSHPType == SHPT_MULTIPOINT
             || nSHPType == SHPT_MULTIPOINTM
             || nSHPType == SHPT_MULTIPOINTZ
             || nSHPType == SHPT_MULTIPOINTZM )
    {
#ifdef notdef
    int32       nPoints;
    int         i, nOffset;

    memcpy( &nPoints, psSHP->pabyRec + 44, 4 );
    if( bBigEndian ) SwapWord( 4, &nPoints );

    psShape->nVertices = nPoints;
        psShape->padfX = (double *) calloc(nPoints,sizeof(double));
        psShape->padfY = (double *) calloc(nPoints,sizeof(double));
        psShape->padfZ = (double *) calloc(nPoints,sizeof(double));
        psShape->padfM = (double *) calloc(nPoints,sizeof(double));

    for( i = 0; i < nPoints; i++ )
    {
        memcpy(psShape->padfX+i, psSHP->pabyRec + 48 + 16 * i, 8 );
        memcpy(psShape->padfY+i, psSHP->pabyRec + 48 + 16 * i + 8, 8 );

        if( bBigEndian ) SwapWord( 8, psShape->padfX + i );
        if( bBigEndian ) SwapWord( 8, psShape->padfY + i );
    }

        nOffset = 48 + 16*nPoints;

/* -------------------------------------------------------------------- */
/*  Get the X/Y bounds.                     */
/* -------------------------------------------------------------------- */
        memcpy( &(psShape->dfXMin), psSHP->pabyRec + 8 +  4, 8 );
        memcpy( &(psShape->dfYMin), psSHP->pabyRec + 8 + 12, 8 );
        memcpy( &(psShape->dfXMax), psSHP->pabyRec + 8 + 20, 8 );
        memcpy( &(psShape->dfYMax), psSHP->pabyRec + 8 + 28, 8 );

    if( bBigEndian ) SwapWord( 8, &(psShape->dfXMin) );
    if( bBigEndian ) SwapWord( 8, &(psShape->dfYMin) );
    if( bBigEndian ) SwapWord( 8, &(psShape->dfXMax) );
    if( bBigEndian ) SwapWord( 8, &(psShape->dfYMax) );

/* -------------------------------------------------------------------- */
/*      If we have a Z coordinate, collect that now.                    */
/* -------------------------------------------------------------------- */
        if( psShape->nSHPType == SHPT_MULTIPOINTZ || psShape->nSHPType == SHPT_MULTIPOINTZM )
        {
            memcpy( &(psShape->dfZMin), psSHP->pabyRec + nOffset, 8 );
            memcpy( &(psShape->dfZMax), psSHP->pabyRec + nOffset + 8, 8 );

            if( bBigEndian ) SwapWord( 8, &(psShape->dfZMin) );
            if( bBigEndian ) SwapWord( 8, &(psShape->dfZMax) );

            for( i = 0; i < nPoints; i++ )
            {
                memcpy( psShape->padfZ + i,
                        psSHP->pabyRec + nOffset + 16 + i*8, 8 );
                if( bBigEndian ) SwapWord( 8, psShape->padfZ + i );
            }

            nOffset += 16 + 8*nPoints;
        }

/* -------------------------------------------------------------------- */
/*      If we have a M measure value, then read it now.  We assume      */
/*      that the measure can be present for any shape if the size is    */
/*      big enough, but really it will only occur for the Z shapes      */
/*      (options), and the M shapes.                                    */
/* -------------------------------------------------------------------- */
        if( psSHP->panRecSize[hEntity]+8 >= nOffset + 16 + 8*nPoints )
        {
            memcpy( &(psShape->dfMMin), psSHP->pabyRec + nOffset, 8 );
            memcpy( &(psShape->dfMMax), psSHP->pabyRec + nOffset + 8, 8 );

            if( bBigEndian ) SwapWord( 8, &(psShape->dfMMin) );
            if( bBigEndian ) SwapWord( 8, &(psShape->dfMMax) );

            for( i = 0; i < nPoints; i++ )
            {
                memcpy( psShape->padfM + i,
                        psSHP->pabyRec + nOffset + 16 + i*8, 8 );
                if( bBigEndian ) SwapWord( 8, psShape->padfM + i );
            }
        }
#endif
    }

/* ==================================================================== */
/*      Extract vertices for a point.                                   */
/* ==================================================================== */
    else if(    nSHPType == SHPT_POINT
             || nSHPType == SHPT_POINTM
             || nSHPType == SHPT_POINTZ
             || nSHPType == SHPT_POINTZM )
    {
        int nOffset;
        double  dfX, dfY, dfZ = 0;

        int bHasZ = (nSHPType == SHPT_POINTZ || nSHPType == SHPT_POINTZM);

        if (nBytes < 4 + 8 + 8 + ((bHasZ) ? 8 : 0))
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Corrupted Shape : nBytes=%d, nSHPType=%d", nBytes, nSHPType);
            return OGRERR_FAILURE;
        }

    memcpy( &dfX, pabyShape + 4, 8 );
    memcpy( &dfY, pabyShape + 4 + 8, 8 );

        CPL_LSBPTR64( &dfX );
        CPL_LSBPTR64( &dfY );
        nOffset = 20 + 8;

        if( bHasZ )
        {
            memcpy( &dfZ, pabyShape + 4 + 16, 8 );
            CPL_LSBPTR64( &dfZ );
        }

        *ppoGeom = new OGRPoint( dfX, dfY, dfZ );

        if( !bHasZ )
            (*ppoGeom)->setCoordinateDimension( 2 );

        return OGRERR_NONE;
    }

    char* pszHex = CPLBinaryToHex( nBytes, pabyShape );
    CPLDebug( "PGEO", "Unsupported geometry type:%d\nnBytes=%d, hex=%s",
              nSHPType, nBytes, pszHex );
    CPLFree(pszHex);

    return OGRERR_FAILURE;
}
