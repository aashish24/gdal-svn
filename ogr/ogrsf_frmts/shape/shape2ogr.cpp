/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Implements translation of Shapefile shapes into OGR
 *           representation.
 * Author:   Frank Warmerdam, warmerda@home.com
 *
 ******************************************************************************
 * Copyright (c) 1999,  Les Technologies SoftMap Inc.
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

#include "ogrshape.h"
#include "cpl_conv.h"

CPL_CVSID("$Id$");

static const double EPSILON = 1E-5;

/************************************************************************/
/*                            epsilonEqual()                            */
/************************************************************************/

static inline bool epsilonEqual(double a, double b, double eps) 
{
    return (::fabs(a - b) < eps);
}

/************************************************************************/
/*                        RingStartEnd                                  */
/*        set first and last vertex for given ring                      */
/************************************************************************/
void RingStartEnd ( SHPObject *psShape, int ring, int *start, int *end )
{
    if( psShape->panPartStart == NULL )
    {
	    *start = 0;
        *end = psShape->nVertices - 1;
    }
    else
    {
        if( ring == psShape->nParts - 1 )
            *end = psShape->nVertices - 1;
        else
            *end = psShape->panPartStart[ring+1] - 1;

        *start = psShape->panPartStart[ring];
    }
}
    
/************************************************************************/
/*                        CreateLinearRing                              */
/*                                                                      */
/************************************************************************/
OGRLinearRing * CreateLinearRing ( SHPObject *psShape, int ring )
{
    OGRLinearRing *poRing;
    int nRingStart, nRingEnd, nRingPoints;

    poRing = new OGRLinearRing();

    RingStartEnd ( psShape, ring, &nRingStart, &nRingEnd );

    nRingPoints = nRingEnd - nRingStart + 1;

    poRing->setPoints( nRingPoints, psShape->padfX + nRingStart, 
	       psShape->padfY + nRingStart, psShape->padfZ + nRingStart );

    return ( poRing );
}


/************************************************************************/
/*                          SHPReadOGRObject()                          */
/*                                                                      */
/*      Read an item in a shapefile, and translate to OGR geometry      */
/*      representation.                                                 */
/************************************************************************/

OGRGeometry *SHPReadOGRObject( SHPHandle hSHP, int iShape )
{
    // CPLDebug( "Shape", "SHPReadOGRObject( iShape=%d )\n", iShape );

    SHPObject   *psShape;
    OGRGeometry *poOGR = NULL;

    psShape = SHPReadObject( hSHP, iShape );

    if( psShape == NULL )
    {
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Point.                                                          */
/* -------------------------------------------------------------------- */
    else if( psShape->nSHPType == SHPT_POINT
             || psShape->nSHPType == SHPT_POINTM
             || psShape->nSHPType == SHPT_POINTZ )
    {
        poOGR = new OGRPoint( psShape->padfX[0], psShape->padfY[0],
                              psShape->padfZ[0] );

        if( psShape->nSHPType == SHPT_POINT )
        {
            poOGR->setCoordinateDimension( 2 );
        }
    }

/* -------------------------------------------------------------------- */
/*      Multipoint.                                                     */
/* -------------------------------------------------------------------- */
    else if( psShape->nSHPType == SHPT_MULTIPOINT
             || psShape->nSHPType == SHPT_MULTIPOINTM
             || psShape->nSHPType == SHPT_MULTIPOINTZ )
    {
        if (psShape->nVertices == 0)
        {
            poOGR = NULL;
        }
        else
        {
            OGRMultiPoint *poOGRMPoint = new OGRMultiPoint();
            int             i;

            for( i = 0; i < psShape->nVertices; i++ )
            {
                OGRPoint    *poPoint;

                poPoint = new OGRPoint( psShape->padfX[i], psShape->padfY[i],
                                        psShape->padfZ[i] );

                poOGRMPoint->addGeometry( poPoint );

                delete poPoint;
            }

            poOGR = poOGRMPoint;

            if( psShape->nSHPType == SHPT_MULTIPOINT )
                poOGR->setCoordinateDimension( 2 );
        }
    }

/* -------------------------------------------------------------------- */
/*      Arc (LineString)                                                */
/*                                                                      */
/*      I am ignoring parts though they can apply to arcs as well.      */
/* -------------------------------------------------------------------- */
    else if( psShape->nSHPType == SHPT_ARC
             || psShape->nSHPType == SHPT_ARCM
             || psShape->nSHPType == SHPT_ARCZ )
    {
        if( psShape->nParts == 0 )
        {
            poOGR = NULL;
        }
        else if( psShape->nParts == 1 )
        {
            OGRLineString *poOGRLine = new OGRLineString();

            poOGRLine->setPoints( psShape->nVertices,
                                  psShape->padfX, psShape->padfY, psShape->padfZ );

            poOGR = poOGRLine;
        }
        else
        {
            int iRing;
            OGRMultiLineString *poOGRMulti;
        
            poOGR = poOGRMulti = new OGRMultiLineString();
            
            for( iRing = 0; iRing < psShape->nParts; iRing++ )
            {
                OGRLineString   *poLine;
                int     nRingPoints;
                int     nRingStart;

                poLine = new OGRLineString();

                if( psShape->panPartStart == NULL )
                {
                    nRingPoints = psShape->nVertices;
                    nRingStart = 0;
                }
                else
                {

                    if( iRing == psShape->nParts - 1 )
                        nRingPoints =
                            psShape->nVertices - psShape->panPartStart[iRing];
                    else
                        nRingPoints = psShape->panPartStart[iRing+1]
                            - psShape->panPartStart[iRing];
                    nRingStart = psShape->panPartStart[iRing];
                }
            
                poLine->setPoints( nRingPoints, 
                                   psShape->padfX + nRingStart,
                                   psShape->padfY + nRingStart,
                                   psShape->padfZ + nRingStart );

                poOGRMulti->addGeometryDirectly( poLine );
            }
        }

        if( poOGR != NULL && psShape->nSHPType == SHPT_ARC )
            poOGR->setCoordinateDimension( 2 );
    }

/* -------------------------------------------------------------------- */
/*      Polygon                                                         */
/*                                                                      */
/* As for now Z coordinate is not handled correctly                     */
/* -------------------------------------------------------------------- */
    else if( psShape->nSHPType == SHPT_POLYGON
             || psShape->nSHPType == SHPT_POLYGONM
             || psShape->nSHPType == SHPT_POLYGONZ )
    {
        int iRing;
        
        //CPLDebug( "Shape", "Shape type: polygon with nParts=%d \n", psShape->nParts );

        if ( psShape->nParts == 0 )
        {
            poOGR = NULL;
        }
        else if ( psShape->nParts == 1 )
        {
            /* Surely outer ring */
            OGRPolygon *poOGRPoly = NULL;
            OGRLinearRing *poRing = NULL;

            poOGR = poOGRPoly = new OGRPolygon();
            poRing = CreateLinearRing ( psShape, 0 );
            poOGRPoly->addRingDirectly( poRing );
        }

        else
        {
            OGRPolygon** tabPolygons = new OGRPolygon*[psShape->nParts];
            for( iRing = 0; iRing < psShape->nParts; iRing++ )
            {
                tabPolygons[iRing] = new OGRPolygon();
                tabPolygons[iRing]->addRingDirectly(CreateLinearRing ( psShape, iRing ));
            }

            int isValidGeometry;
            poOGR = OGRGeometryFactory::organizePolygons( 
                (OGRGeometry**)tabPolygons, psShape->nParts, &isValidGeometry );

            if (!isValidGeometry)
            {
                CPLError(CE_Warning, CPLE_AppDefined, 
                        "Geometry of polygon of fid %d cannot be translated to Simple Geometry. "
                        "All polygons will be contained in a multipolygon.\n",
                        iShape);
            }

            delete[] tabPolygons;
        }

        if( poOGR != NULL && psShape->nSHPType == SHPT_POLYGON )
        {
            poOGR->setCoordinateDimension( 2 );
        }
    }

/* -------------------------------------------------------------------- */
/*      Otherwise for now we just ignore the object.  Eventually we     */
/*      should implement multipatch.                                    */
/* -------------------------------------------------------------------- */
    else
    {
        if( psShape->nSHPType != SHPT_NULL )
        {
            CPLDebug( "OGR", "Unsupported shape type in SHPReadOGRObject()" );
        }

        /* nothing returned */
    }
    
/* -------------------------------------------------------------------- */
/*      Cleanup shape, and set feature id.                              */
/* -------------------------------------------------------------------- */
    SHPDestroyObject( psShape );

    return poOGR;
}

/************************************************************************/
/*                         SHPWriteOGRObject()                          */
/************************************************************************/

OGRErr SHPWriteOGRObject( SHPHandle hSHP, int iShape, OGRGeometry *poGeom )

{
/* ==================================================================== */
/*      Write "shape" with no geometry or with empty geometry           */
/* ==================================================================== */
    if( poGeom == NULL || poGeom->IsEmpty() )
    {
        SHPObject       *psShape;

        psShape = SHPCreateSimpleObject( SHPT_NULL, 0, NULL, NULL, NULL );
        SHPWriteObject( hSHP, iShape, psShape );
        SHPDestroyObject( psShape );
    }

/* ==================================================================== */
/*      Write point geometry.                                           */
/* ==================================================================== */
    else if( hSHP->nShapeType == SHPT_POINT
             || hSHP->nShapeType == SHPT_POINTM
             || hSHP->nShapeType == SHPT_POINTZ )
    {
        SHPObject       *psShape;
        OGRPoint        *poPoint = (OGRPoint *) poGeom;
        double          dfX, dfY, dfZ = 0;

        if( poGeom->getGeometryType() != wkbPoint
            && poGeom->getGeometryType() != wkbPoint25D )        
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Attempt to write non-point (%s) geometry to"
                      " point shapefile.",
                      poGeom->getGeometryName() );

            return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
        }

        dfX = poPoint->getX();
        dfY = poPoint->getY();
        dfZ = poPoint->getZ();
        
        psShape = SHPCreateSimpleObject( hSHP->nShapeType, 1,
                                         &dfX, &dfY, &dfZ );
        SHPWriteObject( hSHP, iShape, psShape );
        SHPDestroyObject( psShape );
    }
/* ==================================================================== */
/*      MultiPoint.                                                     */
/* ==================================================================== */
    else if( hSHP->nShapeType == SHPT_MULTIPOINT
             || hSHP->nShapeType == SHPT_MULTIPOINTM
             || hSHP->nShapeType == SHPT_MULTIPOINTZ )
    {
        OGRMultiPoint   *poMP = (OGRMultiPoint *) poGeom;
        double          *padfX, *padfY, *padfZ;
        int             iPoint;
        SHPObject       *psShape;

        if( wkbFlatten(poGeom->getGeometryType()) != wkbMultiPoint )
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Attempt to write non-multipoint (%s) geometry to "
                      "multipoint shapefile.",
                      poGeom->getGeometryName() );

            return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
        }

        padfX = (double *) CPLMalloc(sizeof(double)*poMP->getNumGeometries());
        padfY = (double *) CPLMalloc(sizeof(double)*poMP->getNumGeometries());
        padfZ = (double *) CPLCalloc(sizeof(double),poMP->getNumGeometries());

        int iDstPoints = 0;
        for( iPoint = 0; iPoint < poMP->getNumGeometries(); iPoint++ )
        {
            OGRPoint    *poPoint = (OGRPoint *) poMP->getGeometryRef(iPoint);

            /* Ignore POINT EMPTY */
            if (poPoint->IsEmpty() == FALSE)
            {
                padfX[iDstPoints] = poPoint->getX();
                padfY[iDstPoints] = poPoint->getY();
                padfZ[iDstPoints] = poPoint->getZ();
                iDstPoints ++;
            }
            else
                CPLDebug( "OGR", 
                              "Ignore POINT EMPTY inside MULTIPOINT in shapefile writer." );
        }

        psShape = SHPCreateSimpleObject( hSHP->nShapeType,
                                         iDstPoints,
                                         padfX, padfY, padfZ );
        SHPWriteObject( hSHP, iShape, psShape );
        SHPDestroyObject( psShape );
        
        CPLFree( padfX );
        CPLFree( padfY );
        CPLFree( padfZ );
    }

/* ==================================================================== */
/*      Arcs from simple line strings.                                  */
/* ==================================================================== */
    else if( (hSHP->nShapeType == SHPT_ARC
              || hSHP->nShapeType == SHPT_ARCM
              || hSHP->nShapeType == SHPT_ARCZ)
             && wkbFlatten(poGeom->getGeometryType()) == wkbLineString )
    {
        OGRLineString   *poArc = (OGRLineString *) poGeom;
        double          *padfX, *padfY, *padfZ;
        int             iPoint;
        SHPObject       *psShape;

        padfX = (double *) CPLMalloc(sizeof(double)*poArc->getNumPoints());
        padfY = (double *) CPLMalloc(sizeof(double)*poArc->getNumPoints());
        padfZ = (double *) CPLCalloc(sizeof(double),poArc->getNumPoints());

        for( iPoint = 0; iPoint < poArc->getNumPoints(); iPoint++ )
        {
            padfX[iPoint] = poArc->getX( iPoint );
            padfY[iPoint] = poArc->getY( iPoint );
            padfZ[iPoint] = poArc->getZ( iPoint );
        }

        psShape = SHPCreateSimpleObject( hSHP->nShapeType,
                                         poArc->getNumPoints(),
                                         padfX, padfY, padfZ );
        SHPWriteObject( hSHP, iShape, psShape );
        SHPDestroyObject( psShape );
        
        CPLFree( padfX );
        CPLFree( padfY );
        CPLFree( padfZ );
    }
/* ==================================================================== */
/*      Arcs - Try to treat as MultiLineString.                         */
/* ==================================================================== */
    else if( hSHP->nShapeType == SHPT_ARC
             || hSHP->nShapeType == SHPT_ARCM
             || hSHP->nShapeType == SHPT_ARCZ )
    {
        OGRMultiLineString *poML;
        double          *padfX=NULL, *padfY=NULL, *padfZ=NULL;
        int             iGeom, iPoint, nPointCount = 0;
        SHPObject       *psShape;
        int             *panRingStart;
        int             nParts = 0;

        poML = (OGRMultiLineString *) 
            OGRGeometryFactory::forceToMultiLineString( poGeom->clone() );

        if( wkbFlatten(poML->getGeometryType()) != wkbMultiLineString )
        {
            delete poML;
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Attempt to write non-linestring (%s) geometry to "
                      "ARC type shapefile.",
                      poGeom->getGeometryName() );

            return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
        }

        panRingStart = (int *) 
            CPLMalloc(sizeof(int) * poML->getNumGeometries());

        for( iGeom = 0; iGeom < poML->getNumGeometries(); iGeom++ )
        {
            OGRLineString *poArc = (OGRLineString *)
                poML->getGeometryRef(iGeom);
            int nNewPoints = poArc->getNumPoints();

            /* Ignore LINESTRING EMPTY */
            if (nNewPoints == 0)
                continue;
            else
                CPLDebug( "OGR", 
                              "Ignore LINESTRING EMPTY inside MULTILINESTRING in shapefile writer." );

            panRingStart[nParts ++] = nPointCount;

            padfX = (double *) 
                CPLRealloc( padfX, sizeof(double)*(nNewPoints+nPointCount) );
            padfY = (double *) 
                CPLRealloc( padfY, sizeof(double)*(nNewPoints+nPointCount) );
            padfZ = (double *) 
                CPLRealloc( padfZ, sizeof(double)*(nNewPoints+nPointCount) );

            for( iPoint = 0; iPoint < nNewPoints; iPoint++ )
            {
                padfX[nPointCount] = poArc->getX( iPoint );
                padfY[nPointCount] = poArc->getY( iPoint );
                padfZ[nPointCount] = poArc->getZ( iPoint );
                nPointCount++;
            }
        }

        CPLAssert(nParts != 0);

        psShape = SHPCreateObject( hSHP->nShapeType, iShape, 
                                    nParts, 
                                    panRingStart, NULL,
                                    nPointCount, padfX, padfY, padfZ, NULL);
        SHPWriteObject( hSHP, iShape, psShape );
        SHPDestroyObject( psShape );

        CPLFree( panRingStart );
        CPLFree( padfX );
        CPLFree( padfY );
        CPLFree( padfZ );

        delete poML;
    }

/* ==================================================================== */
/*      Polygons/MultiPolygons                                          */
/* ==================================================================== */
    else if( hSHP->nShapeType == SHPT_POLYGON
             || hSHP->nShapeType == SHPT_POLYGONM
             || hSHP->nShapeType == SHPT_POLYGONZ )
    {
        OGRPolygon      *poPoly;
        OGRLinearRing   *poRing, **papoRings=NULL;
        double          *padfX=NULL, *padfY=NULL, *padfZ=NULL;
        int             iPoint, iRing, nRings, nVertex=0, *panRingStart;
        SHPObject       *psShape;

        /* Collect list of rings */

        if( wkbFlatten(poGeom->getGeometryType()) == wkbPolygon )
        {
            poPoly =  (OGRPolygon *) poGeom;

            if( poPoly->getExteriorRing() == NULL ||
                poPoly->getExteriorRing()->IsEmpty() )
            {
                CPLDebug( "OGR", 
                          "Ignore POLYGON EMPTY in shapefile writer." );
                nRings = 0;
            }
            else
            {
                int nSrcRings = poPoly->getNumInteriorRings()+1;
                nRings = 0;
                papoRings = (OGRLinearRing **) CPLMalloc(sizeof(void*)*nSrcRings);
                for( iRing = 0; iRing < nSrcRings; iRing++ )
                {
                    if( iRing == 0 )
                        papoRings[nRings] = poPoly->getExteriorRing();
                    else
                        papoRings[nRings] = poPoly->getInteriorRing( iRing-1 );

                    /* Ignore LINEARRING EMPTY */
                    if (papoRings[nRings]->getNumPoints() != 0)
                        nRings ++;
                    else
                        CPLDebug( "OGR", 
                                "Ignore LINEARRING EMPTY inside POLYGON in shapefile writer." );
                }
            }
        }
        else if( wkbFlatten(poGeom->getGeometryType()) == wkbMultiPolygon
                 || wkbFlatten(poGeom->getGeometryType()) 
                                                == wkbGeometryCollection )
        {
            OGRGeometryCollection *poGC = (OGRGeometryCollection *) poGeom;
            int         iGeom;

            nRings = 0;
            for( iGeom=0; iGeom < poGC->getNumGeometries(); iGeom++ )
            {
                poPoly =  (OGRPolygon *) poGC->getGeometryRef( iGeom );

                if( wkbFlatten(poPoly->getGeometryType()) != wkbPolygon )
                {
                    CPLFree( papoRings );
                    CPLError( CE_Failure, CPLE_AppDefined,
                              "Attempt to write non-polygon (%s) geometry to "
                              " type shapefile.",
                              poGeom->getGeometryName() );

                    return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
                }

                /* Ignore POLYGON EMPTY */
                if( poPoly->getExteriorRing() == NULL ||
                    poPoly->getExteriorRing()->IsEmpty() )
                {
                    CPLDebug( "OGR", 
                              "Ignore POLYGON EMPTY inside MULTIPOLYGON in shapefile writer." );
                    continue;
                }

                papoRings = (OGRLinearRing **) CPLRealloc(papoRings, 
                     sizeof(void*) * (nRings+poPoly->getNumInteriorRings()+1));
                for( iRing = 0; 
                     iRing < poPoly->getNumInteriorRings()+1; 
                     iRing++ )
                {
                    if( iRing == 0 )
                        papoRings[nRings] = poPoly->getExteriorRing();
                    else
                        papoRings[nRings] = 
                            poPoly->getInteriorRing( iRing-1 );

                    /* Ignore LINEARRING EMPTY */
                    if (papoRings[nRings]->getNumPoints() != 0)
                        nRings ++;
                    else
                        CPLDebug( "OGR", 
                              "Ignore LINEARRING EMPTY inside POLYGON in shapefile writer." );
                }
            }
        }
        else 
        {
            CPLError( CE_Failure, CPLE_AppDefined,
                      "Attempt to write non-polygon (%s) geometry to "
                      " type shapefile.",
                      poGeom->getGeometryName() );

            return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
        }

/* -------------------------------------------------------------------- */
/*      If we only had emptypolygons or unacceptable geometries         */
/*      write NULL geometry object.                                     */
/* -------------------------------------------------------------------- */
        if( nRings == 0 )
        {
            SHPObject       *psShape;
            
            psShape = SHPCreateSimpleObject( SHPT_NULL, 0, NULL, NULL, NULL );
            SHPWriteObject( hSHP, iShape, psShape );
            SHPDestroyObject( psShape );
            return OGRERR_NONE;
        }
        
        /* count vertices */
        nVertex = 0;
        for( iRing = 0; iRing < nRings; iRing++ )
            nVertex += papoRings[iRing]->getNumPoints();

        panRingStart = (int *) CPLMalloc(sizeof(int) * nRings);
        padfX = (double *) CPLMalloc(sizeof(double)*nVertex);
        padfY = (double *) CPLMalloc(sizeof(double)*nVertex);
        padfZ = (double *) CPLMalloc(sizeof(double)*nVertex);

        /* collect vertices */
        nVertex = 0;
        for( iRing = 0; iRing < nRings; iRing++ )
        {
            poRing = papoRings[iRing];
            panRingStart[iRing] = nVertex;

            for( iPoint = 0; iPoint < poRing->getNumPoints(); iPoint++ )
            {
                padfX[nVertex] = poRing->getX( iPoint );
                padfY[nVertex] = poRing->getY( iPoint );
                padfZ[nVertex] = poRing->getZ( iPoint );
                nVertex++;
            }
        }

        psShape = SHPCreateObject( hSHP->nShapeType, iShape, nRings,
                                   panRingStart, NULL,
                                   nVertex, padfX, padfY, padfZ, NULL );
        SHPRewindObject( hSHP, psShape );
        SHPWriteObject( hSHP, iShape, psShape );
        SHPDestroyObject( psShape );
        
        CPLFree( papoRings );
        CPLFree( panRingStart );
        CPLFree( padfX );
        CPLFree( padfY );
        CPLFree( padfZ );
    }
    else
    {
        /* do nothing for multipatch */
        return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
    }

    return OGRERR_NONE;
}

/************************************************************************/
/*                       SHPReadOGRFeatureDefn()                        */
/************************************************************************/

OGRFeatureDefn *SHPReadOGRFeatureDefn( const char * pszName,
                                       SHPHandle hSHP, DBFHandle hDBF )

{
    OGRFeatureDefn      *poDefn = new OGRFeatureDefn( pszName );
    int                 iField;

    poDefn->Reference();

    for( iField = 0; 
         hDBF != NULL && iField < DBFGetFieldCount( hDBF ); 
         iField++ )
    {
        char            szFieldName[20];
        int             nWidth, nPrecision;
        DBFFieldType    eDBFType;
        OGRFieldDefn    oField("", OFTInteger);
        char            chNativeType;

        chNativeType = DBFGetNativeFieldType( hDBF, iField );
        eDBFType = DBFGetFieldInfo( hDBF, iField, szFieldName,
                                    &nWidth, &nPrecision );

        oField.SetName( szFieldName );
        oField.SetWidth( nWidth );
        oField.SetPrecision( nPrecision );

        if( chNativeType == 'D' )
        {
            /* XXX - mloskot:
             * Shapefile date has following 8-chars long format: 20060101.
             * OGR splits it as YYYY/MM/DD, so 2 additional characters are required.
             * Is this correct assumtion? What about time part of date?
             * Shouldn't this format look as datetime: YYYY/MM/DD HH:MM:SS
             * with 4 additional characters?
             */
            oField.SetWidth( nWidth + 2 );
            oField.SetType( OFTDate );
        }
        else if( eDBFType == FTDouble )
            oField.SetType( OFTReal );
        else if( eDBFType == FTInteger )
            oField.SetType( OFTInteger );
        else
            oField.SetType( OFTString );

        poDefn->AddFieldDefn( &oField );
    }

    if( hSHP == NULL )
        poDefn->SetGeomType( wkbNone );
    else
    {
        switch( hSHP->nShapeType )
        {
          case SHPT_POINT:
          case SHPT_POINTM:
            poDefn->SetGeomType( wkbPoint );
            break;

          case SHPT_POINTZ:
            poDefn->SetGeomType( wkbPoint25D );
            break;

          case SHPT_ARC:
          case SHPT_ARCM:
            poDefn->SetGeomType( wkbLineString );
            break;

          case SHPT_ARCZ:
            poDefn->SetGeomType( wkbLineString25D );
            break;

          case SHPT_MULTIPOINT:
          case SHPT_MULTIPOINTM:
            poDefn->SetGeomType( wkbMultiPoint );
            break;

          case SHPT_MULTIPOINTZ:
            poDefn->SetGeomType( wkbMultiPoint25D );
            break;

          case SHPT_POLYGON:
          case SHPT_POLYGONM:
            poDefn->SetGeomType( wkbPolygon );
            break;

          case SHPT_POLYGONZ:
            poDefn->SetGeomType( wkbPolygon25D );
            break;
            
        }
    }

    return poDefn;
}

/************************************************************************/
/*                         SHPReadOGRFeature()                          */
/************************************************************************/

OGRFeature *SHPReadOGRFeature( SHPHandle hSHP, DBFHandle hDBF,
                               OGRFeatureDefn * poDefn, int iShape )

{
    if( iShape < 0 
        || (hSHP != NULL && iShape >= hSHP->nRecords)
        || (hDBF != NULL && iShape >= hDBF->nRecords) )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Attempt to read shape with feature id (%d) out of available"
                  " range.", iShape );
        return NULL;
    }

    if( hDBF && DBFIsRecordDeleted( hDBF, iShape ) )
    {
        CPLError( CE_Failure, CPLE_AppDefined, 
                  "Attempt to read shape with feature id (%d), but it is marked deleted.",
                  iShape );
        return NULL;
    }

    OGRFeature  *poFeature = new OGRFeature( poDefn );

/* -------------------------------------------------------------------- */
/*      Fetch geometry from Shapefile to OGRFeature.                    */
/* -------------------------------------------------------------------- */
    if( hSHP != NULL )
    {
        OGRGeometry* poGeometry = NULL;
        poGeometry = SHPReadOGRObject( hSHP, iShape );

        /*
         * NOTE - mloskot:
         * Two possibilities are expected here (bot hare tested by GDAL Autotests):
         * 1. Read valid geometry and assign it directly.
         * 2. Read and assign null geometry if it can not be read correctly from a shapefile
         *
         * It's NOT required here to test poGeometry == NULL.
         */

        poFeature->SetGeometryDirectly( poGeometry );
    }

/* -------------------------------------------------------------------- */
/*      Fetch feature attributes to OGRFeature fields.                  */
/* -------------------------------------------------------------------- */

    for( int iField = 0; iField < poDefn->GetFieldCount(); iField++ )
    {
        // Skip null fields.
        if( DBFIsAttributeNULL( hDBF, iShape, iField ) )
            continue;

        switch( poDefn->GetFieldDefn(iField)->GetType() )
        {
          case OFTString:
            poFeature->SetField( iField,
                                 DBFReadStringAttribute( hDBF, iShape,
                                                         iField ) );
            break;

          case OFTInteger:
            poFeature->SetField( iField,
                                 DBFReadIntegerAttribute( hDBF, iShape,
                                                          iField ) );
            break;

          case OFTReal:
            poFeature->SetField( iField,
                                 DBFReadDoubleAttribute( hDBF, iShape,
                                                         iField ) );
            break;

          case OFTDate:
          {
              OGRField sFld;
              int nFullDate = 
                  DBFReadIntegerAttribute( hDBF, iShape, iField );
              
              memset( &sFld, 0, sizeof(sFld) );
              sFld.Date.Year = nFullDate / 10000;
              sFld.Date.Month = (nFullDate / 100) % 100;
              sFld.Date.Day = nFullDate % 100;
              
              poFeature->SetField( iField, &sFld );
          }
          break;

          default:
            CPLAssert( FALSE );
        }
    }

    if( poFeature != NULL )
        poFeature->SetFID( iShape );

    return( poFeature );
}

/************************************************************************/
/*                         SHPWriteOGRFeature()                         */
/*                                                                      */
/*      Write to an existing feature in a shapefile, or create a new    */
/*      feature.                                                        */
/************************************************************************/

OGRErr SHPWriteOGRFeature( SHPHandle hSHP, DBFHandle hDBF,
                           OGRFeatureDefn * poDefn, 
                           OGRFeature * poFeature )

{
#ifdef notdef
/* -------------------------------------------------------------------- */
/*      Don't write objects with missing geometry.                      */
/* -------------------------------------------------------------------- */
    if( poFeature->GetGeometryRef() == NULL && hSHP != NULL )
    {
        CPLError( CE_Failure, CPLE_AppDefined,
                  "Attempt to write feature without geometry not supported"
                  " for shapefile driver." );
        
        return OGRERR_UNSUPPORTED_GEOMETRY_TYPE;
    }
#endif

/* -------------------------------------------------------------------- */
/*      Write the geometry.                                             */
/* -------------------------------------------------------------------- */
    OGRErr      eErr;

    if( hSHP != NULL )
    {
        eErr = SHPWriteOGRObject( hSHP, poFeature->GetFID(),
                                  poFeature->GetGeometryRef() );
        if( eErr != OGRERR_NONE )
            return eErr;
    }
    
/* -------------------------------------------------------------------- */
/*      If this is a new feature, establish it's feature id.            */
/* -------------------------------------------------------------------- */
    if( poFeature->GetFID() == OGRNullFID )
        poFeature->SetFID( DBFGetRecordCount( hDBF ) );

/* -------------------------------------------------------------------- */
/*      If this is the first feature to be written, verify that we      */
/*      have at least one attribute in the DBF file.  If not, create    */
/*      a dummy FID attribute to satisfy the requirement that there     */
/*      be at least one attribute.                                      */
/* -------------------------------------------------------------------- */
    if( DBFGetRecordCount( hDBF ) == 0 && DBFGetFieldCount( hDBF ) == 0 )
    {
        CPLDebug( "OGR", 
               "Created dummy FID field for shapefile since schema is empty.");
        DBFAddField( hDBF, "FID", FTInteger, 11, 0 );
    }

/* -------------------------------------------------------------------- */
/*      Write out dummy field value if it exists.                       */
/* -------------------------------------------------------------------- */
    if( DBFGetFieldCount( hDBF ) == 1 && poDefn->GetFieldCount() == 0 )
    {
        DBFWriteIntegerAttribute( hDBF, poFeature->GetFID(), 0, 
                                  poFeature->GetFID() );
    }

/* -------------------------------------------------------------------- */
/*      Write all the fields.                                           */
/* -------------------------------------------------------------------- */
    for( int iField = 0; iField < poDefn->GetFieldCount(); iField++ )
    {
        if( !poFeature->IsFieldSet( iField ) )
        {
            DBFWriteNULLAttribute( hDBF, poFeature->GetFID(), iField );
            continue;
        }

        switch( poDefn->GetFieldDefn(iField)->GetType() )
        {
          case OFTString:
            DBFWriteStringAttribute( hDBF, poFeature->GetFID(), iField, 
                                     poFeature->GetFieldAsString( iField ));
            break;

          case OFTInteger:
            DBFWriteIntegerAttribute( hDBF, poFeature->GetFID(), iField, 
                                      poFeature->GetFieldAsInteger(iField) );
            break;

          case OFTReal:
            DBFWriteDoubleAttribute( hDBF, poFeature->GetFID(), iField, 
                                     poFeature->GetFieldAsDouble(iField) );
            break;

          case OFTDate:
          {
              int  nYear, nMonth, nDay;

              if( poFeature->GetFieldAsDateTime( iField, &nYear, &nMonth, &nDay,
                                                 NULL, NULL, NULL, NULL ) )
              {
                  DBFWriteIntegerAttribute( hDBF, poFeature->GetFID(), iField, 
                                            nYear*10000 + nMonth*100 + nDay );
              }
          }
          break;

          default:
          {
              /* Ignore fields of other types */
              break;
          }
        }
    }

    return OGRERR_NONE;
}

