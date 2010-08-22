/******************************************************************************
 * $Id$
 *
 * Project:  GML Reader
 * Purpose:  Implementation of GMLParseXSD()
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 2005, Frank Warmerdam
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 ****************************************************************************/

#include "parsexsd.h"
#include "cpl_error.h"
#include "cpl_conv.h"
#include "ogr_core.h"

/************************************************************************/
/*                              StripNS()                               */
/*                                                                      */
/*      Return potentially shortened form of string with namespace      */
/*      stripped off if there is one.  Returns pointer into             */
/*      original string.                                                */
/************************************************************************/

const char *StripNS( const char *pszFullValue )

{
    const char *pszColon = strstr( pszFullValue, ":" );
    if( pszColon == NULL )
        return pszFullValue;
    else
        return pszColon + 1;
}

/************************************************************************/
/*                   GetSimpleTypeProperties()                          */
/************************************************************************/

static
int GetSimpleTypeProperties(CPLXMLNode *psTypeNode,
                            GMLPropertyType *pGMLType,
                            int *pnWidth,
                            int *pnPrecision)
{
    const char *pszBase =
            StripNS( CPLGetXMLValue( psTypeNode,
                                        "restriction.base", "" ));

    if( EQUAL(pszBase,"decimal") )
    {
        *pGMLType = GMLPT_Real;
        const char *pszWidth =
            CPLGetXMLValue( psTypeNode,
                        "restriction.totalDigits.value", "0" );
        const char *pszPrecision =
            CPLGetXMLValue( psTypeNode,
                        "restriction.fractionDigits.value", "0" );
        *pnWidth = atoi(pszWidth);
        *pnPrecision = atoi(pszPrecision);
        return TRUE;
    }

    else if( EQUAL(pszBase,"float")
                || EQUAL(pszBase,"double") )
    {
        *pGMLType = GMLPT_Real;
        return TRUE;
    }

    else if( EQUAL(pszBase,"integer") )
    {
        *pGMLType = GMLPT_Integer;
        const char *pszWidth =
            CPLGetXMLValue( psTypeNode,
                        "restriction.totalDigits.value", "0" );
        *pnWidth = atoi(pszWidth);
        return TRUE;
    }

    else if( EQUAL(pszBase,"string") )
    {
        *pGMLType = GMLPT_String;
        const char *pszWidth =
            CPLGetXMLValue( psTypeNode,
                        "restriction.maxLength.value", "0" );
        *pnWidth = atoi(pszWidth);
        return TRUE;
    }

    return FALSE;
}

/************************************************************************/
/*                      LookForSimpleType()                             */
/************************************************************************/

static
int LookForSimpleType(CPLXMLNode *psSchemaNode,
                      const char* pszStrippedNSType,
                      GMLPropertyType *pGMLType,
                      int *pnWidth,
                      int *pnPrecision)
{
    CPLXMLNode *psThis;
    for( psThis = psSchemaNode->psChild;
         psThis != NULL; psThis = psThis->psNext )
    {
        if( psThis->eType == CXT_Element
           && EQUAL(psThis->pszValue,"simpleType")
           && EQUAL(CPLGetXMLValue(psThis,"name",""),pszStrippedNSType) )
        {
            break;
        }
    }
    if (psThis == NULL)
        return FALSE;

    return GetSimpleTypeProperties(psThis, pGMLType, pnWidth, pnPrecision);
}

/************************************************************************/
/*                      ParseFeatureType()                              */
/************************************************************************/

typedef struct
{
    const char* pszName;
    OGRwkbGeometryType eType;
} AssocNameType;

static const AssocNameType apsPropertyTypes [] =
{
    {"GeometryPropertyType", wkbUnknown},
    {"PointPropertyType", wkbPoint},
    {"LineStringPropertyType", wkbLineString},
    {"CurvePropertyType", wkbLineString},
    {"PolygonPropertyType", wkbPolygon},
    {"SurfacePropertyType", wkbPolygon},
    {"MultiPointPropertyType", wkbMultiPoint},
    {"MultiLineStringPropertyType", wkbMultiLineString},
    {"MultiCurvePropertyType", wkbMultiLineString},
    {"MultiPolygonPropertyType", wkbMultiPolygon},
    {"MultiSurfacePropertyType", wkbMultiPolygon},
    {"MultiGeometryPropertyType", wkbGeometryCollection},
    {NULL, wkbUnknown},
};

static
GMLFeatureClass* GMLParseFeatureType(CPLXMLNode *psSchemaNode,
                                const char* pszName,
                                const char *pszType)
{
    CPLXMLNode *psThis;
    for( psThis = psSchemaNode->psChild;
         psThis != NULL; psThis = psThis->psNext )
    {
        if( psThis->eType == CXT_Element
           && EQUAL(psThis->pszValue,"complexType")
           && EQUAL(CPLGetXMLValue(psThis,"name",""),pszType) )
        {
            break;
        }
    }
    if (psThis == NULL)
        return NULL;

/* -------------------------------------------------------------------- */
/*      Grab the sequence of extensions greatgrandchild.                */
/* -------------------------------------------------------------------- */
    CPLXMLNode *psAttrSeq =
        CPLGetXMLNode( psThis,
                        "complexContent.extension.sequence" );

    if( psAttrSeq == NULL )
    {
        return NULL;
    }

/* -------------------------------------------------------------------- */
/*      We are pretty sure this going to be a valid Feature class       */
/*      now, so create it.                                              */
/* -------------------------------------------------------------------- */
    GMLFeatureClass *poClass = new GMLFeatureClass( pszName );

/* -------------------------------------------------------------------- */
/*      Loop over each of the attribute elements being defined for      */
/*      this feature class.                                             */
/* -------------------------------------------------------------------- */
    CPLXMLNode *psAttrDef;
    int nAttributeIndex = 0;

    for( psAttrDef = psAttrSeq->psChild;
            psAttrDef != NULL;
            psAttrDef = psAttrDef->psNext )
    {
        if( !EQUAL(psAttrDef->pszValue,"element") )
            continue;

        /* MapServer WFS writes element type as an attribute of element */
        /* not as a simpleType definition */
        const char* pszType = CPLGetXMLValue( psAttrDef, "type", NULL );
        if (pszType != NULL)
        {
            const char* pszStrippedNSType = StripNS(pszType);
            int nWidth = 0, nPrecision = 0;

            GMLPropertyType gmlType = GMLPT_Untyped;
            if (EQUAL(pszStrippedNSType, "string") ||
                EQUAL(pszStrippedNSType, "Character"))
                gmlType = GMLPT_String;
            else if (EQUAL(pszStrippedNSType, "real") ||
                     EQUAL(pszStrippedNSType, "double") ||
                     EQUAL(pszStrippedNSType, "float") ||
                     EQUAL(pszStrippedNSType, "decimal"))
                gmlType = GMLPT_Real;
            else if (EQUAL(pszStrippedNSType, "short") ||
                     EQUAL(pszStrippedNSType, "int") ||
                     EQUAL(pszStrippedNSType, "integer") ||
                     EQUAL(pszStrippedNSType, "long"))
                gmlType = GMLPT_Integer;
            else if (strncmp(pszType, "gml:", 4) == 0)
            {
                const AssocNameType* psIter = apsPropertyTypes;
                while(psIter->pszName)
                {
                    if (strncmp(pszType + 4, psIter->pszName, strlen(psIter->pszName)) == 0)
                    {
                        poClass->SetGeometryElement(CPLGetXMLValue( psAttrDef, "name", NULL ));
                        poClass->SetGeometryType(psIter->eType);
                        poClass->SetGeometryAttributeIndex( nAttributeIndex );

                        nAttributeIndex ++;
                        
                        break;
                    }

                    psIter ++;
                }
                continue;
            }

            /* Integraph stuff */
            else if (strcmp(pszType, "G:Point_MultiPointPropertyType") == 0)
            {
                poClass->SetGeometryElement(CPLGetXMLValue( psAttrDef, "name", NULL ));
                poClass->SetGeometryType(wkbMultiPoint);
                poClass->SetGeometryAttributeIndex( nAttributeIndex );

                nAttributeIndex ++;
                continue;
            }
            else if (strcmp(pszType, "G:LineString_MultiLineStringPropertyType") == 0)
            {
                poClass->SetGeometryElement(CPLGetXMLValue( psAttrDef, "name", NULL ));
                poClass->SetGeometryType(wkbMultiLineString);
                poClass->SetGeometryAttributeIndex( nAttributeIndex );

                nAttributeIndex ++;
                continue;
            }
            else if (strcmp(pszType, "G:Polygon_MultiPolygonPropertyType") == 0)
            {
                poClass->SetGeometryElement(CPLGetXMLValue( psAttrDef, "name", NULL ));
                poClass->SetGeometryType(wkbMultiPolygon);
                poClass->SetGeometryAttributeIndex( nAttributeIndex );

                nAttributeIndex ++;
                continue;
            }

            else
            {
                gmlType = GMLPT_Untyped;
                if ( ! LookForSimpleType(psSchemaNode, pszStrippedNSType,
                                         &gmlType, &nWidth, &nPrecision) )
                {
                    //CPLDebug("GML", "Unknown type (%s).", pszType);
                }
            }

            GMLPropertyDefn *poProp = new GMLPropertyDefn(
                CPLGetXMLValue( psAttrDef, "name", "unnamed" ),
                CPLGetXMLValue( psAttrDef, "name", "unnamed" ) );

            poProp->SetType( gmlType );
            poProp->SetAttributeIndex( nAttributeIndex );
            poProp->SetWidth( nWidth );
            poProp->SetPrecision( nPrecision );

            poClass->AddProperty( poProp );

            nAttributeIndex ++;

            continue;
        }

        // For now we skip geometries .. fixup later.
        CPLXMLNode* psSimpleType = CPLGetXMLNode( psAttrDef, "simpleType" );
        if( psSimpleType == NULL )
            continue;

        GMLPropertyDefn *poProp = new GMLPropertyDefn(
            CPLGetXMLValue( psAttrDef, "name", "unnamed" ),
            CPLGetXMLValue( psAttrDef, "name", "unnamed" ) );


        GMLPropertyType eType = GMLPT_Untyped;
        int nWidth = 0, nPrecision = 0;
        GetSimpleTypeProperties(psSimpleType, &eType, &nWidth, &nPrecision);
        poProp->SetType( eType );
        poProp->SetWidth( nWidth );
        poProp->SetPrecision( nPrecision );
        poProp->SetAttributeIndex( nAttributeIndex );

        nAttributeIndex ++;

        poClass->AddProperty( poProp );
    }

/* -------------------------------------------------------------------- */
/*      Class complete, add to reader class list.                       */
/* -------------------------------------------------------------------- */
    poClass->SetSchemaLocked( TRUE );

    return poClass;
}

/************************************************************************/
/*                          GMLParseXSD()                               */
/************************************************************************/

int GMLParseXSD( const char *pszFile,
                 std::vector<GMLFeatureClass*> & aosClasses)

{
    if( pszFile == NULL )
        return FALSE;

/* -------------------------------------------------------------------- */
/*      Load the raw XML file.                                          */
/* -------------------------------------------------------------------- */
    CPLXMLNode *psXSDTree = CPLParseXMLFile( pszFile );
    
    if( psXSDTree == NULL )
        return FALSE;

/* -------------------------------------------------------------------- */
/*      Strip off any namespace qualifiers.                             */
/* -------------------------------------------------------------------- */
    CPLStripXMLNamespace( psXSDTree, NULL, TRUE );

/* -------------------------------------------------------------------- */
/*      Find <schema> root element.                                     */
/* -------------------------------------------------------------------- */
    CPLXMLNode *psSchemaNode = CPLGetXMLNode( psXSDTree, "=schema" );
    if( psSchemaNode == NULL )
    {
        CPLDestroyXMLNode( psXSDTree );
        return FALSE;
    }

/* ==================================================================== */
/*      Process each feature class definition.                          */
/* ==================================================================== */
    CPLXMLNode *psThis;

    for( psThis = psSchemaNode->psChild; 
         psThis != NULL; psThis = psThis->psNext )
    {
/* -------------------------------------------------------------------- */
/*      Check for <xs:element> node.                                    */
/* -------------------------------------------------------------------- */
        if( psThis->eType != CXT_Element 
            || !EQUAL(psThis->pszValue,"element") )
            continue;

/* -------------------------------------------------------------------- */
/*      Check the substitution group.                                   */
/* -------------------------------------------------------------------- */
        const char *pszSubGroup = 
            StripNS(CPLGetXMLValue(psThis,"substitutionGroup",""));

        // Old OGR produced elements for the feature collection.
        if( EQUAL(pszSubGroup, "_FeatureCollection") )
            continue;

        if( !EQUAL(pszSubGroup, "_Feature") )
        {
            continue;
        }
        
/* -------------------------------------------------------------------- */
/*      Get name                                                        */
/* -------------------------------------------------------------------- */
        const char *pszName;

        pszName = CPLGetXMLValue( psThis, "name", NULL );
        if( pszName == NULL )
        {
            continue;
        }

/* -------------------------------------------------------------------- */
/*      Get type and verify relationship with name.                     */
/* -------------------------------------------------------------------- */
        const char *pszType;

        pszType = CPLGetXMLValue( psThis, "type", NULL );
        if (pszType == NULL)
            continue;
        if( strstr( pszType, ":" ) != NULL )
            pszType = strstr( pszType, ":" ) + 1; 
        if( !EQUALN(pszType,pszName,strlen(pszName))
            || !(EQUAL(pszType+strlen(pszName),"_Type") ||
                    EQUAL(pszType+strlen(pszName),"Type")) )
        {
            continue;
        }

        GMLFeatureClass* poClass =
                GMLParseFeatureType(psSchemaNode, pszName, pszType);
        if (poClass)
            aosClasses.push_back(poClass);
    }

    CPLDestroyXMLNode( psXSDTree );

    if( aosClasses.size() > 0 )
    {
        return TRUE;
    }
    else
        return FALSE;
}
