/******************************************************************************
 * $Id$
 *
 * Project:  GPX Translator
 * Purpose:  Implements OGRGPXLayer class.
 * Author:   Even Rouault, even dot rouault at mines dash paris dot org
 *
 ******************************************************************************
 * Copyright (c) 2007, Even Rouault
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

#include "ogr_gpx.h"
#include "cpl_conv.h"
#include "cpl_string.h"
#include "ogr_p.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*                            OGRGPXLayer()                             */
/*                                                                      */
/*      Note that the OGRGPXLayer assumes ownership of the passed       */
/*      file pointer.                                                   */
/************************************************************************/

OGRGPXLayer::OGRGPXLayer( const char* pszFilename,
                          const char* pszLayerName,
                          GPXGeometryType gpxGeomType,
                          OGRGPXDataSource* poDS,
                          int bWriteMode)

{
    const char* gpxVersion = poDS->GetVersion();

    int i;

    eof = FALSE;
    nNextFID = 0;
    
    this->poDS = poDS;
    this->bWriteMode = bWriteMode;
    this->gpxGeomType = gpxGeomType;
    
    pszElementToScan = pszLayerName;
    
    nMaxLinks = atoi(CPLGetConfigOption("GPX_N_MAX_LINKS", "2"));
    if (nMaxLinks < 0)
        nMaxLinks = 2;
    if (nMaxLinks > 100)
        nMaxLinks = 100;

    nFeatures = 0;
    
    bEleAs25D =  CSLTestBoolean(CPLGetConfigOption("GPX_ELE_AS_25D", "NO"));
    
    int bShortNames  = CSLTestBoolean(CPLGetConfigOption("GPX_SHORT_NAMES", "NO"));
    
    poFeatureDefn = new OGRFeatureDefn( pszLayerName );
    poFeatureDefn->Reference();
    
    if (gpxGeomType == GPX_TRACK_POINT)
    {
        /* Don't move this code. This fields must be number 0, 1 and 2 */
        /* in order to make OGRGPXLayer::startElementCbk work */
        OGRFieldDefn oFieldTrackFID("track_fid", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldTrackFID );
        
        OGRFieldDefn oFieldTrackSegID((bShortNames) ? "trksegid" : "track_seg_id", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldTrackSegID );
        
        OGRFieldDefn oFieldTrackSegPointID((bShortNames) ? "trksegptid" : "track_seg_point_id", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldTrackSegPointID );
    }
    else if (gpxGeomType == GPX_ROUTE_POINT)
    {
        /* Don't move this code. See above */
        OGRFieldDefn oFieldRouteFID("route_fid", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldRouteFID );
        
        OGRFieldDefn oFieldRoutePointID((bShortNames) ? "rteptid" : "route_point_id", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldRoutePointID );
    }

    if (gpxGeomType == GPX_WPT ||
        gpxGeomType == GPX_TRACK_POINT ||
        gpxGeomType == GPX_ROUTE_POINT)
    {
        poFeatureDefn->SetGeomType((bEleAs25D) ? wkbPoint25D : wkbPoint);
        /* Position info */
        
        OGRFieldDefn oFieldEle("ele", OFTReal );
        poFeatureDefn->AddFieldDefn( &oFieldEle );
        
        OGRFieldDefn oFieldTime("time", OFTDateTime );
        poFeatureDefn->AddFieldDefn( &oFieldTime );
        
        if (gpxGeomType == GPX_TRACK_POINT &&
            gpxVersion && strcmp(gpxVersion, "1.0") == 0)
        {
            OGRFieldDefn oFieldCourse("course", OFTReal );
            poFeatureDefn->AddFieldDefn( &oFieldCourse );
            
            OGRFieldDefn oFieldSpeed("speed", OFTReal );
            poFeatureDefn->AddFieldDefn( &oFieldSpeed );
        }
        
        OGRFieldDefn oFieldMagVar("magvar", OFTReal );
        poFeatureDefn->AddFieldDefn( &oFieldMagVar );
    
        OGRFieldDefn oFieldGeoidHeight("geoidheight", OFTReal );
        poFeatureDefn->AddFieldDefn( &oFieldGeoidHeight );
            
        /* Description info */
        
        OGRFieldDefn oFieldName("name", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldName );
        
        OGRFieldDefn oFieldCmt("cmt", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldCmt );
        
        OGRFieldDefn oFieldDesc("desc", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldDesc );
        
        OGRFieldDefn oFieldSrc("src", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldSrc );
        
        if (gpxVersion && strcmp(gpxVersion, "1.0") == 0)
        {
            OGRFieldDefn oFieldUrl("url", OFTString );
            poFeatureDefn->AddFieldDefn( &oFieldUrl );
            
            OGRFieldDefn oFieldUrlName("urlname", OFTString );
            poFeatureDefn->AddFieldDefn( &oFieldUrlName );
        }
        else
        {
            for(i=1;i<=nMaxLinks;i++)
            {
                char szFieldName[32];
                sprintf(szFieldName, "link%d_href", i);
                OGRFieldDefn oFieldLinkHref( szFieldName, OFTString );
                poFeatureDefn->AddFieldDefn( &oFieldLinkHref );
                
                sprintf(szFieldName, "link%d_text", i);
                OGRFieldDefn oFieldLinkText( szFieldName, OFTString );
                poFeatureDefn->AddFieldDefn( &oFieldLinkText );
                
                sprintf(szFieldName, "link%d_type", i);
                OGRFieldDefn oFieldLinkType( szFieldName, OFTString );
                poFeatureDefn->AddFieldDefn( &oFieldLinkType );
            }
        }
        
        OGRFieldDefn oFieldSym("sym", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldSym );
        
        OGRFieldDefn oFieldType("type", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldType );
    
        /* Accuracy info */
        
        OGRFieldDefn oFieldFix("fix", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldFix );
        
        OGRFieldDefn oFieldSat("sat", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldSat );
        
        OGRFieldDefn oFieldHdop("hdop", OFTReal );
        poFeatureDefn->AddFieldDefn( &oFieldHdop );
        
        OGRFieldDefn oFieldVdop("vdop", OFTReal );
        poFeatureDefn->AddFieldDefn( &oFieldVdop );
        
        OGRFieldDefn oFieldPdop("pdop", OFTReal );
        poFeatureDefn->AddFieldDefn( &oFieldPdop );
        
        OGRFieldDefn oFieldAgeofgpsdata("ageofdgpsdata", OFTReal );
        poFeatureDefn->AddFieldDefn( &oFieldAgeofgpsdata );
        
        OGRFieldDefn oFieldDgpsid("dgpsid", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldDgpsid );
    }
    else
    {
        if (gpxGeomType == GPX_TRACK)
            poFeatureDefn->SetGeomType((bEleAs25D) ? wkbMultiLineString25D : wkbMultiLineString);
        else
            poFeatureDefn->SetGeomType((bEleAs25D) ? wkbLineString25D : wkbLineString);
        
        OGRFieldDefn oFieldName("name", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldName );
        
        OGRFieldDefn oFieldCmt("cmt", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldCmt );
        
        OGRFieldDefn oFieldDesc("desc", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldDesc );
        
        OGRFieldDefn oFieldSrc("src", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldSrc );
        
        for(i=1;i<=nMaxLinks;i++)
        {
            char szFieldName[32];
            sprintf(szFieldName, "link%d_href", i);
            OGRFieldDefn oFieldLinkHref( szFieldName, OFTString );
            poFeatureDefn->AddFieldDefn( &oFieldLinkHref );
            
            sprintf(szFieldName, "link%d_text", i);
            OGRFieldDefn oFieldLinkText( szFieldName, OFTString );
            poFeatureDefn->AddFieldDefn( &oFieldLinkText );
            
            sprintf(szFieldName, "link%d_type", i);
            OGRFieldDefn oFieldLinkType( szFieldName, OFTString );
            poFeatureDefn->AddFieldDefn( &oFieldLinkType );
        }
        
        OGRFieldDefn oFieldNumber("number", OFTInteger );
        poFeatureDefn->AddFieldDefn( &oFieldNumber );
        
        OGRFieldDefn oFieldType("type", OFTString );
        poFeatureDefn->AddFieldDefn( &oFieldType );
    }
    
    /* Number of 'standard' GPX attributes */
    nGPXFields = poFeatureDefn->GetFieldCount();
   
    ppoFeatureTab = NULL;
    nFeatureTabIndex = 0;
    nFeatureTabLength = 0;
    pszSubElementName = NULL;
    pszSubElementValue = NULL;
    nSubElementValueLen = 0;
    bStopParsing = FALSE;

    poSRS = new OGRSpatialReference("GEOGCS[\"WGS 84\", "
        "   DATUM[\"WGS_1984\","
        "       SPHEROID[\"WGS 84\",6378137,298.257223563,"
        "           AUTHORITY[\"EPSG\",\"7030\"]],"
        "           AUTHORITY[\"EPSG\",\"6326\"]],"
        "       PRIMEM[\"Greenwich\",0,"
        "           AUTHORITY[\"EPSG\",\"8901\"]],"
        "       UNIT[\"degree\",0.01745329251994328,"
        "           AUTHORITY[\"EPSG\",\"9122\"]],"
        "           AUTHORITY[\"EPSG\",\"4326\"]]");

    poFeature = NULL;

#ifdef HAVE_EXPAT
    oParser = NULL;
#endif

    if (bWriteMode == FALSE)
    {
        fpGPX = VSIFOpenL( pszFilename, "r" );
        if( fpGPX == NULL )
        {
            CPLError(CE_Failure, CPLE_AppDefined, "Cannot open %s", pszFilename);
            return;
        }

        if (poDS->GetUseExtensions() ||
            CSLTestBoolean(CPLGetConfigOption("GPX_USE_EXTENSIONS", "FALSE")))
        {
            LoadExtensionsSchema();
        }
    }
    else
        fpGPX = NULL;

    ResetReading();
}

/************************************************************************/
/*                            ~OGRGPXLayer()                            */
/************************************************************************/

OGRGPXLayer::~OGRGPXLayer()

{
#ifdef HAVE_EXPAT
    if (oParser)
        XML_ParserFree(oParser);
#endif
    poFeatureDefn->Release();
    
    if( poSRS != NULL )
        poSRS->Release();

    CPLFree(pszSubElementName);
    CPLFree(pszSubElementValue);

    int i;
    for(i=nFeatureTabIndex;i<nFeatureTabLength;i++)
        delete ppoFeatureTab[i];
    CPLFree(ppoFeatureTab);

    if (poFeature)
        delete poFeature;

    if (fpGPX)
        VSIFCloseL( fpGPX );
}

#ifdef HAVE_EXPAT

static void XMLCALL startElementCbk(void *pUserData, const char *pszName, const char **ppszAttr)
{
    ((OGRGPXLayer*)pUserData)->startElementCbk(pszName, ppszAttr);
}

static void XMLCALL endElementCbk(void *pUserData, const char *pszName)
{
    ((OGRGPXLayer*)pUserData)->endElementCbk(pszName);
}

static void XMLCALL dataHandlerCbk(void *pUserData, const char *data, int nLen)
{
    ((OGRGPXLayer*)pUserData)->dataHandlerCbk(data, nLen);
}

#endif

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRGPXLayer::ResetReading()

{
    eof = FALSE;
    nNextFID = 0;
    if (fpGPX)
    {
        VSIFSeekL( fpGPX, 0, SEEK_SET );
#ifdef HAVE_EXPAT
        if (oParser)
            XML_ParserFree(oParser);

        oParser = OGRCreateExpatXMLParser();
        XML_SetElementHandler(oParser, ::startElementCbk, ::endElementCbk);
        XML_SetCharacterDataHandler(oParser, ::dataHandlerCbk);
        XML_SetUserData(oParser, this);
#endif
    }
    hasFoundLat = FALSE;
    hasFoundLon = FALSE;
    inInterestingElement = FALSE;
    CPLFree(pszSubElementName);
    pszSubElementName = NULL;
    CPLFree(pszSubElementValue);
    pszSubElementValue = NULL;
    nSubElementValueLen = 0;
    
    int i;
    for(i=nFeatureTabIndex;i<nFeatureTabLength;i++)
        delete ppoFeatureTab[i];
    CPLFree(ppoFeatureTab);
    nFeatureTabIndex = 0;
    nFeatureTabLength = 0;
    ppoFeatureTab = NULL;
    if (poFeature)
        delete poFeature;
    poFeature = NULL;
    multiLineString = NULL;
    lineString = NULL;

    depthLevel = 0;
    interestingDepthLevel = 0;
    
    trkFID = trkSegId = trkSegPtId = 0;
    rteFID = rtePtId = 0;
}

#ifdef HAVE_EXPAT

/************************************************************************/
/*                        startElementCbk()                            */
/************************************************************************/

/** Replace ':' from XML NS element name by '_' more OGR friendly */
static char* OGRGPX_GetOGRCompatibleTagName(const char* pszName)
{
    char* pszModName = CPLStrdup(pszName);
    int i;
    for(i=0;pszModName[i] != 0;i++)
    {
        if (pszModName[i] == ':')
            pszModName[i] = '_';
    }
    return pszModName;
}

void OGRGPXLayer::AddStrToSubElementValue(const char* pszStr)
{
    int len = strlen(pszStr);
    char* pszNewSubElementValue = (char*)
            VSIRealloc(pszSubElementValue, nSubElementValueLen + len + 1);
    if (pszNewSubElementValue == NULL)
    {
        CPLError(CE_Failure, CPLE_OutOfMemory, "Out of memory");
        XML_StopParser(oParser, XML_FALSE);
        bStopParsing = TRUE;
        return;
    }
    pszSubElementValue = pszNewSubElementValue;
    memcpy(pszSubElementValue + nSubElementValueLen, pszStr, len);
    nSubElementValueLen += len;
}

void OGRGPXLayer::startElementCbk(const char *pszName, const char **ppszAttr)
{
    int i;

    if (bStopParsing) return;

    nWithoutEventCounter = 0;

    if ((gpxGeomType == GPX_WPT && strcmp(pszName, "wpt") == 0) ||
        (gpxGeomType == GPX_ROUTE_POINT && strcmp(pszName, "rtept") == 0) ||
        (gpxGeomType == GPX_TRACK_POINT && strcmp(pszName, "trkpt") == 0))
    {
        interestingDepthLevel = depthLevel;

        if (poFeature)
            delete poFeature;

        poFeature = new OGRFeature( poFeatureDefn );
        inInterestingElement = TRUE;
        hasFoundLat = FALSE;
        hasFoundLon = FALSE;
        inExtensions = FALSE;
        inLink = FALSE;
        iCountLink = 0;

        for (i = 0; ppszAttr[i]; i += 2)
        {
            if (strcmp(ppszAttr[i], "lat") == 0)
            {
                hasFoundLat = TRUE;
                latVal = CPLAtof(ppszAttr[i + 1]);
            }
            else if (strcmp(ppszAttr[i], "lon") == 0)
            {
                hasFoundLon = TRUE;
                lonVal = CPLAtof(ppszAttr[i + 1]);
            }
        }

        if (hasFoundLat && hasFoundLon)
        {
            poFeature->SetFID( nNextFID++ );
            poFeature->SetGeometryDirectly( new OGRPoint( lonVal, latVal ) );

            if (gpxGeomType == GPX_ROUTE_POINT)
            {
                rtePtId++;
                poFeature->SetField( 0, rteFID-1);
                poFeature->SetField( 1, rtePtId-1);
            }
            else if (gpxGeomType == GPX_TRACK_POINT)
            {
                trkSegPtId++;

                poFeature->SetField( 0, trkFID-1);
                poFeature->SetField( 1, trkSegId-1);
                poFeature->SetField( 2, trkSegPtId-1);
            }
        }
    }
    else if (gpxGeomType == GPX_TRACK && strcmp(pszName, "trk") == 0)
    {
        interestingDepthLevel = depthLevel;

        if (poFeature)
            delete poFeature;
        inExtensions = FALSE;
        inLink = FALSE;
        iCountLink = 0;
        poFeature = new OGRFeature( poFeatureDefn );
        inInterestingElement = TRUE;

        multiLineString = new OGRMultiLineString ();
        lineString = NULL;

        poFeature->SetFID( nNextFID++ );
        poFeature->SetGeometryDirectly( multiLineString );
    }
    else if (gpxGeomType == GPX_TRACK_POINT && strcmp(pszName, "trk") == 0)
    {
        trkFID++;
        trkSegId = 0;
    }
    else if (gpxGeomType == GPX_TRACK_POINT && strcmp(pszName, "trkseg") == 0)
    {
        trkSegId++;
        trkSegPtId = 0;
    }
    else if (gpxGeomType == GPX_ROUTE && strcmp(pszName, "rte") == 0)
    {
        interestingDepthLevel = depthLevel;
        
        if (poFeature)
            delete poFeature;

        poFeature = new OGRFeature( poFeatureDefn );
        inInterestingElement = TRUE;
        inExtensions = FALSE;
        inLink = FALSE;
        iCountLink = 0;

        lineString = new OGRLineString ();
        poFeature->SetFID( nNextFID++ );
        poFeature->SetGeometryDirectly( lineString );
    }
    else if (gpxGeomType == GPX_ROUTE_POINT && strcmp(pszName, "rte") == 0)
    {
        rteFID++;
        rtePtId = 0;
    }
    else if (inInterestingElement)
    {
        if (gpxGeomType == GPX_TRACK && strcmp(pszName, "trkseg") == 0 &&
            depthLevel == interestingDepthLevel + 1)
        {
            if (multiLineString)
            {
                lineString = new OGRLineString ();
                multiLineString->addGeometryDirectly( lineString );
            }
        }
        else if (gpxGeomType == GPX_TRACK && strcmp(pszName, "trkpt") == 0 &&
                 depthLevel == interestingDepthLevel + 2)
        {
            if (lineString)
            {
                hasFoundLat = FALSE;
                hasFoundLon = FALSE;
                for (i = 0; ppszAttr[i]; i += 2)
                {
                    if (strcmp(ppszAttr[i], "lat") == 0)
                    {
                        hasFoundLat = TRUE;
                        latVal = CPLAtof(ppszAttr[i + 1]);
                    }
                    else if (strcmp(ppszAttr[i], "lon") == 0)
                    {
                        hasFoundLon = TRUE;
                        lonVal = CPLAtof(ppszAttr[i + 1]);
                    }
                }

                if (hasFoundLat && hasFoundLon)
                {
                    lineString->addPoint(lonVal, latVal);
                }
            }
        }
        else if (gpxGeomType == GPX_ROUTE && strcmp(pszName, "rtept") == 0 &&
                 depthLevel == interestingDepthLevel + 1)
        {
            if (lineString)
            {
                hasFoundLat = FALSE;
                hasFoundLon = FALSE;
                for (i = 0; ppszAttr[i]; i += 2)
                {
                    if (strcmp(ppszAttr[i], "lat") == 0)
                    {
                        hasFoundLat = TRUE;
                        latVal = CPLAtof(ppszAttr[i + 1]);
                    }
                    else if (strcmp(ppszAttr[i], "lon") == 0)
                    {
                        hasFoundLon = TRUE;
                        lonVal = CPLAtof(ppszAttr[i + 1]);
                    }
                }

                if (hasFoundLat && hasFoundLon)
                {
                    lineString->addPoint(lonVal, latVal);
                }
            }
        }
        else if (bEleAs25D &&
                 strcmp(pszName, "ele") == 0 &&
                 lineString != NULL &&
                 ((gpxGeomType == GPX_ROUTE && depthLevel == interestingDepthLevel + 2) ||
                  (gpxGeomType == GPX_TRACK && depthLevel == interestingDepthLevel + 3)))
        {
            CPLFree(pszSubElementName);
            pszSubElementName = CPLStrdup(pszName);
        }
        else if (depthLevel == interestingDepthLevel + 1 &&
                 strcmp(pszName, "extensions") == 0)
        {
            if (poDS->GetUseExtensions())
            {
                inExtensions = TRUE;
            }
        }
        else if (depthLevel == interestingDepthLevel + 1 ||
                 (inExtensions && depthLevel == interestingDepthLevel + 2) )
        {
            CPLFree(pszSubElementName);
            pszSubElementName = NULL;
            iCurrentField = -1;
            
            if (strcmp(pszName, "link") == 0)
            {
                iCountLink++;
                if (iCountLink <= nMaxLinks)
                {
                    if (ppszAttr[0] && ppszAttr[1] &&
                        strcmp(ppszAttr[0], "href") == 0)
                    {
                        char szFieldName[32];
                        sprintf(szFieldName, "link%d_href", iCountLink);
                        iCurrentField = poFeatureDefn->GetFieldIndex(szFieldName);
                        poFeature->SetField( iCurrentField, ppszAttr[1]);
                    }
                }
                else
                {
                    static int once = 1;
                    if (once)
                    {
                        once = 0;
                        CPLError(CE_Warning, CPLE_AppDefined,
                                 "GPX driver only reads %d links per element. Others will be ignored. "
                                 "This can be changed with the GPX_N_MAX_LINKS environment variable",
                                 nMaxLinks);
                    }
                }
                inLink = TRUE;
                iCurrentField = -1;
            }
            else
            {
                for( int iField = 0; iField < poFeatureDefn->GetFieldCount(); iField++ )
                {
                    int bMatch;
                    if (iField >= nGPXFields)
                    {
                        char* pszCompatibleName = OGRGPX_GetOGRCompatibleTagName(pszName);
                        bMatch = (strcmp(poFeatureDefn->GetFieldDefn(iField)->GetNameRef(),
                                        pszCompatibleName ) == 0);
                        CPLFree(pszCompatibleName);
                    }
                    else
                        bMatch = (strcmp(poFeatureDefn->GetFieldDefn(iField)->GetNameRef(),
                                        pszName ) == 0);
    
                    if (bMatch)
                    {
                        iCurrentField = iField;
                        pszSubElementName = CPLStrdup(pszName);
                        break;
                    }
                }
            }
        }
        else if (depthLevel == interestingDepthLevel + 2 && inLink)
        {
            char szFieldName[32];
            CPLFree(pszSubElementName);
            pszSubElementName = NULL;
            iCurrentField = -1;
            if (iCountLink <= nMaxLinks)
            {
                if (strcmp(pszName, "type") == 0)
                {
                    sprintf(szFieldName, "link%d_type", iCountLink);
                    iCurrentField = poFeatureDefn->GetFieldIndex(szFieldName);
                    pszSubElementName = CPLStrdup(pszName);
                }
                else if (strcmp(pszName, "text") == 0)
                {
                    sprintf(szFieldName, "link%d_text", iCountLink);
                    iCurrentField = poFeatureDefn->GetFieldIndex(szFieldName);
                    pszSubElementName = CPLStrdup(pszName);
                }
            }
        }
        else if (inExtensions && depthLevel > interestingDepthLevel + 2)
        {
            AddStrToSubElementValue(
               (ppszAttr[0] == NULL) ? CPLSPrintf("<%s>", pszName) :
                                    CPLSPrintf("<%s ", pszName));
            int i;
            for (i = 0; ppszAttr[i]; i += 2)
            {
                AddStrToSubElementValue(
                    CPLSPrintf("%s=\"%s\" ", ppszAttr[i], ppszAttr[i + 1]));
            }
            if (ppszAttr[0] != NULL)
            {
                AddStrToSubElementValue(">");
            }
        }
    }

    depthLevel++;
}

/************************************************************************/
/*                           endElementCbk()                            */
/************************************************************************/

void OGRGPXLayer::endElementCbk(const char *pszName)
{
    if (bStopParsing) return;

    nWithoutEventCounter = 0;

    depthLevel--;

    if (inInterestingElement)
    {
        if ((gpxGeomType == GPX_WPT && strcmp(pszName, "wpt") == 0) ||
            (gpxGeomType == GPX_ROUTE_POINT && strcmp(pszName, "rtept") == 0) ||
            (gpxGeomType == GPX_TRACK_POINT && strcmp(pszName, "trkpt") == 0))
        {
            int bIsValid = (hasFoundLat && hasFoundLon);
            inInterestingElement = FALSE;

            if( bIsValid
                &&  (m_poFilterGeom == NULL
                    || FilterGeometry( poFeature->GetGeometryRef() ) )
                && (m_poAttrQuery == NULL
                    || m_poAttrQuery->Evaluate( poFeature )) )
            {
                if( poFeature->GetGeometryRef() != NULL )
                {
                    poFeature->GetGeometryRef()->assignSpatialReference( poSRS );

                    if (bEleAs25D)
                    {
                        for( int iField = 0; iField < poFeatureDefn->GetFieldCount(); iField++ )
                        {
                            if (strcmp(poFeatureDefn->GetFieldDefn(iField)->GetNameRef(), "ele" ) == 0)
                            {
                                if( poFeature->IsFieldSet( iField ) )
                                {
                                    double val =  poFeature->GetFieldAsDouble( iField);
                                    ((OGRPoint*)poFeature->GetGeometryRef())->setZ(val);
                                    poFeature->GetGeometryRef()->setCoordinateDimension(3);
                                }
                                break;
                            }
                        }
                    }
                }

                ppoFeatureTab = (OGRFeature**)
                        CPLRealloc(ppoFeatureTab,
                                    sizeof(OGRFeature*) * (nFeatureTabLength + 1));
                ppoFeatureTab[nFeatureTabLength] = poFeature;
                nFeatureTabLength++;
            }
            else
            {
                delete poFeature;
            }
            poFeature = NULL;
        }
        else if (gpxGeomType == GPX_TRACK && strcmp(pszName, "trk") == 0)
        {
            inInterestingElement = FALSE;
            if( (m_poFilterGeom == NULL
                    || FilterGeometry( poFeature->GetGeometryRef() ) )
                && (m_poAttrQuery == NULL
                    || m_poAttrQuery->Evaluate( poFeature )) )
            {
                if( poFeature->GetGeometryRef() != NULL )
                {
                    poFeature->GetGeometryRef()->assignSpatialReference( poSRS );
                }

                ppoFeatureTab = (OGRFeature**)
                        CPLRealloc(ppoFeatureTab,
                                    sizeof(OGRFeature*) * (nFeatureTabLength + 1));
                ppoFeatureTab[nFeatureTabLength] = poFeature;
                nFeatureTabLength++;
            }
            else
            {
                delete poFeature;
            }
            poFeature = NULL;
            multiLineString = NULL;
            lineString = NULL;
        }
        else if (gpxGeomType == GPX_TRACK && strcmp(pszName, "trkseg") == 0 &&
                 depthLevel == interestingDepthLevel + 1)
        {
            lineString = NULL;
        }
        else if (gpxGeomType == GPX_ROUTE && strcmp(pszName, "rte") == 0)
        {
            inInterestingElement = FALSE;
            if( (m_poFilterGeom == NULL
                    || FilterGeometry( poFeature->GetGeometryRef() ) )
                && (m_poAttrQuery == NULL
                    || m_poAttrQuery->Evaluate( poFeature )) )
            {
                if( poFeature->GetGeometryRef() != NULL )
                {
                    poFeature->GetGeometryRef()->assignSpatialReference( poSRS );
                }

                ppoFeatureTab = (OGRFeature**)
                        CPLRealloc(ppoFeatureTab,
                                    sizeof(OGRFeature*) * (nFeatureTabLength + 1));
                ppoFeatureTab[nFeatureTabLength] = poFeature;
                nFeatureTabLength++;
            }
            else
            {
                delete poFeature;
            }
            poFeature = NULL;
            lineString = NULL;
        }
        else if (bEleAs25D &&
                 strcmp(pszName, "ele") == 0 &&
                 lineString != NULL &&
                 ((gpxGeomType == GPX_ROUTE && depthLevel == interestingDepthLevel + 2) ||
                 (gpxGeomType == GPX_TRACK && depthLevel == interestingDepthLevel + 3)))
        {
            poFeature->GetGeometryRef()->setCoordinateDimension(3);

            if (nSubElementValueLen)
            {
                pszSubElementValue[nSubElementValueLen] = 0;
    
                double val = CPLAtof(pszSubElementValue);
                int i = lineString->getNumPoints() - 1;
                if (i >= 0)
                    lineString->setPoint(i, lineString->getX(i), lineString->getY(i), val);
            }

            CPLFree(pszSubElementName);
            pszSubElementName = NULL;
            CPLFree(pszSubElementValue);
            pszSubElementValue = NULL;
            nSubElementValueLen = 0;
        }
        else if (depthLevel == interestingDepthLevel + 1 &&
                 strcmp(pszName, "extensions") == 0)
        {
            inExtensions = FALSE;
        }
        else if ((depthLevel == interestingDepthLevel + 1 ||
                 (inExtensions && depthLevel == interestingDepthLevel + 2) ) &&
                 pszSubElementName && strcmp(pszName, pszSubElementName) == 0)
        {
            if (poFeature && pszSubElementValue && nSubElementValueLen)
            {
                pszSubElementValue[nSubElementValueLen] = 0;
                if (strcmp(pszSubElementName, "time") == 0)
                {
                    int year, month, day, hour, minute, TZ;
                    float second;
                    if (OGRParseXMLDateTime(pszSubElementValue, &year, &month, &day, &hour, &minute, &second, &TZ))
                    {
                        poFeature->SetField(iCurrentField, year, month, day, hour, minute, (int)(second + .5), TZ);
                    }
                    else
                    {
                        CPLError(CE_Warning, CPLE_AppDefined,
                                 "Could not parse %s as a valid dateTime", pszSubElementValue);
                    }
                }
                else
                {
                    poFeature->SetField( iCurrentField, pszSubElementValue);
                }
            }
            if (strcmp(pszName, "link") == 0)
                inLink = FALSE;

            CPLFree(pszSubElementName);
            pszSubElementName = NULL;
            CPLFree(pszSubElementValue);
            pszSubElementValue = NULL;
            nSubElementValueLen = 0;
        }
        else if (inLink && depthLevel == interestingDepthLevel + 2)
        {
            if (iCurrentField != -1 && pszSubElementName &&
                strcmp(pszName, pszSubElementName) == 0 && poFeature && pszSubElementValue && nSubElementValueLen)
            {
                pszSubElementValue[nSubElementValueLen] = 0;
                poFeature->SetField( iCurrentField, pszSubElementValue);
            }

            CPLFree(pszSubElementName);
            pszSubElementName = NULL;
            CPLFree(pszSubElementValue);
            pszSubElementValue = NULL;
            nSubElementValueLen = 0;
        }
        else if (inExtensions && depthLevel > interestingDepthLevel + 2)
        {
            AddStrToSubElementValue(CPLSPrintf("</%s>", pszName));
        }
    }
}

/************************************************************************/
/*                          dataHandlerCbk()                            */
/************************************************************************/

void OGRGPXLayer::dataHandlerCbk(const char *data, int nLen)
{
    if (bStopParsing) return;

    nDataHandlerCounter ++;
    if (nDataHandlerCounter >= BUFSIZ)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "File probably corrupted (million laugh pattern)");
        XML_StopParser(oParser, XML_FALSE);
        bStopParsing = TRUE;
        return;
    }

    nWithoutEventCounter = 0;

    if (pszSubElementName)
    {
        if (inExtensions && depthLevel > interestingDepthLevel + 2)
        {
            if (data[0] == '\n')
                return;
        }
        char* pszNewSubElementValue = (char*) VSIRealloc(pszSubElementValue, nSubElementValueLen + nLen + 1);
        if (pszNewSubElementValue == NULL)
        {
            CPLError(CE_Failure, CPLE_OutOfMemory, "Out of memory");
            XML_StopParser(oParser, XML_FALSE);
            bStopParsing = TRUE;
            return;
        }
        pszSubElementValue = pszNewSubElementValue;
        memcpy(pszSubElementValue + nSubElementValueLen, data, nLen);
        nSubElementValueLen += nLen;
        if (nSubElementValueLen > 100000)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Too much data inside one element. File probably corrupted");
            XML_StopParser(oParser, XML_FALSE);
            bStopParsing = TRUE;
        }
    }
}
#endif

/************************************************************************/
/*                           GetNextFeature()                           */
/************************************************************************/

OGRFeature *OGRGPXLayer::GetNextFeature()
{
    if (bWriteMode)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "Cannot read features when writing a GPX file");
        return NULL;
    }

    if (fpGPX == NULL)
        return NULL;

    if (bStopParsing)
        return NULL;

#ifdef HAVE_EXPAT
    if (nFeatureTabIndex < nFeatureTabLength)
    {
        return ppoFeatureTab[nFeatureTabIndex++];
    }
    
    if (VSIFEofL(fpGPX))
        return NULL;
    
    char aBuf[BUFSIZ];
    
    CPLFree(ppoFeatureTab);
    ppoFeatureTab = NULL;
    nFeatureTabLength = 0;
    nFeatureTabIndex = 0;
    nWithoutEventCounter = 0;

    int nDone;
    do
    {
        nDataHandlerCounter = 0;
        unsigned int nLen = (unsigned int)VSIFReadL( aBuf, 1, sizeof(aBuf), fpGPX );
        nDone = VSIFEofL(fpGPX);
        if (XML_Parse(oParser, aBuf, nLen, nDone) == XML_STATUS_ERROR)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "XML parsing of GPX file failed : %s at line %d, column %d",
                     XML_ErrorString(XML_GetErrorCode(oParser)),
                     (int)XML_GetCurrentLineNumber(oParser),
                     (int)XML_GetCurrentColumnNumber(oParser));
            bStopParsing = TRUE;
            break;
        }
        nWithoutEventCounter ++;
    } while (!nDone && nFeatureTabLength == 0 && !bStopParsing && nWithoutEventCounter < 10);

    if (nWithoutEventCounter == 10)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Too much data inside one element. File probably corrupted");
        bStopParsing = TRUE;
    }

    return (nFeatureTabLength) ? ppoFeatureTab[nFeatureTabIndex++] : NULL;
#else
    return NULL;
#endif
}

/************************************************************************/
/*                           GetSpatialRef()                            */
/************************************************************************/

OGRSpatialReference *OGRGPXLayer::GetSpatialRef()

{
    return poSRS;
}

/************************************************************************/
/*                      WriteFeatureAttributes()                        */
/************************************************************************/


static char* OGRGPX_GetXMLCompatibleTagName(const char* pszExtensionsNS,
                                            const char* pszName)
{
    /* Skip "ogr_" for example if NS is "ogr". Usefull for GPX -> GPX roundtrip */
    if (strncmp(pszName, pszExtensionsNS, strlen(pszExtensionsNS)) == 0 &&
        pszName[strlen(pszExtensionsNS)] == '_')
    {
        pszName += strlen(pszExtensionsNS) + 1;
    }

    char* pszModName = CPLStrdup(pszName);
    int i;
    for(i=0;pszModName[i] != 0;i++)
    {
        if (pszModName[i] == ' ')
            pszModName[i] = '_';
    }
    return pszModName;
}

void OGRGPXLayer::WriteFeatureAttributes( OGRFeature *poFeature )
{
    FILE* fp = poDS->GetOutputFP();
    int i;
    
    /* Begin with standard GPX fields */
    for(i=0;i<nGPXFields;i++)
    { 
        OGRFieldDefn *poFieldDefn = poFeatureDefn->GetFieldDefn( i );
        if( poFeature->IsFieldSet( i ) )
        {
            const char* pszName = poFieldDefn->GetNameRef();
            if (strcmp(pszName, "time") == 0)
            {
                int year, month, day, hour, minute, second, TZFlag;
                if (poFeature->GetFieldAsDateTime(i, &year, &month, &day,
                                                  &hour, &minute, &second, &TZFlag))
                {
                    char* pszDate = OGRGetXMLDateTime(year, month, day, hour, minute, second, TZFlag);
                    VSIFPrintf(fp, "  <time>%s</time>\n", pszDate);
                    CPLFree(pszDate);
                }
            }
            else if (strncmp(pszName, "link", 4) == 0)
            {
                if (strstr(pszName, "href"))
                {
                    VSIFPrintf(fp, "  <link href=\"%s\">", poFeature->GetFieldAsString( i ));
                    if( poFeature->IsFieldSet( i + 1 ) )
                        VSIFPrintf(fp, "<text>%s</text>", poFeature->GetFieldAsString( i + 1 ));
                    if( poFeature->IsFieldSet( i + 2 ) )
                        VSIFPrintf(fp, "<type>%s</type>", poFeature->GetFieldAsString( i + 2 ));
                    VSIFPrintf(fp, "</link>\n");
                }
            }
            else
            {
                char* pszValue =
                        OGRGetXML_UTF8_EscapedString(poFeature->GetFieldAsString( i ));
                VSIFPrintf(fp, "  <%s>%s</%s>\n",
                        pszName, pszValue, pszName);
                CPLFree(pszValue);
            }
        }
    }

    /* Write "extra" fields within the <extensions> tag */
    int n = poFeatureDefn->GetFieldCount();
    if (i < n)
    {
        const char* pszExtensionsNS = poDS->GetExtensionsNS();
        VSIFPrintf(fp, "  <extensions>\n");
        for(;i<n;i++)
        {
            OGRFieldDefn *poFieldDefn = poFeatureDefn->GetFieldDefn( i );
            if( poFeature->IsFieldSet( i ) )
            {
                char* compatibleName =
                        OGRGPX_GetXMLCompatibleTagName(pszExtensionsNS, poFieldDefn->GetNameRef());

                /* Remove leading spaces for a numeric field */
                const char *pszRaw = poFeature->GetFieldAsString( i );
                if (poFieldDefn->GetType() == OFTInteger || poFieldDefn->GetType() == OFTReal)
                {
                    while( *pszRaw == ' ' )
                        pszRaw++;
                }

                char *pszEscaped = OGRGetXML_UTF8_EscapedString( pszRaw );
                VSIFPrintf(fp, "    <%s:%s>%s</%s:%s>\n",
                        pszExtensionsNS,
                        compatibleName,
                        pszEscaped,
                        pszExtensionsNS,
                        compatibleName);
                CPLFree(compatibleName);
                CPLFree(pszEscaped);
            }
        }
        VSIFPrintf(fp, "  </extensions>\n");
    }
}

/************************************************************************/
/*                CheckAndFixCoordinatesValidity()                      */
/************************************************************************/

OGRErr OGRGPXLayer::CheckAndFixCoordinatesValidity( double* pdfLatitude, double* pdfLongitude )
{
    if (pdfLatitude != NULL && (*pdfLatitude < -90 || *pdfLatitude > 90))
    {
        static int bFirstWarning = TRUE;
        if (bFirstWarning)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Latitude %f is invalid. Valid range is [-90,90]. This warning will not be issued any more",
                     *pdfLatitude);
            bFirstWarning = FALSE;
        }
        return CE_Failure;
    }

    if (pdfLongitude != NULL && (*pdfLongitude < -180 || *pdfLongitude > 180))
    {
        static int bFirstWarning = TRUE;
        if (bFirstWarning)
        {
            CPLError(CE_Warning, CPLE_AppDefined,
                     "Longitude %f has been modified to fit into range [-180,180]. This warning will not be issued any more",
                     *pdfLongitude);
            bFirstWarning = FALSE;
        }

        if (*pdfLongitude > 180)
            *pdfLongitude -= ((int) ((*pdfLongitude+180)/360)*360);
        else if (*pdfLongitude < -180)
            *pdfLongitude += ((int) (180 - *pdfLongitude)/360)*360;

        return CE_None;
    }

    return CE_None;
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/

OGRErr OGRGPXLayer::CreateFeature( OGRFeature *poFeature )

{
    FILE* fp = poDS->GetOutputFP();
    if (fp == NULL)
        return CE_Failure;
    
    OGRGeometry     *poGeom = poFeature->GetGeometryRef();
    
    if (gpxGeomType == GPX_WPT)
    {
        if (poDS->GetLastGPXGeomTypeWritten() == GPX_ROUTE)
        {
            CPLError( CE_Failure, CPLE_NotSupported,
                        "Cannot write a 'wpt' element after a 'rte' element.\n");
            return OGRERR_FAILURE;
        }
        else
        if (poDS->GetLastGPXGeomTypeWritten() == GPX_TRACK)
        {
            CPLError( CE_Failure, CPLE_NotSupported,
                        "Cannot write a 'wpt' element after a 'trk' element.\n");
            return OGRERR_FAILURE;
        }
        
        poDS->SetLastGPXGeomTypeWritten(gpxGeomType);

        if ( poGeom == NULL )
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                      "Features without geometry not supported by GPX writer in waypoints layer." );
            return OGRERR_FAILURE;
        }

        switch( poGeom->getGeometryType() )
        {
            case wkbPoint:
            case wkbPoint25D:
            {
                OGRPoint* point = (OGRPoint*)poGeom;
                double lat = point->getY();
                double lon = point->getX();
                CheckAndFixCoordinatesValidity(&lat, &lon);
                poDS->AddCoord(lon, lat);
                VSIFPrintf(fp, "<wpt lat=\"%.15f\" lon=\"%.15f\">\n", lat, lon);
                WriteFeatureAttributes(poFeature);
                VSIFPrintf(fp, "</wpt>\n");
                break;
            }
            
            default:
            {
                CPLError( CE_Failure, CPLE_NotSupported,
                    "Geometry type of `%s' not supported fort 'wpt' element.\n",
                    OGRGeometryTypeToName(poGeom->getGeometryType()) );
                return OGRERR_FAILURE;
            }
        }
    }
    else if (gpxGeomType == GPX_ROUTE)
    {
        if (poDS->GetLastGPXGeomTypeWritten() == GPX_TRACK)
        {
            CPLError( CE_Failure, CPLE_NotSupported,
                        "Cannot write a 'rte' element after a 'trk' element.\n");
            return OGRERR_FAILURE;
        }
        
        poDS->SetLastGPXGeomTypeWritten(gpxGeomType);

        OGRLineString* line = NULL;

        if ( poGeom == NULL )
        {
            VSIFPrintf(fp, "<rte>\n");
            WriteFeatureAttributes(poFeature);
            VSIFPrintf(fp, "</rte>\n");
            return OGRERR_NONE;
        }

        switch( poGeom->getGeometryType() )
        {
            case wkbLineString:
            case wkbLineString25D:
            {
                line = (OGRLineString*)poGeom;
                break;
            }

            case wkbMultiLineString:
            case wkbMultiLineString25D:
            {
                int nGeometries = ((OGRGeometryCollection*)poGeom)->getNumGeometries ();
                if (nGeometries == 0)
                {
                    line = NULL;
                }
                else if (nGeometries == 1)
                {
                    line = (OGRLineString*) ( ((OGRGeometryCollection*)poGeom)->getGeometryRef(0) );
                }
                else
                {
                    CPLError( CE_Failure, CPLE_NotSupported,
                            "Multiline with more than one line is not supported for 'rte' element.\n");
                    return OGRERR_FAILURE;
                }
                break;
            }

            default:
            {
                CPLError( CE_Failure, CPLE_NotSupported,
                            "Geometry type of `%s' not supported for 'rte' element.\n",
                            OGRGeometryTypeToName(poGeom->getGeometryType()) );
                return OGRERR_FAILURE;
            }
        }

        int n = (line) ? line->getNumPoints() : 0;
        int i;
        VSIFPrintf(fp, "<rte>\n");
        WriteFeatureAttributes(poFeature);
        for(i=0;i<n;i++)
        {
            double lat = line->getY(i);
            double lon = line->getX(i);
            CheckAndFixCoordinatesValidity(&lat, &lon);
            poDS->AddCoord(lon, lat);
            VSIFPrintf(fp, "  <rtept lat=\"%.15f\" lon=\"%.15f\">\n", lat, lon);
            if (poGeom->getGeometryType() == wkbLineString25D ||
                poGeom->getGeometryType() == wkbMultiLineString25D)
            {
                VSIFPrintf(fp, "    <ele>%f</ele>\n", line->getZ(i));
            }
            VSIFPrintf(fp, "  </rtept>\n");
        }
        VSIFPrintf(fp, "</rte>\n");
    }
    else
    {
        poDS->SetLastGPXGeomTypeWritten(gpxGeomType);

        if (poGeom == NULL)
        {
            VSIFPrintf(fp, "<trk>\n");
            WriteFeatureAttributes(poFeature);
            VSIFPrintf(fp, "</trk>\n");
            return OGRERR_NONE;
        }

        switch( poGeom->getGeometryType() )
        {
            case wkbLineString:
            case wkbLineString25D:
            {
                OGRLineString* line = (OGRLineString*)poGeom;
                int n = line->getNumPoints();
                int i;
                VSIFPrintf(fp, "<trk>\n");
                WriteFeatureAttributes(poFeature);
                VSIFPrintf(fp, "  <trkseg>\n");
                for(i=0;i<n;i++)
                {
                    double lat = line->getY(i);
                    double lon = line->getX(i);
                    CheckAndFixCoordinatesValidity(&lat, &lon);
                    poDS->AddCoord(lon, lat);
                    VSIFPrintf(fp, "    <trkpt lat=\"%.15f\" lon=\"%.15f\">\n", lat, lon);
                    if (line->getGeometryType() == wkbLineString25D)
                    {
                        VSIFPrintf(fp, "        <ele>%f</ele>\n", line->getZ(i));
                    }
                    VSIFPrintf(fp, "    </trkpt>\n");
                }
                VSIFPrintf(fp, "  </trkseg>\n");
                VSIFPrintf(fp, "</trk>\n");
                break;
            }

            case wkbMultiLineString:
            case wkbMultiLineString25D:
            {
                int nGeometries = ((OGRGeometryCollection*)poGeom)->getNumGeometries ();
                VSIFPrintf(fp, "<trk>\n");
                WriteFeatureAttributes(poFeature);
                int j;
                for(j=0;j<nGeometries;j++)
                {
                    OGRLineString* line = (OGRLineString*) ( ((OGRGeometryCollection*)poGeom)->getGeometryRef(j) );
                    int n = (line) ? line->getNumPoints() : 0;
                    int i;
                    VSIFPrintf(fp, "  <trkseg>\n");
                    for(i=0;i<n;i++)
                    {
                        double lat = line->getY(i);
                        double lon = line->getX(i);
                        CheckAndFixCoordinatesValidity(&lat, &lon);
                        poDS->AddCoord(lon, lat);
                        VSIFPrintf(fp, "    <trkpt lat=\"%.15f\" lon=\"%.15f\">\n", lat, lon);
                        if (line->getGeometryType() == wkbLineString25D)
                        {
                            VSIFPrintf(fp, "        <ele>%f</ele>\n", line->getZ(i));
                        }
                        VSIFPrintf(fp, "    </trkpt>\n");
                    }
                    VSIFPrintf(fp, "  </trkseg>\n");
                }
                VSIFPrintf(fp, "</trk>\n");
                break;
            }

            default:
            {
                CPLError( CE_Failure, CPLE_NotSupported,
                            "Geometry type of `%s' not supported for 'trk' element.\n",
                            OGRGeometryTypeToName(poGeom->getGeometryType()) );
                return OGRERR_FAILURE;
            }
        }
    }

    return OGRERR_NONE;
}



/************************************************************************/
/*                            CreateField()                             */
/************************************************************************/


OGRErr OGRGPXLayer::CreateField( OGRFieldDefn *poField, int bApproxOK )

{
    for( int iField = 0; iField < poFeatureDefn->GetFieldCount(); iField++ )
    {
        if (strcmp(poFeatureDefn->GetFieldDefn(iField)->GetNameRef(),
                   poField->GetNameRef() ) == 0)
        {
            return OGRERR_NONE;
        }
    }
    if (poDS->GetUseExtensions() == FALSE)
    {
        CPLError(CE_Failure, CPLE_NotSupported,
                "Field of name '%s' is not supported in GPX schema. "
                 "Use GPX_USE_EXTENSIONS creation option to allow use of the <extensions> element.",
                 poField->GetNameRef());
        return OGRERR_FAILURE;
    }
    else
    {
        poFeatureDefn->AddFieldDefn( poField );
        return OGRERR_NONE;
    }
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRGPXLayer::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,OLCSequentialWrite) )
        return bWriteMode;

    else if( EQUAL(pszCap,OLCStringsAsUTF8) )
        return TRUE;

    else
        return FALSE;
}


/************************************************************************/
/*                       LoadExtensionsSchema()                         */
/************************************************************************/

#ifdef HAVE_EXPAT

static void XMLCALL startElementLoadSchemaCbk(void *pUserData, const char *pszName, const char **ppszAttr)
{
    ((OGRGPXLayer*)pUserData)->startElementLoadSchemaCbk(pszName, ppszAttr);
}

static void XMLCALL endElementLoadSchemaCbk(void *pUserData, const char *pszName)
{
    ((OGRGPXLayer*)pUserData)->endElementLoadSchemaCbk(pszName);
}

static void XMLCALL dataHandlerLoadSchemaCbk(void *pUserData, const char *data, int nLen)
{
    ((OGRGPXLayer*)pUserData)->dataHandlerLoadSchemaCbk(data, nLen);
}


/** This function parses the whole file to detect the extensions fields */
void OGRGPXLayer::LoadExtensionsSchema()
{
    oSchemaParser = OGRCreateExpatXMLParser();
    XML_SetElementHandler(oSchemaParser, ::startElementLoadSchemaCbk, ::endElementLoadSchemaCbk);
    XML_SetCharacterDataHandler(oSchemaParser, ::dataHandlerLoadSchemaCbk);
    XML_SetUserData(oSchemaParser, this);

    VSIFSeekL( fpGPX, 0, SEEK_SET );

    inInterestingElement = FALSE;
    inExtensions = FALSE;
    depthLevel = 0;
    currentFieldDefn = NULL;
    pszSubElementName = NULL;
    pszSubElementValue = NULL;
    nSubElementValueLen = 0;
    nWithoutEventCounter = 0;
    bStopParsing = FALSE;

    char aBuf[BUFSIZ];
    int nDone;
    do
    {
        nDataHandlerCounter = 0;
        unsigned int nLen = (unsigned int)VSIFReadL( aBuf, 1, sizeof(aBuf), fpGPX );
        nDone = VSIFEofL(fpGPX);
        if (XML_Parse(oSchemaParser, aBuf, nLen, nDone) == XML_STATUS_ERROR)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "XML parsing of GPX file failed : %s at line %d, column %d",
                     XML_ErrorString(XML_GetErrorCode(oSchemaParser)),
                     (int)XML_GetCurrentLineNumber(oSchemaParser),
                     (int)XML_GetCurrentColumnNumber(oSchemaParser));
            bStopParsing = TRUE;
            break;
        }
        nWithoutEventCounter ++;
    } while (!nDone && !bStopParsing && nWithoutEventCounter < 10);

    if (nWithoutEventCounter == 10)
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Too much data inside one element. File probably corrupted");
        bStopParsing = TRUE;
    }

    XML_ParserFree(oSchemaParser);
    oSchemaParser = NULL;

    VSIFSeekL( fpGPX, 0, SEEK_SET );
}


/************************************************************************/
/*                  startElementLoadSchemaCbk()                         */
/************************************************************************/


void OGRGPXLayer::startElementLoadSchemaCbk(const char *pszName, const char **ppszAttr)
{
    if (bStopParsing) return;

    nWithoutEventCounter = 0;

    if (gpxGeomType == GPX_WPT && strcmp(pszName, "wpt") == 0)
    {
        inInterestingElement = TRUE;
        inExtensions = FALSE;
        interestingDepthLevel = depthLevel;
    }
    else if (gpxGeomType == GPX_TRACK && strcmp(pszName, "trk") == 0)
    {
        inInterestingElement = TRUE;
        inExtensions = FALSE;
        interestingDepthLevel = depthLevel;
    }
    else if (gpxGeomType == GPX_ROUTE && strcmp(pszName, "rte") == 0)
    {
        inInterestingElement = TRUE;
        inExtensions = FALSE;
        interestingDepthLevel = depthLevel;
    }
    else if (gpxGeomType == GPX_TRACK_POINT && strcmp(pszName, "trkpt") == 0)
    {
        inInterestingElement = TRUE;
        inExtensions = FALSE;
        interestingDepthLevel = depthLevel;
    }
    else if (gpxGeomType == GPX_ROUTE_POINT && strcmp(pszName, "rtept") == 0)
    {
        inInterestingElement = TRUE;
        inExtensions = FALSE;
        interestingDepthLevel = depthLevel;
    }
    else if (inInterestingElement)
    {
        if (depthLevel == interestingDepthLevel + 1 &&
            strcmp(pszName, "extensions") == 0)
        {
            inExtensions = TRUE;
            extensionsDepthLevel = depthLevel;
        }
        else if (inExtensions && depthLevel == extensionsDepthLevel + 1)
        {
            CPLFree(pszSubElementName);
            pszSubElementName = CPLStrdup(pszName);

            int iField;
            for(iField = 0; iField < poFeatureDefn->GetFieldCount(); iField++ )
            {
                int bMatch;
                if (iField >= nGPXFields)
                {
                    char* pszCompatibleName = OGRGPX_GetOGRCompatibleTagName(pszName);
                    bMatch = (strcmp(poFeatureDefn->GetFieldDefn(iField)->GetNameRef(), pszCompatibleName ) == 0);
                    CPLFree(pszCompatibleName);
                }
                else
                    bMatch = (strcmp(poFeatureDefn->GetFieldDefn(iField)->GetNameRef(), pszName ) == 0);
                
                if (bMatch)
                {
                    currentFieldDefn = poFeatureDefn->GetFieldDefn(iField);
                    break;
                }
            }
            if (iField == poFeatureDefn->GetFieldCount())
            {
                char* pszCompatibleName = OGRGPX_GetOGRCompatibleTagName(pszName);
                OGRFieldDefn newFieldDefn(pszCompatibleName, OFTInteger);
                CPLFree(pszCompatibleName);
                
                poFeatureDefn->AddFieldDefn(&newFieldDefn);
                currentFieldDefn = poFeatureDefn->GetFieldDefn(poFeatureDefn->GetFieldCount() - 1);

                if (poFeatureDefn->GetFieldCount() == 100)
                {
                    CPLError(CE_Failure, CPLE_AppDefined,
                            "Too many fields. File probably corrupted");
                    XML_StopParser(oSchemaParser, XML_FALSE);
                    bStopParsing = TRUE;
                }
            }
        }
    }

    depthLevel++;
}


/************************************************************************/
/*                   endElementLoadSchemaCbk()                           */
/************************************************************************/

static int OGRGPXIsInt(const char* pszStr)
{
    int i;

    while(*pszStr == ' ')
        pszStr++;

    for(i=0;pszStr[i];i++)
    {
        if (pszStr[i] == '+' || pszStr[i] == '-')
        {
            if (i != 0)
                return FALSE;
        }
        else if (!(pszStr[i] >= '0' && pszStr[i] <= '9'))
            return FALSE;
    }
    return TRUE;
}


void OGRGPXLayer::endElementLoadSchemaCbk(const char *pszName)
{
    if (bStopParsing) return;

    nWithoutEventCounter = 0;

    depthLevel--;

    if (inInterestingElement)
    {
        if (gpxGeomType == GPX_WPT && strcmp(pszName, "wpt") == 0)
        {
            inInterestingElement = FALSE;
            inExtensions = FALSE;
        }
        else if (gpxGeomType == GPX_TRACK && strcmp(pszName, "trk") == 0)
        {
            inInterestingElement = FALSE;
            inExtensions = FALSE;
        }
        else if (gpxGeomType == GPX_ROUTE && strcmp(pszName, "rte") == 0)
        {
            inInterestingElement = FALSE;
            inExtensions = FALSE;
        }
        else if (gpxGeomType == GPX_TRACK_POINT && strcmp(pszName, "trkpt") == 0)
        {
            inInterestingElement = FALSE;
            inExtensions = FALSE;
        }
        else if (gpxGeomType == GPX_ROUTE_POINT && strcmp(pszName, "rtept") == 0)
        {
            inInterestingElement = FALSE;
            inExtensions = FALSE;
        }
        else if (depthLevel == interestingDepthLevel + 1 &&
                 strcmp(pszName, "extensions") == 0)
        {
            inExtensions = FALSE;
        }
        else if (inExtensions && depthLevel == extensionsDepthLevel + 1 &&
                 pszSubElementName && strcmp(pszName, pszSubElementName) == 0)
        {
            if (pszSubElementValue && nSubElementValueLen && currentFieldDefn)
            {
                pszSubElementValue[nSubElementValueLen] = 0;
                if (currentFieldDefn->GetType() == OFTInteger ||
                    currentFieldDefn->GetType() == OFTReal)
                {
                    char* pszRemainingStr = NULL;
                    CPLStrtod(pszSubElementValue, &pszRemainingStr);
                    if (pszRemainingStr == NULL ||
                        *pszRemainingStr == 0 ||
                        *pszRemainingStr == ' ')
                    {
                        if (currentFieldDefn->GetType() == OFTInteger)
                        {
                            if (OGRGPXIsInt(pszSubElementValue) == FALSE)
                            {
                                currentFieldDefn->SetType(OFTReal);
                            }
                        }
                    }
                    else
                    {
                        currentFieldDefn->SetType(OFTString);
                    }
                }
            }

            CPLFree(pszSubElementName);
            pszSubElementName = NULL;
            CPLFree(pszSubElementValue);
            pszSubElementValue = NULL;
            nSubElementValueLen = 0;
            currentFieldDefn = NULL;
        }
    }
}

/************************************************************************/
/*                   dataHandlerLoadSchemaCbk()                         */
/************************************************************************/

void OGRGPXLayer::dataHandlerLoadSchemaCbk(const char *data, int nLen)
{
    if (bStopParsing) return;

    nDataHandlerCounter ++;
    if (nDataHandlerCounter >= BUFSIZ)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "File probably corrupted (million laugh pattern)");
        XML_StopParser(oSchemaParser, XML_FALSE);
        bStopParsing = TRUE;
        return;
    }

    nWithoutEventCounter = 0;

    if (pszSubElementName)
    {
        char* pszNewSubElementValue = (char*) VSIRealloc(pszSubElementValue, nSubElementValueLen + nLen + 1);
        if (pszNewSubElementValue == NULL)
        {
            CPLError(CE_Failure, CPLE_OutOfMemory, "Out of memory");
            XML_StopParser(oSchemaParser, XML_FALSE);
            bStopParsing = TRUE;
            return;
        }
        pszSubElementValue = pszNewSubElementValue;
        memcpy(pszSubElementValue + nSubElementValueLen, data, nLen);
        nSubElementValueLen += nLen;
        if (nSubElementValueLen > 100000)
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Too much data inside one element. File probably corrupted");
            XML_StopParser(oSchemaParser, XML_FALSE);
            bStopParsing = TRUE;
        }
    }
}
#else
void OGRGPXLayer::LoadExtensionsSchema()
{
}
#endif
