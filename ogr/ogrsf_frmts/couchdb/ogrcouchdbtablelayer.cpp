/******************************************************************************
 * $Id$
 *
 * Project:  CouchDB Translator
 * Purpose:  Implements OGRCouchDBTableLayer class.
 * Author:   Even Rouault, <even dot rouault at mines dash paris dot org>
 *
 ******************************************************************************
 * Copyright (c) 2011, Even Rouault <even dot rouault at mines dash paris dot org>
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

#include "ogr_couchdb.h"
#include "ogrgeojsonreader.h"
#include "ogrgeojsonwriter.h"
#include "ogrgeojsonutils.h"
#include "json_object_private.h" // json_object_iter, complete type required
#include "swq.h"

#include <algorithm>

CPL_CVSID("$Id$");

/************************************************************************/
/*                       OGRCouchDBTableLayer()                         */
/************************************************************************/

OGRCouchDBTableLayer::OGRCouchDBTableLayer(OGRCouchDBDataSource* poDS,
                                           const char* pszName) :
                                                        OGRCouchDBLayer(poDS)

{
    osName = pszName;

    bInTransaction = FALSE;

    eGeomType = wkbUnknown;

    nNextFIDForCreate = -1;
    bHasLoadedMetadata = FALSE;
    bMustWriteMetadata = FALSE;

    bHasInstalledSpatialFilter = FALSE;
    bServerSideSpatialFilteringWorks = TRUE;
    bHasOGRSpatial = -1;

    bServerSideAttributeFilteringWorks = TRUE;
    bHasInstalledAttributeFilter = FALSE;

    nUpdateSeq = -1;

    bExtentValid = FALSE;
    bExtentSet = FALSE;
    dfMinX = 0;
    dfMinY = 0;
    dfMaxX = 0;
    dfMaxY = 0;
}

/************************************************************************/
/*                      ~OGRCouchDBTableLayer()                         */
/************************************************************************/

OGRCouchDBTableLayer::~OGRCouchDBTableLayer()

{
    if( bMustWriteMetadata )
        WriteMetadata();

    for(int i=0;i<(int)aoTransactionFeatures.size();i++)
    {
        json_object_put(aoTransactionFeatures[i]);
    }
}

/************************************************************************/
/*                            ResetReading()                            */
/************************************************************************/

void OGRCouchDBTableLayer::ResetReading()

{
    OGRCouchDBLayer::ResetReading();

    json_object_put(poFeatures);
    poFeatures = NULL;
    aoFeatures.resize(0);
}

/************************************************************************/
/*                           TestCapability()                           */
/************************************************************************/

int OGRCouchDBTableLayer::TestCapability( const char * pszCap )

{
    if( EQUAL(pszCap,OLCFastFeatureCount) )
        return m_poFilterGeom == NULL && m_poAttrQuery == NULL;

    else if( EQUAL(pszCap,OLCFastGetExtent) )
        return bExtentValid;

    else if( EQUAL(pszCap,OLCRandomRead) )
        return TRUE;

    else if( EQUAL(pszCap,OLCSequentialWrite)
             || EQUAL(pszCap,OLCRandomWrite)
             || EQUAL(pszCap,OLCDeleteFeature) )
        return poDS->IsReadWrite();

    else if( EQUAL(pszCap,OLCCreateField) )
        return poDS->IsReadWrite();

    else if( EQUAL(pszCap, OLCTransactions) )
        return poDS->IsReadWrite();

    return OGRCouchDBLayer::TestCapability(pszCap);
}

/************************************************************************/
/*                   FetchNextRowsAnalyseDocs()                         */
/************************************************************************/

int OGRCouchDBTableLayer::FetchNextRowsAnalyseDocs(json_object* poAnswerObj)
{
    if (poAnswerObj == NULL)
        return FALSE;

    if ( !json_object_is_type(poAnswerObj, json_type_object) )
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "FetchNextRowsAnalyseDocs() failed");
        json_object_put(poAnswerObj);
        return FALSE;
    }

    if (poDS->IsError(poAnswerObj, "FetchNextRowsAnalyseDocs() failed"))
    {
        json_object_put(poAnswerObj);
        return FALSE;
    }

    json_object* poRows = json_object_object_get(poAnswerObj, "rows");
    if (poRows == NULL ||
        !json_object_is_type(poRows, json_type_array))
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "FetchNextRowsAnalyseDocs() failed");
        json_object_put(poAnswerObj);
        return FALSE;
    }

    int nRows = json_object_array_length(poRows);
    for(int i=0;i<nRows;i++)
    {
        json_object* poRow = json_object_array_get_idx(poRows, i);
        if ( poRow == NULL ||
             !json_object_is_type(poRow, json_type_object) )
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "FetchNextRowsAnalyseDocs() failed");
            json_object_put(poAnswerObj);
            return FALSE;
        }

        json_object* poDoc = json_object_object_get(poRow, "doc");
        if ( poDoc == NULL ||
             !json_object_is_type(poDoc, json_type_object) )
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "FetchNextRowsAnalyseDocs() failed");
            json_object_put(poAnswerObj);
            return FALSE;
        }

        json_object* poId = json_object_object_get(poDoc, "_id");
        const char* pszId = json_object_get_string(poId);
        if (pszId != NULL && strncmp(pszId, "_design/", 8) != 0)
        {
            aoFeatures.push_back(poDoc);
        }
    }

    bEOF = nRows < GetFeaturesToFetch();

    poFeatures = poAnswerObj;

    return TRUE;
}

/************************************************************************/
/*                   FetchNextRowsSpatialFilter()                       */
/************************************************************************/

int OGRCouchDBTableLayer::FetchNextRowsSpatialFilter()
{
    if (bHasInstalledSpatialFilter)
    {
        bHasInstalledSpatialFilter = FALSE;

        CPLAssert(nOffset == 0);

        aosIdsToFetch.resize(0);

        if (bHasOGRSpatial < 0)
        {
            CPLString osURI("/");
            osURI += osName;
            osURI += "/_design/ogr_spatial";

            json_object* poAnswerObj = poDS->GET(osURI);
            bHasOGRSpatial = (poAnswerObj != NULL &&
                json_object_is_type(poAnswerObj, json_type_object) &&
                json_object_object_get(poAnswerObj, "spatial") != NULL);
            json_object_put(poAnswerObj);
            if (!bHasOGRSpatial)
            {
                CPLDebug("CouchDB",
                         "Geocouch not working --> client-side spatial filtering");
                bServerSideSpatialFilteringWorks = FALSE;
                return FALSE;
            }
        }

        OGREnvelope sEnvelope;
        m_poFilterGeom->getEnvelope( &sEnvelope );

        CPLString osURI("/");
        osURI += osName;
        osURI += "/_design/ogr_spatial/_spatial/spatial?bbox=";
        osURI += CPLSPrintf("%.9f,%.9f,%.9f,%.9f",
                            sEnvelope.MinX, sEnvelope.MinY,
                            sEnvelope.MaxX, sEnvelope.MaxY);

        json_object* poAnswerObj = poDS->GET(osURI);
        if (poAnswerObj == NULL)
        {
            CPLDebug("CouchDB",
                     "Geocouch not working --> client-side spatial filtering");
            bServerSideSpatialFilteringWorks = FALSE;
            return FALSE;
        }

        if ( !json_object_is_type(poAnswerObj, json_type_object) )
        {
            CPLDebug("CouchDB",
                     "Geocouch not working --> client-side spatial filtering");
            bServerSideSpatialFilteringWorks = FALSE;
            CPLError(CE_Failure, CPLE_AppDefined,
                     "FetchNextRowsSpatialFilter() failed");
            json_object_put(poAnswerObj);
            return FALSE;
        }

        /* Catch error for a non geocouch database */
        json_object* poError = json_object_object_get(poAnswerObj, "error");
        json_object* poReason = json_object_object_get(poAnswerObj, "reason");

        const char* pszError = json_object_get_string(poError);
        const char* pszReason = json_object_get_string(poReason);

        if (pszError && pszReason && strcmp(pszError, "not_found") == 0 &&
            strcmp(pszReason, "Document is missing attachment") == 0)
        {
            CPLDebug("CouchDB",
                     "Geocouch not working --> client-side spatial filtering");
            bServerSideSpatialFilteringWorks = FALSE;
            json_object_put(poAnswerObj);
            return FALSE;
        }

        if (poDS->IsError(poAnswerObj, "FetchNextRowsSpatialFilter() failed"))
        {
            CPLDebug("CouchDB",
                     "Geocouch not working --> client-side spatial filtering");
            bServerSideSpatialFilteringWorks = FALSE;
            json_object_put(poAnswerObj);
            return FALSE;
        }

        json_object* poRows = json_object_object_get(poAnswerObj, "rows");
        if (poRows == NULL ||
            !json_object_is_type(poRows, json_type_array))
        {
            CPLDebug("CouchDB",
                     "Geocouch not working --> client-side spatial filtering");
            bServerSideSpatialFilteringWorks = FALSE;
            CPLError(CE_Failure, CPLE_AppDefined,
                     "FetchNextRowsSpatialFilter() failed");
            json_object_put(poAnswerObj);
            return FALSE;
        }

        int nRows = json_object_array_length(poRows);
        for(int i=0;i<nRows;i++)
        {
            json_object* poRow = json_object_array_get_idx(poRows, i);
            if ( poRow == NULL ||
                !json_object_is_type(poRow, json_type_object) )
            {
                CPLError(CE_Failure, CPLE_AppDefined,
                         "FetchNextRowsSpatialFilter() failed");
                json_object_put(poAnswerObj);
                return FALSE;
            }

            json_object* poId = json_object_object_get(poRow, "id");
            const char* pszId = json_object_get_string(poId);
            if (pszId != NULL)
            {
                aosIdsToFetch.push_back(pszId);
            }
        }

        std::sort(aosIdsToFetch.begin(), aosIdsToFetch.end());

        json_object_put(poAnswerObj);
    }

    CPLString osContent("{\"keys\":[");
    int nLimit = MIN(nOffset + GetFeaturesToFetch(), (int)aosIdsToFetch.size());
    for(int i=nOffset;i<nLimit;i++)
    {
        if (i > nOffset)
            osContent += ",";
        osContent += "\"";
        osContent += aosIdsToFetch[i];
        osContent += "\"";
    }
    osContent += "]}";

    CPLString osURI("/");
    osURI += osName;
    osURI += "/_all_docs?include_docs=true";
    json_object* poAnswerObj = poDS->POST(osURI, osContent);
    return FetchNextRowsAnalyseDocs(poAnswerObj);
}

/************************************************************************/
/*                HasFilterOnFieldOrCreateIfNecessary()                 */
/************************************************************************/

int OGRCouchDBTableLayer::HasFilterOnFieldOrCreateIfNecessary(const char* pszFieldName)
{
    std::map<CPLString, int>::iterator oIter = oMapFilterFields.find(pszFieldName);
    if (oIter != oMapFilterFields.end())
        return oIter->second;

    CPLString osURI("/");
    osURI += osName;
    osURI += "/_design/ogr_filter_";
    osURI += pszFieldName;

    int bFoundFilter = FALSE;

    json_object* poAnswerObj = poDS->GET(osURI);
    if (poAnswerObj &&
        json_object_is_type(poAnswerObj, json_type_object) &&
        json_object_object_get(poAnswerObj, "views") != NULL)
    {
        bFoundFilter = TRUE;
    }
    json_object_put(poAnswerObj);

    if (!bFoundFilter)
    {
        json_object* poDoc = json_object_new_object();
        json_object* poViews = json_object_new_object();
        json_object* poFilter = json_object_new_object();

        CPLString osMap;

        if (bGeoJSONDocument)
        {
            osMap = CPLSPrintf("function(doc) { if (doc && doc.properties && doc.properties.%s) emit(doc.properties.%s, null); }",
                               pszFieldName, pszFieldName);
        }
        else
        {
            osMap = CPLSPrintf("function(doc) { if (doc && doc.%s) emit(doc.%s, null); }",
                               pszFieldName, pszFieldName);
        }

        json_object_object_add(poDoc, "views", poViews);
        json_object_object_add(poViews, "filter", poFilter);
        json_object_object_add(poFilter, "map", json_object_new_string(osMap));

        json_object* poAnswerObj = poDS->PUT(osURI,
                                            json_object_to_json_string(poDoc));

        json_object_put(poDoc);

        if (poDS->IsOK(poAnswerObj, "Filter creation failed"))
        {
            bFoundFilter = TRUE;
            bMustWriteMetadata = TRUE;
            nUpdateSeq++;
        }

        json_object_put(poAnswerObj);
    }

    oMapFilterFields[pszFieldName] = bFoundFilter;

    return bFoundFilter;
}

/************************************************************************/
/*                         OGRCouchDBGetOpStr()                         */
/************************************************************************/

static const char* OGRCouchDBGetOpStr(int nOperation)
{
    switch(nOperation)
    {
        case SWQ_EQ: return "=";
        case SWQ_GE: return ">=";
        case SWQ_LE: return "<=";
        case SWQ_GT: return ">";
        case SWQ_LT: return "<";
        default:     return "unknown op";
    }
}

/************************************************************************/
/*                   FetchNextRowsAttributeFilter()                     */
/************************************************************************/

int OGRCouchDBTableLayer::FetchNextRowsAttributeFilter()
{
    if (bHasInstalledAttributeFilter)
    {
        osURIAttributeFilter = "";
        bHasInstalledAttributeFilter = FALSE;

        CPLAssert(nOffset == 0);

        int bCanHandleFilter = FALSE;

        swq_expr_node * pNode = (swq_expr_node *) m_poAttrQuery->GetSWGExpr();
        if (pNode->eNodeType == SNT_OPERATION &&
            (pNode->nOperation == SWQ_EQ ||
             pNode->nOperation == SWQ_GE ||
             pNode->nOperation == SWQ_LE ||
             pNode->nOperation == SWQ_GT ||
             pNode->nOperation == SWQ_LT) &&
            pNode->nSubExprCount == 2 &&
            pNode->papoSubExpr[0]->eNodeType == SNT_COLUMN &&
            pNode->papoSubExpr[1]->eNodeType == SNT_CONSTANT)
        {
            int nIndex = pNode->papoSubExpr[0]->field_index;
            swq_field_type eType = pNode->papoSubExpr[1]->field_type;
            const char* pszFieldName = poFeatureDefn->GetFieldDefn(nIndex)->GetNameRef();

            if (pNode->nOperation == SWQ_EQ &&
                nIndex == _ID_FIELD && eType == SWQ_STRING)
            {
                bCanHandleFilter = TRUE;

                osURIAttributeFilter = "/";
                osURIAttributeFilter += osName;
                osURIAttributeFilter += "/_all_docs?";
            }
            else if (nIndex >= FIRST_FIELD &&
                (eType == SWQ_STRING || eType == SWQ_INTEGER || eType == SWQ_FLOAT))
            {
                int bFoundFilter = HasFilterOnFieldOrCreateIfNecessary(pszFieldName);
                if (bFoundFilter)
                {
                    bCanHandleFilter = TRUE;

                    osURIAttributeFilter = "/";
                    osURIAttributeFilter += osName;
                    osURIAttributeFilter += "/_design/ogr_filter_";
                    osURIAttributeFilter += pszFieldName;
                    osURIAttributeFilter += "/_view/filter?";
                }
            }

            if (bCanHandleFilter)
            {
                const char* pszVal = pNode->papoSubExpr[1]->string_value;
                double dfVal = pNode->papoSubExpr[1]->float_value;
                int nVal = pNode->papoSubExpr[1]->int_value;
                const char* pszOp = OGRCouchDBGetOpStr(pNode->nOperation);
                if (eType == SWQ_STRING)
                    CPLDebug("CouchDB", "Evaluating %s %s '%s'", pszFieldName, pszOp, pszVal);
                else if (eType == SWQ_INTEGER)
                    CPLDebug("CouchDB", "Evaluating %s %s %d", pszFieldName, pszOp, nVal);
                else if (eType == SWQ_FLOAT)
                    CPLDebug("CouchDB", "Evaluating %s %s %.9f", pszFieldName, pszOp, dfVal);

                if (pNode->nOperation == SWQ_EQ)
                {
                    osURIAttributeFilter += "key=";
                }
                else if (pNode->nOperation == SWQ_GE ||
                            pNode->nOperation == SWQ_GT)
                {
                    osURIAttributeFilter += "startkey=";
                }
                else if (pNode->nOperation == SWQ_LE ||
                            pNode->nOperation == SWQ_LT)
                {
                    osURIAttributeFilter += "endkey=";
                }
                if (eType == SWQ_STRING)
                {
                    osURIAttributeFilter += "\"";
                    osURIAttributeFilter += pszVal;
                    osURIAttributeFilter += "\"";
                }
                else if (eType == SWQ_INTEGER)
                {
                    osURIAttributeFilter += CPLSPrintf("%d", nVal);
                }
                else if (eType == SWQ_FLOAT)
                {
                    osURIAttributeFilter += CPLSPrintf("%.9f", dfVal);
                }
            }
        }
        else if (pNode->eNodeType == SNT_OPERATION &&
                 pNode->nOperation == SWQ_AND &&
                 pNode->nSubExprCount == 2 &&
                 pNode->papoSubExpr[0]->eNodeType == SNT_OPERATION &&
                 pNode->papoSubExpr[1]->eNodeType == SNT_OPERATION &&
                 (((pNode->papoSubExpr[0]->nOperation == SWQ_GE ||
                    pNode->papoSubExpr[0]->nOperation == SWQ_GT) &&
                   (pNode->papoSubExpr[1]->nOperation == SWQ_LE ||
                    pNode->papoSubExpr[1]->nOperation == SWQ_LT)) ||
                  ((pNode->papoSubExpr[0]->nOperation == SWQ_LE ||
                    pNode->papoSubExpr[0]->nOperation == SWQ_LT) &&
                   (pNode->papoSubExpr[1]->nOperation == SWQ_GE ||
                    pNode->papoSubExpr[1]->nOperation == SWQ_GT))) &&
                pNode->papoSubExpr[0]->nSubExprCount == 2 &&
                pNode->papoSubExpr[1]->nSubExprCount == 2 &&
                pNode->papoSubExpr[0]->papoSubExpr[0]->eNodeType == SNT_COLUMN &&
                pNode->papoSubExpr[0]->papoSubExpr[1]->eNodeType == SNT_CONSTANT &&
                pNode->papoSubExpr[1]->papoSubExpr[0]->eNodeType == SNT_COLUMN &&
                pNode->papoSubExpr[1]->papoSubExpr[1]->eNodeType == SNT_CONSTANT)
        {
            int nIndex0 = pNode->papoSubExpr[0]->papoSubExpr[0]->field_index;
            swq_field_type eType0 = pNode->papoSubExpr[0]->papoSubExpr[1]->field_type;
            int nIndex1 = pNode->papoSubExpr[1]->papoSubExpr[0]->field_index;
            swq_field_type eType1 = pNode->papoSubExpr[1]->papoSubExpr[1]->field_type;
            const char* pszFieldName = poFeatureDefn->GetFieldDefn(nIndex0)->GetNameRef();

            if (nIndex0 == nIndex1 && eType0 == eType1 &&
                nIndex0 == _ID_FIELD && eType0 == SWQ_STRING)
            {
                bCanHandleFilter = TRUE;

                osURIAttributeFilter = "/";
                osURIAttributeFilter += osName;
                osURIAttributeFilter += "/_all_docs?";
            }
            else if (nIndex0 == nIndex1 && eType0 == eType1 &&
                nIndex0 >= FIRST_FIELD &&
                (eType0 == SWQ_STRING || eType0 == SWQ_INTEGER || eType0 == SWQ_FLOAT))
            {
                int bFoundFilter = HasFilterOnFieldOrCreateIfNecessary(pszFieldName);
                if (bFoundFilter)
                {
                    bCanHandleFilter = TRUE;

                    osURIAttributeFilter = "/";
                    osURIAttributeFilter += osName;
                    osURIAttributeFilter += "/_design/ogr_filter_";
                    osURIAttributeFilter += pszFieldName;
                    osURIAttributeFilter += "/_view/filter?";
                }
            }

            if (bCanHandleFilter)
            {
                swq_field_type eType = eType0;
                const char* pszVal0 = pNode->papoSubExpr[0]->papoSubExpr[1]->string_value;
                double dfVal0 = pNode->papoSubExpr[0]->papoSubExpr[1]->float_value;
                int nVal0 = pNode->papoSubExpr[0]->papoSubExpr[1]->int_value;
                const char* pszVal1 = pNode->papoSubExpr[1]->papoSubExpr[1]->string_value;
                double dfVal1 = pNode->papoSubExpr[1]->papoSubExpr[1]->float_value;
                int nVal1 = pNode->papoSubExpr[1]->papoSubExpr[1]->int_value;
                int nOperation0 = pNode->papoSubExpr[0]->nOperation;
                int nOperation1 = pNode->papoSubExpr[1]->nOperation;

                const char* pszOp0 = OGRCouchDBGetOpStr(nOperation0);
                const char* pszOp1 = OGRCouchDBGetOpStr(nOperation1);

                if (eType == SWQ_STRING)
                    CPLDebug("CouchDB", "Evaluating %s %s '%s' AND %s %s '%s'",
                             pszFieldName, pszOp0, pszVal0,
                             pszFieldName, pszOp1, pszVal1);
                else if (eType == SWQ_INTEGER)
                    CPLDebug("CouchDB", "Evaluating %s %s %d AND %s %s %d",
                             pszFieldName, pszOp0, nVal0,
                             pszFieldName, pszOp1, nVal1);
                else if (eType == SWQ_FLOAT)
                    CPLDebug("CouchDB", "Evaluating %s %s %.9f AND %s %s %.9f",
                             pszFieldName, pszOp0, dfVal0,
                             pszFieldName, pszOp1, dfVal1);

                if (nOperation0 == SWQ_GE ||
                    nOperation0 == SWQ_GT)
                {
                    osURIAttributeFilter += "startkey=";
                }
                else if (nOperation0 == SWQ_LE ||
                            nOperation0 == SWQ_LT)
                {
                    osURIAttributeFilter += "endkey=";
                }
                if (eType == SWQ_STRING)
                {
                    osURIAttributeFilter += "\"";
                    osURIAttributeFilter += pszVal0;
                    osURIAttributeFilter += "\"";
                }
                else if (eType == SWQ_INTEGER)
                {
                    osURIAttributeFilter += CPLSPrintf("%d", nVal0);
                }
                else if (eType == SWQ_FLOAT)
                {
                    osURIAttributeFilter += CPLSPrintf("%.9f", dfVal0);
                }
                osURIAttributeFilter += "&";
                if (nOperation1 == SWQ_GE ||
                    nOperation1 == SWQ_GT)
                {
                    osURIAttributeFilter += "startkey=";
                }
                else if (nOperation1 == SWQ_LE ||
                            nOperation1 == SWQ_LT)
                {
                    osURIAttributeFilter += "endkey=";
                }
                if (eType == SWQ_STRING)
                {
                    osURIAttributeFilter += "\"";
                    osURIAttributeFilter += pszVal1;
                    osURIAttributeFilter += "\"";
                }
                else if (eType == SWQ_INTEGER)
                {
                    osURIAttributeFilter += CPLSPrintf("%d", nVal1);
                }
                else if (eType == SWQ_FLOAT)
                {
                    osURIAttributeFilter += CPLSPrintf("%.9f", dfVal1);
                }
            }
        }
        else if (pNode->eNodeType == SNT_OPERATION &&
                 pNode->nOperation == SWQ_BETWEEN &&
                 pNode->nSubExprCount == 3 &&
                 pNode->papoSubExpr[0]->eNodeType == SNT_COLUMN &&
                 pNode->papoSubExpr[1]->eNodeType == SNT_CONSTANT &&
                 pNode->papoSubExpr[2]->eNodeType == SNT_CONSTANT)
        {
            int nIndex = pNode->papoSubExpr[0]->field_index;
            swq_field_type eType = pNode->papoSubExpr[0]->field_type;
            const char* pszFieldName = poFeatureDefn->GetFieldDefn(nIndex)->GetNameRef();

            if (nIndex == _ID_FIELD && eType == SWQ_STRING)
            {
                bCanHandleFilter = TRUE;

                osURIAttributeFilter = "/";
                osURIAttributeFilter += osName;
                osURIAttributeFilter += "/_all_docs?";
            }
            else if (nIndex >= FIRST_FIELD &&
                (eType == SWQ_STRING || eType == SWQ_INTEGER || eType == SWQ_FLOAT))
            {
                int bFoundFilter = HasFilterOnFieldOrCreateIfNecessary(pszFieldName);
                if (bFoundFilter)
                {
                    bCanHandleFilter = TRUE;

                    osURIAttributeFilter = "/";
                    osURIAttributeFilter += osName;
                    osURIAttributeFilter += "/_design/ogr_filter_";
                    osURIAttributeFilter += pszFieldName;
                    osURIAttributeFilter += "/_view/filter?";
                }
            }

            if (bCanHandleFilter)
            {
                const char* pszVal0 = pNode->papoSubExpr[1]->string_value;
                double dfVal0 = pNode->papoSubExpr[1]->float_value;
                int nVal0 = pNode->papoSubExpr[1]->int_value;
                const char* pszVal1 = pNode->papoSubExpr[2]->string_value;
                double dfVal1 = pNode->papoSubExpr[2]->float_value;
                int nVal1 = pNode->papoSubExpr[2]->int_value;

                if (eType == SWQ_STRING)
                    CPLDebug("CouchDB", "Evaluating %s BETWEEN '%s' AND '%s'",
                             pszFieldName, pszVal0, pszVal1);
                else if (eType == SWQ_INTEGER)
                    CPLDebug("CouchDB", "Evaluating %s BETWEEN %d AND %d",
                             pszFieldName, nVal0, nVal1);
                else if (eType == SWQ_FLOAT)
                    CPLDebug("CouchDB", "Evaluating %s BETWEEN %.9f AND %.9f",
                             pszFieldName, dfVal0, dfVal1);

                osURIAttributeFilter += "startkey=";
                if (eType == SWQ_STRING)
                {
                    osURIAttributeFilter += "\"";
                    osURIAttributeFilter += pszVal0;
                    osURIAttributeFilter += "\"";
                }
                else if (eType == SWQ_INTEGER)
                {
                    osURIAttributeFilter += CPLSPrintf("%d", nVal0);
                }
                else if (eType == SWQ_FLOAT)
                {
                    osURIAttributeFilter += CPLSPrintf("%.9f", dfVal0);
                }
                osURIAttributeFilter += "&";
                osURIAttributeFilter += "endkey=";
                if (eType == SWQ_STRING)
                {
                    osURIAttributeFilter += "\"";
                    osURIAttributeFilter += pszVal1;
                    osURIAttributeFilter += "\"";
                }
                else if (eType == SWQ_INTEGER)
                {
                    osURIAttributeFilter += CPLSPrintf("%d", nVal1);
                }
                else if (eType == SWQ_FLOAT)
                {
                    osURIAttributeFilter += CPLSPrintf("%.9f", dfVal1);
                }
            }
        }

        if (!bCanHandleFilter)
        {
            CPLDebug("CouchDB",
                     "Turning to client-side attribute filtering");
            bServerSideAttributeFilteringWorks = FALSE;
            return FALSE;
        }
    }

    CPLString osURI(osURIAttributeFilter);
    osURI += CPLSPrintf("&limit=%d&skip=%d&include_docs=true",
                        GetFeaturesToFetch(), nOffset);
    json_object* poAnswerObj = poDS->GET(osURI);
    return FetchNextRowsAnalyseDocs(poAnswerObj);
}

/************************************************************************/
/*                           FetchNextRows()                            */
/************************************************************************/

int OGRCouchDBTableLayer::FetchNextRows()
{
    json_object_put(poFeatures);
    poFeatures = NULL;
    aoFeatures.resize(0);

    if( m_poFilterGeom != NULL && bServerSideSpatialFilteringWorks )
    {
        int bRet = FetchNextRowsSpatialFilter();
        if (bRet || bServerSideSpatialFilteringWorks)
            return bRet;
    }

    if( m_poAttrQuery != NULL && bServerSideAttributeFilteringWorks )
    {
        int bRet = FetchNextRowsAttributeFilter();
        if (bRet || bServerSideAttributeFilteringWorks)
            return bRet;
    }

    CPLString osURI("/");
    osURI += osName;
    osURI += CPLSPrintf("/_all_docs?limit=%d&skip=%d&include_docs=true",
                        GetFeaturesToFetch(), nOffset);
    json_object* poAnswerObj = poDS->GET(osURI);
    return FetchNextRowsAnalyseDocs(poAnswerObj);
}


/************************************************************************/
/*                            GetFeature()                              */
/************************************************************************/

OGRFeature * OGRCouchDBTableLayer::GetFeature( long nFID )
{
    GetLayerDefn();

    CPLString osURI("/");
    osURI += osName;
    osURI += "/";
    osURI += CPLSPrintf("%09d", (int)nFID);
    json_object* poAnswerObj = poDS->GET(osURI);
    if (poAnswerObj == NULL)
        return NULL;

    if ( !json_object_is_type(poAnswerObj, json_type_object) )
    {
        CPLError(CE_Failure, CPLE_AppDefined, "GetFeature(%ld) failed",
                 nFID);
        json_object_put(poAnswerObj);
        return NULL;
    }

    if ( poDS->IsError(poAnswerObj, CPLSPrintf("GetFeature(%ld) failed", nFID)) )
    {
        json_object_put(poAnswerObj);
        return NULL;
    }

    OGRFeature* poFeature = TranslateFeature( poAnswerObj );

    json_object_put( poAnswerObj );

    return poFeature;
}

/************************************************************************/
/*                           GetLayerDefn()                             */
/************************************************************************/

OGRFeatureDefn * OGRCouchDBTableLayer::GetLayerDefn()
{
    if (poFeatureDefn == NULL)
        LoadMetadata();

    if (poFeatureDefn == NULL)
    {
        poFeatureDefn = new OGRFeatureDefn( osName );
        poFeatureDefn->Reference();

        poFeatureDefn->SetGeomType(eGeomType);

        OGRFieldDefn oFieldId("_id", OFTString);
        poFeatureDefn->AddFieldDefn(&oFieldId);

        OGRFieldDefn oFieldRev("_rev", OFTString);
        poFeatureDefn->AddFieldDefn(&oFieldRev);

        if (nNextFIDForCreate == 0)
        {
            return poFeatureDefn;
        }

        CPLString osURI("/");
        osURI += osName;
        osURI += "/_all_docs?limit=10&include_docs=true";
        json_object* poAnswerObj = poDS->GET(osURI);
        if (poAnswerObj == NULL)
            return poFeatureDefn;

        if ( !json_object_is_type(poAnswerObj, json_type_object) )
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Layer definition creation failed");
            json_object_put(poAnswerObj);
            return poFeatureDefn;
        }

        if (poDS->IsError(poAnswerObj, "Layer definition creation failed"))
        {
            json_object_put(poAnswerObj);
            return poFeatureDefn;
        }

        json_object* poRows = json_object_object_get(poAnswerObj, "rows");
        if (poRows == NULL ||
            !json_object_is_type(poRows, json_type_array))
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Layer definition creation failed");
            json_object_put(poAnswerObj);
            return poFeatureDefn;
        }

        int nRows = json_object_array_length(poRows);

        json_object* poRow = NULL;
        for(int i=0;i<nRows;i++)
        {
            json_object* poTmpRow = json_object_array_get_idx(poRows, i);
            if (poTmpRow != NULL &&
                json_object_is_type(poTmpRow, json_type_object))
            {
                json_object* poId = json_object_object_get(poTmpRow, "id");
                const char* pszId = json_object_get_string(poId);
                if (pszId != NULL && pszId[0] != '_')
                {
                    poRow = poTmpRow;
                    break;
                }
            }
        }

        if ( poRow == NULL )
        {
            json_object_put(poAnswerObj);
            return poFeatureDefn;
        }

        json_object* poDoc = json_object_object_get(poRow, "doc");
        if ( poDoc == NULL ||
             !json_object_is_type(poDoc, json_type_object) )
        {
            CPLError(CE_Failure, CPLE_AppDefined,
                     "Layer definition creation failed");
            json_object_put(poAnswerObj);
            return poFeatureDefn;
        }

    /* -------------------------------------------------------------------- */
    /*      Read collection of properties.                                  */
    /* -------------------------------------------------------------------- */
        json_object* poObjProps = json_object_object_get( poDoc,
                                                          "properties" );
        json_object_iter it;
        it.key = NULL;
        it.val = NULL;
        it.entry = NULL;
        if( NULL != poObjProps )
        {
            json_object_object_foreachC( poObjProps, it )
            {
                if( -1 == poFeatureDefn->GetFieldIndex( it.key ) )
                {
                    OGRFieldDefn fldDefn( it.key,
                        GeoJSONPropertyToFieldType( it.val ) );
                    poFeatureDefn->AddFieldDefn( &fldDefn );
                }
            }
        }
        else
        {
            bGeoJSONDocument = FALSE;

            json_object_object_foreachC( poDoc, it )
            {
                if( strcmp(it.key, "_id") != 0 &&
                    strcmp(it.key, "_rev") != 0 &&
                    strcmp(it.key, "geometry") != 0 &&
                    -1 == poFeatureDefn->GetFieldIndex( it.key ) )
                {
                    OGRFieldDefn fldDefn( it.key,
                        GeoJSONPropertyToFieldType( it.val ) );
                    poFeatureDefn->AddFieldDefn( &fldDefn );
                }
            }
        }

        if( json_object_object_get( poDoc, "geometry" ) == NULL )
        {
            eGeomType = wkbNone;
            poFeatureDefn->SetGeomType(eGeomType);
        }

        json_object_put(poAnswerObj);
    }

    return poFeatureDefn;
}

/************************************************************************/
/*                          GetFeatureCount()                           */
/************************************************************************/

int OGRCouchDBTableLayer::GetFeatureCount(int bForce)
{
    GetLayerDefn();

    if (m_poFilterGeom != NULL || m_poAttrQuery != NULL)
        return OGRCouchDBLayer::GetFeatureCount(bForce);

    return GetTotalFeatureCount();
}

/************************************************************************/
/*                          GetFeatureCount()                           */
/************************************************************************/

int OGRCouchDBTableLayer::GetTotalFeatureCount()
{
    int nTotalRows = -1;

    CPLString osURI("/");
    osURI += osName;
    osURI += "/_all_docs?startkey_docid=_&endkey_docid=_zzzzzzzzzzzzzzz";
    json_object* poAnswerObj = poDS->GET(osURI);
    if (poAnswerObj == NULL)
        return nTotalRows;

    if ( !json_object_is_type(poAnswerObj, json_type_object) )
    {
        json_object_put(poAnswerObj);
        return nTotalRows;
    }

    json_object* poTotalRows = json_object_object_get(poAnswerObj,
                                                        "total_rows");
    if (poTotalRows != NULL &&
        json_object_is_type(poTotalRows, json_type_int))
    {
        nTotalRows = json_object_get_int(poTotalRows);
    }

    json_object* poRows = json_object_object_get(poAnswerObj, "rows");
    if (poRows == NULL ||
        !json_object_is_type(poRows, json_type_array))
    {
        json_object_put(poAnswerObj);
        return nTotalRows;
    }

    bHasOGRSpatial = FALSE;

    int nSpecialRows = json_object_array_length(poRows);
    for(int i=0;i<nSpecialRows;i++)
    {
        json_object* poRow = json_object_array_get_idx(poRows, i);
        if ( poRow != NULL &&
             json_object_is_type(poRow, json_type_object) )
        {
            json_object* poId = json_object_object_get(poRow, "id");
            const char* pszId = json_object_get_string(poId);
            if ( pszId && strcmp(pszId, "_design/ogr_spatial") == 0)
            {
                bHasOGRSpatial = TRUE;
            }
        }
    }

    if (!bHasOGRSpatial)
    {
        bServerSideSpatialFilteringWorks = FALSE;
    }

    if (nTotalRows >= nSpecialRows)
        nTotalRows -= nSpecialRows;

    json_object_put(poAnswerObj);

    return nTotalRows;
}

/************************************************************************/
/*                            CreateField()                             */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::CreateField( OGRFieldDefn *poField,
                                 int bApproxOK )
{

    if (!poDS->IsReadWrite())
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Operation not available in read-only mode");
        return OGRERR_FAILURE;
    }

    GetLayerDefn();

    poFeatureDefn->AddFieldDefn(poField);

    bMustWriteMetadata = TRUE;

    return OGRERR_NONE;
}

/************************************************************************/
/*                         OGRCouchDBWriteFeature                       */
/************************************************************************/

static json_object* OGRCouchDBWriteFeature( OGRFeature* poFeature,
                                            OGRwkbGeometryType eGeomType,
                                            int bGeoJSONDocument)
{
    CPLAssert( NULL != poFeature );

    json_object* poObj = json_object_new_object();
    CPLAssert( NULL != poObj );

    if (poFeature->IsFieldSet(_ID_FIELD))
    {
        const char* pszId = poFeature->GetFieldAsString(_ID_FIELD);
        json_object_object_add( poObj, "_id",
                                json_object_new_string(pszId) );

        if ( poFeature->GetFID() != OGRNullFID &&
             strcmp(CPLSPrintf("%09ld", poFeature->GetFID()), pszId) != 0 )
        {
            CPLDebug("CouchDB",
                     "_id field = %s, but FID = %09ld --> taking into account _id field only",
                     pszId,
                     poFeature->GetFID());
        }
    }
    else if ( poFeature->GetFID() != OGRNullFID )
    {
        json_object_object_add( poObj, "_id",
                                json_object_new_string(CPLSPrintf("%09ld", poFeature->GetFID())) );
    }

    if (poFeature->IsFieldSet(_REV_FIELD))
    {
        const char* pszRev = poFeature->GetFieldAsString(_REV_FIELD);
        json_object_object_add( poObj, "_rev",
                                json_object_new_string(pszRev) );
    }

    if (bGeoJSONDocument)
    {
        json_object_object_add( poObj, "type",
                                json_object_new_string("Feature") );
    }

/* -------------------------------------------------------------------- */
/*      Write feature attributes to GeoJSON "properties" object.        */
/* -------------------------------------------------------------------- */
    json_object* poObjProps = NULL;

    poObjProps = OGRGeoJSONWriteAttributes( poFeature );
    if (poObjProps)
    {
        json_object_object_del(poObjProps, "_id");
        json_object_object_del(poObjProps, "_rev");
    }

    if (bGeoJSONDocument)
    {
        json_object_object_add( poObj, "properties", poObjProps );
    }
    else
    {
        json_object_iter it;
        it.key = NULL;
        it.val = NULL;
        it.entry = NULL;
        json_object_object_foreachC( poObjProps, it )
        {
            json_object_object_add( poObj, it.key, json_object_get(it.val) );
        }
        json_object_put(poObjProps);
    }

/* -------------------------------------------------------------------- */
/*      Write feature geometry to GeoJSON "geometry" object.            */
/*      Null geometries are allowed, according to the GeoJSON Spec.     */
/* -------------------------------------------------------------------- */
    if (eGeomType != wkbNone)
    {
        json_object* poObjGeom = NULL;

        OGRGeometry* poGeometry = poFeature->GetGeometryRef();
        if ( NULL != poGeometry )
        {
            poObjGeom = OGRGeoJSONWriteGeometry( poGeometry );
            if ( poObjGeom != NULL &&
                 wkbFlatten(poGeometry->getGeometryType()) != wkbPoint &&
                 !poGeometry->IsEmpty() )
            {
                OGREnvelope sEnvelope;
                poGeometry->getEnvelope(&sEnvelope);

                json_object* bbox = json_object_new_array();
                json_object_array_add(bbox, json_object_new_double(sEnvelope.MinX));
                json_object_array_add(bbox, json_object_new_double(sEnvelope.MinY));
                json_object_array_add(bbox, json_object_new_double(sEnvelope.MaxX));
                json_object_array_add(bbox, json_object_new_double(sEnvelope.MaxY));
                json_object_object_add( poObjGeom, "bbox", bbox );
            }
        }

        json_object_object_add( poObj, "geometry", poObjGeom );
    }

    return poObj;
}

/************************************************************************/
/*                           GetMaximumId()                             */
/************************************************************************/

int OGRCouchDBTableLayer::GetMaximumId()
{
    CPLString osURI("/");
    osURI += osName;
    osURI += "/_all_docs?startkey_docid=999999999&endkey_docid=000000000&descending=true&limit=1";
    json_object* poAnswerObj = poDS->GET(osURI);
    if (poAnswerObj == NULL)
        return -1;

    if ( !json_object_is_type(poAnswerObj, json_type_object) )
    {
        CPLError(CE_Failure, CPLE_AppDefined, "GetMaximumId() failed");
        json_object_put(poAnswerObj);
        return -1;
    }

    if (poDS->IsError(poAnswerObj, "GetMaximumId() failed"))
    {
        json_object_put(poAnswerObj);
        return -1;
    }

    json_object* poRows = json_object_object_get(poAnswerObj, "rows");
    if (poRows == NULL ||
        !json_object_is_type(poRows, json_type_array))
    {
        CPLError(CE_Failure, CPLE_AppDefined, "GetMaximumId() failed");
        json_object_put(poAnswerObj);
        return -1;
    }

    int nRows = json_object_array_length(poRows);
    if (nRows != 1)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "GetMaximumId() failed");
        json_object_put(poAnswerObj);
        return -1;
    }
    
    json_object* poRow = json_object_array_get_idx(poRows, 0);
    if ( poRow == NULL ||
            !json_object_is_type(poRow, json_type_object) )
    {
        CPLError(CE_Failure, CPLE_AppDefined, "GetMaximumId() failed");
        json_object_put(poAnswerObj);
        return -1;
    }

    json_object* poId = json_object_object_get(poRow, "id");
    const char* pszId = json_object_get_string(poId);
    if (pszId != NULL)
    {
        int nId = atoi(pszId);
        json_object_put(poAnswerObj);
        return nId;
    }

    json_object_put(poAnswerObj);
    return -1;
}

/************************************************************************/
/*                           CreateFeature()                            */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::CreateFeature( OGRFeature *poFeature )

{
    GetLayerDefn();

    if (!poDS->IsReadWrite())
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Operation not available in read-only mode");
        return OGRERR_FAILURE;
    }

    if (poFeature->IsFieldSet(_REV_FIELD))
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "CreateFeature() should be called with an unset _id field");
        return OGRERR_FAILURE;
    }

    if (nNextFIDForCreate < 0)
    {
        nNextFIDForCreate = GetMaximumId();
        if (nNextFIDForCreate >= 0)
            nNextFIDForCreate ++;
        else
            nNextFIDForCreate = GetTotalFeatureCount();
    }

    OGRGeometry* poGeom = poFeature->GetGeometryRef();
    if (bExtentValid && poGeom != NULL && !poGeom->IsEmpty())
    {
        OGREnvelope sEnvelope;
        poGeom->getEnvelope(&sEnvelope);
        if (!bExtentSet)
        {
            dfMinX = sEnvelope.MinX;
            dfMinY = sEnvelope.MinY;
            dfMaxX = sEnvelope.MaxX;
            dfMaxY = sEnvelope.MaxY;
            bExtentSet = TRUE;
        }
        if (sEnvelope.MinX < dfMinX)
            dfMinX = sEnvelope.MinX;
        if (sEnvelope.MinY < dfMinY)
            dfMinY = sEnvelope.MinY;
        if (sEnvelope.MaxX > dfMaxX)
            dfMaxX = sEnvelope.MaxX;
        if (sEnvelope.MaxY > dfMaxY)
            dfMaxY = sEnvelope.MaxY;
    }

    if (bExtentValid && eGeomType != wkbNone)
        bMustWriteMetadata = TRUE;

    int nFID = nNextFIDForCreate ++;
    CPLString osFID;
    if (!poFeature->IsFieldSet(_ID_FIELD))
    {
        if (poFeature->GetFID() != OGRNullFID)
        {
            nFID = (int)poFeature->GetFID();
        }
        osFID = CPLSPrintf("%09d", nFID);

        poFeature->SetField(_ID_FIELD, osFID);
        poFeature->SetFID(nFID);
    }
    else
    {
        const char* pszId = poFeature->GetFieldAsString(_ID_FIELD);
        osFID = pszId;
    }

    json_object* poObj = OGRCouchDBWriteFeature(poFeature, eGeomType, bGeoJSONDocument);

    if (bInTransaction)
    {
        aoTransactionFeatures.push_back(poObj);

        return OGRERR_NONE;
    }

    const char* pszJson = json_object_to_json_string( poObj );
    CPLString osURI("/");
    osURI += osName;
    osURI += "/";
    osURI += osFID;
    json_object* poAnswerObj = poDS->PUT(osURI, pszJson);
    json_object_put( poObj );

    if (poAnswerObj == NULL)
        return OGRERR_FAILURE;

    if (!poDS->IsOK(poAnswerObj, "Feature creation failed"))
    {
        json_object_put(poAnswerObj);
        return OGRERR_FAILURE;
    }

    json_object* poId = json_object_object_get(poAnswerObj, "id");
    json_object* poRev = json_object_object_get(poAnswerObj, "rev");

    const char* pszId = json_object_get_string(poId);
    const char* pszRev = json_object_get_string(poRev);

    if (pszId)
    {
        poFeature->SetField(_ID_FIELD, pszId);

        int nFID = atoi(pszId);
        const char* pszFID = CPLSPrintf("%09d", nFID);
        if (strcmp(pszId, pszFID) == 0)
            poFeature->SetFID(nFID);
        else
            poFeature->SetFID(-1);
    }
    if (pszRev)
    {
        poFeature->SetField(_REV_FIELD, pszRev);
    }

    json_object_put(poAnswerObj);

    nUpdateSeq ++;

    return OGRERR_NONE;
}

/************************************************************************/
/*                           SetFeature()                               */
/************************************************************************/

OGRErr      OGRCouchDBTableLayer::SetFeature( OGRFeature *poFeature )
{
    GetLayerDefn();

    if (!poDS->IsReadWrite())
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Operation not available in read-only mode");
        return OGRERR_FAILURE;
    }

    if (!poFeature->IsFieldSet(_ID_FIELD))
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "SetFeature() requires non null _id field");
        return OGRERR_FAILURE;
    }

    json_object* poObj = OGRCouchDBWriteFeature(poFeature, eGeomType, bGeoJSONDocument);

    const char* pszJson = json_object_to_json_string( poObj );
    CPLString osURI("/");
    osURI += osName;
    osURI += "/";
    osURI += poFeature->GetFieldAsString(_ID_FIELD);
    json_object* poAnswerObj = poDS->PUT(osURI, pszJson);
    json_object_put( poObj );

    if (poAnswerObj == NULL)
        return OGRERR_FAILURE;

    if (!poDS->IsOK(poAnswerObj, "Feature update failed"))
    {
        json_object_put(poAnswerObj);
        return OGRERR_FAILURE;
    }

    json_object* poRev = json_object_object_get(poAnswerObj, "rev");
    const char* pszRev = json_object_get_string(poRev);
    poFeature->SetField(_REV_FIELD, pszRev);

    json_object_put(poAnswerObj);

    if (bExtentValid && eGeomType != wkbNone)
    {
        bExtentValid = FALSE;
        bMustWriteMetadata = TRUE;
    }
    nUpdateSeq ++;

    return OGRERR_NONE;
}

/************************************************************************/
/*                          DeleteFeature()                             */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::DeleteFeature( long nFID )
{
    GetLayerDefn();

    if (!poDS->IsReadWrite())
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Operation not available in read-only mode");
        return OGRERR_FAILURE;
    }

    OGRFeature* poFeature = GetFeature(nFID);
    if (poFeature == NULL)
        return OGRERR_FAILURE;

    if (!poFeature->IsFieldSet(_REV_FIELD))
    {
        delete poFeature;
        return OGRERR_FAILURE;
    }

    const char* pszRev = poFeature->GetFieldAsString(_REV_FIELD);

    CPLString osURI("/");
    osURI += osName;
    osURI += "/";
    osURI += CPLSPrintf("%09d?rev=%s", (int)nFID, pszRev);

    if (bExtentValid && eGeomType != wkbNone)
        bMustWriteMetadata = TRUE;

    OGRGeometry* poGeom = poFeature->GetGeometryRef();
    if (bExtentValid && bExtentSet && poGeom != NULL && !poGeom->IsEmpty())
    {
        OGREnvelope sEnvelope;
        poGeom->getEnvelope(&sEnvelope);
        if (dfMinX == sEnvelope.MinX ||
            dfMinY == sEnvelope.MinY ||
            dfMaxX == sEnvelope.MaxX ||
            dfMaxY == sEnvelope.MaxY)
        {
            bExtentValid = FALSE;
        }
    }

    delete poFeature;

    json_object* poAnswerObj = poDS->DELETE(osURI);

    if (poAnswerObj == NULL)
        return OGRERR_FAILURE;

    if (!poDS->IsOK(poAnswerObj, "Feature deletion failed"))
    {
        json_object_put(poAnswerObj);
        return OGRERR_FAILURE;
    }

    nUpdateSeq ++;

    json_object_put(poAnswerObj);

    return OGRERR_NONE;
}

/************************************************************************/
/*                         StartTransaction()                           */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::StartTransaction()
{
    GetLayerDefn();

    if (bInTransaction)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Already in transaction");
        return OGRERR_FAILURE;
    }

    if (!poDS->IsReadWrite())
    {
        CPLError(CE_Failure, CPLE_AppDefined,
                 "Operation not available in read-only mode");
        return OGRERR_FAILURE;
    }

    bInTransaction = TRUE;

    return OGRERR_NONE;
}

/************************************************************************/
/*                         CommitTransaction()                          */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::CommitTransaction()
{
    GetLayerDefn();

    if (!bInTransaction)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Should be in transaction");
        return OGRERR_FAILURE;
    }

    bInTransaction = FALSE;

    if (aoTransactionFeatures.size() == 0)
        return OGRERR_NONE;

    CPLString osPost("{ \"docs\": [");
    for(int i=0;i<(int)aoTransactionFeatures.size();i++)
    {
        if (i>0) osPost += ",";
        const char* pszJson = json_object_to_json_string( aoTransactionFeatures[i] );
        osPost += pszJson;
        json_object_put(aoTransactionFeatures[i]);
    }
    osPost += "] }";
    aoTransactionFeatures.resize(0);

    CPLString osURI("/");
    osURI += osName;
    osURI += "/_bulk_docs";
    json_object* poAnswerObj = poDS->POST(osURI, osPost);

    if (poAnswerObj == NULL)
        return OGRERR_FAILURE;

    if ( json_object_is_type(poAnswerObj, json_type_object) )
    {
        poDS->IsError(poAnswerObj, "Bulk feature creation failed");

        json_object_put(poAnswerObj);
        return OGRERR_FAILURE;
    }

    if ( !json_object_is_type(poAnswerObj, json_type_array) )
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Bulk feature creation failed");
        json_object_put(poAnswerObj);

        return OGRERR_FAILURE;
    }

    int nRows = json_object_array_length(poAnswerObj);
    for(int i=0;i<nRows;i++)
    {
        json_object* poRow = json_object_array_get_idx(poAnswerObj, i);
        if ( poRow != NULL &&
             json_object_is_type(poRow, json_type_object) )
        {
            json_object* poId = json_object_object_get(poRow, "id");
            json_object* poRev = json_object_object_get(poRow, "rev");
            json_object* poError = json_object_object_get(poRow, "error");
            json_object* poReason = json_object_object_get(poRow, "reason");

            const char* pszId = json_object_get_string(poId);

            if (poError != NULL)
            {
                const char* pszError = json_object_get_string(poError);
                const char* pszReason = json_object_get_string(poReason);

                CPLError(CE_Failure, CPLE_AppDefined,
                         "Bulk feature creation failed : for %s: %s, %s",
                         pszId ? pszId : "",
                         pszError ? pszError : "",
                         pszReason ? pszReason : "");
            }
            else if (poRev != NULL)
            {
                const char* pszRev = json_object_get_string(poRev);
                CPLDebug("CouchDB", "id = %s, rev = %s",
                         pszId ? pszId : "", pszRev ? pszRev : "");

                nUpdateSeq ++;
            }
        }
    }

    json_object_put(poAnswerObj);

    return OGRERR_NONE;
}

/************************************************************************/
/*                        RollbackTransaction()                         */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::RollbackTransaction()
{
    GetLayerDefn();

    if (!bInTransaction)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Should be in transaction");
        return OGRERR_FAILURE;
    }
    bInTransaction = FALSE;
    for(int i=0;i<(int)aoTransactionFeatures.size();i++)
    {
        json_object_put(aoTransactionFeatures[i]);
    }
    aoTransactionFeatures.resize(0);
    return OGRERR_NONE;
}

/************************************************************************/
/*                         SetAttributeFilter()                         */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::SetAttributeFilter( const char *pszQuery )

{
    GetLayerDefn();

    bServerSideAttributeFilteringWorks = TRUE;

    OGRErr eErr = OGRCouchDBLayer::SetAttributeFilter(pszQuery);

    if (eErr == OGRERR_NONE)
    {
        bHasInstalledAttributeFilter = TRUE;
    }

    return eErr;
}


/************************************************************************/
/*                          SetSpatialFilter()                          */
/************************************************************************/

void OGRCouchDBTableLayer::SetSpatialFilter( OGRGeometry * poGeomIn )

{
    GetLayerDefn();

    if( InstallFilter( poGeomIn ) )
    {
        bHasInstalledSpatialFilter = TRUE;

        ResetReading();
    }
}

/************************************************************************/
/*                          SetInfoAfterCreation()                      */
/************************************************************************/

void OGRCouchDBTableLayer::SetInfoAfterCreation(OGRwkbGeometryType eGType,
                                                OGRSpatialReference* poSRSIn,
                                                int nUpdateSeqIn,
                                                int bGeoJSONDocumentIn)
{
    eGeomType = eGType;
    nNextFIDForCreate = 0;
    bMustWriteMetadata = TRUE;
    bExtentValid = TRUE;
    bHasLoadedMetadata = TRUE;
    nUpdateSeq = nUpdateSeqIn;
    bGeoJSONDocument = bGeoJSONDocumentIn;

    CPLAssert(poSRS == NULL);
    poSRS = poSRSIn;
    if (poSRS)
        poSRS->Reference();
}

/************************************************************************/
/*                          GetSpatialRef()                             */
/************************************************************************/

OGRSpatialReference* OGRCouchDBTableLayer::GetSpatialRef()
{
    LoadMetadata();

    return poSRS;
}

/************************************************************************/
/*                          LoadMetadata()                              */
/************************************************************************/

void OGRCouchDBTableLayer::LoadMetadata()
{
    if (bHasLoadedMetadata)
        return;

    bHasLoadedMetadata = TRUE;

    CPLString osURI("/");
    osURI += osName;
    osURI += "/_design/ogr_metadata";
    json_object* poAnswerObj = poDS->GET(osURI);
    if (poAnswerObj == NULL)
        return;

    if ( !json_object_is_type(poAnswerObj, json_type_object) )
    {
        CPLError(CE_Failure, CPLE_AppDefined, "LoadMetadata() failed");
        json_object_put(poAnswerObj);
        return;
    }

    json_object* poRev = json_object_object_get(poAnswerObj, "_rev");
    const char* pszRev = json_object_get_string(poRev);
    if (pszRev)
        osMetadataRev = pszRev;

    json_object* poError = json_object_object_get(poAnswerObj, "error");
    const char* pszError = json_object_get_string(poError);
    if (pszError && strcmp(pszError, "not_found") == 0)
    {
        json_object_put(poAnswerObj);
        return;
    }

    if (poDS->IsError(poAnswerObj, "LoadMetadata() failed"))
    {
        json_object_put(poAnswerObj);
        return;
    }

    json_object* poJsonSRS = json_object_object_get(poAnswerObj, "srs");
    const char* pszSRS = json_object_get_string(poJsonSRS);
    if (pszSRS != NULL)
    {
        poSRS = new OGRSpatialReference();
        if (poSRS->importFromWkt((char**)&pszSRS) != OGRERR_NONE)
        {
            delete poSRS;
            poSRS = NULL;
        }
    }

    json_object* poGeomType = json_object_object_get(poAnswerObj, "geomtype");
    const char* pszGeomType = json_object_get_string(poGeomType);

    if (pszGeomType)
    {
        if (EQUAL(pszGeomType, "NONE"))
        {
            eGeomType = wkbNone;
            bExtentValid = TRUE;
        }
        else
        {
            eGeomType = OGRFromOGCGeomType(pszGeomType);

            json_object* poIs25D = json_object_object_get(poAnswerObj, "is_25D");
            if (poIs25D && json_object_get_boolean(poIs25D))
                eGeomType = (OGRwkbGeometryType) (eGeomType | wkb25DBit);

            json_object* poExtent = json_object_object_get(poAnswerObj, "extent");
            if (poExtent && json_object_get_type(poExtent) == json_type_object)
            {
                json_object* poUpdateSeq =
                    json_object_object_get(poExtent, "validity_update_seq");
                if (poUpdateSeq && json_object_get_type(poUpdateSeq) == json_type_int)
                {
                    int nValidityUpdateSeq = json_object_get_int(poUpdateSeq);
                    if (nUpdateSeq < 0)
                        nUpdateSeq = FetchUpdateSeq();
                    if (nUpdateSeq != nValidityUpdateSeq)
                    {
                        CPLDebug("CouchDB",
                                 "_design/ogr_metadata.extent.validity_update_seq "
                                 "doesn't match database update_seq --> ignoring stored extent");
                        poUpdateSeq = NULL;
                    }
                }
                else
                    poUpdateSeq = NULL;

                json_object* poBbox = json_object_object_get(poExtent, "bbox");
                if (poUpdateSeq && poBbox &&
                    json_object_get_type(poBbox) == json_type_array &&
                    json_object_array_length(poBbox) == 4 &&
                    json_object_get_type(json_object_array_get_idx(poBbox, 0)) == json_type_double &&
                    json_object_get_type(json_object_array_get_idx(poBbox, 1)) == json_type_double &&
                    json_object_get_type(json_object_array_get_idx(poBbox, 2)) == json_type_double &&
                    json_object_get_type(json_object_array_get_idx(poBbox, 3)) == json_type_double)
                {
                    dfMinX = json_object_get_double(json_object_array_get_idx(poBbox, 0));
                    dfMinY = json_object_get_double(json_object_array_get_idx(poBbox, 1));
                    dfMaxX = json_object_get_double(json_object_array_get_idx(poBbox, 2));
                    dfMaxY = json_object_get_double(json_object_array_get_idx(poBbox, 3));
                    bExtentValid = bExtentSet = TRUE;
                }
            }
        }
    }

    json_object* poGeoJSON = json_object_object_get(poAnswerObj, "geojson_documents");
    if (poGeoJSON && json_object_is_type(poGeoJSON, json_type_boolean))
        bGeoJSONDocument = json_object_get_boolean(poGeoJSON);

    json_object* poFields = json_object_object_get(poAnswerObj, "fields");
    if (poFields && json_object_is_type(poFields, json_type_array))
    {
        poFeatureDefn = new OGRFeatureDefn( osName );
        poFeatureDefn->Reference();

        poFeatureDefn->SetGeomType(eGeomType);

        OGRFieldDefn oFieldId("_id", OFTString);
        poFeatureDefn->AddFieldDefn(&oFieldId);

        OGRFieldDefn oFieldRev("_rev", OFTString);
        poFeatureDefn->AddFieldDefn(&oFieldRev);

        int nFields = json_object_array_length(poFields);
        for(int i=0;i<nFields;i++)
        {
            json_object* poField = json_object_array_get_idx(poFields, i);
            if (poField && json_object_is_type(poField, json_type_object))
            {
                json_object* poName = json_object_object_get(poField, "name");
                const char* pszName = json_object_get_string(poName);
                if (pszName)
                {
                    json_object* poType = json_object_object_get(poField, "type");
                    const char* pszType = json_object_get_string(poType);
                    OGRFieldType eType = OFTString;
                    if (pszType)
                    {
                        if (strcmp(pszType, "integer") == 0)
                            eType = OFTInteger;
                        else if (strcmp(pszType, "integerlist") == 0)
                            eType = OFTIntegerList;
                        else if (strcmp(pszType, "real") == 0)
                            eType = OFTReal;
                        else if (strcmp(pszType, "reallist") == 0)
                            eType = OFTRealList;
                        else if (strcmp(pszType, "string") == 0)
                            eType = OFTString;
                        else if (strcmp(pszType, "stringlist") == 0)
                            eType = OFTStringList;
                    }

                    OGRFieldDefn oField(pszName, eType);
                    poFeatureDefn->AddFieldDefn(&oField);
                }
            }
        }
    }

    json_object_put(poAnswerObj);

    return;
}

/************************************************************************/
/*                          WriteMetadata()                             */
/************************************************************************/

void OGRCouchDBTableLayer::WriteMetadata()
{
    GetLayerDefn();

    CPLString osURI;
    osURI = "/";
    osURI += osName;
    osURI += "/_design/ogr_metadata";

    json_object* poDoc = json_object_new_object();

    if (osMetadataRev.size() > 0)
    {
        json_object_object_add(poDoc, "_rev",
                               json_object_new_string(osMetadataRev));
    }

    if (poSRS)
    {
        char* pszWKT = NULL;
        poSRS->exportToWkt(&pszWKT);
        if (pszWKT)
        {
            json_object_object_add(poDoc, "srs",
                                   json_object_new_string(pszWKT));
            CPLFree(pszWKT);
        }
    }

    if (eGeomType != wkbNone)
    {
        json_object_object_add(poDoc, "geomtype",
                    json_object_new_string(OGRToOGCGeomType(eGeomType)));
        if (poFeatureDefn->GetGeomType() & wkb25DBit)
        {
            json_object_object_add(poDoc, "is_25D",
                               json_object_new_boolean(TRUE));
        }

        if (bExtentValid && bExtentSet && nUpdateSeq >= 0)
        {
            json_object* poExtent = json_object_new_object();
            json_object_object_add(poDoc, "extent", poExtent);

            json_object_object_add(poExtent, "validity_update_seq",
                                   json_object_new_int(nUpdateSeq + 1));

            json_object* poBbox = json_object_new_array();
            json_object_object_add(poExtent, "bbox", poBbox);
            json_object_array_add(poBbox, json_object_new_double(dfMinX));
            json_object_array_add(poBbox, json_object_new_double(dfMinY));
            json_object_array_add(poBbox, json_object_new_double(dfMaxX));
            json_object_array_add(poBbox, json_object_new_double(dfMaxY));
        }
    }
    else
    {
        json_object_object_add(poDoc, "geomtype",
                               json_object_new_string("NONE"));
    }

    json_object_object_add(poDoc, "geojson_documents",
                           json_object_new_boolean(bGeoJSONDocument));

    json_object* poFields = json_object_new_array();
    json_object_object_add(poDoc, "fields", poFields);


    for(int i=FIRST_FIELD;i<poFeatureDefn->GetFieldCount();i++)
    {
        json_object* poField = json_object_new_object();
        json_object_array_add(poFields, poField);

        json_object_object_add(poField, "name",
            json_object_new_string(poFeatureDefn->GetFieldDefn(i)->GetNameRef()));

        const char* pszType = NULL;
        switch (poFeatureDefn->GetFieldDefn(i)->GetType())
        {
            case OFTInteger: pszType = "integer"; break;
            case OFTReal: pszType = "real"; break;
            case OFTString: pszType = "string"; break;
            case OFTIntegerList: pszType = "integerlist"; break;
            case OFTRealList: pszType = "reallist"; break;
            case OFTStringList: pszType = "stringlist"; break;
            default: pszType = "string"; break;
        }

        json_object_object_add(poField, "type",
                               json_object_new_string(pszType));
    }

    json_object* poAnswerObj = poDS->PUT(osURI,
                                         json_object_to_json_string(poDoc));

    json_object_put(poDoc);

    if (poDS->IsOK(poAnswerObj, "Metadata creation failed"))
    {
        nUpdateSeq++;

        json_object* poRev = json_object_object_get(poAnswerObj, "_rev");
        const char* pszRev = json_object_get_string(poRev);
        if (pszRev)
            osMetadataRev = pszRev;
    }

    json_object_put(poAnswerObj);
}

/************************************************************************/
/*                            GetExtent()                               */
/************************************************************************/

OGRErr OGRCouchDBTableLayer::GetExtent(OGREnvelope *psExtent, int bForce)
{
    LoadMetadata();

    if (!bExtentValid)
        return OGRCouchDBLayer::GetExtent(psExtent, bForce);

    psExtent->MinX = 0.0;
    psExtent->MaxX = 0.0;
    psExtent->MinY = 0.0;
    psExtent->MaxY = 0.0;

    if (!bExtentSet)
        return OGRERR_FAILURE;

    psExtent->MinX = dfMinX;
    psExtent->MaxX = dfMaxX;
    psExtent->MinY = dfMinY;
    psExtent->MaxY = dfMaxY;

    return OGRERR_NONE;
}

/************************************************************************/
/*                          FetchUpdateSeq()                            */
/************************************************************************/

int OGRCouchDBTableLayer::FetchUpdateSeq()
{
    if (nUpdateSeq >= 0)
        return nUpdateSeq;

    CPLString osURI("/");
    osURI += osName;
    osURI += "/";

    json_object* poAnswerObj = poDS->GET(osURI);
    if (poAnswerObj != NULL &&
        json_object_is_type(poAnswerObj, json_type_object) &&
        json_object_object_get(poAnswerObj, "update_seq") != NULL)
    {
        nUpdateSeq = json_object_get_int(json_object_object_get(poAnswerObj,
                                                                "update_seq"));
    }
    else
    {
        poDS->IsError(poAnswerObj, "FetchUpdateSeq() failed");
    }

    json_object_put(poAnswerObj);

    return nUpdateSeq;
}
