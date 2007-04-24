/******************************************************************************
 * $Id$
 *
 * Project:  Interlis 1 Reader
 * Purpose:  Implementation of ILI1Reader class.
 * Author:   Pirmin Kalberer, Sourcepole AG
 *
 ******************************************************************************
 * Copyright (c) 2004, Pirmin Kalberer, Sourcepole AG
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

#include "ogr_ili1.h"
#include "cpl_conv.h"
#include "cpl_string.h"
#include "ogr_api.h"
#include "ogr_geos.h"

#include "ilihelper.h"
#include "iomhelper.h"
#include "ili1reader.h"
#include "ili1readerp.h"

#ifdef HAVE_GEOS
#  define POLYGONIZE_AREAS
#endif

#ifndef POLYGONIZE_AREAS
#  if defined(__GNUC_PREREQ)
#    warning Interlis 1 Area polygonizing disabled. Needs GEOS >= 2.1.0
#  endif
#endif

CPL_CVSID("$Id$");


//
// ILI1Reader
//
IILI1Reader::~IILI1Reader() {
}

ILI1Reader::ILI1Reader() {
  fpItf = NULL;
  nLayers = 0;
  papoLayers = NULL;
  nAreaLayers = 0;
  papoAreaLayers = NULL;
  papoAreaLineLayers = NULL;
  nSurfaceLayers = 0;
  papoSurfaceLayers = NULL;
  papoSurfacePolyLayers = NULL;
  SetArcDegrees(1);
}

ILI1Reader::~ILI1Reader() {
 if (fpItf) VSIFClose( fpItf );
}

void ILI1Reader::SetArcDegrees(double arcDegrees) {
  arcIncr = arcDegrees*PI/180;
}

/* -------------------------------------------------------------------- */
/*      Open the source file.                                           */
/* -------------------------------------------------------------------- */
int ILI1Reader::OpenFile( const char *pszFilename ) {
    fpItf = VSIFOpen( pszFilename, "r" );
    if( fpItf == NULL )
    {
          CPLError( CE_Failure, CPLE_OpenFailed, 
                    "Failed to open ILI file `%s'.", 
                    pszFilename );

        return FALSE;
    }
    return TRUE;
}

const char* ILI1Reader::GetLayerNameString(const char* topicname, const char* tablename) {
    static char layername[512];
    layername[0] = '\0';
    strcat(layername, topicname);
    strcat(layername, "__");
    strcat(layername, tablename);
    return layername;
}

const char* ILI1Reader::GetLayerName(IOM_BASKET model, IOM_OBJECT table) {
    static char layername[512];
    IOM_OBJECT topic = GetAttrObj(model, table, "container");
    layername[0] = '\0';
    strcat(layername, iom_getattrvalue(topic, "name"));
    strcat(layername, "__");
    strcat(layername, iom_getattrvalue(table, "name"));
    return layername;
}

void ILI1Reader::AddCoord(OGRILI1Layer* layer, IOM_BASKET model, IOM_OBJECT modelele, IOM_OBJECT typeobj) {
  unsigned int dim = ::GetCoordDim(model, typeobj);
  for (unsigned int i=0; i<dim; i++) {
    OGRFieldDefn fieldDef(CPLSPrintf("%s_%d", iom_getattrvalue(modelele, "name"), i), OFTReal);
    layer->GetLayerDefn()->AddFieldDefn(&fieldDef);
    CPLDebug( "OGR_ILI", "Field %s: OFTReal", fieldDef.GetNameRef());
  }
}

OGRILI1Layer* ILI1Reader::AddGeomTable(const char* datalayername, const char* geomname, OGRwkbGeometryType eType) {
  static char layername[512];
  layername[0] = '\0';
  strcat(layername, datalayername);
  strcat(layername, "_");
  strcat(layername, geomname);

  OGRILI1Layer* geomlayer = new OGRILI1Layer(layername, NULL, 0, eType, NULL);
  AddLayer(geomlayer);
  OGRFieldDefn fieldDef("_TID", OFTString);
  geomlayer->GetLayerDefn()->AddFieldDefn(&fieldDef);
  if (eType == wkbPolygon)
  {
     OGRFieldDefn fieldDefRef("_RefTID", OFTString);
     geomlayer->GetLayerDefn()->AddFieldDefn(&fieldDefRef);
  }
  OGRFieldDefn fieldDef2("ILI_Geometry", OFTString); //in write mode only?
  geomlayer->GetLayerDefn()->AddFieldDefn(&fieldDef2);
  return geomlayer;
}

void ILI1Reader::AddField(OGRILI1Layer* layer, IOM_BASKET model, IOM_OBJECT obj) {
  const char* typenam = "Reference";
  if (EQUAL(iom_getobjecttag(obj),"iom04.metamodel.LocalAttribute")) typenam = GetTypeName(model, obj);
  CPLDebug( "OGR_ILI", "Field %s: %s", iom_getattrvalue(obj, "name"), typenam);
  if (EQUAL(typenam, "iom04.metamodel.SurfaceType")) {
    OGRILI1Layer* polyLayer = AddGeomTable(layer->GetLayerDefn()->GetName(), iom_getattrvalue(obj, "name"), wkbPolygon);
    AddSurfaceLayer(layer, polyLayer);
    //TODO: add line attributes to geometry
  } else if (EQUAL(typenam, "iom04.metamodel.AreaType")) {
    IOM_OBJECT controlPointDomain = GetAttrObj(model, GetTypeObj(model, obj), "controlPointDomain");
    if (controlPointDomain) {
      AddCoord(layer, model, obj, GetTypeObj(model, controlPointDomain));
      layer->GetLayerDefn()->SetGeomType(wkbPoint);
    }
    OGRILI1Layer* areaLineLayer = AddGeomTable(layer->GetLayerDefn()->GetName(), iom_getattrvalue(obj, "name"), wkbMultiLineString);
#ifdef POLYGONIZE_AREAS
    AddAreaLayer(layer, areaLineLayer);
#endif
  } else if (EQUAL(typenam, "iom04.metamodel.PolylineType") ) {
    layer->GetLayerDefn()->SetGeomType(wkbMultiLineString);
  } else if (EQUAL(typenam, "iom04.metamodel.CoordType")) {
    AddCoord(layer, model, obj, GetTypeObj(model, obj));
    if (layer->GetLayerDefn()->GetGeomType() == wkbUnknown) layer->GetLayerDefn()->SetGeomType(wkbPoint);
  } else {
    OGRFieldDefn fieldDef(iom_getattrvalue(obj, "name"), OFTString);
    layer->GetLayerDefn()->AddFieldDefn(&fieldDef);
  }
}

int ILI1Reader::ReadModel(const char *pszModelFilename) {

  IOM_BASKET model;
  IOM_ITERATOR modelelei;
  IOM_OBJECT modelele;

  iom_init();

  // set error listener to a iom provided one, that just 
  // dumps all errors to stderr
  iom_seterrlistener(iom_stderrlistener);

  // compile ili model
  char *iomarr[1] = {(char *)pszModelFilename};
  model=iom_compileIli(1, iomarr);
  if(!model){
    CPLError( CE_Failure, CPLE_FileIO, "iom_compileIli failed." );
    iom_end();
    return FALSE;
  }

  // read tables
  modelelei=iom_iteratorobject(model);
  modelele=iom_nextobject(modelelei);
  while(modelele){
    const char *tag=iom_getobjecttag(modelele);

    if (tag && EQUAL(tag,"iom04.metamodel.Table")) {
      const char* topic = iom_getattrvalue(GetAttrObj(model, modelele, "container"), "name");
      if (!EQUAL(topic, "INTERLIS")) {
        const char* layername = GetLayerName(model, modelele);
        OGRSpatialReference *poSRSIn = NULL;
        int bWriterIn = 0;
        OGRwkbGeometryType eReqType = wkbUnknown;
        OGRILI1DataSource *poDSIn = NULL;
        OGRILI1Layer* layer = new OGRILI1Layer(layername, poSRSIn, bWriterIn, eReqType, poDSIn);
        AddLayer(layer);
        CPLDebug( "OGR_ILI", "Reading table model '%s'", layername );

        // read fields
        IOM_OBJECT fields[255];
        IOM_OBJECT roledefs[255];
        memset(fields, 0, 255);
        memset(roledefs, 0, 255);
        int maxIdx = -1;
        IOM_ITERATOR fieldit=iom_iteratorobject(model);
        for (IOM_OBJECT fieldele=iom_nextobject(fieldit); fieldele; fieldele=iom_nextobject(fieldit)){
          const char *etag=iom_getobjecttag(fieldele);
          if (etag && (EQUAL(etag,"iom04.metamodel.ViewableAttributesAndRoles"))) {
            IOM_OBJECT table = GetAttrObj(model, fieldele, "viewable");
            if (table == modelele) {
              IOM_OBJECT obj = GetAttrObj(model, fieldele, "attributesAndRoles");
              int ili1AttrIdx = GetAttrObjPos(fieldele, "attributesAndRoles")-1;
              if (EQUAL(iom_getobjecttag(obj),"iom04.metamodel.RoleDef")) {
                int ili1AttrIdxOppend = atoi(iom_getattrvalue(GetAttrObj(model, obj, "oppend"), "ili1AttrIdx"));
                if (ili1AttrIdxOppend>=0) roledefs[ili1AttrIdxOppend] = obj;
              } else {
                fields[ili1AttrIdx] = obj;
              }
              if (ili1AttrIdx > maxIdx) maxIdx = ili1AttrIdx;
              //CPLDebug( "OGR_ILI", "Field %s Pos: %d", iom_getattrvalue(obj, "name"), ili1AttrIdx);
            }
          }
          iom_releaseobject(fieldele);
        }
        iom_releaseiterator(fieldit);

        OGRFieldDefn fieldDef("_TID", OFTString);
        layer->GetLayerDefn()->AddFieldDefn(&fieldDef);
        for (int i=0; i<=maxIdx; i++) {
          IOM_OBJECT obj = fields[i];
          IOM_OBJECT roleobj = roledefs[i];
          if (roleobj) AddField(layer, model, roleobj);
          if (obj) AddField(layer, model, obj);
        }
      }
    }
    iom_releaseobject(modelele);

    modelele=iom_nextobject(modelelei);
  }

  iom_releaseiterator(modelelei);

  iom_releasebasket(model);

  iom_end();

  return 0;
}

int ILI1Reader::ReadFeatures() {
    char **tokens = NULL;
    const char *firsttok = NULL;
    const char *pszLine;
    const char *topic = NULL;
    int ret = TRUE;
    
    while (ret && (tokens = ReadParseLine()))
    {
      firsttok = tokens[0];
      if (EQUAL(firsttok, "SCNT"))
      {
        //read description
        do 
        {
          pszLine = CPLReadLine( fpItf );
        }
        while (pszLine && !EQUALN(pszLine, "////", 4));
        ret = (pszLine != NULL);
      }
      else if (EQUAL(firsttok, "MOTR"))
      {
        //read model
        do 
        {
          pszLine = CPLReadLine( fpItf );
        }
        while (pszLine && !EQUALN(pszLine, "////", 4));
        ret = (pszLine != NULL);
      }
      else if (EQUAL(firsttok, "MTID"))
      {
      }
      else if (EQUAL(firsttok, "MODL"))
      {
      }
      else if (EQUAL(firsttok, "TOPI"))
      {
        topic = CPLStrdup(CSLGetField(tokens, 1));
      }
      else if (EQUAL(firsttok, "TABL"))
      {
        CPLDebug( "OGR_ILI", "Reading table '%s'", GetLayerNameString(topic, CSLGetField(tokens, 1)) );
        curLayer = GetLayerByName(GetLayerNameString(topic, CSLGetField(tokens, 1)));
        if (curLayer == NULL) { //create one
          CPLDebug( "OGR_ILI", "No model found, using default field names." );
          OGRSpatialReference *poSRSIn = NULL;
          int bWriterIn = 0;
          OGRwkbGeometryType eReqType = wkbUnknown;
          OGRILI1DataSource *poDSIn = NULL;
          curLayer = new OGRILI1Layer(GetLayerNameString(topic, CSLGetField(tokens, 1)), poSRSIn, bWriterIn, eReqType, poDSIn);
          AddLayer(curLayer);
        }
        for (int i=0; i < curLayer->GetLayerDefn()->GetFieldCount(); i++) {
          CPLDebug( "OGR_ILI", "Field %d: %s", i,  curLayer->GetLayerDefn()->GetFieldDefn(i)->GetNameRef());
        }
        ret = ReadTable();
      }
      else if (EQUAL(firsttok, "ETOP"))
      {
      }
      else if (EQUAL(firsttok, "EMOD"))
      {
      }
      else if (EQUAL(firsttok, "ENDE"))
      {
        JoinSurfaceLayers();
        PolygonizeAreaLayers();
        return TRUE;
      }
      else
      {
        CPLDebug( "OGR_ILI", "Unexpected token: %s", firsttok );
      }
      
      CSLDestroy(tokens);
    }
    return ret;
}

int ILI1Reader::AddIliGeom(OGRFeature *feature, int iField, long fpos)
{
#if defined(_WIN32) || defined(__WIN32__)
    //Other positions on Windows !?
#else
    long nBlockLen = VSIFTell( fpItf )-fpos;
    VSIFSeek( fpItf, fpos, SEEK_SET );

    char *pszRawData = (char *) CPLMalloc(nBlockLen+1);
    if( (int) VSIFRead( pszRawData, 1, nBlockLen, fpItf ) != nBlockLen )
    {
        CPLFree( pszRawData );

        CPLError( CE_Failure, CPLE_FileIO, "Read of transfer file failed." );
        return FALSE;
    }
    pszRawData[nBlockLen]= '\0';
    feature->SetField(iField, pszRawData);
#endif
    return TRUE;
}

OGRMultiPolygon* ILI1Reader::Polygonize( OGRGeometryCollection* poLines )
{
    OGRMultiPolygon *poPolygon = new OGRMultiPolygon();

#if defined(POLYGONIZE_AREAS)
    GEOSGeom *ahInGeoms = NULL;
    int       i = 0;
    
    ahInGeoms = (GEOSGeom *) CPLCalloc(sizeof(void*),poLines->getNumGeometries());
    for( i = 0; i < poLines->getNumGeometries(); i++ )
        ahInGeoms[i] = poLines->getGeometryRef(i)->exportToGEOS();

    
    GEOSGeom hResultGeom = GEOSPolygonize( ahInGeoms, 
                                           poLines->getNumGeometries() );

    for( i = 0; i < poLines->getNumGeometries(); i++ )
        GEOSGeom_destroy( ahInGeoms[i] );
    CPLFree( ahInGeoms );

    if( hResultGeom == NULL )
        return NULL;

    OGRGeometry *poMP = OGRGeometryFactory::createFromGEOS( hResultGeom );
    
    GEOSGeom_destroy( hResultGeom );

    return (OGRMultiPolygon *) poMP;

#endif

    return poPolygon;
}


void ILI1Reader::PolygonizeAreaLayers()
{
    for(int iLayer = 0; iLayer < nAreaLayers; iLayer++ )
    {
      OGRILI1Layer *poAreaLayer = papoAreaLayers[iLayer];
      OGRILI1Layer *poLineLayer = papoAreaLineLayers[iLayer];

      //add all lines from poLineLayer to collection
      OGRGeometryCollection *gc = new OGRGeometryCollection();
      poLineLayer->ResetReading();
      while (OGRFeature *feature = poLineLayer->GetNextFeatureRef())
          gc->addGeometry(feature->GetGeometryRef());

      //polygonize lines
      CPLDebug( "OGR_ILI", "Polygonizing layer %s with %d multilines", poAreaLayer->GetLayerDefn()->GetName(), gc->getNumGeometries());
      OGRMultiPolygon* polys = Polygonize( gc );

      //associate polygon feature with data row according to centroid
      int i;
      OGRPolygon emptyPoly;
#if defined(POLYGONIZE_AREAS)
      GEOSGeom *ahInGeoms = NULL;

      ahInGeoms = (GEOSGeom *) CPLCalloc(sizeof(void*),polys->getNumGeometries());
      for( i = 0; i < polys->getNumGeometries(); i++ )
      {
          ahInGeoms[i] = polys->getGeometryRef(i)->exportToGEOS();
          if (!GEOSisValid(ahInGeoms[i])) ahInGeoms[i] = NULL;
      }
      poAreaLayer->ResetReading();
      while (OGRFeature *feature = poAreaLayer->GetNextFeatureRef())
      {
        GEOSGeom point = (GEOSGeom)feature->GetGeometryRef()->exportToGEOS();
        for (i = 0; i < polys->getNumGeometries(); i++ )
        {
          if (ahInGeoms[i] && GEOSWithin(point, ahInGeoms[i]))
          {
            delete feature->StealGeometry(); //point
            feature->SetGeometry( polys->getGeometryRef(i) );
            break;
          }
        }
        if (i == polys->getNumGeometries())
        {
          CPLDebug( "OGR_ILI", "Association between area and point failed.");
          feature->SetGeometry( &emptyPoly );
        }
        GEOSGeom_destroy( point );
      }
      poAreaLayer->GetLayerDefn()->SetGeomType(wkbPolygon);
      for( i = 0; i < polys->getNumGeometries(); i++ )
          GEOSGeom_destroy( ahInGeoms[i] );
      CPLFree( ahInGeoms );
#endif
    }
}


void ILI1Reader::JoinSurfaceLayers()
{
    for(int iLayer = 0; iLayer < nSurfaceLayers; iLayer++ )
    {
      OGRILI1Layer *poSurfaceLayer = papoSurfaceLayers[iLayer];
      OGRILI1Layer *poPolyLayer = papoSurfacePolyLayers[iLayer];

      poSurfaceLayer->GetLayerDefn()->SetGeomType(poPolyLayer->GetLayerDefn()->GetGeomType());
      poSurfaceLayer->ResetReading();
      while (OGRFeature *feature = poSurfaceLayer->GetNextFeatureRef())
      {
        OGRFeature *polyfeature = poPolyLayer->GetFeatureRef(feature->GetFID());
        if (polyfeature) {
          feature->SetGeometry(polyfeature->GetGeometryRef());
        }

      }
    }
}


int ILI1Reader::ReadTable() {
    
    char **tokens = NULL;
    const char *firsttok = NULL;
    int ret = TRUE;
    int warned = FALSE;
    int fIndex;

    OGRFeatureDefn *featureDef = curLayer->GetLayerDefn();
    OGRFieldDefn *fieldDef = NULL;
    OGRFeature *feature = NULL;

    long fpos = VSIFTell(fpItf);

    while (ret && (tokens = ReadParseLine()))
    {
      firsttok = CSLGetField(tokens, 0);
      if (EQUAL(firsttok, "OBJE"))
      {
        //Check for features spread over multiple objects
        if (featureDef->GetGeomType() == wkbPolygon)
        {
          //Multiple polygon rings
          feature = curLayer->GetFeatureRef(atol(CSLGetField(tokens, 2)));
        }
        else if (featureDef->GetGeomType() == wkbGeometryCollection)
        {
          //AREA lines spread over mutltiple objects
        }
        else
        {
          feature = NULL;
        }

        if (feature == NULL)
        {
          //start new feature
          feature = new OGRFeature(featureDef);

          if (featureDef->GetFieldCount() == 0)
          {
            CPLDebug( "OGR_ILI", "No field definition found for table: %s", featureDef->GetName() );
            //Model not read - use heuristics
            for (fIndex=1; fIndex<CSLCount(tokens); fIndex++)
            {
              fieldDef = new OGRFieldDefn(CPLStrdup("Field00"), OFTString);
              *(char *)(fieldDef->GetNameRef()+strlen(fieldDef->GetNameRef())-2) = '0'+fIndex/10;
              *(char *)(fieldDef->GetNameRef()+strlen(fieldDef->GetNameRef())-1) = '0'+fIndex%10;
              featureDef->AddFieldDefn(fieldDef);
            }
          }
  
          int fieldno = 0;
          for (fIndex=1; fIndex<CSLCount(tokens) && fieldno < featureDef->GetFieldCount(); fIndex++, fieldno++)
          {
            if (!EQUAL(tokens[fIndex], "@")) {
              //CPLDebug( "OGR_ILI", "Adding Field %d: %s", fieldno, tokens[fIndex]);
              feature->SetField(fieldno, CPLStrdup(tokens[fIndex]));
              if (featureDef->GetFieldDefn(fieldno)->GetType() == OFTReal
                  && fieldno > 0
                  && featureDef->GetFieldDefn(fieldno-1)->GetType() == OFTReal
                  && featureDef->GetGeomType() == wkbPoint) {
                //add Point geometry
                OGRPoint *ogrPoint = new OGRPoint(atof(tokens[fIndex-1]), atof(tokens[fIndex]));
                feature->SetGeometryDirectly(ogrPoint);
              }
            }
          }
          if (!warned && featureDef->GetFieldCount() != CSLCount(tokens)-1 && !(featureDef->GetFieldCount() == CSLCount(tokens) && EQUAL(featureDef->GetFieldDefn(featureDef->GetFieldCount()-1)->GetNameRef(), "ILI_Geometry"))) {
            CPLDebug( "OGR_ILI", "Field count doesn't match. %d declared, %d found", featureDef->GetFieldCount(), CSLCount(tokens)-1);
            warned = TRUE;
          }
          if (featureDef->GetGeomType() == wkbPolygon)
            feature->SetFID(atol(feature->GetFieldAsString(1)));
          else if (feature->GetFieldCount() > 0)
            feature->SetFID(atol(feature->GetFieldAsString(0)));
          curLayer->AddFeature(feature);
        }
      }
      else if (EQUAL(firsttok, "STPT"))
      {
        ReadGeom(tokens, featureDef->GetGeomType(), feature);
        if (EQUAL(featureDef->GetFieldDefn(featureDef->GetFieldCount()-1)->GetNameRef(), "ILI_Geometry"))
        {
          AddIliGeom(feature, featureDef->GetFieldCount()-1, fpos); //TODO: append multi-OBJECT geometries
        }
      }
      else if (EQUAL(firsttok, "ELIN"))
      {
        //empty geom
      }
      else if (EQUAL(firsttok, "EDGE"))
      {
        tokens = ReadParseLine(); //STPT
        ReadGeom(tokens, wkbMultiLineString, feature);
        if (EQUAL(featureDef->GetFieldDefn(featureDef->GetFieldCount()-1)->GetNameRef(), "ILI_Geometry"))
        {
          AddIliGeom(feature, featureDef->GetFieldCount()-1, fpos);
        }
      }
      else if (EQUAL(firsttok, "PERI"))
      {
      }
      else if (EQUAL(firsttok, "ETAB"))
      {
        CSLDestroy(tokens);
        return TRUE;
      }
      else
      {
        CPLDebug( "OGR_ILI", "Unexpected token: %s", firsttok );
      }
      
      CSLDestroy(tokens);
      fpos = VSIFTell(fpItf);
    }
    
    return ret;
}

void ILI1Reader::ReadGeom(char **stgeom, OGRwkbGeometryType eType, OGRFeature *feature) {
    
    char **tokens = NULL;
    const char *firsttok = NULL;
    int end = FALSE;
    int isArc = FALSE;
    OGRLineString *ogrLine = NULL; //current line
    OGRPolygon *ogrPoly = NULL; //current polygon
    OGRPoint ogrPoint, arcPoint, endPoint; //points for arc interpolation
    OGRMultiLineString *ogrMultiLine = NULL; //current multi line

    //tokens = ["STPT", "1111", "22222"]
    ogrPoint.setX(atof(stgeom[1])); ogrPoint.setY(atof(stgeom[2]));
    ogrLine = (eType == wkbPolygon) ? new OGRLinearRing() : new OGRLineString();
    ogrLine->addPoint(&ogrPoint);

    //Set feature geometry
    if (eType == wkbMultiLineString)
    {
      ogrMultiLine = new OGRMultiLineString();
      feature->SetGeometryDirectly(ogrMultiLine);
    }
    else if (eType == wkbGeometryCollection) //AREA
    {
      if (feature->GetGeometryRef())
        ogrMultiLine = (OGRMultiLineString *)feature->GetGeometryRef();
      else
      {
        ogrMultiLine = new OGRMultiLineString();
        feature->SetGeometryDirectly(ogrMultiLine);
      }
    }
    else if (eType == wkbPolygon)
    {
      if (feature->GetGeometryRef())
        ogrPoly = (OGRPolygon *)feature->GetGeometryRef();
      else
      {
        ogrPoly = new OGRPolygon();
        feature->SetGeometryDirectly(ogrPoly);
      }
    }
    else
    {
      feature->SetGeometryDirectly(ogrLine);
    }

    //Parse geometry
    while (!end && (tokens = ReadParseLine()))
    {
      firsttok = CSLGetField(tokens, 0);
      if (EQUAL(firsttok, "LIPT"))
      {
        if (isArc) {
          endPoint.setX(atof(tokens[1])); endPoint.setY(atof(tokens[2]));
          interpolateArc(ogrLine, &ogrPoint, &arcPoint, &endPoint, arcIncr);
        }
        ogrPoint.setX(atof(tokens[1])); ogrPoint.setY(atof(tokens[2])); isArc = FALSE;
        ogrLine->addPoint(&ogrPoint);
      }
      else if (EQUAL(firsttok, "ARCP"))
      {
        isArc = TRUE;
        arcPoint.setX(atof(tokens[1])); arcPoint.setY(atof(tokens[2]));
      }
      else if (EQUAL(firsttok, "ELIN"))
      {
        if (ogrMultiLine)
        {
          ogrMultiLine->addGeometryDirectly(ogrLine);
        }
        if (ogrPoly)
        {
          ogrPoly->addRingDirectly((OGRLinearRing *)ogrLine);
        }
        end = TRUE;
      }
      else if (EQUAL(firsttok, "EEDG"))
      {
        end = TRUE;
      }
      else if (EQUAL(firsttok, "LATT"))
      {
        //Line Attributes (ignored)
      }
      else if (EQUAL(firsttok, "EFLA"))
      {
        end = TRUE;
      }
      else if (EQUAL(firsttok, "ETAB"))
      {
        end = TRUE;
      }
      else
      {
        CPLDebug( "OGR_ILI", "Unexpected token: %s", firsttok );
      }
    
      CSLDestroy(tokens);
    }
}

/************************************************************************/
/*                              AddLayer()                              */
/************************************************************************/

void ILI1Reader::AddLayer( OGRILI1Layer * poNewLayer )

{
    papoLayers = (OGRILI1Layer **)
        CPLRealloc( papoLayers, sizeof(void*) * ++nLayers );
    
    papoLayers[nLayers-1] = poNewLayer;
}

/************************************************************************/
/*                              AddAreaLayer()                              */
/************************************************************************/

void ILI1Reader::AddAreaLayer( OGRILI1Layer * poAreaLayer,  OGRILI1Layer * poLineLayer )

{
    ++nAreaLayers;

    papoAreaLayers = (OGRILI1Layer **)
        CPLRealloc( papoAreaLayers, sizeof(void*) * nAreaLayers );
    papoAreaLineLayers = (OGRILI1Layer **)
        CPLRealloc( papoAreaLineLayers, sizeof(void*) * nAreaLayers );
    
    papoAreaLayers[nAreaLayers-1] = poAreaLayer;
    papoAreaLineLayers[nAreaLayers-1] = poLineLayer;
}

/************************************************************************/
/*                              AddSurfaceLayer()                              */
/************************************************************************/

void ILI1Reader::AddSurfaceLayer( OGRILI1Layer * poDataLayer,  OGRILI1Layer * poPolyLayer )

{
    ++nSurfaceLayers;

    papoSurfaceLayers = (OGRILI1Layer **)
        CPLRealloc( papoSurfaceLayers, sizeof(void*) * nSurfaceLayers );
    papoSurfacePolyLayers = (OGRILI1Layer **)
        CPLRealloc( papoSurfacePolyLayers, sizeof(void*) * nSurfaceLayers );
    
    papoSurfaceLayers[nSurfaceLayers-1] = poDataLayer;
    papoSurfacePolyLayers[nSurfaceLayers-1] = poPolyLayer;
}

/************************************************************************/
/*                              GetLayer()                              */
/************************************************************************/

OGRILI1Layer *ILI1Reader::GetLayer( int iLayer )

{
    if( iLayer < 0 || iLayer >= nLayers )
        return NULL;
    else
        return papoLayers[iLayer];
}

OGRILI1Layer *ILI1Reader::GetLayerByName( const char* pszLayerName )

{
    for(int iLayer = 0; iLayer < nLayers; iLayer++ )
    {
        if( EQUAL(pszLayerName,
                  papoLayers[iLayer]->GetLayerDefn()->GetName()) )
            return papoLayers[iLayer];
    }
    return NULL;
}

/************************************************************************/
/*                           GetLayerCount()                            */
/************************************************************************/

int ILI1Reader::GetLayerCount()

{
    return nLayers;
}

/************************************************************************/
/*     Read one logical line, and return split into fields.  The return */
/*     result is a stringlist, in the sense of the CSL functions.       */
/************************************************************************/

char ** ILI1Reader::ReadParseLine()
{
    const char  *pszLine;
    char **tokens;
    char **conttok;
    char *token;

    CPLAssert( fpItf != NULL );
    if( fpItf == NULL )
        return( NULL );
    
    pszLine = CPLReadLine( fpItf );
    if( pszLine == NULL )
        return( NULL );
    
    if (strlen(pszLine) == 0) return NULL;
      
    tokens = CSLTokenizeString2( pszLine, " ", CSLT_PRESERVEESCAPES );
    token = tokens[CSLCount(tokens)-1];
    
    //Append CONT lines
    while (strlen(pszLine) && EQUALN(token, "\\", 2))
    {
       //remove last token
      CPLFree(tokens[CSLCount(tokens)-1]);
      tokens[CSLCount(tokens)-1] = NULL;
      
      pszLine = CPLReadLine( fpItf );
      conttok = CSLTokenizeString2( pszLine, " ", CSLT_PRESERVEESCAPES );
      if (!conttok || !EQUAL(conttok[0], "CONT")) break;
      
      //append
      tokens = CSLInsertStrings(tokens, -1, &conttok[1]);
      token = tokens[CSLCount(tokens)-1];
      
      CSLDestroy(conttok);
    }
    return tokens;
}



IILI1Reader *CreateILI1Reader() {
    return new ILI1Reader();
}
