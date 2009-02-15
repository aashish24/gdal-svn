/******************************************************************************
 * $Id$
 *
 * Name:     ogr_java.i
 * Project:  GDAL SWIG Interface
 * Purpose:  Typemaps for Java bindings
 * Author:   Benjamin Collins, The MITRE Corporation
 *
 *
 * $Log$
 * Revision 1.2  2006/02/16 17:21:12  collinsb
 * Updates to Java bindings to keep the code from halting execution if the native libraries cannot be found.
 *
 * Revision 1.1  2006/02/02 20:56:07  collinsb
 * Added Java specific typemap code
 *
 *
*/
#ifdef SWIGJAVA
%{
typedef char retStringAndCPLFree;
%}
#endif

%pragma(java) jniclasscode=%{
  private static boolean available = false;

  static {
    try {
      System.loadLibrary("ogrjni");
      available = true;
    } catch (UnsatisfiedLinkError e) {
      available = false;
      System.err.println("Native library load failed.");
      System.err.println(e);
    }
  }
  
  public static boolean isAvailable() {
    return available;
  }
%}


%rename (GetFieldType) GetType;
%rename (GetDriverCount) OGRGetDriverCount;
%rename (GetOpenDSCount) OGRGetOpenDSCount;
%rename (SetGenerate_DB2_V72_BYTE_ORDER) OGRSetGenerate_DB2_V72_BYTE_ORDER;
%rename (RegisterAll) OGRRegisterAll();


%rename (GeometryTypeToName) OGRGeometryTypeToName;
const char *OGRGeometryTypeToName( OGRwkbGeometryType eType );


/*
 *
 */ 
 
%pragma(java) jniclassimports=%{
import org.gdal.osr.SpatialReference;
import org.gdal.osr.CoordinateTransformation;
%}
 
%pragma(java) moduleimports=%{
import org.gdal.osr.SpatialReference;
%}

%typemap(javaimports) OGRLayerShadow %{
import org.gdal.osr.SpatialReference;
%}
%typemap(javaimports) OGRDataSourceShadow %{
import org.gdal.osr.SpatialReference;
%}

%typemap(javacode) OGRDataSourceShadow %{

  public boolean equals(Object obj) {
    boolean equal = false;
    if (obj instanceof $javaclassname)
      equal = ((($javaclassname)obj).swigCPtr == this.swigCPtr);
    return equal;
  }

  public int hashCode() {
     return (int)swigCPtr;
  }

  public Layer GetLayer(int index)
  {
      return GetLayerByIndex(index);
  }

  public Layer GetLayer(String layerName)
  {
      return GetLayerByName(layerName);
  }

%}

%typemap(javacode) OGRLayerShadow %{
  private Object parentReference;

  protected static long getCPtrAndDisown($javaclassname obj) {
    if (obj != null) obj.swigCMemOwn= false;
    obj.parentReference = null;
    return getCPtr(obj);
  }

  /* Ensure that the GC doesn't collect any parent instance set from Java */
  protected void addReference(Object reference) {
    parentReference = reference;
  }

  public boolean equals(Object obj) {
    boolean equal = false;
    if (obj instanceof $javaclassname)
      equal = ((($javaclassname)obj).swigCPtr == this.swigCPtr);
    return equal;
  }

  public int hashCode() {
     return (int)swigCPtr;
  }

  public int GetExtent(double[] argout, boolean force)
  {
      return GetExtent(argout, (force) ? 1 : 0);
  }

  public double[] GetExtent(boolean force)
  {
      double[] argout = new double[4];
      try
      {
          GetExtent(argout, force);
          return argout;
      }
      catch(RuntimeException e)
      {
          return null;
      }
  }
%}

%typemap(javaimports) OGRGeometryShadow %{
import org.gdal.ogr.ogr;
import org.gdal.osr.SpatialReference;
import org.gdal.osr.CoordinateTransformation;
%}


%typemap(javainterfaces) OGRGeometryShadow "Cloneable"

%typemap(javacode) OGRGeometryShadow %{
  private Object parentReference;

  protected static long getCPtrAndDisown($javaclassname obj) {
    if (obj != null) obj.swigCMemOwn= false;
    obj.parentReference = null;
    return getCPtr(obj);
  }

  /* Ensure that the GC doesn't collect any parent instance set from Java */
  protected void addReference(Object reference) {
    parentReference = reference;
  }

  public boolean equals(Object obj) {
    boolean equal = false;
    if (obj instanceof $javaclassname)
      equal = Equal(($javaclassname)obj);
    return equal;
  }

  public int hashCode() {
     return (int)swigCPtr;
  }

  public Object clone()
  {
      return Clone();
  }

  public double[] GetPoint_2D(int iPoint)
  {
      double[] coords = new double[2];
      GetPoint_2D(iPoint, coords);
      return coords;
  }

  public double[] GetPoint(int iPoint)
  {
      double[] coords = new double[3];
      GetPoint(iPoint, coords);
      return coords;
  }

  public static Geometry CreateFromWkt(String wkt)
  {
      return ogr.CreateGeometryFromWkt(wkt);
  }

  public static Geometry CreateFromWkb(byte wkb[])
  {
      return ogr.CreateGeometryFromWkb(wkb);
  }

  public static Geometry CreateFromGML(String gml)
  {
      return ogr.CreateGeometryFromGML(gml);
  }

  public static Geometry CreateFromJson(String json)
  {
      return ogr.CreateGeometryFromJson(json);
  }
%}


// Add a Java reference to prevent premature garbage collection and resulting use
// of dangling C++ pointer. Intended for methods that return pointers or
// references to a member variable.
%typemap(javaout) OGRGeometryShadow* GetSpatialFilter,
                  OGRFeatureDefnShadow* GetLayerDefn,
                  OGRLayerShadow* CreateLayer,
                  OGRLayerShadow* CopyLayer,
                  OGRLayerShadow* GetLayerByIndex,
                  OGRLayerShadow* GetLayerByName,
                  OGRLayerShadow* ExecuteSQL,
                  OGRFeatureDefnShadow* GetDefnRef,
                  OGRFieldDefnShadow* GetFieldDefnRef,
                  OGRFieldDefnShadow* GetFieldDefn,
                  OGRGeometryShadow* GetGeometryRef {
    long cPtr = $jnicall;
    $javaclassname ret = null;
    if (cPtr != 0) {
      ret = new $javaclassname(cPtr, $owner);
      ret.addReference(this);
    }
    return ret;
  }


%include typemaps_java.i
