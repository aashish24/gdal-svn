/******************************************************************************
 * $Id$
 *
 * Name:     osr_java.i
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

%include arrays_java.i
%include typemaps_java.i

%pragma(java) jniclasscode=%{
  private static boolean available = false;

  static {
    try {
      System.loadLibrary("osrjni");
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

/*
 *  Needed to make the Constructor and getCptr 'public' and not 'protected'.
 *   There is likely a better way to do this (with javamethodmodifiers) but
 *   none worked for me. 
 */ 
%typemap(javabody) OSRSpatialReferenceShadow, OSRCoordinateTransformationShadow %{
  private long swigCPtr;
  protected boolean swigCMemOwn;

  public $javaclassname(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }
  
  public static long getCPtr($javaclassname obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }
%}

%typemap(javacode) OSRSpatialReferenceShadow %{

  public String toString() {
    return __str__();
  }

  public String ExportToWkt() {
    String array[] = new String[] {null};
    ExportToWkt(array);
    return array[0];
  }

  public String ExportToPrettyWkt(int simplify) {
    String array[] = new String[] {null};
    ExportToPrettyWkt(array, simplify);
    return array[0];
  }

  public String ExportToPrettyWkt() {
    String array[] = new String[] {null};
    ExportToPrettyWkt(array);
    return array[0];
  }

  public String ExportToProj4() {
    String array[] = new String[] {null};
    ExportToProj4(array);
    return array[0];
  }

  public String ExportToXML( String dialect) {
    String array[] = new String[] {null};
    ExportToXML(array, dialect);
    return array[0];
  }

  public String ExportToXML() {
    String array[] = new String[] {null};
    ExportToXML(array);
    return array[0];
  }

  public String ExportToMICoordSys() {
    String array[] = new String[] {null};
    ExportToMICoordSys(array);
    return array[0];
  }

  public double[] GetTOWGS84()
  {
      double array[] = new double[7];
      GetTOWGS84(array);
      return array;
  }
%}


%typemap(javacode) OSRCoordinateTransformationShadow %{
  public double[] TransformPoint(double x, double y, double z) {
    double[] ret = new double[3];
    TransformPoint(ret, x, y, z);
    return ret;
  }

  public double[] TransformPoint(double x, double y) {
    return TransformPoint(x, y, 0);
  }
%}
    
/******************************************************************************
 *
 *  Global methods
 *
 */

/************************************************************************/
/*                        GetWellKnownGeogCSAsWKT()                     */
/************************************************************************/

%{
typedef char retStringAndCPLFree;
%}

%inline %{
retStringAndCPLFree* GetWellKnownGeogCSAsWKT( const char *name ) {
  char* argout = NULL;
  OGRSpatialReferenceH srs = OSRNewSpatialReference("");
  OGRErr rcode = OSRSetWellKnownGeogCS( srs, name );
  if( rcode == OGRERR_NONE )
      rcode = OSRExportToWkt ( srs, &argout );  
  OSRDestroySpatialReference( srs );
  return argout;
}
%}

/************************************************************************/
/*                           GetUserInputAsWKT()                        */
/************************************************************************/
%inline %{
retStringAndCPLFree* GetUserInputAsWKT( const char *name ) {
  char* argout = NULL;
  OGRSpatialReferenceH srs = OSRNewSpatialReference("");
  OGRErr rcode = OSRSetFromUserInput( srs, name );
  if( rcode == OGRERR_NONE )
      rcode = OSRExportToWkt ( srs, &argout );  
  OSRDestroySpatialReference( srs );
  return argout;
}
%}
