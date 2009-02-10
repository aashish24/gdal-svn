/******************************************************************************
 * $Id$
 *
 * Name:     typemaps_java.i
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

%include "arrays_java.i";
%include "typemaps.i"

/* DISOWN implementation */
%typemap(javacode) SWIGTYPE %{
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

%}

%typemap(javain) SWIGTYPE *DISOWN "$javaclassname.getCPtrAndDisown($javainput)" 


/* JAVA TYPEMAPS */

/***************************************************
 * Typemaps for (double *val, int*hasval)
 ***************************************************/

%typemap(in) (double *val, int*hasval) ( double tmpval, int tmphasval ) {
  /* %typemap(in,numinputs=0) (double *val, int*hasval) */
  $1 = &tmpval;
  $2 = &tmphasval;
  if($input == NULL || jenv->GetArrayLength($input) < 1) {
    SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "null array or empty array");
    return $null;
  }
}
%typemap(argout) (double *val, int*hasval) {
  /* %typemap(argout) (double *val, int*hasval) */
  const jclass Double = jenv->FindClass("java/lang/Double");
  const jmethodID ctor = jenv->GetMethodID(Double, "<init>",
    "(D)V");
  if(*$2) {
    jobject dbl = jenv->NewObject(Double, ctor, tmpval$argnum);
    jenv->SetObjectArrayElement($input, (jsize)0, dbl);
  } else {
    jenv->SetObjectArrayElement($input, (jsize)0, 0);
  }
}

%typemap(jni) (double *val, int*hasval) "jobjectArray"
%typemap(jtype) (double *val, int*hasval) "Double[]"
%typemap(jstype) (double *val, int*hasval) "Double[]"
%typemap(javain) (double *val, int*hasval) "$javainput"
%typemap(javaout) (double *val, int*hasval) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for (GDALColorEntry *)
 ***************************************************/

%typemap(in) (GDALColorEntry *) (GDALColorEntry tmp) {
  /* %typemap(in) (GDALColorEntry *) (GDALColorEntry tmp) */
  $1 = NULL;
  float *colorptr = 0;
  const jclass Color = jenv->FindClass("java/awt/Color");
  const jmethodID colors = jenv->GetMethodID(Color, "getRGBComponents",
    "([F)[F");

  jfloatArray colorArr = jenv->NewFloatArray(4);
  colorArr = (jfloatArray)jenv->CallObjectMethod($input, colors, colorArr); 

  colorptr = (float *)jenv->GetFloatArrayElements(colorArr, 0);
  tmp.c1 = (short)(colorptr[0] * 255);
  tmp.c2 = (short)(colorptr[1] * 255);
  tmp.c3 = (short)(colorptr[2] * 255);
  tmp.c4 = (short)(colorptr[3] * 255);
  /*printf( "  %d, %d, %d, %d\n",
                    tmp.c1, tmp.c2, tmp.c3, tmp.c4 );*/
  $1 = &tmp;
}

%typemap(out) (GDALColorEntry *) {
  /* %typemap(out) (GDALColorEntry *) */
  const jclass Color = jenv->FindClass("java/awt/Color");
  const jmethodID ccon = jenv->GetMethodID(Color, "<init>",
    "(IIII)V");
  $result = jenv->NewObject(Color, ccon, $1->c1, $1->c2, $1->c3, $1->c4);
}

%typemap(jni) (GDALColorEntry *) "jobject"
%typemap(jtype) (GDALColorEntry *) "java.awt.Color"
%typemap(jstype) (GDALColorEntry *) "java.awt.Color"
%typemap(javain) (GDALColorEntry *) "$javainput"
%typemap(javaout) (GDALColorEntry *) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for (int nGCPs, GDAL_GCP const * pGCPs)
 ***************************************************/

%typemap(in, numinputs=1) (int nGCPs, GDAL_GCP const * pGCPs)
{
  /* %typemap(in, numinputs=1) (int nGCPs, GDAL_GCP const * pGCPs) */
  if ($input)
  {
    $1 = jenv->GetArrayLength($input);
    if ($1 == 0)
       $2 = NULL;
    else
    {
        $2 = (GDAL_GCP*) malloc(sizeof(GDAL_GCP) * $1);
        int i;
        for (i = 0; i<$1; i++) {
            jobject obj = (jobject)jenv->GetObjectArrayElement($input, i);
            if (obj == NULL)
            {
                free ($2 );
                SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "null object in array");
                return $null;
            }
            const jclass klass = jenv->FindClass("org/gdal/gdal/GCP");
            const jmethodID getCPtr = jenv->GetStaticMethodID(klass, "getCPtr", "(Lorg/gdal/gdal/GCP;)J");
            $2[i] = *(GDAL_GCP*) jenv->CallStaticLongMethod(klass, getCPtr, obj);
        }
    }
  }
  else
  {
    $1 = 0;
    $2 = NULL;
  }
}


%typemap(freearg) (int nGCPs, GDAL_GCP const * pGCPs)
{
  /* %typemap(freearg) (int nGCPs, GDAL_GCP const * pGCPs) */
  if ($2) {
    free((void*) $2);
  }
}
%typemap(jni) (int nGCPs, GDAL_GCP const * pGCPs ) "jobjectArray"
%typemap(jtype) (int nGCPs, GDAL_GCP const * pGCPs ) "GCP[]"
%typemap(jstype) (int nGCPs, GDAL_GCP const * pGCPs ) "GCP[]"
%typemap(javain) (int nGCPs, GDAL_GCP const * pGCPs ) "$javainput"
%typemap(javaout) (int nGCPs, GDAL_GCP const * pGCPs ) {
    return $jnicall;
  }

/***************************************************
 * Typemap argout of GDAL_GCP* used in Dataset::GetGCPs( )
 ***************************************************/

%typemap(in, numinputs=1) (int *nGCPs, GDAL_GCP const **pGCPs ) (int nGCPs=0, GDAL_GCP *pGCPs=0 )
{
  /* %typemap(in,numinputs=1) (int *nGCPs, GDAL_GCP const **pGCPs ) */
  $1 = &nGCPs;
  $2 = &pGCPs;
}
%typemap(argout) (int *nGCPs, GDAL_GCP const **pGCPs )
{
  /* %typemap(argout) (int *nGCPs, GDAL_GCP const **pGCPs ) */
  const jclass GCPClass = jenv->FindClass("org/gdal/gdal/GCP");
  const jclass vectorClass = jenv->FindClass("java/util/Vector");
  const jmethodID add = jenv->GetMethodID(vectorClass, "add", "(Ljava/lang/Object;)Z");
  const jmethodID GCPcon = jenv->GetMethodID(GCPClass, "<init>",
    "(DDDDDLjava/lang/String;Ljava/lang/String;)V");

  for( int i = 0; i < *$1; i++ ) {
    jstring stringInfo = jenv->NewStringUTF((*$2)[i].pszInfo);
    jstring stringId = jenv->NewStringUTF((*$2)[i].pszId);
    jobject GCPobj = jenv->NewObject(GCPClass, GCPcon, 
                                (*$2)[i].dfGCPX,
                                (*$2)[i].dfGCPY,
                                (*$2)[i].dfGCPZ,
                                (*$2)[i].dfGCPPixel,
                                (*$2)[i].dfGCPLine,
                                stringInfo,
                                stringId);
    jenv->DeleteLocalRef(stringInfo);
    jenv->DeleteLocalRef(stringId);
    jenv->CallBooleanMethod($input, add, GCPobj);
  }
}

%typemap(jni) (int *nGCPs, GDAL_GCP const **pGCPs ) "jobject"
%typemap(jtype) (int *nGCPs, GDAL_GCP const **pGCPs ) "java.util.Vector"
%typemap(jstype) (int *nGCPs, GDAL_GCP const **pGCPs ) "java.util.Vector"
%typemap(javain) (int *nGCPs, GDAL_GCP const **pGCPs ) "$javainput"
%typemap(javaout) (int *nGCPs, GDAL_GCP const **pGCPs ) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for (int nLen, unsigned char *pBuf )
 ***************************************************/

%typemap(in) (int nLen, unsigned char *pBuf ) (jboolean isCopy)
{
  /* %typemap(in) (int nLen, unsigned char *pBuf ) */
  $1 = 0;
  $2 = NULL;
  if ($input)
  {
    $1 = jenv->GetArrayLength($input);
    $2 = (unsigned char *)jenv->GetByteArrayElements($input, &isCopy);
  }
}

%typemap(argout) (int nLen, unsigned char *pBuf )
{
  /* %typemap(argout) (int nLen, unsigned char *pBuf ) */
}

%typemap(freearg) (int nLen, unsigned char *pBuf )
{
  /* %typemap(freearg) (int nLen, unsigned char *pBuf ) */
  /* This calls JNI_ABORT, so any modifications will not be passed back
      into the Java caller
   */
  if($2 && isCopy$argnum == JNI_TRUE) {
    jenv->ReleaseByteArrayElements($input, (jbyte *)$2, JNI_ABORT);
  }
}

%typemap(jni) (int nLen, unsigned char *pBuf ) "jbyteArray"
%typemap(jtype) (int nLen, unsigned char *pBuf ) "byte[]"
%typemap(jstype) (int nLen, unsigned char *pBuf ) "byte[]"
%typemap(javain) (int nLen, unsigned char *pBuf ) "$javainput"
%typemap(javaout) (int nLen, unsigned char *pBuf ) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for  (const char *pszHex, int *pnBytes)
 ***************************************************/

%typemap(in) (const char *pszHex, int *pnBytes) (int nBytes)
{
    /* %typemap(in) (const char *pszHex, int *pnBytes) (int nBytes) */
    if ($input)
    {
        $1 = (char *)jenv->GetStringUTFChars($input, 0);
    }
    else
    {
        SWIG_JavaException(jenv, SWIG_ValueError, "Received a NULL pointer."); return $null; 
    }
    $2 = &nBytes;
}

%typemap(argout) (const char *pszHex, int *pnBytes)
{
    /* %typemap(argout) (const char *pszHex, int *pnBytes) */
    if ($input)
    {
        jenv->ReleaseStringUTFChars($input, (char*)$1);
    }
    $result = jenv->NewByteArray(nBytes$argnum);
    jenv->SetByteArrayRegion($result, (jsize)0, (jsize)nBytes$argnum, (jbyte*)result);
    CPLFree(result);
}

%typemap(jni) (const char *pszHex, int *pnBytes) "jstring"
%typemap(jtype) (const char *pszHex, int *pnBytes) "String"
%typemap(jstype) (const char *pszHex, int *pnBytes) "String"
%typemap(javain) (const char *pszHex, int *pnBytes) "$javainput"
%typemap(javaout) (const char *pszHex, int *pnBytes) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for  (GByte* outBytes )
 ***************************************************/

%typemap(out) (GByte* outBytes )
{
    /* %typemap(out) (GByte* outBytes ) */
}

%typemap(jni) (GByte* outBytes ) "jbyteArray"
%typemap(jtype) (GByte* outBytes ) "byte[]"
%typemap(jstype) (GByte* outBytes ) "byte[]"
%typemap(javain) (GByte* outBytes ) "$javainput"
%typemap(javaout) (GByte* outBytes ) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for  (const char* stringWithDefaultValue)
 ***************************************************/

%typemap(in) (const char* stringWithDefaultValue)
{
  /* %typemap(in) (const char* stringWithDefaultValue) */
  if ($input)
  {
    $1 = (char *)jenv->GetStringUTFChars($input, 0);
  }
}
%typemap(freearg) (const char* stringWithDefaultValue)
{
  /* %typemap(freearg) (const char* stringWithDefaultValue) */
  if ($input)
  {
    jenv->ReleaseStringUTFChars($input, (char*)$1);
  }
}

%typemap(jni) (const char* stringWithDefaultValue) "jstring"
%typemap(jtype) (const char* stringWithDefaultValue) "String"
%typemap(jstype) (const char* stringWithDefaultValue) "String"
%typemap(javain) (const char* stringWithDefaultValue) "$javainput"
%typemap(javaout) (const char* stringWithDefaultValue) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for  (tostring argin)
 ***************************************************/

%typemap(in) (tostring argin)
{
  /* %typemap(in) (tostring argin) */
  if ($input)
    $1 = (char *)jenv->GetStringUTFChars($input, 0);
}
%typemap(freearg) (tostring argin)
{
  /* %typemap(in) (tostring argin) */
  if ($input)
    jenv->ReleaseStringUTFChars($input, (char*)$1);
}

%typemap(jni) (tostring argin) "jstring"
%typemap(jtype) (tostring argin) "String"
%typemap(jstype) (tostring argin) "String"
%typemap(javain) (tostring argin) "$javainput"
%typemap(javaout) (tostring argin) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for  (char **ignorechange)
 ***************************************************/
%typemap(out) (retStringAndCPLFree*)
{
    /* %typemap(out) (retStringAndCPLFree*) */
    if(result)
    {
        $result = jenv->NewStringUTF((const char *)result);
        CPLFree(result);
    }
}

%typemap(jni) (retStringAndCPLFree*) "jstring"
%typemap(jtype) (retStringAndCPLFree*) "String"
%typemap(jstype) (retStringAndCPLFree*) "String"
%typemap(javain) (retStringAndCPLFree*) "$javainput"
%typemap(javaout) (retStringAndCPLFree*) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for  (char **ignorechange)
 ***************************************************/

%typemap(in) (char **ignorechange) (char* ori_val, char *val)
{
  /* %typemap(in) (char **ignorechange) */
  ori_val = val = (char *)jenv->GetStringUTFChars($input, 0);
  $1 = &val;
}
%typemap(freearg) (char **ignorechange)
{
  /* %typemap(freearg) (char **ignorechange) */
  jenv->ReleaseStringUTFChars($input, ori_val$argnum);
}

%typemap(jni) (char **ignorechange) "jstring"
%typemap(jtype) (char **ignorechange) "String"
%typemap(jstype) (char **ignorechange) "String"
%typemap(javain) (char **ignorechange) "$javainput"
%typemap(javaout) (char **ignorechange) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for  (int *nLen, char **pBuf)
 ***************************************************/

%typemap(in,numinputs=0) (int *nLen, char **pBuf ) ( int nLen, char *pBuf )
{
  /* %typemap(in) (int *nLen, char **pBuf ) */
  $1 = &nLen;
  $2 = &pBuf;
}

%typemap(argout) (int *nLen, char **pBuf )
{
  /* %typemap(argout) (int *nLen, char **pBuf ) */
  jbyteArray byteArray = jenv->NewByteArray(nLen$argnum);
  jenv->SetByteArrayRegion(byteArray, (jsize)0, (jsize)nLen$argnum, (jbyte*)pBuf$argnum);
  $result = byteArray;
}

%typemap(freearg) (int *nLen, char **pBuf )
{
  /* %typemap(freearg) (int *nLen, char **pBuf ) */
  if( nLen$argnum ) {
    CPLFree( pBuf$argnum );
  }
}

%typemap(jni) (int *nLen, char **pBuf ) "jbyteArray"
%typemap(jtype) (int *nLen, char **pBuf ) "byte[]"
%typemap(jstype) (int *nLen, char **pBuf ) "byte[]"
%typemap(javain) (int *nLen, char **pBuf ) "$javainput"
%typemap(javaout) (int *nLen, char **pBuf ) {
    return $jnicall;
  }


%import "ogr_error_map.i"

%typemap(out,fragment="OGRErrMessages") OGRErr
{
  /* %typemap(out) OGRErr */
  if (result != 0) {
    SWIG_JavaThrowException(jenv, SWIG_JavaRuntimeException,
      OGRErrMessages(result));
    return $null;
  }
  $result = (jint)result;
}
%typemap(ret) OGRErr
{
  /* %typemap(ret) OGRErr */

}


/* GDAL Typemaps */

%typemap(out) IF_ERR_RETURN_NONE
{
  /* %typemap(out) IF_ERR_RETURN_NONE */
  $result = 0;
}
%typemap(ret) IF_ERR_RETURN_NONE
{
 /* %typemap(ret) IF_ERR_RETURN_NONE */
}
%typemap(out) IF_FALSE_RETURN_NONE
{
  /* %typemap(out) IF_FALSE_RETURN_NONE */
  $result = 0;
}
%typemap(ret) IF_FALSE_RETURN_NONE
{
 /* %typemap(ret) IF_FALSE_RETURN_NONE */
}

/***************************************************
 *
 *  Java typemaps for (int nList, int* pList)
 *
 ***************************************************/ 
%typemap(in) (int nList, int* pList)
{
  /* %typemap(in) (int nList, int* pList) */
  /* check if is List */
  if ($input)
  {
    $1 = jenv->GetArrayLength($input);
    if ($1 == 0)
       $2 = NULL;
    else
       $2 = (int *)jenv->GetIntArrayElements($input, NULL);
  }
  else
  {
    $1 = 0;
    $2 = NULL;
  }
}

%typemap(argout) (int nList, int* pList)
{
  /* %typemap(argout) (int nList, int* pList) */
}

%typemap(freearg) (int nList, int* pList)
{
  /* %typemap(freearg) (int nList, int* pList) */
  if ($2) {
    jenv->ReleaseIntArrayElements($input, $2, JNI_ABORT);
  }
}

%typemap(jni) (int nList, int* pList) "jintArray"
%typemap(jtype) (int nList, int* pList) "int[]"
%typemap(jstype) (int nList, int* pList) "int[]"
%typemap(javain) (int nLen, int* pList ) "$javainput"
%typemap(javaout) (int nLen, int* pList ) {
    return $jnicall;
  }


/***************************************************
 *
 *  Java typemaps for (int nList, int* pListOut)
 *
 ***************************************************/ 
%typemap(in) (int nList, int* pListOut)
{
  /* %typemap(in) (int nList, int* pListOut) */
  /* check if is List */
  if ($input)
  {
    $1 = jenv->GetArrayLength($input);
    $2 = (int*) malloc($1 * sizeof(int));
  }
  else
  {
    $1 = 0;
    $2 = NULL;
  }
}

%typemap(argout) (int nList, int* pListOut)
{
  /* %typemap(argout) (int nList, int* pListOut) */
}

%typemap(freearg) (int nList, int* pListOut)
{
  /* %typemap(freearg) (int nList, int* pListOut) */
  if ($2) {
    jenv->SetIntArrayRegion($input, 0, $1, $2);
    free((void*) $2);
  }
}

%typemap(jni) (int nList, int* pListOut) "jintArray"
%typemap(jtype) (int nList, int* pListOut) "int[]"
%typemap(jstype) (int nList, int* pListOut) "int[]"
%typemap(javain) (int nLen, int* pListOut ) "$javainput"
%typemap(javaout) (int nLen, int* pListOut ) {
    return $jnicall;
  }


/***************************************************
 *
 *  Java typemaps for (int* pnList, int** ppListOut)
 *
 ***************************************************/ 
%typemap(in) (int* pnList, int** ppListOut) (int nLen, int* pBuf)
{
  /* %typemap(in) (int* pnList, int** ppListOut) */
  $1 = &nLen;
  $2 = &pBuf;
}

%typemap(argout) (int* pnList, int** ppListOut)
{
  /* %typemap(argout) (int* pnList, int** ppListOut) */
  if($input && jenv->GetArrayLength($input) >= 1)
  {
    jintArray intArray = jenv->NewIntArray(nLen$argnum);
    jenv->SetIntArrayRegion(intArray, (jsize)0, (jsize)nLen$argnum, pBuf$argnum);
    jenv->SetObjectArrayElement($input,0,intArray);
    jenv->DeleteLocalRef(intArray);
  }
  CPLFree(pBuf$argnum);
}

%typemap(freearg) (int* pnList, int** ppListOut)
{
  /* %typemap(freearg) (int* pnList, int** ppListOut) */
}

%typemap(jni) (int* pnList, int** ppListOut) "jobjectArray"
%typemap(jtype) (int* pnList, int** ppListOut) "int[][]"
%typemap(jstype) (int* pnList, int** ppListOut) "int[][]"
%typemap(javain) (int* pnList, int** ppListOut) "$javainput"
%typemap(javaout) (int* pnList, int** ppListOut) {
    return $jnicall;
  }

/***************************************************
 *
 *  Java typemaps for (int id, int *nLen, const int **pList)
 *
 ***************************************************/ 
%typemap(in) (int id, int *nLen, const int **pList) (int nLen, int* pList)
{
  /* %typemap(in) (int id, int *nLen, const int **pList) */
  $1 = $input;
  $2 = &nLen;
  $3 = &pList;
}

%typemap(argout) (int id, int *nLen, const int **pList)
{
  /* %typemap(argout) (int id, int *nLen, const int **pList) */
  jintArray intArray = jenv->NewIntArray(nLen$argnum);
  jenv->SetIntArrayRegion(intArray, (jsize)0, (jsize)nLen$argnum, pList$argnum);
  $result = intArray;
}

%typemap(freearg) (int id, int *nLen, const int **pList)
{
  /* %typemap(freearg) (int id, int *nLen, const int **pList) */
}

%typemap(jni) (int id, int *nLen, const int **pList) "jint"
%typemap(jtype) (int id, int *nLen, const int **pList) "int"
%typemap(jstype) (int id, int *nLen, const int **pList) "int"
%typemap(javain) (int id, int *nLen, const int **pList) "$javainput"
%typemap(javaout) (int id, int *nLen, const int **pList) {
    return $jnicall;
  }

%typemap(out) (retIntArray)
{
}

%typemap(jni) (retIntArray) "jintArray"
%typemap(jtype) (retIntArray) "int[]"
%typemap(jstype) (retIntArray) "int[]"
%typemap(javain) (retIntArray) "$javainput"
%typemap(javaout) (retIntArray) {
    return $jnicall;
  }

/***************************************************
 *
 *  Java typemaps for (int id, int *nLen, const double **pList)
 *
 ***************************************************/ 
%typemap(in) (int id, int *nLen, const double **pList) (int nLen, double* pList)
{
  /* %typemap(in) (int id, int *nLen, const double **pList) */
  $1 = $input;
  $2 = &nLen;
  $3 = &pList;
}

%typemap(argout) (int id, int *nLen, const double **pList)
{
  /* %typemap(argout) (int id, int *nLen, const double **pList) */
  jdoubleArray doubleArray = jenv->NewDoubleArray(nLen$argnum);
  jenv->SetDoubleArrayRegion(doubleArray, (jsize)0, (jsize)nLen$argnum, pList$argnum);
  $result = doubleArray;
}

%typemap(freearg) (int id, int *nLen, const double **pList)
{
  /* %typemap(freearg) (int id, int *nLen, const double **pList) */
}

%typemap(jni) (int id, int *nLen, const double **pList) "jint"
%typemap(jtype) (int id, int *nLen, const double **pList) "int"
%typemap(jstype) (int id, int *nLen, const double **pList) "int"
%typemap(javain) (int id, int *nLen, const double **pList) "$javainput"
%typemap(javaout) (int id, int *nLen, const double **pList) {
    return $jnicall;
  }

%typemap(out) (retDoubleArray)
{
}

%typemap(jni) (retDoubleArray) "jdoubleArray"
%typemap(jtype) (retDoubleArray) "double[]"
%typemap(jstype) (retDoubleArray) "double[]"
%typemap(javain) (retDoubleArray) "$javainput"
%typemap(javaout) (retDoubleArray) {
    return $jnicall;
  }
/***************************************************
 *
 *  Java typemaps for (int nList, double* pList)
 *
 ***************************************************/ 
%typemap(in) (int nList, double* pList)
{
  /* %typemap(in) (int nList, double* pList) */
  /* check if is List */
  if ($input)
  {
    $1 = jenv->GetArrayLength($input);
    if ($1 == 0)
       $2 = NULL;
    else
       $2 = (double *)jenv->GetDoubleArrayElements($input, NULL);
  }
  else
  {
    $1 = 0;
    $2 = NULL;
  }
}

%typemap(argout) (int nList, double* pList)
{
  /* %typemap(argout) (int nList, double* pList) */
}

%typemap(freearg) (int nList, double* pList)
{
  /* %typemap(freearg) (int nList, double* pList) */
  if ($2) {
    jenv->ReleaseDoubleArrayElements($input, $2, JNI_ABORT);
  }
}

%typemap(jni) (int nList, double* pList) "jdoubleArray"
%typemap(jtype) (int nList, double* pList) "double[]"
%typemap(jstype) (int nList, double* pList) "double[]"
%typemap(javain) (int nLen, double *pList ) "$javainput"
%typemap(javaout) (int nLen, double *pList ) {
    return $jnicall;
  }

/***************************************************
 *
 *  Java typemaps for (int object_list_count, GDALRasterBandShadow **poObjects)
 *
 ***************************************************/ 
%typemap(in) (int object_list_count, GDALRasterBandShadow **poObjects)
{
  /* %typemap(in)(int object_list_count, GDALRasterBandShadow **poObjects) */
  /* check if is List */
  if ($input)
  {
    $1 = jenv->GetArrayLength($input);
    if ($1 == 0)
       $2 = NULL;
    else
    {
        $2 = (GDALRasterBandShadow**) malloc(sizeof(GDALRasterBandShadow*) * $1);
        int i;
        for (i = 0; i<$1; i++) {
            jobject obj = (jobject)jenv->GetObjectArrayElement($input, i);
            if (obj == NULL)
            {
                free ($2 );
                SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "null object in array");
                return $null;
            }
            const jclass klass = jenv->FindClass("org/gdal/gdal/Band");
            const jmethodID getCPtr = jenv->GetStaticMethodID(klass, "getCPtr", "(Lorg/gdal/gdal/Band;)J");
            $2[i] = (GDALRasterBandShadow*) jenv->CallStaticLongMethod(klass, getCPtr, obj);
        }
    }
  }
  else
  {
    $1 = 0;
    $2 = NULL;
  }
}

%typemap(argout) (int object_list_count, GDALRasterBandShadow **poObjects)
{
  /* %typemap(argout) (int object_list_count, GDALRasterBandShadow **poObjects) */
}

%typemap(freearg) (int object_list_count, GDALRasterBandShadow **poObjects)
{
  /* %typemap(freearg) (int object_list_count, GDALRasterBandShadow **poObjects) */
  if ($2) {
    free((void*) $2);
  }
}

%typemap(jni) (int object_list_count, GDALRasterBandShadow **poObjects) "jobjectArray"
%typemap(jtype) (int object_list_count, GDALRasterBandShadow **poObjects) "Band[]"
%typemap(jstype) (int object_list_count, GDALRasterBandShadow **poObjects) "Band[]"
%typemap(javain) (int object_list_count, GDALRasterBandShadow **poObjects) "$javainput"
%typemap(javaout) (int object_list_count, GDALRasterBandShadow **poObjects) {
    return $jnicall;
  }

/***************************************************
 * Typemaps converts the Hashtable to a char array *
 ***************************************************/

%typemap(in) char **dict
{
  /* %typemap(in) char **dict */
  /* Convert the Hashtable to a char array */
  $1 = NULL;
  if($input != 0) {
    const jclass hashtable = jenv->FindClass("java/util/Hashtable");
    const jclass enumeration = jenv->FindClass("java/util/Enumeration");
    const jmethodID get = jenv->GetMethodID(hashtable, "get",
      "(Ljava/lang/Object;)Ljava/lang/Object;");
    const jmethodID keys = jenv->GetMethodID(hashtable, "keys",
      "()Ljava/lang/Enumeration;");
    const jmethodID hasMoreElements = jenv->GetMethodID(enumeration, 
      "hasMoreElements", "()Z");
    const jmethodID getNextElement = jenv->GetMethodID(enumeration,
      "getNextElement", "()Ljava/lang/Object;");
    for (jobject keyset = jenv->CallObjectMethod($input, keys);
          jenv->CallBooleanMethod(keyset, hasMoreElements) == JNI_TRUE;) {
      jstring key = (jstring)jenv->CallObjectMethod(keyset, getNextElement);
      jstring value = (jstring)jenv->CallObjectMethod($input, get, key);
      const char *keyptr = jenv->GetStringUTFChars(key, 0);
      const char *valptr = jenv->GetStringUTFChars(value, 0);
      $1 = CSLAddNameValue($1, keyptr, valptr);
      jenv->ReleaseStringUTFChars(key, keyptr);
      jenv->ReleaseStringUTFChars(value, valptr);
    }
  }
}

%typemap(out) char **dict
{
  /* %typemap(out) char ** -> to hash */
  /* Convert a char array to a Hashtable */
  char **stringarray = $1;
  const jclass hashtable = jenv->FindClass("java/util/Hashtable");
  const jmethodID constructor = jenv->GetMethodID(hashtable, "<init>", "()V");
  const jmethodID put = jenv->GetMethodID(hashtable, "put",
    "(Ljava/lang/Object;Ljava/lang/Object;)Ljava/lang/Object;");
  $result = jenv->NewObject(hashtable, constructor);
  if ( stringarray != NULL ) {
    while (*stringarray != NULL ) {
      char const *valptr;
      char *keyptr;
      valptr = CPLParseNameValue( *stringarray, &keyptr );
      if ( valptr != 0 ) {
        jstring name = jenv->NewStringUTF(keyptr);
        jstring value = jenv->NewStringUTF(valptr);
        jenv->CallObjectMethod($result, put, name, value);
        jenv->DeleteLocalRef(name);
        jenv->DeleteLocalRef(value);
        CPLFree( keyptr );
      }
      stringarray++;
    }
  }
}

%typemap(freearg) char **dict
{
  /* %typemap(freearg) char **dict */
  CSLDestroy( $1 );
}

%typemap(jni) (char **dict) "jobject"
%typemap(jtype) (char **dict) "java.util.Hashtable"
%typemap(jstype) (char **dict) "java.util.Hashtable"
%typemap(javain) (char **dict) "$javainput"
%typemap(javaout) (char **dict) {
    return $jnicall;
  }



/***************************************************
 * Typemaps maps char** arguments from a Vector
 ***************************************************/

%typemap(in) char **options
{
  /* %typemap(in) char **options */
  $1 = NULL;
  if($input != 0) {
    const jclass vector = jenv->FindClass("java/util/Vector");
    const jclass enumeration = jenv->FindClass("java/util/Enumeration");
    const jmethodID elements = jenv->GetMethodID(vector, "elements",
      "()Ljava/util/Enumeration;");
    const jmethodID hasMoreElements = jenv->GetMethodID(enumeration, 
      "hasMoreElements", "()Z");
    const jmethodID getNextElement = jenv->GetMethodID(enumeration,
      "nextElement", "()Ljava/lang/Object;");
    if(vector == NULL || enumeration == NULL || elements == NULL ||
        hasMoreElements == NULL || getNextElement == NULL) {
          fprintf(stderr, "Could not load (options **) jni types.\n");
          return $null;
        }
    for (jobject keys = jenv->CallObjectMethod($input, elements);
          jenv->CallBooleanMethod(keys, hasMoreElements) == JNI_TRUE;) {
      jstring value = (jstring)jenv->CallObjectMethod(keys, getNextElement);
      const char *valptr = jenv->GetStringUTFChars(value, 0);
      $1 = CSLAddString($1,  valptr);
      jenv->ReleaseStringUTFChars(value, valptr);
    }
  }
}
%typemap(freearg) char **options
{
  /* %typemap(freearg) char **options */
  CSLDestroy( $1 );
}
%typemap(out) char **options
{
  /* %typemap(out) char ** -> Vector */
  char **stringarray = $1;
  const jclass vector = jenv->FindClass("java/util/Vector");
  const jmethodID constructor = jenv->GetMethodID(vector, "<init>", "()V");
  const jmethodID add = jenv->GetMethodID(vector, "add", "(Ljava/lang/Object;)Z");

  $result = jenv->NewObject(vector, constructor);
  if ( stringarray != NULL ) {
    while(*stringarray != NULL) {
      /*printf("working on string %s\n", *stringarray);*/
      jstring value = (jstring)jenv->NewStringUTF(*stringarray);
      jenv->CallBooleanMethod($result, add, value);
      jenv->DeleteLocalRef(value);
      stringarray++;
    }
  }
}

%typemap(jni) (char **options) "jobject"
%typemap(jtype) (char **options) "java.util.Vector"
%typemap(jstype) (char **options) "java.util.Vector"
%typemap(javain) (char **options) "$javainput"
%typemap(javaout) (char **options) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for retAsStringArrayNoFree
 ***************************************************/

%typemap(out) char **retAsStringArrayNoFree
{
  /* %typemap(out) char **retAsStringArrayNoFree -> String[] */
  char **stringarray = result;
  int i;
  int len=CSLCount(result);
  jstring temp_string;
  const jclass clazz = jenv->FindClass("java/lang/String");

  $result = jenv->NewObjectArray(len, clazz, NULL);
  /* exception checking omitted */

  for (i=0; i<len; i++) {
      temp_string = jenv->NewStringUTF(*stringarray++);
      jenv->SetObjectArrayElement(jresult, i, temp_string);
      jenv->DeleteLocalRef(temp_string);
  }
}

%typemap(jni) (char **retAsStringArrayNoFree) "jobjectArray"
%typemap(jtype) (char **retAsStringArrayNoFree) "String[]"
%typemap(jstype) (char **retAsStringArrayNoFree) "String[]"
%typemap(javain) (char **retAsStringArrayNoFree) "$javainput"
%typemap(javaout) (char **retAsStringArrayNoFree) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for retAsStringArrayNoFree
 ***************************************************/

%typemap(out) char **retAsStringArrayAndFree
{
  /* %typemap(out) char **retAsStringArrayAndFree -> String[] */
  char **stringarray = result;
  int i;
  int len=CSLCount(result);
  jstring temp_string;
  const jclass clazz = jenv->FindClass("java/lang/String");

  $result = jenv->NewObjectArray(len, clazz, NULL);
  /* exception checking omitted */

  for (i=0; i<len; i++) {
      temp_string = jenv->NewStringUTF(*stringarray++);
      jenv->SetObjectArrayElement(jresult, i, temp_string);
      jenv->DeleteLocalRef(temp_string);
  }

  CSLDestroy(result);
}

%typemap(jni) (char **retAsStringArrayAndFree) "jobjectArray"
%typemap(jtype) (char **retAsStringArrayAndFree) "String[]"
%typemap(jstype) (char **retAsStringArrayAndFree) "String[]"
%typemap(javain) (char **retAsStringArrayAndFree) "$javainput"
%typemap(javaout) (char **retAsStringArrayAndFree) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for char **OUTPUT 
 ***************************************************/

%typemap(in) char **OUTPUT (char* ret)
{
    /* %typemap(in) char **OUTPUT (char* ret) */
    if (!$input) {
      SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "array null");
      return $null;
    }
    if (jenv->GetArrayLength($input) == 0) {
      SWIG_JavaThrowException(jenv, SWIG_JavaIndexOutOfBoundsException, "Array must contain at least 1 element");
      return $null;
    }
    $1 = &ret;
}

%typemap(argout) char **OUTPUT
{
  /* %typemap(argout) char **OUTPUT */
  jstring temp_string = jenv->NewStringUTF(ret$argnum);
  jenv->SetObjectArrayElement($input, 0, temp_string);
  jenv->DeleteLocalRef(temp_string);
}

%typemap(jni) (char **OUTPUT) "jobjectArray"
%typemap(jtype) (char **OUTPUT) "String[]"
%typemap(jstype) (char **OUTPUT) "String[]"
%typemap(javain) (char **OUTPUT) "$javainput"
%typemap(javaout) (char **OUTPUT) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for char **out_ppsz_and_free
 ***************************************************/

/* Almost same as %typemap(out) char **options */
/* but we CSLDestroy the char** pointer at the end */
%typemap(out) char **out_ppsz_and_free
{
  /* %typemap(out) char **out_ppsz_and_free -> vector of strings */
  char **stringarray = $1;
  const jclass vector = jenv->FindClass("java/util/Vector");
  const jmethodID constructor = jenv->GetMethodID(vector, "<init>", "()V");
  const jmethodID add = jenv->GetMethodID(vector, "add", "(Ljava/lang/Object;)Z");

  $result = jenv->NewObject(vector, constructor);
  if ( stringarray != NULL ) {
    while(*stringarray != NULL) {
      /*printf("working on string %s\n", *stringarray);*/
      jstring value = (jstring)jenv->NewStringUTF(*stringarray);
      jenv->CallBooleanMethod($result, add, value);
      jenv->DeleteLocalRef(value);
      stringarray++;
    }
  }
  CSLDestroy($1);
}

%typemap(jni) (char **out_ppsz_and_free) "jobject"
%typemap(jtype) (char **out_ppsz_and_free) "java.util.Vector"
%typemap(jstype) (char **out_ppsz_and_free) "java.util.Vector"
%typemap(javain) (char **out_ppsz_and_free) "$javainput"
%typemap(javaout) (char **out_ppsz_and_free) {
    return $jnicall;
  }

/***************************************************
 ***************************************************/
%define OPTIONAL_POD(type,argstring)
%typemap(in) (type *optional_##type)
{
  /* %typemap(in) (type *optional_##type) */
  $1 = ($1_type)$input;
}
%typemap(argout) (type *optional_##type)
{
  /* %typemap(in) (type *optional_##type) */
}
%typemap(typecheck,precedence=0) (type *optional_##type)
{
  /* %typemap(typecheck,precedence=0) (type *optionalInt) */
}

%typemap(jni) (type *optional_##type) "jintArray"
%typemap(jtype) (type *optional_##type) "int[]"
%typemap(jstype) (type *optional_##type) "int[]"
%typemap(javain) (type *optional_##type) "$javainput"
%typemap(javaout) (type *optional_##type) {
    return $jnicall;
  }
%enddef

OPTIONAL_POD(int,i);


/***************************************************
 * Typemaps for char **argout. 
 ***************************************************/

%typemap(in) (char **argout) ( char *argout=0 )
{
  /* %typemap(in) (char **argout) */
  $1 = &argout;
}

%typemap(argout) (char **argout)
{
  /* %typemap(argout) (char **argout) */
  jstring temp_string;

  if($input != NULL && (int)jenv->GetArrayLength($input) >= 1) {
    temp_string = jenv->NewStringUTF(argout$argnum);
    jenv->SetObjectArrayElement($input, 0, temp_string);
    jenv->DeleteLocalRef(temp_string);
  }
}

%typemap(freearg) (char **argout)
{
  /* %typemap(freearg) (char **argout) */
  if($1) {
    CPLFree((void *)argout$argnum);
  }
}

%typemap(jni) (char **argout) "jobjectArray"
%typemap(jtype) (char **argout) "String[]"
%typemap(jstype) (char **argout) "String[]"
%typemap(javain) (char **argout) "$javainput"
%typemap(javaout) (char **argout) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for double *argout[ANY]
 ***************************************************/

%typemap(in) (double *argout[ANY]) (double *argout[$dim0])
{
  /* %typemap(in) (double *argout[ANY]) */
  if($input == NULL || jenv->GetArrayLength($input) != $dim0) {
      char errorMsg[512];
      sprintf(errorMsg, "array of size %d expected", $dim0);
      SWIG_JavaThrowException(jenv, SWIG_JavaIndexOutOfBoundsException, errorMsg);
      return $null;
  }
  $1 = argout;
}

%typemap(argout) (double *argout[ANY])
{
  /* %typemap(argout) (double *argout[ANY]) */
  jenv->SetDoubleArrayRegion($input, (jsize)0, (jsize)$dim0, $1[0]);
}

%typemap(freearg) (double *argout[ANY])
{
  /* %typemap(freearg) (double *argout[ANY]) */
  CPLFree($1);
}

%typemap(jni) (double *argout[ANY]) "jdoubleArray"
%typemap(jtype) (double *argout[ANY]) "double[]"
%typemap(jstype) (double *argout[ANY]) "double[]"
%typemap(javain) (double *argout[ANY]) "$javainput"
%typemap(javaout) (double *argout[ANY]) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for double argin[ANY]
 ***************************************************/

%typemap(in) (double argin[ANY]) (jboolean isCopy)
{
  /* %typemap(in) (double argin[ANY]) */
  if($input == NULL || jenv->GetArrayLength($input) != $dim0) {
      char errorMsg[512];
      sprintf(errorMsg, "array of size %d expected", $dim0);
      SWIG_JavaThrowException(jenv, SWIG_JavaIndexOutOfBoundsException, errorMsg);
      return $null;
  }
  $1 = (double *)jenv->GetDoubleArrayElements($input, &isCopy);
}

%typemap(argout) (double argin[ANY])
{
  /* %typemap(argout) (double argin[ANY]) */
}

%typemap(freearg) (double argin[ANY])
{
  /* %typemap(in) (double argin[ANY]) */
  if(isCopy$argnum == JNI_TRUE) {
    jenv->ReleaseDoubleArrayElements($input, (jdouble *)$1, JNI_ABORT);
  }
}

%typemap(jni) (double argin[ANY]) "jdoubleArray"
%typemap(jtype) (double argin[ANY]) "double[]"
%typemap(jstype) (double argin[ANY]) "double[]"
%typemap(javain) (double argin[ANY]) "$javainput"
%typemap(javaout) (double argin[ANY]) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for double argout[ANY]
 ***************************************************/

%typemap(in) (double argout[ANY])
{
  /* %typemap(in) (double argout[ANY]) */
  if($input == NULL || jenv->GetArrayLength($input) != $dim0) {
      char errorMsg[512];
      sprintf(errorMsg, "array of size %d expected", $dim0);
      SWIG_JavaThrowException(jenv, SWIG_JavaIndexOutOfBoundsException, errorMsg);
      return $null;
  }
  $1 = (double *)jenv->GetDoubleArrayElements($input, NULL);
}

%typemap(argout) (double argout[ANY])
{
  /* %typemap(argout) (double argout[ANY]) */
}

%typemap(freearg) (double argout[ANY])
{
  /* %typemap(in) (double argout[ANY]) */
  jenv->ReleaseDoubleArrayElements($input, (jdouble *)$1, 0);
}

%typemap(jni) (double argout[ANY]) "jdoubleArray"
%typemap(jtype) (double argout[ANY]) "double[]"
%typemap(jstype) (double argout[ANY]) "double[]"
%typemap(javain) (double argout[ANY]) "$javainput"
%typemap(javaout) (double argout[ANY]) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for char ** 
 ***************************************************/

/* This tells SWIG to treat char ** as a special case when used as a parameter
   in a function call */
%typemap(in) char ** (jint size) {
  /* %typemap(in) char ** (jint size) */
    int i = 0;
    size = jenv->GetArrayLength($input);
    $1 = (char **) malloc((size+1)*sizeof(char *));
    /* make a copy of each string */
    for (i = 0; i<size; i++) {
        jstring j_string = (jstring)jenv->GetObjectArrayElement($input, i);
        const char * c_string = jenv->GetStringUTFChars(j_string, 0);
        $1[i] = (char *)malloc(strlen((c_string)+1)*sizeof(const char *));
        strcpy($1[i], c_string);
        jenv->ReleaseStringUTFChars(j_string, c_string);
        jenv->DeleteLocalRef(j_string);
    }
    $1[i] = 0;
}

/* This cleans up the memory we malloc'd before the function call */
%typemap(freearg) char ** {
  /* %typemap(freearg) char ** */
    int i;
    for (i=0; i<size$argnum-1; i++)
      free($1[i]);
    free($1);
}

/* This allows a C function to return a char ** as a Java String array */
%typemap(out) char ** {
  /* %typemap(out) char ** */
    int i;
    int len=0;
    jstring temp_string;
    const jclass clazz = jenv->FindClass("java/lang/String");

    while ($1[len]) len++;    
    jresult = jenv->NewObjectArray(len, clazz, NULL);
    /* exception checking omitted */

    for (i=0; i<len; i++) {
      temp_string = jenv->NewStringUTF(*result++);
      jenv->SetObjectArrayElement(jresult, i, temp_string);
      jenv->DeleteLocalRef(temp_string);
    }
}

%typemap(jni) char ** "jobjectArray"
%typemap(jtype) char ** "String[]"
%typemap(jstype) char ** "String[]"
%typemap(javain) char ** "$javainput"
%typemap(javaout) char ** {
    return $jnicall;
  }


/***************************************************
 * Typemaps for (void * nioBuffer, long nioBufferSize)
 ***************************************************/

%typemap(in, numinputs=1) (void * nioBuffer, long nioBufferSize) 
{
    /* %typemap(in, numinputs=1) (void * nioBuffer, long nioBufferSize)  */
    if ($input == 0)
    {
        SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "null array");
        return $null;
    }
    $1 = jenv->GetDirectBufferAddress($input);
    $2 = jenv->GetDirectBufferCapacity($input);
}


/* These 3 typemaps tell SWIG what JNI and Java types to use */
%typemap(jni) (void * nioBuffer, long nioBufferSize)  "jobject"
%typemap(jtype) (void * nioBuffer, long nioBufferSize)  "java.nio.ByteBuffer"
%typemap(jstype) (void * nioBuffer, long nioBufferSize)  "java.nio.ByteBuffer"
%typemap(javain) (void * nioBuffer, long nioBufferSize)  "$javainput"
%typemap(javaout) (void * nioBuffer, long nioBufferSize) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for GIntBig
 ***************************************************/


%typemap(out) (GIntBig)
{
    /* %typemap(out) (GIntBig) */
    $result = result;
}
%typemap(jni) (GIntBig) "jlong"
%typemap(jtype) (GIntBig) "long"
%typemap(jstype) (GIntBig) "long"
%typemap(javain) (GIntBig) "$javainput"
%typemap(javaout) (GIntBig) {
    return $jnicall;
  }

/***************************************************
 * Typemaps for ( int nCount, double *x, double *y, double *z ) 
 ***************************************************/

%typemap(in) ( int nCount, double *x, double *y, double *z ) (int xyzLen)
{
    /* %typemap(in) ( int nCount, double *x, double *y, double *z ) (int xyzLen) */
    $1 = ($input) ? jenv->GetArrayLength($input) : 0;
    xyzLen = $1;
    $2 = (double*)CPLMalloc($1 * sizeof(double));
    $3 = (double*)CPLMalloc($1 * sizeof(double));
    $4 = (double*)CPLMalloc($1 * sizeof(double));
    int i;
    for (i = 0; i<$1; i++) {
        jdoubleArray doubleArray = (jdoubleArray)jenv->GetObjectArrayElement($input, i);
        if (doubleArray == NULL)
        {
            CPLFree ($2);
            CPLFree ($3);
            CPLFree ($4);
            SWIG_JavaThrowException(jenv, SWIG_JavaNullPointerException, "null object in array");
            return $null;
        }
        int nDim = jenv->GetArrayLength(doubleArray);
        if (nDim != 2 && nDim != 3)
        {
            CPLFree ($2);
            CPLFree ($3);
            CPLFree ($4);
            SWIG_JavaThrowException(jenv, SWIG_JavaIllegalArgumentException, "wrong array dimensions");
            return $null;
        }
        double* pElements = jenv->GetDoubleArrayElements(doubleArray, NULL);
        $2[i] = pElements[0];
        $3[i] = pElements[1];
        if (nDim == 3)
            $4[i] = pElements[2];
        else
            $4[i] = 0;
        jenv->ReleaseDoubleArrayElements(doubleArray, pElements, JNI_ABORT);
    }
}

%typemap(argout) ( int nCount, double *x, double *y, double *z )
{
    /* %typemap(argout) ( int nCount, double *x, double *y, double *z ) */
    int i;
    for (i = 0; i<$1; i++) {
        jdoubleArray doubleArray = (jdoubleArray)jenv->GetObjectArrayElement($input, i);
        int nDim = jenv->GetArrayLength(doubleArray);
        jenv->SetDoubleArrayRegion(doubleArray, (jsize)0, (jsize)1, &$2[i]);
        jenv->SetDoubleArrayRegion(doubleArray, (jsize)1, (jsize)1, &$3[i]);
        if (nDim == 3)
                jenv->SetDoubleArrayRegion(doubleArray, (jsize)2, (jsize)1, &$4[i]);
    }
    CPLFree($2);
    CPLFree($3);
    CPLFree($4);
}

%typemap(jni) ( int nCount, double *x, double *y, double *z ) "jobjectArray"
%typemap(jtype) ( int nCount, double *x, double *y, double *z ) "double[][]"
%typemap(jstype) ( int nCount, double *x, double *y, double *z ) "double[][]"
%typemap(javain) ( int nCount, double *x, double *y, double *z ) "$javainput"
%typemap(javaout) ( int nCount, double *x, double *y, double *z ) {
    return $jnicall;
  }


/***************************************************
 * Typemaps for ( int *panSuccess )
 ***************************************************/

%typemap(in) ( int *panSuccess )
{
    /* %typemap(in) ( int *panSuccess ) */
    if ($input == NULL)
    {
        $1 = (int*)CPLCalloc(xyzLen3, sizeof(int));
    }
    else
    {
        int len = jenv->GetArrayLength($input);
        /* HACK */
        if (len != xyzLen3)
        {
            SWIG_JavaThrowException(jenv, SWIG_JavaIllegalArgumentException, "wrong array dimensions");
            return $null;
        }
        $1 = (int*)CPLCalloc(len, sizeof(int));
    }
}

%typemap(argout) ( int *panSuccess )
{
    /* %typemap(argout) ( int *panSuccess ) */
    if ($input)
        jenv->SetIntArrayRegion($input, (jsize)0, (jsize)xyzLen3, $1);
    CPLFree($1);
}

%typemap(jni) ( int *panSuccess ) "jintArray"
%typemap(jtype) ( int *panSuccess ) "int[]"
%typemap(jstype) ( int *panSuccess ) "int[]"
%typemap(javain) ( int *panSuccess ) "$javainput"
%typemap(javaout) ( int *panSuccess ) {
    return $jnicall;
  }
