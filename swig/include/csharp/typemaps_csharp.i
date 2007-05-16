/******************************************************************************
 * $Id$
 *
 * Name:     typemaps_csharp.i
 * Project:  GDAL SWIG Interface
 * Purpose:  Typemaps for C# bindings
 * Author:   Howard Butler, Tamas Szekeres
 *
*/

%include "typemaps.i"

/* CSHARP TYPEMAPS */

%fragment("OGRErrMessages","header") %{
static char const *
OGRErrMessages( int rc ) {
  switch( rc ) {
  case 0:
    return "OGR Error %d: None";
  case 1:
    return "OGR Error %d: Not enough data";
  case 2:
    return "OGR Error %d: Unsupported geometry type";
  case 3:
    return "OGR Error %d: Unsupported operation";
  case 4:
    return "OGR Error %d: Corrupt data";
  case 5:
    return "OGR Error %d: General Error";
  case 6:
    return "OGR Error %d: Unsupported SRS";
  default:
    return "OGR Error %d: Unknown";
  }
}
%}

%typemap(out,fragment="OGRErrMessages",canthrow=1) OGRErr
{
  /* %typemap(out,fragment="OGRErrMessages",canthrow=1) OGRErr */
  $result = result;
}
%typemap(ret) OGRErr
{
  /* %typemap(ret) OGRErr */

}

%typemap(in) (tostring argin) (string str)
{
  /* %typemap(in) (tostring argin) */
  $1 = ($1_ltype)$input;
}

%typemap(in) (char **ignorechange) ( char *val )
{
  /* %typemap(in) (char **ignorechange) */
	/*TODO*/
	$1 = $null;
}


/* GDAL Typemaps */

%typemap(out) IF_FALSE_RETURN_NONE
{
  /* %typemap(out) IF_FALSE_RETURN_NONE */

}
%typemap(ret) IF_FALSE_RETURN_NONE
{
 /* %typemap(ret) IF_FALSE_RETURN_NONE */

}

%typemap(out) IF_ERROR_RETURN_NONE
{
  /* %typemap(out) IF_ERROR_RETURN_NONE */
}

/*
 * Typemap char ** -> dict
 */
%typemap(out) char **dict
{
  /* %typemap(out) char ** -> to hash */
  /*TODO*/
	$result = $null;
}

/*
 * Typemap char **<- dict
 */
%typemap(in) char **dict
{
  /* %typemap(in) char **dict */

}
%typemap(freearg) char **dict
{
  /* %typemap(freearg) char **dict */
  CSLDestroy( $1 );
}

%define OPTIONAL_POD(type,argstring)
%typemap(in) (type *optional_##type) ( type val )
{
  /* %typemap(in) (type *optional_##type) */
  $1 = ($1_type)$input;
}
%typemap(typecheck,precedence=0) (type *optional_##type)
{
  /* %typemap(typecheck,precedence=0) (type *optionalInt) */

}
%enddef

OPTIONAL_POD(int,i);

/*
 * Typemap for GIntBig (int64)
 */

%typemap(ctype, out="GIntBig") GIntBig  %{GIntBig%}
%typemap(imtype, out="long") GIntBig "long"
%typemap(cstype) GIntBig %{long%}
%typemap(out) GIntBig %{ $result = $1; %}
%typemap(csout, excode=SWIGEXCODE) GIntBig {
    long res = $imcall;$excode
    return res;
}


/******************************************************************************
 * Marshaler for NULL terminated string arrays                                *
 *****************************************************************************/

%pragma(csharp) imclasscode=%{
  public class StringListMarshal : IDisposable {
    public readonly IntPtr[] _ar;
    public StringListMarshal(string[] ar) {
      _ar = new IntPtr[ar.Length+1];
      for (int cx = 0; cx < ar.Length; cx++) {
	      _ar[cx] = System.Runtime.InteropServices.Marshal.StringToHGlobalAnsi(ar[cx]);
      }
      _ar[ar.Length] = IntPtr.Zero;
    }
    public virtual void Dispose() {
	  for (int cx = 0; cx < _ar.Length-1; cx++) {
          System.Runtime.InteropServices.Marshal.FreeHGlobal(_ar[cx]);
      }
      GC.SuppressFinalize(this);
    }
  }
%}

/*
 * Typemap for char** options
 */

%typemap(imtype, out="IntPtr") char **options "IntPtr[]"
%typemap(cstype) char **options %{string[]%}
%typemap(in) char **options %{ $1 = ($1_ltype)$input; %}
%typemap(out) char **options %{ $result = $1; %}
%typemap(csin) char **options "new $modulePINVOKE.StringListMarshal($csinput)._ar"
%typemap(csout, excode=SWIGEXCODE) char**options {
        /* %typemap(csout) char**options */
        IntPtr cPtr = $imcall;
        IntPtr objPtr;
        int count = 0;
        if (cPtr != IntPtr.Zero) {
            while (Marshal.ReadIntPtr(cPtr, count*IntPtr.Size) != IntPtr.Zero)
                ++count;
        }
        string[] ret = new string[count];
        if (count > 0) {       
	        for(int cx = 0; cx < count; cx++) {
                objPtr = System.Runtime.InteropServices.Marshal.ReadIntPtr(cPtr, cx * System.Runtime.InteropServices.Marshal.SizeOf(typeof(IntPtr)));
                ret[cx]= (objPtr == IntPtr.Zero) ? null : System.Runtime.InteropServices.Marshal.PtrToStringAnsi(objPtr);
            }
        }
        $excode
        return ret;
}
 
%typemap(freearg) char **options
{
  /* %typemap(freearg) char **options */
  //CSLDestroy( $1 );
}

/*
 * Typemap for char **argout. 
 */
%typemap(imtype) (char **argout), (char **username), (char **usrname), (char **type) "out string"
%typemap(cstype) (char **argout), (char **username), (char **usrname), (char **type) "out string"
%typemap(csin) (char** argout), (char **username), (char **usrname), (char **type) "out $csinput"
  
%typemap(in) (char **argout), (char **username), (char **usrname), (char **type)
{
  /* %typemap(in) (char **argout) */
	$1 = ($1_ltype)$input;
}
%typemap(argout) (char **argout), (char **username), (char **usrname), (char **type)
{
  /* %typemap(argout) (char **argout) */
  char* temp_string;
  temp_string = SWIG_csharp_string_callback(*$1);
  if (*$1)
		free(*$1);
  *$1 = temp_string;
}
%typemap(freearg) (char **argout), (char **username), (char **usrname), (char **type)
{
  /* %typemap(freearg) (char **argout) */
}

/*
 * Typemap for double argout[ANY]. 
 */
%typemap(imtype) (double argout[ANY]) "double[]"
%typemap(cstype) (double argout[ANY]) "double[]"
%typemap(csin) (double argout[ANY]) "$csinput"

%typemap(in) (double argout[ANY])
{
  /* %typemap(in) (double argout[ANY]) */
  $1 = ($1_ltype)$input;
}

%typemap(in,numinputs=0) ( double *argout[ANY]) (double *argout[$dim0])
{
  /* %typemap(in,numinputs=0) (double *argout[ANY]) */
  $1 = (double**)&argout;
}
%typemap(argout) ( double *argout[ANY])
{
  /* %typemap(argout) (double *argout[ANY]) */

}
%typemap(freearg) (double *argout[ANY])
{
  /* %typemap(freearg) (double *argout[ANY]) */

}

%apply double argout[ANY] {double *inout}

/*
 * Typemap for double argin[ANY]. 
 */

%typemap(imtype) (double argin[ANY])  "double[]"
%typemap(cstype) (double argin[ANY]) "double[]"
%typemap(csin) (double argin[ANY])  "$csinput"

%typemap(in) (double argin[ANY])
{
  /* %typemap(in) (double argin[ANY]) */
  $1 = ($1_ltype)$input;
}

/*
 * Typemap for double inout[ANY]. 
 */

%typemap(imtype) (double inout[ANY])  "double[]"
%typemap(cstype) (double inout[ANY]) "double[]"
%typemap(csin) (double inout[ANY])  "$csinput"

%typemap(in) (double inout[ANY])
{
  /* %typemap(in) (double inout[ANY]) */
  $1 = ($1_ltype)$input;
}

%typemap(argout) (double inout[ANY])
{
  /* %typemap(argout) (double inout[ANY]) */
}

/*
 * Typemap for double *defaultval. 
 */

%typemap(imtype) (double *defaultval)  "ref double"
%typemap(cstype) (double *defaultval) "ref double"
%typemap(csin) (double *defaultval)  "ref $csinput"

%typemap(in) (double *defaultval)
{
  /* %typemap(in) (double inout[ANY]) */
  $1 = ($1_ltype)$input;
}

/*
 * Typemap for out double.
 */

%typemap(imtype) (double *val), (double *min), (double *max), (double *mean), (double *stddev) "out double"
%typemap(cstype) (double *val), (double *min), (double *max), (double *mean), (double *stddev) "out double"
%typemap(csin) (double *val), (double *min), (double *max), (double *mean), (double *stddev) "out $csinput"

%typemap(in) (double *val), (double *min), (double *max), (double *mean), (double *stddev)
{
  /* %typemap(in) (double *val) */
  $1 = ($1_ltype)$input;
}

/*
 * Typemap for 'out int'.
 */

%typemap(imtype) (int *hasval)  "out int"
%typemap(cstype) (int *hasval) "out int"
%typemap(csin) (int *hasval)  "out $csinput"

%typemap(in) (int *hasval)
{
  /* %typemap(in) (int *hasval) */
  $1 = ($1_ltype)$input;
}

/******************************************************************************
 * GDAL raster R/W support                                                    *
 *****************************************************************************/
 
%typemap(imtype, out="IntPtr") void *buffer_ptr "IntPtr"
%typemap(cstype) void *buffer_ptr %{IntPtr%}
%typemap(in) void *buffer_ptr %{ $1 = ($1_ltype)$input; %}
%typemap(out) void *buffer_ptr %{ $result = $1; %}
%typemap(csin) void *buffer_ptr "$csinput"
%typemap(csout, excode=SWIGEXCODE) void *buffer_ptr {
      IntPtr ret = $imcall;$excode
      return ret;
}

