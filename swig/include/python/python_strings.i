/******************************************************************************
 * $Id$
 *
 * Name:     python_strings.i
 * Project:  GDAL Python Interface
 * Author:   Even Rouault, <even dot rouault at mines dash paris dot org>
 *
 ******************************************************************************
 * Copyright (c) 2009, Even Rouault
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
 *****************************************************************************/
 
%{

/* Return a PyObject* from a NULL terminated C String */
static PyObject* GDALPythonObjectFromCStr(const char *pszStr)
{
#if PY_VERSION_HEX >= 0x03000000
  const unsigned char* pszIter = (const unsigned char*) pszStr;
  while(*pszIter != 0)
  {
    if (*pszIter > 127)
        return PyBytes_FromString( pszStr );
    pszIter ++;
  }
  return PyUnicode_FromString(pszStr); 
#else
  return PyString_FromString(pszStr);
#endif
}

/* Return a NULL terminated c String from a PyObject */
/* Result must be freed with GDALPythonFreeCStr */
static char* GDALPythonObjectToCStr(PyObject* pyObject)
{
#if PY_VERSION_HEX >= 0x03000000
  if (PyUnicode_Check(pyObject))
  {
      char *pszStr;
      char *pszNewStr;
      int nLen;
      PyObject* pyUTF8Str = PyUnicode_AsUTF8String(pyObject);
      PyBytes_AsStringAndSize(pyUTF8Str, &pszStr, &nLen);
      pszNewStr = (char *) malloc(nLen+1);
      memcpy(pszNewStr, pszStr, nLen+1);
      Py_XDECREF(pyUTF8Str);
      return pszNewStr;
  }
  else if (PyBytes_Check(pyObject))
  {
      char *pszStr;
      char *pszNewStr;
      int nLen;
      PyBytes_AsStringAndSize(pyObject, &pszStr, &nLen);
      pszNewStr = (char *) malloc(nLen+1);
      memcpy(pszNewStr, pszStr, nLen+1);
      return pszNewStr;
  }
  else
  {
      char *pszStr = (char *) malloc(1);
      pszStr[0] = '\0';
      return pszStr;
  }
#else
  return PyString_AsString(pyObject);
#endif
}

#if PY_VERSION_HEX >= 0x03000000
#define GDALPythonFreeCStr(x) free( (void*) (x) )
#else
#define GDALPythonFreeCStr(x) 
#endif

%}
