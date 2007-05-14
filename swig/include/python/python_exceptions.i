/*
 * This was the cpl_exceptions.i code.  But since python is the only one
 * different (should support old method as well as new one)
 * it was moved into this file.
 */
%{
int bUseExceptions=0;

void VeryQuiteErrorHandler(CPLErr eclass, int code, const char *msg ) {
  /* If the error class is CE_Fatal, we want to have a message issued
     because the CPL support code does an abort() before any exception
     can be generated */
  if (eclass == CE_Fatal ) {
    CPLDefaultErrorHandler(eclass, code, msg );
  }
}
%}

%inline %{
void UseExceptions() {
  bUseExceptions = 1;
  CPLSetErrorHandler( (CPLErrorHandler) VeryQuiteErrorHandler );
}

void DontUseExceptions() {
  bUseExceptions = 0;
  CPLSetErrorHandler( CPLDefaultErrorHandler );
}
%}

%include exception.i

%exception {

    $action
    if ( bUseExceptions ) {
      CPLErr eclass = CPLGetLastErrorType();
      if ( eclass == CE_Failure || eclass == CE_Fatal ) {
        SWIG_exception( SWIG_RuntimeError, CPLGetLastErrorMsg() );
      }
    }
}
