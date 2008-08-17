/******************************************************************************
 * $Id: $
 *
 * Name:     oci_wrapper.cpp.cpp
 * Project:  Oracle Spatial GeoRaster Driver
 * Purpose:  Limited wrapper for OCI (Oracle Call Interfaces)
 * Author:   Ivan Lucena [ivan.lucena@pmldnet.com]
 *
 ******************************************************************************
 * Copyright (c) 2008, Ivan Lucena
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

#include "oci_wrapper.h"

static const OW_CellDepth ahOW_CellDepth[] = {
    {"8BIT_U",          GDT_Byte},
    {"16BIT_U",         GDT_UInt16},
    {"16BIT_S",         GDT_Int16},
    {"32BIT_U",         GDT_UInt32},
    {"32BIT_S",         GDT_Int32},
    {"32BIT_REAL",      GDT_Float32},
    {"64BIT_REAL",      GDT_Float64},
    {"32BIT_COMPLEX",   GDT_CFloat32},
    {"64BIT_COMPLEX",   GDT_CFloat64},
    {"1BIT",            GDT_Byte},
    {"2BIT",            GDT_Byte},
    {"4BIT",            GDT_Byte}
};

/*****************************************************************************/
/*                            OWConnection                                   */
/*****************************************************************************/

OWConnection::OWConnection( const char* pszUserIn,
                            const char* pszPasswordIn,
                            const char* pszServerIn )
{
    pszUser         = CPLStrdup( pszUserIn );
    pszPassword     = CPLStrdup( pszPasswordIn );
    pszServer       = CPLStrdup( pszServerIn );
    hEnv            = NULL;
    hError          = NULL;
    hSvcCtx         = NULL;
    hDescribe       = NULL;
    hNumArrayTDO    = NULL;
    hGeometryTDO    = NULL;
    hGeoRasterTDO   = NULL;
    hColormapTDO    = NULL;
    bSuceeded       = false;

    // ------------------------------------------------------
    //  Create Environment
    // ------------------------------------------------------

    ub4 nMode = ( OCI_DEFAULT | OCI_OBJECT );

    CheckError( OCIEnvCreate( 
        &hEnv, 
        nMode, 
        NULL, 
        NULL, 
        NULL, 
        NULL, 
        (size_t) 0, 
        (dvoid**) NULL ), NULL );

    // ------------------------------------------------------
    //  Create Error Handle
    // ------------------------------------------------------

    CheckError( OCIHandleAlloc( 
        (dvoid*) hEnv, 
        (dvoid**) (dvoid*) &hError, 
        (ub4) OCI_HTYPE_ERROR, 
        (size_t) 0, 
        (void**) NULL ), hError );

    // ------------------------------------------------------
    //  Logon to Oracle Server
    // ------------------------------------------------------

    if( CheckError( OCILogon( 
        hEnv, 
        hError, 
        &hSvcCtx, 
        (text*) pszUser, 
        (ub4) strlen(pszUser),
        (text*) pszPassword, 
        (ub4) strlen(pszPassword), 
        (text*) pszServer, 
        (ub4) strlen(pszServer) ), hError ) )
    {
        bSuceeded = false;
        return;
    }
    else
    {
        bSuceeded = true;
    }

    // ------------------------------------------------------
    //  Get Server Version
    // ------------------------------------------------------

    char szVersionTxt[OWTEXT];

    OCIServerVersion ( 
        hSvcCtx,
        hError,
        (text*) szVersionTxt,
        (ub4) OWTEXT,
        (ub1) OCI_HTYPE_SVCCTX );

    nVersion = OWParseServerVersion( szVersionTxt );

    // ------------------------------------------------------
    //  Initialize/Describe types
    // ------------------------------------------------------

    CheckError( OCIHandleAlloc( 
        (dvoid*) hEnv, 
        (dvoid**) (dvoid*) &hDescribe, 
        (ub4) OCI_HTYPE_DESCRIBE, 
        (size_t) 0, 
        (dvoid**) NULL ), hError );

    hNumArrayTDO    = DescribeType( SDO_NUMBER_ARRAY );
    hGeometryTDO    = DescribeType( SDO_GEOMETRY );
    hColormapTDO    = DescribeType( SDO_COLORMAP );
    hGeoRasterTDO   = DescribeType( SDO_GEORASTER );
}

OWConnection::~OWConnection()
{
    if( hDescribe )
    {
        OCIHandleFree( (dvoid*) hDescribe, (ub4) OCI_HTYPE_DESCRIBE);
    }

    if( hSvcCtx )
    {
        OCILogoff( hSvcCtx, hError );

        OCIHandleFree( (dvoid*) hSvcCtx, (ub4) OCI_HTYPE_SVCCTX);
        OCIHandleFree( (dvoid*) hError, (ub4) OCI_HTYPE_ERROR);
        OCIHandleFree( (dvoid*) hEnv, (ub4) OCI_HTYPE_ENV);
    }

    CPLFree_nt( pszUser );
    CPLFree_nt( pszPassword );
    CPLFree_nt( pszServer );
}

OCIType* OWConnection::DescribeType( char *pszTypeName )
{
    OCIParam* hParam    = NULL;
    OCIRef*   hRef      = NULL;
    OCIType*  hType     = NULL;

    CheckError( OCIDescribeAny( 
        hSvcCtx, 
        hError, 
        (text*) pszTypeName,
        (ub4) strlen( pszTypeName ),
        (ub1) OCI_OTYPE_NAME,
        (ub1) OCI_DEFAULT, 
        (ub1) OCI_PTYPE_TYPE, 
        hDescribe ), hError );

    CheckError( OCIAttrGet( 
        hDescribe,
        (ub4) OCI_HTYPE_DESCRIBE,
        (dvoid*) &hParam,
        (ub4*) NULL,
        (ub4) OCI_ATTR_PARAM, 
        hError ), hError );

    CheckError( OCIAttrGet( 
        hParam,
        (ub4) OCI_DTYPE_PARAM,
        (dvoid*) &hRef,
        (ub4*) NULL,
        (ub4) OCI_ATTR_REF_TDO, 
        hError ), hError );

    CheckError( OCIObjectPin( 
        hEnv, 
        hError,
        hRef,
        (OCIComplexObject*) NULL,
        (OCIPinOpt) OCI_PIN_ANY,
        (OCIDuration) OCI_DURATION_SESSION,
        (OCILockOpt) OCI_LOCK_NONE, 
        (dvoid**) (dvoid*) &hType ), hError );

    return hType;
}

void OWConnection::CreateType( sdo_geometry** pphData )
{
    CheckError( OCIObjectNew(
        hEnv,
        hError,
        hSvcCtx,
        OCI_TYPECODE_OBJECT, 
        hGeometryTDO, 
        (dvoid *) 0,
        OCI_DURATION_CALL, 
        TRUE, 
        (dvoid **) pphData), hError );
}

void OWConnection::DestroyType( sdo_geometry** pphData )
{
    CheckError( OCIObjectFree(
        hEnv,
        hError, 
        (dvoid *) *pphData, 
        (ub2) 0), NULL );
}

OWStatement* OWConnection::CreateStatement( const char* pszStatementIn )
{
    OWStatement* poStatement = new OWStatement( this, pszStatementIn );

    return poStatement;
}

/*****************************************************************************/
/*                           OWStatement                                     */
/*****************************************************************************/

OWStatement::OWStatement( OWConnection* pConnection,
                          const char* pszStatementIn )
{
    poConnect       = pConnection;
    pszStatement    = CPLStrdup( pszStatementIn );
    nStmtMode       = OCI_DEFAULT;
    nNextCol        = 0;
    nNextBnd        = 0;
    hError          = pConnection->hError;

    //  -----------------------------------------------------------
    //  Create Statement handler
    //  -----------------------------------------------------------

    OCIStmt* hStatement;

    CheckError( OCIHandleAlloc( (dvoid*) poConnect->hEnv, 
        (dvoid**) (dvoid*) &hStatement, 
        (ub4) OCI_HTYPE_STMT, 
        (size_t) 0, 
        (dvoid**) NULL), hError );

    hStmt = hStatement;   // Save Statement Handle

    //  -----------------------------------------------------------
    //  Prepare Statement
    //  -----------------------------------------------------------

    CheckError( OCIStmtPrepare( hStmt, 
        hError, 
        (text*) pszStatement, 
        (ub4) strlen(pszStatement),
        (ub4) OCI_NTV_SYNTAX, 
        (ub4) OCI_DEFAULT ), hError );

    //  -----------------------------------------------------------
    //  Get Statement type 
    //  -----------------------------------------------------------

    ub2 nStmtType;

    CheckError( OCIAttrGet( (dvoid*) hStmt, 
        (ub4) OCI_HTYPE_STMT, 
        (dvoid*) &nStmtType, 
        (ub4*) 0, 
        (ub4) OCI_ATTR_STMT_TYPE, 
        hError ), hError );

    //  -----------------------------------------------------------
    //  Set Statement mode
    //  -----------------------------------------------------------

    if( nStmtType != OCI_STMT_SELECT )
    {
        nStmtMode = OCI_COMMIT_ON_SUCCESS;
    }
}

OWStatement::~OWStatement()
{
    CPLFree_nt( pszStatement );

    OCIHandleFree( (dvoid*) hStmt, (ub4) OCI_HTYPE_STMT);
}

bool OWStatement::Execute( int nRows )
{
    CPLDebug("GEOR","Execute:\n%s\n", pszStatement );

    sword nStatus = OCIStmtExecute( poConnect->hSvcCtx, 
        hStmt, 
        hError, 
        (ub4) ( nStmtMode != OCI_DEFAULT ), 
        (ub4) nRows, 
        (OCISnapshot*) NULL, 
        (OCISnapshot*) NULL, 
        nStmtMode );

    if( CheckError( nStatus, hError ) )
    {
        return false;
    }

    if( nStatus == OCI_SUCCESS_WITH_INFO || nStatus == OCI_NO_DATA )
    {
        return false;
    }

    return true;
}

bool OWStatement::Fetch( int nRows )
{
    sword nStatus = 0;

    nStatus = OCIStmtFetch2 ( 
        (OCIStmt*) hStmt,
        (OCIError*) poConnect->hError,
        (ub4) nRows,
        (ub2) OCI_FETCH_NEXT,
        (sb4) 0,
        (ub4) OCI_DEFAULT );

    if( nStatus == OCI_NO_DATA )
    {
        return false;
    }

    if( CheckError( nStatus, poConnect->hError ) )
    {
        return false;
    }

    return true;
}

void OWStatement::Define( int* pnData )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    CheckError( OCIDefineByPos( hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) pnData, 
        (sb4) sizeof(int),
        (ub2) SQLT_INT, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), hError );
}

void OWStatement::Bind( int* pnData )
{
    OCIBind* hBind = NULL;

    nNextBnd++;

    CheckError( OCIBindByPos(
        hStmt,
        &hBind,
        hError,
        (ub4) nNextBnd,
        (dvoid*) pnData,
        (sb4) sizeof(int),
        (ub2) SQLT_INT,
        (void*) NULL,
        (ub2*) NULL,
        (ub2*) NULL,
        (ub4) NULL,
        (ub4) NULL,
        (ub4) OCI_DEFAULT ),
        hError );
}

void OWStatement::Bind( double* pnData )
{
    OCIBind* hBind = NULL;

    nNextBnd++;

    CheckError( OCIBindByPos( 
        hStmt,
        &hBind,
        hError,
        (ub4) nNextBnd,
        (dvoid*) pnData,
        (sb4) sizeof(double),
        (ub2) SQLT_BDOUBLE,
        (void*) NULL,
        (ub2*) NULL,
        (ub2*) NULL,
        (ub4) NULL,
        (ub4) NULL,
        (ub4) OCI_DEFAULT ),
        hError );
}

void OWStatement::Define( double* pfdData )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    CheckError( OCIDefineByPos( hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) pfdData, 
        (sb4) sizeof(double),
        (ub2) SQLT_BDOUBLE, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), hError );
}

void OWStatement::Define( char* pszData, int nSize )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    CheckError( OCIDefineByPos( 
        hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) pszData, 
        (sb4) nSize,
        (ub2) SQLT_STR, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), 
        hError );
}

void OWStatement::Bind( char* pszData, int nSize )
{
    OCIBind* hBind = NULL;

    nNextBnd++;

    CheckError( OCIBindByPos( 
        hStmt, 
        &hBind, 
        hError,
        (ub4) nNextBnd, 
        (dvoid*) pszData, 
        (sb4) nSize,
        (ub2) SQLT_STR, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) NULL, 
        (ub4) NULL, 
        (ub4) OCI_DEFAULT ), 
        hError );
}

void OWStatement::Define( OCILobLocator** pphLocator, bool bBLOB ) 
{
    OCIDefine*  hDefine = NULL;

    nNextCol++;

    CheckError( OCIDescriptorAlloc( 
        poConnect->hEnv,
        (void**) pphLocator, 
        OCI_DTYPE_LOB, 
        0, 
        0), 
        hError );

    ub2 eDTY = bBLOB ? SQLT_BLOB : SQLT_CLOB;

    CheckError( OCIDefineByPos( 
        hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) pphLocator, 
        (sb4) 0,
        (ub2) eDTY, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), 
        hError );
}

void OWStatement::Define( OCIArray** pphData )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    CheckError( OCIDefineByPos( hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) NULL, 
        (sb4) 0,
        (ub2) SQLT_NTY, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), hError );

    CheckError( OCIDefineObject( hDefine, 
        hError, 
        poConnect->hNumArrayTDO,
        (dvoid**) pphData, 
        (ub4*) NULL, 
        (dvoid**) NULL, 
        (ub4*) NULL ), hError );
}

void OWStatement::Define( sdo_georaster** pphData )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    CheckError( OCIDefineByPos( hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) NULL, 
        (sb4) 0,
        (ub2) SQLT_NTY, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), hError );

    CheckError( OCIDefineObject( hDefine, 
        hError, 
        poConnect->hGeoRasterTDO,
        (dvoid**) pphData, 
        (ub4*) NULL, 
        (dvoid**) NULL, 
        (ub4*) NULL ), hError );
}

void OWStatement::Define( sdo_geometry** pphData )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    CheckError( OCIDefineByPos( hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) NULL, 
        (sb4) 0,
        (ub2) SQLT_NTY, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), hError );

    CheckError( OCIDefineObject( hDefine, 
        hError, 
        poConnect->hGeometryTDO,
        (dvoid**) pphData, 
        (ub4*) NULL, 
        (dvoid**) NULL, 
        (ub4*) NULL ), hError );
}

void OWStatement::Define( sdo_geor_colormap** pphData )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    CheckError( OCIDefineByPos( hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) NULL, 
        (sb4) 0,
        (ub2) SQLT_NTY, 
        (void*) NULL, 
        (ub2*) NULL, 
        (ub2*) NULL, 
        (ub4) OCI_DEFAULT ), hError );

    CheckError( OCIDefineObject( hDefine, 
        hError, 
        poConnect->hColormapTDO,
        (dvoid**) pphData, 
        (ub4*) NULL, 
        (dvoid**) NULL, 
        (ub4*) NULL ), hError );
}

void OWStatement::Define( OCILobLocator** pphLocator, int nIterations )
{
    OCIDefine* hDefine = NULL;

    nNextCol++;

    int i, nLobEmpty;

    sword nStatus = 0;

    for (i = 0; i < nIterations; i++)
    { 
        nStatus = OCIDescriptorAlloc( poConnect->hEnv,
            (void**) &pphLocator[i], 
            OCI_DTYPE_LOB, 
            0, 
            0);

        nLobEmpty = 0;

        nStatus = OCIAttrSet(pphLocator[i], 
            OCI_DTYPE_LOB, 
            &nLobEmpty, 
            0, 
            OCI_ATTR_LOBEMPTY, 
            hError);
    }

    CheckError( OCIDefineByPos( hStmt, 
        &hDefine, 
        hError,
        (ub4) nNextCol, 
        (dvoid*) pphLocator, 
        (sb4) -1,
        (ub2) SQLT_BLOB, 
        (void*) 0, 
        (ub2*) 0, 
        (ub2*) 0, 
        (ub4) OCI_DEFAULT ), hError );
}

int OWStatement::GetInteger( OCINumber* ppoData )
{
    sb4 nRetVal;

    CheckError( OCINumberToInt( 
        hError, 
        ppoData, 
        (uword) sizeof(sb4), 
        OCI_NUMBER_SIGNED,
        (dvoid *) &nRetVal ), 
        hError );

    return nRetVal;
}

double OWStatement::GetDouble( OCINumber* ppoData )
{
    double dfRetVal;

    CheckError( OCINumberToReal( 
        hError, 
        ppoData, 
        (uword) sizeof(dfRetVal), 
        (dvoid*) &dfRetVal ), 
        hError );

    return dfRetVal;
}

char* OWStatement::GetString( OCIString* ppoData )
{
    return (char*) OCIStringPtr( 
        poConnect->hEnv, 
        ppoData );
}

void OWStatement::Free( OCILobLocator** pphLocator, int nCount )
{
    if( nCount > 0 && pphLocator != NULL )
    {
        int i = 0;
        for (i = 0; i < nCount; i++)
        {
            if( pphLocator[i] != NULL )
            {
                OCIDescriptorFree(&pphLocator[i], OCI_DTYPE_LOB);
            }
        }
    }
}

int OWStatement::GetElement( OCIArray** ppoData, int nIndex, int* pnResult )
{
    boolean        exists;
    OCINumber      *oci_number;
    ub4            element_type;

    *pnResult = 0;

    if( CheckError( OCICollGetElem( 
        poConnect->hEnv, 
        hError, 
        (OCIColl*) *ppoData, 
        (sb4) nIndex, 
        (boolean*) &exists, 
        (dvoid**) (dvoid*) &oci_number, 
        (dvoid**) NULL ), hError ) )
    {
        return *pnResult;
    }

    if( CheckError( OCINumberToInt( 
        hError, 
        oci_number, 
        (uword) sizeof(ub4), 
        OCI_NUMBER_UNSIGNED,
        (dvoid *) &element_type ), hError ) )
    {
        return *pnResult;
    }

    *pnResult = (int) element_type;

    return *pnResult;
}

double OWStatement::GetElement( OCIArray** ppoData, 
                               int nIndex, double* pdfResult )
{
    boolean        exists;
    OCINumber      *oci_number;
    double         element_type;

    *pdfResult = 0.0;

    if( CheckError( OCICollGetElem( 
        poConnect->hEnv, 
        hError, 
        (OCIColl*) *ppoData, 
        (sb4) nIndex, 
        (boolean*) &exists, 
        (dvoid**) (dvoid*) &oci_number, NULL ), hError ) )
    {
        return *pdfResult;
    }

    if( CheckError( OCINumberToReal( 
        hError, 
        oci_number, 
        (uword) sizeof(double), 
        (dvoid *) &element_type ), hError ) )
    {
        return *pdfResult;
    }

    *pdfResult = (double) element_type;

    return *pdfResult;
}


bool OWStatement::ReadBlob( OCILobLocator* phLocator, void* pBuffer, 
                            int nSize )
{
    ub4 nAmont      = (ub4) nSize;

    if( CheckError( OCILobRead( 
        poConnect->hSvcCtx,
        hError,
        phLocator,
        (ub4*) &nAmont,
        (ub4) 1,
        (dvoid*) pBuffer, 
        (ub4) nSize, 
        0, 0, 0, 0 ),
        hError ) )
    {
        return false;
    }

    return ( nAmont == (ub4) nSize );
}

bool OWStatement::WriteBlob( OCILobLocator* phLocator, void* pBuffer, 
                             int nSize )
{
    ub4 nAmont  = (ub4) nSize;

    if( CheckError( OCILobWrite( 
        poConnect->hSvcCtx,
        hError,
        phLocator,
        (ub4*) &nAmont,
        (ub4) 1,
        (dvoid*) pBuffer,
        (ub4) nSize,
        (ub1) OCI_ONE_PIECE,
        (dvoid*) NULL,
        NULL,
        (ub2) 0,
        (ub1) SQLCS_IMPLICIT ), 
        hError ) )
    {
        return false;
    }

    return ( nAmont == (ub4) nSize );
}

char* OWStatement::ReadClob( OCILobLocator* phLocator )
{
    ub4 nSize  = 0;
    ub4 nAmont = 0;

    char* pszBuffer = NULL;

    if( CheckError( OCILobGetLength ( 
        poConnect->hSvcCtx,
        hError,
        phLocator,
        (ub4*) &nSize ), 
        hError ) )
    {
        return NULL;
    }

    pszBuffer = (char*) VSIMalloc( sizeof(char*) * nSize );

    if( pszBuffer == NULL)
    {
        return NULL;
    }

    if( CheckError( OCILobRead( 
        poConnect->hSvcCtx,
        hError,
        phLocator,
        (ub4*) &nAmont,
        (ub4) 1,
        (dvoid*) pszBuffer, 
        (ub4) nSize, 
        0, 0, 0, 0), 
        hError ) )
    {
        CPLFree_nt( pszBuffer );
        return NULL;
    }

    if( nAmont == nSize )
    {
        pszBuffer[nAmont] = '\0';
    }
    else
    {
        CPLFree_nt( pszBuffer);
    }

    return pszBuffer;
}

void OWStatement::BindName( char* pszName, int* pnData )
{
    OCIBind* hBind = NULL;

    CheckError( OCIBindByName( 
        (OCIStmt*) hStmt,
        (OCIBind**) &hBind,
        (OCIError*) hError,
        (text*) pszName,
        (sb4) -1,
        (dvoid*) pnData,
        (sb4) sizeof(int),
        (ub2) SQLT_INT,
        (dvoid*) NULL,
        (ub2*) NULL,
        (ub2*) NULL,
        (ub4) 0,
        (ub4*) NULL,
        (ub4) OCI_DEFAULT ), 
        hError );
}

void OWStatement::BindName( char* pszName, char* pszData, int nSize )
{
    OCIBind* hBind = NULL;

    CheckError( OCIBindByName( 
        (OCIStmt*) hStmt,
        (OCIBind**) &hBind,
        (OCIError*) hError,
        (text*) pszName,
        (sb4) -1,
        (dvoid*) pszData,
        (sb4) nSize,
        (ub2) SQLT_STR,
        (dvoid*) NULL,
        (ub2*) NULL,
        (ub2*) NULL,
        (ub4) 0,
        (ub4*) NULL,
        (ub4) OCI_DEFAULT ), 
        hError );
}

void OWStatement::BindName( char* pszName, OCILobLocator** pphLocator )
{
    OCIBind* hBind = NULL;

    CheckError( OCIDescriptorAlloc( 
        poConnect->hEnv,
        (void**) pphLocator, 
        OCI_DTYPE_LOB, 
        0, 
        0), 
        hError );

    CheckError( OCIBindByName( 
        (OCIStmt*) hStmt,
        (OCIBind**) &hBind,
        (OCIError*) hError,
        (text*) pszName,
        (sb4) -1,
        (dvoid*) pphLocator,
        (sb4) -1,
        (ub2) SQLT_BLOB,
        (dvoid*) NULL,
        (ub2*) NULL,
        (ub2*) NULL,
        (ub4) 0,
        (ub4*) NULL,
        (ub4) OCI_DEFAULT ), 
        hError );
}

/*****************************************************************************/
/*               Check for valid integer number in a string                  */
/*****************************************************************************/

bool OWIsNumeric( const char *pszText )
{
    if( pszText == NULL )
    {
        return false;
    }

    const char* pszPos = pszText;

    while( *pszPos != '\0' )
    {
        if( *pszPos < '0' || 
            *pszPos > '9' )
            return false;
        pszPos++;
    }

    return true;
}

/*****************************************************************************/
/*                            Replace String at given char token             */
/*****************************************************************************/

const char* OWReplaceToken( const char* pszBaseString, char cToken, 
                           const char* pszOWReplaceToken )
{
    char pszResult[2048];

    char* psz1 = (char*) pszBaseString;
    char* psz2 = (char*) pszOWReplaceToken;
    char* psz3 = (char*) pszResult;

    for( ; *psz1 != '\0'; psz1++ )
    {
        if( *psz1 == cToken )
        {
            for( ; *psz2 != '\0'; psz2++ )
            {
                *(psz3++) = *psz2;
            }
        }
        else
        {
            *(psz3++) = *psz1;
        }
    }

    *psz3 = '\0';

    return CPLStrdup( pszResult );
}

/*****************************************************************************/
/*                     Parse Value after a Hint on a string                  */
/*****************************************************************************/

int OWParseValue( const char* pszText, const char* pszHint, int nOffset )
{
    if( pszText == NULL ) return 0;

    int i       = 0;
    int nCode   = 0;
    int nCount  = 0;

    char **papszTokens = CSLTokenizeString2( pszText, " .",
        CSLT_HONOURSTRINGS | CSLT_ALLOWEMPTYTOKENS );

    nCount = CSLCount( papszTokens );

    for( i = 0; i < nCount; i++ )
    {
        if( EQUAL( papszTokens[i], pszHint ) && ( i + nOffset ) < nCount )
        {
            nCode = atoi( papszTokens[i + nOffset] );
            break;
        }
    }

    CSLDestroy( papszTokens );

    return nCode;
}

/*****************************************************************************/
/*                            Parse Release Version                          */
/*****************************************************************************/

int OWParseServerVersion( const char* pszText )
{
    return OWParseValue( pszText, "Release", 1 );
}

/*****************************************************************************/
/*                            Parse EPSG Codes                               */
/*****************************************************************************/

/* Input Examples:
*
*      DATUM["World Geodetic System 1984 (EPSG ID 6326)",
*      SPHEROID["WGS 84 (EPSG ID 7030)",6378137,298.257223563]],
*      PROJECTION["UTM zone 50N (EPSG OP 16050)"],
*/

int OWParseEPSG( const char* pszText )
{
    return OWParseValue( pszText, "EPSG", 2 );
}

/*****************************************************************************/
/*                            Convert Data type description                  */
/*****************************************************************************/

const GDALDataType OWGetDataType( const char* pszCellDepth )
{
    unsigned int i;

    for( i = 0; 
        i < (sizeof(ahOW_CellDepth) / sizeof(OW_CellDepth)); 
        i++ )
    {
        if( EQUAL( ahOW_CellDepth[i].pszValue, pszCellDepth ) )
        {
            return ahOW_CellDepth[i].eDataType;
        }
    }

    return GDT_Unknown;
}

/*****************************************************************************/
/*                            Convert Data type description                  */
/*****************************************************************************/

const char* OWSetDataType( const GDALDataType eType )

{
    unsigned int i;

    for( i = 0; 
        i < (sizeof(ahOW_CellDepth) / sizeof(OW_CellDepth)); 
        i++ )
    {
        if( ahOW_CellDepth[i].eDataType == eType )
        {
            return ahOW_CellDepth[i].pszValue;
        }
    }

    return "Unknown";
}

/*****************************************************************************/
/*                            Check for Failure                              */
/*****************************************************************************/

bool CheckError( sword nStatus, OCIError* hError )
{
    text    szMsg[OWTEXT];
    sb4     nCode = 0;

    switch ( nStatus )
    {
    case OCI_SUCCESS:
        return false;
        break;
    case OCI_SUCCESS_WITH_INFO:
        CPLError( CE_Failure, CPLE_AppDefined, "OCI_SUCCESS_WITH_INFO\n" );
        break;
    case OCI_NEED_DATA:
        CPLError( CE_Failure, CPLE_AppDefined, "OCI_NEED_DATA\n" );
        break;
    case OCI_NO_DATA:
        CPLError( CE_Failure, CPLE_AppDefined, "OCI_NODATA\n" );
        break;
    case OCI_INVALID_HANDLE:
        CPLError( CE_Failure, CPLE_AppDefined, "OCI_INVALID_HANDLE\n" );
        break;
    case OCI_STILL_EXECUTING:
        CPLError( CE_Failure, CPLE_AppDefined, "OCI_STILL_EXECUTE\n" );
        break;
    case OCI_CONTINUE:
        CPLError( CE_Failure, CPLE_AppDefined, "OCI_CONTINUE\n" );
        break;
    case OCI_ERROR:

        if( hError == NULL)
        {
            CPLError( CE_Failure, CPLE_AppDefined, 
                "OCI_ERROR with no error handler\n" );
        }

        OCIErrorGet( (dvoid *) hError, (ub4) 1, 
            (text *) NULL, &nCode, szMsg, 
            (ub4) sizeof(szMsg), OCI_HTYPE_ERROR);

        if( nCode == 1405 ) //6550
        {
            CPLDebug( "geroaster", "ORA-ERROR 1405 - Null field" );
            return false;
        }

        CPLError( CE_Failure, CPLE_AppDefined, "%.*s\n", 
            sizeof(szMsg), szMsg );
        break;

    default:
        break;
    }

    return true;
}

