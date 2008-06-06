/******************************************************************************
 * $Id$
 *
 * Project:  Hierarchical Data Format Release 5 (HDF5)
 * Purpose:  HDF5 Datasets. Open HDF5 file, fetch metadata and list of
 *           subdatasets.
 *           This driver initially based on code supplied by Markus Neteler
 * Author:  Denis Nadeau <denis.nadeau@gmail.com>
 *
 ******************************************************************************
 * Copyright (c) 2005, Frank Warmerdam <warmerdam@pobox.com>
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

#define H5_USE_16_API

#include "hdf5.h"

#include "gdal_priv.h"
#include "cpl_string.h"
#include "hdf5dataset.h"

CPL_CVSID("$Id$");

CPL_C_START
void	GDALRegister_HDF5(void);
CPL_C_END



/************************************************************************/
/* ==================================================================== */
/*				HDF5Dataset				*/
/* ==================================================================== */
/************************************************************************/

/************************************************************************/
/*                        GDALRegister_HDF5()				*/
/************************************************************************/
void GDALRegister_HDF5()
    
{
    GDALDriver	*poDriver;
    if( GDALGetDriverByName("HDF5") == NULL )
	{
	    poDriver = new GDALDriver();
	    poDriver->SetDescription("HDF5");
	    poDriver->SetMetadataItem(GDAL_DMD_LONGNAME, 
				      "Hierarchical Data Format Release 5");
	    poDriver->SetMetadataItem(GDAL_DMD_HELPTOPIC, 
				      "frmt_hdf5.html");
	    poDriver->SetMetadataItem(GDAL_DMD_EXTENSION, "hdf5");
	    poDriver->pfnOpen = HDF5Dataset::Open;
	    GetGDALDriverManager()->RegisterDriver(poDriver);
	}
}

/************************************************************************/
/*                           HDF5Dataset()                      	*/
/************************************************************************/
HDF5Dataset::HDF5Dataset()
{
    papszSubDatasets    = NULL;
    papszMetadata       = NULL;
    poH5RootGroup       = NULL;
    pszFilename         = NULL;
    nSubDataCount       = 0;
    hHDF5               = -1;
    hDatasetID          = -1;
    hGroupID            = -1;
    bIsHDFEOS           = FALSE;
    nDatasetType        = -1;
}

/************************************************************************/
/*                            ~HDF5Dataset()                            */
/************************************************************************/
HDF5Dataset::~HDF5Dataset()
{
    if( papszMetadata )
	CSLDestroy( papszMetadata );
    if( hHDF5 > 0 )
	H5Fclose( hHDF5 );
    if( hGroupID > 0 )
	H5Gclose( hGroupID );
    if( papszSubDatasets )
	CSLDestroy( papszSubDatasets );
    if( pszFilename != NULL )
	CPLFree( pszFilename );
    if( poH5RootGroup != NULL ){
	DestroyH5Objects( poH5RootGroup );
	CPLFree( poH5RootGroup->pszName );
	CPLFree( poH5RootGroup->pszPath );
	CPLFree( poH5RootGroup->pszUnderscorePath );
	CPLFree( poH5RootGroup->poHchild );
	delete poH5RootGroup;
    }
}

/************************************************************************/
/*                            GetDataType()                             */
/*                                                                      */
/*      Transform HDF5 datatype to GDAL datatype                        */
/************************************************************************/
GDALDataType HDF5Dataset::GetDataType(hid_t TypeID) 
{
    if( H5Tequal( H5T_NATIVE_CHAR,        TypeID ) )
	return GDT_Byte;
    else if( H5Tequal( H5T_NATIVE_UCHAR,  TypeID ) ) 
	return GDT_Byte;
    else if( H5Tequal( H5T_NATIVE_SHORT,  TypeID ) )
	return GDT_Int16;
    else if( H5Tequal( H5T_NATIVE_USHORT, TypeID ) ) 
	return GDT_UInt16;
    else if( H5Tequal( H5T_NATIVE_INT,    TypeID ) ) 
	return GDT_Int16;      
    else if( H5Tequal( H5T_NATIVE_UINT,   TypeID ) ) 
	return GDT_UInt16;
    else if( H5Tequal( H5T_NATIVE_LONG,   TypeID ) ) 
	return GDT_Int32;      
    else if( H5Tequal( H5T_NATIVE_ULONG,  TypeID ) ) 
	return GDT_UInt32;
    else if( H5Tequal( H5T_NATIVE_FLOAT,  TypeID ) ) 
	return GDT_Float32;
    else if( H5Tequal( H5T_NATIVE_DOUBLE, TypeID ) ) 
	return GDT_Float64;
    else if( H5Tequal( H5T_NATIVE_LLONG,  TypeID ) ) 
	return GDT_Unknown;
    else if( H5Tequal( H5T_NATIVE_ULLONG, TypeID ) ) 
	return GDT_Unknown;
    else if( H5Tequal( H5T_NATIVE_DOUBLE, TypeID ) ) 
	return GDT_Unknown;

    return GDT_Unknown;
}

/************************************************************************/
/*                          GetDataTypeName()                           */
/*                                                                      */
/*      Return the human readable name of data type                     */
/************************************************************************/
const char *HDF5Dataset::GetDataTypeName(hid_t TypeID)
{
    if( H5Tequal( H5T_NATIVE_CHAR,        TypeID ) )
	return "8-bit character";
    else if( H5Tequal( H5T_NATIVE_UCHAR,  TypeID ) ) 
	return "8-bit unsigned character";    
    else if( H5Tequal( H5T_NATIVE_SHORT,  TypeID ) )
	return "8-bit integer";
    else if( H5Tequal( H5T_NATIVE_USHORT, TypeID ) ) 
	return "8-bit unsigned integer";
    else if( H5Tequal( H5T_NATIVE_INT,    TypeID ) ) 
	return "16-bit integer";
    else if( H5Tequal( H5T_NATIVE_UINT,   TypeID ) ) 
	return "16-bit unsigned integer";
    else if( H5Tequal( H5T_NATIVE_LONG,   TypeID ) ) 
	return "32-bit integer";
    else if( H5Tequal( H5T_NATIVE_ULONG,  TypeID ) ) 
	return "32-bit unsigned integer";
    else if( H5Tequal( H5T_NATIVE_FLOAT,  TypeID ) ) 
	return "32-bit floating-point";
    else if( H5Tequal( H5T_NATIVE_DOUBLE, TypeID ) ) 
	return "64-bit floating-point";
    else if( H5Tequal( H5T_NATIVE_LLONG,  TypeID ) ) 
	return "64-bit integer";
    else if( H5Tequal( H5T_NATIVE_ULLONG, TypeID ) ) 
	return "64-bit unsigned integer";
    else if( H5Tequal( H5T_NATIVE_DOUBLE, TypeID ) ) 
      return "64-bit floating-point";
    
    return "Unknown";
}

/************************************************************************/
/*                            GetMetadata()                             */
/************************************************************************/
char **HDF5Dataset::GetMetadata( const char *pszDomain )
{
    if( pszDomain != NULL && EQUALN( pszDomain, "SUBDATASETS", 11 ) )
        return papszSubDatasets;
    else
        return GDALDataset::GetMetadata( pszDomain );
}

 
/************************************************************************/
/*                                Open()                                */
/************************************************************************/
GDALDataset *HDF5Dataset::Open( GDALOpenInfo * poOpenInfo )
{
    HDF5Dataset *poDS;
    CPLErr      Err;
    
    if( poOpenInfo->nHeaderBytes < 32 )
        return NULL;

    if( !EQUALN((const char *) (poOpenInfo->pabyHeader+1), "HDF",3) )
        return NULL;

/* -------------------------------------------------------------------- */
/*  We have special routine in the HDF library for format checking!     */
/* -------------------------------------------------------------------- */
    if( !H5Fis_hdf5( poOpenInfo->pszFilename ) ) {
	return NULL;
    }
    
/* -------------------------------------------------------------------- */
/*      Create datasource.                                              */
/* -------------------------------------------------------------------- */
    poDS = new HDF5Dataset();
    
    poDS->fp = poOpenInfo->fp;
    poOpenInfo->fp = NULL;
    
    poDS->pszFilename = strdup( poOpenInfo->pszFilename );

/* -------------------------------------------------------------------- */
/*      Try opening the dataset.                                        */
/* -------------------------------------------------------------------- */
    poDS->hHDF5 = H5Fopen( poOpenInfo->pszFilename, 
			   H5F_ACC_RDONLY, 
			   H5P_DEFAULT );
    if( poDS->hHDF5 < 0 )  {
        delete poDS;
   	return NULL;
    }
    
    poDS->hGroupID = H5Gopen( poDS->hHDF5, "/" ); 
    if( poDS->hGroupID < 0 ){
	poDS->bIsHDFEOS=false;
        delete poDS;
	return NULL;
    }
    
    poDS->bIsHDFEOS=true;
    Err = poDS->ReadGlobalAttributes( true );

    poDS->SetMetadata( poDS->papszMetadata  );
    
    return( poDS );
}

/************************************************************************/
/*                          DestroyH5Objects()                          */
/*                                                                      */
/*      Erase all objects                                               */
/************************************************************************/
void HDF5Dataset::DestroyH5Objects( HDF5GroupObjects *poH5Object )
{
    int i;


/* -------------------------------------------------------------------- */
/*      Visit all objects                                               */
/* -------------------------------------------------------------------- */

    for( i=0; i < poH5Object->nbObjs; i++ )
	if( poH5Object->poHchild+i != NULL )
	    DestroyH5Objects( poH5Object->poHchild+i );

    if( poH5Object->poHparent ==NULL )
	return;

/* -------------------------------------------------------------------- */
/*      Erase some data                                                 */
/* -------------------------------------------------------------------- */
    if( poH5Object->paDims != NULL ) {
	CPLFree( poH5Object->paDims );
    }

    if( poH5Object->pszPath != NULL ) {
    	CPLFree( poH5Object->pszPath );
    }

    if( poH5Object->pszName != NULL ) {
	CPLFree( poH5Object->pszName );
    }

    if( poH5Object->pszUnderscorePath != NULL ) {
	CPLFree( poH5Object->pszUnderscorePath );
    }
/* -------------------------------------------------------------------- */
/*      All Children are visited and can be deleted.                    */
/* -------------------------------------------------------------------- */
    if( ( i==poH5Object->nbObjs ) && ( poH5Object->nbObjs!=0 ) ) {
	CPLFree( poH5Object->poHchild );
    }

}

/************************************************************************/
/*                             CreatePath()                             */
/*                                                                      */
/*      Find Dataset path for HDopen                                    */
/************************************************************************/
char* CreatePath( HDF5GroupObjects *poH5Object )
{
    char pszPath[8192];
    char pszUnderscoreSpaceInName[8192];
    char *popszPath;
    int  i;
    char **papszPath;

/* -------------------------------------------------------------------- */
/*      Recurse to the root path                                        */
/* -------------------------------------------------------------------- */
    pszPath[0]='\0';
    if( poH5Object->poHparent !=NULL ) {
	popszPath=CreatePath( poH5Object->poHparent );
	strcpy( pszPath,popszPath );
    }
	
/* -------------------------------------------------------------------- */
/*      add name to the path                                            */
/* -------------------------------------------------------------------- */
    if( !EQUAL( poH5Object->pszName,"/" ) ){
	strcat( pszPath,"/" );
	strcat( pszPath,poH5Object->pszName );
    }

/* -------------------------------------------------------------------- */
/*      fill up path for each object                                    */
/* -------------------------------------------------------------------- */
    if( poH5Object->pszPath == NULL ) {

	if( strlen( poH5Object->pszName ) == 1 ) {
	    strcat(pszPath, poH5Object->pszName );
	    strcpy(pszUnderscoreSpaceInName, poH5Object->pszName);
	}
	else {
/* -------------------------------------------------------------------- */
/*      Change space for underscore                                     */
/* -------------------------------------------------------------------- */
	    papszPath = CSLTokenizeString2( pszPath,
					    " ", CSLT_HONOURSTRINGS );
	    
	    strcpy(pszUnderscoreSpaceInName,papszPath[0]);
	    for( i=1; i < CSLCount( papszPath ); i++ ) {
		strcat( pszUnderscoreSpaceInName, "_" );
		strcat( pszUnderscoreSpaceInName, papszPath[ i ] );
	    }
	    CSLDestroy(papszPath);

	}
	poH5Object->pszUnderscorePath  = 
	    (char *)strdup( pszUnderscoreSpaceInName );
	poH5Object->pszPath  = (char *)strdup( pszPath );
    }

    return( poH5Object->pszPath );
}


/************************************************************************/
/*                      HDF5CreateGroupObjs()                           */
/*                                                                      */
/*      Create HDF5 hierarchy into a linked list                        */
/************************************************************************/
herr_t HDF5CreateGroupObjs(hid_t hHDF5, const char *pszObjName, 
			   void *poHObjParent)
{
    herr_t	ret;			/* error return status */
    hid_t	hGroupID;		/* identifier of group */
    hid_t	hDatasetID;		/* identifier of dataset */
    hsize_t     nbObjs=0;		/* number of objects in a group */
    int         nbAttrs=0;		/* number of attributes in object */
    int		idx;
    int         n_dims;
    H5G_stat_t  oStatbuf;
    hsize_t     *dims=NULL;
    hsize_t     *maxdims=NULL;
    hid_t       datatype;
    hid_t       dataspace;
    hid_t       native;
    herr_t status;

    char *CreatePath( HDF5GroupObjects *poH5Object );

    HDF5GroupObjects *poHchild;
    HDF5GroupObjects *poHparent;

    poHparent = ( HDF5GroupObjects * ) poHObjParent;
    poHchild=poHparent->poHchild;

    if( H5Gget_objinfo( hHDF5, pszObjName, FALSE, &oStatbuf ) < 0  )
	return -1;

    
/* -------------------------------------------------------------------- */
/*      Look for next child                                             */
/* -------------------------------------------------------------------- */
    for( idx=0; idx < poHparent->nbObjs; idx++ )	{
	if( poHchild->pszName == NULL ) break;
	poHchild++;
    }

    if( idx == poHparent->nbObjs ) 
	return -1;  // all children parsed
    
/* -------------------------------------------------------------------- */
/*      Save child information                                          */
/* -------------------------------------------------------------------- */
    poHchild->pszName  = (char *)strdup( pszObjName );
    
    poHchild->nType  = oStatbuf.type;
    poHchild->nIndex = idx;
    poHchild->poHparent = poHparent;
    poHchild->nRank     = 0;
    poHchild->paDims    = 0;
    poHchild->HDatatype = 0;
    if( poHchild->pszPath == NULL ) {
	poHchild->pszPath  = CreatePath( poHchild );
    }
    if( poHparent->pszPath == NULL ) {
	poHparent->pszPath = CreatePath( poHparent );
    }


    switch ( oStatbuf.type ) 
	{
	case H5G_LINK:
	    poHchild->nbAttrs = 0;
	    poHchild->nbObjs = 0;
	    poHchild->poHchild = NULL;
	    poHchild->nRank      = 0;
	    poHchild->paDims    = 0;
	    poHchild->HDatatype = 0;
	    break;
	    
	case H5G_GROUP:
	    if( ( hGroupID = H5Gopen( hHDF5, pszObjName ) ) == -1  ) {
		printf( "Error: unable to access \"%s\" group.\n", 
			pszObjName );
		return -1;
	    }
	    nbAttrs          = H5Aget_num_attrs( hGroupID );
	    ret              = H5Gget_num_objs( hGroupID, &nbObjs );
	    poHchild->nbAttrs= nbAttrs;
	    poHchild->nbObjs = nbObjs;
	    poHchild->nRank      = 0;
	    poHchild->paDims    = 0;
	    poHchild->HDatatype = 0;
	    
	    if( nbObjs > 0 ) {
		poHchild->poHchild =( HDF5GroupObjects * )
		    CPLCalloc( nbObjs, sizeof( HDF5GroupObjects ) );
		memset( poHchild->poHchild,0,
			sizeof( HDF5GroupObjects ) * nbObjs );
	    }
	    else 
		poHchild->poHchild = NULL;
	    
	    ret = H5Giterate( hHDF5, pszObjName, NULL, 
			     HDF5CreateGroupObjs,  (void*) poHchild );
	    ret = H5Gclose( hGroupID );
	    break;
	    
	case H5G_DATASET:
	    
	    if( ( hDatasetID = H5Dopen( hHDF5, pszObjName ) ) == -1  ) {
		printf( "Error: unable to access \"%s\" dataset.\n", 
		       pszObjName );
		return -1;
	    }
	    nbAttrs      = H5Aget_num_attrs( hDatasetID );
	    datatype     = H5Dget_type( hDatasetID );
	    dataspace    = H5Dget_space( hDatasetID );
	    n_dims       = H5Sget_simple_extent_ndims( dataspace );
	    native       = H5Tget_native_type( datatype, H5T_DIR_ASCEND );
	    
	    if( n_dims > 0 ) {
		dims     = (hsize_t *) CPLCalloc( n_dims,sizeof( hsize_t ) );
		maxdims  = (hsize_t *) CPLCalloc( n_dims,sizeof( hsize_t ) );
	    }
	    status     = H5Sget_simple_extent_dims( dataspace, dims, maxdims );
	    if( maxdims != NULL )
		CPLFree( maxdims );
	    
	    if( n_dims > 0 ) {
		poHchild->nRank     = n_dims;   // rank of the array
		poHchild->paDims    = dims;      // dimmension of the array.
		poHchild->HDatatype = datatype;  // HDF5 datatype
	    }
	    else  {
		poHchild->nRank     = -1;
		poHchild->paDims    = NULL;
		poHchild->HDatatype = 0;
	}
	    poHchild->nbAttrs   = nbAttrs;
	    poHchild->nbObjs    = 0;
	    poHchild->poHchild  = NULL;
	    poHchild->native    = native;
	    ret                 = H5Dclose( hDatasetID );
	    break;
	    
	case H5G_TYPE:
	    poHchild->nbAttrs = 0;
	    poHchild->nbObjs = 0;
	    poHchild->poHchild = NULL;
	    poHchild->nRank      = 0;
	    poHchild->paDims    = 0;
	    poHchild->HDatatype = 0;
	    break;
	    
	default:
	    break;
	}
    
    return 0;
}


/************************************************************************/
/*                          HDF5AttrIterate()                           */
/************************************************************************/

herr_t HDF5AttrIterate( hid_t hH5ObjID, 
			const char *AttrName, 
			void *pDS )
{
    hid_t           hAttrID;
    hid_t           hAttrTypeID;
    hid_t           hAttrNativeType;
    hid_t           hAttrSpace;

    char            szData[8192];
    hsize_t        nSize[64];
    unsigned int            nAttrElmts;
    hsize_t        nAttrSize;
    hsize_t        i;
    void           *buf;
    unsigned int             nAttrDims;


    HDF5Dataset    *poDS;
    char            szTemp[8192];
    char            szValue[8192];

    poDS = (HDF5Dataset *) pDS;
    sprintf( szTemp, "%s:%s", poDS->poH5CurrentObject->pszName, 
	     AttrName );

    hAttrID          = H5Aopen_name( hH5ObjID, AttrName );
    hAttrTypeID      = H5Aget_type( hAttrID );
    hAttrNativeType  = H5Tget_native_type( hAttrTypeID, H5T_DIR_DEFAULT );
    hAttrSpace       = H5Aget_space( hAttrID );
    nAttrDims        = H5Sget_simple_extent_dims( hAttrSpace, nSize, NULL );


    szValue[0] ='\0';

    if( H5Tget_class( hAttrNativeType ) == H5T_STRING ) {
	nAttrSize = H5Tget_size( hAttrTypeID );
	H5Aread( hAttrID, hAttrNativeType, szData  );
	szData[nAttrSize]='\0';
	sprintf( szValue, "%s", szData );

    }
    else {
	nAttrElmts = 1;
	for( i=0; i < nAttrDims; i++ ) {
	    nAttrElmts *= nSize[i];
	}
	if( nAttrElmts > 0 ){
	    buf = (void *) CPLMalloc( nAttrElmts*
				      H5Tget_size( hAttrNativeType ));
	    H5Aread( hAttrID, hAttrNativeType, buf );
	}
	if( H5Tequal( H5T_NATIVE_CHAR, hAttrNativeType ) ){
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%c ", ((char *) buf)[i]);
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_UCHAR,  hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%c", ((char *) buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_SHORT,  hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%d ", ((short *) buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_USHORT, hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%ud ", ((unsigned short *) buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_INT,    hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%d ", ((int *) buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_UINT,   hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%ud ", ((unsigned int *) buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_LONG,   hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%ld ", ((long *)buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_ULONG,  hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%ld ", ((unsigned long *)buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_FLOAT,  hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%f ",  ((float *)buf)[i] );
		strcat(szValue,szData);
	    }
	}
	else if( H5Tequal( H5T_NATIVE_DOUBLE, hAttrNativeType ) ) {
	    for( i=0; i < nAttrElmts; i++ ) {
		sprintf( szData, "%g ",  ((double *)buf)[i] );
		strcat(szValue,szData);
	    }
	}
	CPLFree( buf );

    }
    H5Aclose( hAttrID );
    //printf( "%s = %s\n",szTemp, szValue );
    poDS->papszMetadata =
	CSLSetNameValue( poDS->papszMetadata, szTemp,  
			 CPLSPrintf( "%s", szValue ) );

    return 0;
}

/************************************************************************/
/*                           CreateMetadata()                           */
/************************************************************************/
CPLErr HDF5Dataset::CreateMetadata( HDF5GroupObjects *poH5Object, int nType)
{
    hid_t	hGroupID;		/* identifier of group */
    hid_t       hDatasetID;
    int         nbAttrs;
    herr_t      ret;

    HDF5Dataset *poDS;

    poDS = this;

    poH5CurrentObject = poH5Object;
    nbAttrs = poH5Object->nbAttrs;

    if( EQUAL(poH5Object->pszPath, "" ) )
	return CE_None;

    switch( nType ) {

    case H5G_GROUP:

	hGroupID = H5Gopen( hHDF5, poH5Object->pszPath );
	if( nbAttrs > 0 ) { 
	    ret = H5Aiterate( hGroupID, NULL, 
			      HDF5AttrIterate, (void *)poDS  );
	    ret = H5Gclose( hGroupID );
	}

	break;

    case H5G_DATASET:

	hDatasetID =  H5Dopen(hHDF5, poH5Object->pszPath );

	if( nbAttrs > 0 ) { 
	    ret = H5Aiterate( hDatasetID, NULL, 
			      HDF5AttrIterate, (void *)poDS );
	    ret = H5Dclose( hDatasetID );
	}
	break;

    default:
	break;
    }

    return CE_None;
}


/************************************************************************/
/*                       HDF5FindDatasetObjectsbyPath()                 */
/*      Find object by name                                             */
/************************************************************************/
HDF5GroupObjects* HDF5Dataset::HDF5FindDatasetObjectsbyPath
    ( HDF5GroupObjects *poH5Objects, char* pszDatasetPath )
{
    int i;
    HDF5Dataset *poDS;
    HDF5GroupObjects *poObjectsFound;
    poDS=this;
    
    if( poH5Objects->nType == H5G_DATASET &&
       EQUAL( poH5Objects->pszUnderscorePath,pszDatasetPath ) )	{
	    /*      printf("found it! %ld\n",(long) poH5Objects);*/
	    return( poH5Objects );
	}
    
    if( poH5Objects->nbObjs >0 )
	for( i=0; i <poH5Objects->nbObjs; i++ )   {
	    poObjectsFound=
		poDS->HDF5FindDatasetObjectsbyPath( poH5Objects->poHchild+i, 
						    pszDatasetPath );
/* -------------------------------------------------------------------- */
/*      Is this our dataset??                                           */
/* -------------------------------------------------------------------- */
	    if( poObjectsFound != NULL ) return( poObjectsFound );
	}
/* -------------------------------------------------------------------- */
/*      Dataset has not been found!                                     */
/* -------------------------------------------------------------------- */
    return( NULL );
    
}


/************************************************************************/
/*                       HDF5FindDatasetObjects()                       */
/*      Find object by name                                             */
/************************************************************************/
HDF5GroupObjects* HDF5Dataset::HDF5FindDatasetObjects
    ( HDF5GroupObjects *poH5Objects, char* pszDatasetName )
{
    int i;
    HDF5Dataset *poDS;
    HDF5GroupObjects *poObjectsFound;
    poDS=this;
    
    if( poH5Objects->nType == H5G_DATASET &&
       EQUAL( poH5Objects->pszName,pszDatasetName ) )	{
	    /*      printf("found it! %ld\n",(long) poH5Objects);*/
	    return( poH5Objects );
	}
    
    if( poH5Objects->nbObjs >0 )
	for( i=0; i <poH5Objects->nbObjs; i++ )   {
	    poObjectsFound=
		poDS->HDF5FindDatasetObjects( poH5Objects->poHchild+i, 
					      pszDatasetName );
/* -------------------------------------------------------------------- */
/*      Is this our dataset??                                           */
/* -------------------------------------------------------------------- */
	    if( poObjectsFound != NULL ) return( poObjectsFound );
	    
	}
/* -------------------------------------------------------------------- */
/*      Dataset has not been found!                                     */
/* -------------------------------------------------------------------- */
    return( NULL );
    
}


/************************************************************************/
/*                        HDF5ListGroupObjects()                        */
/*                                                                      */
/*      List all objects in HDF5                                        */
/************************************************************************/
CPLErr HDF5Dataset::HDF5ListGroupObjects( HDF5GroupObjects *poRootGroup,
					  int bSUBDATASET )
{
    int i;
    char szTemp[8192];
    char szDim[8192];
    HDF5Dataset *poDS;
    poDS=this;
    
    if( poRootGroup->nbObjs >0 )  
	for( i=0; i < poRootGroup->nbObjs; i++ ) {
	    poDS->HDF5ListGroupObjects( poRootGroup->poHchild+i, bSUBDATASET );
	}
    

    if( poRootGroup->nType == H5G_GROUP ) {
	CreateMetadata( poRootGroup, H5G_GROUP );
    }

/* -------------------------------------------------------------------- */
/*      Create Sub dataset list                                         */
/* -------------------------------------------------------------------- */
    if( (poRootGroup->nType == H5G_DATASET ) && bSUBDATASET ) {
	
	szDim[0]='\0';
	switch( poRootGroup->nRank ) {
	case 3: 
	    sprintf( szTemp,"%dx%dx%d",
		    (int)poRootGroup->paDims[0],
		    (int)poRootGroup->paDims[1],
		    (int)poRootGroup->paDims[2] );
	    break;
	    
  	case 2: 
	    sprintf( szTemp,"%dx%d",
		    (int)poRootGroup->paDims[0],
		    (int)poRootGroup->paDims[1] );
	  break;
        default:
	    return CE_None;
	    
	}
	strcat( szDim,szTemp );
	
	sprintf( szTemp, "SUBDATASET_%d_NAME", poDS->nSubDataCount );


	poDS->papszSubDatasets =
	    CSLSetNameValue( poDS->papszSubDatasets, szTemp,
			    CPLSPrintf( "HDF5:\"%s\":%s",
					poDS->pszFilename,
					poRootGroup->pszUnderscorePath ) );
	
	sprintf(  szTemp, "SUBDATASET_%d_DESC", poDS->nSubDataCount++ );
	
	poDS->papszSubDatasets =
	    CSLSetNameValue( poDS->papszSubDatasets, szTemp,
			    CPLSPrintf( "[%s] %s (%s)", 
					szDim,
					poRootGroup->pszUnderscorePath,
					poDS->GetDataTypeName
					( poRootGroup->native ) ) );

    }
    
    return CE_None;
}


/************************************************************************/
/*                       ReadGlobalAttributes()                         */
/************************************************************************/
CPLErr HDF5Dataset::ReadGlobalAttributes(int bSUBDATASET)
{
    
    HDF5GroupObjects *poRootGroup;
    
    poRootGroup = new HDF5GroupObjects;
    
    poH5RootGroup=poRootGroup;
    poRootGroup->pszName   = strdup( "/" );
    poRootGroup->nType     = H5G_GROUP;
    poRootGroup->poHparent = NULL;
    poRootGroup->pszPath = NULL;
    
    if( hHDF5 < 0 )  {
	printf( "hHDF5 <0!!\n" );
	return CE_None;
    }
    
    hGroupID = H5Gopen( hHDF5, "/" ); 
    if( hGroupID < 0 ){
	printf( "hGroupID <0!!\n" );
	return CE_None;
    }
    
    poRootGroup->nbAttrs = H5Aget_num_attrs( hGroupID );

    H5Gget_num_objs( hGroupID, (hsize_t *) &( poRootGroup->nbObjs ) );
    
    if( poRootGroup->nbObjs > 0 ) {
	poRootGroup->poHchild = ( HDF5GroupObjects * ) 
	    CPLCalloc( poRootGroup->nbObjs,
		sizeof( HDF5GroupObjects ) );
	H5Giterate( hGroupID, "/", NULL, 
		   HDF5CreateGroupObjs, (void *)poRootGroup );
    }
    else poRootGroup->poHchild = NULL;
    
    HDF5ListGroupObjects( poRootGroup, bSUBDATASET );
    return CE_None;
}
