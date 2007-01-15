/******************************************************************************
 * $Id$
 *
 * Project:  Hierarchical Data Format Release 4 (HDF4)
 * Purpose:  HDF4 Datasets. Open HDF4 file, fetch metadata and list of
 *           subdatasets.
 *           This driver initially based on code supplied by Markus Neteler
 * Author:   Andrey Kiselev, dron@remotesensing.org
 *
 ******************************************************************************
 * Copyright (c) 2002, Andrey Kiselev <dron@remotesensing.org>
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
 ******************************************************************************
 * 
 * $Log$
 * Revision 1.35  2006/12/18 18:00:58  fwarmerdam
 * Make sure error stack is cleared before HEshutdown() or else later
 * requests will crash it.
 *
 * Revision 1.34  2006/12/18 02:17:17  fwarmerdam
 * call HEshutdown() to clear error stack.
 *
 * Revision 1.33  2006/11/23 13:23:56  dron
 * Added more logic to guess HDF-EOS datasets.
 *
 * Revision 1.32  2005/04/25 20:05:27  dron
 * Memory leak fixed.
 *
 * Revision 1.31  2005/04/19 12:21:03  dron
 * Meory leak fixed.
 *
 * Revision 1.30  2005/03/30 11:21:15  dron
 * Remove inline keyword from function definitions.
 *
 * Revision 1.29  2005/03/29 12:42:53  dron
 * Added AnyTypeToDouble() method.
 *
 * Revision 1.28  2004/09/08 17:55:13  dron
 * Few problems fixed.
 *
 * Revision 1.27  2004/06/19 21:37:31  dron
 * Use HDF-EOS library for appropriate datasets; major cpde rewrite.
 *
 * Revision 1.26  2003/11/07 15:48:01  dron
 * TranslateHDF4Attributes() improved, added GetDataTypeSize().
 *
 * Revision 1.25  2003/11/05 17:05:02  warmerda
 * Fixed type casting problem (bug 431).
 *
 * Revision 1.24  2003/06/26 20:42:31  dron
 * Support for Hyperion Level 1 data product.
 *
 * Revision 1.23  2003/06/25 08:26:18  dron
 * Support for Aster Level 1A/1B/2 products.
 *
 * Revision 1.22  2003/06/12 15:07:04  dron
 * Added support for SeaWiFS Level 3 Standard Mapped Image Products.
 *
 * Revision 1.21  2003/06/10 09:32:31  dron
 * Added support for MODIS Level 3 products.
 *
 * Revision 1.20  2003/05/21 14:11:43  dron
 * MODIS Level 1B earth-view (EV) product now supported.
 *
 * Revision 1.19  2003/03/01 15:54:43  dron
 * Significant improvements in HDF EOS metadata parsing.
 *
 * Revision 1.18  2003/02/27 14:28:55  dron
 * Fixes in HDF-EOS metadata parsing algorithm.
 *
 * Revision 1.17  2002/12/19 17:42:57  dron
 * Size of string buffer in TranslateHDF4EOSAttributes() fixed.
 *
 * Revision 1.16  2002/11/29 18:25:22  dron
 * MODIS determination improved.
 *
 * Revision 1.15  2002/11/13 06:43:15  warmerda
 * quote filename in case it contains a drive indicator
 *
 * Revision 1.14  2002/11/13 06:00:04  warmerda
 * avoid use of long long type as it doesn't exist perse on windows
 *
 * Revision 1.13  2002/11/08 18:29:04  dron
 * Added Create() method.
 *
 * Revision 1.12  2002/11/07 13:23:44  dron
 * Support for projection information writing.
 *
 * Revision 1.11  2002/11/06 15:47:14  dron
 * Added support for 3D datasets creation
 *
 * Revision 1.10  2002/10/25 14:28:54  dron
 * Initial support for HDF4 creation.
 *
 * Revision 1.9  2002/09/25 14:44:40  warmerda
 * Fixed iSubdatasetType initialization.
 *
 * Revision 1.8  2002/09/06 10:42:23  dron
 * Georeferencing for ASTER Level 1b datasets and ASTER DEMs.
 *
 * Revision 1.7  2002/09/04 06:50:37  warmerda
 * avoid static driver pointers
 *
 * Revision 1.6  2002/07/23 12:27:58  dron
 * General Raster Interface support added.
 *
 * Revision 1.5  2002/07/19 13:39:17  dron
 * Lists supported in HDF-EOS attributes.
 *
 * Revision 1.4  2002/07/17 16:24:31  dron
 * MODIS support improved a bit.
 *
 * Revision 1.3  2002/07/17 13:36:18  dron
 * <hdf.h> and <mfhdf.h> changed to "hdf.h" and "mfhdf.h".
 *
 * Revision 1.2  2002/07/16 17:51:10  warmerda
 * removed hdf/ from include statements
 *
 * Revision 1.1  2002/07/16 11:04:11  dron
 * New driver: HDF4 datasets. Initial version.
 *
 *
 */

#include "hdf.h"
#include "mfhdf.h"

#include "HdfEosDef.h"

#include "gdal_priv.h"
#include "cpl_string.h"
#include "hdf4dataset.h"

CPL_CVSID("$Id$");

CPL_C_START
void	GDALRegister_HDF4(void);
CPL_C_END

extern const char *pszGDALSignature;

/************************************************************************/
/* ==================================================================== */
/*				HDF4Dataset				*/
/* ==================================================================== */
/************************************************************************/

/************************************************************************/
/*                           HDF4Dataset()                      	*/
/************************************************************************/

HDF4Dataset::HDF4Dataset()

{
    fp = NULL;
    hHDF4 = 0;
    hSD = 0;
    hGR = 0;
    papszGlobalMetadata = NULL;
    papszSubDatasets = NULL;
    bIsHDFEOS = 0;
}

/************************************************************************/
/*                            ~HDF4Dataset()                            */
/************************************************************************/

HDF4Dataset::~HDF4Dataset()

{
    if ( hSD )
	SDend( hSD );
    if ( hGR )
	GRend( hGR );
    if ( papszSubDatasets )
	CSLDestroy( papszSubDatasets );
    if ( papszGlobalMetadata )
	CSLDestroy( papszGlobalMetadata );
    if( fp != NULL )
        VSIFClose( fp );
    if( hHDF4 > 0 )
	Hclose( hHDF4 );
}

/************************************************************************/
/*                            GetMetadata()                             */
/************************************************************************/

char **HDF4Dataset::GetMetadata( const char *pszDomain )

{
    if( pszDomain != NULL && EQUALN( pszDomain, "SUBDATASETS", 11 ) )
        return papszSubDatasets;
    else
        return GDALDataset::GetMetadata( pszDomain );
}

/************************************************************************/
/*                           SPrintArray()                              */
/*	Prints numerical arrays in string buffer.			*/
/*	This function takes pfaDataArray as a pointer to printed array,	*/
/*	nValues as a number of values to print and pszDelimiter as a	*/
/*	field delimiting strings.					*/
/*	Pointer to filled buffer will be returned.			*/
/************************************************************************/

char *SPrintArray( GDALDataType eDataType, void *paDataArray,
                          int nValues, char * pszDelimiter )
{
    char        *pszString, *pszField;
    int         i, iFieldSize, iStringSize;

    iFieldSize = 32 + strlen( pszDelimiter );
    pszField = (char *)CPLMalloc( iFieldSize + 1 );
    iStringSize = nValues * iFieldSize + 1;
    pszString = (char *)CPLMalloc( iStringSize );
    memset( pszString, 0, iStringSize );
    for ( i = 0; i < nValues; i++ )
    {
        switch ( eDataType )
        {
            case GDT_Byte:
                sprintf( pszField, "%d%s", ((GByte *)paDataArray)[i],
                     (i < nValues - 1)?pszDelimiter:"" );
                break;
            case GDT_UInt16:
                sprintf( pszField, "%u%s", ((GUInt16 *)paDataArray)[i],
                     (i < nValues - 1)?pszDelimiter:"" );
                break;
            case GDT_Int16:
            default:
                sprintf( pszField, "%d%s", ((GInt16 *)paDataArray)[i],
                     (i < nValues - 1)?pszDelimiter:"" );
                break;
            case GDT_UInt32:
                sprintf( pszField, "%u%s", ((GUInt32 *)paDataArray)[i],
                     (i < nValues - 1)?pszDelimiter:"" );
                break;
            case GDT_Int32:
                sprintf( pszField, "%d%s", ((GInt32 *)paDataArray)[i],
                     (i < nValues - 1)?pszDelimiter:"" );
                break;
            case GDT_Float32:
                sprintf( pszField, "%.7g%s", ((float *)paDataArray)[i],
                     (i < nValues - 1)?pszDelimiter:"" );
                break;
            case GDT_Float64:
                sprintf( pszField, "%.15g%s", ((double *)paDataArray)[i],
                     (i < nValues - 1)?pszDelimiter:"" );
                break;
        }
        strcat( pszString, pszField );
    }
    
    CPLFree( pszField );
    return pszString;
}

/************************************************************************/
/*              Translate HDF4 data type into GDAL data type            */
/************************************************************************/
GDALDataType HDF4Dataset::GetDataType( int32 iNumType ) const
{
    switch (iNumType)
    {
        case DFNT_CHAR8: // The same as DFNT_CHAR
        case DFNT_UCHAR8: // The same as DFNT_UCHAR
        case DFNT_INT8:
        case DFNT_UINT8:
            return GDT_Byte;
        case DFNT_INT16:
            return GDT_Int16;
        case DFNT_UINT16:
            return GDT_UInt16;
        case DFNT_INT32:
            return GDT_Int32;
        case DFNT_UINT32:
            return GDT_UInt32;
        case DFNT_INT64:
            return GDT_Unknown;
        case DFNT_UINT64:
            return GDT_Unknown;
        case DFNT_FLOAT32:
            return GDT_Float32;
        case DFNT_FLOAT64:
            return GDT_Float64;
        default:
            return GDT_Unknown;
    }
}

/************************************************************************/
/*		Return the human readable name of data type		*/
/************************************************************************/

const char *HDF4Dataset::GetDataTypeName( int32 iNumType ) const
{
    switch (iNumType)
    {
        case DFNT_CHAR8: // The same as DFNT_CHAR
	    return "8-bit character";
	case DFNT_UCHAR8: // The same as DFNT_UCHAR
	    return "8-bit unsigned character";
        case DFNT_INT8:
	    return "8-bit integer";
        case DFNT_UINT8:
	    return "8-bit unsigned integer";
        case DFNT_INT16:
	    return "16-bit integer";
        case DFNT_UINT16:
	    return "16-bit unsigned integer";
        case DFNT_INT32:
	    return "32-bit integer";
        case DFNT_UINT32:
	    return "32-bit unsigned integer";
        case DFNT_INT64:
	    return "64-bit integer";
        case DFNT_UINT64:
	    return "64-bit unsigned integer";
        case DFNT_FLOAT32:
	    return "32-bit floating-point";
        case DFNT_FLOAT64:
	    return "64-bit floating-point";
	default:
	    return "unknown type";
    }
}

/************************************************************************/
/*  Return the size of data type in bytes           	                */
/************************************************************************/

int HDF4Dataset::GetDataTypeSize( int32 iNumType ) const
{
    switch (iNumType)
    {
        case DFNT_CHAR8: // The same as DFNT_CHAR
	case DFNT_UCHAR8: // The same as DFNT_UCHAR
        case DFNT_INT8:
        case DFNT_UINT8:
	    return 1;
        case DFNT_INT16:
        case DFNT_UINT16:
	    return 2;
        case DFNT_INT32:
        case DFNT_UINT32:
        case DFNT_FLOAT32:
	    return 4;
        case DFNT_INT64:
        case DFNT_UINT64:
        case DFNT_FLOAT64:
	    return 8;
	default:
	    return 0;
    }
}

/************************************************************************/
/*  Convert value stored in the input buffer to double value.           */
/************************************************************************/

double HDF4Dataset::AnyTypeToDouble( int32 iNumType, void *pData ) const
{
    switch ( iNumType )
    {
        case DFNT_INT8:
            return (double)*(char *)pData;
        case DFNT_UINT8:
            return (double)*(unsigned char *)pData;
        case DFNT_INT16:
            return (double)*(short *)pData;
        case DFNT_UINT16:
            return (double)*(unsigned short *)pData;
        case DFNT_INT32:
            return (double)*(long *)pData;
        case DFNT_UINT32:
            return (double)*(unsigned long *)pData;
        case DFNT_INT64:
            return (double)*(char *)pData;
        case DFNT_UINT64:
            return (double)*(GIntBig *)pData;
        case DFNT_FLOAT32:
            return (double)*(float *)pData;
        case DFNT_FLOAT64:
            return (double)*(double *)pData;
        default:
            return 0.0;
    }
}

/************************************************************************/
/*         Tokenize HDF-EOS attributes.                                 */
/************************************************************************/

char **HDF4Dataset::HDF4EOSTokenizeAttrs( const char * pszString ) const

{
    const char  *pszDelimiters = " \t\n\r";
    char        **papszRetList = NULL;
    char        *pszToken;
    int         nTokenMax, nTokenLen;

    pszToken = (char *) CPLCalloc( 10, 1 );
    nTokenMax = 10;
    
    while( pszString != NULL && *pszString != '\0' )
    {
        int     bInString = FALSE, bInBracket = FALSE;

        nTokenLen = 0;
        
        // Try to find the next delimeter, marking end of token
        for( ; *pszString != '\0'; pszString++ )
        {

            // End if this is a delimeter skip it and break.
            if ( !bInBracket && !bInString
                 && strchr(pszDelimiters, *pszString) != NULL )
            {
                pszString++;
                break;
            }

            // Sometimes in bracketed tokens we may found a sort of
            // paragraph formatting. We will remove unneeded spaces and new
            // lines. 
            if ( bInBracket )
                if ( strchr("\r\n", *pszString) != NULL
                     || ( *pszString == ' '
                          && strchr(" \r\n", *(pszString - 1)) != NULL ) )
                continue;
            
            if ( *pszString == '"' )
            {
                if ( bInString )
                {
                    bInString = FALSE;
                    continue;
                }
                else
                {
                    bInString = TRUE;
                    continue;
                }
            }
            else if ( *pszString == '(' )
	    {
                bInBracket = TRUE;
		continue;
	    }
	    else if ( *pszString == ')' )
	    {
		bInBracket = FALSE;
		continue;
	    }

	    if( nTokenLen >= nTokenMax - 2 )
            {
                nTokenMax = nTokenMax * 2 + 10;
                pszToken = (char *) CPLRealloc( pszToken, nTokenMax );
            }

            pszToken[nTokenLen] = *pszString;
            nTokenLen++;
        }

        pszToken[nTokenLen] = '\0';

        if( pszToken[0] != '\0' )
        {
            papszRetList = CSLAddString( papszRetList, pszToken );
        }

        // If the last token is an empty token, then we have to catch
        // it now, otherwise we won't reenter the loop and it will be lost. 
        if ( *pszString == '\0' && strchr(pszDelimiters, *(pszString-1)) )
        {
            papszRetList = CSLAddString( papszRetList, "" );
        }
    }

    if( papszRetList == NULL )
        papszRetList = (char **) CPLCalloc( sizeof(char *), 1 );

    CPLFree( pszToken );

    return papszRetList;
}

/************************************************************************/
/*     Find object name and its value in HDF-EOS attributes.            */
/*     Function returns pointer to the string in list next behind       */
/*     recognized object.                                               */
/************************************************************************/

char **HDF4Dataset::HDF4EOSGetObject( char **papszAttrList, char **ppszAttrName,
                                      char **ppszAttrValue ) const
{
    int	    iCount, i, j;
    *ppszAttrName = NULL;
    *ppszAttrValue = NULL;

    iCount = CSLCount( papszAttrList );
    for ( i = 0; i < iCount - 2; i++ )
    {
	if ( EQUAL( papszAttrList[i], "OBJECT" ) )
	{
	    i += 2;
	    for ( j = 1; i + j < iCount - 2; j++ )
	    {
	        if ( EQUAL( papszAttrList[i + j], "END_OBJECT" ) ||
		     EQUAL( papszAttrList[i + j], "OBJECT" ) )
	            return &papszAttrList[i + j];
	        else if ( EQUAL( papszAttrList[i + j], "VALUE" ) )
	        {
		    *ppszAttrName = papszAttrList[i];
	            *ppszAttrValue = papszAttrList[i + j + 2];

		    return &papszAttrList[i + j + 2];
	        }
	    }
	}
    }

    return NULL;
}

/************************************************************************/
/*         Translate HDF4-EOS attributes in GDAL metadata items         */
/************************************************************************/

char** HDF4Dataset::TranslateHDF4EOSAttributes( int32 iHandle,
    int32 iAttribute, int32 nValues, char **papszMetadata ) const
{
    char	*pszData;
    
    pszData = (char *)CPLMalloc( (nValues + 1) * sizeof(char) );
    pszData[nValues] = '\0';
    SDreadattr( iHandle, iAttribute, pszData );
    // HDF4-EOS attributes has followed structure:
    // 
    // GROUP = <name>
    //   GROUPTYPE = <name>
    //
    //   GROUP = <name>
    //   
    //     OBJECT = <name>
    //       CLASS = <string>
    //       NUM_VAL = <number>
    //       VALUE = <string> or <number>
    //     END_OBJECT = <name>
    //     
    //     .......
    //     .......
    //     .......
    //     
    //   END_GROUP = <name>
    //   
    // .......
    // .......
    // .......
    //
    // END_GROUP = <name>
    // END
    //
    // Used data types:
    // <name>   --- unquoted character strings
    // <string> --- quoted character strings
    // <number> --- numerical value
    // If NUM_VAL != 1 then values combined in lists:
    // (<string>,<string>,...)
    // or
    // (<number>,<number>,...)
    //
    // Records within objects may follows in any order, objects may contains
    // other objects (and lacks VALUE record), groups contains other groups
    // and objects. Names of groups and objects are not unique and may repeat.
    // Objects may contains other types of records.
    //
    // We are interested in OBJECTS structures only.

    char *pszAttrName, *pszAttrValue;
    char *pszAddAttrName = NULL;
    char **papszAttrList, **papszAttrs;
    
    papszAttrList = HDF4EOSTokenizeAttrs( pszData );
    papszAttrs = papszAttrList;
    while ( papszAttrs )
    {
	papszAttrs =
	    HDF4EOSGetObject( papszAttrs, &pszAttrName, &pszAttrValue );
	if ( pszAttrName && pszAttrValue )
	{
	    // Now we should recognize special type of HDF EOS metastructures:
	    // ADDITIONALATTRIBUTENAME = <name>
	    // PARAMETERVALUE = <value>
	    if ( EQUAL( pszAttrName, "ADDITIONALATTRIBUTENAME" ) )
		pszAddAttrName = pszAttrValue;
	    else if ( pszAddAttrName && EQUAL( pszAttrName, "PARAMETERVALUE" ) )
	    {
		papszMetadata =
		    CSLAddNameValue( papszMetadata, pszAddAttrName, pszAttrValue );
		pszAddAttrName = NULL;
	    }
	    else
	    {
		papszMetadata =
		    CSLAddNameValue( papszMetadata, pszAttrName, pszAttrValue );
	    }
	}
    }
    
    CSLDestroy( papszAttrList );
    CPLFree( pszData );

    return papszMetadata;
}

/************************************************************************/
/*         Translate HDF4 attributes in GDAL metadata items             */
/************************************************************************/

char** HDF4Dataset::TranslateHDF4Attributes( int32 iHandle,
    int32 iAttribute, char *pszAttrName, int32 iNumType, int32 nValues,
    char **papszMetadata ) const
{
    void	*pData = NULL;
    char	*pszTemp = NULL;
    
/* -------------------------------------------------------------------- */
/*     Allocate a buffer to hold the attribute data.                    */
/* -------------------------------------------------------------------- */
    if ( iNumType == DFNT_CHAR8 || iNumType == DFNT_UCHAR8 )
        pData = CPLMalloc( (nValues + 1) * GetDataTypeSize(iNumType) );
    else
        pData = CPLMalloc( nValues * GetDataTypeSize(iNumType) );

/* -------------------------------------------------------------------- */
/*     Read the attribute data.                                         */
/* -------------------------------------------------------------------- */
    SDreadattr( iHandle, iAttribute, pData );
    if ( iNumType == DFNT_CHAR8 || iNumType == DFNT_UCHAR8 )
    {
        ((char *)pData)[nValues] = '\0';
        papszMetadata = CSLAddNameValue( papszMetadata, pszAttrName, 
                                         (const char *) pData );
    }
    else
    {
        pszTemp = SPrintArray( GetDataType(iNumType), pData, nValues, ", " );
        papszMetadata = CSLAddNameValue( papszMetadata, pszAttrName, pszTemp );
        if ( pszTemp )
	    CPLFree( pszTemp );
    }
    
    if ( pData )
	CPLFree( pData );

    return papszMetadata;
}

/************************************************************************/
/*                       ReadGlobalAttributes()                         */
/************************************************************************/

CPLErr HDF4Dataset::ReadGlobalAttributes( int32 iHandler )
{
    int32	iAttribute, nValues, iNumType, nAttributes;
    char	szAttrName[MAX_NC_NAME];

/* -------------------------------------------------------------------- */
/*     Obtain number of SDSs and global attributes in input file.       */
/* -------------------------------------------------------------------- */
    if ( SDfileinfo( hSD, &nDatasets, &nAttributes ) != 0 )
	return CE_Failure;

    // Loop through the all attributes
    for ( iAttribute = 0; iAttribute < nAttributes; iAttribute++ )
    {
        // Get information about the attribute. Note that the first
        // parameter is an SD interface identifier.
        SDattrinfo( iHandler, iAttribute, szAttrName, &iNumType, &nValues );

        if ( EQUALN( szAttrName, "coremetadata.", 13 )    ||
	     EQUALN( szAttrName, "archivemetadata.", 16 ) ||
	     EQUALN( szAttrName, "productmetadata.", 16 ) ||
             EQUALN( szAttrName, "badpixelinformation", 19 ) ||
	     EQUALN( szAttrName, "product_summary", 15 ) ||
	     EQUALN( szAttrName, "dem_specific", 12 ) ||
	     EQUALN( szAttrName, "bts_specific", 12 ) ||
	     EQUALN( szAttrName, "etse_specific", 13 ) ||
	     EQUALN( szAttrName, "dst_specific", 12 ) ||
	     EQUALN( szAttrName, "acv_specific", 12 ) ||
	     EQUALN( szAttrName, "act_specific", 12 ) ||
	     EQUALN( szAttrName, "etst_specific", 13 ) ||
	     EQUALN( szAttrName, "level_1_carryover", 17 ) )
        {
            bIsHDFEOS = 1;
            papszGlobalMetadata = TranslateHDF4EOSAttributes( iHandler,
		iAttribute, nValues, papszGlobalMetadata );
        }

        // Skip "StructMetadata.N" records. We will fetch information
        // from them using HDF-EOS API
	else if ( EQUALN( szAttrName, "structmetadata.", 15 ) )
        {
            bIsHDFEOS = 1;
            continue;
        }

        else
        {
	    papszGlobalMetadata = TranslateHDF4Attributes( iHandler,
		iAttribute, szAttrName,	iNumType, nValues, papszGlobalMetadata );
        }
    }

    return CE_None;
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *HDF4Dataset::Open( GDALOpenInfo * poOpenInfo )

{
    int32	i;
    
    if( poOpenInfo->fp == NULL )
        return NULL;

    // We have special routine in the HDF library for format checking!
    if ( !Hishdf(poOpenInfo->pszFilename) )
    {
        HEPclear();
        HEshutdown();
	return NULL;
    }

/* -------------------------------------------------------------------- */
/*      Try opening the dataset.                                        */
/* -------------------------------------------------------------------- */
    int32	hHDF4;
    
    hHDF4 = Hopen(poOpenInfo->pszFilename, DFACC_READ, 0);
    
    if( hHDF4 <= 0 )
        return( NULL );

/* -------------------------------------------------------------------- */
/*      Create a corresponding GDALDataset.                             */
/* -------------------------------------------------------------------- */
    HDF4Dataset 	*poDS;

    poDS = new HDF4Dataset();

    poDS->fp = poOpenInfo->fp;
    poOpenInfo->fp = NULL;
    
/* -------------------------------------------------------------------- */
/*          Open HDF SDS Interface.                                     */
/* -------------------------------------------------------------------- */
    poDS->hSD = SDstart( poOpenInfo->pszFilename, DFACC_READ );
    if ( poDS->hSD == -1 )
    {
        Hclose( hHDF4 );
	delete poDS;
        return NULL;
    }
   
/* -------------------------------------------------------------------- */
/*		Now read Global Attributes.				*/
/* -------------------------------------------------------------------- */
    if ( poDS->ReadGlobalAttributes( poDS->hSD ) != CE_None )
    {
        Hclose( hHDF4 );
	delete poDS;
        return NULL;
    }

    poDS->SetMetadata( poDS->papszGlobalMetadata, "" );

/* -------------------------------------------------------------------- */
/*		Determine type of file we read.				*/
/* -------------------------------------------------------------------- */
    const char	*pszValue;
    
    if ( (pszValue = CSLFetchNameValue(poDS->papszGlobalMetadata,
                                       "Signature"))
	 && EQUAL( pszValue, pszGDALSignature ) )
    {
	poDS->iSubdatasetType = GDAL_HDF4;
	poDS->pszSubdatasetType = "GDAL_HDF4";
    }

    else if ( (pszValue = CSLFetchNameValue(poDS->papszGlobalMetadata, "Title"))
	 && EQUAL( pszValue, "SeaWiFS Level-1A Data" ) )
    {
	poDS->iSubdatasetType = SEAWIFS_L1A;
	poDS->pszSubdatasetType = "SEAWIFS_L1A";
    }

    else if ( (pszValue = CSLFetchNameValue(poDS->papszGlobalMetadata, "Title"))
	&& EQUAL( pszValue, "SeaWiFS Level-2 Data" ) )
    {
	poDS->iSubdatasetType = SEAWIFS_L2;
	poDS->pszSubdatasetType = "SEAWIFS_L2";
    }

    else if ( (pszValue = CSLFetchNameValue(poDS->papszGlobalMetadata, "Title"))
	&& EQUAL( pszValue, "SeaWiFS Level-3 Standard Mapped Image" ) )
    {
	poDS->iSubdatasetType = SEAWIFS_L3;
	poDS->pszSubdatasetType = "SEAWIFS_L3";
    }

    else if ( (pszValue = CSLFetchNameValue(poDS->papszGlobalMetadata,
                                            "L1 File Generated By"))
	&& EQUALN( pszValue, "HYP version ", 12 ) )
    {
	poDS->iSubdatasetType = HYPERION_L1;
	poDS->pszSubdatasetType = "HYPERION_L1";
    }

    else
    {
	poDS->iSubdatasetType = UNKNOWN;
	poDS->pszSubdatasetType = "UNKNOWN";
    }

/* -------------------------------------------------------------------- */
/*  If we have HDF-EOS dataset, process it here.                  	*/
/* -------------------------------------------------------------------- */
    char	szName[VSNAMELENMAX + 1], szTemp[8192];
    char	*pszString;
    const char  *pszName;
    int		nCount;
    int32	aiDimSizes[MAX_VAR_DIMS];
    int32	iRank, iNumType, nAttrs;

    // Sometimes "HDFEOSVersion" attribute is not defined and we will
    // determine HDF-EOS datasets using other records
    // (see ReadGlobalAttributes() method).
    if ( poDS->bIsHDFEOS
         || CSLFetchNameValue(poDS->papszGlobalMetadata, "HDFEOSVersion") )
    {
        int32   nSubDatasets, nStrBufSize;

/* -------------------------------------------------------------------- */
/*  Close previously opened dataset.                                    */
/* -------------------------------------------------------------------- */
        Hclose( hHDF4 );

/* -------------------------------------------------------------------- */
/*  Process swath layers.                                           	*/
/* -------------------------------------------------------------------- */
        hHDF4 = SWopen( poOpenInfo->pszFilename, DFACC_READ );
        nSubDatasets = SWinqswath(poOpenInfo->pszFilename, NULL, &nStrBufSize);
#if DEBUG
        CPLDebug( "HDF4", "Number of HDF-EOS swaths: %d", nSubDatasets );
#endif
        if ( nSubDatasets > 0 && nStrBufSize > 0 )
        {
            char    *pszSwathList;
            char    **papszSwaths;

            pszSwathList = (char *)CPLMalloc( nStrBufSize + 1 );
            SWinqswath( poOpenInfo->pszFilename, pszSwathList, &nStrBufSize );
            pszSwathList[nStrBufSize] = '\0';

#if DEBUG
            CPLDebug( "HDF4", "List of HDF-EOS swaths: %s", pszSwathList );
#endif

            papszSwaths =
                CSLTokenizeString2( pszSwathList, ",", CSLT_HONOURSTRINGS );
            CPLFree( pszSwathList );

            if ( nSubDatasets != CSLCount(papszSwaths) )
            {
                CSLDestroy( papszSwaths );
                delete poDS;
                CPLDebug( "HDF4", "Can not parse list of HDF-EOS grids." );
                return NULL;
            }

            for ( i = 0; i < nSubDatasets; i++)
            {
                char    *pszFieldList;
                char    **papszFields;
                int32   *paiRank, *paiNumType;
                int32   hSW, nFields, j;

                hSW = SWattach( hHDF4, papszSwaths[i] );

                nFields = SWnentries( hSW, HDFE_NENTDFLD, &nStrBufSize );
                pszFieldList = (char *)CPLMalloc( nStrBufSize + 1 );
                paiRank = (int32 *)CPLMalloc( nFields * sizeof(int32) );
                paiNumType = (int32 *)CPLMalloc( nFields * sizeof(int32) );

                SWinqdatafields( hSW, pszFieldList, paiRank, paiNumType );

#if DEBUG
                {
                    char *pszTmp =
                        SPrintArray( GDT_UInt32, paiRank, nFields, "," );

                    CPLDebug( "HDF4", "Number of data fields in swath %ld: %d",
                              i, nFields );
                    CPLDebug( "HDF4", "List of data fields in swath %ld: %s",
                              i, pszFieldList );
                    CPLDebug( "HDF4", "Data fields ranks: %s", pszTmp );

                    CPLFree( pszTmp );
                }
#endif

                papszFields = CSLTokenizeString2( pszFieldList, ",",
                                                  CSLT_HONOURSTRINGS );
                
                for ( j = 0; j < nFields; j++ )
                {
                    SWfieldinfo( hSW, papszFields[j], &iRank, aiDimSizes,
                                 &iNumType, NULL );

                    if ( iRank < 2 )
                        continue;

	            // Add field to the list of GDAL subdatasets
                    nCount = CSLCount( poDS->papszSubDatasets ) / 2;
                    sprintf( szTemp, "SUBDATASET_%d_NAME", nCount + 1 );
	            // We will use the field index as an identificator.
                    poDS->papszSubDatasets =
                        CSLSetNameValue( poDS->papszSubDatasets, szTemp,
                                CPLSPrintf("HDF4_EOS:EOS_SWATH:\"%s\":%s:%s",
                                           poOpenInfo->pszFilename,
                                           papszSwaths[i], papszFields[j]) );

                    sprintf( szTemp, "SUBDATASET_%d_DESC", nCount + 1 );
                    pszString = SPrintArray( GDT_UInt32, aiDimSizes,
                                             iRank, "x" );
                    poDS->papszSubDatasets =
                        CSLSetNameValue( poDS->papszSubDatasets, szTemp,
                                         CPLSPrintf( "[%s] %s (%s)", pszString,
                                         papszFields[j],
                                         poDS->GetDataTypeName(iNumType) ) );
                    CPLFree( pszString );
                }

                CSLDestroy( papszFields );
                CPLFree( paiNumType );
                CPLFree( paiRank );
                CPLFree( pszFieldList );
                SWdetach( hSW );
            }

            CSLDestroy( papszSwaths );
            SWclose( hHDF4 );
        }

/* -------------------------------------------------------------------- */
/*  Process grid layers.                                           	*/
/* -------------------------------------------------------------------- */
        hHDF4 = GDopen( poOpenInfo->pszFilename, DFACC_READ );
        nSubDatasets = GDinqgrid( poOpenInfo->pszFilename, NULL, &nStrBufSize );
#if DEBUG
        CPLDebug( "HDF4", "Number of HDF-EOS grids: %d", nSubDatasets );
#endif
        if ( nSubDatasets > 0 && nStrBufSize > 0 )
        {
            char    *pszGridList;
            char    **papszGrids;

            pszGridList = (char *)CPLMalloc( nStrBufSize + 1 );
            GDinqgrid( poOpenInfo->pszFilename, pszGridList, &nStrBufSize );

#if DEBUG
            CPLDebug( "HDF4", "List of HDF-EOS grids: %s", pszGridList );
#endif

            papszGrids =
                CSLTokenizeString2( pszGridList, ",", CSLT_HONOURSTRINGS );
            CPLFree( pszGridList );

            if ( nSubDatasets != CSLCount(papszGrids) )
            {
                CSLDestroy( papszGrids );
                delete poDS;
                CPLDebug( "HDF4", "Can not parse list of HDF-EOS grids." );
                return NULL;
            }

            for ( i = 0; i < nSubDatasets; i++)
            {
                char    *pszFieldList;
                char    **papszFields;
                int32   *paiRank, *paiNumType;
                int32   hGD, nFields, j;

                hGD = GDattach( hHDF4, papszGrids[i] );

                nFields = GDnentries( hGD, HDFE_NENTDFLD, &nStrBufSize );
                pszFieldList = (char *)CPLMalloc( nStrBufSize + 1 );
                paiRank = (int32 *)CPLMalloc( nFields * sizeof(int32) );
                paiNumType = (int32 *)CPLMalloc( nFields * sizeof(int32) );

                GDinqfields( hGD, pszFieldList, paiRank, paiNumType );

#if DEBUG
                CPLDebug( "HDF4", "Number of fields in grid %ld: %d",
                          i, nFields );
                CPLDebug( "HDF4", "List of fields in grid %ld: %s",
                          i, pszFieldList );
                CPLDebug( "HDF4", "Fields ranks: %s",
                          SPrintArray( GDT_UInt32, paiRank, nFields, "," ) );
#endif

                papszFields = CSLTokenizeString2( pszFieldList, ",",
                                                  CSLT_HONOURSTRINGS );
                
                for ( j = 0; j < nFields; j++ )
                {
                    GDfieldinfo( hGD, papszFields[j], &iRank, aiDimSizes,
                                 &iNumType, NULL );

                    if ( iRank < 2 )
                        continue;

	            // Add field to the list of GDAL subdatasets
                    nCount = CSLCount( poDS->papszSubDatasets ) / 2;
                    sprintf( szTemp, "SUBDATASET_%d_NAME", nCount + 1 );
	            // We will use the field index as an identificator.
                    poDS->papszSubDatasets =
                        CSLSetNameValue(poDS->papszSubDatasets, szTemp,
                                CPLSPrintf( "HDF4_EOS:EOS_GRID:\"%s\":%s:%s",
                                            poOpenInfo->pszFilename,
                                            papszGrids[i], papszFields[j]));

                    sprintf( szTemp, "SUBDATASET_%d_DESC", nCount + 1 );
                    pszString = SPrintArray( GDT_UInt32, aiDimSizes,
                                             iRank, "x" );
                    poDS->papszSubDatasets =
                        CSLSetNameValue( poDS->papszSubDatasets, szTemp,
                                         CPLSPrintf("[%s] %s (%s)", pszString,
                                             papszFields[j],
                                             poDS->GetDataTypeName(iNumType)) );
                    CPLFree( pszString );
                }

                CSLDestroy( papszFields );
                CPLFree( paiNumType );
                CPLFree( paiRank );
                CPLFree( pszFieldList );
                GDdetach( hGD );
            }

            CSLDestroy( papszGrids );
            GDclose( hHDF4 );
        }
        GDclose( hHDF4 );
    }

    else
    {

/* -------------------------------------------------------------------- */
/*  Make a list of subdatasets from SDSs contained in input HDF file.	*/
/* -------------------------------------------------------------------- */
    int32	iSDS;

    for ( i = 0; i < poDS->nDatasets; i++ )
    {
	iSDS = SDselect( poDS->hSD, i );
	if ( SDgetinfo( iSDS, szName, &iRank, aiDimSizes, &iNumType, &nAttrs) != 0 )
	    return NULL;
	
	if ( iRank == 1 )		// Skip 1D datsets
		continue;

	// Do sort of known datasets. We will display only image bands
        if ( (poDS->iSubdatasetType == SEAWIFS_L1A ) &&
		  !EQUALN( szName, "l1a_data", 8 ) )
		continue;
        else
            pszName = szName;
	
	// Add datasets with multiple dimensions to the list of GDAL subdatasets
        nCount = CSLCount( poDS->papszSubDatasets ) / 2;
        sprintf( szTemp, "SUBDATASET_%d_NAME", nCount + 1 );
	// We will use SDS index as an identificator, because SDS names
	// are not unique. Filename also needed for further file opening
        poDS->papszSubDatasets = CSLSetNameValue(poDS->papszSubDatasets, szTemp, 
              CPLSPrintf( "HDF4_SDS:%s:\"%s\":%d", poDS->pszSubdatasetType,
			  poOpenInfo->pszFilename, i) );
        sprintf( szTemp, "SUBDATASET_%d_DESC", nCount + 1 );
	pszString = SPrintArray( GDT_UInt32, aiDimSizes, iRank, "x" );
        poDS->papszSubDatasets = CSLSetNameValue(poDS->papszSubDatasets, szTemp,
	    CPLSPrintf( "[%s] %s (%s)", pszString,
		        pszName, poDS->GetDataTypeName(iNumType)) );
	CPLFree( pszString );

	SDendaccess( iSDS );
    }
    SDend( poDS->hSD );

/* -------------------------------------------------------------------- */
/*              The same list builds for raster images.                 */
/* -------------------------------------------------------------------- */
    int32	iGR;
    int32	iInterlaceMode; 

    poDS->hGR = GRstart( hHDF4 );
    if ( poDS->hGR == -1 )
    {fprintf(stderr,"GRstart failed\n");}

    if ( GRfileinfo( poDS->hGR, &poDS->nImages, &nAttrs ) != 0 )
	return NULL;
   
    for ( i = 0; i < poDS->nImages; i++ )
    {
	iGR = GRselect( poDS->hGR, i );
	// iRank in GR interface has another meaning. It represents number
	// of samples per pixel. aiDimSizes has only two dimensions.
	if ( GRgetiminfo( iGR, szName, &iRank, &iNumType, &iInterlaceMode,
			  aiDimSizes, &nAttrs ) != 0 )
	    return NULL;
        nCount = CSLCount( poDS->papszSubDatasets ) / 2;
        sprintf( szTemp, "SUBDATASET_%d_NAME", nCount + 1 );
        poDS->papszSubDatasets = CSLSetNameValue(poDS->papszSubDatasets, szTemp,
              CPLSPrintf( "HDF4_GR:UNKNOWN:\"%s\":%d", poOpenInfo->pszFilename, i));
        sprintf( szTemp, "SUBDATASET_%d_DESC", nCount + 1 );
	pszString = SPrintArray( GDT_UInt32, aiDimSizes, 2, "x" );
        poDS->papszSubDatasets = CSLSetNameValue(poDS->papszSubDatasets, szTemp,
              CPLSPrintf( "[%sx%d] %s (%s)", pszString,
		          iRank, szName, poDS->GetDataTypeName(iNumType)) );
	CPLFree( pszString );

	GRendaccess( iGR );
    }
    GRend( poDS->hGR );

    Hclose( hHDF4 );

    }
    
    poDS->nRasterXSize = poDS->nRasterYSize = 512; // XXX: bogus values

/* -------------------------------------------------------------------- */
/*      If we have single subdataset only, open it immediately          */
/* -------------------------------------------------------------------- */
    if ( CSLCount( poDS->papszSubDatasets ) / 2 == 1 )
    {
	char *pszSDSName;
	pszSDSName = CPLStrdup( CSLFetchNameValue( poDS->papszSubDatasets,
				        "SUBDATASET_1_NAME" ));
	delete poDS;
	poDS = (HDF4Dataset *) GDALOpen( pszSDSName, GA_ReadOnly );
	CPLFree( pszSDSName );
    }

    return( poDS );
}

/************************************************************************/
/*                        GDALRegister_HDF4()				*/
/************************************************************************/

void GDALRegister_HDF4()

{
    GDALDriver	*poDriver;

    if( GDALGetDriverByName( "HDF4" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "HDF4" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "Hierarchical Data Format Release 4" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, 
                                   "frmt_hdf4.html" );

        poDriver->pfnOpen = HDF4Dataset::Open;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}

