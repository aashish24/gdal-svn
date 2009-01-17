/******************************************************************************
 * $Id: $
 *
 * Name:     georaster_rasterband.cpp
 * Project:  Oracle Spatial GeoRaster Driver
 * Purpose:  Implement GeoRasterRasterBand methods
 * Author:   Ivan Lucena [ivan.lucena@pmldnet.com]
 *
 ******************************************************************************
 * Copyright (c) 2008, Ivan Lucena
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files ( the "Software" ),
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

#include "georaster_priv.h"

//  ---------------------------------------------------------------------------
//                                                        GeoRasterRasterBand()
//  ---------------------------------------------------------------------------

GeoRasterRasterBand::GeoRasterRasterBand( GeoRasterDataset *poGDS,
                                          int nBand,
                                          int nLevel )
{
    poDS                = (GDALDataset*) poGDS;
    poGeoRaster         = poGDS->poGeoRaster;
    this->nBand         = nBand;
	this->eDataType		= OWGetDataType( poGeoRaster->pszCellDepth );
    poColorTable        = new GDALColorTable();
    poDefaultRAT        = NULL;
    pszVATName          = NULL;
    nRasterXSize        = poGeoRaster->nRasterColumns;
    nRasterYSize        = poGeoRaster->nRasterRows;
    nBlockXSize         = poGeoRaster->nColumnBlockSize;
    nBlockYSize         = poGeoRaster->nRowBlockSize;
    dfNoData            = 0.0;
    bValidStats         = false;
    nOverviewLevel      = nLevel;
    paphOverviewLinks   = NULL;
    nOverviewLinks      = 0;

    if( nLevel )
    {
        double dfScale  = pow( (double) 2.0, (double) nLevel );
        nRasterXSize    = (int) ceil( nRasterXSize / dfScale );
        nRasterYSize    = (int) ceil( nRasterYSize / dfScale );
    }
}

//  ---------------------------------------------------------------------------
//                                                       ~GeoRasterRasterBand()
//  ---------------------------------------------------------------------------

GeoRasterRasterBand::~GeoRasterRasterBand()
{
    delete poColorTable;
    delete poDefaultRAT;
    CPLFree( pszVATName );

    if( nOverviewLinks )
    {
        int i;

        for( i = 0; i < nOverviewLinks; i++ )
        {
            CPLFree( paphOverviewLinks[i] );
        }

        CPLFree( paphOverviewLinks );
    }

}

//  ---------------------------------------------------------------------------
//                                                                 IReadBlock()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterRasterBand::IReadBlock( int nBlockXOff,
                                        int nBlockYOff,
                                        void *pImage )
{
    if( poDS->GetAccess() == GA_Update )
    {
        return CE_None;
    }

    if( poGeoRaster->GetDataBlock( nBand,
                                   nOverviewLevel,
                                   nBlockXOff,
                                   nBlockYOff,
                                   pImage ) )
    {
        return CE_None;
    }
    else
    {
        CPLError(CE_Failure, CPLE_AppDefined,
            "Error reading GeoRaster ofsett X (%d) offset Y (%d) band (%d)",
            nBlockXOff, nBlockYOff, nBand );

        return CE_Failure;
    }
}

//  ---------------------------------------------------------------------------
//                                                                IWriteBlock()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterRasterBand::IWriteBlock( int nBlockXOff,
                                         int nBlockYOff,
                                         void *pImage )
{
    if( poGeoRaster->SetDataBlock( nBand,
                                   nOverviewLevel,
                                   nBlockXOff,
                                   nBlockYOff,
                                   pImage ) )
    {
        return CE_None;
    }
    else
    {
        CPLError(CE_Failure, CPLE_AppDefined,
            "Error writing GeoRaster ofsett X (%d) offset Y (%d) band (%d)",
            nBlockXOff, nBlockYOff, nBand );

        return CE_Failure;
    }
}
//  ---------------------------------------------------------------------------
//                                                     GetColorInterpretation()
//  ---------------------------------------------------------------------------

GDALColorInterp GeoRasterRasterBand::GetColorInterpretation()
{
    GeoRasterDataset* poGDS = (GeoRasterDataset*) poDS;

    if( eDataType == GDT_Byte && poGDS->nBands > 2 )
    {
        if( nBand == poGeoRaster->iDefaultRedBand )
        {
            return GCI_RedBand;
        }
        else if ( nBand == poGeoRaster->iDefaultGreenBand )
        {
            return GCI_GreenBand;
        }
        else if ( nBand == poGeoRaster->iDefaultBlueBand )
        {
            return GCI_BlueBand;
        }
        else
        {
            return GCI_GrayIndex;
        }
    }

    if( poGeoRaster->HasColorMap( nBand ) )
    {
        return GCI_PaletteIndex;
    }
    else
    {
        return GCI_GrayIndex;
    }
}

//  ---------------------------------------------------------------------------
//                                                              GetColorTable()
//  ---------------------------------------------------------------------------

GDALColorTable *GeoRasterRasterBand::GetColorTable()
{
    poGeoRaster->GetColorMap( nBand, poColorTable );

    if( poColorTable->GetColorEntryCount() == 0 )
    {
        return NULL;
    }

    return poColorTable;
}

//  ---------------------------------------------------------------------------
//                                                              SetColorTable()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterRasterBand::SetColorTable( GDALColorTable *poInColorTable )
{
    if( poInColorTable == NULL )
    {
        return CE_None;
    }

    if( poInColorTable->GetColorEntryCount() == 0 )
    {
        return CE_None;
    }

    delete poColorTable;

    poColorTable = poInColorTable->Clone();

    poGeoRaster->SetColorMap( nBand, poColorTable );

    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                                 GetMinimum()
//  ---------------------------------------------------------------------------

double GeoRasterRasterBand::GetMinimum( int *pbSuccess )
{
    *pbSuccess = (int) bValidStats;

    return dfMin;
}

//  ---------------------------------------------------------------------------
//                                                                 GetMaximum()
//  ---------------------------------------------------------------------------

double GeoRasterRasterBand::GetMaximum( int *pbSuccess )
{
    *pbSuccess = (int) bValidStats;

    return dfMax;
}

//  ---------------------------------------------------------------------------
//                                                              GetStatistics()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterRasterBand::GetStatistics( int bApproxOK, int bForce,
                                           double *pdfMin, double *pdfMax,
                                           double *pdfMean, double *pdfStdDev )
{
    (void) bForce;
    (void) bApproxOK;

    if( ! bValidStats )
    {
        bValidStats = poGeoRaster->GetStatistics( nBand,
                          dfMin, dfMax, dfMean, dfStdDev );
    }

    if( bValidStats )
    {
        *pdfMin     = dfMin;
        *pdfMax     = dfMax;
        *pdfMean    = dfMean;
        *pdfStdDev  = dfStdDev;

        return CE_None;
    }

    return CE_Failure;
}

//  ---------------------------------------------------------------------------
//                                                              SetStatistics()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterRasterBand::SetStatistics( double dfMin, double dfMax,
                                           double dfMean, double dfStdDev )
{
    dfMin       = dfMin;
    dfMax       = dfMax;
    dfMean      = dfMax;
    dfStdDev    = dfStdDev;
    bValidStats = true;

    poGeoRaster->SetStatistics( dfMin, dfMax, dfMean, dfStdDev, nBand );

    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                             GetNoDataValue()
//  ---------------------------------------------------------------------------

double GeoRasterRasterBand::GetNoDataValue( int *pbSuccess )
{
    if( pbSuccess )
    {
        *pbSuccess = (int) poGeoRaster->GetNoData( &dfNoData );
    }

    return dfNoData;
}

//  ---------------------------------------------------------------------------
//                                                             SetNoDataValue()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterRasterBand::SetNoDataValue( double dfNoDataValue )
{
    poGeoRaster->SetNoData( dfNoDataValue );

    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                              SetDefaultRAT()
//  ---------------------------------------------------------------------------

CPLErr GeoRasterRasterBand::SetDefaultRAT( const GDALRasterAttributeTable *poRAT )
{
    GeoRasterDataset* poGDS = (GeoRasterDataset*) poDS;

    if( ! poRAT )
    {
        return CE_Failure;
    }

    if( poDefaultRAT )
    {
        delete poDefaultRAT;
    }

    poDefaultRAT = poRAT->Clone();

    // ----------------------------------------------------------
    // Create VAT named based on RDT and RID (potential multi-reference)
    // ----------------------------------------------------------

    if( ! pszVATName )
    {
        pszVATName = CPLStrdup( CPLSPrintf(
            "RAT_%s_%d", poGeoRaster->pszDataTable, poGeoRaster->nRasterId ) );
    }

    // ----------------------------------------------------------
    // Format Table description
    // ----------------------------------------------------------

    char szDescription[OWTEXT];
    int  iCol = 0;

    strcpy( szDescription, "( ID NUMBER" );

    for( iCol = 0; iCol < poRAT->GetColumnCount(); iCol++ )
    {
        strcpy( szDescription, CPLSPrintf( "%s, %s",
            szDescription, poRAT->GetNameOfCol( iCol ) ) );

        if( poRAT->GetTypeOfCol( iCol ) == GFT_Integer )
        {
            strcpy( szDescription, CPLSPrintf( "%s NUMBER",
                szDescription ) );
        }
        if( poRAT->GetTypeOfCol( iCol ) == GFT_Real )
        {
            strcpy( szDescription, CPLSPrintf( "%s FLOAT",
                szDescription ) );
        }
        if( poRAT->GetTypeOfCol( iCol ) == GFT_String )
        {
            strcpy( szDescription, CPLSPrintf( "%s VARCHAR2(128)",
                szDescription ) );
        }
    }
    strcpy( szDescription, CPLSPrintf( "%s )", szDescription ) );

    // ----------------------------------------------------------
    // Create VAT table
    // ----------------------------------------------------------

    OWStatement* poStmt = poGeoRaster->poConnection->CreateStatement( CPLSPrintf(
        "DECLARE\n"
        "  TAB VARCHAR2(68)  := UPPER(:1);\n"
        "  CNT NUMBER        := 0;\n"
        "BEGIN\n"
        "  EXECUTE IMMEDIATE 'SELECT COUNT(*) FROM USER_TABLES\n"
        "    WHERE TABLE_NAME = :1' INTO CNT USING TAB;\n"
        "\n"
        "  IF NOT CNT = 0 THEN\n"
        "    EXECUTE IMMEDIATE 'DROP TABLE '||TAB||' PURGE';\n"
        "  END IF;\n"
        "\n"
        "  EXECUTE IMMEDIATE 'CREATE TABLE '||TAB||' %s';\n"
        "END;", szDescription ) );

    poStmt->Bind( pszVATName );

    if( ! poStmt->Execute() )
    {
        delete poStmt;
        CPLError( CE_Failure, CPLE_AppDefined, "Create VAT Table Error!" );
        return CE_Failure;
    }

    delete poStmt;

    // ----------------------------------------------------------
    // Create virtual file with INSERT instatements
    // ----------------------------------------------------------

    char* pszInserts = NULL;
    int nSize        = 0;
    int iEntry       = 0;
    int nEntryCount  = poRAT->GetRowCount();
    int nColunsCount = poRAT->GetColumnCount();
    int nBytesCount  = 0;

    char szInsert[OWTEXT];

    for( iEntry = 0; iEntry < nEntryCount; iEntry++ )
    {
        szInsert[0] = '\0';

        strcat( szInsert, CPLSPrintf ( "  INSERT INTO %s VALUES (%d", 
            pszVATName, iEntry ) );

        for( iCol = 0; iCol < nColunsCount; iCol++ )
        {
            if( poRAT->GetTypeOfCol( iCol ) == GFT_String )
            {
                strcat( szInsert, CPLSPrintf ( ", '%s'", 
                    poRAT->GetValueAsString( iEntry, iCol ) ) );

            }
            if( poRAT->GetTypeOfCol( iCol ) == GFT_Integer ||
                poRAT->GetTypeOfCol( iCol ) == GFT_Real )
            {
                strcat( szInsert, CPLSPrintf ( ", %s", 
                    poRAT->GetValueAsString( iEntry, iCol ) ) );

            }
        }
        strcat( szInsert, ");\n" );

        nBytesCount += strlen( szInsert );
        pszInserts = (char*) CPLRealloc( pszInserts, sizeof(char) * ( nBytesCount + 1 ));
        strcpy( &pszInserts[nSize], szInsert );
        nSize = nBytesCount;
    }

    // ----------------------------------------------------------
    // Insert Data to VAT
    // ----------------------------------------------------------

    poStmt = poGeoRaster->poConnection->CreateStatement( CPLSPrintf(
        "DECLARE\n"
        "  TAB  VARCHAR2(68)    := UPPER(:1);\n"
        "  GR1  SDO_GEORASTER   := NULL;\n"
        "BEGIN\n"
        "%s"
        "\n"
        "  SELECT %s INTO GR1 FROM %s T WHERE"
        "    T.%s.RasterDataTable = :rdt AND"
        "    T.%s.RasterId = :rid FOR UPDATE;\n"
        "  SDO_GEOR.setVAT(GR1, %d, '%s');\n"
        "  UPDATE %s T SET %s = GR1 WHERE"
        "    T.%s.RasterDataTable = :rdt AND"
        "    T.%s.RasterId = :rid;\n"
        "END;",
        pszInserts,
        poGeoRaster->pszColumn, poGeoRaster->pszTable,
        poGeoRaster->pszColumn, poGeoRaster->pszColumn,
        nBand, pszVATName,
        poGeoRaster->pszTable,  poGeoRaster->pszColumn,
        poGeoRaster->pszColumn, poGeoRaster->pszColumn  ) );

    poStmt->Bind( pszVATName );
    poStmt->BindName( (char*) ":rdt", poGDS->poGeoRaster->pszDataTable );
    poStmt->BindName( (char*) ":rid", &poGDS->poGeoRaster->nRasterId );

    if( ! poStmt->Execute() )
    {
        CPLFree( pszInserts );
        CPLError( CE_Failure, CPLE_AppDefined, "Insert/registering VAT Error!" );
        return CE_Failure;
    }

    CPLFree( pszInserts );

    delete poStmt;

    poGDS->poGeoRaster->SetVAT( nBand, pszVATName );

    return CE_None;
}

//  ---------------------------------------------------------------------------
//                                                              GetDefaultRAT()
//  ---------------------------------------------------------------------------

const GDALRasterAttributeTable *GeoRasterRasterBand::GetDefaultRAT()
{
    if( poDefaultRAT )
    {
        return poDefaultRAT->Clone();
    }
    else
    {
        poDefaultRAT = new GDALRasterAttributeTable();
    }

    GeoRasterDataset* poGDS = (GeoRasterDataset*) poDS;

    // ----------------------------------------------------------
    // Get the name of the VAT Table
    // ----------------------------------------------------------

    char* pszVATName = poGDS->poGeoRaster->GetVAT( nBand );

    if( pszVATName == NULL )
    {
        return NULL;
    }

    OCIParam* phDesc = NULL;

    phDesc = poGDS->poGeoRaster->poConnection->GetDescription( pszVATName );

    if( phDesc == NULL )
    {
        return NULL;
    }

    // ----------------------------------------------------------
    // Create the RAT and the SELECT statemet based on fields description
    // ----------------------------------------------------------

    int   iCol = 0;
    char  szField[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;

    char szColumnList[OWTEXT];
    szColumnList[0] = '\0';

    while( poGDS->poGeoRaster->poConnection->GetNextField(
                phDesc, iCol, szField, &hType, &nSize, &nPrecision, &nScale ) )
    {
        switch( hType )
        {
            case SQLT_FLT:
                poDefaultRAT->CreateColumn( szField, GFT_Real, GFU_Generic );
                break;
            case SQLT_NUM:
                if( nPrecision == 0 )
                {
                    poDefaultRAT->CreateColumn( szField, GFT_Integer,
                        GFU_Generic );
                }
                else
                {
                    poDefaultRAT->CreateColumn( szField, GFT_Real,
                        GFU_Generic );
                }
                break;
            case SQLT_CHR:
            case SQLT_AFC:
            case SQLT_DAT:
            case SQLT_DATE:
            case SQLT_TIMESTAMP:
            case SQLT_TIMESTAMP_TZ:
            case SQLT_TIMESTAMP_LTZ:
            case SQLT_TIME:
            case SQLT_TIME_TZ:
                    poDefaultRAT->CreateColumn( szField, GFT_String, 
                        GFU_Generic );
                break;
            default:
                CPLDebug("GEORASTER", "VAT (%s) Column (%s) type (%d) not supported"
                    "as GDAL RAT", pszVATName, szField, hType );
                continue;
        }
        strcpy( szColumnList, CPLSPrintf( "%s substr(%s,1,%d),",
            szColumnList, szField, MIN(nSize,OWNAME) ) );

        iCol++;
    }

    szColumnList[strlen(szColumnList) - 1] = '\0'; // remove the last comma

    // ----------------------------------------------------------
    // Read VAT and load RAT
    // ----------------------------------------------------------

    OWStatement* poStmt = NULL;

    poStmt = poGeoRaster->poConnection->CreateStatement( CPLSPrintf (
        "SELECT %s FROM %s", szColumnList, pszVATName ) );

    char** papszValue = (char**) CPLMalloc( sizeof(char**) * iCol );

    int i = 0;

    for( i = 0; i < iCol; i++ )
    {
        papszValue[i] = (char*) CPLMalloc( sizeof(char*) * OWNAME );
        poStmt->Define( papszValue[i] );
    }

    if( ! poStmt->Execute() )
    {
        CPLError( CE_Failure, CPLE_AppDefined, "Error reading VAT %s",
            pszVATName );
        return NULL;
    }

    int iRow = 0;

    while( poStmt->Fetch() )
    {
        for( i = 0; i < iCol; i++ )
        {
           poDefaultRAT->SetValue( iRow, i, papszValue[i] );
        }
        iRow++;
    }

    for( i = 0; i < iCol; i++ )
    {
        CPLFree( papszValue[i] );
    }
    CPLFree( papszValue );

    delete poStmt;

    CPLFree( pszVATName );

    return poDefaultRAT;
}

//  ---------------------------------------------------------------------------
//                                                           GetOverviewCount()
//  ---------------------------------------------------------------------------

int GeoRasterRasterBand::GetOverviewCount()
{
    return poGeoRaster->nPyramidMaxLevel;
}

//  ---------------------------------------------------------------------------
//                                                           GetOverviewCount()
//  ---------------------------------------------------------------------------

GDALRasterBand* GeoRasterRasterBand::GetOverview( int nLevel )
{
    GeoRasterDataset* poGDS = (GeoRasterDataset*) poDS;

    //  -----------------------------------------------------------------------
    //  Check if there is a previous stored overview link
    //  -----------------------------------------------------------------------

    int i = 0;

    for( i = 0; i < nOverviewLinks; i++ )
    {
        if( nLevel == paphOverviewLinks[i]->nLevel )
        {
            return paphOverviewLinks[i]->poOverview;
        }
    }

    //  -----------------------------------------------------------------------
    //  Creat RasterBand as Overview 
    //  -----------------------------------------------------------------------

    GeoRasterRasterBand* poOVR = NULL;
    poOVR       = new GeoRasterRasterBand( poGDS, nBand, ( nLevel + 1 ) );

    //  -----------------------------------------------------------------------
    //  Create a Overview link
    //  -----------------------------------------------------------------------

    OverviewLink* phOVRLink = NULL;
    phOVRLink   = (OverviewLink*) CPLMalloc( sizeof(OverviewLink) );
    phOVRLink->nLevel       = nLevel;
    phOVRLink->poOverview   = poOVR;

    //  -----------------------------------------------------------------------
    //  Save a Overview link
    //  -----------------------------------------------------------------------

    nOverviewLinks++;

    paphOverviewLinks = (OverviewLink**) CPLRealloc(
        paphOverviewLinks, sizeof(OverviewLink*) * nOverviewLinks );

    paphOverviewLinks[ nOverviewLinks - 1 ] = phOVRLink;

    return (GDALRasterBand*) poOVR;
}
