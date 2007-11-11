/******************************************************************************
 * $Id: adrgdataset.cpp
 *
 * Purpose:  ADRG reader
 * Author:   Even Rouault, even.rouault at mines-paris.org
 *
 ******************************************************************************
 * Copyright (c) 2007, Even Rouault
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

#include "gdal_pam.h"
#include "ogr_spatialref.h"
#include "cpl_string.h"
#include "iso8211.h"

#define N_ELEMENTS(x)  (sizeof(x)/sizeof(x[0]))

class ADRGDataset : public GDALPamDataset
{
    friend class ADRGRasterBand;

    FILE*        fdIMG;
    int*         TILEINDEX;
    int          offsetInIMG;
    int          NFC;
    int          NFL;
    double       LSO;
    double       PSO;
    int          ARV;
    int          BRV;

    ADRGDataset* poOverviewDS;
    
    /* For creation */
    int          bCreation;
    FILE*        fdGEN;
    FILE*        fdTHF;
    int          bGeoTransformValid;
    double       adfGeoTransform[6];
    int          nNextAvailableBlock;
    CPLString    baseFileName;

  public:
                 ADRGDataset();
    virtual     ~ADRGDataset();
    
    virtual const char *GetProjectionRef(void);
    virtual CPLErr GetGeoTransform( double * padfGeoTransform );
    virtual CPLErr SetGeoTransform( double * padfGeoTransform );

    static CPLString GetGENFromTHF(const char* fileName);
    static ADRGDataset* GetFromRecord(const char* fileName, DDFRecord * record, int isGIN);
    static GDALDataset *Open( GDALOpenInfo * );
    static GDALDataset *Create(const char* pszFilename, int nXSize, int nYSize,
                               int nBands, GDALDataType eType, char **papszOptions);
    
    static double GetLongitudeFromString(const char* str);
    static double GetLatitudeFromString(const char* str);
    
    void WriteGENFile();
    void WriteTHFFile();
};

/************************************************************************/
/* ==================================================================== */
/*                            ADRGRasterBand                             */
/* ==================================================================== */
/************************************************************************/

class ADRGRasterBand : public GDALPamRasterBand
{
    friend class ADRGDataset;

  public:
                            ADRGRasterBand( ADRGDataset *, int );

    virtual GDALColorInterp GetColorInterpretation();
    virtual CPLErr          IReadBlock( int, int, void * );
    virtual CPLErr          IWriteBlock( int, int, void * );

    virtual double          GetNoDataValue( int *pbSuccess = NULL );

//    virtual int             GetOverviewCount();
//    virtual GDALRasterBand* GetOverview(int i);
};


/************************************************************************/
/*                           ADRGRasterBand()                            */
/************************************************************************/

ADRGRasterBand::ADRGRasterBand( ADRGDataset *poDS, int nBand )

{
    this->poDS = poDS;
    this->nBand = nBand;
    
    eDataType = GDT_Byte;

    nBlockXSize = 128;
    nBlockYSize = 128;
}

#if 0

/* We have a problem with the overview. Its geo bounding box doesn't match */
/* exactly the one of the main image. We should handle the shift between */
/* the two top level corners... */

/************************************************************************/
/*                          GetOverviewCount()                          */
/************************************************************************/

int ADRGRasterBand::GetOverviewCount()

{
    ADRGDataset* poDS = (ADRGDataset*)this->poDS;
    if( poDS->poOverviewDS )
        return 1;
    else
        return GDALRasterBand::GetOverviewCount();
}

/************************************************************************/
/*                            GetOverview()                             */
/************************************************************************/

GDALRasterBand *ADRGRasterBand::GetOverview( int i )

{
    ADRGDataset* poDS = (ADRGDataset*)this->poDS;
    if( poDS->poOverviewDS )
    {
        if( i < 0 || i >= 1 )
            return NULL;
        else
            return poDS->poOverviewDS->GetRasterBand(nBand);
    }
    else
        return GDALRasterBand::GetOverview( i );
}
#endif

/************************************************************************/
/*                            GetNoDataValue()                          */
/************************************************************************/

double  ADRGRasterBand::GetNoDataValue( int *pbSuccess )
{
    if (pbSuccess)
        *pbSuccess = TRUE;

    return 0;
}

/************************************************************************/
/*                       GetColorInterpretation()                       */
/************************************************************************/

GDALColorInterp ADRGRasterBand::GetColorInterpretation()

{
    if( nBand == 1 )
        return GCI_RedBand;

    else if( nBand == 2 )
        return GCI_GreenBand;

    else 
        return GCI_BlueBand;
}

/************************************************************************/
/*                             IReadBlock()                             */
/************************************************************************/

CPLErr ADRGRasterBand::IReadBlock( int nBlockXOff, int nBlockYOff,
                                  void * pImage )

{
    ADRGDataset* poDS = (ADRGDataset*)this->poDS;
    int offset;
    int nBlock = nBlockYOff * poDS->NFC + nBlockXOff;
    if (nBlockXOff >= poDS->NFC || nBlockYOff >= poDS->NFL)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "nBlockXOff=%d, NFC=%d, nBlockYOff=%d, NFL=%d",
                 nBlockXOff, poDS->NFC, nBlockYOff, poDS->NFL);
        return CE_Failure;
    }
    CPLDebug("ADRG", "(%d,%d) -> nBlock = %d", nBlockXOff, nBlockYOff, nBlock);
    
    if (poDS->bCreation)
    {
        memset(pImage, 0, 128 * 128);
        return CE_None;
    }
    
    if (poDS->TILEINDEX)
    {
        if (poDS->TILEINDEX[nBlock] == 0)
        {
            memset(pImage, 0, 128 * 128);
            return CE_None;
        }
        offset = poDS->offsetInIMG + (poDS->TILEINDEX[nBlock] - 1) * 128 * 128 * 3 + (nBand - 1) * 128 * 128;
    }
    else
        offset = poDS->offsetInIMG + nBlock * 128 * 128 * 3 + (nBand - 1) * 128 * 128;
    
    if (VSIFSeekL(poDS->fdIMG, offset, SEEK_SET) != 0)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot seek to offset %d", offset);
        return CE_Failure;
    }
    if (VSIFReadL(pImage, 1, 128 * 128, poDS->fdIMG) != 128 * 128)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot read data at offset %d", offset);
        return CE_Failure;
    }
    
    return CE_None;
}

/************************************************************************/
/*                            IWriteBlock()                             */
/************************************************************************/

CPLErr ADRGRasterBand::IWriteBlock( int nBlockXOff, int nBlockYOff,
                                  void * pImage )

{
    ADRGDataset* poDS = (ADRGDataset*)this->poDS;
    int offset;
    int nBlock = nBlockYOff * poDS->NFC + nBlockXOff;
    if (nBlockXOff >= poDS->NFC || nBlockYOff >= poDS->NFL)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "nBlockXOff=%d, NFC=%d, nBlockYOff=%d, NFL=%d",
                 nBlockXOff, poDS->NFC, nBlockYOff, poDS->NFL);
        return CE_Failure;
    }
    CPLDebug("ADRG", "(%d,%d) -> nBlock = %d", nBlockXOff, nBlockYOff, nBlock);
    
    if (poDS->TILEINDEX[nBlock] == 0)
    {
        unsigned int i;
        int* pi = (int*)pImage;
        for(i=0;i<128*128 / sizeof(int);i++)
        {
            if (pi[i])
                break;
        }
        if (i == 128*128 / sizeof(int))
        {
            return CE_None;
        }

        poDS->TILEINDEX[nBlock] = poDS->nNextAvailableBlock ++;
    }

    offset = poDS->offsetInIMG + (poDS->TILEINDEX[nBlock] - 1) * 128 * 128 * 3 + (nBand - 1) * 128 * 128;

    if (VSIFSeekL(poDS->fdIMG, offset, SEEK_SET) != 0)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot seek to offset %d", offset);
        return CE_Failure;
    }
    if (VSIFWriteL(pImage, 1, 128 * 128, poDS->fdIMG) != 128 * 128)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Cannot read data at offset %d", offset);
        return CE_Failure;
    }

    return CE_None;
}

static unsigned int WriteSubFieldStr(FILE* fd, const char* pszStr, unsigned int size)
{
    char* str = (char*)CPLMalloc(size+1);
    memset(str, ' ', size);
    if (strlen(pszStr) > size)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "strlen(pszStr) > size");
        CPLFree(str);
        return size;
    }
    strcpy(str, pszStr);
    str[strlen(pszStr)] = ' ';
    VSIFWriteL(str, 1, size, fd);
    CPLFree(str);
    return size;
}

static unsigned int WriteSubFieldInt(FILE* fd, int val, unsigned int size)
{
    char* str = (char*)CPLMalloc(size+1);
    char formatStr[32];
    sprintf( formatStr, "%%0%dd", size);
    sprintf( str, formatStr, val);
    VSIFWriteL(str, 1, size, fd);
    CPLFree(str);
    return size;
}

static unsigned int WriteFieldTerminator(FILE* fd)
{
    char fieldTerminator = 30;
    VSIFWriteL(&fieldTerminator, 1, 1, fd);
    return 1;
}

static unsigned int WriteUnitTerminator(FILE* fd)
{
    char fieldTerminator = 31;
    VSIFWriteL(&fieldTerminator, 1, 1, fd);
    return 1;
}

static unsigned int WriteLongitude(FILE* fd, double val)
{
    char str[11+1];
    char sign = (val >= 0) ? '+' : '-';
    if (val < 0) val = -val;
    int ddd = (int)val;
    int mm = (int)((val - ddd) * 60);
    double ssdotss = ((val - ddd) * 60 - mm) * 60;
    sprintf(str, "%c%03d%02d%02.2f", sign, ddd, mm, ssdotss);
    VSIFWriteL(str, 1, 11, fd);
    return 11;
}

static unsigned int WriteLatitude(FILE* fd, double val)
{
    char str[10+1];
    char sign = (val >= 0) ? '+' : '-';
    if (val < 0) val = -val;
    int dd = (int)val;
    int mm = (int)((val - dd) * 60);
    double ssdotss = ((val - dd) * 60 - mm) * 60;
    sprintf(str, "%c%02d%02d%02.2f", sign, dd, mm, ssdotss);
    VSIFWriteL(str, 1, 10, fd);
    return 10;
}

static int BeginLeader(FILE* fd, int sizeFieldLength, int sizeFieldPos, int sizeFieldTag,
                       int nFields)
{
    int pos = VSIFTellL(fd);
    VSIFSeekL(fd, 24 + (sizeFieldLength + sizeFieldPos + sizeFieldTag) * nFields + 1, SEEK_CUR);
    return pos;
}

static void FinishWriteLeader(FILE* fd, int beginPos, int sizeFieldLength, int sizeFieldPos, int sizeFieldTag,
                             int nFields, int* sizeOfFields, const char** nameOfFields)
{
    int endPos = VSIFTellL(fd);
    VSIFSeekL(fd, beginPos, SEEK_SET);
    
    int nLeaderSize = 24;
    char szLeader[24+1];
    memset(szLeader, ' ', nLeaderSize);
    
    int i;
    int nDataSize = 0;
    int nFieldOffset = 0;
    for(i=0;i<nFields;i++)
        nDataSize += sizeOfFields[i];
    nFieldOffset = (sizeFieldLength + sizeFieldPos + sizeFieldTag) * nFields + 1;
    nDataSize += nFieldOffset;
    
    sprintf( szLeader+0, "%05d", (int) (nDataSize + nLeaderSize) );
    szLeader[5] = ' ';
    szLeader[6] = 'D';
    
    sprintf( szLeader + 12, "%05d", (int) (nFieldOffset + nLeaderSize) );
    szLeader[17] = ' ';

    szLeader[20] = (char) ('0' + sizeFieldLength);
    szLeader[21] = (char) ('0' + sizeFieldPos);
    szLeader[22] = '0';
    szLeader[23] = (char) ('0' + sizeFieldTag);

    VSIFWriteL(szLeader, 1, nLeaderSize, fd);
    
    int acc = 0;
    for(i=0;i<nFields;i++)
    {
        VSIFWriteL(nameOfFields[i], 1, sizeFieldTag, fd);
        WriteSubFieldInt(fd, sizeOfFields[i], sizeFieldLength);
        WriteSubFieldInt(fd, acc, sizeFieldPos);
        acc += sizeOfFields[i];
    }
    WriteFieldTerminator(fd);
    
    VSIFSeekL(fd, endPos, SEEK_SET);
}


static int BeginHeader(FILE* fd, int sizeFieldLength, int sizeFieldPos, int sizeFieldTag,
                       int nFields)
{
    int pos = VSIFTellL(fd);
    VSIFSeekL(fd, 24 + (sizeFieldLength + sizeFieldPos + sizeFieldTag) * nFields + 1, SEEK_CUR);
    return pos;
}

static void FinishWriteHeader(FILE* fd, int beginPos, int sizeFieldLength, int sizeFieldPos, int sizeFieldTag,
                             int nFields, int* sizeOfFields, const char** nameOfFields)
{
    int endPos = VSIFTellL(fd);
    VSIFSeekL(fd, beginPos, SEEK_SET);
    
    int nLeaderSize = 24;
    char szLeader[24+1];
    memset(szLeader, ' ', nLeaderSize);
    
    int i;
    int nDataSize = 0;
    int nFieldOffset = 0;
    for(i=0;i<nFields;i++)
        nDataSize += sizeOfFields[i];
    nFieldOffset = (sizeFieldLength + sizeFieldPos + sizeFieldTag) * nFields + 1;
    nDataSize += nFieldOffset;
    
    sprintf( szLeader+0, "%05d", (int) (nDataSize + nLeaderSize) );
    szLeader[5] = '2';
    szLeader[6] = 'L';
    
    szLeader[10] = '0';
    szLeader[11] = '6';
    sprintf( szLeader + 12, "%05d", (int) (nFieldOffset + nLeaderSize) );
    szLeader[17] = ' ';

    szLeader[20] = (char) ('0' + sizeFieldLength);
    szLeader[21] = (char) ('0' + sizeFieldPos);
    szLeader[22] = '0';
    szLeader[23] = (char) ('0' + sizeFieldTag);

    VSIFWriteL(szLeader, 1, nLeaderSize, fd);
    
    int acc = 0;
    for(i=0;i<nFields;i++)
    {
        VSIFWriteL(nameOfFields[i], 1, sizeFieldTag, fd);
        WriteSubFieldInt(fd, sizeOfFields[i], sizeFieldLength);
        WriteSubFieldInt(fd, acc, sizeFieldPos);
        acc += sizeOfFields[i];
    }
    WriteFieldTerminator(fd);
    
    VSIFSeekL(fd, endPos, SEEK_SET);
}

static int WriteFieldDecl(FILE* fd, char _data_struct_code , char _data_type_code, const char* _fieldName,
                           const char* _arrayDescr, const char* _formatControls)
{
    VSIFWriteL(&_data_struct_code, 1, 1, fd);
    VSIFWriteL(&_data_type_code, 1, 1, fd);
    if (_data_struct_code == ' ')
    {
        VSIFWriteL("    ", 1 , 4, fd);
    }
    else
    {
        VSIFWriteL("00;&", 1 , 4, fd);
    }
    int len = 6;
    VSIFWriteL(_fieldName, 1, strlen(_fieldName), fd);
    len += strlen(_fieldName);
    if (_arrayDescr[0])
    {
        len += WriteUnitTerminator(fd);
        VSIFWriteL(_arrayDescr, 1, strlen(_arrayDescr), fd);
        len += strlen(_arrayDescr);

        len += WriteUnitTerminator(fd);
        VSIFWriteL(_formatControls, 1, strlen(_formatControls), fd);
        len += strlen(_formatControls);
    }
    len += WriteFieldTerminator(fd);
    return len;
}


/************************************************************************/
/*                          ADRGDataset()                               */
/************************************************************************/

ADRGDataset::ADRGDataset()
{
    bCreation = FALSE;
    poOverviewDS = NULL;
    fdIMG = NULL;
    fdGEN = NULL;
    fdTHF = NULL;
    TILEINDEX = NULL;
}

/************************************************************************/
/*                          ~ADRGDataset()                              */
/************************************************************************/

ADRGDataset::~ADRGDataset()
{
    if (poOverviewDS)
    {
        delete poOverviewDS;
    }
    
    if (bCreation)
    {
        GDALPamDataset::FlushCache();
        
        /* Write header and padding of image */
        VSIFSeekL(fdIMG, 0, SEEK_SET);
        {
            FILE* fd = fdIMG;
            int nFields = 0;
            int sizeOfFields[] = { 0, 0, 0, 0 };
            const char* nameOfFields[] = { "000", "001", "PAD", "SCN" };
            int pos = BeginHeader(fd, 3, 4, 3, N_ELEMENTS(sizeOfFields));
    
            sizeOfFields[nFields++] += WriteFieldDecl(fd, ' ', ' ', "GEO_DATA_FILE", "", ""); /* 000 */
            sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "RECORD_ID_FIELD", /* 001 */
                                                    "RTY!RID",
                                                    "(A(3),A(2))");
            sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "PADDING_FIELD", /* PAD */
                                                    "PAD",
                                                    "(A)");
            sizeOfFields[nFields++] += WriteFieldDecl(fd, '2', '0', "PIXEL_FIELD", /* SCN */
                                                    "*PIX",
                                                    "(A(1))");
    
            FinishWriteHeader(fd, pos, 3, 4, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
            
            /* Write IMAGE_RECORD */
            {
                int nFields = 0;
                int sizeOfFields[] = {0, 0, 0};
                const char* nameOfFields[] = { "001", "PAD", "SCN" };
                int pos = BeginLeader(fd, 9, 9, 3, N_ELEMENTS(sizeOfFields));
        
                /* Field 001 */
                sizeOfFields[nFields] += WriteSubFieldStr(fd, "IMG", 3); /* RTY */
                sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
                sizeOfFields[nFields] += WriteFieldTerminator(fd);
                nFields++;
        
                /* Field PAD */
                int endPos = VSIFTellL(fd);
                char* pad = (char*)CPLMalloc(2047 - endPos);
                memset(pad, ' ', 2047 - endPos);
                VSIFWriteL(pad, 1, 2047 - endPos, fd);
                CPLFree(pad);
                WriteFieldTerminator(fd);
                sizeOfFields[nFields] += 2047 - endPos + 1;
                nFields++;
                
                /* Field SCN */
                sizeOfFields[nFields] = (nNextAvailableBlock - 1) * 128 * 128 * 3;
                nFields++;
        
                FinishWriteLeader(fd, pos, 9, 9, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
            }
        }

        /* Write terminal field terminator */
        int offset = offsetInIMG + (nNextAvailableBlock - 1) * 128 * 128 * 3;
        VSIFSeekL(fdIMG, offset, SEEK_SET);
        WriteFieldTerminator(fdIMG);
        
        WriteGENFile();
        WriteTHFFile();
    }

    if (fdIMG)
    {
        VSIFCloseL(fdIMG);
    }
    
    if (fdGEN)
    {
        VSIFCloseL(fdGEN);
    }
    if (fdTHF)
    {
        VSIFCloseL(fdTHF);
    }

    if (TILEINDEX)
    {
        delete [] TILEINDEX;
    }
}

/************************************************************************/
/*                        GetProjectionRef()                            */
/************************************************************************/

const char* ADRGDataset::GetProjectionRef()
{
    return( "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]" );
}

/************************************************************************/
/*                        GetGeoTransform()                             */
/************************************************************************/

CPLErr ADRGDataset::GetGeoTransform( double * padfGeoTransform)
{
    padfGeoTransform[0] = LSO;
    padfGeoTransform[1] = 360. / ARV;
    padfGeoTransform[2] = 0.0;
    padfGeoTransform[3] = PSO;
    padfGeoTransform[4] = 0.0;
    padfGeoTransform[5] = - 360. / BRV;

    return CE_None;
}

/************************************************************************/
/*                          SetGeoTransform()                           */
/************************************************************************/

CPLErr ADRGDataset::SetGeoTransform( double * padfGeoTransform )

{
    memcpy( adfGeoTransform, padfGeoTransform, sizeof(double)*6 );
    bGeoTransformValid = TRUE;
    return CE_None;
}

/************************************************************************/
/*                     GetLongitudeFromString()                         */
/************************************************************************/

double ADRGDataset::GetLongitudeFromString(const char* str)
{
    char ddd[3+1] = { 0 };
    char mm[2+1] = { 0 };
    char ssdotss[5+1] = { 0 };
    int sign = (str[0] == '+') ? 1 : - 1;
    str++;
    strncpy(ddd, str, 3);
    str+=3;
    strncpy(mm, str, 2);
    str+=2;
    strncpy(ssdotss, str, 5);
    return sign * (atof(ddd) + atof(mm) / 60 + atof(ssdotss) / 3600);
}

/************************************************************************/
/*                      GetLatitudeFromString()                         */
/************************************************************************/

double ADRGDataset::GetLatitudeFromString(const char* str)
{
    char ddd[2+1] = { 0 };
    char mm[2+1] = { 0 };
    char ssdotss[5+1] = { 0 };
    int sign = (str[0] == '+') ? 1 : - 1;
    str++;
    strncpy(ddd, str, 2);
    str+=2;
    strncpy(mm, str, 2);
    str+=2;
    strncpy(ssdotss, str, 5);
    return sign * (atof(ddd) + atof(mm) / 60 + atof(ssdotss) / 3600);
}


/************************************************************************/
/*                           GetFromRecord()                            */
/************************************************************************/

ADRGDataset* ADRGDataset::GetFromRecord(const char* fileName, DDFRecord * record, int isGIN)
{
    int SCA;
    int ZNA;
    double PSP;
    int ARV;
    int BRV;
    double LSO;
    double PSO;
    int NFL;
    int NFC;
    CPLString BAD;
    int TIF;
    int* TILEINDEX = NULL;
    int i;

    DDFField* field;
    DDFFieldDefn *fieldDefn;
    DDFSubfieldDefn* subfieldDefn;

    
    field = record->GetField(2);
    fieldDefn = field->GetFieldDefn();
    
    if (isGIN)
    {
        if (!(strcmp(fieldDefn->GetName(), "GEN") == 0 &&
                fieldDefn->GetSubfieldCount() == 21))
        {
            return NULL;
        }
        
        subfieldDefn = fieldDefn->GetSubfield(0);
        if (!(strcmp(subfieldDefn->GetName(), "STR") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I' &&
                subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 1, NULL) == 3))
        {
            return NULL;
        }
        
        subfieldDefn = fieldDefn->GetSubfield(12);
        if (!(strcmp(subfieldDefn->GetName(), "SCA") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I'))
        {
            return NULL;
        }
        
        SCA = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 9, NULL);
        CPLDebug("ADRG", "SCA=%d", SCA);
        
        subfieldDefn = fieldDefn->GetSubfield(13);
        if (!(strcmp(subfieldDefn->GetName(), "ZNA") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I'))
        {
            return NULL;
        }
        
        ZNA = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 2, NULL);
        CPLDebug("ADRG", "ZNA=%d", ZNA);
        
        subfieldDefn = fieldDefn->GetSubfield(14);
        if (!(strcmp(subfieldDefn->GetName(), "PSP") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'R'))
        {
            return NULL;
        }
        
        PSP = subfieldDefn->ExtractFloatData(field->GetSubfieldData(subfieldDefn), 5, NULL);
        CPLDebug("ADRG", "PSP=%f", PSP);
        if (PSP != 100)
        {
            return NULL;
        }
        
        subfieldDefn = fieldDefn->GetSubfield(16);
        if (!(strcmp(subfieldDefn->GetName(), "ARV") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I'))
        {
            return NULL;
        }
        
        ARV = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 8, NULL);
        CPLDebug("ADRG", "ARV=%d", ARV);
        
        subfieldDefn = fieldDefn->GetSubfield(17);
        if (!(strcmp(subfieldDefn->GetName(), "BRV") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I'))
        {
            return NULL;
        }
        
        BRV = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 8, NULL);
        CPLDebug("ADRG", "BRV=%d", BRV);
        
        
        subfieldDefn = fieldDefn->GetSubfield(18);
        if (!(strcmp(subfieldDefn->GetName(), "LSO") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'A'))
        {
            return NULL;
        }
        
        LSO = GetLongitudeFromString(subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 11, NULL));
        CPLDebug("ADRG", "LSO=%f", LSO);
        
        subfieldDefn = fieldDefn->GetSubfield(19);
        if (!(strcmp(subfieldDefn->GetName(), "PSO") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'A'))
        {
            return NULL;
        }
        
        PSO = GetLatitudeFromString(subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 10, NULL));
        CPLDebug("ADRG", "PSO=%f", PSO);
    }
    else
    {
        if (!(strcmp(fieldDefn->GetName(), "OVI") == 0 &&
                fieldDefn->GetSubfieldCount() == 5))
        {
            return NULL;
        }
        
        subfieldDefn = fieldDefn->GetSubfield(0);
        if (!(strcmp(subfieldDefn->GetName(), "STR") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I' &&
                subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 1, NULL) == 3))
        {
            return NULL;
        }
        
        subfieldDefn = fieldDefn->GetSubfield(1);
        if (!(strcmp(subfieldDefn->GetName(), "ARV") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I'))
        {
            return NULL;
        }
        
        ARV = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 8, NULL);
        CPLDebug("ADRG", "ARV=%d", ARV);
        
        subfieldDefn = fieldDefn->GetSubfield(2);
        if (!(strcmp(subfieldDefn->GetName(), "BRV") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'I'))
        {
            return NULL;
        }
        
        BRV = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 8, NULL);
        CPLDebug("ADRG", "BRV=%d", BRV);
        
        
        subfieldDefn = fieldDefn->GetSubfield(3);
        if (!(strcmp(subfieldDefn->GetName(), "LSO") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'A'))
        {
            return NULL;
        }
        
        LSO = GetLongitudeFromString(subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 11, NULL));
        CPLDebug("ADRG", "LSO=%f", LSO);
        
        subfieldDefn = fieldDefn->GetSubfield(4);
        if (!(strcmp(subfieldDefn->GetName(), "PSO") == 0 &&
                (subfieldDefn->GetFormat())[0] == 'A'))
        {
            return NULL;
        }
        
        PSO = GetLatitudeFromString(subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 10, NULL));
        CPLDebug("ADRG", "PSO=%f", PSO);
    }
    
    field = record->GetField(3);
    fieldDefn = field->GetFieldDefn();
    
    if (!(strcmp(fieldDefn->GetName(), "SPR") == 0 &&
            fieldDefn->GetSubfieldCount() == 15))
    {
        return NULL;
    }
    
    subfieldDefn = fieldDefn->GetSubfield(4);
    if (!(strcmp(subfieldDefn->GetName(), "NFL") == 0 &&
            (subfieldDefn->GetFormat())[0] == 'I'))
    {
        return NULL;
    }
    
    NFL = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 3, NULL);
    CPLDebug("ADRG", "NFL=%d", NFL);
    
    subfieldDefn = fieldDefn->GetSubfield(5);
    if (!(strcmp(subfieldDefn->GetName(), "NFC") == 0 &&
            (subfieldDefn->GetFormat())[0] == 'I'))
    {
        return NULL;
    }
    
    NFC = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 3, NULL);
    CPLDebug("ADRG", "NFC=%d", NFC);
    
    subfieldDefn = fieldDefn->GetSubfield(6);
    if (!(strcmp(subfieldDefn->GetName(), "PNC") == 0 &&
            (subfieldDefn->GetFormat())[0] == 'I'))
    {
        return NULL;
    }
    
    int PNC = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 6, NULL);
    CPLDebug("ADRG", "PNC=%d", PNC);
    if (PNC != 128)
    {
        return NULL;
    }
    
    subfieldDefn = fieldDefn->GetSubfield(7);
    if (!(strcmp(subfieldDefn->GetName(), "PNL") == 0 &&
            (subfieldDefn->GetFormat())[0] == 'I'))
    {
        return NULL;
    }
    
    int PNL = subfieldDefn->ExtractIntData(field->GetSubfieldData(subfieldDefn), 6, NULL);
    CPLDebug("ADRG", "PNL=%d", PNL);
    if (PNL != 128)
    {
        return NULL;
    }
    
    subfieldDefn = fieldDefn->GetSubfield(13);
    if (!(strcmp(subfieldDefn->GetName(), "BAD") == 0 &&
            (subfieldDefn->GetFormat())[0] == 'A'))
    {
        return NULL;
    }
    
    BAD = subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 12, NULL);
    {
        char* c = (char*) strchr(BAD, ' ');
        if (c)
            *c = 0;
    }
    CPLDebug("ADRG", "BAD=%s", (const char*)BAD);
    
    subfieldDefn = fieldDefn->GetSubfield(14);
    if (!(strcmp(subfieldDefn->GetName(), "TIF") == 0 &&
            (subfieldDefn->GetFormat())[0] == 'A'))
    {
        return NULL;
    }
    
    TIF = subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 1, NULL)[0] == 'Y';
    CPLDebug("ADRG", "TIF=%d", TIF);
    
    if (TIF)
    {
        if (record->GetFieldCount() != 6)
        {
            return NULL;
        }
        
        field = record->GetField(5);
        fieldDefn = field->GetFieldDefn();
    
        if (!(strcmp(fieldDefn->GetName(), "TIM") == 0))
        {
            return NULL;
        }
        
        if (field->GetDataSize() != 5 * NFL * NFC + 1)
        {
            return NULL;
        }
    
        TILEINDEX = new int [NFL * NFC];
        const char* ptr = field->GetData();
        char offset[5+1]={0};
        for(i=0;i<NFL*NFC;i++)
        {
            strncpy(offset, ptr, 5);
            ptr += 5;
            TILEINDEX[i] = atoi(offset);
            //CPLDebug("ADRG", "TSI[%d]=%d", i, TILEINDEX[i]);
        }
    }
    
    CPLString dirname = CPLGetDirname(fileName);
    CPLString imgname = CPLFormFilename(dirname, (const char*)BAD, NULL);

    FILE* fdIMG = VSIFOpenL((const char*)imgname, "rb");
    if (fdIMG == NULL)
    {
        char** dirContent = VSIReadDir((const char*)dirname);
        char** ptr = dirContent;
        if (ptr)
        {
            while(*ptr)
            {
                if (EQUAL(*ptr, (const char*)BAD))
                {
                    imgname = CPLFormFilename(dirname, *ptr, NULL);
                    fdIMG = VSIFOpenL((const char*)imgname, "rb");
                    if (fdIMG)
                        break;
                }
                ptr ++;
            }
        }
        CSLDestroy(dirContent);
        
        if (fdIMG == NULL)
        {
            CPLError(CE_Failure, CPLE_AppDefined, "Cannot open %s\n", (const char*)imgname);
            return NULL;
        }
    }
    
    if (ZNA == 9 || ZNA == 18)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "Polar cases are not handled by ADRG driver");
        return NULL;
    }
    
    
    /* Skip ISO8211 header of IMG file */
    int offsetInIMG = 0;
    char c;
    char recordName[3];
    if (VSIFReadL(&c, 1, 1, fdIMG) != 1)
    {
        return NULL;
    }
    while (!VSIFEofL(fdIMG))
    {
        if (c == 30)
        {
            if (VSIFReadL(recordName, 1, 3, fdIMG) != 3)
            {
                return NULL;
            }
            offsetInIMG += 3;
            if (strncmp(recordName,"IMG",3) == 0)
            {
                offsetInIMG += 4;
                if (VSIFSeekL(fdIMG,3,SEEK_CUR) != 0)
                {
                    return NULL;
                }
                if (VSIFReadL(&c, 1, 1, fdIMG) != 1)
                {
                    return NULL;
                }
                while(c ==' ')
                {
                    offsetInIMG ++;
                    if (VSIFReadL(&c, 1, 1, fdIMG) != 1)
                    {
                        return NULL;
                    }
                }
                offsetInIMG ++;
                break;
            }
        }

        offsetInIMG ++;
        if (VSIFReadL(&c, 1, 1, fdIMG) != 1)
        {
            return NULL;
        }
    }
    
    if (VSIFEofL(fdIMG))
    {
        return NULL;
    }
    
    CPLDebug("ADRG", "Img offset data = %d", offsetInIMG);
    
    ADRGDataset* poDS = new ADRGDataset();
    
    poDS->NFC = NFC;
    poDS->NFL = NFL;
    poDS->nRasterXSize = NFC * 128;
    poDS->nRasterYSize = NFL * 128;
    poDS->LSO = LSO;
    poDS->PSO = PSO;
    poDS->ARV = ARV;
    poDS->BRV = BRV;
    poDS->TILEINDEX = TILEINDEX;
    poDS->fdIMG = fdIMG;
    poDS->offsetInIMG = offsetInIMG;
    poDS->poOverviewDS = NULL;
    
    if (isGIN)
    {
        char pszValue[32];
        sprintf(pszValue, "%d", SCA);
        poDS->SetMetadataItem( "ADRG_SCA", pszValue );
    }
    
    poDS->nBands = 3;
    for( i = 0; i < poDS->nBands; i++ )
        poDS->SetBand( i+1, new ADRGRasterBand( poDS, i+1 ) );

    return poDS;
}

/************************************************************************/
/*                          GetGENFromTHF()                             */
/************************************************************************/

CPLString ADRGDataset::GetGENFromTHF(const char* fileName)
{
    DDFModule module;
    DDFRecord * record;
    DDFField* field;
    DDFFieldDefn *fieldDefn;
    DDFSubfieldDefn* subfieldDefn;
    int i;

    if (!module.Open((const char*)fileName, TRUE))
        return "";
    
    while ((record = module.ReadRecord()) != NULL)
    {
        if (record->GetFieldCount() >= 2)
        {
            field = record->GetField(0);
            fieldDefn = field->GetFieldDefn();
            if (!(strcmp(fieldDefn->GetName(), "001") == 0 &&
                fieldDefn->GetSubfieldCount() == 2))
            {
                continue;
            }

            subfieldDefn = fieldDefn->GetSubfield(0);
            if (!(strcmp(subfieldDefn->GetName(), "RTY") == 0 &&
                  (subfieldDefn->GetFormat())[0] == 'A'))
            {
                continue;
            }
            
            const char* RTY = subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 3, NULL);
            if (! ( strcmp(RTY, "TFN") == 0 ))
            {
                continue;
            }
            
            for (i = 1; i < record->GetFieldCount() ; i++)
            {
                field = record->GetField(i);
                fieldDefn = field->GetFieldDefn();
                
                if (!(strcmp(fieldDefn->GetName(), "VFF") == 0 &&
                      fieldDefn->GetSubfieldCount() == 1))
                {
                    continue;
                }
            
                subfieldDefn = fieldDefn->GetSubfield(0);
                if (!(strcmp(subfieldDefn->GetName(), "VFF") == 0 &&
                    (subfieldDefn->GetFormat())[0] == 'A'))
                {
                    continue;
                }
                
                CPLString subFileName(subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 300, NULL));
                char* c = (char*) strchr(subFileName, ' ');
                if (c)
                    *c = 0;
                if (EQUAL(CPLGetExtension((const char*)subFileName), "GEN"))
                {
                    CPLDebug("ADRG", "Found GEN file in THF : %s", (const char*)subFileName);
                    CPLString GENFileName(CPLGetDirname(fileName));
                    char** tokens = CSLTokenizeString2( subFileName, "/\"", 0);
                    char** ptr = tokens;
                    if (ptr == NULL)
                        continue;
                    while(*ptr)
                    {
                        char** dirContent = VSIReadDir((const char*)GENFileName);
                        char** ptrDir = dirContent;
                        if (ptrDir)
                        {
                            while(*ptrDir)
                            {
                                if (EQUAL(*ptrDir, *ptr))
                                {
                                    GENFileName = CPLFormFilename((const char*)GENFileName, *ptrDir, NULL);
                                    CPLDebug("ADRG", "Building GEN full file name : %s", (const char*)GENFileName);
                                    break;
                                }
                                ptrDir ++;
                            }
                        }
                        if (ptrDir == NULL)
                            break;
                        CSLDestroy(dirContent);
                        ptr++;
                    }
                    int isNameValid = *ptr == NULL;
                    CSLDestroy(tokens);
                    if (isNameValid)
                        return GENFileName;
                }
            }
        }
    }
    return "";
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *ADRGDataset::Open( GDALOpenInfo * poOpenInfo )
{
    DDFModule module;
    DDFRecord * record;
    DDFField* field;
    DDFFieldDefn *fieldDefn;
    DDFSubfieldDefn* subfieldDefn;
    ADRGDataset* overviewDS = NULL;
    CPLString fileName(poOpenInfo->pszFilename);
    CPLString NAM;
    
    if (EQUAL(CPLGetExtension((const char*)fileName), "thf"))
    {
        fileName = GetGENFromTHF((const char*)fileName);
    }
    
    if (!EQUAL(CPLGetExtension((const char*)fileName), "gen"))
        return NULL;

    if (!module.Open((const char*)fileName, TRUE))
        return NULL;

    while ((record = module.ReadRecord()) != NULL)
    {
        if (record->GetFieldCount() >= 5)
        {
            field = record->GetField(0);
            fieldDefn = field->GetFieldDefn();
            if (!(strcmp(fieldDefn->GetName(), "001") == 0 &&
                fieldDefn->GetSubfieldCount() == 2))
            {
                continue;
            }

            subfieldDefn = fieldDefn->GetSubfield(0);
            if (!(strcmp(subfieldDefn->GetName(), "RTY") == 0 &&
                  (subfieldDefn->GetFormat())[0] == 'A'))
            {
                continue;
            }
            
            const char* RTY = subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 3, NULL);
            if (! ( strcmp(RTY, "GIN") == 0 || strcmp(RTY, "OVV") == 0 ))
            {
                continue;
            }
            
            int isGIN = strcmp(RTY, "GIN") == 0;

            field = record->GetField(1);
            fieldDefn = field->GetFieldDefn();

            if (!(strcmp(fieldDefn->GetName(), "DSI") == 0 &&
                  fieldDefn->GetSubfieldCount() == 2))
            {
                continue;
            }
            
            subfieldDefn = fieldDefn->GetSubfield(0);
            if (!(strcmp(subfieldDefn->GetName(), "PRT") == 0 &&
                 (subfieldDefn->GetFormat())[0] == 'A' &&
                 strcmp(subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 4, NULL), "ADRG") == 0))
            {
                continue;
            }
            
            subfieldDefn = fieldDefn->GetSubfield(1);
            if (!(strcmp(subfieldDefn->GetName(), "NAM") == 0 &&
                  (subfieldDefn->GetFormat())[0] == 'A'))
            {
                continue;
            }

            NAM = subfieldDefn->ExtractStringData(field->GetSubfieldData(subfieldDefn), 8, NULL);
            CPLDebug("ADRG", "NAM=%s", (const char*)NAM);

            if (isGIN)
            {
                ADRGDataset* poDS = GetFromRecord((const char*)fileName, record, TRUE);
                if (poDS)
                {
                    poDS->SetMetadataItem( "ADRG_NAM", (const char*)NAM );

                    poDS->poOverviewDS = overviewDS;

                    /* -------------------------------------------------------------------- */
                    /*      Check for external overviews.                                   */
                    /* -------------------------------------------------------------------- */
                    poDS->oOvManager.Initialize( poDS, poOpenInfo->pszFilename );

                    /* -------------------------------------------------------------------- */
                    /*      Initialize any PAM information.                                 */
                    /* -------------------------------------------------------------------- */
                    //poDS->SetDescription( poOpenInfo->pszFilename );
                    //poDS->TryLoadXML();

                }
                else if (overviewDS)
                {
                    delete overviewDS;
                }
                return poDS;
            }
#if 0
            else
                overviewDS = GetFromRecord((const char*)fileName, record, FALSE);
#endif

        }
    }
    
    if (overviewDS)
        delete overviewDS;
    
    return NULL;
}

/************************************************************************/
/*                                Open()                                */
/************************************************************************/

GDALDataset *ADRGDataset::Create(const char* pszFilename, int nXSize, int nYSize,
                                 int nBands, GDALDataType eType, char **papszOptions)
{
    int i;

    if( eType != GDT_Byte)
    {
        CPLError( CE_Failure, CPLE_AppDefined,
              "Attempt to create ADRG dataset with an illegal\n"
              "data type (%s), only Byte supported by the format.\n",
              GDALGetDataTypeName(eType) );

        return NULL;
    }

    if( nBands != 3 )
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                  "ADRG driver doesn't support %d bands. Must be 3 (rgb) bands.\n",
                  nBands );
        return NULL;
    }

    if(nXSize < 1 || nYSize < 1)
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                "Specified pixel dimensions (% d x %d) are bad.\n",
                nXSize, nYSize );
    }
    
    if (!EQUAL(CPLGetExtension(pszFilename), "gen"))
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                "Invalid filename. Must be ABCDEF01.GEN\n");
        return NULL;
    }

    CPLString baseFileName(CPLGetBasename(pszFilename));
    if (strlen(baseFileName) != 8 || baseFileName[6] != '0' || baseFileName[7] != '1')
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                "Invalid filename. Must be xxxxxx01.GEN where x is between A and Z\n");
        return NULL;
    }
    
    for(i=0;i<6;i++)
    {
        if (!(baseFileName[i] >= 'A' && baseFileName[i] <= 'Z'))
        {
            CPLError( CE_Failure, CPLE_NotSupported,
                "Invalid filename. Must be xxxxxx01.GEN where x is between A and Z\n");
            return NULL;
        }
    }

    FILE* fdGEN = VSIFOpenL((const char*)pszFilename, "wb");
    if (fdGEN == NULL)
    {
        CPLError( CE_Failure, CPLE_NotSupported,
                "Cannot create GEN file.\n");
        return NULL;
    }
    
    CPLString dirname(CPLGetDirname(pszFilename));
    FILE* fdTHF = VSIFOpenL(CPLFormFilename((const char*)dirname, "TRANSH01.THF", NULL), "wb");
    if (fdTHF == NULL)
    {
        VSIFCloseL(fdGEN);
        CPLError( CE_Failure, CPLE_NotSupported,
                "Cannot create THF file.\n");
        return NULL;
    }
    
    CPLString imgFilename = CPLResetExtension(pszFilename, "IMG");
    FILE* fdIMG = VSIFOpenL((const char*)imgFilename, "wb");
    if (fdIMG == NULL)
    {
        VSIFCloseL(fdGEN);
        VSIFCloseL(fdTHF);
        CPLError( CE_Failure, CPLE_NotSupported,
                "Cannot create image file.\n");
        return NULL;
    }
    
    ADRGDataset* poDS = new ADRGDataset();

    poDS->fdGEN = fdGEN;
    poDS->fdIMG = fdIMG;
    poDS->fdTHF = fdTHF;

    poDS->baseFileName = baseFileName;
    poDS->bCreation = TRUE;
    poDS->nNextAvailableBlock = 1;
    poDS->NFC = (nXSize + 127) / 128;
    poDS->NFL = (nYSize + 127) / 128;
    poDS->nRasterXSize = nXSize;
    poDS->nRasterYSize = nYSize;
    poDS->bGeoTransformValid = FALSE;
    poDS->TILEINDEX = new int [poDS->NFC*poDS->NFL];
    memset(poDS->TILEINDEX, 0, sizeof(int)*poDS->NFC*poDS->NFL);
    poDS->offsetInIMG = 2048;
    poDS->poOverviewDS = NULL;

    poDS->nBands = 3;
    for( i = 0; i < poDS->nBands; i++ )
        poDS->SetBand( i+1, new ADRGRasterBand( poDS, i+1 ) );

    return poDS;
}

void ADRGDataset::WriteGENFile()
{
    if (!bGeoTransformValid)
    {
        CPLError(CE_Failure, CPLE_AppDefined, "No geo transform available !");
        adfGeoTransform[0] = 0;
        adfGeoTransform[3] = 0;
        adfGeoTransform[1] = 1;
        adfGeoTransform[5] = 1;
    }

    LSO = adfGeoTransform[0];
    PSO = adfGeoTransform[3];
    ARV = (int)floor(360. / adfGeoTransform[1] + .5);
    BRV = (int)floor(-360. / adfGeoTransform[5] + .5);
    
    /*ARV = ((ARV + 255) / 512) * 512;
    BRV = ((BRV + 255) / 512) * 512;*/
    
    int SCA = (int)floor(1000000. * 400384 / BRV + 0.5);

    int nOvSizeX = nRasterXSize; // FIXME
    int nOvSizeY = nRasterYSize; // FIXME

    FILE* fd = fdGEN;

    /* Write header */
    {
        int nFields = 0;
        int sizeOfFields[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, };
        const char* nameOfFields[] = { "000", "001", "DRF", "DSI", "OVI", "GEN", "SPR", "BDF", "TIM" };
        int pos = BeginHeader(fd, 3, 4, 3, N_ELEMENTS(sizeOfFields));

        sizeOfFields[nFields++] += WriteFieldDecl(fd, ' ', ' ', "GENERAL_INFORMATION_FILE", "", ""); /* 000 */
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "RECORD_ID_FIELD", /* 001 */
                                                  "RTY!RID",
                                                  "(A(3),A(2))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '1', "DATA_SET_DESCRIPTION_FIELD", /* DRF */
                                                  "NSH!NSV!NOZ!NOS",
                                                  "(4I(2))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "DATA_SET-ID_FIELD", /* DSI */
                                                  "PRT!NAM",
                                                  "(A(4),A(8))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "OVERVIEW_INFORMATION_FIELD", /* OVI */
                                                  "STR!ARV!BRV!LSO!PSO",
                                                  "(I(1),I(8),I(8),A(11),A(10))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "GENERAL_INFORMATION_FIELD", /* GEN */
                                                  "STR!LOD!LAD!UNIloa!SWO!SWA!NWO!NWA!NEO!NEA!SEO!SEA!SCA!ZNA!PSP!IMR!ARV!BRV!LSO!PSO!TXT",
                                                  "(I(1),2R(6),I(3),A(11),A(10),A(11),A(10),A(11),A(10),A(11),A(10),I(9),I(2),R(5),A(1),2I(8),A(11),A(10),A(64))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "DATA_SET_PARAMETERS_FIELD", /* SPR */
                                                  "NUL!NUS!NLL!NLS!NFL!NFC!PNC!PNL!COD!ROD!POR!PCB!PVB!BAD!TIF",
                                                  "(4I(6),2I(3),2I(6),5I(1),A(12),A(1))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '2', '6', "BAND_ID_FIELD", /* BDF */
                                                  "*BID!WS1!WS2",
                                                  "(A(5),I(5),I(5))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '2', '1', "TILE_INDEX_MAP_FIELD", /* TIM */
                                                  "*TSI",
                                                  "(I(5))");
        
        FinishWriteHeader(fd, pos, 3, 4, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }

    /* Write DATA_SET_DESCRIPTION_RECORD */
    {
        int nFields = 0;
        int sizeOfFields[] = {0, 0};
        const char* nameOfFields[] = { "001", "DRF" };
        int pos = BeginLeader(fd, 3, 4, 3, N_ELEMENTS(sizeOfFields));

        /* Field 001 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "DSS", 3); /* RTY */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field DRF */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 2); /* NSH */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 2); /* NSV */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 2); /* NOZ */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 2); /* NOS */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        FinishWriteLeader(fd, pos, 3, 4, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }
    
    /* Write OVERVIEW_RECORD */
    {
        int nFields = 0;
        int sizeOfFields[] = {0, 0, 0, 0, 0, 0};
        const char* nameOfFields[] = { "001", "DSI", "OVI", "SPR", "BDF", "TIM" };
        int pos = BeginLeader(fd, 9, 9, 3, N_ELEMENTS(sizeOfFields));
        
        /* Field 001 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "OVV", 3); /* RTY */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field DSI */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "ADRG", 4); /* PRT */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, baseFileName, 8); /* NAM */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        /* Field OVI */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 3, 1); /* STR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, ARV, 8); /* ARV */   /* FIXME */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, BRV, 8); /* BRV */   /* FIXME */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO); /* LSO */   /* FIXME */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO); /* PSO */    /* FIXME */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        /* Field SPR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 6); /* NUL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, nOvSizeX-1, 6); /* NUS */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, nOvSizeY-1, 6); /* NLL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 6); /* NLS */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, (nOvSizeY + 127) / 128, 3); /* NFL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, (nOvSizeX + 127) / 128, 3); /* NFC */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 128, 6); /* PNC */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 128, 6); /* PNL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* COD */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 1); /* ROD */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* POR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* PCB */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 8, 1); /* PVB */
        char tmp[12+1];
        sprintf(tmp, "%s.IMG", (const char*)baseFileName); /* FIXME */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, tmp, 12); /* BAD */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Y", 1); /* TIF */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        /* Field BDF */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Red", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Green", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Blue", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        /* Field TIM */
        int i;
        for(i=0;i<NFL*NFC;i++)
        {
            sizeOfFields[nFields] += WriteSubFieldInt(fd, TILEINDEX[i], 5); /* TSI */
        }
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        FinishWriteLeader(fd, pos, 9, 9, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }
    
    /* Write GENERAL_INFORMATION_RECORD */
    {
        int nFields = 0;
        int sizeOfFields[] = {0, 0, 0, 0, 0, 0};
        const char* nameOfFields[] = { "001", "DSI", "GEN", "SPR", "BDF", "TIM" };
        int pos = BeginLeader(fd, 9, 9, 3, N_ELEMENTS(sizeOfFields));
        
        /* Field 001 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "GIN", 3); /* RTY */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field DSI */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "ADRG", 4); /* PRT */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, baseFileName, 8); /* NAM */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        /* Field `GEN */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 3, 1); /* STR */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "0099.9", 6); /* LOD */   /* FIXME */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "0099.9", 6); /* LAD */   /* FIXME */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 16, 3); /* UNIloa */   /* FIXME */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO); /* SWO */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO + nRasterYSize * adfGeoTransform[5]); /* SWA */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO); /* NWO */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO); /* NWA */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO + nRasterXSize * adfGeoTransform[1]); /* NEO */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO); /* NEA */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO + nRasterXSize * adfGeoTransform[1]); /* SEO */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO + nRasterYSize * adfGeoTransform[5]); /* SEA */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, SCA, 9); /* SCA */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 2); /* ZNA */  /* FIXME */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "100.0", 5); /* PSP */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "N", 1); /* IMR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, ARV, 8); /* ARV */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, BRV, 8); /* BRV */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO); /* LSO */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO); /* PSO */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 64); /* TXT */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field SPR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 6); /* NUL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, nRasterXSize-1, 6); /* NUS */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, nRasterYSize-1, 6); /* NLL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 6); /* NLS */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, (nRasterYSize + 127) / 128, 3); /* NFL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, (nRasterXSize + 127) / 128, 3); /* NFC */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 128, 6); /* PNC */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 128, 6); /* PNL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* COD */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 1); /* ROD */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* POR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* PCB */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 8, 1); /* PVB */
        char tmp[12+1];
        sprintf(tmp, "%s.IMG", (const char*)baseFileName);
        sizeOfFields[nFields] += WriteSubFieldStr(fd, tmp, 12); /* BAD */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Y", 1); /* TIF */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        /* Field BDF */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Red", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Green", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Blue", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;
        
        /* Field TIM */
        int i;
        for(i=0;i<NFL*NFC;i++)
        {
            sizeOfFields[nFields] += WriteSubFieldInt(fd, TILEINDEX[i], 5); /* TSI */
        }
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        FinishWriteLeader(fd, pos, 9, 9, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }
}

void ADRGDataset::WriteTHFFile()
{
    FILE* fd = fdTHF;

    /* Write header */
    {
        int nFields = 0;
        int sizeOfFields[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        const char* nameOfFields[] = { "000", "001", "VDR", "FDR", "QSR", "QUV", "CPS", "CPT", "SPR", "BDF", "VFF"};
        int pos = BeginHeader(fd, 3, 4, 3, N_ELEMENTS(sizeOfFields));

        sizeOfFields[nFields++] += WriteFieldDecl(fd, ' ', ' ', "TRANSMITTAL_HEADER_FILE", "", ""); /* 000 */
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "RECORD_ID_FIELD", /* 001 */
                                                  "RTY!RID",
                                                  "(A(3),A(2))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "TRANSMITTAL_HEADER_FIELD", /* VDR */
                                                  "MSD!VOO!ADR!NOV!SQN!NOF!URF!EDN!DAT",
                                                  "(A(1),A(200),A(1),I(1),I(1),I(3),A(16),I(3),A(12))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "DATA_SET_DESCRIPTION_FIELD", /* FDR */
                                                  "NAM!STR!PRT!SWO!SWA!NEO!NEA",
                                                  "(A(8),I(1),A(4),A(11),A(10),A(11),A(10))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "SECURITY_AND_RELEASE_FIELD", /* QSR */
                                                  "QSS!QOD!DAT!QLE",
                                                  "(A(1),A(1),A(12),A(200))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "VOLUME_UP_TO_DATENESS_FIELD", /* QUV */
                                                  "SRC!DAT!SPA",
                                                  "(A(100),A(12),A(20))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "TEST_PATCH_IDENTIFIER_FIELD", /* CPS */
                                                  "PNM!DWV!REF!PUR!PIR!PIG!PIB",
                                                  "(A(7),I(6),R(5),R(5),I(3),I(3),I(3))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "TEST_PATCH_INFORMATION_FIELD", /* CPT */
                                                  "STR!SCR",
                                                  "(I(1),A(100))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '6', "DATA_SET_PARAMETERS_FIELD", /* SPR */
                                                  "NUL!NUS!NLL!NLS!NFL!NFC!PNC!PNL!COD!ROD!POR!PCB!PVB!BAD!TIF",
                                                  "(I(6),I(6),I(6),I(6),I(3),I(3),I(6),I(6),I(1),I(1),I(1),I(1),I(1),A(12),A(1))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '2', '6', "BAND_ID_FIELD", /* BDF */
                                                  "*BID!WS1!WS2",
                                                  "(A(5),I(5),I(5))");
        sizeOfFields[nFields++] += WriteFieldDecl(fd, '1', '0', "TRANSMITTAL_FILENAMES_FIELD", /* VFF */
                                                  "VFF",
                                                  "(A(51))");

        FinishWriteHeader(fd, pos, 3, 4, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }

    /* Write TRANSMITTAL_DESCRIPTION_RECORD */
    {
        int nFields = 0;
        int sizeOfFields[] = {0, 0, 0};
        const char* nameOfFields[] = { "001", "VDR", "FDR" };
        int pos = BeginLeader(fd, 3, 4, 3, N_ELEMENTS(sizeOfFields));

        /* Field 001 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "VTH", 3); /* RTY */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field VDR */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, " ", 1); /* MSD */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 200); /* VOO */ /* Title and address of originator */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, " ", 1); /* ADR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 1); /* NOV */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 1); /* SQN */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 3); /* NOF */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 16); /* URF */ /* DMA stock number for this CDROM */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 3); /* EDN */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "017,19940101", 12); /* DAT */  /* Publication date */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field FDR */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, baseFileName, 8); /* NAM */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 3, 1); /* STR */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "ADRG", 4); /* PRT */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO); /* SWO */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO + nRasterYSize * adfGeoTransform[5]); /* SWA */
        sizeOfFields[nFields] += WriteLongitude(fd, LSO + nRasterXSize * adfGeoTransform[1]); /* NEO */
        sizeOfFields[nFields] += WriteLatitude(fd, PSO); /* NEA */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        FinishWriteLeader(fd, pos, 3, 4, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }

    /* Write SECURITY_AND_UPDATE_RECORD */
    {
        int nFields = 0;
        int sizeOfFields[] = {0, 0, 0};
        const char* nameOfFields[] = { "001", "QSR", "QUV" };
        int pos = BeginLeader(fd, 3, 4, 3, N_ELEMENTS(sizeOfFields));

        /* Field 001 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "LCF", 3); /* RTY */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field VDR */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "U", 1); /* QSS */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "N", 1); /* QOD */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 12); /* DAT */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 200); /* QLE */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field FDR */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "MILITARY SPECIFICATION ARC DIGITIZED RASTER GRAPHICS (ADRG)", 100); /* SRC */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "022,19900222", 12); /* DAT */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "MIL-A-89007", 20); /* SPA */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        FinishWriteLeader(fd, pos, 3, 4, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }

    /* Write TEST_PATCH_DATA_RECORD */
    {
        int nFields = 0;
        int sizeOfFields[] = {0, 0, 0, 0, 0};
        const char* nameOfFields[] = { "001", "CPS", "CPT", "SPR", "BDF" };
        int pos = BeginLeader(fd, 3, 4, 3, N_ELEMENTS(sizeOfFields));

        /* Field 001 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "TPA", 3); /* RTY */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field CPS */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Black", 7); /* PNM */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 6); /* DMV */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 5); /* REF */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 5); /* PUR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 3); /* PIR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 3); /* PIG */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 3); /* PIB */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field CPT */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 3, 1); /* STR */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "", 100); /* SCR */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        int nPatchXSize = 512;
        int nPatchYSize = 512;

        /* Field SPR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 6); /* NUL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, nPatchXSize-1, 6); /* NUS */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, nPatchYSize-1, 6); /* NLL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 6); /* NLS */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, (nPatchYSize + 127) / 128, 3); /* NFL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, (nPatchXSize + 127) / 128, 3); /* NFC */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 128, 6); /* PNC */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 128, 6); /* PNL */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* COD */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 1, 1); /* ROD */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* POR */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 1); /* PCB */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 8, 1); /* PVB */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "TESTPA01.CPH", 12); /* BAD */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "N", 1); /* TIF */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field BDF */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Red", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Green", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "Blue", 5); /* BID */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS1 */
        sizeOfFields[nFields] += WriteSubFieldInt(fd, 0, 5); /* WS2 */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        FinishWriteLeader(fd, pos, 3, 4, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }

    /* Write TRANSMITTAL_FILENAMES_RECORD */
    {
        char tmp[12+1];

        int nFields = 0;
        int sizeOfFields[] = {0, 0, 0, 0, 0};
        const char* nameOfFields[] = { "001", "VFF", "VFF", "VFF", "VFF" };
        int pos = BeginLeader(fd, 9, 9, 3, N_ELEMENTS(sizeOfFields));

        /* Field 001 */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "TFN", 3); /* RTY */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "01", 2); /* RID */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field VFF */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "TRANSH01.THF", 51); /* VFF */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field VFF */
        sizeOfFields[nFields] += WriteSubFieldStr(fd, "TESTPA01.CPH", 51); /* VFF */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field VFF */
        sprintf(tmp, "%s.GEN", (const char*)baseFileName);
        sizeOfFields[nFields] += WriteSubFieldStr(fd, tmp, 51); /* VFF */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        /* Field VFF */
        sprintf(tmp, "%s.IMG", (const char*)baseFileName);
        sizeOfFields[nFields] += WriteSubFieldStr(fd, tmp, 51); /* VFF */
        sizeOfFields[nFields] += WriteFieldTerminator(fd);
        nFields++;

        FinishWriteLeader(fd, pos, 9, 9, 3, N_ELEMENTS(sizeOfFields), sizeOfFields, nameOfFields);
    }
}

/************************************************************************/
/*                         GDALRegister_ADRG()                          */
/************************************************************************/

void GDALRegister_ADRG()

{
    GDALDriver  *poDriver;

    if( GDALGetDriverByName( "ADRG" ) == NULL )
    {
        poDriver = new GDALDriver();
        
        poDriver->SetDescription( "ADRG" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME, 
                                   "ARC Digitilized Raster Graphics" );
        poDriver->SetMetadataItem( GDAL_DMD_HELPTOPIC, 
                                   "frmt_various.html#ADRG" );
        poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "gen" );
        poDriver->SetMetadataItem( GDAL_DMD_CREATIONDATATYPES, 
                                   "Byte" );
        
        poDriver->pfnOpen = ADRGDataset::Open;
        poDriver->pfnCreate = ADRGDataset::Create;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}

