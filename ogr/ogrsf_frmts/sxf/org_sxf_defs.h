/******************************************************************************
 * $Id: org_sxf_defs.h  $
 *
 * Project:  SXF Translator
 * Purpose:  Include file defining Records Structures for file reading and 
 *           basic constants.
 * Author:   Ben Ahmed Daho Ali, bidandou(at)yahoo(dot)fr
 *
 ******************************************************************************
 * Copyright (c) 2011, Ben Ahmed Daho Ali
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
 *
 ******************************************************************************
 * Structure of the SXF file : 
 * ----------------------
 *    - Header 
 *    - Passport
 *    - Descriptor of data
 *    - Records 
 *         - Title of the record 
 *         - The certificate of the object (the geomety)
 *             - sub-objects
 *             - The graphic description of object
 *             - The description of the vector of the tying of the 3d- model of object
 *         - Semantics of object
 *
 * Notes  : 
 * -------
 * Note 1.  Flag of the state of data (2 bits):
 * xxxxxx11- given in the state e (size of the exchange of data).
 *
 * Note 2.  Flag of the correspondence to projection (1 bit):
 * xxxxx0xx - do not correspond to the projection (i.e. map it can have turning 
 *                 relative to true position and certain deformation);
 * xxxxx1xx - correspond to projection.
 *
 * Note 3.  Flag of the presence of real coordinates (2 bits):
 * xxx00xxx - entire certificate of objects is represented in the conditional 
 *                                 system of coordinates (in the samples);
 * xxx11xxx - entire certificate of objects is represented in the real coordinates 
 *            in the locality in accordance with the specifications of sheet 
 *            (projection, the coordinate system, unit of measurement), 
 *            the data about the scale and the discretion of digitization bear 
 *            reference nature.
 *
 * Note 4. Flag of the method of coding (2 bits):
 * x00xxxxx - the classification codes of objects and semantic characteristics 
 *          are represented by the decimal numbers, recorded in the binary 
 *          form (for example: the code of the object “32100000” will be written 
 *          down in the form 0x01E9CEA0, the code of semantics “253” - in the form 0x00FD).
 *
 * Note 5. Table of generalization (1 bit):
 * 0xxxxxxx - the level of generalization is assigned according to the table of the 
 *           general maps (it is described in Table 2.4);
 * 1xxxxxxx - noload condition the level of generalization is assigned according to 
 *           the table of the large-scale maps (it is described in Table 2.5).
 *
 * Note 6.  Flag of coding the texts of the Texts objects (1 bytes):
 * 0- in the coding ASCIIZ (Dos);
 * 1- in the coding ANSI (Windows);
 * 2- in the coding KOI-8 (Unix).
 *
 * Note 7.  Flag of the accuracy of coordinates (1 bytes):
 * 0 – are not established;
 * 1 – the increased accuracy of storage of coordinates (meters, radians or degrees);
 * 2 – of coordinate are recorded with an accuracy to centimeter (meters, 2 signs after comma);
 * 3 – coordinates are recorded with an accuracy to millimeter (meters, 3 sign after comma).
 *
 * Note 8. Form of the framework (1 byte):
 * -1- it is not established;
 *  0- map is unconfined by the framework;
 *  1- trapeziform without the salient points;
 *  2- trapeziform with the salient points;
 *  3- rectangular;
 *  4- circular;
 *  5- arbitrary.
 *
 * Note 9. Sign of output to the framework (4 bits):
 * 0000xxxx - there are no outputs to the framework;
 * 1000xxxx - northern framework;
 * 0100xxxx - eastern framework;
 * 0010xxxx - southern framework;
 * 0001xxxx - western framework.
 *
 * Note 10. Size of the element of certificate (1 bit):
 * xxxxx0xx - 2 bytes (for the integer value); 
 *            4 bytes (for the floating point); 
 * xxxxx1xx - 4 bytes (for the integer value); 
 *            8 bytes (for the floating point).
 *
 * Note 11. Sign of certificate with the text (1 bit): 
 * xxxx0xxx - certificate contains only the coordinates of points; 
 * xxxx1xxx - no-load condition certificate contains the text of signature, 
 *         is allowed only for the objects of the type "signature" or 
 *         "the template of signature".
 *
 * Note 12. [Masshtabiruemost] of drawing (sign) (1 bit):
 * xx0xxxxx - arbitrary symbol of object not scaled;
 * xx1xxxxx - the arbitrary symbol of object is scaled during the mapping.
 * 
 * Note 13. Sign of the construction of spline on the certificate (2 bits):
 * 00xxxxxx – the construction of spline with the visualization is not carried out;
 * 01xxxxxx – smoothing out spline (cutting angles);
 * 10xxxxxx – enveloping spline (it penetrates all points of certificate).
 ****************************************************************************/

#ifndef SXF_DEFS_H
#define SXF_DEFS_H

#define IDSXF          0x00465853     /* SXF  */
#define IDSXFVERSION4   0x00040000     /* 4.0  */
#define IDSXFVERSION3   0x00000300     /* 3.0  */
#define IDSXFVERSIONUNKNOWN   0x00000000

#define IDSXFDATA      0x00544144     /* DAT  */
#define IDSXFOBJ       0X7FFF7FFF     /* Object */
#define IDSXFGRAPH     0X7FFF7FFE     /* graphics section */
#define IDSXFVECT3D    0X7FFF7FFD     /* 3D vector section */

#include <map>

#include "cpl_port.h"

typedef GUInt32 SXFVersion;

enum SXFDataState /* Flag of the state of the data (Note 1) */
{
    SXF_DS_UNKNOWN = 0,
    SXF_DS_EXCHANGE = 3
};

typedef struct
{
    SXFDataState   dataState;
    bool projectionDataCompliance; /* Flag of the correspondence to the projection (Note 2) */
    bool realCoordinatesCompliance; /* Flag of the presence of the real coordinates (Note 3) */
} SXFInformationFlags;

enum SXFCoordinateType
{
    SXF_CT_RECTANGULAR = 0,
    SXF_CT_GEODETIC
};

/* COORDINATES OF THE ANGLES OF SHEET */
typedef struct
{
    long double   dfXsw  ; /* X the South Western angle (vertical line) */
    long double   dfYsw  ; /* Y  */
    long double   dfXnw  ; /* X the North Western angle */
    long double   dfYnw  ; /* Y  */
    long double   dfXne  ; /* X the North Eastern angle */
    long double   dfYne  ; /* Y  */
    long double   dfXse  ; /* X the South Eastern angle */
    long double   dfYse  ; /* Y  */
} SheetCornersCoordinates;


/*
 *  "Panorama" projection codes
 */
typedef enum
{
    PROJ_UNKNOWN = -1,
    PROJ_TM = 1,      // Gauss-Kruger (Transverse Mercator)
    PROJ_LCC = 2,      // Lambert Conformal Conic 2SP
    PROJ_UTM = 17     // Universal Transverse Mercator (UTM)

} PanaramaProjCode;

/*
 *  "Panorama" datum codes
 */
typedef enum
{
    DATUM_UNKNOWN = 0,
    DATUM_PUL_42 = 1      // Pulkovo, 1942

} PanaramaDatumCode;

/*
 *  "Panorama" ellips codes
 */
typedef enum
{
    ELLIPS_UNKNOWN = -1,
    ELLIPS_42 = 1,      // 1942
    ELLIPS_WGS_76 = 2      // WGS-76

}PanaramaEllips;

typedef enum
{
    CMU_METRE=1,
    CMU_DECIMETRE,
    CMU_CENTIMETRE,
    CMU_MILLIMETRE,
    CMU_DEGREE,
    CMU_RADIAN
} CoordinateMeasUnit;

typedef struct
{
    PanaramaProjCode iProjSys;      /* Projection of initial. the material */
    PanaramaDatumCode iDatum;       /* Coordinate system */
    PanaramaEllips iEllips;         /* Form of the ellipsoid */
    CoordinateMeasUnit unitInPlan;
} SXFMathBase;

typedef struct
{
    long double   dfMainPar1      ; /* First main parallel (contains 0) */
    long double   dfMainPar2      ; /* Second main parallel (contains the value of the scale factor) */
    long double   dfAxialMer      ; /* Axial meridian (contains the value of the length of principal point) */
    long double   dfMainPtPar     ; /* Parallel of the principal point (contains the value of the latitude of principal point) */
    long double   dfFalseNorthing ; /* Displacement to the north (in the meters) */
    long double   dfFalseEasting  ; /* Displacement to the east (in the meters) */
} ProjectionInfo;


/*
 * List of SXF file format geometry types.
 */
typedef enum org_sxf_geometry_type
{
    sxfLine    = 0,      /* MultiLineString geometric object                  */
    sxfPolygon = 1,      /* Polygon geometric object                          */
    sxfPoint   = 2,      /* MultiPoint geometric object                       */
    sxfText    = 3,      /* LineString geometric object with associated label */
    sxfVector  = 4,      /* Vector geometric object with associated label */
    sxfTextTemplate    = 5      /*  */
} OGRsxfGeometryType;

typedef struct
{
    bool bHasTextSign;        /* Sign of certificate with the text (Note 11) */

    GUInt16 nPointsCount;     /* Number of points of the certificate */

    GByte bElemSize;          /* Size of the element of the certificate (Note 10) */
    GByte bElemType;          /* Type of the element of the certificate (0- Integers, 2- Float)(H always in float) */
    GByte bDim;               /* Dimensionality of the idea (0- 2D, 1- 3D) */
    GUInt16 nSubObjCount;     /* Number of sub-objects */

} SXFRecordCertifInfo;

typedef struct
{
    GInt32   iRecordID;         /* IDENTIFIER OF THE BEGINNING OF RECORD (0x7FFF7FFF) */
    GInt32   nRecordLength;     /* THE OVERALL LENGTH OF RECORD (with the title) */
    GInt32   nCertifLength;     /* LENGTH OF CERTIFICATE (in bytes) */
    GInt32   iCC;               /* CLASSIFICATION CODE */

    GUInt16  nObjNumb;  /* Number in the group */
    GUInt16  nGrpNumb;  /* The number of the group */

    /*
     *  REFERENCE DATA
     */
    OGRsxfGeometryType bGeomType;

    bool bHazSemantics;                 /* Presence of semantics*/
    bool bHazTyingVect;                /* The presence of the vector of the tying (0-not present, 1-present) */

    SXFRecordCertifInfo certifInfo;

}SXFRecordInfo;


/************************************************************************/
/*                         RSCInfo                                      */
/************************************************************************/
typedef std::map<GUInt32, std::string> RSCObjects;
struct RSCLayer
{
    GByte szLayerId;
    std::string szLayerName;
    RSCObjects rscObjects;
};
typedef std::map<GByte, RSCLayer> RSCLayers;


/************************************************************************/
/*                         SXFPassport                                  */
/************************************************************************/
struct SXFDeviceInfo
{
    GInt32  iDeviceCapability;
    SheetCornersCoordinates deviceFrameCoordinates;
};

struct SXFPassport
{
    SXFVersion version;
    SXFInformationFlags informationFlags;
    SXFMathBase mathBase;
    SXFDeviceInfo deviceInfo;
    GUInt32  nScale;
    SheetCornersCoordinates sheetRectCoordinates;
};

/*
 *  Semantics Attribute types
 */
typedef enum
{
    sctASCIIZ_DOS = 0,
    sctOneByte = 1,
    sctTwoByte = 2,
    sctForeByte = 4,
    sctEightByte = 8,
    sctANSI_Windows = 126,
    sctUNICODE_UNIX = 127,
    sctBigString = 128
} AttributeTypeSXFOBJ;

/*
 * HEADER OF THE RSC FILE
 */
typedef struct  {
    unsigned nOffset;
    unsigned nLenght;
    unsigned nRecordCount;
} RecordRSCSection;

typedef struct{
    char szID[4];
    unsigned nFileLength;
    unsigned nVersion;
    unsigned nEncoding;
    unsigned nFileState;
    unsigned nFileModState;
    unsigned nLang;//1 - en, 2 - rus
    unsigned nNextID;
    char date[8];
    char szMapType[32];
    char szClassifyName[32];
    char szClassifyCode[8];
    long nScale;
    char nScales[4];
    RecordRSCSection Objects;
    RecordRSCSection Semantic;
    RecordRSCSection ClassifySemantic;
    RecordRSCSection Defaults;
    RecordRSCSection Semantics;
    RecordRSCSection Layers;
    RecordRSCSection Limits;
    RecordRSCSection Parameters;
    RecordRSCSection Print;
    RecordRSCSection Palettes;
    RecordRSCSection Fonts;
    RecordRSCSection Libs;
    RecordRSCSection ImageParams;
    RecordRSCSection Tables;
    char nFlagKeysAsCodes;
    char nFlagPalleteMods;
    char Reserved[30];
    char szFontEnc[4];
    unsigned nColorsInPalette;
} RecordRSCHEAD;

#endif  /* SXF_DEFS_H */
