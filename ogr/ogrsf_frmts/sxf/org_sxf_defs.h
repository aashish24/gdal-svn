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
#define IDSXFVERSION   0x00040000     /* 4.0  */
#define IDSXFDATA      0x00544144     /* DAT  */
#define IDSXFOBJ       0X7FFF7FFF     /* Object */
#define IDSXFGRAPH     0X7FFF7FFE     /* graphics section */
#define IDSXFVECT3D    0X7FFF7FFD     /* 3D vector section */

#define FLAGSXFDATACOMMUNICATION 3		/* state of the data (Note 1) - data commenication state */
#define FLAGSXFDATAINPROJECTION 1		/* data orresponded to the projection */
#define FLAGSXFREALCOORDINATES 1		/* certificate in real coordinates */

#include "cpl_port.h"

/**
 * HEADER OF THE SXF FILE
 */
typedef struct {
	GUInt32 nFormatID  ;  /* IDENTIFIER OF THE FILE   0x00465853 (SXF) */
	GUInt32 nPSPLength ;  /* LENGTH OF PASSPORT IN THE BYTES           */
	GInt32 nVersion   ;  /* FILE VERSION              */
	GInt32 nCheckSum  ;  /* THE CHECK SUM OF THE FILE */
} RecordSXFHEAD;


 /**
 * PASSPORT OF THE SXF FILE
 */
 typedef struct {
	char szCreateDate[12] ;  /* DATE OF THE CREATION OF FILE (DD/MM/GG \ 0) */
	char szName[32]       ;  /* NOMENCLATURE OF THE SHEET (ANSI) */
	GUInt32  nScale        ;  /* SCALE OF SHEET (DENOMINATOR) */
	char szSheetName[32]  ;  /* CODE NAME OF THE SHEET  (ASCIIZ) */

               /* INFORMATION FLAGS */
	GByte   bDataFlag       :2, /* Flag of the state of the data (Note 1) */
			bProjCorres     :1, /* Flag of the correspondence to the projection (Note 2) */
			bpnalrealk      :2, /* Flag of the presence of the real coordinates (Note 3) */
			bCodingFlag     :2, /* Flag of the method of the coding (Note 4) */
			bTableGen       :1; /* Table of the generalization (Note 5) */
 
	GByte   bSigCoding      ;   /* flag of coding the signature (Note 6) */
	GByte   bCoordAccu      ;   /* flag of the accuracy of the coordinates (Note 7) */
	GByte   bFrameFlag      :1, /* sign of the special sorting of the data (regulated in a special manner) */
			bReserve1       :7; /* Reserve = 0 */
 
	GUInt32  iMapCC   ; /* Classifier of the map objects */
 
	/* RECTANGULAR COORDINATES OF THE ANGLES OF SHEET (in meters) */
	long double   dfXsw  ; /* X southwestern angle (vertical line) */
	long double      dfYsw  ; /* Y  */
	long double   dfXnw  ; /* X the North Western angle */
	long double   dfYnw  ; /* Y  */
	long double   dfXne  ; /* X the northeastern angle */
	long double   dfYne  ; /* Y  */
	long double   dfXse  ; /* X the southeastern angle */
	long double   dfYse  ; /* Y  */

	/* GEODETIC COORDINATES OF THE ANGLES OF THE SHEET (in radians) */
	long double   dfBsw   ; /* B  */
	long double   dfLsw   ; /* L  */
	long double   dfBnw   ; /* B  */
	long double   dfLnw   ; /* L  */
	long double   dfBne   ; /* B  */
	long double   dfLne   ; /* L  */
	long double   dfBse   ; /* B  */
	long double   dfLse   ; /* L  */

	GByte iEllips       ; /* Form of the ellipsoid */
	GByte iHeightSys    ; /* System of the heights */
	GByte iProjSys      ; /* Projection of initial. the material */
	GByte iDatum        ; /* Coordinate system */
	GByte iPlanUoM      ; /* Unit of measurement in the plan 
						   * (0- meters (or samples), 64 - radians, 65 - degrees) */
	GByte iVertUoM      ; /* Unit of measurement on the height (0- meters) */
	GByte iFrameForm    ; /* Form of the framework (Note 8) */
	GByte iMapType      ; /* Generalized type of the map */

	char   szDateSurv[12]   ; /* Date of ground survey (DD/MM/OF GG \ 0) */
	GByte  iMatForme        ; /* Form of the source material (1-Map, 2-Orthophoto, 3-Photo) */
	GByte  iMatType         ; /* Type of the source material (1-Map, 2-Photo) */
	GByte  iZoneID          ; /* The identifier of zone [MSK]-63 (A-X or 0) */
	GByte  iLimit           ; /* The sign of the limitation of map by the framework */
							/* (1 – map is limited by the framework in the radians) */

	long double   dfMagDecl     ; /* The magnetic declination */
	long double   dfAvgMerConv  ; /* Average convergence of meridians */
	long double   dfMagDeclChg  ; /* The annual change in the magnetic declination */
	char          szDateMag[12] ; /* Date of the measurement of the declination */
	GInt32           iReserve2     ; /* Reserve */

	long double    dfCntrInterval   ; /* Contour interval (in meters) */
	long double    dfRotAngle        ; /* Angle of the turn of axes for the local coordinate systems
									   * (in radians clockwise) */


	GInt32  iDeviceCapability         ; /* CHARACTERISTICS OF INSTRUMENT  */
		/* The resolution of the instrument 
		 * Points per meter (if the value more than zero, usually - 20 000).
		 * If the flag of the presence of real coordinates is not equal to zero - it is ignored. */

	/* Arrangement of the framework on the instrument: (in the system of instrument) 
	 * X on the vertical line
	 * Y along the horizontal */

	GUInt32   dfXswp     ; /* X  */
	GUInt32   dfYswp     ; /* Y  */
	GUInt32   dfXnwp     ; /* X  */
	GUInt32   dfYnwp     ; /* Y  */
	GUInt32   dfXnep     ; /* X  */
	GUInt32   dfYnep     ; /* Y  */
	GUInt32   dfXsep     ; /* X  */
	GUInt32   dfYsep     ; /* Y  */
 

	GInt32   iFrameCC  ; /* CLASSIFICATION CODE OF THE FRAMEWORK OF THE OBJECT 
					   * From the classifier of the objects */

	/* Special features of filling of the data about the projection
	 * fields are filled up as follows for the generalized type of map "topographic local": */
	long double   dfMainPar1      ; /* First main parallel (contains 0) */
	long double   dfMainPar2      ; /* Second main parallel (contains the value of the scale factor) */
	long double   dfAxialMer      ; /* Axial meridian (contains the value of the length of principal point) */
	long double   dfMainPtPar     ; /* Parallel of the principal point (contains the value of the latitude of principal point) */
	long double   dfFalseNorthing ; /* Displacement to the north (in the meters) */
	long double   dfFalseEasting  ; /* Displacement to the east (in the meters) */
}RecordSXFPSP;


/**
 * DESCRIPTOR OF THE SXF FILE
 */
 
typedef struct {
GInt32   iDataID        ; /* Identifier of data (0x00544144 (DAT)) */
GInt32   nDescLength    ; /* Length of the descriptor */
char  szSheetNom[32] ; /* Nomenclature of sheet (ASCIIZ) */ 
GInt32   nObjCount      ; /* Number of recordings of the data */

               /* INFORMATION FLAGS */
 GByte   bDataFlag       :2, /* Flag of the state of the data (Note 1) */
         bProjCorres     :1, /* Flag of the correspondence to the projection (Note 2) */
         bpnalrealk      :2, /* Flag of the presence of the real coordinates (Note 3) */
         bCodingFlag     :2, /* Flag of the method of the coding (Note 4) */
         bTableGen       :1; /* Table of the generalization (Note 5) */

 GByte   bSigCoding      ; /* flag of coding the signature (Note 6) */
 GUInt16 iPlaceholder2    ; /* Reserve  = 0                 */

 GUInt16   iObjClass  ; /* Classifier of the objects */
 GUInt16   iSemClass  ; /* Classifier of semantics */
} RecordSXFDSC;

/**
 *    SXF RECORD HEADER 
 */
 
typedef struct {
GInt32   iRecordID      ;  /* IDENTIFIER OF THE BEGINNING OF RECORD (0x7FFF7FFF) */
GInt32   nRecordLength  ;  /* THE OVERALL LENGTH OF RECORD (with the title) */
GInt32   nCertifLength  ;  /* LENGTH OF CERTIFICATE (in bytes) */
GInt32   iCC            ;  /* CLASSIFICATION CODE */
 
GUInt16  nObjNumb          ; /* Number in the group */
GUInt16  nGrpNumb          ; /* The number of the group */
 
/* REFERENCE DATA */
GUInt16  bGeomType   : 4, /* Nature of the localization (0- Linear, 2- Area, 3- Point, 4- Text) */
      bFrameOutput  : 4, /* Sign of output to the framework (Note 9) */
      bClosureSign  : 1, /* Sign of the closure (0- not Locked, 1- Locked) */
      bHazSemantics : 1, /* Presence of semantics (0-not present, 1-present) */
      bCertifSize   : 1, /* Size of the element of the certificate (Note 10) */
      bHazTyingVect : 1, /* The presence of the vector of the tying (0-not present, 1-present) */
      bUnicodeSign  : 1, /* The sign of text in UNICODE (0- , 1- UNICODE) */
      bReserve      : 3; /* Reserve  */

GByte  bRecordFormat : 1, /* Format of the record of the certificate (0- linear size, 1-vector format ) */
       bDimIdea      : 1, /* Dimensionality of the idea (0- 2D, 1- 3D) */
       bElemType     : 1, /* Type of the element of the certificate (0- Integers, 2- Float)(H always in float) */
       bTextSign     : 1, /* Sign of certificate with the text (Note 11) */
       bHazDrawing   : 1, /* The presence of drawing (0-not present, 1-present) */
       bMasDrawing     : 1, /* [Masshtabiruemost] of the drawing (Note 12) */
       bSplnConst    : 2; /* The sign of the construction of spline on the certificate (Note 13) */

GByte    nVisLow : 4, /* Lower boundary of the visibility */
         nVisHig : 4; /* Upper boundary of the visibility */

GUInt32   nLOPointsCount ; /* Number of points of certificate for the large objects */
GUInt16   nSubObjCount   ; /* Number of sub-objects */
GUInt16   nPointsCount   ; /* Number of points of the certificate */
} RecordSXFOBJ;



/**
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
	sctBigString = 128,
} AttributeTypeSXFOBJ;


/**
 *  "Panorama" projection codes
 */
typedef enum
{
	PROJ_TM = 1,      // Gauss-Kruger (Transverse Mercator)
	PROJ_LCC = 2,      // Lambert Conformal Conic 2SP
	PROJ_UTM = 17,     // Universal Transverse Mercator (UTM)

} PanaramaProjCode;

/**
 *  "Panorama" datum codes
 */
typedef enum
{
	DATUM_PUL_42 = 1,      // Pulkovo, 1942

} PanaramaDatumCode;
#endif  /* SXF_DEFS_H */


