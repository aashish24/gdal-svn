/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Function to register all known OGR drivers.
 * Author:   Frank Warmerdam, warmerdam@pobox.com
 *
 ******************************************************************************
 * Copyright (c) 1999,  Les Technologies SoftMap Inc.
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
 * Revision 1.33  2006/11/28 15:34:41  osemykin
 * Added new Informix DataBlade driver (IDB)
 *
 * Revision 1.32  2006/11/01 17:38:41  fwarmerdam
 * Removed Register e00 commented out call.  handled via avc call.
 *
 * Revision 1.31  2006/06/07 14:39:09  mloskot
 * Added new KML (Beta) driver to OGR. Thanks to Christopher Condit - the author of KML drvier.
 *
 * Revision 1.30  2006/04/03 01:48:28  fwarmerdam
 * fixup contact info
 *
 * Revision 1.29  2006/02/19 21:39:13  mloskot
 * All RegisterOGR* calls wrapped with #ifdef *_ENABLED checks
 * NOTE: Makefiles need to be updated to define/check those macros too. makefile.evc4 for Windows CE has been updated.
 *
 * Revision 1.28  2005/11/22 17:01:09  fwarmerdam
 * added SDE support
 *
 * Revision 1.27  2005/11/10 21:36:56  fwarmerdam
 * Added DXF/DWG support
 *
 * Revision 1.26  2005/09/05 19:32:52  fwarmerdam
 * Added PGEO
 *
 * Revision 1.25  2005/08/05 15:34:49  fwarmerdam
 * added grass support
 *
 * Revision 1.24  2005/07/12 16:06:06  fwarmerdam
 * Don't register ILI unless enabled.
 *
 * Revision 1.23  2005/07/08 22:10:57  pka
 * Initial import of OGR Interlis driver
 *
 * Revision 1.22  2005/01/19 20:34:38  fwarmerdam
 * added autoloaddrivers call
 *
 * Revision 1.21  2004/10/06 14:01:19  fwarmerdam
 * added MYSQL support.
 *
 * Revision 1.20  2004/07/20 19:19:11  warmerda
 * added CSV
 *
 * Revision 1.19  2004/07/10 05:03:42  warmerda
 * added SQLite
 *
 * Revision 1.18  2004/01/30 02:33:29  warmerda
 * added OGRDODS
 *
 * Revision 1.17  2003/11/07 22:06:10  warmerda
 * reorder registration
 *
 * Revision 1.16  2003/11/07 17:51:30  warmerda
 * Added VRT format
 *
 * Revision 1.15  2003/10/07 05:07:41  warmerda
 * Only include ODBC support if enabled.
 *
 * Revision 1.14  2003/10/06 19:16:38  warmerda
 * added ODBC support
 *
 * Revision 1.13  2003/04/08 19:32:06  warmerda
 * added memory driver
 *
 * Revision 1.12  2003/02/03 21:17:03  warmerda
 * added .rec driver
 *
 * Revision 1.11  2002/12/28 04:10:11  warmerda
 * added Oracle(OCI) support
 *
 * Revision 1.10  2002/05/29 20:33:11  warmerda
 * added FME support
 *
 * Revision 1.9  2002/02/18 20:56:55  warmerda
 * register AVCBin
 *
 * Revision 1.8  2002/01/25 20:40:51  warmerda
 * register gml
 *
 * Revision 1.7  2001/07/18 04:55:16  warmerda
 * added CPL_CSVID
 *
 * Revision 1.6  2000/11/23 06:03:09  warmerda
 * fixed PG support
 *
 * Revision 1.5  2000/10/17 17:53:04  warmerda
 * added postgresql support
 *
 * Revision 1.4  2000/08/24 04:44:05  danmo
 * Added optional OGDI driver in OGR
 *
 * Revision 1.3  1999/12/22 15:36:45  warmerda
 * RegisterOGRMIF no longer exists
 *
 * Revision 1.2  1999/11/14 18:10:44  svillene
 * add RegisterOGRMIF RegisterOGRTAB
 *
 * Revision 1.1  1999/11/04 21:10:51  warmerda
 * New
 *
 */

#include "ogrsf_frmts.h"

CPL_CVSID("$Id$");

/************************************************************************/
/*                           OGRRegisterAll()                           */
/************************************************************************/

void OGRRegisterAll()
{
    OGRSFDriverRegistrar::GetRegistrar()->AutoLoadDrivers();

#ifdef SHAPE_ENABLED
    RegisterOGRShape();
#endif
#ifdef TAB_ENABLED
    RegisterOGRTAB();
#endif
#ifdef NTF_ENABLED
    RegisterOGRNTF();
#endif
#ifdef SDTS_ENABLED
    RegisterOGRSDTS();
#endif
#ifdef TIGER_ENABLED
    RegisterOGRTiger();
#endif
#ifdef S57_ENABLED
    RegisterOGRS57();
#endif
#ifdef DGN_ENABLED
    RegisterOGRDGN();
#endif
#ifdef VRT_ENABLED
    RegisterOGRVRT();
#endif
#ifdef AVCBIN_ENABLED
    RegisterOGRAVCBin();
#endif
#ifdef REC_ENABLED
    RegisterOGRREC();
#endif
#ifdef MEM_ENABLED
    RegisterOGRMEM();
#endif
#ifdef CSV_ENABLED
    RegisterOGRCSV();
#endif
#ifdef GML_ENABLED
    RegisterOGRGML();
#endif
#ifdef KML_ENABLED
    RegisterOGRKML();
#endif
#ifdef ILI_ENABLED
    RegisterOGRILI1();
    RegisterOGRILI2();
#endif
#ifdef SQLITE_ENABLED
    RegisterOGRSQLite();
#endif
#ifdef DODS_ENABLED
    RegisterOGRDODS();
#endif
#ifdef ODBC_ENABLED
    RegisterOGRODBC();
#endif    
#ifdef PGEO_ENABLED
    RegisterOGRPGeo();
#endif
#ifdef OGDI_ENABLED
    RegisterOGROGDI();
#endif
#ifdef PG_ENABLED
    RegisterOGRPG();
#endif
#ifdef MYSQL_ENABLED
    RegisterOGRMySQL();
#endif
#ifdef OCI_ENABLED
    RegisterOGROCI();
#endif
#ifdef SDE_ENABLED
    RegisterOGRSDE();
#endif
#ifdef DWGDIRECT_ENABLED
    RegisterOGRDXFDWG();
#endif
#ifdef GRASS_ENABLED
    RegisterOGRGRASS();
#endif
#ifdef FME_ENABLED
    RegisterOGRFME();
#endif
#ifdef IDB_ENABLED
    RegisterOGRIDB();
#endif
} /* OGRRegisterAll */

