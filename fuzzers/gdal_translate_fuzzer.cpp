/******************************************************************************
 *
 * Project:  GDAL
 * Purpose:  Fuzzer
 * Author:   Even Rouault, even.rouault at spatialys.com
 *
 ******************************************************************************
 * Copyright (c) 2017, Even Rouault <even.rouault at spatialys.com>
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

#include "gdal.h"
#include "cpl_conv.h"
#include "cpl_string.h"
#include "cpl_vsi.h"
#include "gdal_alg.h"
#include "gdal_utils.h"
#include "gdal_frmts.h"

extern "C" int LLVMFuzzerInitialize(int* argc, char*** argv);
extern "C" int LLVMFuzzerTestOneInput(const uint8_t *buf, size_t len);

int LLVMFuzzerInitialize(int* /*argc*/, char*** argv)
{
    const char* exe_path = (*argv)[0];
    CPLSetConfigOption("GDAL_DATA", CPLGetPath(exe_path));
    CPLSetConfigOption("CPL_TMPDIR", "/tmp");
    CPLSetConfigOption("DISABLE_OPEN_REAL_NETCDF_FILES", "YES");
    // Disable PDF text rendering as fontconfig cannot access its config files
    CPLSetConfigOption("GDAL_PDF_RENDERING_OPTIONS", "RASTER,VECTOR");
    // to avoid timeout in WMS driver
    CPLSetConfigOption("GDAL_WMS_ABORT_CURL_REQUEST", "YES");
    GDALAllRegister();
    return 0;
}

int LLVMFuzzerTestOneInput(const uint8_t *buf, size_t len)
{
    VSILFILE* fp = VSIFileFromMemBuffer( "/vsimem/test.tar",
            reinterpret_cast<GByte*>(const_cast<uint8_t*>(buf)), len, FALSE );
    VSIFCloseL(fp);

    CPLPushErrorHandler(CPLQuietErrorHandler);

    char** papszArgv = NULL;
    fp = VSIFOpenL("/vsitar//vsimem/test.tar/cmd.txt", "rb");
    if( fp != NULL )
    {
        const char* pszLine = NULL;
        while( (pszLine = CPLReadLineL(fp)) != NULL )
            papszArgv = CSLAddString(papszArgv, pszLine);
        VSIFCloseL(fp);
    }

    if( papszArgv != NULL )
    {
        GDALTranslateOptions* psOptions = GDALTranslateOptionsNew(papszArgv, NULL);
        if( psOptions )
        {
            GDALDatasetH hSrcDS = GDALOpen( "/vsitar//vsimem/test.tar/in", GA_ReadOnly );
            if( hSrcDS != NULL )
            {
                GDALDatasetH hOutDS = GDALTranslate("/vsimem/out", hSrcDS,
                                                    psOptions, NULL);
                if( hOutDS )
                    GDALClose(hOutDS);
                GDALClose(hSrcDS);
            }
            GDALTranslateOptionsFree(psOptions);
        }
        CSLDestroy(papszArgv);
    }

    VSIUnlink("/vsimem/test.tar");
    VSIUnlink("/vsimem/out");

    CPLPopErrorHandler();

    return 0;
}
