/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features Reference Implementation
 * Purpose:  Test dynamic loading of SQLite Virtual Table module using OGR layers
 * Author:   Even Rouault, even dot rouault at mines dash paris dot org
 *
 ******************************************************************************
 * Copyright (c) 2012, Even Rouault <even dot rouault at mines dash paris dot org>
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

#ifdef HAVE_SPATIALITE
  #ifdef SPATIALITE_AMALGAMATION
    /*
    / using an AMALGAMATED version of SpatiaLite
    / a private internal copy of SQLite is included:
    / so we are required including the SpatiaLite's 
    / own header 
    /
    / IMPORTANT NOTICE: using AMALAGATION is only
    / useful on Windows (to skip DLL hell related oddities)
    */
    #include <spatialite/sqlite3.h>
  #else
    /*
    / You MUST NOT use AMALGAMATION on Linux or any
    / other "sane" operating system !!!!
    */
    #include "sqlite3.h"
  #endif
#else
#include "sqlite3.h"
#endif

#include <stdio.h>
#include <stdlib.h>

#if _MSC_VER
#define snprintf _snprintf
#endif

int main(int argc, char* argv[])
{
    sqlite3* db = NULL;
    char* pszErrMsg = NULL;
    char** papszResult = NULL;
    int nRowCount = 0, nColCount = 0;
    int rc;
    char szBuffer[256];

    if( argc < 2 || argc > 4 )
    {
        fprintf(stderr, "Usage: test_load_virtual_ogr libgdal.so|gdalXX.dll [datasource_name] [layer_name]\n");
        exit(1);
    }

    sqlite3_open(":memory:", &db);
    if( db == NULL )
    {
        fprintf(stderr, "cannot open DB.\n");
        exit(1);
    }

    rc = sqlite3_enable_load_extension(db, 1);
    if( rc != SQLITE_OK )
    {
        fprintf(stderr, "sqlite3_enable_load_extension() failed\n");
        exit(1);
    }

    rc = sqlite3_load_extension(db, argv[1], NULL, &pszErrMsg);
    if( rc != SQLITE_OK )
    {
        fprintf(stderr, "sqlite3_load_extension(%s) failed: %s\n", argv[1], pszErrMsg);
        sqlite3_free( pszErrMsg );
        exit(1);
    }

    rc = sqlite3_get_table( db, "SELECT ogr_version()",
                            &papszResult, &nRowCount, &nColCount,
                            &pszErrMsg );

    if( rc == SQLITE_OK && nRowCount == 1 && nColCount == 1 )
    {
        printf("SELECT ogr_version() returned : %s\n", papszResult[1]);
    }
    else
    {
        fprintf(stderr, "SELECT ogr_version() failed: %s\n", pszErrMsg);
        sqlite3_free( pszErrMsg );
        exit(1);
    }
    sqlite3_free_table(papszResult);

    if( argc >= 3 )
    {
        if( argc == 3 )
        {
            snprintf(szBuffer, sizeof(szBuffer),
                     "CREATE VIRTUAL TABLE foo USING VirtualOGR('%s')",
                     argv[2]);
        }
        else
        {
            snprintf(szBuffer, sizeof(szBuffer),
                     "CREATE VIRTUAL TABLE foo USING VirtualOGR('%s', 0, '%s')",
                     argv[2], argv[3]);
        }

        rc = sqlite3_exec(db, szBuffer, NULL, NULL, &pszErrMsg);
        if( rc != SQLITE_OK )
        {
            fprintf(stderr, "%s failed: %s\n", szBuffer, pszErrMsg);
            sqlite3_free( pszErrMsg );
            exit(1);
        }
        else
        {
            if( argc == 3 )
                printf("Managed to open '%s'\n", argv[2]);
            else
                printf("Managed to open '%s':'%s'\n", argv[2], argv[3]);
        }
    }

    sqlite3_close(db);

    return 0;
}
