# ******************************************************************************
# * Project:  CMake4GDAL
# * Purpose:  CMake build scripts
# * Author: Dmitriy Baryshnikov (aka Bishop), polimax@mail.ru
# ******************************************************************************
# * Copyright (C) 2012,2013 Bishop
# * 
# * Permission is hereby granted, free of charge, to any person obtaining a
# * copy of this software and associated documentation files (the "Software"),
# * to deal in the Software without restriction, including without limitation
# * the rights to use, copy, modify, merge, publish, distribute, sublicense,
# * and/or sell copies of the Software, and to permit persons to whom the
# * Software is furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included
# * in all copies or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# * DEALINGS IN THE SOFTWARE.
# ******************************************************************************

cmake_minimum_required (VERSION 2.8.10)

set(LIB_NAME "gcore")
project ("lib${LIB_NAME}")

if(GDAL_USE_CPL)
	add_definitions(-DUSE_CPL)
endif(GDAL_USE_CPL)

option(GDAL_PAM_ENABLED "Set ON to enable pam" ON)
if(GDAL_PAM_ENABLED)
	add_definitions(-DPAM_ENABLED)
endif(GDAL_PAM_ENABLED)

include_directories(${GDAL_ROOT_SOURCE_DIR}/ogr)
include_directories(${GDAL_ROOT_SOURCE_DIR}/alg)
include_directories(${GDAL_ROOT_SOURCE_DIR}/frmts/gtiff)

set(LIB_HHEADERS
    gdal_version.h
    gdal.h
    gdaljp2metadata.h
    gdal_frmts.h
    gdal_pam.h
    gdal_priv.h
    gdal_proxy.h
    gdal_rat.h
	gdalexif.h
)

set(LIB_CSOURCES
    gdalmajorobject.cpp
    gdaldataset.cpp
    gdaldriver.cpp
    rasterio.cpp
    gdalrasterblock.cpp    
    gdalnodatamaskband.cpp
    gdalcolortable.cpp    
    gdalrasterband.cpp    
    gdalallvalidmaskband.cpp    
    gdalnodatavaluesmaskband.cpp    
    gdaldrivermanager.cpp    
    gdaldefaultasync.cpp    
    gdalmultidomainmetadata.cpp    
    gdaldefaultoverviews.cpp    
    gdalopeninfo.cpp    
    gdal_rat.cpp
    gdalproxypool.cpp
    gdalproxydataset.cpp
    overview.cpp
    gdaldllmain.cpp
    gdalgmlcoverage.cpp
    gdal_rpcimdio.cpp
    gdal_misc.cpp
	gdalexif.cpp
	gdaljp2box.cpp
	gdaljp2metadata.cpp
    gdalclientserver.cpp
)

if(GDAL_PAM_ENABLED)
	set(LIB_HHEADERS ${LIB_HHEADERS}
		gdal_pam.h
	)
	set(LIB_CSOURCES ${LIB_CSOURCES}
		gdalpamdataset.cpp
		gdalpamrasterband.cpp
		gdalpamproxydb.cpp
	)
endif(GDAL_PAM_ENABLED)

set(GDAL_TARGET_OBJECTS ${GDAL_TARGET_OBJECTS} $<TARGET_OBJECTS:${LIB_NAME}> PARENT_SCOPE)
add_library(${LIB_NAME} OBJECT ${LIB_HHEADERS} ${LIB_CSOURCES})

if(WIN32)
    install(FILES ${LIB_HHEADERS} DESTINATION ${GDAL_ROOT_SOURCE_DIR}/include/gdal)
else(WIN32)
    install(FILES ${LIB_HHEADERS} DESTINATION include/gdal)
endif(WIN32)	
