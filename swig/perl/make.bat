dmake -f Makefile_Geo__GDAL %1

if "%1" == "test" goto clean

dmake -f Makefile_Geo__GDAL__Const %1
dmake -f Makefile_Geo__OGR %1
dmake -f Makefile_Geo__OSR %1

if "%1" == "clean" goto clean

goto end

:clean
echo off
del tmp_ds_*
del *.gmt
rd tmp_ds_ESRIShapefile /S /Q
rd tmp_ds_MapInfoFile /S /Q
rd tmp_ds_KML /S /Q
rd tmp_ds_BNA /S /Q
rd tmp_ds_GMT /S /Q
rd tmp_ds_GPX /S /Q
rd tmp_ds_GeoJSON /S /Q

if "%1" == "test" goto end

del *.c
del *.cpp
del *.old
rd lib /S /Q

:end
