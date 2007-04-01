#!/bin/sh

if [ $# -lt 1 ] ; then
  echo "Usage: mkgdaldist.sh version [-date date] [-install] [-nologin]"
  echo
  echo "Example: mkgdaldist.sh 1.1.4"
  exit
fi

GDAL_VERSION=$1
COMPRESSED_VERSION=`echo $GDAL_VERSION | tr -d .`

if test "$GDAL_VERSION" != "`cat VERSION`" ; then
  echo
  echo "NOTE: local VERSION file (`cat VERSION`) does not match supplied version ($GDAL_VERSION)."
  echo "      Consider updating local VERSION file, and commiting to SVN." 
  echo
fi

if test "$2" = "-date" ; then 
  forcedate=$3
  shift
  shift
else
  forcedate=no
fi
  
rm -rf dist_wrk  
mkdir dist_wrk
cd dist_wrk

SVNURL="http://svn.osgeo.org/gdal"
SVNBRANCH="trunk"
SVNMODULE="gdal"

svn checkout ${SVNURL}/${SVNBRANCH}/${SVNMODULE} ${SVNMODULE}

if [ \! -d gdal ] ; then
    echo "svn checkout reported an error ... abandoning mkgdaldist"
  cd ..
  rm -rf dist_wrk
  exit
fi

if test "$forcedate" != "no" ; then
  echo "Forcing Date To: $forcedate"
  rm -f gdal/gcore/gdal_new.h  
  sed -e "/define GDAL_RELEASE_DATE/s/20[0-9][0-9][0-9][0-9][0-9][0-9]/$forcedate/" gdal/gcore/gdal.h > gdal/gcore/gdal_new.h
  mv gdal/gcore/gdal_new.h gdal/gcore/gdal.h
fi

find gdal -name .svn -exec rm -rf {} \;

# Generate SWIG interface for C#
cwd=${PWD}
cd gdal/swig/csharp
./mkinterface.sh
cd ${cwd}

rm -rf gdal/viewer
rm -rf gdal/dist_docs

rm -f gdal/VERSION
echo $GDAL_VERSION > gdal/VERSION

mv gdal gdal-${GDAL_VERSION}

rm -f ../gdal-${GDAL_VERSION}.tar.gz ../gdal${COMPRESSED_VERSION}.zip

tar cf ../gdal-${GDAL_VERSION}.tar gdal-${GDAL_VERSION}
gzip -9 ../gdal-${GDAL_VERSION}.tar
zip -r ../gdal${COMPRESSED_VERSION}.zip gdal-${GDAL_VERSION}

cd ..
rm -rf dist_wrk

TARGETDIR=remotesensing.org:/ftp/remotesensing/pub/gdal
if test "$2" = "-install" ; then

  echo "Installing: " $TARGETDIR/gdal-${GDAL_VERSION}.tar.gz 
  echo "       and: " $TARGETDIR/gdal${COMPRESSED_VERSION}.zip
  scp gdal-${GDAL_VERSION}.tar.gz $TARGETDIR/gdal${COMPRESSED_VERSION}.zip $TARGETDIR
fi
