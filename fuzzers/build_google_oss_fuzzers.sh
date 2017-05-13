#!/bin/bash

if [ "$SRC" == "" ]; then
    echo "SRC env var not defined"
    exit 1
fi

if [ "$OUT" == "" ]; then
    echo "OUT env var not defined"
    exit 1
fi

if [ "$CXX" == "" ]; then
    echo "CXX env var not defined"
    exit 1
fi

SRC_DIR=$(dirname $0)/..
fuzzerFiles=$(find $(dirname $0) -name "*.cpp")
mkdir -p $SRC/install/lib
cp $SRC_DIR/libgdal.a $SRC/install/lib
for F in $fuzzerFiles; do
    fuzzerName=$(basename $F .cpp)
    echo "Building fuzzer $fuzzerName"
    $CXX $CXXFLAGS -std=c++11 -I$SRC_DIR/port -I$SRC_DIR/gcore -I$SRC_DIR/alg -I$SRC_DIR/ogr -I$SRC_DIR/ogr/ogrsf_frmts \
        $F -o $OUT/$fuzzerName $SRC/install/lib/*.a
done
