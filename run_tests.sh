#!/bin/sh

BUILDDIR=BUILD_TEST
TESTNAME=bridge_test

abort () {
    rm -fR "$BUILDDIR"
    echo "Test run failed. Exiting..." >&2
    exit 1
}

trap 'abort' 0

set -e

if [ -d "$BUILDDIR" ]; then
    rm -rR "$BUILDDIR"
fi

mkdir "$BUILDDIR"
gcc -c ringbuf.c -o "$BUILDDIR"/ringbuf.o -I.
g++ -o "$BUILDDIR"/"$TESTNAME" test/test_bridge.cpp "$BUILDDIR"/ringbuf.o -I. -L"$BUILDDIR" -lgtest -lgtest_main -pthread -std=c++0x
"$BUILDDIR"/"$TESTNAME"

trap : 0
