#!/bin/bash

make -C test test
rc=$?

if [ $rc -ne 0 ]; then
    growlnotify -a Xcode -n simple-ci -d bravo5.simpleci -t CI -m "BUILD FAILED" 
# else
#     growlnotify -a Xcode -n simple-ci -d bravo5.simpleci -m "Success" 
fi

exit $rc
