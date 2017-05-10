#!/bin/sh

if [ $# -ne 1 ]; then
    echo "Error: one input parameter expected, the RT Plugin name"
    exit 1
fi

# SVN HACK thanks to gituhb (from master)
svn export https://gitlab.advrcloud.iit.it/advr_humanoids/XCM/tree/skeleton/skeleton > /dev/null 
mv control/* . && rm -rf control

# find and replace 
find . -maxdepth 3 -type f -not -path '*/\.*' -exec sed -i -e "s/_MODULE_PREFIX_/$1/g" {} \;
# rename
find . -maxdepth 3 -type f -not -path '*/\.*' -not -name "CMakeLists.txt" -not -name "README.md" -exec bash -c 'dir=$(dirname $0) && file=$(basename $0) && mv $0 "$dir/$1_$file"' {} $1 \;

echo "XBot RT Plugin skeleton is ready. Have fun!"