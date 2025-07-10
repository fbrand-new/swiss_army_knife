#!/bin/bash

# This fixes trailing whitespace in the specified files.

# find -o is for the or logic
FILES=""
for f in files;
do
  if [ -f "$f" ]; then
    echo $f | xargs -I {} sed -i 's/[[:space:]]*$//' {}
  fi
done

# Original command reference
# find . -name "*.cmake.in" -o -name "CMakeLists.txt" | grep -E ${PATTERN} | xargs -I {} sed -i 's/[[:space:]]*$//' {}
