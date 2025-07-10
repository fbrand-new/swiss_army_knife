#!/bin/bash

FILES="a.txt b.txt"
for file in $FILES;
do
  echo "" >> "$file"
done