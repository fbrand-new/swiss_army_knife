#!/bin/bash

line_1="// SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia"
line_2="// SPDX-License-Identifier: BSD-3-Clause\n"

# Read list of files from stdin
echo "Reading files from stdin..."
while IFS= read -r file; do
    if [ -n "$file" ]; then  # Check if line is not empty
        echo "Processing file: $file"
        sed -i "1i\\${line_2}" "$file"
        sed -i "1i\\${line_1}" "$file"
    fi
done 