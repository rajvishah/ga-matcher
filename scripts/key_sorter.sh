#!/bin/bash
filename="$1"
echo "1" > ids.txt
(awk '{if((NR-2)%8==0) print $3 " " (NR)}' "$filename" | sort -r -nk1 | awk '{for(c=0;c<8;c++) print $2+c " " (NR-1)*8+2+c}' | sort -nk1 | awk '{print $2}') >> ids.txt
paste ids.txt "$filename" | sort -nk1 | cut -f 2- > sorted
cp sorted "$filename"
rm ids.txt sorted
