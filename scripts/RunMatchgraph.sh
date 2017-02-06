#!/bin/bash
#
# RunMatchgraph.sh
#
# A script for creating a match graph from a set of images
#
# Usage: RunMatchgraph.sh [image_dir]
#
# The image_dir argument is the directory containing the input images.

BINPATH=../bin/KeyMatchGeometryAware
SIFT=../bin/sift
TO_SIFT=./ToSiftSorted.sh

if [ $# -eq 1 ]; 
then
    IMAGE_DIR=$1
else
  echo "[RunMatchgraph] USAGE: ./RunMatchgraph <IMAGE_DIR>"
fi

# Rename ".JPG" to ".jpg"
echo "[- Renaming \".JPG\" to \".jpg\" -]"
for d in `ls -1 $IMAGE_DIR | egrep ".JPG$"`
do
    mv $IMAGE_DIR/$d $IMAGE_DIR/${d/.JPG/.jpg}
done

if [ -d temp ]
then
  rm -r temp
fi
mkdir temp
identify $IMAGE_DIR/*.jpg | cut -d' ' -f3 | sed 's/x/ /g'> temp/dims.init.txt

# Extract SIFT features
touch temp/list_keys.txt
for d in `ls -1 $IMAGE_DIR | egrep "jpg$"`
do 
    pgm_file=$IMAGE_DIR/`echo $d | sed 's/jpg$/pgm/'`
    key_file=$IMAGE_DIR/`echo $d | sed 's/jpg$/key/'`
    gzipped_key_file=$key_file".gz"

    if [ -f $key_file ]; 
    then
      echo "Sorting exisiting key file " $key_file
      key_sorter.sh $key_file;
      echo $key_file >> temp/list_keys.txt
    elif [ -f $gzipped_key_file ]; 
    then
      echo "Found gzipped key, uncompressing and sorting-- " $gzipped_key_file
      gunzip $gzipped_key_file 
      key_sorter.sh $key_file;
      echo $key_file >> temp/list_keys.txt
    else
      mogrify -format pgm $IMAGE1
      $SIFT < $pgm_file > $key_file 
      rm $pgm_file
      key_sorter.sh $key_file;
      echo $key_file >> temp/list_keys.txt
    fi
done

# Match images (can take a while)
echo "[- Matching keypoints (this can take a while) -]"
#$BINPATH --keyfile_list=temp/list_keys.txt --image_dimension_list=temp/dims.init.txt --matches_file=temp/matches.init.txt --twoway_global_match 
$BINPATH --keyfile_list=temp/list_keys.txt --image_dimension_list=temp/dims.init.txt --matches_file=temp/matches.init.txt 

