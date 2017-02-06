#!/bin/bash
#
# RunPairwise.sh
#   copyright 2015 Rajvi Shah
#
# A script for preparing options and running pairwise matching 

if [ $# -ne 2 ]
then
  echo "[RunPairwise] USAGE: ./RunPairwise.sh <ImagePath1> <ImagePath2>"  
  exit
fi

IMAGE1=$1
IMAGE2=$2
BINPATH=../bin/match_pairs
SIFT=../bin/sift

pgm_file=`echo $IMAGE1 | sed 's/jpg$/pgm/'`
key_file=`echo $IMAGE1 | sed 's/jpg$/key/'`
gzipped_key_file=$key_file".gz"

if [ -f $key_file ]; 
then
  echo "Sorting exisiting key file " $key_file
  key_sorter.sh $key_file;
elif [ -f $gzipped_key_file ]; 
then
  echo "Found gzipped key, uncompressing and sorting --" $gzipped_key_file
  gunzip $gzipped_key_file 
  key_sorter.sh $key_file;
else
  mogrify -format pgm $IMAGE1
  $SIFT < $pgm_file > $key_file 
  rm $pgm_file
fi

echo --source_image=$IMAGE1 > options.txt
echo --source_key=$key_file >> options.txt
echo --source_dimension=`identify $IMAGE1 | cut -d' ' -f3` >> options.txt

pgm_file=`echo $IMAGE2 | sed 's/jpg$/pgm/'`
key_file=`echo $IMAGE2 | sed 's/jpg$/key/'`
gzipped_key_file=$key_file".gz"

if [ -f $key_file ]; 
then
  echo "Sorting exisiting key file " $key_file
  key_sorter.sh $key_file;
elif [ -f $gzipped_key_file ]; 
then
  echo "Found gzipped key, sorting exisiting gzipped key file" $gzipped_key_file
  gunzip $gzipped_key_file 
  key_sorter.sh $key_file;
  gzip -f $key_file
else
  mogrify -format pgm $IMAGE2; 
  $SIFT < $pgm_file > $key_file; 
  rm $pgm_file
fi

echo --target_image=$IMAGE2 >> options.txt
echo --target_key=$key_file >> options.txt
echo --target_dimension=`identify $IMAGE2 | cut -d' ' -f3` >> options.txt

echo "--result_path=../results/" >> options.txt
#echo "--visualize" >> options.txt
#echo "--save_visualization" >> options.txt

$BINPATH --options_file=options.txt
