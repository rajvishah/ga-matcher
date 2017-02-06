#!/bin/bash
#
# RunBundler.sh
#   copyright 2008 Noah Snavely
#
# A script for preparing a set of image for use with the Bundler 
# structure-from-motion system.
#
# Usage: RunBundler.sh [image_dir]
#
# The image_dir argument is the directory containing the input images.
# If image_dir is omitted, the current directory is used.
#

# Set this variable to your base install path (e.g., /home/foo/bundler)
# Modified version by Rajvi Shah

time_beg_all=`date +%s`

BASE_PATH=/home/rajvi/PhD/Projects/MultistageSfM/bundler/

if [ $BASE_PATH == "TODO" ]
then
    echo "Please modify this script (RunBundler.sh) with the base path of your bundler installation.";
    exit;
fi

EXTRACT_FOCAL=$BASE_PATH/bin/extract_focal.pl
TO_SIFT=$BASE_PATH/bin/ToSift.sh

OS=`uname -s`

MATCHKEYS=$BASE_PATH/bin/KeyMatchGeometryAware
BUNDLER=$BASE_PATH/bin/bundler
BUNDLE2PMVS=$BASE_PATH/bin/Bundle2PMVS
GENOPTION=$BASE_PATH/bin/genOption
CMVS=$BASE_PATH/bin/cmvs
POISSONRECON=$BASE_PATH/bin/PoissonRecon

if [ $# -eq 1 ]; then
    IMAGE_DIR=$1
else
    IMAGE_DIR="."
fi
echo "Using directory '$IMAGE_DIR'"

# Rename ".JPG" to ".jpg"
echo "[- Renaming \".JPG\" to \".jpg\" -]"
for d in `ls -1 $IMAGE_DIR | egrep ".JPG$"`
do
    mv $IMAGE_DIR/$d $IMAGE_DIR/${d/.JPG/.jpg}
done

identify $IMAGE_DIR/*.jpg | cut -d' ' -f3 | sed 's/x/ /g'> dims.init.txt

# Create the list of images and extract focal information
echo "[- Creating images' list -]"
find $IMAGE_DIR -maxdepth 1 | egrep ".jpg$" | sort > list_tmp.txt
$EXTRACT_FOCAL list_tmp.txt
cp prepare/list.txt .

# Run the ToSift script to generate a list of SIFT commands and execute
echo "[- Extracting keypoints -]"
time_beg_sift=`date +%s`
rm -f sift.txt
$TO_SIFT $IMAGE_DIR > sift.txt 
#for d in `ls -1 $IMAGE_DIR | egrep "key.gz$"`
#do 
#    rm -rf $IMAGE_DIR/$d
#done
#sh sift.txt
time_end_sift=`date +%s`
time_sift=`expr $time_end_sift - $time_beg_sift`

# Match images (can take a while)
echo "[- Matching keypoints (this can take a while) -]"
time_beg_match=`date +%s`
sed 's/\.jpg$/\.key/' list_tmp.txt > list_keys.txt
sleep 1
echo $MATCHKEYS --keyfile_list=list_keys.txt --image_dimension_list=dims.init.txt --matches_file=matches.init.txt 
$MATCHKEYS --keyfile_list=list_keys.txt --image_dimension_list=dims.init.txt --matches_file=matches.init.txt 
time_end_match=`date +%s`
time_match=`expr $time_end_match - $time_beg_match`

# Generate the options file for running bundler 
mkdir bundle
rm -f options.txt
echo "--match_table matches.init.txt" >> options.txt
echo "--output bundle.out" >> options.txt
echo "--output_all bundle_" >> options.txt
echo "--output_dir bundle" >> options.txt
echo "--variable_focal_length" >> options.txt
echo "--use_focal_estimate" >> options.txt
echo "--constrain_focal" >> options.txt
echo "--constrain_focal_weight 0.0001" >> options.txt
echo "--estimate_distortion" >> options.txt
echo "--run_bundle" >> options.txt

report_content="Time Report\n"
report_content=$report_content"--- Extract Sift Points: "$time_sift" second(s)\n"
report_content=$report_content"--- Match Sift Points:   "$time_match" second(s)\n"
echo -e $report_content > $IMAGE_DIR/time_report_geo_aware.txt

# Run Bundler!
echo "[- Running Bundler -]"
time_beg_bundler=`date +%s`
rm -f constraints.txt
rm -f pairwise_scores.txt
$BUNDLER list.txt --options_file options.txt > bundle/out
time_end_bundler=`date +%s`
time_bundler=`expr $time_end_bundler - $time_beg_bundler`

# Prepare For PMVS
echo "[- Prepare For PMVS -]"
time_beg_bundle2pmvs=`date +%s`
$BUNDLE2PMVS list.txt ./bundle/bundle.out pmvs 
time_end_bundle2pmvs=`date +%s`
time_bundle2pmvs=`expr $time_end_bundle2pmvs - $time_beg_bundle2pmvs`

# Patch and run prep_pmvs.sh
echo "[- Running prep_pmvs.sh -]"
time_beg_prep_pmvs=`date +%s`
KEYWORD="BUNDLER_BIN_PATH="
REPLACE=$(echo "BUNDLER_BIN_PATH=$BASE_PATH/bin" | sed -e 's/\//\\\//g')
sed -e "s/${KEYWORD}/${REPLACE}/g" pmvs/prep_pmvs.sh > prep_pmvs.sh
sh ./prep_pmvs.sh
time_end_prep_pmvs=`date +%s`
time_prep_pmvs=`expr $time_end_prep_pmvs - $time_beg_prep_pmvs`

# Run CMVS
echo "[- Running CMVS -]"
time_beg_cmvs=`date +%s`
#rm -rf $IMAGE_DIR/bundle
#mv ./bundle $IMAGE_DIR/
#rm -rf $IMAGE_DIR/prepare
#mv ./prepare $IMAGE_DIR/
#mv -f ./list*.txt $IMAGE_DIR/
#mv -f ./matches.*.txt $IMAGE_DIR/
#mv -f ./nmatches.*.txt $IMAGE_DIR/
#mv -f ./options.txt $IMAGE_DIR/
#mv -f ./sift.txt $IMAGE_DIR/
#mv -f ./constraints.txt $IMAGE_DIR/
#mv -f ./pairwise_scores.txt $IMAGE_DIR/
#$CMVS $IMAGE_DIR/pmvs/
$CMVS pmvs/
time_end_cmvs=`date +%s`
time_cmvs=`expr $time_end_cmvs - $time_beg_cmvs`

# Run GenOption
echo "[- Running GenOption -]"
time_beg_gen_option=`date +%s`
$GENOPTION pmvs/
echo "All right"

cat "pmvs/pmvs.sh" |awk '{if($1=="pmvs2") {out="'$BASE_PATH'/bin/" $1 " " $2 " "$3;print out}}' > pmvs.sh
#cat "pmvs/pmvs.sh" |awk '{if($1=="pmvs2") {out="'$BASE_PATH'/bin/" $1 " '$IMAGE_DIR'/" $2 " "$3;print out}}' 

echo "All right"

sh ./pmvs.sh
mv -f ./prep_pmvs.sh $IMAGE_DIR/
mv -f ./pmvs.sh $IMAGE_DIR/
time_end_gen_option=`date +%s`
time_gen_option=`expr $time_end_gen_option - $time_beg_gen_option`

# Run PoissonRecon
#echo "[- Running PoissionRecon -]"
#time_beg_poission_recon=`date +%s`
#cat "pmvs/pmvs.sh" |awk '{if($1=="pmvs2") {out="'$BASE_PATH'/bin/" $1 " '$IMAGE_DIR'/" $2 " "$3;print out}}' > pmvs.sh
#$POISSONRECON --in pmvs/models/option-0000.ply --out pmvs/models/poissonrecon.ply --depth 10 --pointWeight 0
#time_end_poission_recon=`date +%s`
#time_poission_recon=`expr $time_end_poission_recon - $time_beg_poission_recon`

echo "[- Done -]"
time_end_all=`date +%s`
time_all=`expr $time_end_all - $time_beg_all`

# Generate Time Report
report_content="Time Report\n"
report_content=$report_content"--- Extract Sift Points: "$time_sift" second(s)\n"
report_content=$report_content"--- Match Sift Points:   "$time_match" second(s)\n"
report_content=$report_content"--- Run Bundler:         "$time_bundler" second(s)\n"
report_content=$report_content"--- Run Bundle2PMVS:     "$time_bundle2pmvs" second(s)\n"
report_content=$report_content"--- Run prev_pmvs.sh:    "$time_prep_pmvs" second(s)\n"
report_content=$report_content"--- Run CMVS:            "$time_cmvs" second(s)\n"
report_content=$report_content"--- Run GenOption:       "$time_gen_option" second(s)\n"
#report_content=$report_content"--- Run PoissionRecon:   "$time_poission_recon" second(s)\n"
report_content=$report_content"--- Overall:             "$time_all" second(s)"
echo -e $report_content >> time_report_geometry_aware.txt
