===============================================================================
       Installation and Usage Guide for Geometry-aware Feature Matcher
             copyright 2015 Rajvi Shah (rajvi.a.shah@gmail.com)
-------------------------------------------------------------------------------

This source package implements a system for fast and accurate geometry-aware 
matching of SIFT-like features as described in the following paper,

1)  Geometry-aware Feature Matching for Structure from Motion Application,
    Rajvi Shah, Vanshika Srivastava, P J Narayanan. [WACV 2015]

Application of geometry-aware feature matching for a multistage structure from  
motion framework is described in the following paper,

2)  Multistage SFM : Revisiting Incremental Structure from Motion. 
    Rajvi Shah, Aditya Deshpande, P J Narayanan [3DV 2014]

For more information and latest version of the code visit,
http://cvit.iiit.ac.in/projects/multistagesfm/

THE SOFTWARE IS PROVIDED "AS IS" AND WITHOUT WARRANTY OF ANY KIND, EXPRESSED, 
IMPLIED OR OTHERWISE.

You can use, copy, modify and distribute this software and its documentation 
for non-commercial purpose without fee, provided that the copyright notices 
appear in all copies of it. If you do so, please cite our WACV'15 paper.  

You can write to Rajvi Shah (rajvi.a.shah@gmail.com) for reporting bugs or for  
doubts/clarification. 

===============================================================================
**CAUTION** : Where is this matching expected to work?
-------------------------------------------------------------------------------
This matching code leverages epipolar geometry between two images for fast and 
accurate feature matching. This makes its application suitable for SFM and 
stereo vision. It will not produce matches if the geometry between the two 
images cannot be captured by epipolar geometry (more precisely a fundamental 
matrix), for example images of a planar scene, or captured by a camera rotating 
around a fixed point (images related by a single Homography).

===============================================================================
I. Source files in this package
-------------------------------------------------------------------------------
The following files provide example code for using the matcher code for simple 
pair-wise matching or match-graph construction from multiple images,
|-- match_image_pair.cpp
|-- match_graph.cpp

This following source files implements the core functionality of this package,
|-- defs.h
    Common definitions and includes
|-- Matcher.h, Matcher.cpp  
    Class with various matching, and other utility functions. 
|-- Gridder.h, Gridder.cpp
    Class responsible for dividing image features into bins. 
    Stores maps from feature to cell and vice versa.
|-- Geometric.h, Geometric.cpp
    Class responsible for various geometric functions.

For reading and representing SIFT keyfiles, we use the code by Noah Snavely,
Original source : http://www.cs.cornell.edu/~snavely/bundler/
|-- keys2a.h, keys2a.cpp
    
This package also uses the argument parsing code by Michael Hanke,
Original source : http://mih.voxindeserto.de/argvparser.html
|-- argvparser.h, argvparser.cpp

===============================================================================
II. About the third party libraries, binaries and helper scripts
-------------------------------------------------------------------------------
OpenCV
------
Our code uses OpenCV for fundamental matrix computation. If you already have 
opencv version 2.4 or higher installed, please ensure that pkg-config can find 
the correct paths to its lib and include files. Use the following commands to 
test if the opencv include paths and libs are properly displayed.

	> pkg-config --cflags opencv
	> pkg-config --libs opencv

For fresh installation of opencv, please follow,
(Ubuntu)  https://help.ubuntu.com/community/OpenCV
(General) http://docs.opencv.org/trunk/doc/tutorials/introduction/linux_install/linux_install.html  

ANN_1_1_CHAR
------------
A version of the approximate nearest neighbors (ANN) library of David M. Mount 
and Sunil Arya, customized for searching verctors of unsigned bytes, is included
in 'lib' directory. (Original code source : http://www.cs.umd.edu/~mount/ANN/).
The customized version, included here is borrowed from Bundler distribution by 
Noah Snavely. (Bundler source: http://www.cs.cornell.edu/~snavely/bundler/).

SIFT Binary
------------
A binary for SIFT feature extraction is also included in the 'bin' directory. 
This binary is not necessary for the functioning of our code but it is included 
as a helper uility. This binary along with other utility scripts is originally 
shared by David Lowe : http://www.cs.ubc.ca/~lowe/keypoints/. Our code is tested 
only using features extracted using Lowe's Sift binary. However, you are free to 
use any feature extractor of your choise as long as you write the description 
files in Lowe's ASCII format. Our code does not depend on this binary.

zlib
----
The files for reading keys2a.h and keys2a.cpp also support reading gzipped file.
For this purpose, it requires zlib to be installed. You can either compile it 
from source (http://zlib.net/) or use the version included with this package 
under 'lib/zlib' directory.  
 
Bundler
-------
Bundler is a system for Structure-from-motion 3D reconstruction from a set of 
images (source: http://www.cs.cornell.edu/~snavely/bundler/). Our code does not 
require Bundler to be installed. However, our code is written to be compatible 
with Bundler for match-graph construction purpose. See (IV) for information on 
how to use our code with Bundler for SFM applications.

===============================================================================
III. How to compile our code?
-------------------------------------------------------------------------------
The source code is written and tested under 64-bit Linux environment. Please 
follow the step-by-step instructions given below to successfully compile it,

1.  Make sure, OpenCV is correctly installed as explained above.

2.  Verify that the zlib and ann_1_1_char libraries and include files exist at 
    the path specified in the Makefile (src/Makefile).

3.  > export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.../lib/ann_1.1_char/lib/
    Expand "..." to your system path to "matching_v0.0.1" directory.
    Add this line to your bashrc file to avoid this step the next time.

3.  > cd .../matching_v0.0.1/src
    > make

    This creates two binaries in .../matching_v0.0.1/bin directory.
    For computing match-graph: 'bin/KeyMatchGeometryAware'
    For matching an image-pair: 'bin/match_pair'    

===============================================================================
IV. How to use this code with Bundler?
-------------------------------------------------------------------------------
This package includes two scripts for using our matching code with Bundler for 
a full structure-from-motion pipeline. To use our code with Bundler,

1. After compiling our code, copy bin/KeyMatchGeometry to BUNDLER_BIN_PATH
2. Copy scripts/key_sorter.sh, scripts/ToSiftSorted.sh to BUNDLER_BIN_PATH
3. In RunBundler.sh and change the BASE_PATH to your bundler installation path.
4.  > mkdir reconstruction
5.  > cd reconstruction
6.  > $.../matching_v0.0.1/scripts/RunBundler.sh <PathToImages>

===============================================================================
V. How to independently use the binary for creating match-graph
-------------------------------------------------------------------------------
For Quick Use
-------------
> cd .../matching_v0.0.1/scripts/
> ./RunMatchgraph [PATH_TO_IMAGE_DIR]

Detailed Guide
--------------
You can also use our code independently to create a match-graph for a set of 
images. You can extract SIFT/SIFT-like features for the images to be matched, 
and write these features in sorted order of scale in david lowe's ASCII format 
(plaintext or gzipped). If you are using lowe's binary for SIFT extraction, you 
can use key_sorter.sh to sort and store these feature files as follows,

> ./key_sorter.sh <key_file_name>

This script shuffles the keys in the original file in descending order of scale. 
Our code can be used to create a match-graph from these extracted features.

The binary for match-graph creation takes the following options,
  -h, --help
  Prints usage of options and expected values.

  --keyfile_list [required]
  Path to a list of key files (full paths) in LOWE'sformat (ASCII or gzipped)

  --image_dimension_list [required]
  Filename with path, file stores <Height Width> per image per line in same
  order as the keyfiles

  This can be created quickly using the image magick command 'identify',
  > identify $IMAGE_DIR/*.jpg | cut -d' ' -f3 | sed 's/x/ /g'> dims.init.txt

  --matches_file [required]
  Output filename with path, file stores key matches in format <im1 im2\n
  num_matches\n keyIdx1 keyIdx2\n...> 

  --topscale_percent
  Percentage of top scale features to use for initial matching, [Default: 20]

  --twoway_global_match
  Use two-way matching for top-scalefeatures (stricter, slow), [Default: False]

These options can be specified in an options file or as a series of command line 
arguments in options-value format as shown below.

-------------
Example Usage 
-------------
> KeyMatchGeometryAware --option_file=matchgraph.options.txt
> KeyMatchGeometryAware --keyfile_list=list_keys.txt --image_dimension_list=
dims.init.txt --matches_file=matches.init.txt 

===============================================================================
VI. How to independently use the binary for pairwise matching
-------------------------------------------------------------------------------
For Quick Use
-------------
> cd .../matching_v0.0.1/scripts/
> ./RunPairwise [PATH_TO_IMAGE1] [PATH_TO_IMAGE2]

Detailed Guide
--------------
To simply match an image pair, please compile the binary for pairwise matching.
As explained before, you need to extract/store SIFT/SIFT-like features for the 
images to be matched, in sorted order of scale in david lowe's ASCII format 
(plaintext or gzipped). If you are using lowe's binary for SIFT extraction, you 
can use key_sorter.sh to sort and store these feature files as follows,

> ./key_sorter.sh <key_file_name>

The binary for pairwise matching takes the following options,
    -h, --help
    Prints usage of options and expected values.

    --source_image [required]
    <Path to Source Image File>

    --target_image [required]
    <Path to Target Image File>

    --source_key [required]
    <Path to Source Key File>

    --target_key [required]
    <Path to Target Key File>

    --source_dimension [required]
    <Source Image WxH>

    --target_dimension [required]
    <Target Image WxH>

    --result_path 
    <Path to save results, make sure the specified directory exists>

    -v, --visualize
    Enables visualization of matches

    -s, --save_visualization
    Saves Visualization

These options can be specified in an options file or as a series of command 
line arguments in options-value format as shown here.

-------------
Example usage
-------------
> match_pairs --option_file=desk.pairwise.opt
See .../matching_v0.0.1/data/pairs/desk.pairwise.opt for an example.

> match_pairs --source_image=../data/hampi/1.jpg --source_key=../data/hampi/1.
key --source_dimension=3000x2250 --target_image=../data/hampi/2.jpg --target_key
=../data/hampi/2.key  --target_dimension=3000x2250 --visualize 
--save_visualization --result_path=../results/pairwise/hampi/

===============================================================================
For Questions/Suggestions/Help contact us
-------------------------------------------------------------------------------

Rajvi Shah (rajvi.a.shah@gmail.com)
Vanshika Srivastava (vanshika.srivastava.iiith@gmail.com)
P J Narayanan (pjn@iiit.ac.in)
