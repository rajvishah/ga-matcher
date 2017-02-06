#include "defs.h"
#include "Matcher.h"
#include "keys2a.h"
#include "Gridder.h"
#include "Geometric.h"
#include "argvparser.h"

#include<opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include <string>
using namespace cv;
using namespace match;
using namespace CommandLineProcessing;


int timeval_subtract(struct timeval *result, 
        struct timeval *t2, 
        struct timeval *t1) {
    long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
    result->tv_sec = diff / 1000000;
    result->tv_usec = diff % 1000000;

    return (diff<0);
}


void writeMatchesToFile(FeatureMatcher &matcher, const char* fileName) {

  FILE* fn = fopen(fileName,"w");
  if(fn == NULL) {
    printf("\nCould not open file to write");
  }
  for(int i=0; i < matcher.matches.size(); i++) {
    fprintf(fn,"%d %d\n", matcher.matches[i].first, matcher.matches[i].second);
  }
  fclose(fn); 
}

void SetupCommandlineParser(ArgvParser& cmd, int argc, char* argv[]) {
  cmd.setIntroductoryDescription("Pair-wise feature matching example");

  //define error codes
  cmd.addErrorCode(0, "Success");
  cmd.addErrorCode(1, "Error");

  cmd.setHelpOption("h", "help",""); 

  cmd.defineOption("source_image", "<Path to Source Image File>", 
      ArgvParser::OptionRequired);
                  
  cmd.defineOption("target_image", "<Path to Target Image File>", ArgvParser::OptionRequired);


  cmd.defineOption("source_key","<Path to Source Key File>",ArgvParser::OptionRequired); 

  cmd.defineOption("target_key", "<Path to Target Key File>", ArgvParser::OptionRequired);
                   
  cmd.defineOption("source_dimension", "<Source Image WxH>",ArgvParser::OptionRequired);

  cmd.defineOption("target_dimension", "<Target Image WxH>", ArgvParser::OptionRequired);
                   
  cmd.defineOption("result_path", "<Path to save results>", ArgvParser::NoOptionAttribute);

  cmd.defineOption("visualize", "enables visualization of matches", ArgvParser::NoOptionAttribute);
  cmd.defineOptionAlternative("visualize","v");

  cmd.defineOption("save_visualization", "Saves Visualization", ArgvParser::NoOptionAttribute);
  cmd.defineOptionAlternative("save_visualization","s");

  string dummyArgv;
  vector< char* > argstr;

  bool optionsFile = false;
  if(argc == 2) {
    string str = argv[1];
    int pos = str.find("=");
    string optStr = str.substr(0,pos);
    if(optStr == "-o" || optStr == "--options_file") {
      string optFileName = str.substr(pos+1, str.length());
      std::ifstream optFile(optFileName.c_str());
      if(!optFile.is_open()) {
        cout << "\nError opening options file " << optFileName; 
        exit(-1);
      }

      int count =0;
      std::string option(argv[0]);  
      do {
        char *arg = new char[option.size() + 1];
        copy(option.begin(), option.end(), arg);
        arg[option.size()] = '\0';
        argstr.push_back(arg);
      } while(std::getline(optFile, option));
      optionsFile = true;
    }
  } 
  
  /// finally parse and handle return codes (display help etc...)

  int result = -1;
  if(optionsFile) {
    result = cmd.parse(argstr.size(), argstr.data());
  } else {
    result = cmd.parse(argc, argv);
  }

  if (result != ArgvParser::NoParserError)
  {
    cout << cmd.parseErrorDescription(result);
    exit(1);
  }

}

bool findMatches(ArgvParser& cmd) {
  // now query the parsing results
  string imagePath1  = cmd.optionValue("source_image");
  string imagePath2  = cmd.optionValue("target_image");

  string keyPath1  = cmd.optionValue("source_key");
  string keyPath2  = cmd.optionValue("target_key");

  string source_dim  = cmd.optionValue("source_dimension");
  string target_dim  = cmd.optionValue("target_dimension");

  string delimiter = "x";
  int pos = source_dim.find(delimiter);

  string qWidth = source_dim.substr(0, pos); 
  string qHeight = source_dim.substr(pos+1,source_dim.length());

  string rWidth = source_dim.substr(0, pos); 
  string rHeight = source_dim.substr(pos+1,target_dim.length());
 
  bool visualize = cmd.foundOption("visualize");
  bool save_visualization = cmd.foundOption("save_visualization");

  string result_path("");
  if(cmd.foundOption("result_path")) {
    result_path = cmd.optionValue("result_path"); 
    //printf("\nResult path is %s", result_path.c_str());
  }

  /// Assignment to variables
  int qH = atoi(qHeight.c_str());
  int qW = atoi(qWidth.c_str());
  int rH = atoi(rHeight.c_str());
  int rW = atoi(rWidth.c_str());

  unsigned char* queryKey;
  unsigned char* refKey;
  keypt_t* queryKeyInfo;
  keypt_t* refKeyInfo;

  int nPts1 = ReadKeyFile(keyPath1.c_str(),
      &queryKey, &queryKeyInfo);

  int nPts2 = ReadKeyFile(keyPath2.c_str(),
      &refKey, &refKeyInfo);

  struct timeval t1, t2, t3;
  gettimeofday(&t1, NULL);
  
  /// Initialize Gridder
  Gridder srcGrid(16, qW, qH, nPts1, queryKeyInfo);
  Gridder refGrid(16, rW, rH, nPts2, refKeyInfo);
  
  vector< vector<double> > refRectEdges;
  vector< vector<double> > srcRectEdges;

  /// Precompute Four Edges of Image Rectangles
  geometry::ComputeRectangleEdges((double)rW, (double)rH, refRectEdges);
  geometry::ComputeRectangleEdges((double)qW, (double)qH, srcRectEdges);

  /// Initialize Matcher
  match::FeatureMatcher matcher;
  matcher.queryImage = imread(imagePath1.c_str(),
      CV_LOAD_IMAGE_COLOR);

  matcher.referenceImage = imread(imagePath2.c_str(),
      CV_LOAD_IMAGE_COLOR);

  matcher.setNumSrcPoints( nPts1 );
  matcher.setSrcKeys(queryKeyInfo, queryKey);

  matcher.setImageDims(qW, qH, rW, rH);

  matcher.setNumRefPoints( nPts2 );
  matcher.setRefKeys(refKeyInfo, refKey);

  matcher.setSrcRectEdges(srcRectEdges);
  matcher.setRefRectEdges(refRectEdges);

  /// Find F estimate using 20% top features
  /// First perform global Kd-tree based matching
 
  matcher.globalMatch(20, true);
  if(matcher.matches.size() < 16) {
    printf("\nCould Not Find 16 Matches: Found %d\n", matcher.matches.size());
    return false;
  }

  vector<double> fMatrix(9);
  double* F_ptr = fMatrix.data();
  int inliers = matcher.computeFmatrix(F_ptr);
  if(inliers == -1) { 
    printf("\nCould Not Find 16 Inliers to computed F, Exiting");
    return false;
  }

  matcher.setQueryGrid(&srcGrid);
  matcher.setRefGrid(&refGrid);
  matcher.setFMatrix(fMatrix); 
  
  /// Actual Computation
  matcher.computeEpipolarLines();
  matcher.clusterPointsFast();

  int numMatches = matcher.match();
  gettimeofday(&t2, NULL);

  /// For two-way matching, initialize another instance of matcher
  /// Initialize with opposite source and target
  /// Pass transpose(F) as F matrix
  /// Find common matches between matcher and matcher1
  


  timeval_subtract(&t3,&t2,&t1);

  printf("\nTotal Time to Match : %ld.%06d", t3.tv_sec, t3.tv_usec);
  printf("\nFound %d matches between %d %d points\n", numMatches, nPts1, nPts2);
  fflush(stdout);

  if(visualize) {
    if(save_visualization) {
      string str = result_path + "matches.jpg";
      matcher.visualizeMatches(str.c_str());
    } else {
      matcher.visualizeMatches(NULL); 
    }
  }

  string str = result_path + "matches.geoaware.txt";
  writeMatchesToFile(matcher, str.c_str());
  return true;
}

int main(int argc, char* argv[]) {
  ArgvParser cmd;
  SetupCommandlineParser(cmd, argc, argv);
  bool status = findMatches(cmd);
  return 0;
}
