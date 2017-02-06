#include "defs.h"
#include "Matcher.h"
#include "keys2a.h"
#include "Gridder.h"
#include "Geometric.h"
#include "argvparser.h"

#include <time.h>

using namespace cv;
using namespace match;
using namespace CommandLineProcessing;

void SetupCommandlineParser(ArgvParser& cmd, int argc, char* argv[]) {
  cmd.setIntroductoryDescription("Match-graph Construction");

  //define error codes
  cmd.addErrorCode(0, "Success");
  cmd.addErrorCode(1, "Error");

  cmd.setHelpOption("h", "help",""); 

  cmd.defineOption("keyfile_list", "List of key files (full paths) in LOWE's" 
      "format (ASCII text or gzipped)", ArgvParser::OptionRequired);
                  
  cmd.defineOption("image_dimension_list", "Filename with path, file stores " 
      "<Height Width> per image per line in same order as the keyfiles", 
      ArgvParser::OptionRequired);

  cmd.defineOption("matches_file", "Filename with path, file stores " 
      "key matches in format <im1 im2\\n num_matches\\n keyIdx1 keyIdx2\\n...", 
      ArgvParser::OptionRequired);

  cmd.defineOption("topscale_percent", "Percentage of top scale features"
      "to use for initial matching, [Default: 20]", ArgvParser::NoOptionAttribute); 
                   
  cmd.defineOption("twoway_global_match", "use two-way matching for top-scale" 
      "features (stricter, slow), [Default: False]", ArgvParser::NoOptionAttribute);

  /// If instead of arguments, options file is supplied
  /// Parse options file to fill-up a dummy argv struct
  /// Parse the dummy argv struct to get true arguments
  vector< char* > argstr;
  bool optionsFile = false;

  if(argc == 2) {
    string str( argv[1] );
    
    /// Find position of = to separate option & value
    int pos = str.find("=");
    string optStr = str.substr(0,pos); //option string

    /// Check if the passed option is that of options file
    if(optStr == "-o" || optStr == "--options_file") {
      
      string optFileName = str.substr(pos+1, str.length());
      std::ifstream optFile(optFileName.c_str());
      if(!optFile.is_open()) {
        cout << "\nError opening options file " << optFileName; 
        exit(-1);
      }

      /// Fill argstr with text from options file
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
    printf("\nProblem is here");
    fflush(stdout);
    cout << cmd.parseErrorDescription(result);
    exit(-1);
  }
}

int main(int argc, char* argv[]) {

  ArgvParser cmd;
  SetupCommandlineParser(cmd, argc, argv);

  string keyList = cmd.optionValue("keyfile_list");
  string dimList = cmd.optionValue("image_dimension_list");
  string matchFileName = cmd.optionValue("matches_file");

  int topscale = 20;
  if(cmd.foundOption("topscale_percent")) {
  
    string str = cmd.optionValue("topscale_percent");
    topscale = atoi(str.c_str());
  }

  bool twoWayGlobalMatch = false;
  if(cmd.foundOption("twoway_global_match")) {
    twoWayGlobalMatch = true;
  }

  clock_t start = clock();
  ifstream keyFile(keyList.c_str());
  ifstream dimFile(dimList.c_str());

  if(!keyFile.is_open() || !dimFile.is_open()) {
    cout << "\nError Opening File";
    return -1;
  }

  string line;
  vector<string> keyFileNames;
  while(getline(keyFile, line)) {
    keyFileNames.push_back(line);
  }

  int numKeys = keyFileNames.size();

  vector< int > widths( numKeys );
  vector< int > heights( numKeys );

  for(int i=0; i < keyFileNames.size(); i++) {
    dimFile >> heights[i] >> widths[i];
  } 

  vector< unsigned char* > keys(numKeys);
  vector< keypt_t* > keysInfo(numKeys);
  vector< int > numFeatures(numKeys);
  vector< Gridder > grids;

  for(int i=0; i < keyFileNames.size(); i++) {
    numFeatures[i] = ReadKeyFile(keyFileNames[i].c_str(),
        &keys[i], &keysInfo[i]);

    Gridder currGrid(16, widths[i], heights[i], numFeatures[i], keysInfo[i]);
    grids.push_back(currGrid);
  }

  ofstream matchFile( matchFileName.c_str(), std::ofstream::out);
  if(!matchFile.is_open()) {
    cout << "\nError opening match file";
  }
  clock_t end = clock();    
  printf("[KeyMatchGeoAware] Reading keys took %0.3fs\n", 
      (end - start) / ((double) CLOCKS_PER_SEC));


  for(int i=0; i < keyFileNames.size(); i++) {
    vector< vector<double> > refRectEdges; 
    geometry::ComputeRectangleEdges((double)widths[i], 
        (double)heights[i], refRectEdges);

    printf("[KeyMatchGeoAware] Matching to image %d\n", i);

    start = clock();

    for(int j=0; j < i; j++) {
      vector< vector<double> > srcRectEdges; 
      geometry::ComputeRectangleEdges((double)widths[j],
          (double)heights[j], srcRectEdges);

      match::FeatureMatcher matcher;

      matcher.setNumSrcPoints( numFeatures[j] );
      matcher.setSrcKeys( keysInfo[j], keys[j] );

      matcher.setNumRefPoints( numFeatures[i] );
      matcher.setRefKeys( keysInfo[i], keys[i] );

      matcher.setImageDims(widths[j], heights[j], widths[i], heights[i]);

      matcher.setSrcRectEdges(srcRectEdges);
      matcher.setRefRectEdges(refRectEdges);

      matcher.setQueryGrid(&grids[j]);
      matcher.setRefGrid(&grids[i]);

      matcher.globalMatch(topscale, twoWayGlobalMatch);
      if(matcher.matches.size() < 16) {
        continue;
      }
      vector< double > fMatrix(9);
      matcher.computeFmatrix(fMatrix.data());
      matcher.setFMatrix( fMatrix );

      matcher.computeEpipolarLines();
      matcher.clusterPoints();

      int numMatches = matcher.match();

      if(numMatches >= 16) {
        printf("Writing %d matches between images %d and %d\n", numMatches, j, i);
        sort(matcher.matches.begin(), matcher.matches.end());
        matchFile << j << " " << i << endl;
        matchFile << numMatches << endl;

        for(int m=0; m < matcher.matches.size(); m++) {
          matchFile << matcher.matches[m].first 
            << " " << matcher.matches[m].second << endl;
        }
      }
    }

    end = clock();    
    printf("[KeyMatchGeoAware] Matching took %0.3fs\n", 
        (end - start) / ((double) CLOCKS_PER_SEC));
    fflush(stdout);
  }
  matchFile.close();

  /// Skiped Freeing keyfile memory due to performance issues
  /// Assuming program exits right afterwards
  /// Please free it if you intend to extend this code beyond this point
  
  for(int i=0; i < numKeys; i++) {
    delete[] keys[i];
    delete[] keysInfo[i];
  }

  return 0;
}
