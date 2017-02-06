CC=g++
#CFLAGS=-O0 -w -c -g -Wall
CFLAGS=-O3 -w -c -Wall

# Change the path in the following to your ANN and zlib distribution paths
# Alternatively, copy the lib and .h files to your global lib and include paths
LIBPATH=-L../lib/ann_1.1_char/lib/ -L../lib/zlib/lib
IFLAGS=-I../lib/ann_1.1_char/include/ -I../lib/zlib/include/
LIBS=-lANN_char -lz

PKGCONFIGFLAG=`pkg-config --cflags --libs opencv`

default: fullgraph pairwise

fullgraph: match_graph
	mv match_graph ../bin/KeyMatchGeometryAware

pairwise: match_pairs
	mv match_pairs ../bin/match_pairs

match_graph: match_graph.o keys2a.o Geometric.o Matcher.o Gridder.o argvparser.o
	$(CC) $(IFLAGS) match_graph.o keys2a.o Geometric.o Matcher.o Gridder.o argvparser.o $(LIBPATH) $(PKGCONFIGFLAG)-Wall -o match_graph $(LIBS)

match_pairs: match_image_pair.o keys2a.o Geometric.o Matcher.o Gridder.o argvparser.o
	$(CC) $(IFLAGS) match_image_pair.o keys2a.o Geometric.o Matcher.o Gridder.o argvparser.o $(LIBPATH) $(PKGCONFIGFLAG)-Wall -o match_pairs $(LIBS)

match_image_pair.o: match_image_pair.cpp Gridder.cpp Gridder.h  defs.h Geometric.cpp Geometric.h Matcher.cpp Matcher.h argvparser.cpp argvparser.h 
	$(CC) $(CFLAGS) $(IFLAGS) match_image_pair.cpp

match_graph.o: match_graph.cpp Gridder.cpp Gridder.h  defs.h Geometric.cpp Geometric.h Matcher.cpp Matcher.h  argvparser.cpp argvparser.h
	$(CC) $(CFLAGS) $(IFLAGS) match_graph.cpp

Gridder.o: Gridder.cpp Gridder.h
	$(CC) $(CFLAGS) $(IFLAGS) Gridder.cpp

Matcher.o: Matcher.cpp Matcher.h
	$(CC) $(CFLAGS) $(IFLAGS) Matcher.cpp

Geometric.o: Geometric.cpp Geometric.h
	$(CC) $(CFLAGS) $(IFLAGS) Geometric.cpp

keys2a.o: keys2a.cpp keys2a.h
	$(CC) $(CFLAGS) $(IFLAGS) keys2a.cpp

argvparser.o: argvparser.cpp argvparser.h
	$(CC) $(CFLAGS) $(IFLAGS) argvparser.cpp

clean:
	rm -rf *o ../bin/KeyMatchGeometryAware ../bin/match_pairs
