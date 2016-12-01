#CC= g++-4.9
#CFLAGS=-std=c++11 -g -O2
#LDFLAGS=
#SOURCES=image_nav.cpp
#OBJECTS=$(SOURCES:.cpp=.o)
#EXECUTABLE=run
#
#all: clean $(SOURCES) $(EXECUTABLE)
#
#$(EXECUTABLE): $(OBJECTS)
#	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
#
#%.o: %.cpp
#	$(CC) $(CFLAGS) $< -c
#
#clean:
#	$(RM) count *.o *~

# your Makefile
# compiler
CC :=g++-4.8
# include files
CFLAGS :=-std=c++11 -g -O2 `pkg-config opencv --cflags` `pkg-config opencv --libs`
LDFLAGS :=
# compile all c++ files in dir
SOURCES := main_RAGS_tester.cpp#$(wildcard *.cpp)
# C++ files .cpp removed file is executable
EXECUTABLE :=$(SOURCES:.cpp=)

all:$(EXECUTABLE)

$(EXECUTABLE):$(SOURCES)
	$(CC) $< $(LDFLAGS) $(CFLAGS) -o $@
clean:  
	rm -rf $(EXECUTABLE)

