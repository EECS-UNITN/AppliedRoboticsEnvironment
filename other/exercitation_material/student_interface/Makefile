TARGET=libstudent_LIBRARY.so
CXX=g++
CXXFLAGS=`pkg-config --cflags opencv` -Ofast -Wall -Werror -Iinclude
LDLIBS=`pkg-config --libs opencv`

SRCS:=$(wildcard ./src/*.cpp)
OBJS:=$(patsubst %.cpp,%.o,$(SRCS))

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -shared -o $@ $^ $(LDLIBS)

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) -fPIC $< -o $@

clean:
	rm -rf $(TARGET) src/*.o
	
.PHONY: all clean

