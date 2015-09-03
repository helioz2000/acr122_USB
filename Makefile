LIBS = -lstdc++ -lm -lpthread -lusb-1.0
CC = gcc
CFLAGS = -g -Wall -Wno-unused -Wno-unknown-pragmas

MODULEDIR = ../hardware/

SOURCES=$(wildcard *.cpp)
OBJECTS=$(patsubst %.cpp,%.o,$(SOURCES))

EXECUTABLE = acr122

.PHONY: default all $(EXECUTABLE) clean

$(EXECUTABLE): $(SOURCES:.cpp=.o)
	$(CC) $(LIBS) $^ -o $@	

%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	-rm -f *.o
	-rm -rf $(EXECUTABLE)

