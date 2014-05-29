CC=g++

LIBDIR=

ROBOT_CXX_FLAGS=
ROBOT_LD_FLAGS=

OPTFLAGS=-O3
CXXFLAGS=$(ROBOT_CXX_FLAGS) `pkg-config --cflags opencv` -I$(LIBDIR) $(OPTFLAGS) #-march=native -mtune=native
LDFLAGS=$(ROBOT_LD_FLAGS) `pkg-config --libs opencv` $(OPTFLAGS) #-march=native -mtune=native

BIN=rc

all: $(BIN) followline avoidgreen averagergb

$(BIN): mainCountRGB.o Makefile
	$(CC) mainCountRGB.o -o $(BIN) $(LDFLAGS)

followline: mainFollowLine.o Makefile
	$(CC) mainFollowLine.o -o followline $(LDFLAGS)

avoidgreen: mainAvoidGreen.o Makefile
	$(CC) mainAvoidGreen.o -o avoidgreen $(LDFLAGS)

averagergb: mainAverageRGB.o Makefile
	$(CC) mainAverageRGB.o -o averagergb $(LDFLAGS)

%.o: %.cc *.h Makefile
	$(CC) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o $(BIN)

