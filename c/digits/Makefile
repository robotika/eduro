CC=g++

LIBDIR=

ROBOT_CXX_FLAGS=
ROBOT_LD_FLAGS=

OPTFLAGS=-O3
CXXFLAGS=$(ROBOT_CXX_FLAGS) `pkg-config --cflags opencv` -I$(LIBDIR) $(OPTFLAGS) #-march=native -mtune=native
LDFLAGS=$(ROBOT_LD_FLAGS) `pkg-config --libs opencv` $(OPTFLAGS) #-march=native -mtune=native

BIN=digits

all: $(BIN)

$(BIN): mainDigits.o Makefile
	$(CC) mainDigits.o -o $(BIN) $(LDFLAGS)

%.o: %.cc *.h Makefile
	$(CC) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o $(BIN)

