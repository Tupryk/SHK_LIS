OBJS = main.o
OUTPUT = x.exe

PY_SITE = $(shell python3 -m site --user-site)
INCLUDE_PATHS += ${PY_SITE}/robotic/include
LIB_PATHS += ${PY_SITE}/robotic

CXX_FLAGS = -g -O3 -Wall -fPIC -std=c++2a $(INCLUDE_PATHS:%=-I%)
LINK_FLAGS = $(LIB_PATHS:%=-L%) $(LIB_PATHS:%=-Wl,-rpath,%)
LIBS = -lrai

default: $(OUTPUT)

%.o: %.cpp
	g++ $(CXX_FLAGS) -o $@ -c $<

$(OUTPUT): $(OBJS)
	g++ $(LINK_FLAGS) -o $@ $(OBJS) $(LIBS)

clean: force
	rm -f $(OUTPUT) $(OBJS)

force:	;
