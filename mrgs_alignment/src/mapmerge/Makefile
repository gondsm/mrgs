# Alterações por Gonçalo Martins.
# Esta é a versão de trabalho, com o propósito de ir preparando a inclusão no ROS

# Nomes dos objectos
OBJECTS = grid_map.o hough.o io.o  manipulatemap.o common.o

# Compilador a usar
C++ = g++

# Flags a passar ao C
CFLAGS =  -fPIC `pkg-config --cflags opencv` 

# Flags a passar ao Linker
LDFLAGS = -L. `pkg-config --libs opencv` -lm

# Flags de compilação dos objectos
FLAGS= -Wall -O2

#all: $(LIBRARY) align
all: align

# Linkagem estática:
align: $(OBJECTS) align.o
	$(C++) $(LDFLAGS) align.o $(OBJECTS) -o align `pkg-config --libs opencv` -lm

# Objectos
%.o : %.cpp
	$(C++) $(CFLAGS) $(FLAGS) -c $<

clean:
	rm -f *.o *~ align
