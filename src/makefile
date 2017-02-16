CC=g++
CFLAGS=-W -Wall -ansi -pedantic -std=gnu++0x
LDFLAGS= -lsqlite3
EXEC=IO
SRC= main.cpp Domain.cpp Evaluator.cpp IObjects.cpp PointP.cpp Scene.cpp
OBJ= $(SRC:.cpp=.o)


all: $(EXEC)
	mv $(EXEC) ..
	make clean

$(EXEC) : $(OBJ)
	$(CC) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CC) -o $@ -c $< $(CFLAGS)

clean:
	rm -rf *.o

mrproper: clean
	rm -rf $(EXEC)
