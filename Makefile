# gsim Makefile

NAME=gsim
SRC=main.c
CC=gcc
STD=-std=c99
OPT=-O3
WFLAGS=-Wall -Wextra -pedantic
INC=-Ispxe
LIB=-lglfw

OS=$(shell uname -s)
ifeq ($(OS),Darwin)
	LIB+=-framework OpenGL
else
	LIB+=-lGL -lGLEW -lm
endif

CFLAGS=$(STD) $(OPT) $(WFLAGS) $(INC) $(LIB)

$(NAME): $(SRC)
	$(CC) $^ -o $@ $(CFLAGS)

clean:
	$(RM) $(NAME)
