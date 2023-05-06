#!/bin/bash

cc=gcc
name=gsim
src=main.c
std=-std=c99
opt=-O2
inc=-Ispxe
opngl=-lglfw

wflags=(
    -Wall
    -Wextra
    -pedantic
)

cmd() {
    echo "$@" && $@
}

build() {
    if echo "$OSTYPE" | grep -q "darwin"; then
        libs=(
            -framework OpenGL
        )
    elif echo "$OSTYPE" | grep -q "linux"; then
        libs=(
            -lm
            -lGL
            -lGLEW
        )
    else
        echo "This OS is not supported by thus build script yet..." && exit
    fi
    cmd $cc $src -o $name $std $opt ${wflags[*]} $inc $opngl ${libs[*]}
}

case "$1" in
    "run")
        shift && build && ./$name $@;;
    "clean")
        cmd rm -f $name;;
    *)
        build $@;;
esac
