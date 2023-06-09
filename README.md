# gsim

> Toyful and simple 2D N-body simulation of gravitational systems written in C. Handles
> collisions and provides fun interactions you can play with. Built on top of my 
> rendering engine, the [simple pixel engine](https://github.com/LogicEu/spxe.git).

![alt text](https://github.com/LogicEu/gsim/blob/main/image.png?raw=true)

## Controls

> Click and drag the mouse on top of the background to move the camera.
> Click and drag on the bodies to move them.

| Key | Action |
| --- | --- |
| W | Move camera up |
| S | Move camera down |
| D | Move camera to the right |
| A | Move camera to the left |
| Z | Zoom in |
| X | Zoom out |
| O | Decrease size of physics time step, slow down time |
| P | Increase size of physics time step, speed up time |
| G | Turn gravity on and off |
| Q | Switch the rendering of the quadtree on and off |
| LShift | Switch the rendering of trails on and off |
| Space | Pause the time of the physics in the simulation |
| R | Restart the simulation |
| Escape | Quit |

## Third Party

> [spxe](https://github.com/LogicEu/spxe.git) depends on OpenGL libraries to manage
> windows and render graphics independently of the platform.

* [GLFW](https://github.com/glfw/glfw.git)
* [GLEW](https://github.com/nigels-com/glew.git) (only on Linux and Windows)

## Try it

> If you have all third party dependencies installed on your system you can
> easily try gsim with the following commands:

```shell
git clone --recursive https://github.com/LogicEu/gsim.git
cd gsim
make # or ./build.sh
./gsim -help
```

## Compile

> Both build script currently work on Linux an MacOS. The compiled binary will
> be created at the root of the repository. 

```shell
make # or ./build.sh
```
> Make sure to clean after compiling.

```shell
make clean # or ./build.sh clean
```

