# Motion Planning Toolkit

## Installation

### Required dependencies

Motion Planning Toolkit has the following required dependencies:

* CMake (version 3.1)

* [Fast Library for Approximate Nearest Neighbors (FLANN)](http://www.cs.ubc.ca/research/flann/) (version 1.8.4)

* [Flexible Collision Library (FCL)](https://github.com/flexible-collision-library/fcl) (version 0.3.2)

* [Boost](http://www.boost.org/) (version 1.57.0)

* [Open Asset Import Library (ASSIMP)](https://github.com/assimp/assimp) (version 3.0.+)

### Optional dependencies

Motion Planning Toolkit has the following optional dependencies:

* [GLFW](http://www.glfw.org/) (version 3.1.1)

* [OpenGL Extension Wrangler Library](https://github.com/nigels-com/glew) (GLEW) (1.12.0)

### Installation on Mac OS X (MacPorts)

Install [MacPorts](www.macports.org).

Install required dependencies:

   `sudo port sync`
   `sudo port install cmake flann boost assimp`

Install all dependencies:

   `sudo port sync`
   `sudo port install cmake flann boost assimp glfw glew`
