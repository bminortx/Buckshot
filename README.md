---------------------------------
SimLAB
---------------------------------
Author: Brandon Minor, Autonomous Robotics Group,
        George Washington University

SimLAB is a MATLAB Physics Simulator that utilizes the Bullet Physics Engine.
This program's goal is to allow for convenient use of the MATLAB libraries
while also guaranteeing accurate simulation through Bullet Physics, a
fully functional and reliable engine.

----------
SETUP
----------
SimLAB only requires a couple of things: 
  * MATLAB
  * Bullet Physics Library: http://bulletphysics.org/wordpress/
  * Eigen Linear Algebra Library: http://bit.ly/LXBsEr
  * Boost C++ Libraries: http://www.boost.org/
  
Install all of these; order doesn't really matter here. Make sure that there is an alias to the mex program (located in MATLAB/Rxxxx/bin/mex)

Once this is done, open STEUP.m in the top directory. After the comments,
you should see these lines: 

```BULLET_DIR = '/Users/Trystan/Code/Builds/Bullet/src/';```

```BULLET_SRC = '/Users/Trystan/Code/Thirdparty/bullet-2.81-rev2613/src/';```

```BOOST_DIR = '/usr/local/include';```

```EIGEN_DIR = '/usr/local/include/eigen3';```

Replace the strings after the variable names with your own paths, for each
respective library. Add the SimLAB folder and all subdirectories to your path.
After that, everything should work!

---------
DEMOS
---------
Each demo has the same GUI (displayed at startup), but they
showcase different things: 

- BulletDemo: Shows off how easy it is to add shapes and constraints to the
  scene. 
- VehicleDemo: Demonstrates a Bullet RaycastVehicle.
- OptimizerDemo: Seeks to perform Gauss-Newton optimization on a path. Still
  pretty buggy; I would stay away, haha. 

Every function in every directory should have a bit of documentation
involved, so just type 'help [name of function]' to look up usage and arguments
for everything. Some of the constraints are a bit tricky, so this might help. 

This documentation will continue to expand as things become available.
If you'd like to work on the project, or point out anything grossly wrong,
just reach out:
gallimatrix ~at~ gmail.com

--------
License
--------

The MIT License (MIT)

Copyright (c) 2014, Brandon Minor

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
