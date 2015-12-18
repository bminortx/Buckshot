```
______            _        _           _   
| ___ \          | |      | |         | |  
| |_/ /_   _  ___| | _____| |__   ___ | |_   
| ___ \ | | |/ __| |/ / __| '_ \ / _ \| __|            
| |_/ / |_| | (__|   <\__ \ | | | (_) | |_             
\____/ \__,_|\___|_|\_\___/_| |_|\___/ \__|            

```
## A MATLAB Interface for the Bullet Physics Engine ##

Buckshot's goal is to allow for convenient use of the MATLAB libraries,
while also guaranteeing accurate simulation through Bullet Physics, a
wonderful physics engine written by Erwin Coumans.

## NOW WITH OpenGL GUI! Scroll down to learn more ##

- - - - - - - - -

## Getting Started ##


#### 1. Requirements ####

* CMake
* MATLAB
* [Bullet Physics Library](http://bulletphysics.org/wordpress/)

For the Graphics pipeline:

* OpenGL
* FreeGLUT

#### 2. MEX Directory ####

Make sure that you're pointing to the correct mex directory in the CMake script; it's a set variable at the very top of bulletInterface/CMakeLists.txt:

`set(MEX /usr/local/MATLAB/R2014a/bin/mex)`

#### 3. SETUP.m ####

Once 1. and 2. are out of the way, Open Matlab and run `BuckshotCreator()` in the top directory. `BuckshotCreator` is a good way to start any bullet-focused MATLAB script, as . Run the function with `true` to
- `BuckshotCreator(false)`		: connects all of Buckshot's paths to MATLAB
- `BuckshotCreator(true)`		: builds the BulletInterface class via CMake, and creates a MATLAB pointer to the resulting Buckshot class

- - - - - - - - -

## Demos ##

[![The Bullet Demo in action](http://img.youtube.com/vi/IzXKR44iJ0o/0.jpg)](https://www.youtube.com/watch?v=IzXKR44iJ0o)

Each demo has the same GUI (displayed at startup), but they
showcase different things: 

- BulletDemo: Shows off how easy it is to add shapes and constraints to the
  scene. 
- VehicleDemo: Demonstrates a Bullet RaycastVehicle.

Every function in every directory should have a bit of documentation
involved, so just type ```help [name of function]``` to look up usage and arguments
for everything. Some of the constraints are a bit tricky, so this might help.

#### The GUI ####

Use OpenGL rendering to draw your Physics scene outside of the confines of MATLAB! Just specify the `useOpenGL` flag when calling `Buckshot.RunSimulation()`, and Buckshot will create an OpenGL context for you to play around in. This is a very new feature, so I'd love to get some feedback on improvements.

OpenGL GUI

- spacebar		: Run/pause the simulation
- 0				: Reset View
- i				: Iterate one frame at a time (must be paused)
- r				: Reset the physics simulation
- c				: Draw constraints
- m				: Change shader effect
- + and -		: Move light up and down
- [ and ]		: Move light angle
- Mouse scroll	: Zoom in/out
- PgUp / PgDwn	: Zoom in/out
- q             : quit the program 

TODOs left here:

* Correct heightmap rendering (defaults to flat plane now)
* Correct constraint rendering
* Add mouse viewing translation/rotation
* Add texturing for every shape type, as well as easy texture file linking through MATLAB
* Add more shaders
* Find a way to quit the GUI without quitting MATLAB

- - - - - - - - -

## WHO IS THIS ##

Author: Brandon Minor | gallimatrix ~at~ gmail.com | [bminortx.github.io](http://bminortx.github.io)

Associated Acts: ARPG @ CU Boulder, [Replica Labs](http://www.replicalabs.com)

If you'd like to work on the project, or point out anything grossly wrong,
just reach out! I would love to make Buckshot a legitimately useful tool. 

- - - - - - - - -

## License ##

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
