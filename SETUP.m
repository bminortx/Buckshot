function [ new_class_ptr ] = SETUP( boolCreatePtr )
%Includes all of the libraries necessary for the Sim bullet wrapper to
%function in MATLAB. The boolCreatePtr that SETUP takes as an argument lets the
%user choose whether or not they want to create a bullet class object (true), or
%merely link the libraries to MATLAB for ease of coding (false).
%

BULLET_DIR = '/Users/Trystan/Code/Builds/Bullet/src/';
BULLET_SRC = '/Users/Trystan/Code/Thirdparty/bullet-2.81-rev2613/src/';
BOOST_DIR = '/usr/local/include';
EIGEN_DIR = '/usr/local/include/eigen3';

BULLET_DYN_LIB = 'BulletDynamics';
BULLET_COL_LIB = 'BulletCollision';
BULLET_LIN_LIB = 'LinearMath';

%Our shapes 
addpath(genpath('Bullet_shapes'));
addpath('Bullet_shapes');         
addpath(genpath('Matlab_shapes'));
addpath('Matlab_shapes'); 
disp('>Shapes linked...');

%The Bullet-MATLAB interface
addpath(genpath('Bullet_interface'));
addpath('Bullet_interface');
disp('>Interface linked...');

%Our constraints
addpath(genpath('Matlab_constraints'));
addpath('Matlab_constraints'); 
disp('>Constraints linked...');

%Our compounds (shapes and constraints)
addpath(genpath('Matlab_compounds'));
addpath('Matlab_compounds'); 
disp('>Compounds linked...');

%Our functions   
addpath(genpath('Matlab_functions'));
addpath('Matlab_functions'); 
disp('>Functions linked...');

%Our demos
addpath(genpath('Demos'));
addpath('Demos');
disp('>Demos linked...');

mex (['-I' BULLET_DIR], ['-I' BOOST_DIR], ['-I' BULLET_SRC], ['-I' EIGEN_DIR],...
     ['-L' BULLET_DIR BULLET_DYN_LIB], ['-l' BULLET_DYN_LIB], ...
     ['-L' BULLET_DIR BULLET_COL_LIB], ['-l' BULLET_COL_LIB], ...
     ['-L' BULLET_DIR BULLET_LIN_LIB], ['-l' BULLET_LIN_LIB], ...
     'Bullet_interface/bullet_interface_mex.cpp');

disp('>Bullet library linked...');
disp('>Linking complete!');
disp('------------------');

if boolCreatePtr == true,   
  new_class_ptr = bullet_interface();
end

end

