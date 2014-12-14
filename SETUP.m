function [ new_class_ptr ] = SETUP( boolCreatePtr )
%Includes all of the libraries necessary for the Sim bullet wrapper to
%function in MATLAB. The boolCreatePtr that SETUP takes as an argument lets the
%user choose whether or not they want to create a bullet class object (true), or
%merely link the libraries to MATLAB for ease of coding (false).
%

%Our shapes 
addpath(genpath('BulletShapes'));
addpath('Bullet_shapes');
disp('> Shapes linked...');

%The Bullet-MATLAB interface
addpath(genpath('Bullet_interface'));
addpath('Bullet_interface');
disp('> Interface linked...');

% matlab components
addpath(genpath('matlabComponents'));
addpath('matlabComponents'); 

%Our demos
addpath(genpath('Demos'));
addpath('Demos');
disp('> Demos linked...');

system('chmod +x createMexFiles.sh')
system('./createMexFiles.sh')

disp('> Bullet library linked...');
disp('> Linking complete!');
disp('------------------');

if boolCreatePtr == true,   
  new_class_ptr = bullet_interface();
end

end

