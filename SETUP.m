function [ new_class_ptr ] = SETUP( isPtrCreated, isSceneGraphOn)
%Includes all of the libraries necessary for the Sim bullet wrapper to
%function in MATLAB. The boolCreatePtr that SETUP takes as an argument lets the
%user choose whether or not they want to create a bullet class object (true), or
%merely link the libraries to MATLAB for ease of coding (false).
%

%Our shapes 
addpath(genpath('ModelGraph'));
addpath('ModelGraph');
disp('> Shapes linked...');

%The Bullet-MATLAB interface
addpath(genpath('bulletInterface'));
addpath('bulletInterface');
disp('> Bullet Interface linked...');

% matlab components
addpath(genpath('matlabComponents'));
addpath('matlabComponents');

%Our demos
addpath(genpath('Demos'));
addpath('Demos');
disp('> Demos linked...');

system('chmod +x createMexFiles.sh')
if isSceneGraphOn,
    system('./createMexFiles.sh true')
else
    system('./createMexFiles.sh false')
end

disp('> Bullet wrapper created...');
disp('> Linking complete!');
disp('------------------');

if boolCreatePtr == true,
  new_class_ptr = bullet_interface();
end

