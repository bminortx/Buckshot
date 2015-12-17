function [ new_class_ptr ] = BuckshotCreator(boolCreatePtr)
%Includes all of the libraries necessary for the Sim bullet wrapper to
%function in MATLAB. The boolCreatePtr that SETUP takes as an argument lets the
%user choose whether or not they want to create a bullet class object (true), or
%merely link the libraries to MATLAB for ease of coding (false).

disp('> Linking directories');
disp('> ...');
% Link all of our directories
addpath(genpath('bulletComponents'));
addpath('bulletComponents');
addpath(genpath('matlabComponents'));
addpath('matlabComponents');
addpath(genpath('Demos'));
addpath('Demos');

disp('> Linking complete!');

if boolCreatePtr == true,
    disp('> CreatePtr == true: Attempting to compile Buckshot Interface');
    system('chmod +x createMexFiles.sh');
    result = system('./createMexFiles.sh false');
    if (result ~= 0),
        disp('Buckshot failed to compile');
        return;
    end
    new_class_ptr = bullet_interface();
    disp('> ...');
    disp('> Buckshot successfully compiled! Way to go, champ.');
    disp('------------------');
end

