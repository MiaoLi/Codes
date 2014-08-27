%%%% To run from Matlab_functions

clear all,
close all,

addpath (pwd) ;

%%%%%%%%%%%%% Cylinder %%%%%%%%%%%%%%%%%

rootname = 'cylinder_p';

% Shape object
% s1 = 0.02;
% s2 = 0.02;
% s3 = 0.075;
% e1 = .1;
% e2 = 1;

s1 = 0.035;
s2 = 0.035;
s3 = 0.065;

e1 = .25;
e2 = 1;

kx = 0;
ky = 0;

shape = [e1 e2 s1 s2 s3 kx ky];

cd ..
mkdir('Cylinder_mod');
cd './Cylinder_mod'


fcreate_mod_file(rootname, shape);
fAll_mod(rootname);

 cd ..