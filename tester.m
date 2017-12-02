close all;
clear all;
clc;

% loads camara parameters
load '/home/imarcher/Dropbox/Tecnico/PIV/Project/cameraparametersAsus.mat';

% loads dataset
base_data_dir='/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/';
d1=dir([base_data_dir 'depth1*']);
d2=dir([base_data_dir 'depth2*']);
r1=dir([base_data_dir 'rgb_image1_*']);
r2=dir([base_data_dir 'rgb_image2_*']);
for i=1:length(d1),
    im1(i).rgb=[base_data_dir r1(i).name];
    im2(i).rgb=[base_data_dir r2(i).name];
    im1(i).depth=[base_data_dir d1(i).name];
    im2(i).depth=[base_data_dir d2(i).name];
end

% gets cam2toW
cam1toW = 1;
cam2toW = 1;

% runs part1
objects = track3D_part1(im1, im2, cam_params, cam1toW, cam2toW);