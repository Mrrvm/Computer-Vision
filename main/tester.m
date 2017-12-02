close all;
clear all;
clc;

% loads camara parameters
load '/home/imarcher/Dropbox/Tecnico/PIV/Project/main/cameraparametersAsus.mat';

% loads dataset
base_data_dir = '/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/';
d1 = dir([base_data_dir 'depth1*']);
d2 = dir([base_data_dir 'depth2*']);
r1 = dir([base_data_dir 'rgb_image1_*']);
r2 = dir([base_data_dir 'rgb_image2_*']);
for i=1:length(d1)
    im1(i).rgb = [base_data_dir r1(i).name];
    im2(i).rgb = [base_data_dir r2(i).name];
    im1(i).depth = [base_data_dir d1(i).name];
    im2(i).depth = [base_data_dir d2(i).name];
end

% world reference
cam1toW.R = [1 0 0; 0 1 0; 0 0 1];
cam1toW.T = [0 0 0];

% define 4 points in cam1
imshow(im1(1).rgb);
[u1 v1] = ginput(4);
cam1_points = round([u1 v1]);
cam1_pts_homo = [cam1_points ones(4,1)];

% define 4 points in cam1
imshow(im2(1).rgb);
[u2 v2]=ginput(4);
cam2_points = round([u2 v2]);
cam2_pts_homo = [cam2_points ones(4,1)];

% obtains xyz of 4 points from cam1
K = cam_params.Kdepth;
M1 = K*([cam1toW.R (cam1toW.T)']);
depth1 = load(im1(1).depth);
depth2 = load(im1(2).depth);
depth1_array = zeros(1, 4);
depth2_array = zeros(1, 4);
xyz = zeros(4, 4);
for i=1:4
    depth1_array(i) = double(depth1.depth_array(cam1_points(i, 1), cam1_points(i, 2)))*0.01;
    depth2_array(i) = double(depth2.depth_array(cam2_points(i, 1), cam2_points(i, 2)))*0.01;
    for j=1:3
        cam1_pts_homo(i, j) = cam1_pts_homo(i, j)*depth1_array(i);
        cam2_pts_homo(i, j) = cam2_pts_homo(i, j)*depth2_array(i);
    end
    xyz(i, :) = linsolve(M1,cam1_pts_homo(i, :)')';
    xyz(i, 4) = 1;
end

% obtains R and T from cam2 to cam1 (aka world)
KRT_matrix = zeros(3, 4);
RT_matrix = zeros(3, 4);
KRT_matrix = linsolve(xyz,cam2_pts_homo)';
RT_matrix = linsolve(K, KRT_matrix);

cam2toW.T = RT_matrix(:,4);
cam2toW.R = RT_matrix(1:3,1:3);

% runs part1
objects = track3D_part1(im1, im2, cam_params, cam1toW, cam2toW);




