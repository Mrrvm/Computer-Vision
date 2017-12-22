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

% image pair used to get R and T
i = 1;

% get image dimension
load(im1(i).depth);
dim = size(depth_array);
clear depth_array;

% world reference
cam1toW.R = [1 0 0; 0 1 0; 0 0 1];
cam1toW.T = [0; 0; 0];

% load depth images
load(im1(i).depth);
depth_array1 = depth_array;
load(im2(i).depth);
depth_array2 = depth_array;
% load rgb images
img_rgb1 = imread(im1(i).rgb);
img_rgb2 = imread(im2(i).rgb);

% get xyz 
xyz1 = get_xyzasus(depth_array1(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
xyz2 = get_xyzasus(depth_array2(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
% get all rgb points in depth plane
img_rgb_indepth1 = get_rgbd(xyz1, img_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
img_rgb_indepth2 = get_rgbd(xyz2, img_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);

n_points=10;

%{
figure(1); imshow(img_rgb_indepth1);
figure(2); imshow(img_rgb_indepth2);
x1=zeros(n_points,1);y1=x1;x2=y1;y2=x1;
for i=1:n_points,
    figure(1);
    [xa ya]=ginput(1); text(xa,ya,int2str(i));
    xa=fix(xa);ya=fix(ya);
    x1(i)=xa;y1(i)=ya;
    figure(2);
    [xa ya]=ginput(1); text(xa,ya,int2str(i));
    xa=fix(xa);ya=fix(ya);
    x2(i)=xa;y2(i)=ya;
end
%}
x1 = [182;177;519;233;263;242;460;463;280;226];
x2 = [171;182;401;332;359;355;428;418;356;315];
y1 = [302;239;190;325;337;270;240;299;365;362];
y2 = [181;92 ;82 ;190;201;126;115;172;233;232];


xyz1_points = zeros(n_points, 3);
xyz2_points = zeros(n_points, 3);
for i = 1:n_points
    xyz1_points(i, :) = xyz1(sub2ind(size(depth_array1), y1(i), x1(i)), :);
    xyz2_points(i, :) = xyz2(sub2ind(size(depth_array2), y2(i), x2(i)), :);
end

[d,xx,tr]=procrustes(xyz1_points, xyz2_points,'scaling',false,'reflection',false);

cam2toW.R = tr.T';
cam2toW.T = tr.c(1,:)';

% runs part1
objects = track3D_part1(im1, im2, cam_params, cam1toW, cam2toW);




