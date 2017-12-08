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
%{
% define 4 points in cam1
%imshow(im1(1).rgb);
%[u1 v1] = ginput(8);
u1 = [176.587352138308;210.217925386715;252.583712465878;252.583712465878;267.870336669700;279.662875341219;448.689262966333;453.493630573248];
v1 = [301.209736123749;361.045950864422;352.310737033667;332.656505914468;331.782984531392;363.229754322111;300.336214740673;229.144222020018];
cam1_points = fix([u1 v1]);
cam1_pointsd =  inv(cam_params.Kdepth)*inv(cam_params.R)*([cam1_points'; ones(3, :)] - cam_params.T'*ones(3,8));

% define 4 points in cam1
%imshow(im2(1).rgb);
%[u2 v2]=ginput(8);
u2 = [164.794813466788;289.271610555050;334.694722474977;340.372611464968;351.728389444950;354.348953594177;406.760236578708;421.610100090992];
v2 = [195.076888080073;241.810282074613;226.960418562329;205.122383985441;204.248862602366;236.132393084622;180.663785259327;111.655595996360];
cam2_points = fix([u2 v2]);
cam2_pointsd =  inv(cam_params.Kdepth)*inv(cam_params.R)*(cam2_points'-cam_params.T');

% obtains xyz for both camaras
K = cam_params.Kdepth;
load(im1(1).depth);
xyz1 = get_xyzasus(depth_array(:), [480 640], (1:640*480)', K, 1, 0);
load(im1(2).depth);
xyz2 = get_xyzasus(depth_array(:), [480 640], (1:640*480)', K, 1, 0);

xyz1_points = zeros(8, 3);
xyz2_points = zeros(8, 3);
for i = 1:8
    xyz1_points(i, :) = xyz1(sub2ind(size(depth_array), cam1_pointsd(i,1), cam1_pointsd(i, 2)), :);
    xyz2_points(i, :) = xyz2(sub2ind(size(depth_array), cam2_pointsd(i,1), cam2_pointsd(i, 2)), :);
end
[d,xx,tr]=procrustes(xyz1_points, xyz2_points,'scaling',false,'reflection',false);
cam2toW.R = tr.T;
cam2toW.T = tr.c(1,:);
%}

% runs part1
objects = track3D_part1(im1, im2, cam_params, cam1toW, cam1toW);




