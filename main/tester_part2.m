clear;
close all;
clc;

load cameraparametersAsus	% Camera parameters

dataset = 'maizena_chocapic';
data_rgb = '\data_rgb\';
%data_rgb = '\';

%% Loads dataset
base_data_dir = cat(2, 'C:\Users\luisr\Desktop\Luis\IST\PIV\pivproject\data\', dataset, data_rgb);

% Depth directories
d1 = dir(cat(2,base_data_dir, 'depth1_*'));
d2 = dir(cat(2,base_data_dir, 'depth2_*'));

% RGB directories
r1 = dir(cat(2,base_data_dir, 'rgb_image1_*'));
r2 = dir(cat(2,base_data_dir, 'rgb_image2_*'));

%% Build sequence of images
for i=1:length(d1)
    temprgb = dir(cat(2,base_data_dir, 'rgb_image1_', num2str(i),'.png'));
    im1(i).rgb = temprgb.name;
    tempdepth = dir(cat(2,base_data_dir, 'depth1_', num2str(i),'.mat'));
    im1(i).depth = tempdepth.name;
    temprgb = dir(cat(2,base_data_dir, 'rgb_image2_', num2str(i),'.png'));
    im2(i).rgb = temprgb.name;
    tempdepth = dir(cat(2,base_data_dir, 'depth2_', num2str(i),'.mat'));
    im2(i).depth = tempdepth.name;
end

% Eliminate unwanted variables
keepvars = {'im1', 'im2', 'cam_params'};
clearvars('-except', keepvars{:});

%% Run object tracking
[objects, cam1toW, cam2toW] = track3D_part2(im1, im2, cam_params);
