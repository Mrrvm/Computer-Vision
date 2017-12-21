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

% get image dimension
load(im1(i).depth);
dim = size(depth_array);
clear depth_array;

% world reference
cam1toW.R = [1 0 0; 0 1 0; 0 0 1];
cam1toW.T = [0; 0; 0];

n_imgs = length(im1);
if n_imgs < 10
    n_sets = n_imgs;
    img_set = zeros(1, n_imgs);
    for i = 1:n_imgs
        img_set(1, i) = i;
    end
else
    p = floor(n_imgs/10);
    n_sets = 10;
    img_set = zeros(1, n_sets);
    p_aux = p;
    for i = 1:n_sets
        img_set(1, i) = p;
        p = p + p_aux;
    end
end

d_best = 10;

for j = 1:n_sets
    j_rand = img_set(j);
    % load depth images
    load(im1(j_rand).depth);
    depth_array1 = depth_array;
    load(im2(j_rand).depth);
    depth_array2 = depth_array;
    % load rgb images
    img_rgb1 = imread(im1(j_rand).rgb);
    img_rgb2 = imread(im2(j_rand).rgb);

    % get xyz 
    xyz1 = get_xyzasus(depth_array1(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
    xyz2 = get_xyzasus(depth_array2(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
    % get all rgb points in depth plane
    img_rgb_indepth1 = get_rgbd(xyz1, img_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
    img_rgb_indepth2 = get_rgbd(xyz2, img_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);

    [kpts1, d1] = vl_sift(im2single(rgb2gray(img_rgb_indepth1)));
    [kpts2, d2] = vl_sift(im2single(rgb2gray(img_rgb_indepth2)));
    [matches, scores] = vl_ubcmatch(d1, d2);
    [drop, perm] = sort(scores, 'descend');
    matches = matches(:, perm);
    scores  = scores(perm);

    c = 1;
    for i = 1:length(matches)
        pt1 = [round(kpts1(1, matches(1, i))) round(kpts1(2, matches(1, i)))];
        pt2 = [round(kpts2(1, matches(2, i))) round(kpts2(2, matches(2, i)))];
        if depth_array1(pt1(2), pt1(1)) ~= 0 && depth_array2(pt2(2), pt2(1)) ~= 0
            matchedPoints1(c, :) = pt1;
            matchedPoints2(c, :) = pt2;
            c = c + 1;
        end
    end

    [fLMedS, inliers] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'NumTrials', 2000, 'Method','RANSAC', 'DistanceThreshold', 0.4);

    xyz1_points = zeros(length(matchedPoints1(inliers,:)), 3);
    xyz2_points = zeros(length(matchedPoints1(inliers,:)), 3);
    for i = 1:length(matchedPoints1)
        if inliers(i) == 1
            xyz1_points(i, :) = xyz1(sub2ind(size(depth_array1), matchedPoints1(i,2), matchedPoints1(i,1)), :);
            xyz2_points(i, :) = xyz2(sub2ind(size(depth_array2), matchedPoints2(i,2), matchedPoints2(i,1)), :);
        end
    end

    [d,xx,tr] = procrustes(xyz1_points, xyz2_points,'scaling',false,'reflection',false);
    if d < d_best
        tr_best = tr;
        d_best = d;
        inliers_best = inliers;
        matchedPoints1_best = matchedPoints1;
        matchedPoints2_best = matchedPoints2;
        img1_best = img_rgb_indepth1;
        img2_best = img_rgb_indepth2;
    end
    
    clear matchedPoints1 matchedPoints2;
end

showMatchedFeatures(img1_best, img2_best, matchedPoints1_best(inliers_best,:),matchedPoints2_best(inliers_best,:),'montage','PlotOptions',{'ro','go','y--'});









