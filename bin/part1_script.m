%% Project of Image Processing and Vision - Part I
% The objective of this first part is to create a function that receives
% every transformation matrix by argument and then return the coordinates
% of a 3D box around the objects detected. This document explains every
% step necessary to reach the objective.
%
% Elaborated by:
%
% - Pedro Pereira - 78806
%
% - Jo�o Belfo - 78913
%
% - Ricardo Nunes - 78946
%

clear;
%close all;

%% Parameters

minimum_area = 1000;
detection_threashold = 0.1;
R = [0.985130116495649  -0.112667448207015  -0.129710060087059
   0.022601625367192   0.833385134922130  -0.552230371694262
   0.170316822727616   0.541087112215161   0.823539262507033];

T = [0.044365588846338  
    -0.658143239111439   
    0.302504708252999];
%% Background estimation

% Load intrinsic and extrinsic parameters between depth and rgb
load /home/imarcher/Dropbox/Tecnico/PIV/Project/main/cameraparametersAsus.mat;
K_d=cam_params.Kdepth;
K_rgb=cam_params.Krgb;

% Load image directory
d_depth_1 = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/depth1_*.mat');
d_rgb_1 = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/rgb_image1_*.png');
d_depth_2 = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/depth2_*.mat');
d_rgb_2 = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/rgb_image2_*.png');

% Declaration of matrix for depth images
imgs_kinetic_1 = zeros(480,640,length(d_depth_1),6);
imgs_kinetic_2 = zeros(480,640,length(d_depth_2),6);

% Cycle that loads every depth image to a matrix
for i=1:length(d_depth_1)
    load(sprintf('%s',d_depth_1(i).name));
    im_depth = double(depth_array)/1000;
    clear depth_array
    im_rgb = imread(d_rgb_1(i).name);
    imgs_kinetic_1(:,:,i,:) = get_kinetic_image(im_rgb,im_depth,K_rgb,K_d,cam_params.R,cam_params.T);
    %imgs_kinetic_1(:,:,i) = im_depth;
    
    load(sprintf('%s',d_depth_2(i).name));
    im_depth = double(depth_array)/1000;
    clear depth_array
    im_rgb = imread(d_rgb_2(i).name);
    imgs_kinetic_2(:,:,i,:) = get_kinetic_image(im_rgb,im_depth,K_rgb,K_d,cam_params.R,cam_params.T);
end
%%
% Background estimation through the median of every pixel in every image
background_kinetic_1 = median(imgs_kinetic_1(:,:,:,6),3); %background_kinetic_1 = median(imgs_kinetic_1,3);
background_kinetic_2 = median(imgs_kinetic_2(:,:,:,6),3); %background_kinetic_2 = median(imgs_kinetic_2,3);

% Plot the background estimation (depth only)
% figure;
% imagesc(background_kinetic_1);
% figure;
% imagesc(background_kinetic_2);

%% Backgound Subtraction
% For each image, the object is detected subtracting the depth image from
% the estimated background. After that, 

for i=1:length(d_depth_1)
    object_1 = bwareafilt(abs(imgs_kinetic_1(:,:,i,6)-background_kinetic_1)>detection_threashold,[minimum_area 480*640]);
    object_2 = bwareafilt(abs(imgs_kinetic_2(:,:,i,6)-background_kinetic_2)>detection_threashold,[minimum_area 480*640]);
    object_1 = bwlabel(object_1);
    object_2 = bwlabel(object_2);
    
    for j=1:max(object_1(:))
        [v,u] = find(object_1 == j);
        ind_imgs_kinetic_1_r = sub2ind(size(imgs_kinetic_1), v, u, i*ones(length(v),1), 1*ones(length(v),1));
        ind_imgs_kinetic_1_g = sub2ind(size(imgs_kinetic_1), v, u, i*ones(length(v),1), 2*ones(length(v),1));
        ind_imgs_kinetic_1_b = sub2ind(size(imgs_kinetic_1), v, u, i*ones(length(v),1), 3*ones(length(v),1));
        
        ind_imgs_kinetic_1_x = sub2ind(size(imgs_kinetic_1), v, u, i*ones(length(v),1), 4*ones(length(v),1));
        ind_imgs_kinetic_1_y = sub2ind(size(imgs_kinetic_1), v, u, i*ones(length(v),1), 5*ones(length(v),1));
        ind_imgs_kinetic_1_z = sub2ind(size(imgs_kinetic_1), v, u, i*ones(length(v),1), 6*ones(length(v),1));
        objects_1(j).xyz = [imgs_kinetic_1(ind_imgs_kinetic_1_x) imgs_kinetic_1(ind_imgs_kinetic_1_y) imgs_kinetic_1(ind_imgs_kinetic_1_z)];
        objects_1(j).rgb = uint8([imgs_kinetic_1(ind_imgs_kinetic_1_r) imgs_kinetic_1(ind_imgs_kinetic_1_g) imgs_kinetic_1(ind_imgs_kinetic_1_b)]);  
    end
    
    for j=1:max(object_2(:))
        [v,u] = find(object_2 == j);
        ind_imgs_kinetic_2_r = sub2ind(size(imgs_kinetic_2), v, u, i*ones(length(v),1), 1*ones(length(v),1));
        ind_imgs_kinetic_2_g = sub2ind(size(imgs_kinetic_2), v, u, i*ones(length(v),1), 2*ones(length(v),1));
        ind_imgs_kinetic_2_b = sub2ind(size(imgs_kinetic_2), v, u, i*ones(length(v),1), 3*ones(length(v),1));
        
        ind_imgs_kinetic_2_x = sub2ind(size(imgs_kinetic_2), v, u, i*ones(length(v),1), 4*ones(length(v),1));
        ind_imgs_kinetic_2_y = sub2ind(size(imgs_kinetic_2), v, u, i*ones(length(v),1), 5*ones(length(v),1));
        ind_imgs_kinetic_2_z = sub2ind(size(imgs_kinetic_2), v, u, i*ones(length(v),1), 6*ones(length(v),1));
        objects_2(j).xyz = [imgs_kinetic_2(ind_imgs_kinetic_2_x) imgs_kinetic_2(ind_imgs_kinetic_2_y) imgs_kinetic_2(ind_imgs_kinetic_2_z)];
        objects_2(j).rgb = uint8([imgs_kinetic_2(ind_imgs_kinetic_2_r) imgs_kinetic_2(ind_imgs_kinetic_2_g) imgs_kinetic_2(ind_imgs_kinetic_2_b)]);  
    end

    for j=1:length(objects_2)
        objects_2(j).xyz = objects_2(j).xyz*R+ones(length(objects_2(j).xyz),1)*T';
    end
    hold off
    %Display point cloud
    for j=1:length(objects_1)
        p=pointCloud(objects_1(j).xyz, 'Color', objects_1(j).rgb);
        showPointCloud(p);
        hold on
    end
    drawnow;
    pause;
    for j=1:length(objects_2)
        p=pointCloud(objects_2(j).xyz, 'Color', objects_2(j).rgb);
        showPointCloud(p);
        hold on
    end
    drawnow;
    pause;
    view([-2 -64]);
    
end