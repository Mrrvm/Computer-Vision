clear all

load '/home/imarcher/Dropbox/Tecnico/PIV/Project/cameraparametersAsus.mat'
K_rgb = cam_params.Krgb; K_depth = cam_params.Kdepth;
clearvars -except R_d_to_rgb T_d_to_rgb K_rgb K_depth

d_depth = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/depth1_*.mat');
d_rgb = dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena2/rgb_image1_*.png');
%imgs_kinetic = zeros(480,640,length(d_depth),6);
imgs_kinetic_1 = zeros(480,640,length(d_depth));


for i=1:length(d_depth)
    %im_rgb = imread(sprintf('twocams_books/%s',d_rgb(i).name));
    load(sprintf('%s',d_depth(i).name));
    im_depth = double(depth_array)/1000;
    clear depth_array
    imgs_kinetic_1(:,:,i) = im_depth;
    %imgs_kinetic(:,:,i,:) = get_kinetic_image(im_rgb,im_depth,K_rgb,K_depth,R_d_to_rgb,T_d_to_rgb);
end

background_kinetic_1 = median(imgs_kinetic_1,3);

for i=1:length(d_depth)
    object = bwareafilt(abs(imgs_kinetic_1(:,:,i)-background_kinetic_1)>0.600,[1000 480*640]);
    object = bwlabel(object);
    imagesc(object);
    drawnow;
    pause;
end
