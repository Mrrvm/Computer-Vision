d=dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena4/rgb_image1_*.png');
dd=dir('/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena4/depth1_*.mat');
ims=[];
imsd=[];
for i=1:length(d)
    im=imread([d(i).folder '/' d(i).name]);
    load([dd(i).folder '/' dd(i).name]);
    ims=[ims im(:)];
    imsd=[imsd depth_array(:)];
end
medim=median(double(ims),2);
meddep=median(double(imsd),2);
bgim=(uint8(reshape(medim,[480 640 3])));
bgimd=reshape(meddep,[480 640]);

dim = [480 640];
load '/home/imarcher/Dropbox/Tecnico/PIV/Project/datasets/maizena4/depth1_1.mat'
foreg_bin = bwareafilt(double(depth_array)-bgimd<-100,[600 dim(1)*dim(2)]);
% erodes in disk shape - THIS CAN BE BETTERED
se = strel('square', 3);
foreg_bin_eroded = imopen(foreg_bin, se);
    
% calculates foreground depth
foreg_depth = uint16(zeros(dim(1),dim(2)));
foreg_gray = uint8(zeros(dim(1),dim(2)));
foreg_rgb = uint8(zeros(dim(1),dim(2),3));
for m = 1:dim(1)
    for n = 1:dim(2)
        if foreg_bin(m, n) > 0

        end
        if foreg_bin_eroded(m, n) > 0
            foreg_depth(m, n) = depth_array(m, n);
            foreg_rgb(m, n, :) = rgb_indepth(m, n, :);
        end
    end
end