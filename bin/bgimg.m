d=dir('data_rgb/rgb_image1_*.png');
dd=dir('data_rgb/depth1_*.mat');
ims=[];
imsd=[];
for i=1:length(d)
    im=rgb2gray(imread(d(i).name));
    imshow(im); colormap(gray);
    load(dd(i).name);
    drawnow;
    ims=[ims im(:)];
    imsd=[imsd depth_array(:)];
end
medim=median(double(ims),2);
meddep=median(double(imsd),2);
%bgim=rgb2gray(uint8(reshape(medim,[480 640 3])));
bgim=(uint8(reshape(medim,[480 640])));
bgimd=reshape(meddep,[480 640]);

for i=1:length(d)
    im=rgb2gray(imread(d(i).name));
    foreg=abs(double(im)-double(bgim))>40;
    load(dd(i).name);
    foregd=abs(double(depth_array)-bgimd)>700;
    figure(1);
    imshow([im 255*foreg 255*imfill(imopen(foreg,strel('disk',5)),'holes')  ]);
    colormap(gray);
    figure(2);
    imagesc([depth_array double(max(depth_array(:)))*[foregd  imopen((foregd),strel('disk',10))]]);
    colormap('jet');
    pause;
end