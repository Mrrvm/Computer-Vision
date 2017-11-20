function objects = track3D_part1(imgseq1, imgseq2, cam_params, cam1toW, cam2toW)

	%remove background fom imgseq1 and imgseq2
	dir_rgb1 = dir('data_rgb/rgb_image1_*.png');
	dir_rgb2 = dir('data_rgb/rgb_image2_*.png');
	dir_depth1 = dir('data_rgb/depth1_*.mat');
	dir_depth2 = dir('data_rgb/depth2_*.mat');

	ims=[];
	imsd=[];
	for i=1:length(dir_rgb1)
	    im=rgb2gray(imread(dir_rgb1(i).name));
	    imshow(im); colormap(gray);
	    load(dir_depth1(i).name);
	    drawnow;
	    ims=[ims im(:)];
	    imsd=[imsd depth_array(:)];
	end

	medim=median(double(ims),2);
	meddep=median(double(imsd),2);
	bgim=(uint8(reshape(medim,[480 640])));
	bgimd=reshape(meddep,[480 640]);

	for i=1:length(dir_rgb1)
	    im=rgb2gray(imread(dir_rgb1(i).name));
	    foreg=abs(double(im)-double(bgim))>40;
	    figure(1);
	    imshow([im 255*foreg 255*imfill(imopen(foreg,strel('disk',5)),'holes')  ]);
    end
    
end



