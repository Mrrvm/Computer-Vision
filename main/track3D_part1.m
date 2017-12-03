function objects = track3D_part1(imgseq1, imgseq2, cam_params, cam1toW, cam2toW)

    % read all images for bg analysis
    ims1=[]; ims2=[];
    imsd1=[]; imsd2=[];
    for i=1:numel(imgseq1)
        im1=rgb2gray(imread(imgseq1(i).rgb));
        im2=rgb2gray(imread(imgseq2(i).rgb));  
        load(imgseq1(i).depth);
        imsd1=[imsd1 depth_array(:)];
        clear depth_array;
        load(imgseq2(i).depth);
        imsd2=[imsd2 depth_array(:)];
        clear depth_array;
        ims1=[ims1 im1(:)];
        ims2=[ims2 im2(:)];
    end
    clear im1; clear im2;
    
    % calculates background image rgb and depth for both cameras
    medim1 = median(double(ims1),2);
    medim2 = median(double(ims2),2);
    meddep1 = median(double(imsd1),2);
    meddep2 = median(double(imsd2),2);
    bgim1 = (uint8(reshape(medim1,[480 640])));
    bgim2 = (uint8(reshape(medim2,[480 640])));
    bgimd1 = reshape(meddep1,[480 640]);
    bgimd2 = reshape(meddep2,[480 640]);
    clear ims1; clear ims2; clear imsd1; clear imsd2;

    % calculates foreground per frame for cam1 and cam 2
    for i = 1:numel(imgseq1)
        
        % camera1 foreground
        [foreg1_bin, foreg1_rgb, foreg1_gray, foreg1_depth] = get_foreg(imgseq1(i).rgb, imgseq1(i).depth, bgim1, bgimd1);
        % camera2 foreground
        [foreg2_bin, foreg2_rgb, foreg2_gray, foreg2_depth] = get_foreg(imgseq2(i).rgb, imgseq2(i).depth, bgim2, bgimd2);
        
        pause;
        
    end

    objects = 1;
end

function [foreg_bin, foreg_rgb, foreg_gray, foreg_depth] = get_foreg(image_rgb_file, image_depth_file, bgim, bgimd)

    foreg_rgb = uint8(zeros(480,640,3));
    foreg_gray = uint8(zeros(480,640));
    im_rgb = imread(image_rgb_file);
    im_gray = rgb2gray(im_rgb);
    foreg_bin = bwlabel(bwareafilt(abs(double(im_gray)-double(bgim))>40,[1000 480*640]));
    for m = 1:480
        for n = 1:640
            if foreg_bin(m, n) > 0
                foreg_rgb(m, n, :) = im_rgb(m, n, :);
                foreg_gray(m, n) = im_gray(m, n);
            end
        end
    end
    load(image_depth_file);
    foreg_depth=abs(double(depth_array)-double(bgimd))*0.001;
    clear depth_array; clear im_rgb; clear im_gray; clear m; clear n;
end


