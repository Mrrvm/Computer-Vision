function objects = track3D_part1(imgseq1, imgseq2, cam_params, cam1toW, cam2toW)

    % get image dimension
    load(imgseq1(1).depth);
    dim = size(depth_array);
    clear depth_array;
    
    % camera1 background
    bgimd1 = get_background(imgseq1, dim);
    % camera2 background
    bgimd2 = get_background(imgseq2, dim);
       
    % calculates foreground per frame for cam1 and cam 2
    for i = 1:numel(imgseq1)
        
        % load depth images
        load(imgseq1(i).depth);
        depth_array1 = depth_array;
        load(imgseq2(i).depth);
        depth_array2 = depth_array;
        % load rgb images
        img_rgb1 = imread(imgseq1(i).rgb);
        img_rgb2 = imread(imgseq2(i).rgb);
        figure(1);
        imshow(img_rgb1);
        figure(2);
        imshow(img_rgb2);
        % get xyz 
        xyz1 = get_xyzasus(depth_array1(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
        xyz2 = get_xyzasus(depth_array2(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
        % get all rgb points in depth plane
        img_rgb_indepth1 = get_rgbd(xyz1, img_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
        img_rgb_indepth2 = get_rgbd(xyz2, img_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);
        % get foreground
        [foreg_bin1, foreg_rgb1, foreg_depth1, foreg_gray1] = get_foreground(depth_array1, img_rgb_indepth1, bgimd1, dim, cam_params);
        [foreg_bin2, foreg_rgb2, foreg_depth2, foreg_gray2] = get_foreground(depth_array2, img_rgb_indepth2, bgimd2, dim, cam_params);
        pause;
        %{
        xyz1 = get_xyzasus(foreg1_depth(:), [480 640], (1:640*480)', cam_params.Kdepth, 1, 0);
        xyz2 = get_xyzasus(foreg2_depth(:), [480 640], (1:640*480)', cam_params.Kdepth, 1, 0);
        rgbd1 = get_rgbd(xyz1, foreg1_rgb, cam_params.R, cam_params.T, cam_params.Krgb);
        rgbd2 = get_rgbd(xyz2, foreg2_rgb, cam_params.R, cam_params.T, cam_params.Krgb);
        pc1 = pointCloud(xyz1, 'Color', reshape(rgbd1,[480*640 3]));
        pc2 = pointCloud(xyz2*cam2toW.R+ones(length(xyz2),1)*cam2toW.T,'Color',reshape(rgbd2,[480*640 3]));
        figure(1);
        showPointCloud(pc1);
        figure(2);
        showPointCloud(pc2);
        figure(3);
        pcshow(pcmerge(pc1,pc2,0.001));
        pause;
        %}
    end

    objects = 1;
end

function bgimd = get_background(imgseq, dim)

    % get all or 100 images for bg analysis
    n_seq = numel(imgseq);
    if n_seq > 100 
        n_seq = 100;
    end
    
    imsd=[];
    for i=1:n_seq
        load(imgseq(i).depth);
        imsd=[imsd depth_array(:)];
    end

    % get depth median
    meddep=median(double(imsd),2);
    bgimd=reshape(meddep,[dim(1) dim(2)]);

    clear imsd; clear n_seq; clear meddep; clear depth_array;
        
end

function [foreg_label, foreg_rgb, foreg_depth, foreg_gray] = get_foreground(depth_array, rgb_indepth, bgimd, dim, cam_params)

    % calculates foreground label
    foreg_label = bwlabel(bwareafilt(double(depth_array)-bgimd<-100,[600 dim(1)*dim(2)]));
    % erodes in disk shape
    se = strel('disk',5);
    foreg_label_eroded = imerode(foreg_label, se);
    
    % calculates foreground depth
    foreg_depth = uint16(zeros(dim(1),dim(2)));
    foreg_depth_eroded = uint16(zeros(dim(1),dim(2)));
    foreg_gray = uint8(zeros(dim(1),dim(2)));
    foreg_rgb = uint8(zeros(dim(1),dim(2),3));
    for m = 1:dim(1)
        for n = 1:dim(2)
            if foreg_label(m, n) > 0
                foreg_depth(m, n) = depth_array(m, n);
                foreg_rgb(m, n, :) = rgb_indepth(m, n, :);
            end
            if foreg_label_eroded(m, n) > 0
                 foreg_depth_eroded(m, n) = depth_array(m, n);
            end
        end
    end
    figure();
    imshow(foreg_rgb);
    figure();
    imagesc([bgimd depth_array foreg_depth foreg_depth_eroded]);
    clear n; clear m; clear se;
end

















