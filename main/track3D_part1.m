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
        %figure(1); imshow(img_rgb1);
        %figure(2); imshow(img_rgb2);
        % get xyz maizena2
        xyz1 = get_xyzasus(depth_array1(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
        xyz2 = get_xyzasus(depth_array2(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
        % get all rgb points in depth plane
        img_rgb_indepth1 = get_rgbd(xyz1, img_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
        img_rgb_indepth2 = get_rgbd(xyz2, img_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);
        % get foreground
        [foreg_bin1, foreg_rgb1, foreg_depth1, foreg_gray1] = get_foreground(depth_array1, img_rgb_indepth1, bgimd1, dim);
        [foreg_bin2, foreg_rgb2, foreg_depth2, foreg_gray2] = get_foreground(depth_array2, img_rgb_indepth2, bgimd2, dim);
        % get xyz foreground
        foreg_xyz1 = zeros(dim(1)*dim(2), 3);
        foreg_xyz2 = zeros(dim(1)*dim(2), 3);
        for m = 1:dim(1)
            for n = 1:dim(2)
                if foreg_depth1(m,n) > 0
                    index = sub2ind(size(foreg_depth1), m, n);
                    foreg_xyz1(index, :) = xyz1(index, :);
                end
                if foreg_depth2(m,n) > 0
                    index = sub2ind(size(foreg_depth2), m, n);
                    foreg_xyz2(index, :) = xyz2(index, :);
                end
            end
        end
        clear xyz1 xyz2 img_rgb_indepth1 img_rgb_indepth2;
        % do pointclouds
        pc1 = pointCloud(foreg_xyz1, 'Color', reshape(foreg_rgb1,[dim(1)*dim(2) 3]));
        foreg_xyz2toW = cam2toW.R*foreg_xyz2'+cam2toW.T*ones(1,length(foreg_xyz2));
        pc2 = pointCloud((foreg_xyz2toW)', 'Color', reshape(foreg_rgb2,[dim(1)*dim(2) 3]));
        pcdown1 = pcdownsample(pc1,'gridAverage',0.03);
        pcdown2 = pcdownsample(pc2,'gridAverage',0.03);
        clear pc1 pc2;
        %figure(); showPointCloud(pcdown1);
        %figure(); showPointCloud(pcdown2);
        
        % merge pointclouds
        pcm = pcmerge(pcdown1, pcdown2, 0.01);
        %figure(); pcshow(pcm);
        
        % label objects with points distancing a max of 20cm
        xyz_label = label_objects(pcm, 0.2);
        % sort vector by labels ascendingly
        xyz_label = sortrows(xyz_label,4);
        
        % detect objects with more than 20 points
        [objects centroids] = detect_object(xyz_label, 20);
        
        % get merged 2D projection
        [image_rgb, image_depth] = pointCloudto2D(pcm, dim, cam_params.Kdepth);   

        % get matches between frames
                  
        % update object coordinates in vector
        
        pause;
    end

    objects = 1;
end


function bgimd = get_background(imgseq, dim)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GET_BACKGROUND - Get the background from a depth image sequence
%                   using the median.
%
%   INPUT
%   imgseq  - a sequence of structures in which imgseq(i).depth
%                   is the image in depth.
%   dim     - image dimension.
%
%   OUPUT
%   bgimd   - the background image in depth.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

    clear imsd n_seq meddep depth_array;
        
end

function [foreg_bin, foreg_rgb, foreg_depth, foreg_gray] = get_foreground(depth_array, rgb_indepth, bgimd, dim)

    % calculates foreground label
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
    foreg_gray = rgb2gray(foreg_rgb);
    %figure(); imshow(foreg_rgb);
    %figure(); imagesc([bgimd depth_array foreg_depth foreg_depth_eroded]);
    clear n m se;
end

function [image_rgb, image_depth] = pointCloudto2D(pc, dim, K)
    
    Kx = K(1,1); Cx = K(1,3); Ky = K(2,2); Cy = K(2,3);
    xyz = pc.Location';
    xyz_rgb = pc.Color;
    
    X = xyz(1,:);
    Y = xyz(2,:);
    Z = xyz(3,:);
    
    dx = round(Kx*X + Cx*Z);
    dy = round(Ky*Y + Cy*Z);
    d  = Z;
    x  = round(dx./d);
    y  = round(dy./d);    
    
    image_depth = zeros(dim(1), dim(2));
    image_rgb = uint8(zeros(dim(1), dim(2), 3));
    for i = 1:length(x)
        if x(i) < dim(2) && y(i) < dim(1) && x(i) > 1 && y(i) > 1
            image_depth(x(i), y(i)) = d(i);
            image_rgb(x(i), y(i), :) = xyz_rgb(i, :);
        end
    end
    clear X Y Z dx dy x y i Kx Ky Cx Cy;
end

function [xyz_label npts] = label_objects(pc, maxdist)

    xyz = pc.Location;
    len = length(xyz);
    xyz_label = [xyz zeros(len, 1)];
    for i = 1:len
        xyz_label(i,4) = i;
    end
                
    for i = 1:len-1
        for j = (i+1):len
            dx = xyz(i, 1) - xyz(j, 1);
            dy = xyz(i, 2) - xyz(j, 2);
            dz = xyz(i, 3) - xyz(j, 3);
            d = sqrt((dx^2)+(dy^2)+(dz^2));
            if d < maxdist
                xyz_label(j,4) = xyz_label(i,4);
            end
        end
    end
        
    clear len xyz i j d dx dy dz;
end

function [objects centroids] = detect_object(xyz_label, nmin)

    len = length(xyz_label);
    obj_counter = zeros(20, 3);
    
    n_obj = 1;
    label = 0;
    
    % get how many objects and points per each there are
    for i=1:len
        if label ~= xyz_label(i, 4)
            if obj_counter(n_obj,2) >= nmin
               n_obj = n_obj+1;
            end
            label = xyz_label(i, 4);
            obj_counter(n_obj, 1) = label; 
            obj_counter(n_obj, 2) = 0;
            obj_counter(n_obj, 3) = i; 
        end
        obj_counter(n_obj, 2) = obj_counter(n_obj, 2) + 1;
    end
    % for the last object found
    if obj_counter(n_obj,2) < nmin
        obj_counter(n_obj, :) = [0 0 0];
        n_obj = n_obj-1;
    end
  
    objs_xyz = repmat(struct, 1, n_obj); 
    % splits xyz_label in each object
    for i=1:n_obj
        init = obj_counter(i, 3);
        finit = init + obj_counter(i, 2) - 1;
        objs_xyz(i).x = xyz_label(init:finit, 1);
        objs_xyz(i).y = xyz_label(init:finit, 2);
        objs_xyz(i).z = xyz_label(init:finit, 3);
        
        % finds max and min in each dimmension
        pmax.x = max(objs_xyz(i).x);
        pmin.x = min(objs_xyz(i).x);
        pmax.y = max(objs_xyz(i).y);
        pmin.y = min(objs_xyz(i).y);
        pmax.z = max(objs_xyz(i).z);
        pmin.z = min(objs_xyz(i).z);
                
    end
 
    centroids = 0;
end





