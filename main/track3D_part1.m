function objects = track3D_part1(imgseq1, imgseq2, cam_params, cam1toW, cam2toW)
    
    % reorder images
    [imgseq1, imgseq2] = reorder_imgseq(imgseq1, imgseq2);
    % get image dimension
    load(imgseq1(1).depth);
    dim = size(depth_array);
    clear depth_array;
    % calculate background
    bgimd1 = get_background(imgseq1, dim);
    bgimd2 = get_background(imgseq2, dim);
    % define objects return vector
    objects = repmat(struct, 1, 20); 
    % get first frame foreground of both cameras
    i = 1;
    [foreg_rgb1, foreg_xyz1, foreg_depth1] = get_foreground(i, imgseq1, imgseq2, cam_params, bgimd1, bgimd2);
    
    pcm1 = get_mergedPC(foreg_xyz1, foreg_rgb1, dim, cam2toW);
    xyz_label = label_objects(pcm1, 0.2);
    xyz_label = sortrows(xyz_label, 4);
    [frame_objs, n_obj1] = detect_objects(xyz_label, 30);
     for i = 1:n_obj1
        objects(i).X = frame_objs(i).X;
        objects(i).Y = frame_objs(i).Y;
        objects(i).Z = frame_objs(i).Z;
        objects(i).frames_tracked = 1;
    end
    clear frame_objs xyz_label;
    
    %% main
    for i = 2:numel(imgseq1)
        
        % get foreground for next frame of both cameras
        [foreg_rgb2, foreg_xyz2, foreg_depth2] = get_foreground(i, imgseq1, imgseq2, cam_params, bgimd1, bgimd2);
        
        % get matches between frames of cam1
        [kpts1.cam1, d1] = vl_sift(im2single(rgb2gray(foreg_rgb1.cam1)));
        [kpts2.cam1, d2] = vl_sift(im2single(rgb2gray(foreg_rgb2.cam1)));
        if ~isempty(kpts1.cam1) && ~isempty(kpts2.cam1)
            [matches.cam1, scores] = vl_ubcmatch(d1, d2, 1.5);
            [drop, perm] = sort(scores, 'descend') ;
            matches.cam1 = matches.cam1(:, perm);
            % transform kpts in 3D for cam1
            kpts1_xyz.cam1 = find_keypointsXYZ(kpts1.cam1, foreg_depth1.cam1, cam_params.Kdepth);
            kpts2_xyz.cam1 = find_keypointsXYZ(kpts2.cam1, foreg_depth2.cam1, cam_params.Kdepth);
        else
            matches.cam1 = [];
            kpts1_xyz.cam1 = [];
            kpts2_xyz.cam1 = [];
        end
        
        % get matches between frames of cam2
        [kpts1.cam2, d1] = vl_sift(im2single(rgb2gray(foreg_rgb1.cam2)));
        [kpts2.cam2, d2] = vl_sift(im2single(rgb2gray(foreg_rgb2.cam2)));
        if ~isempty(kpts1.cam2) && ~isempty(kpts2.cam2)
            [matches.cam2, scores] = vl_ubcmatch(d1, d2, 1.5);
            [drop, perm] = sort(scores, 'descend') ;
            matches.cam2 = matches.cam2(:, perm) ;
            % transform kpts in 3D for cam1
            kpts1_xyz.cam2 = find_keypointsXYZ(kpts1.cam2, foreg_depth1.cam2, cam_params.Kdepth);
            kpts2_xyz.cam2 = find_keypointsXYZ(kpts2.cam2, foreg_depth2.cam2, cam_params.Kdepth);
            % convert kpts of frame 2 to world
            kpts1_xyz.cam2 = (cam2toW.R*kpts1_xyz.cam2'+cam2toW.T*ones(1,length(kpts1_xyz.cam2)))';    
            kpts2_xyz.cam2 = (cam2toW.R*kpts2_xyz.cam2'+cam2toW.T*ones(1,length(kpts2_xyz.cam2)))';
        else
            matches.cam2 = [];
            kpts1_xyz.cam2 = [];
            kpts2_xyz.cam2 = [];
        end
              
        % get merged pointcloud for next frame
        pcm2 = get_mergedPC(foreg_xyz2, foreg_rgb2, dim, cam2toW);
        xyz_label = label_objects(pcm2, 0.2);
        xyz_label = sortrows(xyz_label, 4);
        [frame_objs, n_obj2] = detect_objects(xyz_label, 20);
              
        obj_scores = zeros(100, 100);
        % get matching objects for cam1
        obj_scores = get_obj_scores(obj_scores, matches.cam1, kpts1_xyz.cam1, kpts2_xyz.cam1, frame_objs, objects, n_obj2, n_obj1, i);       
        % get matching objects for cam2
        obj_scores = get_obj_scores(obj_scores, matches.cam2, kpts1_xyz.cam2, kpts2_xyz.cam2, frame_objs, objects, n_obj2, n_obj1, i);
   
        % update objects vector
        [objects, n_obj1] = update_objects(objects, frame_objs, obj_scores, i, n_obj1);
        
        clear obj_scores pcm1 foreg_rbg1 foreg_depth1 kpts1 kpts2 matches drop perm d1 d2 frame_objs xyz_label;
        foreg_rgb1 = foreg_rgb2;
        foreg_depth1 = foreg_depth2;

    end
end

function [imgseq1, imgseq2] = reorder_imgseq(imgseq1, imgseq2)
    im_fields = {'depth'; 'rgb'};
    im_cell = struct2cell(imgseq1);
    sz = size(im_cell);
    im_cell = reshape(im_cell, sz(1), []);
    im_cell = im_cell'; 
    im_cell = natsort(im_cell);
    im_cell = reshape(im_cell', sz);
    imgseq1 = cell2struct(im_cell, im_fields, 1);
    
    im_cell = struct2cell(imgseq2);
    sz = size(im_cell);
    im_cell = reshape(im_cell, sz(1), []);
    im_cell = im_cell'; 
    im_cell = natsort(im_cell);
    im_cell = reshape(im_cell', sz);
    imgseq2 = cell2struct(im_cell, im_fields, 1);
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

function [foreg_rgb, foreg_depth] = calc_foreground(depth_array, rgb_indepth, bgimd, dim)

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
    %figure(); imshow(foreg_rgb);
    %figure(); imagesc([bgimd depth_array foreg_depth foreg_depth_eroded]);
    clear n m se;
end

function [foreg_rgb, foreg_xyz, foreg_depth] = get_foreground(i, imgseq1, imgseq2, cam_params, bgimd1, bgimd2)

    % load depth images
    load(imgseq1(i).depth);
    depth_array1 = depth_array;
    load(imgseq2(i).depth);
    depth_array2 = depth_array;
    % get dimension
    dim = size(depth_array);
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
    [foreg_rgb.cam1, foreg_depth.cam1] = calc_foreground(depth_array1, img_rgb_indepth1, bgimd1, dim);
    [foreg_rgb.cam2, foreg_depth.cam2] = calc_foreground(depth_array2, img_rgb_indepth2, bgimd2, dim);
     % get xyz foreground
    foreg_xyz.cam1 = zeros(dim(1)*dim(2), 3);
    foreg_xyz.cam2 = zeros(dim(1)*dim(2), 3);
    for m = 1:dim(1)
        for n = 1:dim(2)
            if foreg_depth.cam1(m,n) > 0
                index = sub2ind(size(foreg_depth.cam1), m, n);
                foreg_xyz.cam1(index, :) = xyz1(index, :);
            end
            if foreg_depth.cam2(m,n) > 0
                index = sub2ind(size(foreg_depth.cam2), m, n);
                foreg_xyz.cam2(index, :) = xyz2(index, :);
            end
        end
    end
    clear xyz1 xyz2 img_rgb_indepth1 img_rgb_indepth2;
  
end

function pcm = get_mergedPC(foreg_xyz, foreg_rgb, dim, cam2toW)

    pcam1 = pointCloud(foreg_xyz.cam1, 'Color', reshape(foreg_rgb.cam1,[dim(1)*dim(2) 3]));
    foreg_xyz_cam2toW = cam2toW.R*foreg_xyz.cam2'+cam2toW.T*ones(1,length(foreg_xyz.cam2));
    pcam2 = pointCloud((foreg_xyz_cam2toW)', 'Color', reshape(foreg_rgb.cam2,[dim(1)*dim(2) 3]));
    pcdown1 = pcdownsample(pcam1,'gridAverage',0.01);
    pcdown2 = pcdownsample(pcam2,'gridAverage',0.01);
    clear pc1 pc2;
    % merge pointclouds
    pcm = pcmerge(pcdown1, pcdown2, 0.001);
end

function xyz_label = label_objects(pc, maxdist)

    xyz = pc.Location;
    size_ = size(xyz);
    len = size_(1);
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
        
    clear len xyz i j d dx dy dz size_;
end

function [frame_objs, n_obj] = detect_objects(xyz_label, nmin)

    size_ = size(xyz_label);
    len = size_(1);
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
  
    frame_objs = repmat(struct, 1, n_obj); 
    % splits xyz_label in each object
    for i=1:n_obj
        init = obj_counter(i, 3);
        finit = init + obj_counter(i, 2) - 1;
        xs = xyz_label(init:finit, 1);
        ys = xyz_label(init:finit, 2);
        zs = xyz_label(init:finit, 3);
        
        % finds max and min in each dimension
        pmax.x = max(xs);
        pmin.x = min(xs);
        pmax.y = max(ys);
        pmin.y = min(ys);
        pmax.z = max(zs);
        pmin.z = min(zs);
        
        % finds box extremes
        frame_objs(i).X = repmat([pmax.x pmin.x], 1, 4);
        frame_objs(i).Y = repmat([pmax.y pmax.y pmin.y pmin.y], 1, 2);
        frame_objs(i).Z = [pmax.z pmax.z pmax.z pmax.z pmin.z pmin.z pmin.z pmin.z];
    end
end

function xyz_point = convert2DpointTo3D(K, image_depth, u, v) 

    Kx = K(1,1); Cx = K(1,3); Ky = K(2,2); Cy = K(2,3);
    xyz_point = zeros(1, 3);
    if image_depth(v, u) ~= 0
        xyz_point(1, 3) = double(image_depth(v, u))*0.001;
        xyz_point(1, 2) = (xyz_point(1, 3)/Ky) * (v-Cy);
        xyz_point(1, 1) = (xyz_point(1, 3)/Kx) * (u-Cx);
    end
end

function kpts_xyz = find_keypointsXYZ(kpts, depth_array, K)

    sz = size(kpts);
    kpts_xyz = zeros(length(kpts), 3);
    for i = 1:sz(2)
        x = round(kpts(1, i));
        y = round(kpts(2, i));
        if x > 640
            x = 640;
        end
        if y > 480 
            y = 480
        end
        kpts_xyz(i, :) = convert2DpointTo3D(K, depth_array, x, y);
    end
end

function is = is_insideObject(object_pts, pt)
    
    is = 0;
    if pt(1) < max(object_pts.X) && pt(1) > min(object_pts.X)
        if pt(2) < max(object_pts.Y) && pt(2) > min(object_pts.Y)
            if pt(3) < max(object_pts.Z) && pt(3) > min(object_pts.Z)
                is = 1;
            end
        end
    end    
end

function obj_scores = get_obj_scores(obj_scores, matches, kpts1_xyz, kpts2_xyz, frame_objs, objects, n_obj2, n_obj1, i)
    sz = size(matches);
    for m = 1:sz(2)
        xyz1 = kpts1_xyz(matches(1,m), :);
        xyz2 = kpts2_xyz(matches(2,m), :);
        obj1 = 0; obj2 = 0;
        if xyz1(1,1) ~= 0 || xyz1(1,2) ~= 0 || xyz1(1,3) ~= 0
            if xyz2(1,1) ~= 0 || xyz2(1,2) ~= 0 || xyz2(1,3) ~= 0
                % find which object it is for frame 2
                for n = 1:n_obj2
                    if is_insideObject(frame_objs(n), xyz2)
                        obj2 = n;
                        n = n_obj2+1;
                    end
                end
                % find which object it is for frame 1
                for n = 1:n_obj1
                    sz = size(objects(n).X);
                    if sz(1) ~= 0  
                        if i-1 == objects(n).frames_tracked(sz(1))
                            obj_aux.X = objects(n).X(sz(1), :);
                            obj_aux.Y = objects(n).Y(sz(1), :);
                            obj_aux.Z = objects(n).Z(sz(1), :);
                            if is_insideObject(obj_aux, xyz1)
                                obj1 = n;
                                n = n_obj1+1;
                            end
                        end
                    end
                end
                if obj1 ~= 0 && obj2 ~= 0
                    obj_scores(obj1, obj2) = obj_scores(obj1, obj2) + 1;
                end
            end
        end     
    end
end

function [objects, n_obj] = update_objects(objects, new_objs, scores, frame, n_obj)

    for i = 1:length(new_objs)
        [scr, ind] = max(scores(:));
        [m, n] = ind2sub(size(scores), ind);
        scores(m, :) = 0;
        scores(:, n) = 0;
        
        if(scr ~= 0)
            objects(m).X = [objects(m).X; new_objs(n).X]; 
            objects(m).Y = [objects(m).Y; new_objs(n).Y];
            objects(m).Z = [objects(m).Z; new_objs(n).Z];
            objects(m).frames_tracked = [objects(m).frames_tracked frame]; 
            new_objs(n).X = []; new_objs(n).Y = []; new_objs(n).Z = [];
        end 
    end
    
    % add untracked objects as new ones
    for i = 1:length(new_objs)
        if ~isempty(new_objs(i).X)
            n_obj = n_obj+1;
            objects(n_obj).X = new_objs(i).X;
            objects(n_obj).Y = new_objs(i).Y;
            objects(n_obj).Z = new_objs(i).Z;
            objects(n_obj).frames_tracked = frame;         
        end
    end
end





