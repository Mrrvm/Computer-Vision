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
        % get xyz 
        xyz1 = get_xyzasus(depth_array1(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
        xyz2 = get_xyzasus(depth_array2(:), [dim(1) dim(2)], (1:dim(1)*dim(2))', cam_params.Kdepth, 1, 0);
        % get all rgb points in depth plane
        img_rgb_indepth1 = get_rgbd(xyz1, img_rgb1, cam_params.R, cam_params.T, cam_params.Krgb);
        img_rgb_indepth2 = get_rgbd(xyz2, img_rgb2, cam_params.R, cam_params.T, cam_params.Krgb);
        % get foreground
        [foreg_bin1, foreg_rgb1, foreg_depth1, foreg_gray1] = get_foreground(depth_array1, img_rgb_indepth1, bgimd1, dim, cam_params);
        [foreg_bin2, foreg_rgb2, foreg_depth2, foreg_gray2] = get_foreground(depth_array2, img_rgb_indepth2, bgimd2, dim, cam_params);
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
        
        % do pointclouds
        pc1 = pointCloud(foreg_xyz1, 'Color', reshape(foreg_rgb1,[dim(1)*dim(2) 3]));
        foreg_xyz2toW = cam2toW.R*foreg_xyz2'+cam2toW.T*ones(1,length(foreg_xyz2));
        pc2 = pointCloud((foreg_xyz2toW)', 'Color', reshape(foreg_rgb2,[dim(1)*dim(2) 3]));
        pcdown1 = pcdownsample(pc1,'gridAverage',0.005);
        pcdown2 = pcdownsample(pc2,'gridAverage',0.005);
        clear pc1 pc2;
        %figure(); showPointCloud(pcdown1);
        %figure(); showPointCloud(pcdown2);
        
        % merge pointclouds
        pcm = pcmerge(pcdown1, pcdown2, 0.001)
        %figure(); pcshow(pcm);
        
        % create graph
        G = create_graph(pcm, 0.2);
        % create MST
        [Tree, pred] = graphminspantree(G);
        % create object vector -> for first frame only!

        % get merged 2D projection
        [image_rgb, image_depth] = pointCloudto2D(pcm, dim, cam_params.Kdepth);      
        
        % get matches between frames
                  
        % update object coordinates in vector
        
        pause;
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

    clear imsd n_seq meddep depth_array;
        
end

function [foreg_label, foreg_rgb, foreg_depth, foreg_gray] = get_foreground(depth_array, rgb_indepth, bgimd, dim, cam_params)

    % calculates foreground label
    foreg_label = bwlabel(bwareafilt(double(depth_array)-bgimd<-100,[600 dim(1)*dim(2)]));
    % erodes in disk shape - THIS CAN BE BETTERED
    se = strel('arbitrary',eye(7))
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
            end
            if foreg_label_eroded(m, n) > 0
                 foreg_depth_eroded(m, n) = depth_array(m, n);
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


function G = create_graph(pc, maxdist)

    xyz = pc.Location;
    length_ = length(xyz);
    conns_matrix = zeros(length_, length_);
        
    for i = 1:length_-1
        for j = (i+1):length_
            dx = xyz(i, 1) - xyz(j, 1);
            dy = xyz(i, 2) - xyz(j, 2);
            dz = xyz(i, 3) - xyz(j, 3);
            d = sqrt((dx^2)+(dy^2)+(dz^2));
            if(d < maxdist)
                conns_matrix(i,j) = d;
            end
        end
    end
    
    G = graph(conns_matrix, 'upper');

end












