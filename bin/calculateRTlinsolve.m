%imshow(im1(1).rgb);
%[u1 v1] = ginput(4);
u1 = [176.219725818736;210.340060929170;279.068164508759;448.207539984768];
v1 = [303.135186595583;360.652322924600;364.064356435644;300.210586443260];
cam1_points = fix([u1 v1]);
cam1_pts_homo = [cam1_points ones(4,1)];

% define 4 points in cam1
%imshow(im2(1).rgb);
%[u2 v2]=ginput(4);
u2 = [163.047770700637;288.398089171975;353.475432211101;405.886715195632];
v2 = [197.260691537762;242.683803457689;237.442675159236;180.663785259327];
cam2_points = fix([u2 v2]);
cam2_pts_homo = [cam2_points ones(4,1)];

K = cam_params.Kdepth;
M1 = K*([cam1toW.R (cam1toW.T)']);
depth1 = load(im1(1).depth);
depth2 = load(im1(2).depth);
depth1_array = zeros(1, 4);
depth2_array = zeros(1, 4);
xyz = zeros(4, 4);
for i=1:4
    depth1_array(i) = double(depth1.depth_array(cam1_points(i, 1), cam1_points(i, 2)))*0.001;
    depth2_array(i) = double(depth2.depth_array(cam2_points(i, 1), cam2_points(i, 2)))*0.001;
    for j=1:3
        cam1_pts_homo(i, j) = cam1_pts_homo(i, j)*depth1_array(i);
        cam2_pts_homo(i, j) = cam2_pts_homo(i, j)*depth2_array(i);
    end
    xyz(i, :) = linsolve(M1,cam1_pts_homo(i, :)')';
    xyz(i, 4) = 1;
end

% obtains R and T from cam2 to cam1 (aka world)
KRT_matrix = zeros(3, 4);
RT_matrix = zeros(3, 4);
KRT_matrix = linsolve(xyz,cam2_pts_homo)';
RT_matrix = linsolve(K, KRT_matrix);

cam2toW.T = RT_matrix(:,4);
cam2toW.R = RT_matrix(1:3,1:3);