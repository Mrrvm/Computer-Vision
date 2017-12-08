% obtains xyz for both camaras
K = cam_params.Kdepth;
load(im1(1).depth);
xyz1 = get_xyzasus(depth_array(:), [480 640], (1:640*480)', K, 1, 0);
load(im1(2).depth);
xyz2 = get_xyzasus(depth_array(:), [480 640], (1:640*480)', K, 1, 0);

xyz1_points = zeros(4, 3);
xyz2_points = zeros(4, 3);
for i = 1:4
    xyz1_points(i, :) = xyz1(sub2ind(size(depth_array), cam1_points(i,1), cam1_points(i, 2)), :);
    xyz2_points(i, :) = xyz2(sub2ind(size(depth_array), cam2_points(i,1), cam2_points(i, 2)), :);
end
[d,xx,tr]=procrustes(xyz1_points, xyz2_points,'scaling',false,'reflection',false);
