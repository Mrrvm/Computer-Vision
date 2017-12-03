depth2 = load(imgseq2(1).depth);
    
depth2_array = double(depth2.depth_array)*0.001;
[rows,cols] = size(depth2_array);
K = cam_params.Kdepth;
KRT_matrix = K*[cam2toW.R cam2toW.T];

u = ones(rows, 1)*(1:640);
v = (1:480)'*ones(1,cols);

alpha = u.*depth2_array;
beta = v.*depth2_array;

xyz = linsolve(KRT_matrix, ([alpha(:) beta(:) depth2_array(:)])');

rgbd = get_rgbd(xyz(1:3,:)', imread(imgseq2(1).rgb), cam_params.R, cam_params.T, K);
cl = reshape(rgbd, 480*640, 3);
ptc = pointCloud(xyz(1:3,:)', 'Color', cl);
figure();
showPointCloud(ptc);