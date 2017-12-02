%clear all

function im_kinetic = get_kinetic_image(im_rgb, im_depth, K_rgb, K_depth, R_d_to_rgb, T_d_to_rgb)
    u = ones(480,1)*(1:640);
    v = (1:480)'*ones(1,640);

    depth_3D_points = inv(K_depth)*[im_depth(:)'.*v(:)';im_depth(:)'.*u(:)';im_depth(:)'];
    depth_3D_points = [depth_3D_points;ones(1,size(depth_3D_points,2))];

    rgb_2D_points = K_rgb*[R_d_to_rgb T_d_to_rgb]*depth_3D_points;
    rgb_2D_points(1,:) = round(rgb_2D_points(1,:)./rgb_2D_points(3,:));
    rgb_2D_points(2,:) = round(rgb_2D_points(2,:)./rgb_2D_points(3,:));
    im_kinetic = zeros(480,640,6);
    im_kinetic(:,:,6) = im_depth;
    rgb_2D_points(1,rgb_2D_points(1,:)>480) = 480;
    rgb_2D_points(2,rgb_2D_points(2,:)>640) = 640;

    for i=1:(480*640)
        im_kinetic(v(i),u(i),1:3) = im_rgb(rgb_2D_points(1,i),rgb_2D_points(2,i),:);
        im_kinetic(v(i),u(i),4:5) = depth_3D_points(1:2,i);
    end
end

% 
% im_rgb = imread('twocams_books/rgb_image1_1.png');
% load twocams_books/depth1_1.mat
% im_depth = double(depth_array)/1000;
% clear depth_array
% 
% load twocams_books/matlab.mat
% K_rgb = RGB_cam.K; K_depth = Depth_cam.K;
% 
% clearvars -except im_rgb im_depth R_d_to_rgb T_d_to_rgb K_rgb K_depth
% 
% s = pointCloud(reshape(im_kinetic(:,:,4:6),640*480,3), 'Color', uint8(reshape(im_kinetic(:,:,1:3),480*640,3)));
% showPointCloud(s);
