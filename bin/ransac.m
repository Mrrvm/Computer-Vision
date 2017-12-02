% RANSAC

% n - number of random samplings
% threashold - threashold to be considered inlier (0.1 is fine)

function [R,T] = ransac(n,threashold,points1,points2)

inliers = [];
for i=1:n
    indices = randperm(length(points1),4);
    ransac_points1 = points1(indices,:);
    ransac_points2 = points2(indices,:);
    matrix = zeros(12,12);
    output_points = ransac_points1'; output_points = output_points(:);
    for j=1:4
        matrix(j*3-2,:) = [ransac_points2(j,:) 0 0 0 0 0 0 1 0 0];
        matrix(j*3-1,:) = [0 0 0 ransac_points2(j,:) 0 0 0 0 1 0];
        matrix(j*3,:) = [0 0 0 0 0 0 ransac_points2(j,:) 0 0 1];
    end
    
    parameters = inv(matrix)*output_points;
    R = [parameters(1:3)';parameters(4:6)';parameters(7:9)'];
    T = parameters(10:12);
    
    points1_t = R*(points2') + repmat(T,1,size(points2,1));
    points1_t = points1_t';
    distances = (points1_t-points1).^2;
    distances = sqrt(distances(:,1)+distances(:,2)+distances(:,3));
    
    if sum(distances<threashold)>length(inliers)
        inliers = find(distances<threashold);
    end
    
end

% Calculate final transformation

points1 = points1(inliers,:);
points2 = points2(inliers,:);
matrix = zeros(length(inliers),12);
output_points = points1'; output_points = output_points(:);
for j=1:length(inliers)
    matrix(j*3-2,:) = [points2(j,:) 0 0 0 0 0 0 1 0 0];
    matrix(j*3-1,:) = [0 0 0 points2(j,:) 0 0 0 0 1 0];
    matrix(j*3,:) = [0 0 0 0 0 0 points2(j,:) 0 0 1];
end
parameters = inv(matrix'*matrix)*matrix'*output_points;

R = [parameters(1:3)';parameters(4:6)';parameters(7:9)'];
T = parameters(10:12);

end
% 
% kinetic_image_21 = R*(kinetic_image_2_xyz') + repmat(T,1,size(kinetic_image_2_xyz,1));
% kinetic_image_21 = kinetic_image_21';
% kinetic_image_21 = [kinetic_image_2(:,1:3) kinetic_image_21];
% 
% showPointCloud(pointCloud(kinetic_image_1(:,4:6), 'Color', uint8(kinetic_image_1(:,1:3))));
% hold on
% showPointCloud(pointCloud(kinetic_image_21(:,4:6), 'Color', uint8(kinetic_image_21(:,1:3))));
