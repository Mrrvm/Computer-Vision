        for j = 1:length(matchedPoints1)
            if j ~= maybeinliers(1) && j ~= maybeinliers(2) && j ~= maybeinliers(3) && j ~= maybeinliers(4)
                pt1 = xyz1(sub2ind(dim, matchedPoints1(j, 2), matchedPoints1(j,1)), :);
                pt2 = xyz2(sub2ind(dim, matchedPoints2(j, 2), matchedPoints2(j,1)), :);
                pt2W = pt2*tr.T+tr.c(1,:);
                dist = sqrt((pt2W(1,1)-pt1(1,1))^2)+((pt2W(1,2)-pt1(1,2))^2)+((pt2W(1,3)-pt1(1,3))^2);
                if dist < 0.3
                    alsoinliers = [alsoinliers j];
                end
            end
        end
        
        alsoinliers = [alsoinliers maybeinliers];
        final_points1(:, :) = xyz1(sub2ind(dim, matchedPoints1(alsoinliers, 2), matchedPoints1(alsoinliers,1)), :);
        final_points2(:, :) = xyz2(sub2ind(dim, matchedPoints2(alsoinliers, 2), matchedPoints2(alsoinliers,1)), :);
        [d, xx, tr] = procrustes(final_points1, final_points2,'scaling',false,'reflection',false);
        clear final_points1 final_points2 xyz1_points xyz2_points;