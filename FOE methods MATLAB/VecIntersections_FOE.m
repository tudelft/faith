%% ======================== Vec. Intersections FOE estimation =======================

% Author: Raoul Dinaux

% Based on the following paper:
% 	Buczko, M., & Willert, V. (2017, June). Monocular outlier detection for visual odometry. 
% 	In 2017 IEEE Intelligent Vehicles Symposium (IV) (pp. 739-745). IEEE.

% Input: (x,y) giving the location of the flow vectors (u,v)
% Output: (x_foe,y_foe) location of FOE estimation

function [x_foe, y_foe] = VecIntersections_FOE(x,y, u, v)
tic

% Set # iterations for RANSAC search
iterations = 100;

% Find first and last index in selected timeslot (required for RANSAC)
col = find(x > 0);
idx_min = min(col);
idx_max = max(col);

% Get angle of optic flow
[OF_theta] = cart2pol(u,v);
beta = pi - OF_theta;
psi = pi/2 - beta;

%% Start RANSAC scheme
for n = 1:iterations
    
    % Select random 2 vectors from timeslot
    idx_rand_1 = idx_min + randi(idx_max-idx_min);
    
    % Calculate vector 1 properties
    x_s1 = x(idx_rand_1);
    y_s1 = y(idx_rand_1);
    theta_s1 = OF_theta_discr(idx_rand_1);
    psi_s1 = psi(idx_rand_1);
    mag_s1 = derotated_rho(idx_rand_1);
    
    % Calculate vector 2 properties
    idx_rand_2 = idx_min + randi(idx_max-idx_min);
    x_s2 = x(idx_rand_2);
    y_s2 = y(idx_rand_2);
    theta_s2 = OF_theta_discr(idx_rand_2);
    psi_s2 = psi(idx_rand_2);
    mag_s2 = derotated_rho(idx_rand_2);
    
    % Check whether the selected vectors are suitable for estimating vector intersection.
    if (abs(theta_s1 - theta_s2) < 100*eps) || (abs(theta_s1 - theta_s2 - pi) < 100*eps) || (abs(theta_s1 - theta_s2 + pi) < 100*eps)
    else
        
        % Calculate intersection (potential FoE) of the two random vectors
        p1 = x_s1 .* cos(psi_s1) + y_s1 .* sin(psi_s1);
        p2 = x_s2 .* cos(psi_s2) + y_s2 .* sin(psi_s2);
        
        cos_sins = [cos(psi_s1) sin(psi_s1) ;cos(psi_s2) sin(psi_s2)];
        p_1_2 = [p1;p2];
        
        FoE_xy_coordinate = cos_sins\p_1_2;
        
        FoE_x_test = FoE_xy_coordinate(1);
        FoE_y_test = FoE_xy_coordinate(2);
        
        % Check if flow of 2 selected vectors is divergent from potential FoE
        d_x = x_s2 - x_s1;
        d_y = y_s2 - y_s1;
        
        [d_theta,~] = cart2pol(d_x,d_y);
        d_theta_s1 = d_theta - theta_s1;
        d_theta_s2 = d_theta - theta_s2;
        
        proj_s1_d = mag_s1 * cos(d_theta_s1);
        proj_s2_d = mag_s2 * cos(d_theta_s2);
        
        % Check if other vectors are divergent from potential FOE location
        if proj_s1_d <= 0 && proj_s2_d <= 0
            d_x_all = (FoE_x_test - x);
            d_y_all = (FoE_y_test - y);
            
            [d_theta_pol,~] = cart2pol(d_x_all,d_y_all);
            d_theta_all = abs(d_theta_pol - OF_theta_discr);
            
            proj_d_all = derotated_rho .* cos(d_theta_all);
            
            % Calculate orthogonal distance to FoE
            p_toFoE = (FoE_x_test - x) .* cos(psi) + (FoE_y_test - y) .* sin(psi);
            
            % Select inliers and outliers based on orthogonal distance threshold
            % and on divergence condition
            inliers = (abs(p_toFoE) < p_threshold) .* (proj_d_all < 0);
            
            nb_inliers = sum(inliers > 0);
            
            % Save the current best fit
            if nb_inliers > nb_inliers_max
                FoE_x_max = FoE_x_test;
                FoE_y_max = FoE_y_test;
                
                nb_inliers_max = nb_inliers;
            end
        end
    end
end

x_foe = FoE_x_max;
y_foe = FoE_y_max;

vecIntersections_timer = toc;

end
