%% ======================== FAITH FOE estimation =======================

% Author: Raoul Dinaux

% Corresponding paper: 
%       [TODO]

% Input: (x,y) giving the location of the flow vectors (u,v)
% Output: (x_foe,y_foe) location of FOE estimation

function [x_foe, y_foe] = FAITH_FOE(x,y, u, v)
tic

% Set # iterations for RANSAC search
iterations = 100;                   

% Set failure threshold in iteration to stop search (standard = 1)
max_failures = 1;                                

u = u(x > 0);
v = v(x > 0);
x1 = x(x > 0);
x2 = x(x > 0) + u;
y1 = y(x > 0);
y2 = y(x > 0) + v;

% In format ax + by + c = 0
A = (y1-y2);
B = (x2-x1);

% In format ax + by + c = 0
norm_C = -(B.*x1 - A.*y1);
norm_lines = [B, -A, norm_C];

% Create a list with the rules for all vectors
% Rule: [a, b, c, sign(v), sign(u)]
all_rules = [norm_lines,sign(v),sign(u)];

% Put line descriptions in same format for halfplane check
all_rules(all_rules(:,1) < 0,1:3) = -all_rules(all_rules(:,1) < 0,1:3);

%% Initialize FOV boundaries to hull
bot_bound = [0 1 0];
left_bound = [1 0 0];
top_bound = [0 1 -180];
right_bound = [1 0 -240];

% Rules: [A, B, C, sign(v), sign(u) vec_idx]
bot_bound_rule = [bot_bound, -1, 0, 0];
left_bound_rule = [left_bound, 0, -1, 0];
top_bound_rule = [top_bound, 1, 0, 0];
right_bound_rule = [right_bound, 0, 1, 0];

init_hull_lines = [bot_bound;left_bound;top_bound;right_bound];
init_hull_points = [0,0; 0,180; 240,180; 240,0];
init_hull_rules = [bot_bound_rule;left_bound_rule;top_bound_rule;right_bound_rule];

% Initialize iteration score
max_score = 0;

%% Start of RANSAC scheme
for n = 1:iterations
    
    % Initiate empty hull
    vec_in_hull = zeros(length(norm_lines(:,1)),1);
    vec_tried = zeros(length(norm_lines(:,1)),1);
    
    hull_lines = init_hull_lines;
    hull_points = init_hull_points;
    hull_rules = init_hull_rules;
    
    stop_search = false;
    stop_cnt = 0;
    
    % Keep adding a randomly chosen vector to the hull
    % Stop if new vector violates hull rules
    
    while stop_search == false
        
        % Choose new random vector
        rand_N = randi(length(norm_lines(:,1)));
        while (vec_in_hull(rand_N) == 1 || vec_tried(rand_N) == 1) && (sum(vec_in_hull) < length(vec_in_hull))
            rand_N = randi(length(norm_lines(:,1)));
        end
        vec_tried(rand_N) = 1;
        
        % Stop also if all vectors belong to hull, or all vectors
        % have been tried
        if (sum(vec_in_hull) == length(vec_in_hull)) || (sum(vec_tried) == length(vec_tried))
            stop_search = true;
        end
        
        vec_N = norm_lines(rand_N,:);
        
        % Rule: [A, B, C, sign(v), sign(u), vec_idx]
        vec_N_rule = [vec_N,sign(v(rand_N)), sign(u(rand_N)), rand_N];
        
        % Use same signs in all line descriptions
        if vec_N(1) < 0
            vec_N = -vec_N;
            vec_N_rule(1:3) = -vec_N_rule(1:3);
        end
        
        %% Calculate and check all intersections of selected vector and hull
        add_check = false;
        illegal_cnt = 0;
        for o = 1:length(hull_lines(:,1))
            A_merge = [hull_lines(o,1), hull_lines(o,2); vec_N(1), vec_N(2)];
            C_merge = [-hull_lines(o,3);-vec_N(3)];
            
            % Skip if lines are parallel (necessary for A inversion)
            if (abs(det(A_merge)) > eps)
                
                % calculate intersection of lines
                intersect_xy = A_merge\C_merge;
                
                inter_x = intersect_xy(1);
                inter_y = intersect_xy(2);
                
                % Check if intersection adheres to hull rules
                illegal = false;
                for w = 1:length(hull_rules(:,1))
                    rule = hull_rules(w,:);
                    if (sign(rule(2)) == 1  && rule(4) == 1) || (sign(rule(2)) == -1 && rule(4) == -1) || (sign(rule(2)) == 0  && rule(5) == 1) || (sign(rule(1)) == 0 && rule(4) == 1)
                        if (rule(1)*inter_x + rule(2)*inter_y + rule(3)) <= eps
                        else
                            illegal = true;
                            illegal_cnt = illegal_cnt + 1;
                        end
                    elseif (sign(rule(2)) == -1 && rule(4) == 1) || (sign(rule(2)) == 1 && rule(4) == -1) || (sign(rule(2)) == 0  && rule(5) == -1) || (sign(rule(1)) == 0 && rule(4) == -1)
                        if (rule(1)*inter_x + rule(2)*inter_y + rule(3)) >= -eps
                        else
                            illegal = true;
                            illegal_cnt = illegal_cnt + 1;
                        end
                    end
                end
                
                %% Add vector to hull if no rules are trespassed
                if illegal == false && add_check == false
                    hull_lines= [hull_lines;vec_N];
                    hull_rules= [hull_rules;vec_N_rule];
                    
                    vec_in_hull(rand_N) = true;
                    add_check = true;
                end
                
                % Add intersection to hull
                if illegal == false
                    hull_points = [hull_points;inter_x,inter_y];
                    add_check = true;
                end
                
                % Stop search if vector is not contributing towards smaller hull
                if illegal_cnt == length(hull_rules(:,1))
                    stop_cnt = stop_cnt+1;
                    if stop_cnt == max_failures
                        stop_search = true;
                    end
                end
            else
                % Count parallel as illegal
                illegal_cnt = illegal_cnt + 1;
            end
        end
        
        %% Check new hull points & lines for hull conditions
        remove_point = [];
        for z = 1:length(hull_points(:,1))
            
            %Select intersection from hull
            point_select_x = hull_points(z,1);
            point_select_y = hull_points(z,2);
            
            % Pick latest rule
            rule_select = hull_rules(end,:);
            
            illegal = false;
            
            % Check selected intersection against latest rule
            %% POSITIVE VERTICAL VECTOR
            if ((sign(rule_select(2)) == 1  && rule_select(4) == 1) || (sign(rule_select(2)) == -1 && rule_select(4) == -1) || (sign(rule_select(2)) == 0  && rule_select(5) == 1) || (sign(rule_select(1)) == 0 && rule_select(4) == 1)) && illegal == false
                if (rule_select(1)*point_select_x + rule_select(2)*point_select_y + rule_select(3)) <= eps
                else
                    remove_point = [remove_point;z];
                    illegal = true;
                end
            elseif ((sign(rule_select(2)) == -1 && rule_select(4) == 1) || (sign(rule_select(2)) == 1 && rule_select(4) == -1) || (sign(rule_select(2)) == 0  && rule_select(5) == -1) || (sign(rule_select(1)) == 0 && rule_select(4) == -1)) && illegal == false
                if (rule_select(1)*point_select_x + rule_select(2)*point_select_y + rule_select(3)) >= -eps
                else
                    remove_point = [remove_point;z];
                    illegal = true;
                end
            end
        end
        
        hull_points(remove_point,:) = [];
        
        % Check hull lines for new hull conditions
        % (hull points are already updated)
        remove_line = [];
        for it = 1:length(hull_lines(:,1))
            rule_check = hull_rules(it,1) .* hull_points(:,1) + hull_rules(it,2) .* hull_points(:,2) + hull_rules(it,3);
            if sum((rule_check < eps).*(rule_check > -eps) ) == 0
                remove_line = [remove_line;it];
                if hull_rules(it,6) ~= 0
                    vec_in_hull(hull_rules(it,6)) = false;
                end
            end
        end
        
        hull_lines(remove_line,:) = [];
        hull_rules(remove_line,:) = [];
        
    end
    
    %% Calculate score (= amount of correct halfplanes over all vectors for proposed FoE center)
    % Center coordinate
    x_it_min = min(hull_points(:,1));
    x_it_max = max(hull_points(:,1));
    
    y_it_min = min(hull_points(:,2));
    y_it_max = max(hull_points(:,2));
    
    x_it_center = (x_it_max + x_it_min)/2;
    y_it_center = (y_it_max + y_it_min)/2;
    
    inlier_cnt = 0;
    for d = 1:length(all_rules(:,1))
        rule = all_rules(d,:);
        
        if (sign(rule(2)) == 1  && rule(4) == 1) || (sign(rule(2)) == -1 && rule(4) == -1) || (sign(rule(2)) == 0  && rule(5) == 1) || (sign(rule(1)) == 0 && rule(4) == 1)
            if (rule(1)*_it_center + rule(2)*y_it_center + rule(3)) <= eps
                inlier_cnt = inlier_cnt + 1;
            else
            end
        elseif (sign(rule(2)) == -1 && rule(4) == 1) || (sign(rule(2)) == 1 && rule(4) == -1) || (sign(rule(2)) == 0  && rule(5) == -1) || (sign(rule(1)) == 0 && rule(4) == -1)
            if (rule(1)*x_it_center + rule(2)*y_it_center + rule(3)) >= -eps
                inlier_cnt = inlier_cnt + 1;
            else
            end
        end
    end
    
    iter_score = inlier_cnt;
    
    if iter_score > max_score
        max_score = iter_score;
        best_hull_p = hull_points;
    end
    clear hull_lines hull_points hull_rules remove_point remove_line
end

% Center coordinate
x_min = min(best_hull_p(:,1));
x_max = max(best_hull_p(:,1));

y_min = min(best_hull_p(:,2));
y_max = max(best_hull_p(:,2));

x_foe = (x_max + x_min)/2;
y_foe = (y_max + y_min)/2;

FAITH_timer = toc;

end % END function
