%% ======================== Half-planes FOE estimation =======================

% Author: Raoul Dinaux

% Based on the following paper:
% 	Clady, X. et al. (2014). Asynchronous visual event-based time-to-contact. 
% 	Frontiers in neuroscience, 8, 9.

% Input: (x,y) giving the location of the flow vectors (u,v)
% Output: (x_foe,y_foe) location of FOE estimation

function [x_foe, y_foe] = halfplanes_FOE(x,y, u, v)
tic

% Create meshgrid as probability map
x_lin = 1:240;
y_lin = 1:180;

[X_grid, Y_grid] = meshgrid(x_lin,y_lin);

% Create 'last timestep' update and probability map 
% [PLACE THIS OUTSIDE OF FUNCTION]
last_update = zeros(length(y_lin),length(x_lin));
prob_field = zeros(length(y_lin),length(x_lin));

% Decay FoE grid from previous timestep
prob_field = prob_field .* exp(-(k - last_update));

%% Start new grid estimation
% In line format ax + by + c = 0
u = u(x > 0);
v = v(x > 0);
x1 = x(x > 0);
x2 = x(x > 0) + u;
y1 = y(x > 0);
y2 = y(x > 0) + v;

A = (y1-y2);
B = (x2-x1);

% Calculate the normal lines of the vectors
norm_C = -(B.*x1 - A.*y1);
norm_lines = [B, -A, norm_C];

% Create a list with the rules for all vectors
% Rule: [A, B, C, sign(v), sign(u)]
all_rules = [norm_lines,sign(v),sign(u)];

% Put line descriptions in same format for halfplane check
all_rules(all_rules(:,1) < 0,1:3) = -all_rules(all_rules(:,1) < 0,1:3);


%% Evaluate all points in probability map for all vectors
for w = 1:length(all_rules(:,1))
    
    rule = all_rules(w,:);
    
    % Create a temporary grid, to add to final grid.
    eval_field = rule(1)*X_grid + rule(2)*Y_grid + rule(3);
    
    % Evaluate meshgrid for all rules, add 1 to new probability field if point is feasible.
    if (sign(rule(2)) == 1  && rule(4) == 1) || (sign(rule(2)) == -1 && rule(4) == -1) || (sign(rule(2)) == 0  && rule(5) == 1) || (sign(rule(1)) == 0 && rule(4) == 1)
        eval_field(eval_field > 0) = 0;
        eval_field(eval_field < 0) = 1;
    elseif (sign(rule(2)) == -1 && rule(4) == 1) || (sign(rule(2)) == 1 && rule(4) == -1) || (sign(rule(2)) == 0  && rule(5) == -1) || (sign(rule(1)) == 0 && rule(4) == -1)
        eval_field(eval_field < 0) = 0;
        eval_field(eval_field > 0) = 1;
    end
    
    % Add the temporary grid to the final grid
    prob_field = prob_field + eval_field;
    
    % Keep record of last update timestep for all pixels
    last_update = max(eval_field*k,last_update);
    
end

% Evaluate the maximum of the probability field, and take mean idx (in case
% of multiple maxima)
max_val = max(max(prob_field));

max_idxs = find(max(prob_field) == max_val);
mean_idx = mean(max_idxs);

x_foe = mean_idx;

% y_foe not in scope of research
y_foe = 0;

halfplane_timer = toc;

end
