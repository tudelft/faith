%% ======================== NESW FOE estimation =======================

% Author: Raoul Dinaux

% Based on the following paper:
% 	Huang, R., & Ericson, S. (2018, June). An Efficient Way to Estimate the Focus of Expansion. 
% 	In 2018 IEEE 3rd International Conference on Image, Vision and Computing (ICIVC) (pp. 691-695). IEEE.

% Input: (x,y) giving the location of the flow vectors (u,v)
% Output: (x_foe,y_foe) location of FOE estimation

function [x_foe, y_foe] = NESW_FOE(x,y, u, v)

tic
% Projection on axis parallel to x- and y-axis

% Count points northeast and southwest of potential x_FoE and Y_FoE
% Sums over columns (per x_FoE candidate)

weighted_x_l = sign(x-x_FoE).*sign(u).*(sign(x-x_FoE).*sign(u) < 0);
weighted_x_h = sign(x-x_FoE).*sign(u).*(sign(x-x_FoE).*sign(u) > 0);

sum_x_lows = sum(weighted_x_l);
sum_x_highs = sum(weighted_x_h);
score_x = sum_x_lows + sum_x_highs;

[~ , I_x] = max(score_x);

weighted_y_l = sign(y-y_FoE).*sign(v).*(sign(y-y_FoE).*sign(v) < 0);
weighted_y_h = sign(y-y_FoE).*sign(v).*(sign(y-y_FoE).*sign(v) > 0);

sum_y_lows = sum(weighted_y_l);
sum_y_highs = sum(weighted_y_h).^2;
score_y = sum_y_lows + sum_y_highs;

[~ , I_y] = max(score_y);

x_foe = I_x;
y_foe = I_y;
    
end
