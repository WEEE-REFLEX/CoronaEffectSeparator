close all
clear variables
clc

geometric_list = load('geometric_list-CU.txt');

par_sel = 3;
% extract coordinates of the par_sel-th particle
conv_hull_pointsN = geometric_list(par_sel,8);
conv_hull_points = reshape(geometric_list(par_sel,9:8+conv_hull_pointsN*2),[2,conv_hull_pointsN]); % hyp: cx_1, cy_1, cx_2, cy_2, ecc...
conv_hull_points = reshape(geometric_list(par_sel,9:8+conv_hull_pointsN*2),[conv_hull_pointsN,2])';% hyp: cx_1, cx_2, cy_1, cy_2, ecc...



polar_coord_theta = cart2pol(conv_hull_points(1,:),conv_hull_points(2,:));
polar_coord_theta = [polar_coord_theta;1:length(polar_coord_theta)];
polar_coord_theta2 = sortrows(polar_coord_theta');

conv_hull_points_sorted = conv_hull_points(:,polar_coord_theta2(:,2));
patch(conv_hull_points_sorted(1,:),conv_hull_points_sorted(2,:),0)
