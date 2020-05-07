clear;clc;
ply_0 = pcread('0.ply');
ply_1 = pcread('1.ply');
tform_init = affine3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
% figure;
% pcshowpair(ply_0, ply_1, 'MarkerSize', 50);

% [tform, ply_1] = pcregistericp(ply_1, ply_0);

[tform_init, ply_1] = my_icp(ply_1, ply_0,  tform_init.T,  100, 0.01, 0.001);
figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);
laser_map = pcmerge(ply_0, ply_1, 0.01);
title('my icp result');

ply_0 = pcread('0.ply');
ply_1 = pcread('1.ply');
tform_init = affine3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
% figure;
% pcshowpair(ply_0, ply_1, 'MarkerSize', 50);

[tform, ply_1] = pcregistericp(ply_1, ply_0,'Metric','pointToPoint', 'InitialTransform', tform_init, 'MaxIterations', 100, 'Tolerance', [0.01, 0.001]);

figure;
pcshowpair(ply_0, ply_1, 'MarkerSize', 50);
laser_map = pcmerge(ply_0, ply_1, 0.01);
title('pcregistericp result');
