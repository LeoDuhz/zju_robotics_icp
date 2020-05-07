clear;clc;

laser_map = pcread('0.ply');

tform_init = affine3d([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
% tform_init = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
tt=zeros(10,3);
robot_tf{1} = tform_init;

for i = 1:1:9
    
    disp(i);
    
    % read
    str = [num2str(i) , '.ply'];  
    curr_ply = pcread(str);
    
    % icp
    [tform_init, curr_ply] = my_icp(curr_ply, laser_map,  tform_init.T,  100, 0.01, 0.001)
%     [tform_init, curr_ply] = pcregistericp(curr_ply, laser_map, 'Metric','pointToPoint', 'InitialTransform', tform_init, 'MaxIterations', 100, 'Tolerance', [0.01, 0.001]);
    tt(i+1,:)=tform_init.T(4,1:3);
    robot_tf{i+1} = affine3d((robot_tf{i}.T' * tform_init.T')');

    % merge
    laser_map = pcmerge(laser_map, curr_ply, 0.01);
   
end

figure;
pcshow(laser_map, 'MarkerSize', 20);
figure;
point=zeros(3,10);
for i=1:10
    T=robot_tf{1,i}.T;
    t=tt(i,:);
    point(:,i+1)=t';
    plot(point(1,i+1),point(2,i+1),'o');
    plot([point(1,i),point(1,i+1)],[point(2,i),point(2,i+1)]);
    hold on
end

    
    