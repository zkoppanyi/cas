%**************************************************************************
% Purpose: Demonstarate that the problem is non continous
%
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************


%% settings
clear all; clc; close all;
addpath('../Commons/');

%% Dataset
load('../data/velodyne');
profiles = cloud_model.profiles;
model_cloud = [];
 for i = 1 : length(profiles),
        prof = profiles{i};
        model_cloud = [model_cloud; prof];
 end;
 model_cloud(:,4)= model_cloud(:,4) - min(model_cloud(:,4));

%% CV model
results = [];

settings.bin_size       = 1.5;
settings.verbose        = 1;
settings.model           = 'CV2DoF';

[X,Y] = meshgrid(-50:5:50,-50:5:50);
[X,Y] = meshgrid(8:0.1:12,-3:0.1:3);
[X,Y] = meshgrid(10:0.001:10.1,0:0.001:0.1);
dstep = 0.0001;
[X,Y] = meshgrid(10:dstep:10+(dstep*50),0:dstep:0+(dstep*50));
Z = zeros(size(X,1), size(X,2));
for i = 1 : size(X,1),
    for j = 1 : size(X,2),
        [netropy, cloud_res2] = mexReconstruct(model_cloud, [X(i,j) Y(i,j)], settings);
        Z(i,j) = netropy;
    end;
end;

figure(1); clf; hold on;
surf(X,Y,Z);
plot3(X(:),Y(:),Z(:), 'r.');

%%
settings.sa_vxstart      = 10;
settings.sa_vystart      = 0;
settings.sa_vxbaund      = 5;
settings.sa_vybaund      = 5;
settings.n_neighb         = 100;
settings.model           = 'CV2DoF';
[params1, bins, cloud_res, iter] = mexEntropyMinimization(model_cloud, settings);
iter = iter(iter(:,1) ~= 0,:);

[X2,Y2] = meshgrid(min(iter(:,2)):0.01:max(iter(:,2)), min(iter(:,3)):0.01:max(iter(:,3)));
Z2 = zeros(size(X2,1), size(X2,2));
for i = 1 : size(X2,1),
    for j = 1 : size(X2,2),
        [netropy, cloud_res2] = mexReconstruct(model_cloud, [X2(i,j) Y2(i,j)], settings);
        Z2(i,j) = netropy;
    end;
end;


figure(2); clf; hold on;
surf(X2,Y2,Z2);
plot3(iter(:,2),iter(:,3),iter(:,end), 'bo-');
plot3(iter(end,2),iter(end,3),iter(end,end), 'rx');








