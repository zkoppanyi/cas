%**************************************************************************
% Purpose: Demonstrating different velocities give different point clouds
% and entropies
%
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************

addpath('../Commons/');

%% Settings
clear all; clc;
bin_size = 0.5;

%% Load
load('../data/velodyne');
profiles = cloud_model.profiles;
model_cloud = [];
 for i = 1 : length(profiles),
        prof = profiles{i};
        model_cloud = [model_cloud; prof];
 end;
 model_cloud(:,4)= model_cloud(:,4) - min(model_cloud(:,4));
 
%% Reconstruct
settings.model          = 'CV2DoF';
settings.bin_size       = 100;
settings.verbose        = 1;
[entropy, model_cloud] = mexReconstruct(model_cloud, [10 0 0], settings);

 %%
 
 % Getting back the occupencies
 bins = mexCalcBins(model_cloud, bin_size, 1); 
 
%% Check the solution
figure(1); clf; hold on;
plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'b.');
draw_bins( bins, [bin_size, bin_size, bin_size]);
set(gca, 'FontSize', 14);
xlabel('X [m]', 'FontSize', 14);
ylabel('Y [m]', 'FontSize', 14);
grid on;
axis equal;



