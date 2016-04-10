%**************************************************************************
% Purpose: Testing mexVolumeMinimization
%
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************


%% settings
clear all; clc; close all;
addpath('../Commons/');

%% Dataset1: dataset from multisensor test
% load('../data/scanners1');
% all = scanners.plane_all;
% ts0 = min(all(:,8));
% all(:,8) = (all(:,8) - ts0)/(1000*1000);
% all = all(and(0 < all(:,8), all(:,8) < 6), :);
% model_cloud = [all(:,1:3), all(:,8)];

%% Dataset2: dataset from simulation
% scenario = 'cessna_ss_a10_v10';
% load('../data/pc_cessna_ss_a10_v10');
% %ts = data(:,1)-data(1,1);
% ts = data(:,1);
% model_cloud = [data(:,6:8), ts];

%% Dataset3: curvylinear motion
load('../data/scanners4');
all = scanners.plane_all;
ts0 = min(all(:,8));
all(:,8) = (all(:,8) - ts0)/(1000*1000);
all = all(and(1 < all(:,8), all(:,8) < 6), :);
all(:,8) = (all(:,8) - min(all(:,8)));
model_cloud = ([all(:,1:3), all(:,8)]);
offset = [0 0.9];


%% Dataset4
% load('../data/velodyne');
% profiles = cloud_model.profiles;
% model_cloud = [];
%  for i = 1 : length(profiles),
%         prof = profiles{i};
%         model_cloud = [model_cloud; prof];
%  end;
%  model_cloud(:,4)= model_cloud(:,4) - min(model_cloud(:,4));

%% CV model
results = [];

settings.bin_size       = 1.5;
settings.verbose        = 1;
settings.n_neighb       = 50;
settings.max_iter       = 200;
settings.model           = 'CV2DoF';
settings.metric            = 'volume';

tic
params = [0 0];
params_baund = [100 100];
[params_cv, bins, cloud_res_cv, iter] = mexEntropyMinimization(model_cloud, params, params_baund, settings);
toc 

start_point = mean(cloud_res_cv, 1);
start_point(1:2) = start_point(1:2) + offset;
[~, trajectory_cv] = mexReconstruct([ repmat(start_point(1:3), size(model_cloud,1), 1) sort(model_cloud(:,4))], -params_cv, settings);


%% CA model 
settings = {};
settings.verbose          = 1;
settings.bin_size         = 1;
settings.n_neighb         = 300;
settings.max_iter         = 100;
settings.model            = 'CA2DoF';
settings.metric            = 'volume';

tic
params = [params_cv 0 0];
params_baund = [30 30 30 30];
[params_ca, bins, cloud_res_ca, iter] = mexEntropyMinimization(model_cloud, params, params_baund, settings);
toc

start_point = mean(cloud_res_ca, 1);
start_point(1:2) = start_point(1:2) + offset;
[~, trajectory_ca] = mexReconstruct([ repmat(start_point(1:3), size(model_cloud,1), 1) sort(model_cloud(:,4))], -params_ca, settings);

% Angle correction
[~, cloud_res_ca] = mexReconstruct(model_cloud, [params_ca  start_point(1:2)], settings);

%% CA model paralelel

%Parallel
% settings.verbose          = 0;
% settings.bin_size         = 2;
% settings.n_neighb         = 20;
% settings.model            = 'CA2DoF';
% %settings.model           = 'CAOA2DoF';
% params = [params_cv 0 0];
% params_baund = [30 30 30 30];
% tic
% [params, bins_res, cloud_res2] = paralelel_reconstruction(model_cloud, params, params_baund, 50, 10, settings);
% toc

 %% Results
 % Display SA results
 figure(2); clf; hold on;
 
 subplot(2,2,1); hold on;
 plot(iter(:,1), iter(:,2), 'r.-');
 xlabel('temp [-]'); ylabel('vx [m/s]');
 set(gca,'XDir','reverse');
 set(gca, 'FontSize', 12)
 grid on;
 
 subplot(2,2,2); hold on;
 plot(iter(:,1), iter(:,3), 'g.-');
 xlabel('temp [-]'); ylabel('vy [m/s]');
 set(gca,'XDir','reverse');
 set(gca, 'FontSize', 12)
 grid on;
 
 subplot(2,2,3); hold on;
 plot(iter(:,1), iter(:,4), 'g.-');
 xlabel('temp [-]'); ylabel('vz [m/s]');
 set(gca,'XDir','reverse');
 set(gca, 'FontSize', 12)
 grid on;
 
 
 subplot(2,2,4); hold on;
 plot3(iter(:,2), iter(:,3), iter(:,end), 'b.-');
 xlabel('v_x [m/s]'); ylabel('v_y [m/s]'); zlabel('entropy [-]');
 view(45,45)
 set(gca, 'FontSize', 12);
 grid on;

 %% Check the solution
figure(1); clf; hold on;
plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'b.');
plot3(cloud_res_cv(:,1), cloud_res_cv(:,2), cloud_res_cv(:,3), 'r.');
plot3(trajectory_cv(:,1), trajectory_cv(:,2), trajectory_cv(:,3), 'r.-', 'LineWidth', 3);
plot3(cloud_res_ca(:,1), cloud_res_ca(:,2), cloud_res_ca(:,3), 'k.');
plot3(trajectory_ca(:,1), trajectory_ca(:,2), trajectory_ca(:,3), 'k.-', 'LineWidth', 3);
plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'b.');
legend('Raw data', 'EM-CV Reconstruction', 'EM-CV Trajectory', 'EM-CA Reconstruction', 'EM-CA Trajectory');
xlabel('X [m]'); ylabel('Y [m]');
set(gca, 'FontSize', 16);
grid on;
axis equal;

%%
figure(2); clf; hold on;
plot3(cloud_res(:,1), cloud_res(:,2), cloud_res(:,3), 'r.');
plot3(model_cloud_recon(:,1), model_cloud_recon(:,2), model_cloud_recon(:,3), 'k.');
axis equal;





