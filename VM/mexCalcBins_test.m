%**************************************************************************
% Purpose: Testing mexCalcBins
%
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************

addpath('../Commons/');

%% Settings
clear all; clc;
cloud_min = 0.8;

%% Load
load('../data/velodyne')
profiles = cloud_model.profiles;

%% Preprocessing

%Select profiles
profsizes = zeros(length(profiles), 1);
dt = nan(1, length(profiles));
for i = 1 : length(profiles),
    profsizes(i) = size(profiles{i}, 1);
    dt(i) = median(profiles{i}(:,4));
end;
msize = max(profsizes);
fidx = find(profsizes > msize*cloud_min);
frames = fidx';
dt = [0 diff(dt)];


%% 
 v = 11; h = 0; bin_size = 0.5;
    
 model_cloud = [];
 for i = 1 : length(frames),
        dx = i*dt(i)*v*cos(h);
        dy = i*dt(i)*v*sin(h);
        prof = profiles{frames(i)};
        prof(:,1:2) = prof(:,1:2) + repmat([dx dy], size(prof, 1), 1);    
        model_cloud = [model_cloud; prof];
 end;
 model_cloud(:,1:3) = model_cloud(:,1:3) - repmat(mean(model_cloud(:,1:3)), size(model_cloud, 1), 1);
 
 % Not getting the occupencies
 tic
 nbins1 = mexCalcBins(model_cloud, bin_size, 0, 0); % entropy
 toc

 % Getting back the occupencies
 bins = mexCalcBins(model_cloud, bin_size, 1, 1); % bin numbers
 nbins2 = size(bins, 1);
 
 % Check that the two solutions are same
 if nbins1 ~= nbins2,
     disp('The two solutions are not equal!');
     return;
 end;
 
 %% Check the solution
figure(1); clf; hold on;
plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'r.');
draw_bins( bins, [bin_size, bin_size, bin_size]);
grid on;
axis equal;



