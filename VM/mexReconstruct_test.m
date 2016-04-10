%**************************************************************************
% Purpose: Testing mexReconstruct
%
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************


%% Settings
clear all; clc; close all;

%% Load test data
load('../data/scanners1');
all = scanners.plane_all;
ts0 = all(1,8);
all(:,8) = (all(:,8) - ts0)/(1000*1000) + 0.1;
all = all(and(1 < all(:,8), all(:,8) < 2), :);
model_cloud = [all(:,1:3), all(:,8)];
vtxyz0 = [17.0485   -0.4057   -0.2275 0 0 0 0 0];

%% The dataset is not reconstructed, so reconstruct it with the CV model
%% It is also a test for CV model reconstruction

settings.bin_size       = 3;
settings.verbose        = 1;
settings.model          = 'CA3DoF';
%settings.metric         = 'volume';
[nbins1, model_cloud] = mexReconstruct(model_cloud, vtxyz0, settings);
[nbins12] = mexCalcBins(model_cloud, settings.bin_size, 0, 0) % entropy
 
%settings.model        = 'general';
%[~, model_cloud] = mexReconstruct(model_cloud, [0 vtxyz0], settings);
original_cloud = model_cloud;

%% Applying known transformation on the dataset
% X translation
% sdx = 0;
% model_cloud(:,1) = model_cloud(:,1) + sdx; 
% sdt = model_cloud(:,4) - model_cloud(1,4);
% vxsim = sdx/max(sdt);
% model_cloud(:,1) = model_cloud(:,1) - vxsim*sdt;
% vtxyz = vtxyz0 + [0 vxsim 0 0];

%dv = @(t) 5*ones(size(t,1), size(t,2));
dvx = @(t) 100./(t+1);
dsx = @(t) dvx(t).*t;
dvy = @(t) 10./(t+1);
dsy = @(t) dvy(t).*t;
for i = 1 : length(model_cloud(:,4)),
    model_cloud(i,1) = model_cloud(i,1) - integral(dvx,1, model_cloud(i,4)); 
    model_cloud(i,2) = model_cloud(i,2) - integral(dvy,1, model_cloud(i,4)); 
end;

%% Preparing the data for the call

% ts = 1:0.1:2;
% vtxyz = [ts', -dv(ts)', zeros(length(ts), 2)];

ts = 1:0.1:2;
ts(1) = [];  ts(end)=[]; 
%ts(2) = [];
vtxyz = [ts', dvx(ts)', dvy(ts'), zeros(length(ts), 1)];

%% Settings
settings.bin_size       = 0.3;      % default: 1
settings.verbose        = 1;        % VERBOSE = 0 - no results on the output, VERBOSE = 1 - put results to the output (default)
settings.model          = 'general';

%% Call
% model_cloud:      Cloud points                   [X Y Z T]
% vtxyz:            Velocity curves as points      [T VX VY VZ]
%                   If one line specified, than CV model
tic
[nbins2, reconstruction] = mexReconstruct(model_cloud, vtxyz , settings);
toc

%% Display the results
figure(1); clf; hold on;
plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'b.');
plot3(reconstruction(:,1), reconstruction(:,2), reconstruction(:,3), 'r.'); 
plot3(original_cloud(:,1), original_cloud(:,2), original_cloud(:,3), 'g.');
grid on;
axis equal;




