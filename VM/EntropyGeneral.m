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
load('../data/scanners1');
all = scanners.plane_all;
ts0 = min(all(:,8));
all(:,8) = (all(:,8) - ts0)/(1000*1000);
all = all(and(0 < all(:,8), all(:,8) < 6), :);
cloud_all = [all(:,1:3), all(:,8)];

%% Dataset2: dataset from multisensor test
% load('../data/cessna_turn');
% da3=s3;da4=s4;
% all = [da3, repmat(3, size(da3,1),1); da4, repmat(4, size(da4,1),1)];
% ts0 = min(all(:,8));
% all(:,8) = (all(:,8) - ts0)/(1000*1000);
% all = all(and(0 < all(:,8), all(:,8) < 6), :);
% cloud_all = [all(:,1:3), all(:,8)];


%% Dataset3: curvylinear motion
% load('../data/scanners4');
% all = scanners.plane_all;
% ts0 = min(all(:,8));
% all(:,8) = (all(:,8) - ts0)/(1000*1000);
% all = all(and(0.3 < all(:,8), all(:,8) < 6), :);
% all(:,8) = (all(:,8) - min(all(:,8)));
% cloud_all = [all(:,1:3), all(:,8)];

%% CV model
results = [];

settings.bin_size       = 1;
settings.verbose        = 0;

settings.sa_vxstart      = 0;
settings.sa_vystart      = 0;
settings.sa_vzstart      = 0; 
settings.sa_axstart      = 0;
settings.sa_aystart      = 0;
settings.sa_azstart      = 0; 

settings.sa_vxbaund      = 50;
settings.sa_vybaund      = 50;
settings.sa_axbaund      = 35;
settings.sa_aybaund      = 35;

%%
settings.verbose          = 0;
settings.bin_size         = 1;
settings.n_neighb         = 500;

settings.model            = 'CA2DoF';
step_size = 0.1;
window_size = 2;

% settings.model            = 'CV2DoF';
% step_size = 0.1;
% window_size = 1;

ts = cloud_all(:,end);
fig_i = 1;
params = [];
total_recon = [];

figure(1); clf;
for i = min(ts) : step_size : max(ts),
        
%     if ((i+window_size) > max(ts))
%         break;
%     end;
    
    fprintf('Timestamp: %.2f\n', i);
    
    model_cloud = cloud_all(and(i <= cloud_all(:,end), cloud_all(:,end) < i+window_size ), :);    
    ts0 = median(model_cloud(:,end));
    %model_cloud(:,end) = model_cloud(:, end) - min(model_cloud(:, end));
    fprintf('Cloud size: %i\n', size(model_cloud, 1));
    
    if (size(model_cloud,1) < 300)
        fprintf('Too small point cloud! Skip...\n')
        continue;
    end;
        
    if (size(params, 1) > 0)
        settings.model_cloud =  total_recon;
    end;
    
    [paramsa, bins, cloud_res, iter] = mexEntropyMinimization(model_cloud, settings);
    %[paramsa, bins, cloud_res] = paralelel_reconstruction(model_cloud, 50, 10, settings);
    
    if (length(paramsa) == 4),
        params = [params; ts0, paramsa, bins];
    end;
    
    if (length(paramsa) == 2),
        params = [params; ts0, paramsa, 0 0, bins];
    end;
    
    %total_recon = [total_recon; cloud_res];
    if (size(params, 1) > 0)
        cloud_procssed = cloud_all(ts<i,:);
        
        params_p = [0, params(1,2:end); params; 100, params(end,2:end)];
        ax = @(x) interp1(params_p(:,1), params_p(:,4), x);
        ay = @(x) interp1(params_p(:,1), params_p(:,5), x);
        vx = @(x) interp1(params_p(:,1), params_p(:,2), x) + integral(ax, 0, x);
        vy = @(x) interp1(params_p(:,1), params_p(:,3), x) + integral(ay, 0, x);
        tss = 0 : 0.01 : max(ts);
        vtxyz = [];
        
        for j = 1 : length(tss),
            vtxyz = [vtxyz; tss(j), vx(tss(j)), vy(tss(j)), 0];
        end;

        settings2.bin_size       = 100;      
        settings2.verbose        = 0;        
        settings2.model          = 'general';
        [~, total_recon] = mexReconstruct(cloud_procssed, params , settings2);        
    end;
        
    fprintf('Parameters: \n'), params(:,1:6)

    
    %% Check the solution
    figure(1); hold on; fig_i = fig_i + 1;
    
    %plot3(cloud_all(:,1), cloud_all(:,2), cloud_all(:,3), 'c.');
    plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'b.');
    plot3(total_recon(:,1), total_recon(:,2), total_recon(:,3), 'g.');
    grid on;
    axis equal;
    pause(0.5);
end;

%% Velocity and acceleration profiles
cloud_procssed = cloud_all(and(min(params(:,1)) < ts, ts < max(params(:,1))),:);
params = [0, params(1,2:end); params; 100, params(end,2:end)];
ax = @(x) interp1(params(:,1), params(:,4), x);
ay = @(x) interp1(params(:,1), params(:,5), x);
vx = @(x) interp1(params(:,1), params(:,2), x) + integral(ax, 0, x);
vy = @(x) interp1(params(:,1), params(:,3), x) + integral(ay, 0, x);
tss = 0 : 0.01 : max(ts);
vtxyz = [];
for i = 1 : length(tss),
    vtxyz = [vtxyz; tss(i), vx(tss(i)), vy(tss(i)), 0];
end;

figure(3); clf; hold on;
subplot(2,2,1); hold on;
plot(tss, ax(tss), 'b.-');

subplot(2,2,2); hold on;
plot(tss, ay(tss), 'b.-');

subplot(2,2,3); hold on;
plot(tss, interp1(params(:,1), params(:,2), tss), 'g.-');
plot(tss, vtxyz(:,2), 'r.-');

subplot(2,2,4); hold on;
plot(tss, interp1(params(:,1), params(:,3), tss), 'g.-');
plot(tss, vtxyz(:,3), 'r.-');

%%
settings.bin_size       = 100;      
settings.verbose        = 1;        
settings.model          = 'general';
[nbins, reconstruction] = mexReconstruct(cloud_procssed, vtxyz , settings);

%% Display the results
figure(2); clf; hold on;
plot3(reconstruction(:,1), reconstruction(:,2), reconstruction(:,3), 'r.'); 
plot3(cloud_procssed(:,1), cloud_procssed(:,2), cloud_procssed(:,3), 'b.');
grid on;
axis equal;






