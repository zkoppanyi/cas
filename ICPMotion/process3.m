%**************************************************************************
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************


%% Settings
clear all; clc; close all;

addpath('../Commons/');
addpath('../VM/');

is_plot = false;

%% Dataset2: dataset from simulation
% load('../data/pc_cessna_ss_cv_v10');
% [val idx] = min(data(:,1));
% %data(:,6:8) = data(:,6:8) - repmat(mean(data(:,6:8)), size(data,1), 1);
% ts = data(:,1)-min(data(:,1));
% %ts = data(:,1);
% model_cloud = [data(:,6:8), ts];
% gaps_threshold = 0.05

%% Dataset3: curvylinear motion
load('../data/scanners4');
all = scanners.plane_all;
ts0 = min(all(:,8));
all(:,8) = (all(:,8) - ts0)/(1000*1000);
all = all(and(1 < all(:,8), all(:,8) < 6), :);
all(:,8) = (all(:,8) - min(all(:,8)));
model_cloud = [all(:,1:3), all(:,8)];
[~, idx] = sort(model_cloud(:,4));
model_cloud = model_cloud(idx,:);
gaps_threshold = 0.05;

%% Dataset1: dataset from multisensor test
% load('../data/scanners1');
% all = scanners.plane_all;
% ts0 = min(all(:,8));
% all(:,8) = (all(:,8) - ts0)/(1000*1000);
% all = all(and(0 < all(:,8), all(:,8) < 6), :);
% model_cloud = [all(:,1:3), all(:,8)];
% gaps_threshold = 0.05

%% Separate frames
newmc = [];
gaps=find(diff(model_cloud(:,4)) > gaps_threshold);
gaps=[1;gaps];
for i = 2 : length(gaps),
    idx = gaps(i-1):gaps(i);
    newmc = [newmc; model_cloud(idx,:), repmat(i-1, length(idx), 1)];
end;
model_cloud = newmc;
frame_num = length(gaps)-1;

%% Approximated solution with Entropy minimization
    results = [];

    settings.bin_size       = 2;
    settings.verbose        = 1;

    settings.sa_vxstart      = 0;
    settings.sa_vystart      = 0;
    settings.sa_vzstart      = 0; 
    settings.sa_axstart      = 0;
    settings.sa_aystart      = 0;
    settings.sa_azstart      = 0; 

    settings.sa_vxbaund      = 100;
    settings.sa_vybaund      = 100;
    settings.sa_vzbaund      = 100;
    settings.sa_axbaund      = 100;
    settings.sa_aybaund      = 100;
    settings.sa_azbaund      = 100;
    settings.n_neighb         = 100;
    settings.model           = 'CA2DoF';
 
[params, bins1, cloud_res1, iter1] = mexEntropyMinimization(model_cloud, settings);

start_point = mean(cloud_res1, 1);
start_point(2) = start_point(2);
[~, trajectory_em] = mexReconstruct([ repmat(start_point(1:3), size(model_cloud,1), 1) model_cloud(:,4)], -params, settings);
bins_size = 0.3;
bins = mexCalcBins(cloud_res1, bins_size, 1); 

% params1 = [-8 0 0 0];
% settings.model          = 'CV2DoF';
% settings.bin_size       = 100;
% settings.verbose        = 1;
% [~, cloud_res1] = mexReconstruct(model_cloud, params1, settings);



%% Check the solution

if is_plot,
    figure(1); clf; hold on;
    cols = repmat('rgbyckrgbyckrgbyckrgbyck',1,10);
    for i = 1 : frame_num,
        idx = find(model_cloud(:,end) == i);
        plot3(cloud_res1(idx,1), cloud_res1(idx,2), cloud_res1(idx,3), [cols(i) '.']);
    end;

    for i = 1 : frame_num,
        idx = find(model_cloud(:,end) == i);
        plot3(model_cloud(idx,1), model_cloud(idx,2), model_cloud(idx,3), [cols(i) '.']);
    end;   

    draw_bins( bins, [bins_size, bins_size, bins_size]);
    legend('All points', 'CV model');
    xlabel('X [m]'); ylabel('Y [m]');
    set(gca, 'FontSize', 12);
    grid on;
    axis equal;
end;

%% Calcualte cube trajectories
reconstructed_cloud = cloud_res1;
A=[];lx=[];ly=[];w=[];
for niter = 1:1,
    
    fprintf('Iteration #: %i', niter);
    bins = mexCalcBins(reconstructed_cloud, bins_size, 1); 
    
    if is_plot,
        figure(3+niter); clf; hold on;
        for i = 1 : frame_num,
            idx = find(model_cloud(:,end) == i);
            plot3(reconstructed_cloud(idx,1), reconstructed_cloud(idx,2), reconstructed_cloud(idx,3), [cols(i) '.']);
        end;
        for i = 1 : frame_num,
            idx = find(model_cloud(:,end) == i);
            plot3(model_cloud(idx,1), model_cloud(idx,2), model_cloud(idx,3), [cols(i) '.']);
        end;

        draw_bins( bins, [bins_size, bins_size, bins_size]);
        legend('All points', 'CV model');
        xlabel('X [m]'); ylabel('Y [m]');
        set(gca, 'FontSize', 12);
        grid on;
        axis equal;
    end;

    
    trajs = [];
    for i = 1 : size(bins, 1),
        xlb = bins(i, 1);
        xub = bins(i, 1)+bins_size;
        ylb = bins(i, 2);
        yub = bins(i, 2)+bins_size;
        zlb = bins(i, 3);
        zub = bins(i, 3)+bins_size;
        cubetraj = [];
        for j = 1 : frame_num,
            rpts = reconstructed_cloud(reconstructed_cloud(:,end) == j, :);            
            pts = model_cloud(model_cloud(:,end) == j, :);
            
            idx = find(and(and(and(rpts(:,1) > xlb, rpts(:,1) < xub),  ...
                           and(rpts(:,2) > ylb, rpts(:,2) < yub)),  ...
                           and(rpts(:,3) > zlb, rpts(:,3) < zub))); 
                       
            if length(idx) > 0,
                cubetraj = [cubetraj, mean(pts(idx,1:4), 1), length(idx)];
            else
                cubetraj = [cubetraj, NaN, NaN, NaN, NaN, NaN];        
            end;
            
        end;
        trajs = [trajs; cubetraj];
    end;

    %% Display cube trajectories
    if is_plot,
        hold on;
        for i = 1 : size(trajs, 1),
            pts = reshape(trajs(i,:),5, size(trajs, 2)/5)';
            idx = find(~isnan(pts(:,4)));
            if length(idx) > 6,
                plot3(pts(idx,1), pts(idx,2), pts(idx,3), 'k.-');    
            end;
        end;
    end;

    %% Display coordinates 
    tij = 0; tij2 = 0; dijx = 0; dijy = 0;
    dijxtij = 0; dijytij = 0; 
    
    ttr = []; xtr = []; ytr = [];
    
    % Frame weights
    wf = zeros(size(trajs, 2)/5, 1);
    for i = 5 : 5 : size(trajs, 2),
      wf(i/5) = sum(~isnan(trajs(:,i)));
    end;
    
    figure(2); clf; hold on;
    for i = 1 : size(trajs, 1),
        pts = reshape(trajs(i,:),5, size(trajs, 2)/5)';
       
        % Method 1      
        idx = find(~isnan(pts(:,4)));
        if length(idx) > 2,        
            
            dxyz = diff(pts(idx,1:3))./repmat(diff(pts(idx,4)), 1, 3);
            t = (pts(idx(2:end),4) + pts(idx(1:length(idx)-1),4))/2;
            mdt = max(diff(t));

            idx2 = find(and(and(~isnan(dxyz(:,1)), abs(dxyz(:,1)) < params(1).*mdt*3), abs(dxyz(:,2)) < params(2).*mdt*3));
            %idx2 = find(~isnan(dxyz(:,1)));
            dxyz = dxyz(idx2,:);
            t = t(idx2);
           
            %plot(t, dxyz(:,1), 'b.');        
            %plot(t, dxyz(:,2), 'r.');  
            
            A = [A;  repmat(1, size(dxyz,1), 1), t];
            lx = [lx;dxyz(:,1)];
            ly = [ly;dxyz(:,2)];
            wt = wf(idx);
            w = [w; (wt(1:(length(wt)-1))+wt(2:(length(wt))))./2];
            
%             A = [A; repmat(1, length(idx), 1), pts(idx,4), pts(idx,4).^2/2];
%             %A = [A; pts(idx,4), pts(idx,4).^2/2];
%             lx = [lx;pts(idx,1)];
%             ly = [ly;pts(idx,2)];            
%             w = [w; wf(idx)];
        end;
        
        % Method 2
        idx = find(~isnan(pts(:,4)));
        if length(idx) > 2,

            %tij = pts(idx,4);    
            %tij = (tij(1:(length(tij)-1))+tij(2:(length(tij))))./2;        

            pts = pts(idx,:);
            pdijx = diff(pts(:,1)) ./ diff(pts(:,4));
            pdijy = diff(pts(:,2)) ./ diff(pts(:,4));
            ptij2 = (pts(2:end,4) + pts(1:length(idx)-1,4))/2;    
            mdt = max(diff(ptij2));
            
            idx2 = find(and(abs(pdijx) < params(1)*mdt*3, abs(pdijy) < params(2)*mdt*3));
            
            ptij = ptij2(idx2);
            pdijx = pdijx(idx2);
            pdijy = pdijy(idx2);
            
            plot(ptij, pdijx, 'b.');        
            plot(ptij, pdijy, 'r.'); 
            
            ttr = [ttr; ptij]; 
            xtr = [xtr; pdijx]; 
            ytr = [ytr; pdijy]; 
                        
            tij = tij + sum(ptij);
            tij2 = tij2 + sum(ptij.^2);

            dijx = dijx + sum(pdijx);
            dijxtij= dijxtij + sum(pdijx.*ptij);

            dijy = dijy + sum(pdijy);
            dijytij = dijytij + sum(pdijy.*ptij);
        end;
        
    end;

    % Method 1
    %W = diag(w./max(w));
    W = eye(size(A,1));
    avx = (A'*W*A)\(A'*W*lx)
    avy = (A'*W*A)\(A'*W*ly)
    avx= -avx; avy = -avy;
    
    % Method 2
    vx = (dijx*tij2 - dijxtij*tij) / (tij2 - tij.^2);
    ax = (dijxtij - dijx*tij) / (tij2 - tij.^2);
    vy = (dijy*tij2 - dijytij*tij) / (tij2 - tij.^2);
    ay = (dijytij - dijy*tij) / (tij2 - tij.^2);
    
    ax = (sum((xtr-mean(xtr)).*(ttr-mean(ttr)))/(length(ttr)-1)) / (sum((ttr-mean(ttr)).^2)/(length(ttr)-1));
    vx = mean(xtr) - mean(ttr)*ax;
    ay = (sum((ytr-mean(ytr)).*(ttr-mean(ttr)))/(length(ttr)-1)) / (sum((ttr-mean(ttr)).^2)/(length(ttr)-1));
    vy = mean(ytr) - mean(ttr)*ay;
        
    %vx = 10; vy = 0; ax = 10; ay = -2;
    %avx = [0 10 0]; avy = [0 0 0]; 
    
    t = model_cloud(:,4);
    
    figure(2); hold on;
    plot(t, vx + t*ax, 'b-');
    plot(t, vy + t*ay, 'r-');
    plot(t, -avx(1) - t*avx(2), 'b--');
    plot(t, -avy(1) - t*avy(2), 'r--');
    
    %reconstructed_cloud(:,1) = model_cloud(:,1) - t.*vx - t.^2*ax/2;
    %reconstructed_cloud(:,2) = model_cloud(:,2) - t.*vy - t.^2*ay/2;
    %reconstructed_cloud(:,1) = model_cloud(:,1) - avx(1) - t.*avx(2) - t.^2*avx(3)/2;
    %reconstructed_cloud(:,2) = model_cloud(:,2) - avy(1) - t.*avy(2) - t.^2*avy(3)/2;
    %reconstructed_cloud(:,3) = model_cloud(:,3);
    
    settings.model           = 'CA2DoF';
    settings.bin_size       = 100;
    settings.verbose        = 1;
    start_point = mean(reconstructed_cloud, 1);
    start_point(2) = start_point(2);
    [~, trajectory_mot] = mexReconstruct([ repmat(start_point(1:3), size(model_cloud,1), 1) model_cloud(:,4)], -[avx(1) avy(1) avx(2) avy(2)], settings);

    settings.model          = 'CA2DoFWithCenter';
    settings.bin_size       = 2;
    settings.verbose        = 1;
    [ent_icp, reconstructed_cloud] = mexReconstruct(model_cloud, [avx(1) avy(1) avx(2) avy(2) start_point(1:2)], settings);
    
    %%
    figure(1); clf; hold on;
    plot3(cloud_res1(:,1), cloud_res1(:,2), cloud_res1(:,3), 'g.');
    plot3(reconstructed_cloud(:,1), reconstructed_cloud(:,2), reconstructed_cloud(:,3), 'b.');
    plot3(trajectory_em(:,1), trajectory_em(:,2), trajectory_em(:,3), 'g.-');
    plot3(trajectory_mot(:,1), trajectory_mot(:,2), trajectory_mot(:,3), 'b.-');
    plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'k.');
    legend('Solution from EM', 'Solution from ICPMotion')
    axis equal;
    grid on;
    
    pause(1)
end;











