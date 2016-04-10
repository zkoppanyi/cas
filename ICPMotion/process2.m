%**************************************************************************
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************


%% Settings
clear all; clc; close all;

addpath('../Commons/');
addpath('../VM/');

load('../data/pc_cessna_ss_cv_v10');

[val idx] = min(data(:,1));
%data(:,6:8) = data(:,6:8) - repmat(mean(data(:,6:8)), size(data,1), 1);
ts = data(:,1)-min(data(:,1));
%ts = data(:,1);
model_cloud = [data(:,6:8), ts];


%% Separate frames

newmc = [];
gaps=find(diff(model_cloud(:,4)) > 0.05);
gaps=[1;gaps];
for i = 2 : length(gaps),
    idx = gaps(i-1):gaps(i);
    newmc = [newmc; model_cloud(idx,:), repmat(i-1, length(idx), 1)];
end;
model_cloud = newmc;
frame_num = length(gaps)-1;

%% Approximated solution with Entropy minimization
%     results = [];
% 
%     settings.bin_size       = 1.5;
%     settings.verbose        = 1;
% 
%     settings.sa_vxstart      = 0;
%     settings.sa_vystart      = 0;
%     settings.sa_vzstart      = 0; 
%     settings.sa_axstart      = 0;
%     settings.sa_aystart      = 0;
%     settings.sa_azstart      = 0; 
% 
%     settings.sa_vxbaund      = 100;
%     settings.sa_vybaund      = 100;
%     settings.sa_vzbaund      = 100;
%     settings.sa_axbaund      = 100;
%     settings.sa_aybaund      = 100;
%     settings.sa_azbaund      = 100;
%     settings.n_neighb         = 100;
%     settings.model           = 'CV2DoF';
%  
%  bestn = 1e10;
%  best_bins = [];
%  for i = 1 : 1,
%   [params, bins1, cloud_res, iter1] = mexEntropyMinimization(model_cloud, settings);
%   if size(bins1,1) < bestn,
%       bestn = size(bins1,1);
%       params1 = params;
%       cloud_res1 = cloud_res;
%       best_bins = bins1;
%   end;
%  end;

params1 = [-8 0 0 0];
settings.model          = 'CV2DoF';
settings.bin_size       = 100;
settings.verbose        = 1;
[~, cloud_res1] = mexReconstruct(model_cloud, params1, settings);

%% Check the solution
figure(1); clf; hold on;

figure(1); clf; hold on;
cols = 'rgbyckrgbyckrgbyckrgbyck';
for i = 1 : frame_num,
    idx = find(model_cloud(:,end) == i);
    plot3(cloud_res1(idx,1), cloud_res1(idx,2), cloud_res1(idx,3), [cols(i) '.']);
end;


cols = 'rgbyckrgbyckrgbyckrgbyck';
for i = 1 : frame_num,
    idx = find(model_cloud(:,end) == i);
    plot3(model_cloud(idx,1), model_cloud(idx,2), model_cloud(idx,3), [cols(i) '.']);
end;

bins_size = 1;
bins = mexCalcBins(cloud_res1, bins_size, 1); 
%draw_bins( bins, [bins_size, bins_size, bins_size]);

legend('All points', 'CV model');
xlabel('X [m]'); ylabel('Y [m]');
set(gca, 'FontSize', 12);
grid on;
axis equal;

%% Calcualte cube trajectories
reconstructed_cloud = cloud_res1;
A=[];lx=[];ly=[];w=[];
for niter = 1:40,
    
    fprintf('Iteration #: %i', niter);
    bins = mexCalcBins(reconstructed_cloud, bins_size, 1); 
    
%     figure(3+niter); clf; hold on;
%     cols = 'rgbyckrgbyckrgbyckrgbyck';
%     for i = 1 : frame_num,
%         idx = find(model_cloud(:,end) == i);
%         plot3(reconstructed_cloud(idx,1), reconstructed_cloud(idx,2), reconstructed_cloud(idx,3), [cols(i) '.']);
%     end;
%     for i = 1 : frame_num,
%         idx = find(model_cloud(:,end) == i);
%         plot3(model_cloud(idx,1), model_cloud(idx,2), model_cloud(idx,3), [cols(i) '.']);
%     end;

    %draw_bins( bins, [bins_size, bins_size, bins_size]);
%     legend('All points', 'CV model');
%     xlabel('X [m]'); ylabel('Y [m]');
%     set(gca, 'FontSize', 12);
%     grid on;
%     axis equal;

    
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
    hold on;
    for i = 1 : size(trajs, 1),
        pts = reshape(trajs(i,:),5, size(trajs, 2)/5)';
        plot3(pts(:,1), pts(:,2), pts(:,3), 'k.-');    
    end;

    %% Display coordinates 
    tij2 = 0; tij3 = 0; tij4 = 0;
    tijdijx = 0; tij2dijx = 0;
    tijdijy = 0; tij2dijy = 0;
    
    % Frame weights
    wf = zeros(size(trajs, 2)/5, 1);
    for i = 5 : 5 : size(trajs, 2),
      wf(i/5) = sum(~isnan(trajs(:,i)));
    end;
    
    for i = 1 : size(trajs, 1),
        pts = reshape(trajs(i,:),5, size(trajs, 2)/5)';
       
        % Method 1      
        idx = find(~isnan(pts(:,4)));
        if length(idx) > 1,        
            
            dxyz = diff(pts(idx,1:3))./repmat(diff(pts(idx,4)), 1, 3);
            t = (pts(idx(2:end),4) + pts(idx(1:length(idx)-1),4))/2;
            
%             figure(2); clf; hold on;
%             plot(t, dxyz(:,1), 'b.');        
%             plot(t, dxyz(:,2), 'r.');  
            
            A = [A;  repmat(1, size(dxyz,1), 1), t/2];
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
        if length(idx) > 1,

            %tij = pts(idx,4);    
            %tij = (tij(1:(length(tij)-1))+tij(2:(length(tij))))./2;        

            tij = diff(pts(idx,4));    
            tij2 = tij2 + sum(tij.^2);
            tij3 = tij3 + sum(tij.^3);
            tij4 = tij4 + sum(tij.^4);

            dijx = diff(pts(idx,1));
            tijdijx = tijdijx + sum(tij.*dijx);
            tij2dijx = tij2dijx + sum(tij.^2.*dijx);

            dijy = diff(pts(idx,2));
            tijdijy = tijdijy + sum(tij.*dijy);
            tij2dijy = tij2dijy + sum(tij.^2.*dijy);
        end;
        
    end;

    % Method 1
    %W = diag(w./max(w));
    W = eye(size(A,1));
    avx = (A'*W*A)\(A'*W*lx)
    avy = (A'*W*A)\(A'*W*ly)
    avx=[0;avx]; avy=[0;avy];
    
    % Method 2
    vx = (tij4*tijdijx - tij3*tij2dijx) / (tij4*tij2 - tij3^2);
    ax = (tij2*tij2dijx - tijdijx*tij3) / (0.5*tij4*tij2 - 0.5*tij3^2);
    vy = (tij4*tijdijy - tij3*tij2dijy) / (tij4*tij2 - tij3^2);
    ay = (tij2*tij2dijy - tijdijy*tij3) / (0.5*tij4*tij2 - 0.5*tij3^2);
    vxc = tijdijx / tij2;
    vyc = tijdijy / tij2;
    
    %vx = 10; vy = 0; ax = 10; ay = -2;
    %avx = [0 10 0]; avy = [0 0 0]; 
    
    t = model_cloud(:,4);
    
%     figure(2); hold on;
% %     plot(t, t.*vx + t.^2*ax/2, 'b-');
% %     plot(t, t.*vy + t.^2*ay/2, 'r-');
%     plot(t, avx(2) + t*avx(3)/2, 'b--');
%     plot(t, avy(2) + t*avy(3)/2, 'r--');
    
    %reconstructed_cloud(:,1) = model_cloud(:,1) - t.*vx - t.^2*ax/2;
    %reconstructed_cloud(:,2) = model_cloud(:,2) - t.*vy - t.^2*ay/2;
    reconstructed_cloud(:,1) = model_cloud(:,1) - avx(1) - t.*avx(2) - t.^2*avx(3)/2;
    reconstructed_cloud(:,2) = model_cloud(:,2) - avy(1) - t.*avy(2) - t.^2*avy(3)/2;
    reconstructed_cloud(:,3) = model_cloud(:,3);
end;

%%
figure(1); clf; hold on;
plot3(reconstructed_cloud(:,1), reconstructed_cloud(:,2), reconstructed_cloud(:,3), 'b.');
plot3(cloud_res1(:,1), cloud_res1(:,2), cloud_res1(:,3), 'g.');
axis equal;









