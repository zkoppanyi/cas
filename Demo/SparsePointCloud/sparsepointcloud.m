%**************************************************************************
% Purpose: Demonstrating different velocities give different point clouds
% and entropies
%
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************

addpath('../../Commons/');

%% Settings
clear all; clc;
bin_size = 1;

%% Load
load('../../data/velodyne');
profiles = cloud_model.profiles;

for i = 1 : length(profiles),
    prof = profiles{i};
    
    figure(1); clf; hold on;
    set(gcf,'color','w');
    plot(prof(:,1), prof(:,2), 'r.', 'MarkerSize', 10);
    ylim([20 35]);
    xlabel('[m]', 'FontSize', 12); ylabel('[m]', 'FontSize', 12);
    title(sprintf('GPS Time: %.2fs', median(prof(:,4))));
    set(gca, 'FontSize', 15);
    grid on;
    axis equal;
        
    frame = getframe(1);
    im = frame2im(frame);
	[imind,cm] = rgb2ind(im,256);
    if i == 1;
          imwrite(imind,cm,'point_clouds.gif','gif', 'Loopcount',inf, 'DelayTime',0.3);
    else
          imwrite(imind,cm,'point_clouds.gif','gif','WriteMode','append', 'DelayTime',0.3);
    end
    pause(0.3);

end;
imwrite(imind,cm,'point_clouds.gif','gif','WriteMode','append', 'DelayTime',2);


