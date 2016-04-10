%**************************************************************************
% Purpose: Demonstrating different velocities give different point clouds
% and entropies
%
% Written by Zoltan Koppanyi, OSU
% E-mail: zoltan.koppanyi@gmail.com
%**************************************************************************

addpath('../../Commons/');
addpath('../../VM/');

%% Settings
clear all; clc;
bin_size = 0.5;

%% Load
load('../../data/velodyne');
profiles = cloud_model.profiles;
model_cloud = [];
 for i = 1 : length(profiles),
        prof = profiles{i};
        model_cloud = [model_cloud; prof];
 end;
 model_cloud(:,4)= model_cloud(:,4) - min(model_cloud(:,4));
 
for i = 0:0.5:10.5,
    %Reconstruct
    settings.model          = 'CV2DoF';
    settings.bin_size       = bin_size;
    settings.verbose        = 1;
    [entropy, result_cloud] = mexReconstruct(model_cloud, [i 0.1 0], settings);

    % Getting back the occupencies
    sbins = mexCalcBins(result_cloud, bin_size, 1, 0); 

    % Solution 1
%     figure(1); clf; hold on;
%     set(gcf,'color','w');
%     plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'b.');
%     plot3(result_cloud(:,1), result_cloud(:,2), result_cloud(:,3), 'r.');
%     title(['v_x = ', num2str(i), ' m/s']);
%     set(gca, 'FontSize', 18);
%     xlabel('X [m]', 'FontSize', 14);
%     ylabel('Y [m]', 'FontSize', 14);
%     axis equal;
%     legend('Raw data', 'Reconstruction')
%     xlim([-20 15]); ylim([20 40]);
%     grid on;

    % Solution 2
    figure(1); clf; hold on;
    set(gcf,'color','w');
    plot3(model_cloud(:,1), model_cloud(:,2), model_cloud(:,3), 'b.');
    plot3(result_cloud(:,1), result_cloud(:,2), result_cloud(:,3), 'r.');
    draw_bins( sbins, [bin_size, bin_size, bin_size]);
    title(['v_x = ' sprintf('%.1f', i), ' m/s Bin #: ' num2str(size(sbins,1)) sprintf(' Entropy: %.3f', entropy)]);
    set(gca, 'FontSize', 18);
    xlabel('X [m]', 'FontSize', 14);
    ylabel('Y [m]', 'FontSize', 14);
    axis equal;
    legend('Raw data', 'Reconstruction')
    xlim([-20 15]); ylim([20 40]);
    grid on;
   
        
    frame = getframe(1);
    im = frame2im(frame);
	[imind,cm] = rgb2ind(im,256);
    if i == 0;
          imwrite(imind,cm,'reconstruction2.gif','gif', 'Loopcount',inf, 'DelayTime',0.3);
    else
          imwrite(imind,cm,'reconstruction2.gif','gif','WriteMode','append', 'DelayTime',0.3);
    end
    pause(0.3);

end;
imwrite(imind,cm,'reconstruction2.gif','gif','WriteMode','append', 'DelayTime',2);


