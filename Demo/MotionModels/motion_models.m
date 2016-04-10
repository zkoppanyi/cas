clear all; clc;

cvx = @(t) t;
cvy = @(t) t;
cax = @(t) t;
cay = @(t) t - t.^2/2*0.05;
gax = @(t) t;
gay = @(t) t + sin(t);

load('plane_demo');
cent = mean(plane_demo, 1);
plane_demo(:,1:3) = plane_demo(:,1:3) - repmat(cent(:,1:3), size(plane_demo,1),1);

t = 0:0.2:20;
for i = 2 : length(t)
    figure(1); clf; hold on;
    set(gcf,'color','w');
    
    subplot(1,3,1); hold on;
    plot(cvx(t), cvy(t), 'k-', 'LineWidth', 2);
    alpha = atan2(cvy(t(i)), cvx(t(i))) - pi/2;
    R = [cos(alpha), -sin(alpha); sin(alpha) cos(alpha)];
    plane_demo_trans = plane_demo(:,1:2)*R + repmat([cvx(t(i)), cvy(t(i))-0.8], size(plane_demo,1), 1);
    plot(plane_demo_trans(:,1), plane_demo_trans(:,2), 'k.');
    axis equal; grid on;
    xlabel('[m]', 'FontSize', 12); ylabel('[m]', 'FontSize', 12);
    title('Constant Velocity (CV)');
    set(gca, 'FontSize', 15);
    xlim([-5 25]); ylim([-5 25]);
    
    subplot(1,3,2); hold on;
    plot(cax(t), cay(t), 'k-', 'LineWidth', 2);
    alpha = -atan2(cay(t(i)) - cay(t(i-1)),cax(t(i)) - cax(t(i-1)));
     R = [cos(alpha), -sin(alpha); sin(alpha) cos(alpha)];
    plane_demo_trans = plane_demo(:,1:2)*R + repmat([cax(t(i)), cay(t(i))-0.8], size(plane_demo,1), 1);
    plot(plane_demo_trans(:,1), plane_demo_trans(:,2), 'k.');
    axis equal; grid on;
    xlabel('[m]', 'FontSize', 12); ylabel('[m]', 'FontSize', 12);
    title('Constant Acceleration (CA)');
    set(gca, 'FontSize', 15);
    xlim([-5 25]); ylim([-5 25]);

    subplot(1,3,3); hold on;
    plot(gax(t), gay(t), 'k-', 'LineWidth', 2);
    alpha = -atan2(gay(t(i)) - gay(t(i-1)),gax(t(i)) - gax(t(i-1)));
     R = [cos(alpha), -sin(alpha); sin(alpha) cos(alpha)];
    plane_demo_trans = plane_demo(:,1:2)*R + repmat([gax(t(i)), gay(t(i))-0.8], size(plane_demo,1), 1);
    plot(plane_demo_trans(:,1), plane_demo_trans(:,2), 'k.');
    axis equal; grid on;
    xlabel('[m]', 'FontSize', 12); ylabel('[m]', 'FontSize', 12);
    title('General Motion (G)');
    set(gca, 'FontSize', 15);
    xlim([-5 25]); ylim([-5 25]);

    frame = getframe(1);
    im = frame2im(frame);
	[imind,cm] = rgb2ind(im,256);
    if i == 2;
          imwrite(imind,cm,'motion_models.gif','gif', 'Loopcount',inf, 'DelayTime',0.01);
    else
          imwrite(imind,cm,'motion_models.gif','gif','WriteMode','append', 'DelayTime',0.01);
    end
    pause(0.01);
end;



