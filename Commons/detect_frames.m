function [profile_vector, frame_num, point_nums] = detect_frames(t, time_separation, verbose, cloud)
    
    if nargin < 1,
            disp('Usage: t                  - time vector [T]');
            disp('       verbose            - 1, if verbose');
            disp('       time_separation    - the time difference betewen the frames');
            error('Not enough input arguments');
    end;
    
    if nargin < 3,
        verbose = 1;
    end;
       
    [t, idx] = sort(t);

    dt = diff(t);
    dtwz = dt(dt~=0);

    if verbose,
        figure(1); clf; hold on;
        plot(1:length(t), t,  'b.-');
        xlabel('Data index [t]'); ylabel('Time [t]');        

        figure(2); clf; hold on;            
        hist(dtwz);

        fprintf('Min time difference: %.3f\n', min(dtwz));
        fprintf('Mean time difference: %.3f\n', mean(dtwz));
        fprintf('STD time difference: %.3f\n', std(dtwz));            
        fprintf('Median time difference: %.3f\n', median(dtwz));            
        fprintf('MAD time difference: %.3f\n', mad(dtwz));                     
        disp('Time difference vector:');
        disp(dtwz');
    end;
        
    if nargin < 2,        
        disp('No time separation has been defined!');
        disp('Please specify it based on the above information');    
        error('No time separation has been defined!');
    end;
    
    pidx = find(dt > time_separation);
    frame_num = length(pidx);
    point_nums = ones(frame_num , 1)*-1;
    profile_vector = ones(length(dt), 1)*-1;
    pidx = [0 pidx' length(t)];
    for i = 2 : length(pidx)
        profile_vector(idx((pidx(i-1)+1):(pidx(i)))) = i-1;
        point_nums(i-1) = length((pidx(i-1)+1):(pidx(i)));
    end; 
    
    %profile_vector = sort_back(profile_vector', idx');
    
    if verbose,
         figure(1); hold on;
         for i = 1 : frame_num,
            ppidx = find(profile_vector == i);
            plot(min(ppidx), min(t(ppidx)), 'ro');
         end;
         legend('Time series', 'The start point on the dataset');
        %plot(pidx, t(pidx), 'ro');
        
        % if the cloud is also given, show the different frames        
        if nargin == 4,
            cols = repmat('rgbcymk', 1, frame_num);
            figure(3); clf; hold on;
            for i = 1 : frame_num,
                ppidx = find(profile_vector == i);
                plot3(cloud(ppidx,1), cloud(ppidx,2), cloud(ppidx,3), [cols(i), '.']);
            end;
            axis equal;
        end;
    end;    
    
    
    