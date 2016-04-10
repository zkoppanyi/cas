function  draw_bins( bins, bin_size)

    for i = 1 : size(bins, 1)
      xp = bins(i, 1); yp = bins(i, 2); zp = bins(i, 3);
      corner1 = [xp yp zp];
      corner2 = [xp+bin_size(1) yp+bin_size(2) zp+bin_size(3)];

      % Draw cube
        edges = [corner1(1), corner1(2), corner1(3);
                 corner2(1), corner1(2), corner1(3);
                 corner2(1), corner2(2), corner1(3);
                 corner1(1), corner2(2), corner1(3);
                 corner1(1), corner1(2), corner1(3);
                 corner1(1), corner1(2), corner2(3);
                 corner2(1), corner1(2), corner2(3);
                 corner2(1), corner2(2), corner2(3);
                 corner1(1), corner2(2), corner2(3);
                 corner1(1), corner1(2), corner2(3);
                ];
       plot3(edges(:,1), edges(:,2), edges(:,3), 'k-');
       edges = [corner1(1), corner2(2), corner1(3); corner1(1), corner2(2), corner2(3)];
       plot3(edges(:,1), edges(:,2), edges(:,3), 'k-');
       edges = [corner2(1), corner2(2), corner1(3); corner2(1), corner2(2), corner2(3)];
       plot3(edges(:,1), edges(:,2), edges(:,3), 'k-');
       edges = [corner2(1), corner1(2), corner1(3); corner2(1), corner1(2), corner2(3)];
       plot3(edges(:,1), edges(:,2), edges(:,3), 'k-');
       
    end;

end

