function angles = get_rotation_angles( rot )
% Get rotation angles from rotation matrix
% Input: 3-by-3 3D rotation matrix
% Output: 3-by-1 angles 

     angles = zeros(3,1);
     angles(2) = asin(rot(1,3));					  
     C =  cos( angles(1) );
     
     if abs( C ) > 0.005,
     
         tr_x      =  rot(3,3) / C;          
         tr_y      = -rot(2,3)  / C;
         angles(1) = atan2( tr_y, tr_x );
         tr_x      =  rot(1,1) / C;				  
         tr_y      = -rot(1,2)/ C;
         angles(3) = atan2( tr_y, tr_x );
    else					                                 
          angles(1) =   0;				                     
          tr_x      =   rot(2,2);                 
          tr_y      =   rot(2,1);
          angles(3) = atan2( tr_y, tr_x );
     end

    % return only positive angles in [0,2PI] 
%     if angles(1) < 0, 
%         angles(1) = angles(1) + 2*pi;
%     end;
% 
%     if angles(2) < 0, 
%         angles(2) = angles(2) + 2*pi;
%     end;
% 
%     if angles(3) < 0, 
%         angles(3) = angles(3) + 2*pi;
%     end;

end

