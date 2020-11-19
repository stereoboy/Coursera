% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
% myResol = param.resol;
% % the initial map size in pixels
% myMap = zeros(param.size);
% % the origin of the map in pixels
% myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
% lo_occ = param.lo_occ;
% lo_free = param.lo_free; 
% lo_max = param.lo_max;
% lo_min = param.lo_min;

% N = size(pose,2);
% for j = 1:N % for each time,
% 
%       
%     % Find grids hit by the rays (in the gird map coordinate)
%   
% 
%     % Find occupied-measurement cells and free-measurement cells
%    
% 
%     % Update the log-odds
%   
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
% end

% % the number of grids for 1 meter.
    myResol = param.resol;
% % the initial map size in pixels
    myMap = zeros(param.size);
% % the origin of the map in pixels
    myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
    lo_occ = param.lo_occ;
    lo_free = param.lo_free; 
    lo_max = param.lo_max;
    lo_min = param.lo_min;

    N = size(pose,2);
    for j = 1:N % for each time,
% 
%       
%     % Find grids hit by the rays (in the gird map coordinate)
        cur_pose_ = pose(1:2, j);
        cur_theta_ = pose(3, j);
        cur_pose = ceil(myResol*cur_pose_) + myorigin;
        
        M = size(scanAngles, 1);
        for i = 1:M
            d = ranges(i, j);
            theta = cur_theta_ + scanAngles(i);
            occu_ = cur_pose_ + [d*cos(theta), -d*sin(theta)]';
            occu = ceil(myResol*occu_) + myorigin;
            [freex, freey] = bresenham(cur_pose(1),cur_pose(2),occu(1),occu(2));  
            % convert to 1d index
            free = sub2ind(size(myMap),freey,freex);
            % set end point value 
            myMap(occu(2),occu(1)) = myMap(occu(2),occu(1)) + lo_occ;
            % set free cell values
            myMap(free) = myMap(free) - lo_free;
        end
        
%   
% 
%     % Find occupied-measurement cells and free-measurement cells
%    
% 
%     % Update the log-odds
%   
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
    end
    
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;

end

