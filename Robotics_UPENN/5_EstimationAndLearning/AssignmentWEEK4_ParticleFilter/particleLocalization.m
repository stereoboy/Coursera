% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

[h, w] = size(map);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResolution = param.resol;
% % the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

sigma_x = 0.05;
sigma_y = 0.05;
sigma_t = 0.1;

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 40;                       % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P = repmat(myPose(:,1), [1, M]);
K = size(scanAngles, 1);
            
weights = zeros(M, 1);
weights(:)=1.0/M;

    for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% 
%     % 1) Propagate the particles

        P_ = zeros(size(P));
        
        x = sigma_x*randn([1, M]);
        y = sigma_y*randn([1, M]);
        t = sigma_t*randn([1, M]);
        P_(1,:) = P(1,:) + x;
        P_(2,:) = P(2,:) + y;
        P_(3,:) = P(3,:) + t;
        
        %x(1:10)
        %y(1:10)
        %t(1:10)*180/3.14

%       

%     % 2) Measurement Update
        scores = zeros(M, 1);
        for i = 1:M % per particle    
            cur_pose_ = P_(1:2,i);
            cur_theta_ = P_(3,i);
            cur_pose = ceil(myResolution*cur_pose_) + myOrigin;
            cur_pose(1) = min(max(1, cur_pose(1)), w);
            cur_pose(2) = min(max(1, cur_pose(2)), h);
            
            for k = 1:K % per scan angle
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
                d = ranges(k, j);
                theta = cur_theta_ + scanAngles(k);
                occu_ = cur_pose_ + [d*cos(theta), -d*sin(theta)]';
                occu = ceil(myResolution*occu_) + myOrigin;
                
                % clip values to fall into map
                occu(1) = min(max(1, occu(1)), w);
                occu(2) = min(max(1, occu(2)), h);
                   
                [freex, freey] = bresenham(cur_pose(1),cur_pose(2),occu(1),occu(2));  
                % convert to 1d index
                free = sub2ind(size(map),freey,freex);
                
%     %   2-2) For each particle, calculate the correlation scores of the particles            
            
                per_scan_score = 1*sum(map(free)<0.4)-5*sum(map(free)>=0.4);
                %per_scan_score
                %-5*(map(occu(2),occu(1))<0) + 10*(map(occu(2),occu(1))>=0)
                per_scan_score = per_scan_score -50*(map(occu(2),occu(1))<0.0) + 30*(map(occu(2),occu(1))>=0.0);
                scores(i) = scores(i) + per_scan_score;
            end

        end
%
%     %   2-3) Update the particle weights         
%  
        j
        [max(scores), min(scores)]
        [max(weights), min(weights)]
        scores(scores<0.0001) = 0.0001;
        weights = weights.*scores;
        weights = weights./sum(weights);
        
%     %   2-4) Choose the best particle to update the pose
        [val, index] = max(weights);
        myPose(:, j) = P_(:, index);
%     
%     % 3) Resample if the effective number of particles is smaller than a threshold
% 
        % https://stackoverflow.com/questions/27092478/matlab-weighted-resampling
        effective_n = 1.0/sum(weights.^2);
        effective_n
        %threshold = 0.85*M;
        threshold = 20;
        if effective_n < threshold
            W = cumsum(weights);
            for i = 1:M
                index = find(rand <= W,1);
                P(:,i) = P_(:,index);
                weights(i) = weights(index);
            end
            %weights = ones(M,1)*1./M;
            weights = weights./sum(weights);
        else
            P = P_;
        end
%     % 4) Visualize the pose on the map as needed
%    
% 
        if j > 120 && mod(j, 30) == 0
            figure;
            imagesc(map); hold on;
            lidar_global(:,1) =  (ranges(:,j).*cos(scanAngles + myPose(3,j)) + myPose(1,j))*param.resol + param.origin(1);
            lidar_global(:,2) = (-ranges(:,j).*sin(scanAngles + myPose(3,j)) + myPose(2,j))*param.resol + param.origin(2);

            plot(lidar_global(:,1), lidar_global(:,2), 'g.'); 

            colormap('gray');
            axis equal;
            hold on;
            plot(myPose(1,1:j)*param.resol+param.origin(1), ...
                myPose(2,1:j)*param.resol+param.origin(2), 'r.-');
        end
    end

end

