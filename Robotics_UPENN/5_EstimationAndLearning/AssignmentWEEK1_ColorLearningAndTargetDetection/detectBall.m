% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 

load("a.mat", 'ball_mean', 'ball_cov');
mu = ball_mean;
sig = ball_cov;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 


X = double(reshape(I, [], 3));

X_1 = -0.5*(X - mu)*inv(sig)*(X - mu).';

y = (2*pi)^-1.5*det(sig)^0.5*exp(diag(X_1));

threshold = 1.0;
%sample_ind = find(y > threshold);
y = reshape(y, [120, 160, 1]);

mask = y > threshold;

%segI = zeros(120,160);
%loc = [100,100];
%segI(sample_ind) = 255;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

% https://kr.mathworks.com/help/images/ref/bwconncomp.html
% https://kr.mathworks.com/help/images/ref/regionprops.html

% create new empty binary image
bw_biggest = false(size(mask));

% http://www.mathworks.com/help/images/ref/bwconncomp.html
CC = bwconncomp(mask);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 

% show the centroid
% http://www.mathworks.com/help/images/ref/regionprops.html
S = regionprops(CC,'Centroid');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

segI = bw_biggest;

loc = S(idx).Centroid;


end
