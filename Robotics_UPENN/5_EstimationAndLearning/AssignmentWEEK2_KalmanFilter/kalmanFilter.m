function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    Sigma_m = 1e-1*eye(4);
    Sigma_o = 1e-2*eye(2);
    C = [ 1.0, 0.0, 0.0, 0.0;
          0.0, 1.0, 0.0, 0.0];
    

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    %vx = (x - state(1)) / (t - previous_t);
    %vy = (y - state(2)) / (t - previous_t);
    dt = t - previous_t;
    A = [  1.0, 0.0, dt, 0.0;
           0.0, 1.0, 0.0 dt;
           0.0, 0.0, 1.0, 0.0;
           0.0, 0.0, 0.0, 1.0];
    
    P = A*param.P*transpose(A) + Sigma_m;
    R = Sigma_o;
    K = P*transpose(C)*inv(R + C*P*transpose(C));
    
    z = [x;y];
    % update
    param.P = (eye(4) - K*C)*param.P;
    
    % predict
    
    state_vector = transpose(state);
    state_vector = A*state_vector + K*(z - C*A*state_vector);
    
    % Predict 330ms into the future
    % predictx = x + vx * 0.330;
    % predicty = y + vy * 0.330;    
    predict_A = [   1.0, 0.0, 0.330, 0.0;
                    0.0, 1.0, 0.0 0.330;
                    0.0, 0.0, 1.0, 0.0;
                    0.0, 0.0, 0.0, 1.0];
    
    predict = predict_A*state_vector;

    predictx = predict(1,1);
    predicty = predict(2,1);
    % State is a four dimensional element
    % state = [x, y, vx, vy];
    state = transpose(state_vector);
end
