% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

function [allX,allP]=simpleImuOrientationForwardEkf(gyros,accels,dt,initX,initP)

%--- Dataset parameters
deltat = dt;             % Sampling period
noise_gyro = 2.4e-3;        % Gyroscope noise(discrete), rad/s
noise_accel = 2.83e-2;      % Accelerometer noise, m/s^2
gravity = 9.81007;          % Gravity magnitude, m/s^2

bias_w = [0 0 0];    % gyroscope bias
bias_a = [0 0 0];    % accelerometer bias


%--- Container of the results
N = length(gyros);
allX = zeros(N, 4);
allP=zeros(4,4,N);

%--- Initialization
if ~isempty(initX)
    x=initX(:);
else
    x = [1 0 0 0]';            % Initial state (quaternion)
end
if ~isempty(initP)
    P=initP;
else
    P = 1e-4 * eye(4);         % Initial covariance
end
allX(1,:) = x';
allP(:,:,1)=P;

for k = 2 : N
    
    %--- 1. Propagation --------------------------
    % Gyroscope measurements
    w = gyros(k,:) - bias_w;
    
    % Compute the F matrix
    Omega =[0     -w(1)   -w(2)   -w(3);...
        w(1)  0       w(3)    -w(2); ...
        w(2)  -w(3)   0        w(1); ...
        w(3)  w(2)    -w(1)    0  ];
    F = eye(4) + deltat * Omega / 2;
    
    % Compute the process noise Q
    G = [-x(2)  -x(3)  -x(4); ...
        x(1)  -x(4)   x(3); ...
        x(4)   x(1)  -x(2); ...
        -x(3)   x(2)   x(1)] / 2;
    Q = (noise_gyro * deltat)^2 * (G * G');
    
    % Propagate the state and covariance
    x = F * x;
    x = x / norm(x);    % Normalize the quaternion
    P = F * P * F' + Q;
    
    
    %--- 2. Update----------------------------------
    % Accelerometer measurements
    a = accels(k,:) - bias_a;
    
    % We use the unit vector of acceleration as observation
    ea = a' / norm(a);
    ea_prediction = [2*(x(2)*x(4)-x(1)*x(3)); ...
        2*(x(3)*x(4)+x(1)*x(2)); ...
        x(1)^2-x(2)^2-x(3)^2+x(4)^2];
    
    % Residual
    y = ea - ea_prediction;
    
    % Compute the measurement matrix H
    H = 2*[-x(3)    x(4)    -x(1)   x(2); ...
        x(2)    x(1)     x(4)   x(3); ...
        x(1)   -x(2)    -x(3)   x(4)];
    
    % Measurement noise R
    R_internal = (noise_accel / norm(a))^2 * eye(3);
    R_external = (1-gravity/norm(a))^2 * eye(3);
    R = R_internal + R_external;
    
    % Update
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(4) - K * H) * P;
    
    
    % 3. Ending
    x = x / norm(x);    % Normalize the quaternion
    P = (P + P') / 2;   % Make sure that covariance matrix is symmetric
    allX(k,:) = x';     % Save the result
    allP(:,:,k)=P;
end

end
