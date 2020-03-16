% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    
    % Defines the q configuration space to be constant increments about the
    % max and minimum q angles for each joint
    tick = linspace(0,1,num_samples)';
    min = repmat(q_min, num_samples,1); % Defines the minimum q value and extends it to the total number of samples in a matrix
    dif = repmat(q_max-q_min, num_samples, 1); % Defines the difference between the maximum and minimum angles and extends it to the total number of samples in a matrix
    disc = repmat(tick,1,4); % Defines the descritization of the 0-1 range and expands it to fit the total number of joints and samples
    qs = min+dif.*disc; % Increments each q angle from it's min to max with constant increments for the number of defined samples
    

end