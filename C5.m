% Input: q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
%        q_goal -> 2x1 vector denoting the goal configuration
%        path -> Mx2 matrix containing a collision-free path from q_start
%                to q_goal (as computed in C3, embedded in distances).
%                The entries of path are grid cell indices, i.e., integers
%                between 1 and N. The first row is the grid cell containing
%                q_start, the final row is the grid cell containing q_goal.
% Output: q_path -> Mx2 matrix containing a collision-free path from
%                   q_start to q_goal. Each row in q_path is a robot
%                   configuration. The first row should be q_start,
%                   the final row should be q_goal.

function q_path = C5(q_grid, q_start, q_goal, path)
    % Pre initialize the q_path matrix to be the length of the path matrix
    % found previously, where each index value will be converted to its
    % corresponding q1 and q2 configuration
    q_path = zeros(length(path), 2);
    
    % Replace the close q configuration found in the path for cspace with
    % the actual starting q configuration
    q_path(1,:) = q_start';
    % For each value in the path matrix place the corresponding q1 and q2
    % configuration into the q_path matrix
    for i = 2:length(path);
        q_path(i,1) = q_grid(path(i,1));
        q_path(i,2) = q_grid(path(i,2));
    end
    % Add the goal configuration to the end of the q_path matrix to finish
    % at the actual goal instead of the closest configuration found in our
    % discretized cspace
    q_path(length(path)+1,:) = q_goal';
    
end