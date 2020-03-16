
function [q1_loc, q2_loc] = check(q_grid, q)
    % This function takes q1 and q2 values as input and outputs the
    % indicies in the q_grid that are closest to these values


    % Preallocate the size of the arrays to find the closest value for both
    % q1 and q2 in the q_grid
    q1_check = zeros(1,length(q_grid));
    q2_check = zeros(1,length(q_grid));
    
    % Finds the magnitude of the difference between each q value in the
    % q_grid and the designated q1 and q2 values
    for i=1:length(q_grid)
        q1_check(i) = abs(q(1)-q_grid(i));
        q2_check(i) = abs(q(2)-q_grid(i));
    end
 
    % Outputs the index location for the configuration in the q_grid that
    % is closest to the desired location
    [~, q1_loc] = min(q1_check);
    [~, q2_loc] = min(q2_check);

end
