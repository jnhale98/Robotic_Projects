% Input: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_start -> 2x1 vector denoting the start configuration
% Output: path -> Mx2 matrix containing a collision-free path from q_start
%                 to q_goal (as computed in C3, embedded in distances).
%                 The entries of path should be grid cell indices, i.e.,
%                 integers between 1 and N. The first row should be the
%                 grid cell containing q_start, the final row should be
%                 the grid cell containing q_goal.

function path = C4(distances, q_grid, q_start)
    % First robot position in path is the start position
    
    % Determines the location in the cspace that is closest to the start
    % position
    [q1_loc,q2_loc] = check(q_grid, q_start);
    % Enters the first indecies to the path for the closest points in
    % cspace to the start configuration
    path(1,:) = [q1_loc, q2_loc];
    % Initialize i to be the begining value in the distance matrix for the
    % first point in the path
    i = distances(path(1,1),path(1,2));
    j=2;
    while i>2
        % Find all of the cells adjacent to the one currently being looked
        % at
        near = adjacent(distances,[q1_loc,q2_loc]);
        escape = 0;
        
        % Loop through all adjacent configuration points in cspace. If one
        % is one less than the current value it is closer to the goal
        % configuration, and we can set it as our new configuration
        for k = -1:1
            for w = -1:1
                if near(k+2,w+2)== i-1
                    q1_loc = q1_loc+k;
                    q2_loc = q2_loc+w;
                    escape = 1;
                    break
                end
            end
            % Prevents code from looping through the rest of k if a closer
            % value is found. Prevents accidentally incrementing i by 2
            if escape==1
                break
            end
        end
        
        % Set the new configuration found to be added to the path
        path(j,:) = [q1_loc,q2_loc];
        j=j+1;
        % Set the new i value to be the value found at our new
        % configuration location in the distances matrix
        i=distances(q1_loc, q2_loc);
    end
end