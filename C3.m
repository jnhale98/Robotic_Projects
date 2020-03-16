% Input: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
%        q_goal -> 2x1 vector denoting the goal configuration
% Output: distances -> NxN matrix containing the distance transform from
%                      the goal configuration
%                      == 0 if cell is unreachable
%                      == 1 if cell is an obstacle
%                      == 2 if cell is the goal
%                      >  2 otherwise

function distances = C3(cspace, q_grid, q_goal)
    % Note, don't need to worry about unreachable configurations
    % because they are already 0
    
    % Copies the cspace to a distance NxN array
    distances = zeros(length(q_grid)); % Preallocates distances array
    for i=1:length(q_grid)
        for j=1:length(q_grid)
            distances(i,j)=cspace(i,j);
        end
    end
    
    % Uses check function to find the poision on the q_grid that is
    % closest to the goal configuration
    [q1_loc, q2_loc] = check(q_grid, q_goal);
    
    % Define the goal configuration location as a 2 in cspace
    distances(q1_loc,q2_loc) = 2;
    i=2;
    % The largest possible number for a configuration would be the number
    % of points in the cspace, so set that as the limit for our while loop
    while i <= length(distances)^2
        % Find the indicies for this respective number in cspace, starting
        % at 2 and increasing
        [q1,q2] = find(distances==i);
        % If that number doesn't exist, then we have defined the entire
        % cspace for reachable configurations
        if isempty([q1,q2])
            break
        end
        
        % Loops through the number of times that the current number located
        % is found in the distances matrix
        for j = 1:length(q1)
            % Set q1 and q2 coordinates to the respective indecies in the
            % find vector
            loc=[q1(j),q2(j)];
            % Use the adjacent function to find a 3x3 matrix around the
            % designated point in cspace, and turn any values that are 0
            % around in to 1 greater than the respective number. Insert
            % these new values into the distances array
            propagation = adjacent(distances, loc);
            for k = -1:1
                for w = -1:1
                    if propagation(k+2,w+2)==0
                        distances(q1(j)+k,q2(j)+w)=i+1;
                    end
                end
            end
        end
        i=i+1;
    end
    
end