% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    e = 2; % allowable error before adding goal configuration to path. Just needed to be preinitialized so it wouldn't prevent the while loop from starting
    alpha = 0.1; % step size
    dof = length(q_max); % DoF for our system
    q_target = zeros(1,dof); % Pre-initialize target configuration to zeros matrix
    nodes = q_start; % Start the nodes array with the starting configuration
    n = 0; % Sets limit for how many nodes to find so it won't run forever
    edge = []; % Preinitialize the edge matrix to add connections between nodes
    
    % Continues to grow trees until it is very close to the goal
    % configuration or it runs too many times
    while e > 0.1 && n<5000
            % Defines the target configuraiton by randomly picking q
            % configurations, for 10% of the tests we will move towards the
            % goal configuration to bias the trees
            for i = 1:dof
                r = rand();
                if r<0.1
                    q_target = q_goal;
                    break
                end
                % Random q angle configurations in the viable space
                q_target(i) = q_min(i)+(q_max(i)-q_min(i))*r;
            end
        % Initialize the distances matrix to be the same size as the number
        % of nodes we currently have 
        distances = zeros(size(nodes, 1),1); % Can't use length because initial nodes had more columns than rows
        
        % Finds the difference between each node already found and the new
        % target node configuration
        for j = 1:size(distances,1)
            distances(j) = norm(q_target-nodes(j,:));
        end
        
        % Find the index of the closest node (the one with the smallest
        % distance)
        [~, minimum_distance] = min(distances);
        % Define the nearest node as the one closest to the target
        % configuration
        q_near = nodes(minimum_distance,:);
        % Move towards the target node from the nearest node by the defined
        % step size and set that location in cspace as the new node to test
        q_new = q_near+alpha*(q_target-q_near)/norm(q_target-q_near);
        
        % Check that the new node is still in the range of joint angles
        for j = 1:dof
            if q_new(j) > q_max(j) || q_new(j) < q_min(j)
               Q = false;
            else
               Q = true;
            end
        end
        
        % If the line between the nearest and new node is not in collision
        % and the new configuration is not in collision add that
        % configuration to the nodes matrix and define the edge as the
        % index of the nearest node, and the current length of the nodes matrix. 
        if check_collision(robot, q_new, link_radius, sphere_centers, sphere_radii)==false & Q
            if check_edge(robot, q_near, q_new, link_radius, sphere_centers, sphere_radii)==false
                nodes = vertcat(nodes,q_new);
                edge = [edge; minimum_distance size(nodes,1)];
                % determine the distance from the new node configuration to
                % the goal configuration
                e = norm(q_goal-q_new);
            end 
        end
        n=n+1;
    end
    
    % Find the node that is closest to the start and end configurations
    nn_start = knnsearch(nodes, q_start);
    nn_end = knnsearch(nodes, q_goal);
    
    % Develop a graph using the edge matrix. We don't need distances
    % because each edge between nodes will be the size of the defined step
    % size. The source will be the nearest node found while the target is
    % the size of the nodes array when we found the corresponding new configuration, because that would
    % correspond to the index of that configuration
    G = graph(edge(:,1),edge(:,2));
    node_path = shortestpath(G,nn_start, nn_end);
    
    % Determine if a path was found or not
    if isempty(node_path)
        path_found = false;
    else
        path_found = true;
    end
       
    
    % Pre-initialize a path array to store the q configurations that
    % correspond to the found nodes
    path = zeros(length(node_path),4);
    
    % Place the corresponding q configurations for each index in the node
    % path from the found nodes to the shortest path
    for i = 1:length(path)
        path(i,:) = nodes(node_path(i),:);
    end
    
    % If the edge between the last node and the end configuration does not
    % collide add the goal configuration as the last configuration in the
    % path
    if check_edge(robot, path(length(path),:),q_goal, link_radius, sphere_centers,sphere_radii)==false
        path = vertcat(path, q_goal);
    end
end