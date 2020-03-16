% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    % This code for smoothing utilizes the code written for M3 to determine
    % the shortest path for the random sample. In this case, the tree path
    % is treated as a sample matrix, and each node is rewired with it's closest
    % neighbor, removing redundant edges that don't move towards the goal

    % Pre initialize matricies to store the nodes closest to the start and
    % end configurations

    % For each value in the path, check the respective node for it's 10
    % nearest neighbors. If the edge between these two nodes do not collide
    % with obstacles then add them to the adjcency matrix
    for i = 1:length(path)
        [Idv,a] = knnsearch(path,path(i,:),'K',10);
            for j = 1:length(Idv)
                if check_edge(robot, path(i,:), path(Idv(j),:),link_radius, sphere_centers, sphere_radii)==false
                adjacency(i,Idv(j)) = a(j);
                adjacency(Idv(j),i) = a(j);                    
                end
            end
    end
    
    % Pre initialize source, target, and edge matricies to contruct our
    % graph object
    source = [];
    target = [];
    edge = [];
    
    % For every value in the adjacency matrix, if these is a connection
    % between the two nodes add the nodes as source and target
    % respectively, and add their weight distance to the edge array
    for i = 1:length(path)
        for j = 1:length(path)
            if adjacency(i,j) ~= 0
                source = [source i];
                target = [target j];
                edge = [edge adjacency(i,j)];
            end
        end
    end
   
    % Find the shortest path between all nodes using the newly found node
    % graph
    node_path = shortestpath(graph(source,target, edge), 1, length(path));
    
    % Pre initialize the smoothed path and add the initial configuration to
    % the start
    smoothed_path = zeros(length(node_path), 4);
    smoothed_path(1,:) = path(1,:);
    
    % For every value in the node path found add the corresponding angle
    % configuration to the smoothed path
    for k = 1:length(node_path)
        smoothed_path(k,:) = path(node_path(k),:);
    end
end