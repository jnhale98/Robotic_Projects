% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
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

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    
    % Initialize empty start and end nodes positions that will be used to
    % find the starting point on our graph
    nn_start = [];
    nn_end = [];
    
    % Use knn to find the node closest to the start position
    Idv = knnsearch(samples,q_start);
        for j = 1:length(Idv)
            % For each node found check the edge to see if it collides with
            % the obstacles, if it doesn't add this node to the starting
            % node matrix
            if check_edge(robot, q_start, samples(Idv(j),:),link_radius, sphere_centers, sphere_radii)==false
                nn_start = [nn_start Idv(j)];
            end
        end
        
    
    % This code preforms the sample task as above for the start condition,
    % finding the closest node sampled to the end condition, and checking
    % that the edge between them doesn't collide with an obstacle before
    % adding it.
    Idv = knnsearch(samples,q_goal);
        for j = 1:length(Idv)
                if check_edge(robot, q_goal, samples(Idv(j),:),link_radius, sphere_centers, sphere_radii)==false
                    nn_end = [nn_end Idv(j)];
                end
        end
    
    % Pre-initialize source, target, and edge matricies for the graph
    % function to be used to find the shortest path
    source = [];
    target = [];
    edge = [];
    
    % If two i, j nodes are connected in the adjacency matrix then add them
    % as source and target nodes, and add their weight distance to the edge
    % matrix. This will provide the vectors we need to define our graph
    for i = 1:length(samples)
        for j = 1:length(samples)
            if adjacency(i,j) ~= 0
                source = [source i];
                target = [target j];
                edge = [edge adjacency(i,j)];
            end
        end
    end
   
    % Develop a graph of the nodes in the samples and find the shortest
    % path from the starting node to the ending node
    node_path = shortestpath(graph(source,target, edge), nn_start, nn_end);
    
    % Tells the program if no path could be found
    if isempty(node_path)
        path_found=false;
        path=path_found;
        return
    end
    
    % Converts our path from cspace nodes to actual q configuratons
    
    % Preinitialize path matrix with added space for actual start and end
    % configurations
    path = zeros(length(node_path)+2, 4);
    % Add the start configuration to the beginning of the path
    path(1,:) = q_start;
    % Add the goal configuraton to the end of the path
    path(length(path),:) = q_goal;
    count = 1;
    for k = 2:length(path)-1
        % Add the samples taken to the path in the order their nodes were
        % listed in the node path found above
        path(k,:) = samples(node_path(count),:);
        count=count+1;
    end
    path_found=true;
end

%%%%%%%%%%%%%%%%%
% Obsolete Code %
%%%%%%%%%%%%%%%%%

% Note, the code below was going to be used to find the node that is
% closest to the start and end node. However, after completing it was found
% that knnsearch function completed this task better. The old code is
% included here for documentation purposes


%     % Finds the closest node in the sample to the start position in cspace
%     test_samples_start = samples;
%     test_samples_end = samples;
%
%     for i = 1:length(samples)
%         check_start_node = abs(test_samples_start-q_start);
%         length_start_check = length(check_start_node);
%         RMS_start = zeros(length(samples),1);
%         for L = 1:length_start_check
%             RMS_start(L) = sqrt(sum(check_start_node(L,:).^2));
%         end
%         [~, start_node] = min(RMS_start);
%         if check_edge(robot, q_start, samples(start_node), link_radius, sphere_centers, sphere_radii)==false
%             nn_start = start_node;
%             break
%         end
%         test_samples_start(start_node)=test_samples_start(start_node)+1000;
%     end
%     
%     % Finds the closest node in the sample to the end position in cspace
%     for j = 1:length(samples)
%         check_end_node = abs(test_samples_end-q_goal);
%         length_end_check = length(check_end_node);
%         RMS_end = zeros(length(samples),1);
%         for L = 1:length_end_check
%                 RMS_end(L) = sqrt(sum(check_end_node(L,:).^2));
%         end
%         RMS_end = RMS_end.^0.5;
%         [~, end_node] = min(RMS_end);
%         if check_edge(robot, q_goal, samples(end_node), link_radius, sphere_centers, sphere_radii)==false
%             nn_end = end_node;
%             break
%         end
%         test_samples_start(end_node)=test_samples_start(end_node)+1000;
%     end