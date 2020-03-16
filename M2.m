% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples in PRM
%        num_neighbors -> Integer denoting number of closest neighbors to
%                         consider in PRM
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: samples -> num_samples x 4 matrix, sampled configurations in the
%                    roadmap (vertices)
%         adjacency -> num_samples x num_samples matrix, the weighted
%                      adjacency matrix denoting edges between roadmap
%                      vertices. adjacency(i,j) == 0 if there is no edge
%                      between vertex i and j; otherwise, its value is the
%                      weight (distance) between the two vertices. For an
%                      undirected graph, the adjacency matrix should be
%                      symmetric: adjacency(i,j) == adjacency(j,i)

function [samples, adjacency] = M2(robot, q_min, q_max, num_samples, num_neighbors, link_radius, sphere_centers, sphere_radii)
    
    v = 0;
    samples = zeros(num_samples, 4); % Pre initialize samples matrix for the number of samples
    adjacency = zeros(num_samples); % Pre allocate the adjacency matrix
    
    % Loop through for the number of samples we'd like to use
    while v<num_samples
        % Pre initialize a sample joint configuration to test
        sample_config = zeros(1,length(q_max));
        % For the number of angles in the system, define random angles
        % between the max and min joint angles for each joint
        for i=1:length(q_max)
            sample_config(i)= q_min(i)+(q_max(i)-q_min(i)).*rand();
        end
        % If the configuration is not in collision with the obstacles, then
        % add the configuration to the samples matrix
        if check_collision(robot, sample_config, link_radius, sphere_centers, sphere_radii)==false
            samples(v+1,:) = sample_config;
            v=v+1;
        end
    end
 
    % For every sample, search for it's 10 nearest neighbors. If the edge
    % between each neighbor is not in collision then add that edge to the
    % adjaceny matrix. We call for 11 neighbors below beacuse the
    % respective sample is included in the search
    for i = 1:length(samples)
        % Outputs the indecies for the neighbors as well as their distances
        [Idv,a] = knnsearch(samples,samples(i,:),'K',num_neighbors+1);
            for j = 1:length(Idv)
                % For each neighbor found, check if the edge collides. If
                % it doesn't add it to the adjacency matrix
                if check_edge(robot, samples(i,:), samples(Idv(j),:),link_radius, sphere_centers, sphere_radii)==false
                    % Add the distance weight for the corresponding node to
                    % the adjacency matrix
                    adjacency(i,Idv(j)) = a(j);
                    adjacency(Idv(j),i) = a(j);                    
                end
            end
    end
end