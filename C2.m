% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    % Preallocate cspace matrix
    cspace = zeros(length(q_grid));
    
    % Loops through each possible joint configuration defined by the input
    % q_grid, and determines if there would be a collision. If there is
    % make the point a 1, if there isn't make the point a 0
    for i = 1:length(q_grid)
        for j = 1:length(q_grid)
            % Use q2poly function to determine the corresponding polyshape
            % for the respective q1 and q2 angles
            [poly1, poly2, ~, ~] = q2poly(robot, [q_grid(i),q_grid(j)]);
            % Determine the amount of interesection between each link and
            % the obstacles
            link1_collision = area(intersect(poly1,obstacles));
            link2_collision = area(intersect(poly2,obstacles));

            % If neither link is in collision then define the q1 and q2
            % coordinate as a 0 in csapce, if they are a 1
            if link1_collision==0 & link2_collision==0
                    cspace(i,j)=0;
            else
                    cspace(i,j)=1;
                
            end
        end
    end
     
end