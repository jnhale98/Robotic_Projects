% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)

    % Pre initialize the number of collisions to be 0
    num_collisions = 0;
    % For each q1 and q2 configuration in the path, determine their
    % corresponding polyshapes
    for i = 1:length(q_path)-1
        [~, start_pos, ~, ~] = q2poly(robot, q_path(i,:)');
        % Determine the verticies that make up the corresponding starting
        % polyshape for this poistion in the path
        xy_start = start_pos.Vertices;
        [~, end_pos, ~, ~] = q2poly(robot, q_path(i+1,:)');
        % Determine the verticies that make up the corresponding ending
        % polyshape for the next position in the path
        xy_end = end_pos.Vertices;
        % Add the x verticies and y vertecies for each shape to two
        % respective vectors to represent overall x and y positions for the
        % two polyshapes
        x = [xy_start(:,1);xy_end(:,1)];
        y = [xy_start(:,2);xy_end(:,2)];
        % Use convhull function to find the vertecies that correspond to an
        % overall polyshape that contains the two configurations
        P = convhull(x,y);
        
        % Define a polyshape using the found convhull points, and if it
        % collides with any obstacles then add to the number of collisions,
        % and display the two configurations
        sweep = polyshape(x(P), y(P));
        if area(intersect(sweep, obstacles))>0
            num_collisions=num_collisions+1;
            c1(robot,q_path(i,:));
            c1(robot,q_path(i+1,:));
        end
    end
    
end