% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    
    % Defines the pivot locations for each link. Link 1 pivot is with
    % respect to the origin and link 2 pivot is with respect to the first
    % link
    origin1 = [robot.pivot1];
    origin2 = [robot.pivot2];
    
    % Homogeneous transforms for each point in the link. Uses the origin
    % points defined above to determine translation
    T_O_1 = [cos(q(1)) -sin(q(1)) 0 origin1(1); sin(q(1)) cos(q(1)) 0 origin1(2); 0 0 1 0; 0 0 0 1];
    T_1_2 = [cos(q(2)) -sin(q(2)) 0 origin2(1); sin(q(2)) cos(q(2)) 0 origin2(2); 0 0 1 0; 0 0 0 1];
    
    % Pre initialize link matrix for speed
    link1 = zeros(length(robot.link1),1);
    link2 = zeros(length(robot.link2),1);
    
    
    % Finds new link vertex locations with respect to the base frame using
    % the homogeneous transformations defined above
    for i = 1:length(robot.link1)
        link1(:,i) = T_O_1*[robot.link1(:,i);0;1];
    end
    
    for i = 1:length(robot.link2)
        link2(:,i) = T_O_1*T_1_2*[robot.link2(:,i);0;1];
    end
    
    % Removes last two rows from resulting coordinates that were added to
    % allow for use of 4x4 homogeneous transformations
    link1(4,:) = [];
    link1(3,:) =[];
    link2(4,:) = [];
    link2(3,:) =[];
   
    % Output polyshapes using new calculated link vertex positions
    poly1 = polyshape(link1(1,:),link1(2,:));
    poly2 = polyshape(link2(1,:),link2(2,:));
    
    % Finds new pivot points using homogeneous transformations
    pivot1 = origin1; % Pivot for robot 2 does not change
    pivot2_ = T_O_1*T_1_2*[0;0;0;1];
    pivot2_(4) = [];
    pivot2_(3) = [];
    pivot2 = pivot2_;
end