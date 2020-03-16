% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at

function C1(robot, q)
    % Uses the q2poly function to obtain new pivot locations and polyshapes
    % for the designated angle for each pivot point
    [link1, link2, origin1, origin2] = q2poly(robot,q);
    
    % Plot each robot link, where the first is red and the second is blue
    plot(link1, 'FaceColor', 'r')
    plot(link2, 'FaceColor', 'b')
    
    % Plot each pivot point as a dot for each respective robot link
    plot(origin1(1), origin1(2), 'k.', 'MarkerSize', 10);
    plot(origin2(1), origin2(2), 'k.', 'MarkerSize', 10);
end