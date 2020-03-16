% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q -> 1x4 vector denoting the configuration to check for collision
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: in_collision -> Boolean, true if the robot at configuration q is
%                         in collision with the given spherical obstacles

function in_collision = check_collision(robot, q, link_radius, sphere_centers, sphere_radii, resolution)
    % This funciton checks for collisions by drawing a descritizing two
    % vectors, one from the base to the 4th link, and one for the 4th link.
    % It then checks what the distance is from each tick on the vector is
    % from the center of the obstacle sees if it is larger than the
    % distance at which they would collide.
    
    % Two issues. First is that the collision detection is limited by the
    % resolution of the discritization of the line. If an obstacle had a
    % radius that when added to the thickness of the link was smaller than
    % the distance to the nearest tick mark it would be able to collide
    % with the robot without being detected. This error could be reduced by
    % increasing the resolution of the interpolation
    
    % Second issue is the collision checking vector goes directly from the
    % base to the end of the third link, undershooting the elbow joint that
    % connects link 2 and 3. As a result the robot could be under the
    % obstacle and collide but not be detected because the line it is
    % checking against is still out of range from the obstacle radius plus
    % the link thickness

    %This block of code defines three points in the 3d workspace. One is
    %the base of the robot at the origin. One is the end of the first three
    %links, and the last is the end of the end effector
    x1 = [0 0 0]';
    T2 = robot.A(1,q) * robot.A(2,q) * robot.A(3,q);
    x2 = T2.t;
    T3 = T2 * robot.A(4,q);
    x3 = T3.t;
    
    if nargin < 6
        resolution = 11;
    end
    
    % 
    ticks = linspace(0, 1, resolution);
    n = length(ticks);
    % Discritizes the length from the robot base to the end of the third
    % link
    x12 = repmat(x1, 1, n) + repmat(x2 - x1, 1, n) .* repmat(ticks, 3, 1);
    
    % Discritizes the length from the end of the third link to the end
    % effector
    x23 = repmat(x2, 1, n) + repmat(x3 - x2, 1, n) .* repmat(ticks, 3, 1);
    points = [x12 x23];
    
    in_collision = false;
    for i = 1:size(sphere_centers, 1)
        % Checks each sphere entered, and ensures that the total distance
        % if you add the radius of the sphere to the link thickness is less
        % than the distance from a link to the obstacle
        if any(sum((points - repmat(sphere_centers(i,:)', 1, size(points, 2))).^2, 1) < (link_radius + sphere_radii(i)).^2)
            in_collision = true;
            break;
        end
    end
end