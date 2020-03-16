% Input: cspace -> NxN matrix: cspace(i,j)
%                  == 1 if [q_grid(i); q_grid(j)] is in collision,
%                  == 0 otherwise
% Output: padded_cspace -> NxN matrix: padded_cspace(i,j)
%                          == 1 if cspace(i,j) == 1, or some neighbor of
%                                  cell (i,j) has value 1 in cspace
%                                  (including diagonal neighbors)
%                          == 0 otherwise

function padded_cspace = C7(cspace)
% Note, cspace resolution had to be increased to 300 to be able to pad
% without covering the start position.
    padded_cspace=cspace;
    % Positions of 1's in cspace
    [q1,q2] = find(cspace==1);
    % Use the adjacent function to find the surrounding cells for each
    % obstructed cell in cspace. If a cell adjacent to a wall is a 0 then
    % we can turn it into a 1 to pad the obstacle. Only checking the cspace
    % for adjacent cells and not the padded_space prevents us from growing
    % the padding forever
    for i = 1:length(q1)
        near = adjacent(cspace, [q1(i),q2(i)]);
        for k = -1:1
            for w = -1:1
                if near(k+2,w+2)==0
                    padded_cspace(q1(i)+k,q2(i)+w)=1;
                end
            end
        end
    end   
end