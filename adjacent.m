function near_cell = adjacent(array, loc)
   
    % Sets sets of the array to determine what the outer bounds would be
    N = length(array);
    % Preallocate arrays for the adjacent cells
    near_cell = zeros(3);
    % array(loc(1),loc(2));
    
    % Define the center of the adjacent matrix to test as the chosen point
    % from the cspace matrix
    near_cell(2,2)=array(loc(1),loc(2));
    
    % Loop through each position of the adjacent matrix and compare to the
    % inputed cspace matrix. If the index it out of the bounds of the
    % matrix make that value -5 so that we do not attempt to enter a value
    % into distance matrix that is outside of the bounds
    for i =-1:1
        for j =-1:1
            if loc(1)+i > N || loc(2)+j > N || loc(1)+i < 1 || loc(2)+j < 1
               near_cell(i+2,j+2) = -5;
            else
                % Populate the adjacent cell matrix with the cells located
                % about the center in the actual cspace or distance matrix
                near_cell(i+2,j+2) = array(loc(1)+i,loc(2)+j);
            end
           
           
           
        end
    end
end
