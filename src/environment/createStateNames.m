function Names = createStateNames(m,n)
% CREATESTATENAMES State names "[row,col]" of an m-by-n grid world.
%   Names = createStateNames(m,n) returns an (m*n)-by-1 string array in
%   linear-index order: "[1,1]", "[2,1]", ..., "[m,1]", "[1,2]", ..., "[m,n]".
[rows,cols] = ndgrid(1:m,1:n);
Names = "[" + rows(:) + "," + cols(:) + "]";
end
