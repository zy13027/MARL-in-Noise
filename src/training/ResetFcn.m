function in = ResetFcn(in,obstacleLoca,L)
% RESETFCN Random start cells for the Guide and the Scout.
%   in = ResetFcn(in,obstacleLoca,L) picks two different cells of an L-by-L
%   grid (L defaults to 64) that are not obstacles, and stores them in the
%   Simulink.SimulationInput object in as the variable s0 = [guide; scout],
%   one [row col] per row. obstacleLoca holds one obstacle [row col] per
%   row, or is empty (or -1) for no obstacles.
%
%   Example:
%       obstacleLoca = initPos(3,:);
%       env.ResetFcn = @(in) ResetFcn(in,obstacleLoca,L);
if nargin < 3
    L = 64;
end
if ~isnumeric(obstacleLoca)
    error('ResetFcn:obstacleLoca', ...
        'obstacleLoca must be numeric [row col] rows, e.g. initPos(3,:), not text.');
end

free = true(L);
if ~isempty(obstacleLoca) && ~isequal(obstacleLoca,-1)
    free(sub2ind([L L],obstacleLoca(:,1),obstacleLoca(:,2))) = false;
end
freeCells = find(free);
pick = freeCells(randperm(numel(freeCells),2));
[rows,cols] = ind2sub([L L],pick);
s0 = [rows(:) cols(:)];

in = setVariable(in,'s0',s0);
end
