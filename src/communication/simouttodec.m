function act2 = simouttodec(data)
% SIMOUTTODEC Most frequent Scout observation in logged decoder output.
%   act2 = simouttodec(data) takes the decoded bits of one or more 4-bit
%   messages and returns the observation (1 to 8) that occurs most often.
%   data can be:
%     4x1xN  logged by a To Workspace block (for example out.simout.Data)
%     Nx4    one message per row
%     4xN    one message per column
%     4x1    a single message
%   A 4x4 array is read as one message per row. Inside a MATLAB Function
%   block use deconvert instead.
if ndims(data) == 3 || size(data,2) ~= 4
    data = reshape(data,4,[]).';   % one message per row
end
act2 = mode(deconvert(data));
end
