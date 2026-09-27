function msg2 = deconvert(msg1)
% DECONVERT Convert decoded 4-bit messages back into Scout observations (1 to 8).
%   msg2 = deconvert(msg1) is the inverse of act1Conv. Each row of msg1 is
%   one message [b3 b2 b1 b0], MSB first. A vector is read as a bit stream
%   of 4-bit messages, so the 4x1 output of the Hamming Decoder block can
%   be passed in directly, also inside a MATLAB Function block.
%   The padding bit b3 is ignored, so a channel error in it cannot push
%   the result outside 1 to 8.
bits = reshape(msg1.',4,[]).';   % one message per row
msg2 = (bits(:,2:4) > 0.5) * [4; 2; 1] + 1;
end
