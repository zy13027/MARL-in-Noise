function msg1 = act1Conv(act1Outcome)
% ACT1CONV Convert Guide actions (1 to 8) into 4-bit messages.
%   msg1 = act1Conv(act1Outcome) returns one row [b3 b2 b1 b0], MSB first,
%   per action, holding the binary value of act1Outcome-1:
%   1 -> [0 0 0 0], 2 -> [0 0 0 1], ..., 8 -> [0 1 1 1].
%   b3 is always 0; it pads the message to the k = 4 bits of the
%   Hamming (7,4) code. Transpose a single message (msg1.') to feed the
%   Hamming Encoder block. deconvert is the inverse.
assert(all(act1Outcome(:) >= 1 & act1Outcome(:) <= 8 & act1Outcome(:) == round(act1Outcome(:))), ...
    'act1Conv:outOfRange','Guide actions must be integers from 1 to 8.');
msg1 = mod(floor((act1Outcome(:) - 1) ./ [8 4 2 1]), 2);
end
