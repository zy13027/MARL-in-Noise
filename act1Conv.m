function msg1 = act1Conv(act1Outcome)
switch act1Outcome
    case 1
        msg1 = [0 0 0 0];
    case 2
        msg1 = [0 0 0 1];
    case 3
        msg1 = [0 0 1 0];
    case 4
        msg1 = [0 0 1 1];
    case 5
        msg1 = [0 1 0 0];
    case 6
        msg1 = [0 1 0 1];
    case 7
         msg1 = [0 1 1 0];
     case 8
         msg1 = [0 1 1 1];
end 
   end