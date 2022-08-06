 function Names = createStateNames(m,n)
            % Create array of state name "[m,n]"   
            
            Names = "";
            ct = 1;
            for ct1 = 1:n
                for ct2 = 1:m
                    Names(ct,1) = sprintf("[%d,%d]",ct2,ct1);
                    ct = ct+1;
                end
            end
        end
        
 