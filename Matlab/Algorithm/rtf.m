function G = rtf(order,input,output)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Empty Transfer Function
G = tf(ones(input,output));

% Make a system which is non derivatives
for inputs = 1:input
    for outputs = 1:input
        for sorder = 1:order
            
            % Random Numerator
            if sorder > 1
                Num = [rand+1,1];
            else
                % Gain between 1 and 20
                Num = 19^(1/order)*rand+1;
                if abs(Num) < 1
                    Num = 1.2;
                end
            end
            % Random Denominator
            Den =  [(60-10)^(1/order)*rand+10,1];
            
            
            G(inputs,outputs) = G(inputs,outputs)*tf(Num,Den,'IODelay',0);
        end
    end
    
    
end
% Random Delay
G.IODelay = Den(1)*rand(input,output)+0.2*Den(1)*eye(input,output);
