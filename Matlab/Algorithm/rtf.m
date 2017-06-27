function G = rtf(order,input,output)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Empty Transfer Function
G = tf(ones(input,output));

% Make a system which is non derivatives
for inputs = 1:input
    for outputs = 1:input
        for sorder = 1:order 
        % Random Delay between 1 and 10 Seconds
        L = (30-10)*rand()+10;
        % Random Numerator
        if sorder > 1
            Num = [(30-1)*rand+1,1];
        else
            Num = 10*rand-10*rand+1;
        end
        % Random Denominator
        Den =  [(30-1)*rand+1,1];
        
        G(inputs,outputs) = G(inputs,outputs)*tf(Num,Den,'IODelay',L);
    end
end


end

