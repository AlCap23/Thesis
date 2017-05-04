% Approximate the sum of two FOTD with a FOTD

clear all
close all
clc

%% Define FOTD
Max_Sample = 1;

for Sample = 1:Max_Sample
   % Create a random Gain 
   K = 5+rand(1,2)*10;
   % Create a random Time constant
   T = 20+rand(1,2)*20;
   % Create a random Delay
   L = 20+rand(1,2)*20;
   % First and second System
   G1 = tf(K(1),[T(1),1],'IODelay',L(1));
   G2 = tf(K(2),[T(2),1],'IODelay',L(2));
   % Approximate the third
   % Delay
   LA = mean(L);
   % Gain
   KA = dcgain(G1+G2);
   % Time Constant
   TA = (K(1)*(T(1)+L(1)) + K(2)*(T(2)+L(2)))/(K(1)+K(2)) - LA;
   % TF
   GA = tf(KA,[TA,1],'IODelay',LA);
   % Plot Step
   figure(1)
   step(G1+G2)
   hold on
   step(GA)
   legend('Real System','Approximation')
   grid on
%    % Plot Bode
%    figure(2)
%    bodeplot(G1+G2)
%    hold on 
%    bodeplot(GA)
%    legend('Real System','Approximation')
%    grid on  
end