%% Benchmarking of different decoupling Methods
%  Julius Martensen, 2017/04/12
%  Decoupling_A - Method based on Paper by Aström et.al
%  Decoupling_D - See Coupling as Disturbance
%  Decoupling_RGA - Neglect coupling and design control just for main
%  diagonal

%% Hygiene
clear all
close all
clc

%% Model Data
%  FO Model for Process
%  PI Control

%% TO DO
%  Add several criteria for the error ( e.g. for overshooting etc )

%% Loop for n cycles and get random variables

n = 100; % Number of Loops
time = 0:1:1000; % Time for simulation
u = ones(length(time),2); % Inputs for the system
IE1 = zeros(n,1); % Preallocate the Integrated Error and the Maximum Error
%IE2 = zeros(n,1);
%IE3 = zeros(n,1);
ME1 = zeros(n,1);
%ME2 = zeros(n,1);
%ME3 = zeros(n,1);


for loop = 1:1:n
   % Create 8 random variables for T and K of a 2x2 Process
   % Time Constant: Uniform Distribution 200+-50 
   T1 = (rand(2,2)*50)+200*ones(2,2);
   T2 = (rand(2,2)*2)+20*ones(2,2);
   % Gain: Uniform Distribution 10+-9 
   K1 = (rand(2,2)*9.0)+10*ones(2,2);
   K2 = ones(2,2);
   % Loop over every entry of the 2x2 System
   for i = 1:2
       for j = 1:2
            % Transfer Function 1
            num1{i,j} = [K1(i,j)];
            den1{i,j} = [T1(i,j),1];
            % Transfer Function 2
            num2{i,j} = [K2(i,j)];
            den2{i,j} = [T2(i,j),1];
            % Model Function
            numM{i,j} = [K2(i,j)];
            denM{i,j} = [T1(i,j),1]+[T2(i,j),1];            
       end
   end
   % Make a Transfer Function of Order 2
   G1 = tf(num1,den1);
   G2 = tf(num1,den1);
   % True Function
   G = G1*G2;
   % Model Function
   G_M = tf(numM,denM);
   % Choose constrains
   c = [0.2,0.2,sqrt(2),sqrt(2)];
   % Make a controller
   C = tf('s');
   for outputs = 1:2
        PM = 2*asin(1/2./c(1,2+outputs))/pi*180;
        opts = pidtuneOptions('PhaseMargin',PM);
        C(outputs,outputs) = pidtune(G_M(outputs,outputs),'PI',opts);
   end
   
   % Get the closed Loop
   CL_M = feedback(G_M*C,eye(2));
   CL = feedback(G*C,eye(2));
   %GG1 = Decoupling_A(G,c);
   %GG2 = Decoupling_D(G,c);
   %GG3 = Decoupling_RGA(G,c);
   % Simulate step response for unit step of the closed loop
   y1 = lsim(CL_M,u,time);
   %y2 = lsim(GG2,u,time);
   y2 = lsim(CL,u,time);
   % Simulate step response for unit step of the system
   y3 = lsim(G_M,u,time);
   y4 = lsim(G,u,time);
   % Compute the integrated Error for each input, add and weight and store for benchmarking
   IE1(loop) = (0.5*sum(sum((y1-y2).*(time(end)/(length(time)+1)))));
   IE2(loop) = (0.5*sum(sum((y3-y4).*(time(end)/(length(time)+1)))));
   %IE3(loop) = (0.5*sum(sum((u-y3).*(time(end)/(length(time)+1)))));
   % Get the Maximum of the Error
   ME1(loop) = 0.5*sum(max(abs(y1-y2)));
   ME2(loop) = 0.5*sum(max(abs(y3-y4)));
   %ME3(loop) = 0.5*sum(max(abs(u-y3)))-1;
   % Add other measurements!
end
% 
% 
% %% Benchmark Model Error 
% % Create Second Order Systems and approximate them with First Order Models
% 
% clear all
% close all
% clc
% 
% %% Loop for n cycles and get random Processes
% 
% n = 50000; % Sample number
% time = 0:1:10000; % Time to simulate
% u = ones(length(time),1); % Input , Step
% Model_Error = zeros(n,1); % Preallocate
% PM = 2*asin(1/2/sqrt(2)); % Phase Margin
% 
% for loop = 1:1:n
%     % Create a Model with two Random Time Constants
%     T = (randn(2)*sqrt(40))+200*ones(2,1);
%     % Create a Gain
%     K = randn(1)*sqrt(9.0)+10;
%     % Create a System
%     P = tf(K, [T(1),1])*tf(1,[T(2),1]);
%     % Approximate the system via sum of the time constants
%     P_Mod = tf(K,[sum(T),1]);
%     % Create a Controller with a selected phase margin
%     opts = pidtuneOptions('PhaseMargin',PM);
%     C = pidtune(P_Mod,'PI',opts);
%     % Make the closed Loop
%     CL_Mod = feedback(P_Mod*C,1);
%     % Get the error via the relative error in the phase margin
%     [gm,pm,wgm,wpm] = margin(CL_Mod);
%     Model_Error(loop) = pm/PM;
% end
