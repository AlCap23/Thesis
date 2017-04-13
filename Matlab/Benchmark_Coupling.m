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
time = 0:1:100; % Time for simulation
u = ones(length(time),2); % Inputs for the system
IE1 = zeros(n,1); % Preallocate the Integrated Error and the Maximum Error
IE2 = zeros(n,1);
IE3 = zeros(n,1);
ME1 = zeros(n,1);
ME2 = zeros(n,1);
ME3 = zeros(n,1);


for loop = 1:1:n
   % Create 8 random variables for T and K of a 2x2 Process
   % Time Constant: mean = 200, standard deviation is sqrt(49) ( produces stable
   % processes)
   T = (rand(2,2)*sqrt(49.0))+200*ones(2,2);
   % Gain: mean = 10, standard deviation is sqrt(9) 
   K = (rand(2,2)*sqrt(9.0))+10*ones(2,2);
   % Loop over every entry of the 2x2 System
   for i = 1:2
       for j = 1:2
            num{i,j} = [K(i,j)];
            den{i,j} = [T(i,j),1];
       end
   end
   % Make a Transfer Function
   G = tf(num,den);
   % Choose constrains
   c = [0.2,0.2,sqrt(2),sqrt(2)];
   % Make a controller
   GG1 = Decoupling_A(G,c);
   GG2 = Decoupling_D(G,c);
   GG3 = Decoupling_RGA(G,c);
   % Simulate step response for unit step
   y1 = lsim(GG1,u,time);
   y2 = lsim(GG2,u,time);
   y3 = lsim(GG3,u,time);
   % Compute the integrated Error for each input, add and weight and store for benchmarking
   IE1(loop) = (0.5*sum(sum((u-y1).*(time(end)/(length(time)+1)))));
   IE2(loop) = (0.5*sum(sum((u-y2).*(time(end)/(length(time)+1)))));
   IE3(loop) = (0.5*sum(sum((u-y3).*(time(end)/(length(time)+1)))));
   % Get the Maximum of the Error
   ME1(loop) = 0.5*sum(max(abs(u-y1)))-1;
   ME2(loop) = 0.5*sum(max(abs(u-y2)))-1;
   ME3(loop) = 0.5*sum(max(abs(u-y3)))-1;
   % Add other measurements!
end

%% Save the Data!

save('Bechmarking_Coupling_Results.csv','IE1','IE2','IE3','ME1','ME2','ME3')

%% Postprocessing Data
% Set a limit for the IE
limit = 1000;
% Check if limit is reached
IE1_PP = IE1;
IE2_PP = IE2;
IE3_PP = IE3;

for loop = 1:1:n
   if IE1_PP(loop) > limit
       IE1_PP(loop) = limit;
   end
   if IE2_PP(loop) > limit
       IE2_PP(loop) = limit;
   end
   if IE3_PP(loop) > limit
       IE3_PP(loop) = limit;
   end
end

% Make a histogram for values from -(limit-10) to +(limit-10)
% Make the range vector
limits = [-(limit-10),+(limit-10)];
figure()
h1 = histogram(IE1_PP,'BinLimits',limits,'NumBins',99*2+1,'Normalization','probability');
%alpha(0.3)
hold on
h2 = histogram(IE2_PP,'BinLimits',limits,'NumBins',99*2+1,'Normalization','probability');
%alpha(0.3)
hold on
h3= histogram(IE3_PP,'BinLimits',limits,'NumBins',99*2+1,'Normalization','probability');
%alpha(0.3)
xlabel('Weighted IE')
ylabel('Propability (Normalized)')
legend('Aström','Disturbance','RGA Control')
grid on

%limits = [-990,990]
figure()
histogram(ME1,'BinLimits',limits,'NumBins',99*2+1,'Normalization','probability')
%alpha(0.3)
hold on
histogram(ME2,'BinLimits',limits,'NumBins',99*2+1,'Normalization','probability')
%alpha(0.3)
hold on
histogram(ME3,'BinLimits',limits,'NumBins',99*2+1,'Normalization','probability')
%alpha(0.3)
xlabel('max(abs(e(t)))-1')
ylabel('Propability (Normalized)')
legend('Aström','Disturbance','RGA Control')
grid on

limits = [-10,10]
figure()
histogram(ME1,'BinLimits',limits,'NumBins',99,'Normalization','probability')
%alpha(0.3)
hold on
histogram(ME2,'BinLimits',limits,'NumBins',99,'Normalization','probability')
%alpha(0.3)
hold on
histogram(ME3,'BinLimits',limits,'NumBins',99,'Normalization','probability')
%alpha(0.3)
xlabel('max(abs(e(t)))-1')
ylabel('Propability (Normalized)')
legend('Aström','Disturbance','RGA Control')
grid on


