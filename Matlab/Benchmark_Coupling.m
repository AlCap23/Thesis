%% Hygiene
clear all
close all
clc

%% Model Data
%  FO Model for Process
%  PI Control

%% Loop for n cycles and get random variables

n = 500;
time = 0:1:100;
u = ones(length(time),2);
IE1 = zeros(n,1);
%IE2 = zeros(n,1);
IE3 = zeros(n,1);
ME1 = zeros(n,1);
%ME2 = zeros(n,1);
ME3 = zeros(n,1);


k = [;];

% Create 8 random variables for T and K of a 2x2 Process
% Time Constant: mean = 200, standard deviation is sqrt(49) ( produces stable
% processes)
T = (randn(2,2)*sqrt(49.0))+200*ones(2,2);
% Gain: mean = 10, standard deviation is sqrt(9) 
K = (randn(2,2)*sqrt(9.0))+10*ones(2,2);
for i = 1:2
    for j = 1:2
        num{i,j} = [K(i,j)];
        den{i,j} = [T(i,j),1];
    end
end
% Make a Transfer Function
G = tf(num,den);


for loop = 1:1:n
   % Choose constrains
   c = [100*rand()+0.1,100*rand()+0.1,sqrt(2),sqrt(2)];
   k = [k c(1,1:2)'];
   % Make a controller
   GG1 = Decoupling_A(G,c);
   GG2 = Decoupling_D(G);
   GG3 = Decoupling_RGA(G,c);
   % Simulate step response for unit step
   y1 = lsim(GG1,u,time);
   y2 = lsim(GG2,u,time);
   y3 = lsim(GG3,u,time);
   % Compute the integrated Error for each input, add and weight and store for benchmarking
   IE1(loop) = (0.5*sum(sum((u-y1).*(time(end)/(length(time)+1)))));
   %IE2(loop) = (0.5*sum(sum((u-y2).*(time(end)/(length(time)+1)))));
   IE3(loop) = (0.5*sum(sum((u-y3).*(time(end)/(length(time)+1)))));
   % Get the Maximum of the Error
   ME1(loop) = 0.5*sum(max(abs(u-y1)))-1;
   %ME2(loop) = 0.5*sum(max(abs(u-y2)))-1;
   ME3(loop) = 0.5*sum(max(abs(u-y3)))-1;
end

%% Postprocessing Data
% Set a limit for the IE
limit = 1000;
% Check if limit is reached
IE1_PP = IE1;
%IE2_PP = IE2;
IE3_PP = IE3;

for loop = 1:1:n
   if IE1_PP(loop) > limit
       IE1_PP(loop) = limit;
   end
   %if IE2_PP(loop) > limit
   %    IE2_PP(loop) = limit;
   %end
   if IE3_PP(loop) > limit
       IE3_PP(loop) = limit;
   end
end
[X,Y] = meshgrid(k(1,:)',k(2,:)');

e1 = scatteredInterpolant(k(1,:)',k(2,:)',IE1_PP);
e2 = scatteredInterpolant(k(1,:)',k(2,:)',IE3_PP);
m1 = scatteredInterpolant(k(1,:)',k(2,:)',ME1);
m2 = scatteredInterpolant(k(1,:)',k(2,:)',ME3);

Z1 = e1(X,Y);
Z2 = e2(X,Y);
Z3 = m1(X,Y);
Z4 = m2(X,Y);

figure()
mesh(X,Y,Z1)


figure()
mesh(X,Y,Z2)

figure()
mesh(X,Y,Z3)

figure()
mesh(X,Y,Z4)