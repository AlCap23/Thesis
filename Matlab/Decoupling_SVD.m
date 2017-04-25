%% Make a SVD Composition from a MIMO TF

clear all
close all
clc

%% Make a TF

G = tf({10,5;3,4},{[1,1],[1,3];[1,1],[1,1]});

%% Get the singular values for the new Matrix Q
Q = tf('s');

Q(1,1) = sqrt(2)*(sqrt((G(1,1)+G(2,2))^2+(G(1,2)-G(2,1))^2 ) - sqrt((G(1,1)-G(2,2))^2+(G(1,2)+G(2,1))^2 ));
Q(2,2) = 2*(sqrt((G(1,1)+G(2,2))^2+(G(1,2)-G(2,1))^2 ))-Q(1,1);



