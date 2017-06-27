function System = FOTD_AREA(y,u,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Check the length of the arrays
if length(u)~=length(y)
    error('Input has to be same Lenght!')
end

% Get the last 5 percent
tenth = round(0.05*length(y))

% Get the DC Gain of the System
K = (mean(y(end-tenth:end)))/mean(u(end-tenth:end));

% Get the average Time
Tar = trapz(t,abs(y(end)-y))/abs(K);

% Get the position
tloc = find(t < Tar);
% Get the Time Constant
T = exp(1)/K*trapz(t(tloc),y(tloc));
% Get the Delay
L = Tar-T;

% Check if any parameter is below zero
% if L<0 || T < 0 || Tar < 0
%    % Search for maximum
%    [ymax,locmax] = max(y);
%    % If Maximum is first entry, search for minimum
%    if locmax == 1
%        [ymax,locmax] = min(y)
%    end
%    % Make shorter vector
%    System = FOTD_AREA(y(1:locmax),u(1:locmax),t(1:locmax));
% end

System = tf([K],[T,1],'IODelay',L)
end

