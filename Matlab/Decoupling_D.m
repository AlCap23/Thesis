function CL = Decoupling_D(TF,Constrains)

% System Size
sys_size = size(TF);

% Constrains
if ~exist('Constrains','var')
    Constrains = [sqrt(2), sqrt(2)];
end

% Get the Phase Margins
MS = Constrains(1,1:2); % Maximum of the sensitivity
PM = 2*asin(1/2./MS); % Phase Margin is at least 2 arcsin(1/2/Ms)

% Subdivide the transferfunction in main Diagonal and minor diagonal
G_M = tf('s');
G_m = tf('s');

for outputs = 1:sys_size(2)
    for inputs = 1:sys_size(1)
        if outputs == inputs
            % Main Diagonal TF
            G_M(outputs,inputs) = TF(outputs,inputs);
        else
            % Minor Diagonal TF
            G_m(outputs,inputs) = TF(outputs,inputs);
        end
    end
end

% Get the Decoupling
% Make PID for the main Diagonal
C = tf('s')
for output = 1:sys_size(2)
    % Tunes for the Phase Margin, which is essentially f(MS)
   opts = pidtuneOptions('PhaseMargin',PM(1,outputs));
   % Tune a PI Controller
   TunedControl = pidtune(G_M(output,output),'PI',opts);
end
H = feedback(G_M*C,eye(sys_size));
C = inv(TF)*H*inv(eye(sys_size)-H);

% Get a corrected system
%Q = TF*D;

% Make the PID for main diagonal
% C = tf('s');
% 
% for output = 1:sys_size(2)
%     % Tunes for the Phase Margin, which is essentially f(MS)
%    opts = pidtuneOptions('PhaseMargin',PM(1,outputs));
%    % Tune a PI Controller
%    TunedControl = pidtune(TF(output,output),'PI',opts);
% end

% Make the closed loop
CL = feedback(TF*C,eye(sys_size));

end