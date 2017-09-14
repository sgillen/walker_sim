% Xinit = [ 1.9193
%     2.4450
%     -0.6410
%     -1.1749
%     0.5265
%     1.2781];
if ~exist('Xinit','var') || isempty(Xinit)
Xinit=[1.9294
    2.4247
   -0.7119
   -1.1819
    0.5618
    1.2083];
end

options = optimoptions('fmincon');
%options = optimoptions('lsqnonlin');

% Set OptimalityTolerance to 1e-3
options = optimoptions(options, 'OptimalityTolerance', 1e-7);

% Set the Display option to 'iter' and StepTolerance to 1e-4
options.Display = 'testing';
options.StepTolerance = 1e-7;
options.MaxFunctionEvaluations = 1e4;

%% Can use either "fmincon" or "lsqnonline" -- or another fn
Xfixed = fmincon(@cg_torso_LCcost,Xinit,[],[],[],[],[],[],[],options) %,);
%Xfixed = lsqnonlin(@cg_torso_LCcost,Xinit,[],[],options); %,[],[],[],[],[],[],[],options);

Xerr = max(abs(Xfixed - cg_torso_step(Xfixed)))



damt = 1e-4;
J = zeros(6,6);

for n=1:6
    d = zeros(6,1); d(n)=damt;
    xtemp = cg_torso_step(Xfixed + d);
    xtemp2 = cg_torso_step(Xfixed - d);
    xnom = cg_torso_step(Xfixed);
    %J(:,n) = (1/damt)*(xtemp-Xfixed);
    %J(:,n) = (1/damt)*(xtemp-xnom); % blue circles with dashed line
    J(:,n) = (1/(2*damt))*(xtemp-xtemp2); % green triangles with '-.' line
    
    
end
[eivec,eival] = eig(J);
eival = diag(eival)