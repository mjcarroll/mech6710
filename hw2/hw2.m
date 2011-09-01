%% MECH 6710 Homework 1 - Part 1
% Position analysis of mechanism 4-1
clear; clc;

phi = 60;             % deg

%% Problem Constraints
AB = 0.09;              % m
BC = 0.40;              % m
CE = 0.25;              % m
CD = 0.12;              % m
EF = 0.21;              % m

a = 0.22;               % m
b = 0.35;               % m
c = 0.40;               % m
phi = deg2rad(phi);     % rad
n = 400;                % rpm


%% Position Analysis

fprintf('Position Analysis, phi = %g (deg)\n',rad2deg(phi));

% Start with Driver Link AB
% A is fixed to ground at origin
xA = 0;
yA = 0;
rA = [xA,yA,0];

% B is solved with simple trig
xB = xA + AB*cos(phi);
yB = yA + AB*sin(phi);
rB = [xB,yB,0];
phi1 = phi;

% Solve Position of C
% D is fixed to ground
xD = a;
yD = b;
rD = [xD,yD,0];

% Solve for C
eqnC1 = '(xCsol - xB)^2 + (yCsol - yB)^2 = BC^2';
eqnC2 = '(xCsol - xD)^2 + (yCsol - yD)^2 = CD^2';

solC = solve(eqnC1,eqnC2,'xCsol,yCsol');
xCpositions = eval(solC.xCsol);
yCpositions = eval(solC.yCsol);
xC1 = xCpositions(1);
xC2 = xCpositions(2);
yC1 = yCpositions(1);
yC2 = yCpositions(2);
xC = xC2; yC = yC2;
rC = [xC,yC,0];

phi3 = atan2(yC - yD, xC - xD);

% Solve for E
phi2 = atan2(yC - yB, xC - xB);   
xE = xB + (BC + CE) * cos(phi2);
yE = yB + (BC + CE) * sin(phi2);
rE = [xE,yE,0];

% Solve for F
yF = c;
eqnF1 = '(xFsol - xE)^2 + (yF - yE)^2 = EF^2';
solF = solve(eqnF1,'xFsol');
xFpositions = eval(solF);
xF1 = xFpositions(1);
xF2 = xFpositions(2);

if xF1 < xE, xF = xF1;
else xF = xF2;
end
rF = [xF,yF,0];
phi4 = atan2(yF - yE, xF - xE);
phi5 = atan2(yE - yF, xE - xF);

fprintf('rA=[%8.3f, %8.3f, %8.3f] (m)\n', rA); 
fprintf('rB=[%8.3f, %8.3f, %8.3f] (m)\n', rB);
fprintf('rC=[%8.3f, %8.3f, %8.3f] (m)\n', rC); 
fprintf('rD=[%8.3f, %8.3f, %8.3f] (m)\n', rD);
fprintf('rE=[%8.3f, %8.3f, %8.3f] (m)\n', rE); 
fprintf('rF=[%8.3f, %8.3f, %8.3f] (m)\n', rF);

fprintf('phi1=[%8.3f] (deg)\n',rad2deg(phi1));
fprintf('phi2=[%8.3f] (deg)\n',rad2deg(phi2));
fprintf('phi3=[%8.3f] (deg)\n',rad2deg(phi3));
fprintf('phi4=[%8.3f] (deg)\n',rad2deg(phi4));
fprintf('phi5=[%8.3f] (deg)\n',rad2deg(phi5));

%% Velocity and Acceleration Analysis

fprintf('Velocity and Accleration Analysis, phi=%g (deg)\n',rad2deg(phi));
fprintf('n = %g (rpm)\n\n',n);
vA = [0,0,0];
aA = [0,0,0];
vD = [0,0,0];
aD = [0,0,0];

omega1 = [0,0,pi/30 * n];
alpha1 = [0,0,0];

vB1 = vA + cross(omega1,rB); 
vB2 = vB1;
aB1 = aA + cross(alpha1,rB) - dot(omega1,omega1)*rB; 
aB2 = aB1;

omega2z = sym('omega2z','real');
omega3z = sym('omega3z','real');
omega2 = [0,0,omega2z];
omega3 = [0,0,omega3z];
eqvC = vB2 + cross(omega2,rC-rB)-(vD+cross(omega3,rC-rD));
eqvCx = eqvC(1);
eqvCy = eqvC(2);
solvC = solve(eqvCx,eqvCy);
omega2zs = eval(solvC.omega2z);
omega3zs = eval(solvC.omega3z);
omega2 = [0 0 omega2zs];
omega3 = [0 0 omega3zs];
vC = vB2 + cross(omega2,rC-rB);

alpha2z = sym('alpha2z','real');
alpha3z = sym('alpha3z','real');
alpha2 = [0,0,alpha2z];
alpha3 = [0,0,alpha3z];
eqaC2 = aB2 + cross(alpha2,rC-rB)-...
    dot(omega2,omega2) * (rC - rB);
eqaC3 = aD + cross(alpha3,rC-rD)-...
    dot(omega3,omega3) * (rC - rD);
eqaC = eqaC2 - eqaC3;
eqaCx = eqaC(1);
eqaCy = eqaC(2);
solaC = solve(eqaCx,eqaCy);
alpha2zs = eval(solaC.alpha2z);
alpha3zs = eval(solaC.alpha3z);
alpha2 = [0,0,alpha2zs];
alpha3 = [0,0,alpha3zs];
aC = aB2 + cross(alpha2,rC-rB) - dot(omega2,omega2)*(rC-rB);


fprintf('vA=[%8.3f, %8.3f, %8.3f] (m/s)\n', vA);
fprintf('aA=[%8.3f, %8.3f, %8.3f] (m/s/s)\n', aA);
fprintf('\n');
fprintf('vB=[%8.3f, %8.3f, %8.3f] (m/s)\n', vB1);
fprintf('aB=[%8.3f, %8.3f, %8.3f] (m/s/s)\n', aB1);
fprintf('\n');
fprintf('vC=[%8.3f, %8.3f, %8.3f] (m/s)\n', vC);
fprintf('aC=[%8.3f, %8.3f, %8.3f] (m/s/s)\n', aC);
fprintf('\n');
fprintf('vD=[%8.3f, %8.3f, %8.3f] (m/s)\n', vD);
fprintf('aD=[%8.3f, %8.3f, %8.3f] (m/s/s)\n', aD); 
fprintf('\n');


fprintf('omega1 = [%8.3f] (rad/s)\n',omega1(3));
fprintf('omega2 = [%8.3f] (rad/s)\n',omega2(3));
fprintf('omega3 = [%8.3f] (rad/s)\n',omega3(3));

fprintf('alpha1 = [%8.3f] (rad/s)\n',alpha1(3));
fprintf('alpha2 = [%8.3f] (rad/s/s)\n',alpha2(3));
fprintf('alpha3 = [%8.3f] (rad/s/s)\n',alpha3(3));