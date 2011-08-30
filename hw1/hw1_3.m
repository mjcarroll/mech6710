%% MECH 6710 Homework 1
% Position analysis of mechanism 4-1
clear; clc;
%% Problem Constraints
AB = 0.09;              % m
BC = 0.40;              % m
CE = 0.25;              % m
CD = 0.12;              % m
EF = 0.21;              % m

a = 0.22;               % m
b = 0.35;               % m
c = 0.40;               % m
phi = 60;               % deg
phi = deg2rad(phi);     % rad
n = 400;                % rpm

simLength = 30;
phiIter = phi + linspace(0,2*pi,simLength);

%% Initial Position

rA = zeros(simLength,2);
rB = zeros(simLength,2);
rC = zeros(simLength,2);
rD = zeros(simLength,2);
rE = zeros(simLength,2);
rF = zeros(simLength,2);

for ii=1:simLength,
    % Start with Driver Link AB
    % A is fixed to ground at origin
    xA = 0;
    yA = 0;
    rA(ii,:) = [xA,yA];
    
    % B is solved with simple trig
    xB = xA + AB*cos(phiIter(ii));
    yB = yA + AB*sin(phiIter(ii));
    rB(ii,:) = [xB,yB];

    % Solve Position of C
    % D is fixed to ground
    xD = a;
    yD = b;
    rD(ii,:) = [xD,yD];
    
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
    rC(ii,:) = [xC,yC];
    
    % Solve for E
    phi2 = atan2(yC - yB, xC - xB);   
    xE = xB + (BC + CE) * cos(phi2);
    yE = yB + (BC + CE) * sin(phi2);
    rE(ii,:) = [xE,yE];
    
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
    rF(ii,:) = [xF,yF];
end

%%
% plot the links
plot([rA(1,1),rB(1,1)],[rA(1,2),rB(1,2)],'k-o','LineWidth',1.5);
hold on;
plot([rB(1,1),rC(1,1)],[rB(1,2),rC(1,2)],'b-o','LineWidth',1.5);
plot([rC(1,1),rE(1,1)],[rC(1,2),rE(1,2)],'b-o','LineWidth',1.5);
plot([rC(1,1),rD(1,1)],[rC(1,2),rD(1,2)],'r-o','LineWidth',1.5);
plot([rF(1,1),rE(1,1)],[rF(1,2),rE(1,2)],'g-o','LineWidth',1.5);

text(rA(ii,1),rA(ii,2),'  A'),text(rB(ii,1),rB(ii,2),'  B'),...
text(rC(ii,1),rC(ii,2),'  C'),text(rD(ii,1),rD(ii,2),'  D'),...
text(rE(ii,1),rE(ii,2),'  E'),text(rF(ii,1),rF(ii,2),'  F');

% plot the trails
plot(rB(:,1),rB(:,2),'k');
plot(rC(:,1),rC(:,2),'r');
plot(rE(:,1),rE(:,2),'b');
plot(rF(:,1),rF(:,2),'g');
axis equal, grid on;

aviobj = close(aviobj);