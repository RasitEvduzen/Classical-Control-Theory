%% Pole Placement Design via Ackerman VS LQR
clc,clear all,close all;
% System Matrix
A = [0 1 0; 0 0 1; -1 -5 -6];
B = [0 0 1]';
J = [-2+4*j -2-4*j -10];  % Close Loop Pole Location
K = acker(A,B,J);
Q = eye(3);
R = 1;
Kopt = lqr(A,B,Q,R);

sys1 = ss((A-B*K),eye(3),eye(3),eye(3));
sys2 = ss((A-B*Kopt),eye(3),eye(3),eye(3));

t = 0:0.01:20;   % Simulation Time
% Ackerman
xa = initial(sys1,[1 2 3]',t);   % System initial State
xa1 = xa(:,1);
xa2 = xa(:,2);
xa3 = xa(:,3);

% LQR
xb = initial(sys2,[1 2 3]',t);   % System initial State
xb1 = xb(:,1);
xb2 = xb(:,2);
xb3 = xb(:,3);


%% Plot
set(gcf,'Position',[100 100 1720 900])

subplot(3,2,1)
plot(t,xa1,'k','LineWidth',2),grid minor,hold on
plot(t,xb1,'r','LineWidth',2)
ylabel('State X1'),xlabel('Time'),legend('Ackerman','LQR')
subplot(3,2,3)
plot(t,xa2,'k','LineWidth',2),grid minor,hold on
plot(t,xb2,'r','LineWidth',2)
ylabel('State X2'),xlabel('Time'),legend('Ackerman','LQR')
subplot(3,2,5)
plot(t,xa3,'k','LineWidth',2),grid minor,hold on
plot(t,xb3,'r','LineWidth',2)
ylabel('State X3'),xlabel('Time'),legend('Ackerman','LQR')

subplot(3,2,[2,4,6])
plot3(xa1,xa2,xa3,'k','LineWidth',2),grid minor,hold on
plot3(xb1,xb2,xb3,'r','LineWidth',2)
title('System Phase Space'),legend('Ackerman','LQR')
xlabel('State X1'),ylabel('State X2'),zlabel('State X3')
