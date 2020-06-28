%%
clc
clear
close all

%% Define the parameters

lw = 0.200; %distance between the origin and the center of mass of the wheel [m]
lp = 0.150; %distance between the origin and the center of mass of the pendulum [m]
mw = 157.5*10^-3; %weight of the wheel [kg]
mp = 468.7*10^-3; %weight of the pendulum stick [kg]
Iw = 9.189*10^-4; %Moment of Inertia of the wheel at his center of mass [kg.m^2]
Ip = 3.1164*10^-4; %Moment of Inertia of the pendulum stick at his center of mass [kg.m^2]
g0 = 9.81; %Gravitational force [m/s^2]
Km = 3.7*10^-2; %Motor torque constant
Imax=5.6; %motor maximum allowed current
m0 = g0*(mp*lp+mw*lw);
m11 = mp*lp^2+mw*lw^2+Ip+Iw;
m12 = Iw;
m21 = Iw;
m22 = Iw;

%% Define transfert function

G_num = [-Km/(m11-m22)]
G_den = [1 0 -m0/(m11-m22)]

%% Define the matrices

A = [0, 1, 0; m22*m0/(m11*m22-m12*m21), 0, 0; -m21*m0/(m11*m22-m12*m21), 0, 0]
B = [0; -m12*Km/(m11*m22-m12*m21); m11*Km/(m11*m22-m12*m21)]
C = eye(3)
D = zeros(3,1,'uint32')

%% Build the system
disp('State-space model')
sys=ss(A,B,C,D,'InputName','u', 'OutputName',{'thetap', 'thetapdot', 'thetawdot'},'StateName',{'thetap', 'thetapdot', 'thetawdot'})
eig(A);
Sc = ctrb(sys);
x0 = [-pi/30; 0; 0];

%% Build PID Controller

Ku = -23.18;
Tu = 93.1612;
Kp = -66.8543;
Ki = -209.7844;
Kd = -9.3624;

%% Start the Control System Designer
%controlSystemDesigner()

% des_poles = [-3; -3; -3];
% K = acker(A, B, des_poles)
 
Q1 = [1000, 0, 0; 0, 1, 0; 0, 0, 10];
R1 = 0.1;
K_lqr1 = lqr(A, B, Q1, R1);

Q = [10, 0, 0; 0, 1, 0; 0, 0, 1];
R = 0.1;
K_lqr = lqr(A, B, Q, R);

Q3 = [10, 0, 0; 0, 1, 0; 0, 0, 1];
R3 = 10;
K_lqr3 = lqr(A, B, Q3, R3);

%% Calculation about A matrix
disp('T : matrix of the eigenvectors of A')
disp('Ahat : matrix of the eigenvalues of A')
% Returns diagonal matrix Ahat of eigenvalues and matrix T whose columns are the corresponding right eigenvectors
[T,Ahat]=eig(A); %T defines the eigenvector & Ahat the eigenvalues
syms x
Charpoly=charpoly(A,x); %characteristic polynomial of matrix A
solve(Charpoly == 0,x); %det(xI-A)=0

%% State-space model to transfer function 
disp('State-space model to transfer function')
H=tf(sys) %Transfer function sys
H_zpk=zpk(sys) %conversion to Zero-Pole-Gain form
size(H) %define the number of inputs and outputs
pzplot(H(1,1),'m',H(2,1),'b',H(3,1),'r')
h=findobj(gca,'type','line');
set(h,'markersize',10,'linewidth',1)
grid on
axis equal