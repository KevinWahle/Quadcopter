%% Section 1
syms theta phi psi

Rx = [1         0                0    ;
      0         cos(phi)    -sin(phi) ;
      0         sin(phi)     cos(phi) ];
  
Ry = [cos(theta)    0       sin(theta);
      0             1           0     ;
     -sin(theta)    0       cos(theta)];
  
Rz = [cos(psi)   -sin(psi)      0     ;
      sin(psi)    cos(psi)      0     ;
        0           0           1    ];
 
R = Rz*Ry*Rz
%% 

roll_dot_measurement = [51, 50, 23, -16, -55, -60, -22, 33, 65, 40, -19, -43, -29, -9, 0, 24, 48, 28, -21, -61, -54, 0, 50, 57, 36, -4, -51, -72, -34, 30, 72, 49, -8, -41, -32, -22, -12, 22, 58, 42, -12, -62, -65, -4, 48, 62, 49, 8, -46, -78, -51, 14, 72, 69, 3, -38, -42, -33, -21, 20, 60, 56, 1, -60, -72, -20, 39, 64, 61, 15, -43, -81, -64, 0, 73, 78, 21, -36, -49, -42, -25, 19, 64, 67, 4, -60, -79, -36, 30, 67, 68, 23, -36, -80, -68, 2, 64, 76, 38, -27, -52, -49, -30, 16, 65, 69, 12, -54, -84, -52, 24, 70, 75, 35, -40, -83, -69, -3, 67, 81, 32, -22, -51, -52, -34, 26, 74, 71, 19, -50, -83, -56, 17, 67, 67, 21, -32, -73, -68, -6, 69, 78, 33, -22, -49, -53, -30, 32, 88, 76, 9, -61, -76, -39, 16, 54, 54, 22, -25, -70, -71, 5, 71, 77, 31, -19, -49, -53, -27, 43, 95, 69, -2, -67, -67, -23, 30, 51, 39, 10, -26, -66, -58, 6, 66, 65, 19, -16, -36, -46, -27, 39, 97, 55, -23, -67, -50, -9, 27, 47, 36, 10, -30, -69, -57, 5, 63, 59, 15, -17, -31, -38, -23, 47, 92, 59, -26, -79, -48, 0, 29, 41, 27, 9, -21, -66, -64, 9, 63, 58, 17, -15, -31, -43, -27, 40, 97, 56, -32, -82, -48, 2, 28, 38, 30, 14, -23, -72, -62, 1, 58, 58, 23, -6, -35, -54, -31, 47, 95, 61, -21, -76, -46, -1, 25, 41, 35, 14, -26, -68, -63, -12, 46, 57, 35, -2, -47, -64, -23, 45, 84, 56, -10, -60, -42, -12, 11, 38, 49, 29, -23, -69, -66, -20, 35, 60, 49, 0, -66, -69, -23, 36, 71, 55, 4, -43, -52, -29, 13, 52, 55, 28, -20, -59, -63, -30, 23, 63, 53, -6, -70, -77, -27, 30, 67, 62, 18, -36, -54, -38, 9, 54, 56, 35, -3, -52, -66, -44, 17, 67, 62, 0, -68, -78, -36, 14, 55, 66, 33, -23, -60, -50, 1, 60, 61, 43, 4, -47, -77, -56, 5, 65, 67, 8, -62, -72, -44, -4, 47, 77, 56, -8, -66, -63, -8, 52, 70, 59, 20, -57, -88, -66, 1, 63, 52, 0, -45, -61, -49, -12, 38, 90, 81, -1, -85, -70, -3, 57, 79, 66, 21, -56, -103, -78, 7, 71, 63, 4, -44, -61, -49, -17, 39, 98, 93, 5, -92, -86, -9, 66, 86, 72, 14, -63, -102, -62, 9, 64, 68, 11, -53, -74, -54, -12, 55, 107, 91, -8, -90, -77, -9, 57, 87, 71, 12, -74, -106, -62, 12, 80, 76, 10, -57, -72, -47, -8, 56, 105, 86, -19, -92, -82, -16, 58, 91, 57, -8, -71, -97, -49, 19, 79, 74, 11, -56, -65, -33, 0, 50, 98, 77, -11, -85, -89, -28, 61, 91, 52, -11, -65, -86, -47, 17, 75, 79, 18, -55, -62, -24, 4, 46, 85, 68, -4, -73, -78, -17, 55, 76, 39, -16, -55, -75, -52, 8, 72, 80, 22, -53, -68, -22, 14, 49, 71, 50, 0, -57, -65, -24, 36, 61, 30, -14, -49, -62, -44, 3, 49, 66, 32, -32, -57, -25, 7, 46, 63, 34, -6, -40, -54, -27, 29, 49, 25, -16, -42, -52, -38, 6, 51, 60, 14, -32, -41, -21, 3, 39, 57, 36, -1, -33, -48, -25, 15, 34, 23, -7, -36, -48, -38, 3, 44, 44, 17, -20, -37, -18, 3, 35, 55, 30, -8, -33, -37, -17, 12, 25, 16, -11, -36, -46, -31, 11, 43, 36, 3, -23, -26, -13, 4, 32, 50, 25, -9, -30, -30, -10, 10, 13, 8, -5, -28, -43, -37, 2, 38, 32, -1, -24, -23, -8, 7, 27, 48, 33, -5, -33, -29, -1, 15, 15, 8, -9, -31, -44, -38, 3, 41, 36, -1, -25, -27, -10, 2, 27, 50, 38, -1, -39, -33, -1, 21, 19, 10, -11, -35, -42, -33, -2, 32, 34, 4, -22, -29, -17, 0, 34, 55, 39, -4, -41, -37, 2, 23, 19, 8, -11, -36, -38, -30, -7, 26, 31, 5, -22, -31, -21, 5, 34, 55, 41, -3, -41, -37, -1, 23, 23, 12, -16, -43, -40, -27, 1, 29, 32, 8, -23, -34, -16, 14, 43, 56, 33, -12, -43, -39, -2, 26, 25, 17, -13, -44, -45, -26, 7, 34, 35, 7, -27, -32, -19, 9, 46, 60, 36, -15, -51, -37, 7, 29, 26, 17, -16, -45, -45, -22, 11, 36, 32, 2, -28, -33, -22, 10, 43, 58, 33, -18, -50, -33, 6, 28, 27, 15, -16, -47, -42, -14, 15, 30, 25, 5, -18, -32, -22, 7, 41, 48, 16, -26, -39, -23, -1, 19, 28, 19, -10, -42, -38, -5, 18, 24, 16, 4, -10, -21, -20, 2, 31, 29, 5, -17, -19, -14, -10, 5, 22, 20, -3, -32, -27, -4, 12, 9, 9, 13, 1, -13, -12, 1, 12, 13, 2, -6, -4, -11, -18, -7, 14, 18, 1, -17, -17, -4, 4, 3, 7, 18, 5, -11, -10, 3, 0, -6, 0, 7, 7, -8, -27, -20, 5, 19, 10, -7, -10, -6, -6, -3, 8, 25, 17, -6, -10, -2, -8, -10, -1, 12, 14, -8, -32, -28, 0, 19, 14, 1, -6, -12, -14, -1, 15, 26, 19, 2, -2, -5, -17, -16, 0, 18, 23, -6, -35, -34, -5, 18, 20, 9, -4, -16, -15, -2, 20, 29, 17, 6, 0, -9, -24, -11, 8, 19, 21, -5, -39, -28, -3, 13, 19, 7, -1, -12, -14, -3, 20, 31, 22, 12, 0, -14, -26, -8, 16, 21, 14, -10, -36, -28, -6, 10, 22, 11, -5, -17, -16, 4, 23, 33, 28, 9, -7, -16, -20, -5, 18, 24, 9, -13, -34, -30, -5, 13, 24, 9, -10, -20, -16, 5, 32, 43, 31, -2, -21, -9, 1, -2, 14, 33, 11, -22, -42, -26, -4, 20, 29, -1, -27, -26, -10, 11, 45, 52, 23, -21, -32, -9, 14, 15, 23, 39, -1, -47, -49, -20, 10, 31, 21, -16, -41, -26, -9, 11, 49, 63, 17, -44, -39, 13, 37, 17, 24, 43, -11, -69, -58, -6, 37, 39, 10, -38, -51, -20, -8, 18, 62, 64, -9, -71, -45, 33, 61, 27, 21, 32, -17, -77, -57, 17, 60, 45, -11, -69, -58, -8, 0, 19, 63, 53, -20, -83, -46, 45, 75, 34, 17, 17, -22, -71, -52, 19, 65, 48, -30, -88, -57, 0, 12, 28, 49, 36, -24, -81, -32, 61, 82, 39, 2, -7, -19, -42, -48, 5, 65, 52, -28, -88, -61, 5, 25, 27, 24, 12, -19, -45, -19, 30, 42, 30, 27, 14, -27, -57, -26, 37, 51, 18, -28, -61, -32, 3, 6, 7, 21, 15, -20, -42, -4, 41, 28, 6, -2, -2, 4, -2, -25, -6, 34, 24, -20, -43, -16, 10, 3, -3, -10, -15, -2, 12, 6, -11, -12, 5, 22, 13, -7, -15, 0, 22, 5, -16, -3, 8, -2, -11, -12, -12, -9, -1, 10, 19, 2, -29, -27, 5, 16, 5, 3, 8, 4, -5, -11, -4, 15, 21, 4, -6, -8, -16, -28, -12, 21, 30, 0, -32, -28, -15, -8, 4, 26, 30, 5, -20, -24, -6, 26, 39, 26, -1, -28, -34, -28, -6, 25, 34, 7, -30, -39, -34, -23, 15, 54, 46, 0, -41, -37, -2, 38, 53, 39, 1, -38, -58, -37, 2, 37, 34, 0, -28, -39, -41, -27, 14, 66, 64, -6, -54, -47, -7, 39, 64, 47, 5, -44, -61, -34, 0, 34, 38, 1, -39, -46, -43, -19, 27, 75, 57, -10, -57, -53, -5, 47, 61, 42, 0, -42, -49, -34, -1, 42, 54, 0, -52, -59, -44, -8, 45, 79, 50, -18, -59, -46, 2, 52, 68, 34, -10, -44, -58, -28, 14, 53, 48, -8, -59, -62, -29, 8, 47, 69, 40, -26, -53, -39, 7, 59, 63, 16, -36, -54, -50, -23, 28, 67, 48, -15, -65, -47, -9, 20, 44, 51, 19, -28, -51, -42, 9, 68, 61, 5, -48, -56, -38, -8, 35, 60, 30, -27, -61, -42, 5, 39, 50, 42, 6, -30, -47, -30, 17, 53, 43, -17, -53, -46, -28, 0, 42, 65, 29, -35, -66, -37, 18, 45, 58, 41, -8, -47, -50, -15, 30, 51, 22, -36, -61, -41, -16, 18, 53, 59, 17, -39, -56, -15, 25, 43, 46, 27, -16, -53, -45, -5, 35, 39, 11, -36, -51, -30, -14, 13, 53, 62, 9, -44, -47, -13, 21, 40, 43, 26, -17, -53, -38, 3, 31, 29, 8, -28, -39, -29, -24, 3, 53, 60, 17, -35, -44, -14, 16, 42, 45, 26, -18, -48, -40, -2, 25, 21, 6, -14, -28, -27, -26, -4, 41, 60, 29, -21, -43, -23, 5, 36, 51, 30, -9, -42, -45, -12, 22, 24, 12, -8, -26, -31, -30, -9, 35, 59, 32, -19, -43, -31, 5, 35, 49, 34, -1, -36, -47, -17, 17, 28, 22, -7, -31, -32, -31, -15, 27, 56, 37, -12, -44, -34, 5, 37, 48, 37, 3, -31, -49, -25, 9, 31, 26, -3, -33, -36, -32, -12, 29, 56, 42, -12, -45, -40, -1, 36, 49, 40, 8, -28, -46, -26, 3, 32, 36, 5, -33, -45, -41, -14, 30, 58, 41, -9, -46, -42, -5, 42, 57, 44, 11, -29, -47, -23, 2, 27, 40, 10, -35, -57, -49, -14, 38, 62, 45, -4, -49, -44, -2, 43, 65, 45, 1, -34, -45, -22, 0, 26, 44, 16, -31, -58, -51, -11, 38, 62, 45, -10, -46, -44, -8, 41, 66, 42, 0, -36, -43, -14, 4, 27, 45, 20, -31, -63, -51, -5, 42, 62, 38, -10, -47, -46, -13, 46, 65, 38, 0, -30, -35, -20, -6, 27, 48, 14, -35, -59, -43, 0, 44, 55, 35, -8, -43, -44, -9, 43, 61, 35, -3, -30, -35, -25, -7, 33, 49, 19, -32, -58, -39, 9, 44, 51, 31, -8, -42, -47, -11, 40, 54, 31, 2, -27, -38, -29, 0, 33, 46, 18, -30, -52, -28, 11, 41, 49, 29, -6, -43, -50, -10, 45, 50, 24, -6, -30, -39, -22, 2, 32, 43, 12, -33, -49, -24, 12, 39, 47, 30, -5, -40, -48, -12, 39, 48, 24, -9, -36, -38, -20, 4, 30, 36, 8, -29, -43, -18, 16, 35, 42, 31, 3, -37, -44, -7, 33, 40, 18, -10, -35, -40, -24, 3, 29, 32, 11, -22, -38, -17, 9, 30, 43, 37, 7, -28, -39, -8, 23, 31, 19, -4, -31, -43, -28, 0, 25, 30, 17, -16, -36, -24, -3, 23, 47, 46, 15, -25, -35, -13, 14, 26, 19, 1, -26, -45, -37, -7, 23, 28, 20, -1, -29, -32, -16, 15, 53, 56, 25, -20, -38, -24, 0, 23, 30, 7, -23, -47, -45, -13, 23, 28, 25, 7, -31, -42, -25, 15, 66, 73, 18, -25, -38, -28, -6, 25, 35, 15, -13, -49, -60, -21, 19, 35, 44, 23, -26, -58, -43, 14, 84, 83, 26, -27, -51, -46, -6, 37, 48, 27, -12, -56, -73, -32, 16, 44, 57, 25, -40, -70, -37, 28, 84, 84, 40, -13, -69, -79, -23, 59, 77, 29, -21, -61, -78, -46, 11, 61, 77, 23, -57, -90, -40, 39, 88, 89, 46, -21, -90, -101, -21, 88, 91, 37, -23, -73, -89, -51, 14, 83, 110, 19, -88, -113, -38, 48, 87, 84, 48, -28, -110, -101, 3, 102, 104, 46, -22, -83, -101, -54, 25, 100, 108, 12, -94, -110, -29, 47, 83, 92, 50, -43, -127, -95, 19, 109, 100, 36, -35, -95, -106, -40, 49, 115, 105, -1, -97, -93, -28, 41, 86, 95, 23, -75, -120, -68, 34, 94, 82, 37, -29, -97, -98, -24, 56, 104, 83, -21, -89, -78, -26, 31, 84, 80, 8, -76, -103, -31, 47, 76, 66, 35, -36, -97, -86, -13, 53, 88, 56, -23, -72, -59, -17, 30, 76, 66, -3, -73, -81, -21, 39, 50, 48, 30, -35, -86, -65, 1, 56, 68, 40, -18, -53, -47, -23, 24, 65, 54, -19, -69, -51, 1, 23, 30, 44, 29, -34, -77, -36, 23, 46, 40, 23, -8, -36, -36, -18, 20, 49, 34, -22, -47, -26, 3, 3, 13, 36, 21, -42, -57]; 
Y = abs(fft(roll_dot_measurement, 2048));
Fs = 1e3;
f = 0:1024;
plot(f*Fs/2048, Y(1:1025))
%%
% Graficando con las ecuaciones
clear all, close all, clc

X0 = [0 0 0 0 0 0 0 0 0 0 0 0];
tspan = [0:0.001:6];
K  = 0;
[t, y] = ode45(@(t,X)nonlinear_function(X, X0, K, 0),tspan ,X0);
mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
for i=1:12   
    subplot (4,3,i);
    plot (t,y(:,i));
    title (mystr(i), 'interpreter' , 'latex');
    axis 'auto y';
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculando la matriz con la ecuaciones
clear all, close all, clc
syms b1 b2 b3 b4 Ixx Iyy Izz ut ux uy U1 U2 U3 U4 a1 a2 a3 m g l X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12
b1 = l/Ixx;
b2 = l/Iyy;
b3 = l/Izz;

ut = cos(X1)*cos(X3);
ux = cos(X1)*sin(X3)*cos(X5) + sin(X1)*sin(X5);
uy = cos(X1)*sin(X3)*sin(X5) - sin(X1)*cos(X5);

a1 = (Iyy - Izz)/Ixx;
a2 = (Izz - Ixx)/Iyy;
a3 = (Ixx- Iyy)/Izz;

f(1) = X2;
f(2) = X4*X6*a1 + b1*U2;
f(3) = X4;
f(4) = X2*X6*a2+b2*U3;
f(5) = X6;
f(6) = X2*X4*a3+b3*U4;
f(7) = X8;
f(8) = -g + ut*(1/m)*U1;
f(9) = X10;
f(10) = ux*(1/m)*U1;
f(11) = X12;
f(12) = uy*(1/m)*U1;

A = sym (zeros (12,12));
B = sym (zeros (12,4));

for i=1:12
   A(i,:) = gradient (f(i), [X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12]).';
end 
for i=1:12
   B(i,:) = gradient (f(i), [U1 U2 U3 U4]).';
end 
   
% m = 0.506;
% g = 9.8;
% l = 0.235;
% Ixx = 8.12e-5;
% Iyy = 8.12e-5;
% Izz = 6.12e-5;
U1=-m*g;
U2=0;
U3=0;
U4=0;
X1=0; X2=0; X3=0; X4=0; X5=pi; X6=0;X7=0; X8=0; X9=0; X10=0; X11=0; X12=0;


A = double(subs(A))
B = double(subs(B))
C =[1 0 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 0 1;]
D = zeros(size(C,1),size(B,2))

ctrl_obs(A,B,C);

eig(A)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% close loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = [-1 -2 -3 -4 -5 -6 -7 -8 -9 -10 -11 -12];

K = place (A, B, p)

X0 = [10*pi/180 0 0 0 0 0 0 0 0 0 0 0].';
SetPoint = [5*pi/180 0 0 0 0 0 0 0 0 0 0 0].';
tspan = [0:0.001:6];
[t, y] = ode45(@(t,X)nonlinear_function(X, SetPoint, K, 1), tspan, X0);

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
for i=1:12   
    subplot (4,3,i);
    plot (t,y(:,i));
    title (mystr(i), 'interpreter' , 'latex');
    axis 'auto y';
end
%% LQR
[A, B, C, D] = getSystem();
Q = eye(12);
Q(1, 1) = 10;
Q(2, 2) = 10;
Q(3, 3) = 60;
Q(4, 4) = 60;
Q(5, 5) = 10;
Q(6, 6) = 10;
Q(7, 7) = 50;
R = 10*eye(4);
K = lqr(A, B, Q, R);
X0 = [0.8 0 0 0 0 0 0 0 0 0 0 0].';
SetPoint = [0 0 0 0 0 0 0 0 0 0 5 0].';
tspan = [0:0.01:7];

[t, y] = ode45(@(t,X)nonlinear_function(X, SetPoint, K, 1), tspan, X0);

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
yLimits = [-15 15; -15 15; -15 15; -15 15; -15 15; -15 15; -10 10; -10 10; -5 5; -5 5; -20 20; -5 5];
units = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 1 1 1 1 1 1];

for i=1:12   
    subplot (4,3,i);
    plot (t,units(i)*y(:,i));
    ylim(yLimits(i, :));
    grid on
    title (mystr(i), 'interpreter' , 'latex');
end
%% Discretizado
[A, B, C, D] = getSystem();
Q = eye(12);
Q(1, 1) = 10;
Q(2, 2) = 10;
Q(3, 3) = 60;
Q(4, 4) = 60;
Q(5, 5) = 10;
Q(6, 6) = 10;
Q(7, 7) = 50;
R = 10*eye(4);
K = lqr(A, B, Q, R);

X0 = [0.8 0 0 0 0 0 0 0 0 0 0 0].';
SetPoint = [0 0 0 0 0 0 0 0 0 0 0 0].';

Tstep = .1e-3;
Ts = 20e-3;
TotalTime = 10;
TotalRefreshes = TotalTime/Ts;  % Esto siempre debe ser entero
pointsPerRefresh = Ts/Tstep;
tspan = 0:Tstep:(10-Tstep);
y = zeros(length(tspan), 12);
t = zeros(length(tspan), 1);
IC = zeros(12, TotalRefreshes);
IC(:, 1) = X0; 


[Kd,S,e] = lqrd(A,B,Q,R,Ts);

for i = 1:TotalRefreshes
    if i == 1
        [taux, yaux] = ode45(@(t,X)nonlinear_discrete_function(X, SetPoint, Kd, 1, IC(:, i)),... 
        tspan((i-1)*pointsPerRefresh + 1 : i*pointsPerRefresh), IC(:, i)); 
    
        t((i-1)*pointsPerRefresh + 1 : i*pointsPerRefresh, 1) = taux;
        y((i-1)*pointsPerRefresh + 1 : i*pointsPerRefresh, :) = yaux;
    else
        [taux2, yaux2] = ode45(@(t,X)nonlinear_discrete_function(X, SetPoint, Kd, 1, IC(:, i)),... 
        tspan((i-1)*pointsPerRefresh : i*pointsPerRefresh), IC(:, i)); 
    
        t((i-1)*pointsPerRefresh + 1 : i*pointsPerRefresh, 1) = taux2(2:end, 1);
        y((i-1)*pointsPerRefresh + 1 : i*pointsPerRefresh, :) = yaux2(2:end, :);    
    end
    if i < TotalRefreshes
        IC(:, i+1) = y(i*pointsPerRefresh, :).';
    end
end

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
yLimits = [-15 15; -15 15; -15 15; -15 15; -15 15; -15 15; -10 10; -10 10; -5 5; -5 5; -20 20; -5 5];
units = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 1 1 1 1 1 1];

for i=1:12   
    subplot (4,3,i);
    plot (t,units(i)*y(:,i));
    ylim(yLimits(i, :));
    grid on
    title (mystr(i), 'interpreter' , 'latex');
end

U = zeros(4, 100000);
for i = 1:100000
    U(:, i) = -K*(y(i, :).'-SetPoint);
end
yLimits = [-1 1; -1 1; -1 1; -1 1];
for i=1:4   
    subplot (4,1,i);
    plot (t, U(i, :));
    %ylim(yLimits(i, :));
    grid on
end

%% Sistema lineal
[A, B, C, D] = getSystem();
Q = eye(8);
Q(1, 1) = 10;
Q(2, 2) = 10;
Q(3, 3) = 10;
Q(4, 4) = 10;
Q(5, 5) = 10;
Q(6, 6) = 10;
Q(7, 7) = 10;
Q(8, 8) = 10;
R = eye(4);
R(1, 1) = 10;
R(2, 2) = 1000000;
R(3, 3) = 1000000;
R(4, 4) = 1000000;
A = A(1:8, 1:8);
B = B(1:8, :);

K = lqr(A, B, Q, R);

X0 = [0.5 0 0 0 0 0 -10 0 0 0 0 0].';
SetPoint = [0 0 0 0 0 0 0 0].';
tspan = [0:0.001:10];

[A, B, C, D] = getSystem();
[t, y] = ode45(@(t,X)linearSystem(X, SetPoint, K, A, B), tspan, X0);

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
yLimits = [-45 45; -45 45; -45 45; -45 45; -45 45; -45 45; -10 10; -10 10; -5 5; -5 5; -20 20; -5 5];
units = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 1 1 1 1 1 1];

for i=1:12   
    subplot (4,3,i);
    plot (t,units(i)*y(:,i));
    ylim(yLimits(i, :));
    grid on
    title (mystr(i), 'interpreter' , 'latex');
end


hold on
U = zeros(4, length(tspan));
for i = 1:length(tspan)
    U(:, i) = -K*(y(i, 1:8).'-SetPoint);
end
yLimits = [-1 1; -1 1; -1 1; -1 1];
for i=1:4   
    subplot (4,1,i);
    plot (t, U(i, :));
    %ylim(yLimits(i, :));
    grid on
end
%% Solo estados angulosos (continuous)
% LQR
[A, B, C, D] = getSystem();
Q = eye(8);
Q(1, 1) = 1000;
Q(2, 2) = 10;
Q(3, 3) = 1000;
Q(4, 4) = 10;
Q(5, 5) = 1000;
Q(6, 6) = 10;
Q(7, 7) = 1000;
Q(8, 8) = 10;
R = eye(4);
R(1, 1) = 10;
R(2, 2) = 1000000;
R(3, 3) = 1000000;
R(4, 4) = 1000000;
A = A(1:8, 1:8);
B = B(1:8, :);

K = lqr(A, B, Q, R);

X0 = [0.5 0 0 0 0 0 -10 0 0 0 0 0].';
SetPoint = [0 0 0 0 0 0 0 0].';
tspan = [0:0.001:10];

[t, y] = ode45(@(t,X)nonlinear_function_angularStates(X, SetPoint, K, 1), tspan, X0);

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
yLimits = [-45 45; -45 45; -45 45; -45 45; -45 45; -45 45; -10 10; -10 10; -5 5; -5 5; -20 20; -5 5];
units = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 1 1 1 1 1 1];

for i=1:12   
    subplot (4,3,i);
    plot (t,units(i)*y(:,i));
    ylim(yLimits(i, :));
    grid on
    title (mystr(i), 'interpreter' , 'latex');
end


hold on
U = zeros(4, length(tspan));
for i = 1:length(tspan)
    U(:, i) = -K*(y(i, 1:8).'-SetPoint);
end
yLimits = [-1 1; -1 1; -1 1; -1 1];
for i=1:4   
    subplot (4,1,i);
    plot (t, U(i, :));
    %ylim(yLimits(i, :));
    grid on
end
% SISTEMA LINEAL PURO


% b = 1;
% X = [
%   (U(4, :) + U(1, :)*b - 2*U(3, :)*b)/(4*b); ...
%   (U(1, :)*b - U(4, :) + 2*U(2, :)*b)/(4*b);...
%   (U(4, :) + U(1, :)*b + 2*U(3, :)*b)/(4*b);...
%  -(U(4, :) - U(1, :)*b + 2*U(2, :)*b)/(4*b) ];
% for i=1:4   
%     subplot (4,1,i);
%     plot (t, X(i, :));
%     %ylim(yLimits(i, :));
%     grid on
% end
%% Solo estados angulosos y Z(continuous) y ademas con INTEGRADOR + DELAY
% LQR
DEG2RAD = pi/180
[A, B, C, D] = getSystem();

A = A(1:8, 1:8);
B = B(1:8, :);
C = [1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 ];
 
Aaug = [ A   zeros(8, 2);
        -C   zeros(2, 2)];
Baug = [      B      ;
         zeros(2, 4)];
     
Q = eye(10);
Q(1, 1) = 17;
Q(2, 2) = 10;
Q(3, 3) = 10;
Q(4, 4) = 10;
Q(5, 5) = 1;
Q(6, 6) = 1;
Q(7, 7) = 100;
Q(8, 8) = 100;
Q(9, 9) = 88;
Q(10, 10) = 1;
R = eye(4);
R(1, 1) = 1;
R(2, 2) = 1;
R(3, 3) = 1;
R(4, 4) = 1;
K = lqr(Aaug, Baug, Q, R);

X0 = [30*DEG2RAD 0 15*DEG2RAD 0 0 0 0 0 0 0 0 0 0 0 0 0].';
SetPointESTADOS = [10*DEG2RAD 0 0 0 0 0 0 0].';
r = [0 0].';
tspan = [0:0.001:10];

[t, y] = ode45(@(t,X)nonlinear_function_angularStates_Integrators_WITH_Z_CONTROL(X, SetPointESTADOS, K, 1, C, r), tspan, X0);

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
yLimits = [-45 45; -45 45; -45 45; -45 45; -45 45; -45 45; -10 10; -10 10; -5 5; -5 5; -20 20; -5 5];
units = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 1 1 1 1 1 1];

for i=1:12   
    subplot (4,3,i);
    plot (t,units(i)*y(:,i));
    ylim(yLimits(i, :));
    grid on
    title (mystr(i), 'interpreter' , 'latex');
end


hold on
U = zeros(4, length(tspan));
for i = 1:length(tspan)
    U(:, i) = -K*([(y(i, 1:8).' - SetPointESTADOS) ; y(i, 13:14).']);
end
yLimits = [-1 1; -1 1; -1 1; -1 1];
for i=1:4   
    subplot (4,1,i);
    plot (t, U(i, :));
    %ylim(yLimits(i, :));
    grid on
end
% SISTEMA LINEAL PURO


% b = 1;
% X = [
%   (U(4, :) + U(1, :)*b - 2*U(3, :)*b)/(4*b); ...
%   (U(1, :)*b - U(4, :) + 2*U(2, :)*b)/(4*b);...
%   (U(4, :) + U(1, :)*b + 2*U(3, :)*b)/(4*b);...
%  -(U(4, :) - U(1, :)*b + 2*U(2, :)*b)/(4*b) ];
% for i=1:4   
%     subplot (4,1,i);
%     plot (t, X(i, :));
%     %ylim(yLimits(i, :));
%     grid on
% end

%%
syms T1 T2 T3 T4 b U1 U2 U3 U4 c
eqn1 = T1 + T2 + T3 + T4 == U1;
eqn2 = T2 - T4 == U2;
eqn3 = T3 - T1 == U3;
eqn4 = c*T1 + c*T3 - c*T2 - c*T4 == U4;
[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4], [T1, T2, T3, T4])
X = linsolve(A,B)
%% Solo estados angulosos, Integrador y DELAY y MISMATCH
% LQR + I
DEG2RAD = pi/180;
[A, B, C, D] = getSystem();

A = A(1:6, 1:6);
B = B(1:6, :);
C = [1 0 0 0 0 0 ;
     0 0 1 0 0 0 ];
 
Aaug = [ A   zeros(6, 2);
         C   zeros(2, 2)];
Baug = [      B      ;
         zeros(2, 4)];
     
Q = eye(8);
Q(1, 1) = 5000;
Q(2, 2) = 10;
Q(3, 3) = 5000;
Q(4, 4) = 10;
Q(5, 5) = 10;
Q(6, 6) = 10;
Q(7, 7) = 400;
Q(8, 8) = 400;
R = eye(4);
R(1, 1) = 1000;
R(2, 2) = 1000;
R(3, 3) = 1000;
R(4, 4) = 1000;
K = lqr(Aaug, Baug, Q, R);

X0 = [20*DEG2RAD 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0].';
SetPointESTADOS = [0 0 0 0 0 0].';
r = [SetPointESTADOS(1) SetPointESTADOS(3)].';
tspan = 0:0.01:20;

[t, y] = ode45(@(t,X)nonlinear_function_angularStates_Integrators(X, SetPointESTADOS, K, 1, C, r), tspan, X0);

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
yLimits = [-45 45; -45 45; -45 45; -45 45; -45 45; -45 45; -10 10; -10 10; -5 5; -5 5; -20 20; -5 5];
units = [180/pi 180/pi 180/pi 180/pi 180/pi 180/pi 1 1 1 1 1 1];

for i=1:12   
    subplot (4,3,i);
    plot (t,units(i)*y(:,i));
    ylim(yLimits(i, :));
    grid on
    title (mystr(i), 'interpreter' , 'latex');
end

% Ploteo del error integral
hold off
mystr = ["Error en $\phi$", "Error en $\theta$"];
for i=13:14 
    subplot (2,1,i-12);
    plot (t,y(:,i));
    grid on
    title (mystr(i - 12), 'interpreter' , 'latex');
end
Ts = 1e-3;
[Kd,S,e] = lqrd(Aaug, Baug, Q, R, Ts);

Kx = Kd(:, 1:6);
Ki = Kd(:, 7:8);

printMatrixAsCcode(Kx);
printMatrixAsCcode(Ki);

hold on
U = zeros(4, length(tspan));
for i = 1:length(tspan)
    U(:, i) = -K*([(y(i, 1:6).' - SetPointESTADOS) ; y(i, 13:14).']);
end
yLimits = [-1 1; -1 1; -1 1; -1 1];
for i=1:4   
    subplot (4,1,i);
    plot (t, U(i, :));
    %ylim(yLimits(i, :));
    grid on
end


c = 10;
X = [
  -(U(4, :) - U(1, :)*c + 2*U(3, :)*c)/(4*c); ...
   (U(4, :) + U(1, :)*c - 2*U(2, :)*c)/(4*c);...
   (U(1, :)*c - U(4, :) + 2*U(3, :)*c)/(4*c);...
   (U(4, :) + U(1, :)*c + 2*U(2, :)*c)/(4*c) ];
for i=1:4   
    subplot (4,1,i);
    plot (t, X(i, :));
    %ylim(yLimits(i, :));
    grid on
end


%% Least Squares curva motor

y = [0.05
0.1
0.15
0.2
0.25
0.3
0.35
0.4
0.45
0.5
0.55
0.6
0.65
0.7
0.75
0.8
0.85
0.9
0.95
1 ].';
x = [0.005
0.01
0.025
0.045
0.07
0.095
0.13
0.16
0.2
0.24
0.28
0.325
0.37
0.41
0.46
0.51
0.56
0.62
0.66
0.72].';


% M3 = [5.435   4.635   3.835   3.035   2.325   1.435   0.635];
% PWM = [1.00     1.28     1.70     2.20     2.97     4.35     7.50  ];
% p = polyfit(x, y, 2);                                               % Quadratic Function Fit
% v = polyval(p, x);                                                  % Evaluate
% TSE = sum((v - y).^2);                                              % Total Squared Error
% figure(1)
% plot(x, y, 'bp')
% hold on
% plot(x, v, '-r')
% grid

PWM = [0.05
0.1
0.15
0.2
0.25
0.3
0.35
0.4
0.45
0.5
0.55
0.6
0.65
0.7
0.75
0.8
0.85
0.9
0.95
1].';

Forza = 10*[0
0.005
0.015
0.025
0.035
0.05
0.065
0.085
0.105
0.125
0.145
0.175
0.205
0.235
0.265
0.3
0.33
0.365
0.39
0.425].';

p = polyfit(Forza, PWM, 2);                                         % Quadratic Function Fit
v = polyval(p, Forza);                                                  % Evaluate
TSE = sum((v - PWM).^2);                                              % Total Squared Error
figure(1)
plot(Forza, PWM, 'bp')
hold on
plot(Forza, v, '-r')
hold on
clear s;
pwm = @(s) p(1)*s.^2 + p(2)*s + p(3);
s = -1:0.01:10;
plot(s, pwm(s));
grid on
%% Solver 
syms F1 F2 F3 F4 b U1 U2 U3 U4 c
eqn1 = F1 + F2 + F3 + F4 == U1;
eqn2 = F3 - F1 == U2;
eqn3 = F4 - F2 == U3; 
eqn4 = +c*F1 + c*F3 - c*F2 - c*F4 == U4;
[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4], [F1, F2, F3, F4])
X = linsolve(A,B)

%% Test para comparar con el codigo en C 
Ts = 0.001;
InputState = [0.1; 0.1; 0.1; 0.1 ;0.1 ;0.1];
Reference = [0; 0.05; 0; 0; 3; 0];
Int = zeros(2, 1);
for i=1:3000
    ErrP = InputState - Reference;
    Ux = Kx * ErrP;
    ErrI = [InputState(1); InputState(3)] - [Reference(1); Reference(3)];
    Int = Int + ErrI * Ts;
    U = -Ux - Ki*Int;
end

%% Curvas de Motor parametricas
syms a b
cantCurvas = 12;
x = zeros(2, cantCurvas);
for i = 1:cantCurvas
    
    eqn1 = a/4 + b/2 == 1/8*(0.5 + 0.1*i);
    eqn2 = a + b ==17/40*(0.5 + 0.1*i);
    [A, B] = equationsToMatrix([eqn1, eqn2], [a, b]);
    X(:, i) = linsolve(A,B);
    
end
f = @(a, b, x) a*x.^2 + b*x ;
t = 0:0.01:1;
X = double(X);
for i = 1:cantCurvas
    plot(9.8 * f(X(1, i), X(2, i), t), t);
    hold on 
    grid on
end


%% En base a las curvas anteriores vamos a
%% Funciones:

% Verifica controlabilidad y observabilidad.
function ctrl_obs(A,B,C)
    Cm=ctrb(A,B);                      
    disp('El sistema tiene');
    display( length(A) - rank(Cm));
    disp('estados no controlables')
    Ob = obsv(A,C);
    disp('El sistema tiene ');
    disp(length(A) - rank(Ob));
    disp('estados no observables');
end
function [A, B, C, D] = getSystem()
    syms b1 b2 b3 b4 Ixx Iyy Izz ut ux uy U1 U2 U3 U4 a1 a2 a3 m g l X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12
    b1 = l/Ixx;
    b2 = l/Iyy;
    b3 = l/Izz;

    ut = cos(X1)*cos(X3);
    ux = cos(X1)*sin(X3)*cos(X5) + sin(X1)*sin(X5);
    uy = cos(X1)*sin(X3)*sin(X5) - sin(X1)*cos(X5);

    a1 = (Iyy - Izz)/Ixx;
    a2 = (Izz - Ixx)/Iyy;
    a3 = (Ixx - Iyy)/Izz;

    f(1) = X2;
    f(2) = X4*X6*a1 + b1*U2;
    f(3) = X4;
    f(4) = X2*X6*a2+b2*U3;
    f(5) = X6;
    f(6) = X2*X4*a3+b3*U4;
    f(7) = X8;
    f(8) = -g + ut*(1/m)*U1;
    f(9) = X10;
    f(10) = ux*(1/m)*U1;
    f(11) = X12;
    f(12) = uy*(1/m)*U1;

    A = sym (zeros (12,12));
    B = sym (zeros (12,4));

    for i=1:12
       A(i,:) = gradient (f(i), [X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12]).';
    end 
    for i=1:12
       B(i,:) = gradient (f(i), [U1 U2 U3 U4]).';
    end 
    Ixx = 7.5e-3;
    Iyy = 7.5e-3;
    Izz = 1.3e-3;
    g = 9.8;
    m = 0.8;
    l = 0.235;

%     Ixx = 8.12e-5;
%     Iyy = 8.12e-5;
%     Izz = 6.12e-5;
%     g = 9.8;
%     m = 0.5;
%     l = 0.235;
    U1= +m*g;
    U2=0;
    U3=0;
    U4=0;
    X1=0; X2=0; X3=0; X4=0; X5=0; X6=0;X7=0; X8=0; X9=0; X10=0; X11=0; X12=0;

    A = double(subs(A));
    B = double(subs(B));
    C =[1 0 0 0 0 0 0 0 0 0 0 0;
        0 0 1 0 0 0 0 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0 0 0 0;
        0 0 0 0 0 0 1 0 0 0 0 0];
    D = zeros(size(C,1),size(B,2));
end 
function f = nonlinear_function(X, SetPoint, K, ccl) %ccl=1 if close loop
    Ixx = 8.12e-5;
    Iyy = 8.12e-5;
    Izz = 6.12e-5;
    g = 9.8;
    m = 0.5;
    l = 0.235;
    b1 = l/Ixx;
    b2 = l/Iyy;
    b3 = l/Izz;
    
    ut = cos(X(1))*cos(X(3));
    ux = cos(X(1))*sin(X(3))*cos(X(5)) + sin(X(1))*sin(X(5));
    uy = cos(X(1))*sin(X(3))*sin(X(5)) - sin(X(1))*cos(X(5));

    a1 = (Iyy - Izz)/Ixx;
    a2 = (Izz - Ixx)/Iyy;
    a3 = (Ixx - Iyy)/Izz;
    if ccl == 1
        U = -K*(X-SetPoint);
    else
        U = [4.9 0 0 0];
    end
    
    f(1,1) = X(2);
    f(2,1) = X(4)*X(6)*a1 + b1*U(2);
    f(3,1) = X(4);
    f(4,1) = X(2)*X(6)*a2+b2*U(3);
    f(5,1) = X(6);
    f(6,1) = X(2)*X(4)*a3+b3*U(4);
    f(7,1) = X(8);
    f(8,1) = -g + ut*(1/m)*U(1);
    f(9,1) = X(10);
    f(10,1) = ux*(1/m)*U(1);
    f(11,1) = X(12);
    f(12,1) = uy*(1/m)*U(1);
end
function f = nonlinear_function_angularStates(X, SetPoint, K, ccl) %ccl=1 if close loop
    Ixx = 8.12e-5;
    Iyy = 8.12e-5;
    Izz = 6.12e-5;
    g = 9.8;
    m = 0.5;
    l = 0.235;
    b1 = l/Ixx;
    b2 = l/Iyy;
    b3 = l/Izz;
    
    ut = cos(X(1))*cos(X(3));
    ux = cos(X(1))*sin(X(3))*cos(X(5)) + sin(X(1))*sin(X(5));
    uy = cos(X(1))*sin(X(3))*sin(X(5)) - sin(X(1))*cos(X(5));

    a1 = (Iyy - Izz)/Ixx;
    a2 = (Izz - Ixx)/Iyy;
    a3 = (Ixx - Iyy)/Izz;
    if ccl == 1
        U = -K*(X(1:8)-SetPoint);
    else
        U = [4.9 0 0 0];
    end
    f(1,1) = X(2);
    f(2,1) = X(4)*X(6)*a1 + b1*(U(2) + 10e-3);
    f(3,1) = X(4);
    f(4,1) = X(2)*X(6)*a2+b2*(U(3));
    f(5,1) = X(6);
    f(6,1) = X(2)*X(4)*a3+b3*U(4);
    f(7,1) = X(8);
    f(8,1) = -g + ut*(1/m)*U(1);
    f(9,1) = X(10);
    f(10,1) = ux*(1/m)*U(1);
    f(11,1) = X(12);
    f(12,1) = uy*(1/m)*U(1);
end
function f = nonlinear_function_angularStates_Integrators(X, SetPointESTADOS, K, ccl, C, r) %ccl=1 if close loop
    Ixx = 7.5e-3;
    Iyy = 7.5e-3;
    Izz = 1.3e-3;
    g = 9.8;
    m = 0.8;
    l = 0.235;
    b1 = l/Ixx;
    b2 = l/Iyy;
    b3 = l/Izz;
    k = 10e-3;
    
    ut = cos(X(1))*cos(X(3));
    ux = cos(X(1))*sin(X(3))*cos(X(5)) + sin(X(1))*sin(X(5));
    uy = cos(X(1))*sin(X(3))*sin(X(5)) - sin(X(1))*cos(X(5));

    a1 = (Iyy - Izz)/Ixx;
    a2 = (Izz - Ixx)/Iyy;
    a3 = (Ixx - Iyy)/Izz;
    if ccl == 1
        U = -K*([(X(1:6) - SetPointESTADOS) ; X(13:14) ]);
    else
        U = [4.9 0 0 0];
    end
    
    F1_MISMATCH = 1.1;
    F2_MISMATCH = 1;
    F3_MISMATCH = 0.9;
    F4_MISMATCH = 1.05;
    
  % U(1) = 9;

    c = 10;
    U(1) = 9;
    
    F1 = (U(4) + U(1)*c - 2*U(2)*c)/(4*c) * F1_MISMATCH;
    F2 = -(U(4) - U(1)*c + 2*U(3)*c)/(4*c) * F2_MISMATCH;
    F3 = (U(4) + U(1)*c + 2*U(2)*c)/(4*c) * F3_MISMATCH;
    F4 = (U(1)*c - U(4) + 2*U(3)*c)/(4*c) * F4_MISMATCH;     
    
    %% ACA va la K en las F
  
    
    U(1) = F1 + F2 + F3 + F4;
    U(2) = F3 - F1;
    U(3) = F4 - F2;
    U(4) = +c*F1 + c*F3 - c*F2 - c*F4;

%     c = 10;
%     F1 = -(U(4) - U(1)*c + 2*U(3)*c)/(4*c) * F1_MISMATCH;
%     F2 = (U(4) + U(1)*c - 2*U(2)*c)/(4*c) * F2_MISMATCH;
%     F3 = (U(1)*c - U(4) + 2*U(3)*c)/(4*c) * F3_MISMATCH;
%     F4 = (U(4) + U(1)*c + 2*U(2)*c)/(4*c) * F4_MISMATCH;
%     
%     U(1) = F1 + F2 + F3 + F4;
%     U(2) = F4 - F2;
%     U(1) = F3 - F1;
%     U(4) = -c*F1 - c*F3 + c*F2 + c*F4;
    
    f(1,1) = X(2);
    f(2,1) = X(4)*X(6)*a1 + b1*(X(15));
    f(3,1) = X(4);
    f(4,1) = X(2)*X(6)*a2+b2*(X(16));
    f(5,1) = X(6);
    f(6,1) = X(2)*X(4)*a3+b3*U(4);
    f(7,1) = X(8);
    f(8,1) = -g + ut*(1/m)*U(1);
    f(9,1) = X(10);
    f(10,1) = ux*(1/m)*U(1);
    f(11,1) = X(12);
    f(12,1) = uy*(1/m)*U(1);
    f(13:14,1) = C*X(1:6) - r;     % +2 variables por el integrador
    f(15,1) = (X(15) - U(2))/(-k); % +1 variable por el delay en phi 
    f(16,1) = (X(16) - U(3))/(-k); % +1 variable por el delay en theta
end
function f = nonlinear_function_angularStates_Integrators_WITH_Z_CONTROL(X, SetPointESTADOS, K, ccl, C, r) %ccl=1 if close loop
    Ixx = 7.5e-3;
    Iyy = 7.5e-3;
    Izz = 1.3e-3;
    g = 9.81;
    m = 0.8;
    l = 0.235;
    b1 = l/Ixx;
    b2 = l/Iyy;
    b3 = l/Izz;
    k = 100e-3;
    
    ut = cos(X(1))*cos(X(3));
    ux = cos(X(1))*sin(X(3))*cos(X(5)) + sin(X(1))*sin(X(5));
    uy = cos(X(1))*sin(X(3))*sin(X(5)) - sin(X(1))*cos(X(5));

    a1 = (Iyy - Izz)/Ixx;
    a2 = (Izz - Ixx)/Iyy;
    a3 = (Ixx - Iyy)/Izz;
    if ccl == 1
        U = -K*([(X(1:8) - SetPointESTADOS) ; X(13:14)]);
    else
        U = [4.9 0 0 0];
    end
    f(1,1) = X(2);
    f(2,1) = X(4)*X(6)*a1 + b1*(X(15));
    f(3,1) = X(4);
    f(4,1) = X(2)*X(6)*a2+b2*(X(16));
    f(5,1) = X(6);
    f(6,1) = X(2)*X(4)*a3+b3*U(4);
    f(7,1) = X(8);
    f(8,1) = -g + ut*(1/m)*U(1);
    f(9,1) = X(10);
    f(10,1) = ux*(1/m)*U(1);
    f(11,1) = X(12);
    f(12,1) = uy*(1/m)*U(1);
    f(13:14,1) = -C*X(1:8) + r;
    f(15,1) = (X(15) - U(2))/(-k);
    f(16,1) = (X(16) - U(3))/(-k);
end
function f = nonlinear_discrete_function(X, SetPoint, K, ccl, lastStateX) %ccl=1 if close loop
    Ixx = 7.5e-3;
    Iyy = 7.5e-3;
    Izz = 1.3e-3;
    g = 9.8;
    m = 0.8;
    l = 0.235;

    b1 = l/Ixx;
    b2 = l/Iyy;
    b3 = l/Izz;
    
    ut = cos(X(1))*cos(X(3));
    ux = cos(X(1))*sin(X(3))*cos(X(5)) + sin(X(1))*sin(X(5));
    uy = cos(X(1))*sin(X(3))*sin(X(5)) - sin(X(1))*cos(X(5));

    a1 = (Iyy - Izz)/Ixx;
    a2 = (Izz - Ixx)/Iyy;
    a3 = (Ixx - Iyy)/Izz;
    if ccl == 1
        U = -K*(lastStateX-SetPoint);
    else
        U = [4.9 0 0 0];
    end
    
    f(1,1) = X(2);
    f(2,1) = X(4)*X(6)*a1 + b1*U(2);
    f(3,1) = X(4);
    f(4,1) = X(2)*X(6)*a2+b2*U(3);
    f(5,1) = X(6);
    f(6,1) = X(2)*X(4)*a3+b3*U(4);
    f(7,1) = X(8);
    f(8,1) = -g + ut*(1/m)*U(1);
    f(9,1) = X(10);
    f(10,1) = ux*(1/m)*U(1);
    f(11,1) = X(12);
    f(12,1) = uy*(1/m)*U(1);
end
function f = linearSystem(X, SetPoint, K, A, B)
    u = -K*(X(1:8, 1) - SetPoint); % k es de 4x8
    f = A*X + B*u;
end
%% Matlab matrix to C code
function printMatrixAsCcode(A)
    fprintf("double A[%d][%d] = {\n", size(A, 1), size(A, 2));
    for i = 1:size(A, 1)
        fprintf("\t{");
        for j = 1:size(A, 2)
            fprintf("%.7f", A(i,j));
            if j < size(A, 2)
                fprintf(", ");
            end
        end
        fprintf("}");
        if i < size(A, 1)
            fprintf(",\n");
        else
            fprintf("\n");
        end
    end
    fprintf("};\n");
end