% Graficando con las ecuaciones
clear all, close all, clc

X0 = [0 0 0 0 0 0 0 0 0 0 0 0];
tspan = [0:0.001:6];
[t, y] = ode45(@(t,X)nonlinear_function(X),tspan ,X0);
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
   
m = 0.506;
g = 9.8;
l = 0.235;
Ixx = 8.12e-5;
Iyy = 8.12e-5;
Izz = 6.12e-5;
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
   0 0 0 0 0 0 1 0 0 0 0 0]
D = zeros(size(C,1),size(B,2))

ctrl_obs(A,B,C);

eig(A)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% close loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = [-1 -2 -3 -4 -5 -6 -7 -8 -9 -10 -11 -12];

K = place (A, B, p)

X0 = [0 0 0.524 0 0 0 0 0 0 0 0 0];
tspan = [0:0.001:6];
[t, y] = ode45(@(t,X)nonlinear_function(X, X0, K, 1),tspan ,X0);

mystr= ["$\Phi$", "$\dot{\Phi}$", "$\Theta$", "$\dot{\Theta}$", "$\Psi$", "$\dot{\Psi}$", "$Z$", "$\dot{Z}$", "$X$", "$\dot{X}$", "$Y$", "$\dot{Y}$"];
for i=1:12   
    subplot (4,3,i);
    plot (t,y(:,i));
    title (mystr(i), 'interpreter' , 'latex');
    axis 'auto y';
end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    LQR    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Q = [1 0 0 0;
    0 1 0 0;
    0 0 10 0;
    0 0 0 100];
R = .0001;

K = lqr(A,B,Q,R)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    LQE    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vd = .1*eye(4);  % disturbance covariance
Vn = 1;       % noise covariance
BF = [B Vd 0*B];  % augment inputs to include disturbance and noise

sysC = ss(A,BF,C,[0 0 0 0 0 Vn]);  % build big state space system... with single output

sysFullOutput = ss(A,BF,eye(4),zeros(4,size(BF,2)))  % system with full state output, disturbance, no noise

[L,P,E] = lqe(A,Vd,C,Vd,Vn)  % design Kalman filter


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% Pole Placement
function[Acl,k] = pol_place(A,B,cp)
    k = place(A,B,cp)
    Acl = A-B*k
end

% Pole Placement con control integral
function[Aacl,Bar,Ca,Da,k] = pol_place_integr(A,B,C,D,cp)
    Aa = [ A zeros(size(A,1), 1) ; -C 0 ];
    Ba = [ B ; 0 ];
    Bar = [zeros(size(B)); 1];
    Ca = [ C 0 ];
    Da = [0];
    k = place(Aa,Ba,cp)
    Aacl = Aa-Ba*k
end

% Convierte de la forma canonica de controlabilidad a variable de fase
function [Avf, Bvf, Cvf] = cc2vf(Acc, Bcc, Ccc)
    Avf = flip(flip(Acc, 1), 2);
    Bvf = flip(Bcc, 1);
    Cvf = flip(Ccc, 2);
end

function f = nonlinear_function(X, X0, K, ccl) %ccl=1 if close loop
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
    a3 = (Ixx- Iyy)/Izz;
    
    if ccl == 1
        U = -K*(X-X0);
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

