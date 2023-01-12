clear all, close all, clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = 9.8;
m = 0.5;
l = 0.235;
Ixx = 8.12e-5;
Iyy = 8.12e-5;
Izz = 6.12e-5;
b1 = l/Ixx;
b2 = l/Iyy;
b3 = l/Izz;



A = [0 1 0 0 0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 1 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 1 0 0;
     0 g 0 0 0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     -g 0 0 0 0 0 0 0 0 0 0 0]
 
 B= [0 0 0 0; 
     0 b1 0 0; 
     0 0 0 0; 
     0 0 b2 0; 
     0 0 0 0; 
     0 0 0 b3; 
     0 0 0 0;
     1/m 0 0 0;
     0 0 0 0; 
     0 0 0 0; 
     0 0 0 0; 
     0 0 0 0]
 
 C =[1 0 0 0 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0 0 0 0;
     0 0 0 0 0 0 1 0 0 0 0 0;]
 
 D = zeros(size(C,1),size(B,2))

X0 = [0 0.001 0 0 0 0 0 0 0 0 0 0]

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%syms f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12 X1 X2 X3 X4 X5 X6 X7 X8 X9 X10 X11 X12 U1 U2 U3 U4 Ixx Iyy Izz
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
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    OBS & CTRL    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[Cm, Om] = ctrl_obs(A,B,C);
sys=ss(A,B,C,D);

initial(sys,X0,30);

A_ = A(1:6,1:6)
B_ = B(1:6,1:end)
C_ = C(1:end,1:6)

[Cm, Om] = ctrl_obs(A_,B_,C_);

sys=ss(A_,B_,C_,D);

ctrl_gram = det(gram(sys,'c'))
obs_gram = det(gram(sys,'o'))

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
function[Cm,Om] = ctrl_obs(A,B,C)
    Cm=ctrb(A,B)                       
    Rank=rank(Cm);
    if (Rank==width(Cm))
        disp('El sistema es controlable');
    else
        disp('El sistema no es controlable');
    end

    Om = obsv(A,C)
    Rank=rank(Om); 
    if (Rank==width(Om))
        disp('El sistema es observable'); 
    else
        disp('El sistema no es observable');
    end
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

function f = nonlinear_function(X)
    Ixx = 8.12e-5;
    Iyy = 8.12e-5;
    Izz = 6.12e-5;
    g = 9.8;
    m = 0.5;
    l = 0.235;
    b1 = l/Ixx;
    b2 = l/Iyy;
    b3 = l/Izz;
    
    ut = cos(X(1))*cos(X(2));
    ux = cos(X(1))*sin(X(3))*cos(X(5)) + sin(X(1))*cos(X(5));
    uy = cos(X(1))*sin(X(3))*sin(X(5)) - sin(X(1))*cos(X(5));

    a1 = (Iyy - Izz)/Ixx;
    a2 = (Izz - Ixx)/Iyy;
    a3 = (Ixx- Iyy)/Izz;
    
    U1 = 4.9;
    U2 = 0;
    U3 = 0;
    U4 = 0;

    f(1,1) = X(2);
    f(2,1) = X(4)*X(6)*a1 + b1*U2;
    f(3,1) = X(4);
    f(4,1) = X(2)*X(6)*a2+b2*U3;
    f(5,1) = X(6);
    f(6,1) = X(2)*X(4)*a3+b3*U4;
    f(7,1) = X(8);
    f(8,1) = -g + ut*(1/m)*U1;
    f(9,1) = X(10);
    f(10,1) = ux*1/3*U1;
    f(11,1) = X(12);
    f(12,1) = uy*1/m*U1;
end