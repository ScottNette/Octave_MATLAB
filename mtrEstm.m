clear
clc


J = 0.001;
b = 0.1;
Kt = 0.01;
R = .1;
L = 0.005;

##X = [theta_d
##     i ];

A = [-b/J   Kt/J
    -Kt/L   -R/L];
B = [0
    1/L];  % voltage
C = [1   0];
D = 0;

sys_ol = ss(A,B,C,D);

%% Close loops system
p1 = -5 + 100i;
p2 = -5 - 100i;


w = 10;
z = 0.51;
p = roots([1 2*z*w w^2]);

Kc = place(A',C',[p(1) p(2)])
sys_cl = ss(A-B*Kc,B,C,D);
Nbar = rscale(sys_ol,Kc)


t = 0:0.01:0.5;
## Observer
op1 = -5000 + 10i;
op2 = -5000 - 10i;

w = 1;
z = 0.51;
op = roots([1 2*z*w w^2]);

L = place(A',C',[op(1) op(2)])';

Ace = [(A-B*Kc)        (B*Kc);
       zeros(size(A)) (A-L*C)];
Bce = [B*Nbar;
       zeros(size(B))];
Cce = [C zeros(size(C))];
Dce = 0; 

states = {'x_dot' 'i'};
inputs = {'r'};
outputs = {'x'; 'phi'};


sys_est_cl = ss(Ace,Bce,Cce,Dce); %,'statename',states,'inputname',inputs,'outputname',outputs);
[Y_ObL, T_ObL, X] = step(sys_est_cl,t);
[Y_CL, T_CL, X] = step(sys_cl*Nbar,t);
[Y_OL, T_OL, X] =step(sys_ol,t);


sys_est_d = c2d(sys_est_cl, 1/1000);
[Y_disc, T_disc, X] = step(sys_est_cl,t);

x0 = zeros(4);
x = x0;
      
A = sys_est_d.a;
B = sys_est_d.b;
C = sys_est_d.c;
D = sys_est_d.d;

u = ones(1,t(end)*1000);
t_disc = 0:1/1000:t(end);
t_disc = t_disc(1:end-1)';

for ii = 1:(t(end)*1000)    
      y(ii,:) = C*x + D*u(ii);
      x = A*x + B*u(ii); 
end


 
figure(4),clf
hold all
plot(t_disc,y(:,1),'-x')
plot(T_disc, Y_disc)
##plot(T_ObL, Y_ObL)
plot(T_CL, Y_CL,'-x')
plot(T_OL, Y_OL,'-o')
grid
title('Step Response with State-Feedback Controller')
##legend(['man disc';'disc'; 'Obs'; 'CL'; 'OL'])
##legend([ 'CL'; 'OL'])





