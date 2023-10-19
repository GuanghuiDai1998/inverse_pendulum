%Modeling
mCart = 0.5;
mPend = 0.2;
b = 0.1;
I = 0.018;
g = 9.8;
L = 0.3;
q = (mCart+mPend)*(I+mPend*L^2)-(mPend*L)^2;

%%State Space
p = I*(mCart+mPend)+mCart*mPend*L^2;

A = [0      1              0           0;
     0 -(I+mPend*L^2)*b/p  (mPend^2*g*L^2)/p   0;
     0      0              0           1;
     0 -(mPend*L*b)/p       mPend*g*L*(mCart+mPend)/p  0];
B = [     0;
     (I+mPend*L^2)/p;
          0;
        mPend*L/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)

poles = eig(A)

co = ctrb(sys_ss);
controllability = rank(co) %controllable


% p1 = -10 + 10i;
% p2 = -10 - 10i;
% p3 = -15 + 10i;
% p4 = -15 - 10i;
% 
% K = place(A,B,[p1 p2 p3 p4]);

Q = C'*C
Q(1,1) = 10;
Q(3,3) = 100


R = 1;
K = lqr(A,B,Q,R)



Ac = (A-B*K);
Bc = B;
Cc = C;
Dc = D;

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:8;

Cn = [1 0 0 0];
sys_ss = ss(A,B,Cn,0);
Nbar = rscale(sys_ss,K);
r =Nbar*10*ones(size(t));

[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')
