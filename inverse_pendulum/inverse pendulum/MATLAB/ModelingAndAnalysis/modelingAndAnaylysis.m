%Modeling
%%Transfer Function
mCart = 0.5;
mPend = 0.2;
b = 0.1;
I = 0.018;
g = 9.8;
L = 0.3;
q = (mCart+mPend)*(I+mPend*L^2)-(mPend*L)^2;
s = tf('s');

P_cart = (((I+mPend*L^2)/q)*s^2 - (mPend*g*L/q))/(s^4 + (b*(I + mPend*L^2))*s^3/q - ((mCart + mPend)*mPend*g*L)*s^2/q - b*mPend*g*L*s/q);
P_pend = (mPend*L*s/q)/(s^3 + (b*(I + mPend*L^2))*s^2/q - ((mCart + mPend)*mPend*g*L)*s/q - b*mPend*g*L/q);

sys_tf = [P_cart ; P_pend]

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)


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

%analysis

%%pole
pole(sys_tf)
eig(A)

%%impulse
t=0:0.01:1;
impulse(sys_ss,t)

%%step response
t = 0:0.05:1;
u = ones(size(t));
[y,t] = lsim(sys_tf,u,t);
figure
plot(t,y)
title('Open-Loop Step Response')
legend('x','phi')
