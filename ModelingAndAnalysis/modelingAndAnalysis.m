%Modeling
%%Transfer Function
mCart = 0.5;
mPend = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
L = 0.3;
q = (mCart+mPend)*(I+mPend*L^2)-(mPend*L)^2;
s = tf('s');

P_cart = (((I+mPend*L^2)/q)*s^2 - (mPend*g*L/q))/(s^4 + (b*(I + mPend*L^2))*s^3/q - ((mCart + mPend)*mPend*g*L)*s^2/q - b*mPend*g*L*s/q);
P_pend = (mPend*L*s/q)/(s^3 + (b*(I + mPend*L^2))*s^2/q - ((mCart + mPend)*mPend*g*L)*s/q - b*mPend*g*L/q);

sys_tf = [P_cart ; P_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf;