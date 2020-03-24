function [state_dot] = dynamic(params, state, taul, taur, dl, dr)

% state(1)  = 0;    %theta
% state(2)  = 0;    %alpha
% state(3)  = 0;    %x
% state(4)  = 0;    %theta_dot
% state(5)  = 0;    %alpha_dot
% state(6)  = 0;    %x_dot


%************  DYNAMICS ************************
Ip = params.Ip;
Iw = params.Iw;
IM = params.IM;
Mw = params.Mw;
m  = params.m;
M  = params.M;
d  = params.d;
r  = params.r;
l  = params.l;
g  = params.g;

state_dot = zeros(6,1);
state_dot(1:3) = state(4:6);
alpha = state(2);
alpha_dot = state(5);
Itheta = Ip + d^2 * (Mw + Iw/(r^2));

state_dot(4) = (d/(r*Itheta))*(taul - taur) + (d/Itheta)*(dl - dr);
MassMatrix = [m*l*cos(alpha), M+m+2*((Iw/(r^2))+Mw)
              m*l^2+IM,       m*l*cos(alpha)];
alpha_x = MassMatrix \ [m*l*alpha_dot^2*sin(alpha) + (taul + taur)/r + dl+dr; m*g*l*sin(alpha)];
state_dot(5) =  alpha_x(1);
state_dot(6) =  alpha_x(2);


    
end