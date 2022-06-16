% ES labs 2021-2022
% Author: Kenrick Trip 

clear all

%% All variables:
l = 0.26; % m
m1 = 0.7; % kg
m2 = 0.08; % kg
r1 = 0.045; % m
r2 = l/sqrt(2); % m

Iy = (2*m1*r1^2)/5 + 4*m2*l^2 % yaw inertia
Ip = (2*m1*r1^2)/5 + 4*m2*r2^2 % pitch inertia
Ir = Ip % roll inertia

% PWM to thrust (T) and torque (Q):
cT = (0.78*9.81)/1000;
cQ = 0.07/1000;

% period of microcontroller
dt = 0.1; % s

%% Model:

% construct state space model
A = [0,0,0,1,0,0;
     0,0,0,0,1,0;
     0,0,0,0,0,1;
     zeros(3,6)];

B = [zeros(3,4);
     (cQ/Iy)*[-1,1,1,-1];
     ((cT*l)/(Ip*sqrt(2)))*[1,1,-1,-1];
     ((cT*l)/(Ir*sqrt(2)))*[1,-1,1,-1]];

C = [eye(3),zeros(3,3)];

D = 0;

% open loop Continuous Time system:
OLsys = ss(A,B,C,D);

OLsys.Inputname = {'s1','s2','s3','s4'};
OLsys.Outputname = {'y','p','r'};

%% Controller:

% Closed loop control
tau = 0.1;

Kpy = 72.4; % controller P gain yaw
Kiy = 4.19; % controller I gain yaw
Kdy = 313.0; % controller D gain yaw

Kpp = 1.85; % controller P gain pitch
Kip = 0.107; % controller I gain pitch
Kdp = 7.98; % controller D gain pitch

Kpr = 1.85; % controller P gain roll
Kir = 0.107; % controller I gain roll
Kdr = 7.98; % controller D gain roll

s = tf('s');
Cy = (Kpy + Kiy/s + Kdy*s)/(tau*s+1); % yaw controller
Cp = (Kpp + Kip/s + Kdp*s)/(tau*s+1); % pitch controller
Cr = (Kpr + Kir/s + Kdr*s)/(tau*s+1); % roll controller

% test PID autotune:
% [C_y1,info] = pidtune(OLsys(1,1),'PID')
% [C_y2,info] = pidtune(OLsys(1,2),'PID')
% [C_y3,info] = pidtune(OLsys(1,3),'PID')
% [C_y4,info] = pidtune(OLsys(1,4),'PID')
% [C_p1,info] = pidtune(OLsys(2,1),'PID')
% [C_p2,info] = pidtune(OLsys(2,2),'PID')
% [C_p3,info] = pidtune(OLsys(2,3),'PID')
% [C_p4,info] = pidtune(OLsys(2,4),'PID')
% [C_r1,info] = pidtune(OLsys(3,1),'PID')
% [C_r2,info] = pidtune(OLsys(3,2),'PID')
% [C_r3,info] = pidtune(OLsys(3,3),'PID')
% [C_r4,info] = pidtune(OLsys(3,4),'PID')

Csys = [-Cy, Cp, Cr;
        Cy, Cp, -Cr; 
        Cy, -Cp, Cr; 
        -Cy, -Cp, -Cr];

Csys.Inputname = {'err_y', 'err_p', 'err_r'};
Csys.Outputname = {'s1','s2','s3','s4'};

Sum1 = sumblk('err_y = y_{ref} - y');
Sum2 = sumblk('err_p = p_{ref} - p');
Sum3 = sumblk('err_r = r_{ref} - r');

CLsys = connect(OLsys,Csys,Sum1,Sum2,Sum3,{'y_{ref}','p_{ref}','r_{ref}'},...
               {'y', 'p', 'r'});
p = pole(CLsys);

% discretise controller
DT_CLsys = c2d(CLsys,dt,'tustin');
DT_Csys = c2d(Csys,dt,'tustin');
DT_OLsys = c2d(OLsys,dt,'tustin');

%% Simulations

% step response system
figure(1)
step(DT_CLsys, 3);

% simulate input control sequences:
t = 0:dt:15;
y_ref = 0.1*square(t);
p_ref = zeros(1,length(t));
r_ref = zeros(1,length(t));
u = [y_ref; p_ref; r_ref];

y = lsim(DT_CLsys,u,t);
mot_sig = lsim(DT_Csys,u-y',t);

% add throttle and saturation
throttle = 200; % between 0 and 255

ESC_unsat = mot_sig + throttle*ones(length(t),4);

ESC_sat = zeros(length(t),4);

UB = 255-throttle;
LB = -UB;

for i = 1:length(t)
    for k = 1:4
        ESC_sat(i,k) = throttle + min(UB, max(LB, mot_sig(i,k)));
    end
end

y_sat = lsim(DT_OLsys,mot_sig,t);

% plot reference tracking
figure(2)
stairs(t,y)
hold on
plot(t,u)
hold off
title('I/O')
legend('y','p','r','y_{ref}','p_{ref}','r_{ref}')
ylabel('Angles (rad)')
xlabel('Time (s)')

% plot ESC signals
figure(3)
stairs(t,mot_sig)
title('ESC signals without throttle or saturation')
legend('s1','s2','s3','s4')
ylabel('PWM signal \pm(0,255)')
xlabel('Time (s)')

% plot ESC signals
figure(4)
stairs(t,ESC_sat)
title('ESC signals with throttle and saturation')
legend('s1','s2','s3','s4')
ylabel('PWM signal \pm(0,255)')
xlabel('Time (s)')

% plot reference tracking
figure(5)
stairs(t,y_sat)
hold on
plot(t,u)
hold off
title('I/O')
legend('y','p','r','y_{ref}','p_{ref}','r_{ref}')
ylabel('Angles (rad)')
xlabel('Time (s)')


