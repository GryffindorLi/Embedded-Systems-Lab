% ES labs 2021-2022
% Author: Kenrick Trip 

syms P0 P1 P2 P3 P4 P5 P6 P7 P8 P9
syms Q0 Q1 Q2
syms freq
syms R0 R1
syms Z0 Z1
syms X0 X1 X2
syms c
syms U0 U1
syms Y0 Y1
syms K0 K1 K2 K3 K4 K5 K6

X_prev = [X0; X1; X2];

% state update:
X_next = [X0 + X1/freq - X2/freq;
          X1 + (c*(U0 - U1))/freq;
          X2]

P_prev = [P0, P1, P2; 
          P3, P4, P5;
          P6, P7, P8];

Q = diag([Q0, Q1, Q2]);

F = [1, 1/freq, -1/freq;
     0, 1, 0;
     0, 0, 1];

% cov update:
P_next = F*P_prev*F' + Q

z = [Z0; Z1];

H = [1, 0, 0;
     0, 1, 0];

% error update:
Y_next = z - H*X_next
Y = [Y0; Y1];

R = diag([R0, R1]);
S_next  = H*P_prev*H' + R
S_next_inv = (1/((P0 + R0)*(P4 + R1) - P1*P3))*[P4 + R1, -P1; -P3, P0 + R0]

% kalman gain
K_next  = P_prev*H'*S_next_inv

K = [K0, K1; K2, K3; K4, K5];

% stata and cov update
X = X_prev + K*Y
P = (eye(3) - K*H)*P_prev