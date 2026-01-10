Ra = 1;      % Armature resistance
La = 0.5;    % Armature inductance
J  = 0.01;   % Rotor inertia
B  = 0.1;    % Viscous friction
Kt = 0.01;   % Torque constant
Kb = 0.01;   % Back EMF constant

num = Kt;
den = [La*J, (La*B + Ra*J), (Ra*B + Kt*Kb)];

G = tf(num, den)
figure;
step(G)
grid on
pole(G)
figure;
T = feedback(G, 1);
step(T)
grid on
figure;
bode(G)
grid on
