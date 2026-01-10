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

J_values = [0.005 0.01 0.02];   % low, nominal, high inertia

figure
hold on
for J = J_values
    den = [La*J, (La*B + Ra*J), (Ra*B + Kt*Kb)];
    G = tf(num, den);
    step(G)
end
hold off
grid on

title('Effect of Inertia J on Step Response')
legend('Low J','Nominal J','High J')


B_values = [0.05 0.1 0.2];   % low, nominal, high friction

figure
hold on
for B = B_values
    den = [La*J, (La*B + Ra*J), (Ra*B + Kt*Kb)];
    G = tf(num, den);
    step(G)
end
hold off
grid on

title('Effect of Friction B on Step Response')
legend('Low B','Nominal B','High B')


Ra_values = [0.5 1 2];   % low, nominal, high resistance

figure
hold on
for Ra = Ra_values
    den = [La*J, (La*B + Ra*J), (Ra*B + Kt*Kb)];
    G = tf(num, den);
    step(G)
end
hold off
grid on

title('Effect of Armature Resistance R_a on Step Response')
legend('Low R_a','Nominal R_a','High R_a')
