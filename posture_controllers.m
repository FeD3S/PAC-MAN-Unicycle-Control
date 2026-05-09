%%cartesian

K_cartesian = [1, 1];
p_d_cartesian = [0;0];
x0_cartesian = [-1;-1;pi/4];

%x0_cartesian = [1;1;0];

%% posture

q_d_posture = [0;0;0];
K_posture = [100;100;0.01];

theta_pos = 0;
gamma_pos = pi/2;
delta_pos = gamma_pos+theta_pos;

err_mag = 2;
err_phase = gamma_pos;

q0_posture = [err_mag;err_phase;delta_pos];