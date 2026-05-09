%% HORIZONTAL SEGMENT

y_h = 0;
x_h_0 = 0;
x_h_f = 6;
theta_h = 0;

q_h_0 = [x_h_0, y_h, theta_h];
q_h_f = [x_h_f, y_h, theta_h];

%% VERTICAL SEGMENT

x_v = 0;
y_v_0 = 0;
y_v_f = 4;
theta_v = pi/2;

q_v_0 = [x_v, y_v_0, theta_v];
q_v_f = [x_v, y_v_f, theta_v];

%% ZIGZAG
zigzag_ampl = 3;
zigzag_period = 6;
zigzag_period_length=3;


%% CIRCULAR

omega_circ = 1;
R_circ = 1;
center_circ = [0,0];

%% ELLIPTICAL
ellipse_x_axis = 3;
ellipse_y_axis = 1;
ellipse_center = [0, 0];
