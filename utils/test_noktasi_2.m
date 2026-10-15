params = rl_params_fcn();

x_t2 = [88000;                          % kütle [kg]
        60; 40; 50;                     % konum [m]
        -5; 3; -24;                     % hız [m/s]
        euler_to_quat([100; 10; 20]);   % yönelim
        1.5; -1.0; 0.3;                 % açısal hız [rad/s]
        0];

u_t2 = [2.0e6;                          % itki [N]
        deg2rad(3);  deg2rad(40);       % motor: 3 derece eğik
        deg2rad(15); deg2rad(200)];     % kamera ekseni