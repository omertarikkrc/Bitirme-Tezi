params = rl_params_fcn();

x_t = [95000;                          % kütle [kg]
       100; 80; 300;                   % konum [m]
       -10; -5; -40;                   % hız [m/s]
       euler_to_quat([30; 10; 5]);     % yönelim
       0.02; -0.03; 0.01;              % açısal hız [rad/s]
       0];                             % ihlal sayacı

u_t = [4e6;                            % itki [N]
       deg2rad(5);  deg2rad(30);       % motor: 5 derece eğik, 30 derece yönünde
       deg2rad(3);  deg2rad(45)];      % kamera ekseni