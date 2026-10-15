function params = rl_params_fcn()
    g_0  = 9.806;     % yerçekimi ivmesi [m/s^2]
    I_sp = 330;     % özgül itki [s]

    m_wet = 100000;    % yakıtlı kütle [kg]
    m_dry = 85000;    % yakıtsız kütle [kg]

    
    n_x=15;
    n_u=5;


    
    r_I_init   = [200, 200, 500]';     % [m]     3x1
    v_I_init   = [0, 0, -50]';     % [m/s]   3x1
    q_B_I_init = euler_to_quat([90, 0, 0]');     % euler_to_quat ile
    w_B_init   = deg2rad([0,0,0])';     % [rad/s] 3x1

    r_I_final   = [0, 0, 0]';
    v_I_final   = [0, 0, -5]';
    q_B_I_final = euler_to_quat([0, 0, 0]');
    w_B_final   = deg2rad([0, 0, 0]');

    % ... final değerleri için aynı dört satır

    x_init  = [m_wet;r_I_init;v_I_init;q_B_I_init;w_B_init;0];            % 15x1
    x_final = [m_dry;r_I_final;v_I_final;q_B_I_final;w_B_final;0];        % 15x1

    % ... struct'a dört alan: n_x, n_u, x_init, x_final

    params = struct();



    params.rho_air=1.225; % air density [kg m^{-3}]
    params.S_area= 545; % area pf rocket [m^{2}]
    params.C_aero=diag([0.4068, 0.4068, 0.0522]);% Drag coefficient
    params.r_cm=[0., 0., -14.]';
    params.r_cp=[0., 0., 3.]';
    params.J_B=diag([60, 60, 1.5]);
    params.J_B_inv=diag([1/60, 1/60, 1/1.5]);



    params.n_x = n_x;
    params.n_u = n_u;

    params.x_init=x_init;
    params.x_final=x_final;

    
    params.m_wet   = m_wet;
    params.m_dry   = m_dry;
    params.g_I     = [0;0;-g_0];    % inertial frame e göre yer çekimi ivme vektörü
    params.alpha_m = 1/(I_sp*g_0);  %itki = mdot*Isp*g_0 // mdot=alfa_m*itki
        


    %Constraintler ile ilgili tanımlamalar
    params.w_stc_1= 100;    % kısıtın ceza katsayısı
    params.theta_max = 90;  % [deg] tilt açısı max
    params.gs_max = 90 - 35; %params.gs_max = 90 - 35;   % [deg] glideslope, YERDEN ölçülen açı (dikeyden 35°)
    params.w_B_max = deg2rad(90);  %  [rad/s] max açısal hız



end

