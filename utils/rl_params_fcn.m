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


    %irtifa tetik 100 metre
    % [m] h1_trig (Tablo 3): kuralların devreye girdiği irtifa
   
    params.alt_stc_trig = 100; 


    % hız/açısal hız STC'leri 1.1*100 = 110 m'de tetiklenir (makalede 100 m)
    params.alpha_alt_stc_trig = 1.1; 

    %2 metrenin üstünde miyim tetiği 
    %glidescope kısıtı düşük irtifada sıkıntı çıkartıyor.
    params.alt_stc_trig_los_gs_done = 2.0;



    % [deg] delta_stc, Tablo 3 (birimi orada yanlışlıkla deg/s yazılmış anlamadık)
    params.delta_stc_cons = 1.0;    



    %% stc 
    params.w_stc_2              = 1000;   % ceza ağırlığı
    params.theta_stc_cons       = 5.0;    % [deg] theta_stc, Tablo 3
    params.alpha_theta_stc_cons = 0.92;   % güvenlik payı: sınır 0.92*5 = 4.6 deg

    %Glideslope 
    params.alpha_gs_stc_cons =0.92; % marj
    params.gs_stc_cons=90 - 5; % derece


    %angular velo stc
    params.omega_stc_cons = 2.5;
    params.alpha_omega_stc_cons = 0.99;



    %hız velo 
    params.spd_stc_cons = 20.0;
    params.alpha_spd_stc_cons = 0.95;
    params.w_stc_3 = 600;
    



    %görüş hattı
    % görüş konisi yarı açısı [deg] (makale: psi_stc)
    params.los_stc_cons       = 5; 
    % güvenlik payı -> 4.9 deg
    params.alpha_los_stc_cons = 0.98;   

    params.alt_stc_trig_los           = 200;   % LOS tetik irtifası [m] (makale: h2_trig)
    params.alpha_alt_alt_stc_trig_los = 1.1;   % güvenlik payı -> 220 m


%% Hız ve Eğim trigger
%roket yavaş VE dik ise itki dar bir aralıkta olsun
%hızlı VEYA eğik ise itki normal bir aralıkta olsun

    params.spd_stc_trig   = 35.0;   % hız tetiği [m/s] (makale: v_I^trig)
    params.theta_stc_trig = 60;     % eğim tetiği [deg] (makale: theta^trig)


    params.T_max       = 2200000*3.0;       % normal aralık üst sınırı [N]  (makale: T_max^stc2)
    params.T_min       = 2200000*0.4*3.0;   % normal aralık alt sınırı [N]  (makale: T_min^stc2)
    params.T_max_aft   = 2200000;           % son yaklaşma üst sınırı [N]   (makale: T_max^stc1)
    params.T_min_aft   = 2200000*0.4;       % son yaklaşma alt sınırı [N]   (makale: T_min^stc1)
    params.alpha_T_min = 1.02;              % güvenlik payı -> 2.6928 MN



end

