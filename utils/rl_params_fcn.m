function params = rl_params_fcn()
% RL_PARAMS_FCN  Problemin bütün sabitleri (Python: rl_params_fcn).
%   Alan adları ve değerler Python'daki gibi; sadece gruplanarak sıralandı.
%   1 roket · 2 durum/kontrol/zaman · 3 başlangıç-bitiş · 4 her zaman açık kısıtlar
%   5 STC'ler (tetiklerine göre) · 6 ölçekleme ve integrasyon · 7 maliyet · 8 SCP döngüsü

    params = struct();

    %% ===== 1. ROKET =====
    g_0   = 9.806;      % yerçekimi ivmesi [m/s^2]
    I_sp  = 330;        % özgül itki [s]
    m_wet = 100000;     % yakıtlı kütle [kg]
    m_dry = 85000;      % yakıtsız kütle [kg]  kullanılabilir yakıt 15 t

    params.m_wet   = m_wet;
    params.m_dry   = m_dry;
    params.g_I     = [0; 0; -g_0];       % yerçekimi vektörü (inertial eksen)
    params.alpha_m = 1/(I_sp*g_0);       % yakıt tüketimi  /%itki = mdot*Isp*g_0 // mdot=alfa_m*itki

    % aerodinamik
    params.rho_air = 1.225;                            % hava yoğunluğu [kg/m^3]
    params.S_area  = 545;                              % referans alan [m^2]
    params.C_aero  = diag([0.4068, 0.4068, 0.0522]);   % gövde eksenlerinde direnç katsayıları

    % geometri ve eylemsizlik (referans: kütle merkezi, gövde z ekseni buruna doğru)
    % ileride Angara modelindeki gibi kayan kütle merkezli yapılacak
    params.r_cm    = [0., 0., -14.]';             % motor, CM'nin 14 m altında (itki momentinin kolu)
    params.r_cp    = [0., 0., 3.]';               % aerodinamik merkez, CM'nin 3 m üstünde
    params.J_B     = diag([60, 60, 1.5]);         % eylemsizlik (kütle başına)
    params.J_B_inv = diag([1/60, 1/60, 1/1.5]);

    %% ===== 2. DURUM, KONTROL, ZAMAN =====
    n_x = 15;     % durum  : [m; r(3); v(3); q(4); w(3); y]
    n_u = 5;      % kontrol: [T; delta_e; phi_e; delta_b; phi_b]
    params.n_x = n_x;
    params.n_u = n_u;

    t_f   = 21;           % toplam süre, ilk tahmin [s]
    K     = 15;           % düğüm sayısı
    t_scp = t_f/(K-1);    % aralık süresi, ilk tahmin [s] = 1.5
    params.t_f   = t_f;
    params.K     = K;
    params.t_scp = t_scp;

    %% ===== 3. BAŞLANGIÇ VE BİTİŞ =====
    r_I_init   = [200, 200, 500]';             % [m]
    v_I_init   = [0, 0, -50]';                 % [m/s] 50 m/s ile düşüyor
    q_B_I_init = euler_to_quat([90, 0, 0]');   % roket yatay
    w_B_init   = deg2rad([0, 0, 0])';          % [rad/s]

    r_I_final   = [0, 0, 0]';                  % iniş noktası
    v_I_final   = [0, 0, -5]';                 % yumuşak temas [m/s]
    q_B_I_final = euler_to_quat([0, 0, 0]');   % roket dik
    w_B_final   = deg2rad([0, 0, 0]');

    params.x_init  = [m_wet; r_I_init;  v_I_init;  q_B_I_init;  w_B_init;  0];   % 15x1
    params.x_final = [m_dry; r_I_final; v_I_final; q_B_I_final; w_B_final; 0];   % 15x1

    %% ===== 4. HER ZAMAN AÇIK KISITLAR  =====
    params.w_stc_1   = 100;            % ceza ağırlığı (aşağıda tetiklerde de kullanılıyor)
    params.theta_max = 90;             % [deg] en büyük eğim
    params.gs_max    = 90 - 35;        % [deg] glideslope, yerden ölçülen açı (dikeyden 35°)
    params.w_B_max   = deg2rad(90);    % [rad/s] en büyük açısal hız

    %% ===== 5. STC'LER (tetiklerine göre) =====
    % ceza ağırlıkları
    %   w_stc_1 (100) : irtifa tetikleri, gimbal, glideslope ve açısal hız STC'leri
    %   w_stc_2 (1000): eğim STC'si, hız ve eğim tetikleri
    %   w_stc_3 (600) : hız, LOS ve itki STC'leri
    params.w_stc_2 = 1000;
    params.w_stc_3 = 600;

    % ortak irtifa eşikleri
    params.alt_stc_trig             = 100;   % [m] h1 (makale): ana irtifa tetiği
    params.alt_stc_trig_los_gs_done = 2.0;   % [m] bunun altında glideslope ve LOS kapanır (makalede yok) yakın mesafede sıkınıtı çıkartıyor.

    % --- 5a. h < 100 m: gimbal ve eğim ---
    params.delta_stc_cons       = 1.0;    % [deg] gimbal ≤ 1° (Tablo 3'te birimi yanlışlıkla deg/s)
    params.theta_stc_cons       = 5.0;    % [deg] eğim ≤ 5°
    params.alpha_theta_stc_cons = 0.92;   % pay: 0.92·5 = 4.6°

    % --- 5b. 2 m < h < 100 m: glideslope ---
    params.gs_stc_cons       = 90 - 5;    % [deg] yerden ölçülen açı (dikeyden 5° koni)
    params.alpha_gs_stc_cons = 0.92;      % pay; koniyi daraltmak yerine genişletiyor

    % --- 5c. h < 110 m: hız ve açısal hız ---
    params.alpha_alt_stc_trig   = 1.1;    % tetik payı: 1.1·100 = 110 m (makalede 100 m)
    params.spd_stc_cons         = 20.0;   % [m/s] hız ≤ 20
    params.alpha_spd_stc_cons   = 0.95;   % pay: 19 m/s
    params.omega_stc_cons       = 2.5;    % [deg/s] açısal hız ≤ 2.5
    params.alpha_omega_stc_cons = 0.99;   % pay: 2.475 °/s

    % --- 5d. 2 m < h < 220 m: görüş hattı (LOS) ---
    params.alt_stc_trig_los           = 200;    % [m] h2 (makale)
    params.alpha_alt_alt_stc_trig_los = 1.1;    % tetik payı: 1.1·200 = 220 m
    params.los_stc_cons               = 5;      % [deg] kamera hedefi 5° içinde görmeli (psi_stc)
    params.alpha_los_stc_cons         = 0.98;   % pay: 4.9°

    % --- 5e. hız ve eğim tetiği: itki bantları ---
    %   yavaş VE dik   -> dar bant   [T_min_aft, T_max_aft]          = 0.88–2.2 MN
    %   hızlı VEYA eğik -> geniş bant [alpha_T_min·T_min, T_max]     = 2.69–6.6 MN
    params.spd_stc_trig   = 35.0;              % [m/s] hız tetiği (v_I^trig)
    params.theta_stc_trig = 60;                % [deg] eğim tetiği (theta^trig)
    params.T_max          = 2200000*3.0;       % [N] geniş bant üst sınırı (T_max^stc2)
    params.T_min          = 2200000*0.4*3.0;   % [N] geniş bant alt sınırı (T_min^stc2)
    params.T_max_aft      = 2200000;           % [N] dar bant üst sınırı (T_max^stc1)
    params.T_min_aft      = 2200000*0.4;       % [N] dar bant alt sınırı (T_min^stc1)
    params.alpha_T_min    = 1.02;              % pay: 1.02·2.64 = 2.69 MN
    % not: T_max ve T_min_aft ayrıca rl_cons_fcn'de sert itki sınırı,
    %      T_max ve T_min initialize_trajectory'de ilk tahmin itkisi olarak kullanılıyor

    %% ===== 6. ÖLÇEKLEME VE İNTEGRASYON =====
    params.time_dil        = true;             % her aralığın süresi ayrı değişken (sigma)
    params.free_final_time = true;             % toplam süreyi optimizasyon seçer
    params.r_scale         = norm(r_I_init);   % uzunluk birimi [m] = 574.46
    params.m_scale         = m_wet;            % kütle birimi [kg] = 100 000
    params.inp_param       = 'FOH';            % kontrol aralık içinde rampa (ZOH: merdiven)

    % gerçek atış (integrate_multiple): arttıkça hassas ama yavaş
    params.rk4_steps_dyn = 20;                 % RK4 nokta sayısı (19 adım)
    % doğrusal model (calculate_discretization): Jacobian'ları biriktiren integrasyon
    params.rk4_steps_J   = 20;

    % V vektörü blok sınırları (Python ile aynı, 0-tabanlı)
    params.i0 = 0;
    params.i1 = params.i0 + n_x;          % 15   x bitti
    params.i2 = params.i1 + n_x*n_x;      % 240  Phi bitti
    params.i3 = params.i2 + n_x*n_u;      % 315  Bm bitti
    params.i4 = params.i3 + n_x*n_u;      % 390  Bp bitti
    params.i5 = params.i4 + n_x;          % 405  S bitti
    params.i6 = params.i5 + n_x;          % 420  z bitti

    %% ===== 7. MALİYET VE ALT PROBLEM KISITLARI (8. adım) =====
    params.cost_type = 'time';     % 'time' (orijinal) | 'fuel' | 'mixed'
    params.w_sigma   = 150;        % süre maliyetinin ağırlığı
    params.w_m       = 150;        % yakıt maliyetinin ağırlığı (sadece 'fuel'/'mixed' deneyleri)

    params.w_con_dyn = 1e4;        % sanal itki (nu) cezası
    params.w_ptr     = 2^12;       % güven bölgesi ağırlığının başlangıç değeri (w_tr = 4096)
    params.w_ds      = 20;         % güven bölgesinde sigma değişimine ek ağırlık

    params.beta_ctcs           = 1e-4;        % CTCS: bir aralıkta y en fazla bu kadar artabilir
    params.delta_engine_max    = 10;          % [deg] gimbal açısı sınırı
    params.delta_boresight_max = 20;          % [deg] kamera açısı sınırı
    params.add_minmax_sig      = true;        % aralık sürelerine alt/üst sınır koy
    params.min_sigma           = t_scp*0.5;   % [s] 0.75
    params.max_sigma           = t_scp*10;    % [s] 15

    %% ===== 8. SCP DÖNGÜSÜ (10. adım) =====
    params.ite           = 85;      % en fazla iterasyon sayısı (ana koşu)
    params.adaptive_step = false;   % Algorithm 1 kapalı (ana koşu); warm-start'ta açılacak
    params.ptr_term      = 1e-4;    % yakınsama eşiği: beklenen iyileşme bundan küçükse dur
    params.w_ptr_min     = 0.001;   % w_tr'nin alt sınırı
    params.r0 = 0.01;               % rho bundan küçük/eşitse: reddet
    params.r1 = 0.1;                % rho bundan küçükse: kabul, güven bölgesini daralt
    params.r2 = 0.8;                % rho bundan büyük/eşitse: kabul, güven bölgesini genişlet
end