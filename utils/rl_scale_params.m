function params = rl_scale_params(params)
% RL_SCALE_PARAMS  Parametreleri yeni birim sistemine çevirir 
%   uzunluk birimi L = r_scale [m], kütle birimi M = m_scale [kg], zaman 1 s

    r_scale = params.r_scale;  % mesafe 
    m_scale = params.m_scale;  % yakıtlı kütle 

    % --- 4.3.2: fizik parametreleri ---
    params.alpha_m = params.alpha_m*r_scale;          % [s/m]   -> x L     %mdot/itki
    params.r_cm    = params.r_cm/r_scale;             % [m]     -> / L
    params.r_cp    = params.r_cp/r_scale;             % [m]     -> / L
    params.g_I     = params.g_I/r_scale;              % [m/s^2] -> / L     %ivme
    params.J_B     = params.J_B/(r_scale^2);          % [m^2]   -> / L^2 (kütle başına atalet)
    params.J_B_inv = params.J_B_inv*(r_scale^2);      % [1/m^2] -> x L^2   %
    params.rho_air = params.rho_air/(r_scale*m_scale)*(r_scale*r_scale);   % = rho*L/M (S_area'nın ölçeği de içinde)

    params.m_wet = params.m_wet/m_scale;              % [kg]    -> / M
    params.m_dry = params.m_dry/m_scale;

        % --- 4.3.3: sınır koşulları ---
    params.x_init(1)   = params.x_init(1)/m_scale;        % kütle
    params.x_init(2:4) = params.x_init(2:4)/r_scale;      % konum
    params.x_init(5:7) = params.x_init(5:7)/r_scale;      % hız

    params.x_final(1)   = params.x_final(1)/m_scale;
    params.x_final(2:4) = params.x_final(2:4)/r_scale;
    params.x_final(5:7) = params.x_final(5:7)/r_scale;

    % --- 4.3.3: başlangıç tahmini (dikkat: satırın tamamı -> (i, :)) ---
    params.X_last(1, :)   = params.X_last(1, :)/m_scale;
    params.X_last(2:4, :) = params.X_last(2:4, :)/r_scale;
    params.X_last(5:7, :) = params.X_last(5:7, :)/r_scale;

    % --- 4.3.3: itki sınırları ve itki tahmini [N] -> / (M*L) ---
    params.T_max     = params.T_max/(m_scale*r_scale);
    params.T_min     = params.T_min/(m_scale*r_scale);
    params.T_max_aft = params.T_max_aft/(m_scale*r_scale);
    params.T_min_aft = params.T_min_aft/(m_scale*r_scale);

    params.U_last(1, :) = params.U_last(1, :)/(m_scale*r_scale);

        % --- 4.3.4: tetik ve kural eşikleri --- 
%kısıtlar da ölçeklendirilmiş büyüklüklerle karşılaştırılıyor ondan
%sebepler bunları da ölçeklemememzi gerekk
    params.alt_stc_trig             = params.alt_stc_trig/r_scale;              % 100 m  -> / L
    params.alt_stc_trig_los         = params.alt_stc_trig_los/r_scale;          % 200 m  -> / L
    params.alt_stc_trig_los_gs_done = params.alt_stc_trig_los_gs_done/r_scale;  % 2 m    -> / L
    params.spd_stc_cons             = params.spd_stc_cons/r_scale;              % 20 m/s -> / L
    params.spd_stc_trig             = params.spd_stc_trig/r_scale;              % 35 m/s -> / L



end