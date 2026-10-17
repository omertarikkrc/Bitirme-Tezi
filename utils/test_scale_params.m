%% test_scale_params.m
% rl_scale_params.m'nin çıktısını orijinal Python (hücre 31, 33, 35, 37) ile karşılaştırır.
% Ölçüt: her eleman için |MATLAB - Python| / max(|Python|, 1); en büyüğü 1e-12'nin altında olmalı.

clear; format long g
params    = rl_params_fcn();
params    = initialize_trajectory(params);
params_sc = rl_scale_params(params);

% {ad, MATLAB değeri, Python değeri}
tablo = {
    'alpha_m', params_sc.alpha_m, 0.1775215745010176
    'r_cm', params_sc.r_cm, [0; 0; -0.024370871833797696]
    'r_cp', params_sc.r_cp, [0; 0; 0.0052223296786709349]
    'g_I', params_sc.g_I, [0; 0; -0.017070054943015726]
    'J_B (diag)', diag(params_sc.J_B), [0.00018181818181818178; 0.00018181818181818178; 4.5454545454545447e-06]
    'J_B_inv (diag)', diag(params_sc.J_B_inv), [5500.0000000000009; 5500.0000000000009; 220000.00000000003]
    'rho_air', params_sc.rho_air, 0.0070370892420090871
    'm_wet', params_sc.m_wet, 1
    'm_dry', params_sc.m_dry, 0.84999999999999998
    'x_init', params_sc.x_init, [1; 0.34815531191139565; 0.34815531191139565; 0.87038827977848909; 0; 0; -0.087038827977848912; 0.70710678118654757; 0.70710678118654746; 0; 0; 0; 0; 0; 0]
    'x_final', params_sc.x_final, [0.84999999999999998; 0; 0; 0; 0; 0; -0.0087038827977848916; 1; 0; 0; 0; 0; 0; 0; 0]
    'X_last (satir toplami)', sum(params_sc.X_last, 2), [13.874999999999998; 2.6111648393354674; 2.6111648393354674; 6.5279120983386685; 0; 0; -0.71807033081725369; 12.803300858899107; 5.3033008588991057; 0; 0; 0; 0; 0; 0]
    'U_last (satir toplami)', sum(params_sc.U_last, 2), [1.2063581557729861; 0; 0; 0; 0]
    'T_max', params_sc.T_max, 0.11489125293076057
    'T_min', params_sc.T_min, 0.04595650117230423
    'T_max_aft', params_sc.T_max_aft, 0.038297084310253526
    'T_min_aft', params_sc.T_min_aft, 0.015318833724101409
    'alt_stc_trig', params_sc.alt_stc_trig, 0.17407765595569782
    'alt_stc_trig_los', params_sc.alt_stc_trig_los, 0.34815531191139565
    'alt_stc_trig_los_gs_done', params_sc.alt_stc_trig_los_gs_done, 0.0034815531191139566
    'spd_stc_cons', params_sc.spd_stc_cons, 0.034815531191139566
    'spd_stc_trig', params_sc.spd_stc_trig, 0.060927179584494236
    'sigma_last (degismemeli)', params_sc.sigma_last, [1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5; 1.5]
    'w_B_max (degismemeli)', params_sc.w_B_max, 1.5707963267948966
    'omega_stc_cons (degismemeli)', params_sc.omega_stc_cons, 2.5
    'theta_stc_trig (degismemeli)', params_sc.theta_stc_trig, 60
    'S_area (degismemeli)', params_sc.S_area, 545
};

fprintf('\n%-30s %-12s %s\n', 'parametre', 'en buyuk', 'durum');
hepsi = true;
for k = 1:size(tablo, 1)
    a = tablo{k, 2}(:);  b = tablo{k, 3}(:);
    if numel(a) ~= numel(b)
        fprintf('%-30s %-12s %s\n', tablo{k, 1}, '-', 'BOYUT FARKLI');  hepsi = false;  continue
    end
    rel  = abs(a - b) ./ max(abs(b), 1);   % göreli hata (küçük değerlerde mutlak)
    hata = max(rel);
    if hata < 1e-12, durum = 'TUTTU'; else, durum = 'KONTROL ET'; hepsi = false; end
    fprintf('%-30s %-12.1e %s\n', tablo{k, 1}, hata, durum);
end
if hepsi, fprintf('\nHepsi tuttu.\n'); end