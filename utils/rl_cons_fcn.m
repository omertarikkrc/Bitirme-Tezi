function vehicle_cons = rl_cons_fcn(X, U, sigma, X_last, U_last, sigma_last, params, npy)
% RL_CONS_FCN  Araç kısıtları (Python hücre 41, rl_cons_fcn).
%   X, U, sigma YALMIP değişkenleri (sdpvar) olmalı; çıktı bir YALMIP kısıt listesi.
%   X_last, U_last, sigma_last, npy kullanılmıyor; Python ile aynı imza için duruyor.

    n_x = params.n_x;
    K   = params.K;

    vehicle_cons = [];

    % --- aralık süreleri ---
    if params.free_final_time
        if params.time_dil
            if params.add_minmax_sig
                vehicle_cons = [vehicle_cons, ...
                    sigma(1, :) >= params.min_sigma, ...
                    sigma(1, :) <= params.max_sigma];
            end
        end
    end

    % --- sınır şartları ---
    vehicle_cons = [vehicle_cons, X(:, 1) == params.x_init];                    % başlangıç: hepsi
    vehicle_cons = [vehicle_cons, X(2:n_x-1, K) == params.x_final(2:n_x-1)];    % bitiş: kütle ve y hariç

    % --- CTCS: bir aralıkta y en fazla beta_ctcs artabilir ---
    vehicle_cons = [vehicle_cons, X(n_x, 2:K) - X(n_x, 1:K-1) <= params.beta_ctcs];

    % --- durum kısıtı ---
    vehicle_cons = [vehicle_cons, X(1, :) >= params.m_dry];

    % --- kontrol kısıtları ---
    vehicle_cons = [vehicle_cons, ...
        U(1, :) <= params.T_max, ...
        params.T_min_aft <= U(1, :), ...
        -params.delta_engine_max*pi/180 <= U(2, :), ...
        U(2, :) <= params.delta_engine_max*pi/180, ...
        U(4, :) <= params.delta_boresight_max*pi/180, ...
        -params.delta_boresight_max*pi/180 <= U(4, :)];
end