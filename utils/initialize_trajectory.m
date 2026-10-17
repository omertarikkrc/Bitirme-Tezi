function params = initialize_trajectory(params)
% INITIALIZE_TRAJECTORY  İlk yörünge tahmini 

    % --- durum: başlangıçtan bitişe doğrusal geçiş ---
    s = linspace(0, 1, params.K);                                     % 1xK: 0, 1/14, ..., 1
    X_last = params.x_init + (params.x_final - params.x_init) .* s;   % n_x x K

    % --- kontrol: itki sabit, açılar sıfır ---
    U_last = zeros(params.n_u, params.K);
    U_last(1, :) = (params.T_max + params.T_min)/2;                  % 4.62 MN (makale: 1.54 MN, B1)

    % --- aralık süreleri ---
    if params.time_dil
        sigma_last = params.t_scp*ones(1, params.K - 1);              % 14 aralık, her biri 1.5 s
    else
        sigma_last = params.t_f;
    end

    params.X_last     = X_last;
    params.U_last     = U_last;
    params.sigma_last = sigma_last;
end