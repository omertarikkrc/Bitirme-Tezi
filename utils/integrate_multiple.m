function x_next = integrate_multiple(X, U, sigma, params)
% INTEGRATE_MULTIPLE  Çok atış: her aralığı kendi düğümünden RK4 ile integre eder
%   X      : n_x x K       düğümlerdeki durumlar
%   U      : n_u x K       düğümlerdeki kontroller
%   sigma  : 1 x (K-1)     aralık süreleri (time_dil açıkken)
%   x_next : n_x x (K-1)   k. sütun = k. düğümden başlayıp bir aralık integre edilmiş durum

    K = params.K;

    % --- aralık süreleri ---
    if params.free_final_time
        if params.time_dil
            sigma_vec = sigma(1, :);                    % her aralığın kendi süresi
        else
            sigma_vec = ones(1, K-1)*sigma/(K-1);       % tek toplam süre, eşit bölünmüş
        end
    else
        sigma_vec = ones(1, K-1)*params.t_scp;          % sabit süre
    end

    % --- her aralığın uçuştaki başlangıç zamanı: 0, s1, s1+s2, ... ---
    tt_vec = cumsum([0, sigma_vec]);
    tt_vec = tt_vec(1:end-1);

    % --- 14 kısa atış (Python: vmap) ---
    x_next = zeros(params.n_x, K-1);
    for k = 1:K-1
        x_k     = X(:, k);
        u_k     = U(:, k);
        u_kp1   = U(:, k+1);
        sigma_k = sigma_vec(k);
        tt      = tt_vec(k);

        y = rk4(@dxdt, x_k, sigma_k, params.rk4_steps_dyn, tt, sigma_k, u_k, u_kp1, params);
        x_next(:, k) = y(:, end);                       % aralığın sonundaki durum
    end
end