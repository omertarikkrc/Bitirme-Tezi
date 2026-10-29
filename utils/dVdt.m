function dV = dVdt(V, t, u_0, u_1, sigma, tt, params)
% DVDT  V treninin (x, Phi, Bm, Bp, S, z) normalize zamana (tau) göre türevi.
%   Python CT-cSTC hücre 19, dVdt fonksiyonunun karşılığı.
%   Matrisler V içinde sütun-sütun (MATLAB sırası) saklanır.
%   tt kullanılmıyor; Python ile aynı imza için duruyor.

    n_x = params.n_x;
    n_u = params.n_u;
    K   = params.K;

    i0 = params.i0;  i1 = params.i1;  i2 = params.i2;  i3 = params.i3;
    i4 = params.i4;  i5 = params.i5;  i6 = params.i6;

    % --- 1. lokomotif: o anki durum ---
    x = V(i0+1:i1);

    % --- 2. FOH ağırlıkları ve o anki kontrol ---
    if strcmp(params.inp_param, 'ZOH')
        beta = 0;
    elseif strcmp(params.inp_param, 'FOH')
        beta = t / params.t_scp;
        if params.free_final_time
            if params.time_dil
                beta = t;
            else
                beta = t / (1/(K - 1));
            end
        end
    end
    alpha = 1 - beta;
    u = u_0 + beta*(u_1 - u_0);

    % --- 3. anlık Jacobian'lar (CasADi), sigma ile çarpılmış ---
    A_subs = sigma * full(params.A_func(x, u));
    B_subs = sigma * full(params.B_func(x, u));
    f_subs = full(params.f_func(x, u));

    if params.free_final_time
        z_t = -A_subs*x - B_subs*u;
    else
        z_t = f_subs - A_subs*x - B_subs*u;
    end

    % --- 4. vagonları aç ---
    Phi = reshape(V(i1+1:i2), n_x, n_x);
    Bm  = reshape(V(i2+1:i3), n_x, n_u);
    Bp  = reshape(V(i3+1:i4), n_x, n_u);
    S   = V(i4+1:i5);
    z   = V(i5+1:i6);

    % --- 5. her vagonun türevi ---
    dPhi = A_subs*Phi;
    dBm  = A_subs*Bm + B_subs*alpha;
    dBp  = A_subs*Bp + B_subs*beta;
    dS   = A_subs*S  + f_subs;
    dz   = A_subs*z  + z_t;

    % --- 6. vagonları kapat, treni geri diz ---
    dV = [sigma*f_subs; dPhi(:); dBm(:); dBp(:); dS; dz];
end
