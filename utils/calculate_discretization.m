function params_z = calculate_discretization(X, U, sigma, params)
% CALCULATE_DISCRETIZATION  Her aralık için doğrusal atış haritasını hesaplar:
%   x_{k+1} ≈ A_bar(:,:,k)*x_k + B_bar(:,:,k)*u_k + C_bar(:,:,k)*u_{k+1}
%             + S_bar(:,k)*sigma_k + z_bar(:,k)
%   Python CT-cSTC hücre 19, calculate_discretization fonksiyonunun karşılığı.
%   Python aralıkları vmap ile aynı anda hesaplar; burada for döngüsü var
%   (aralıklar birbirinden bağımsız, sıra önemsiz).
%   Çıktılar 3B dizi:
%     A_bar  n_x × n_x × (K-1)
%     B_bar, C_bar  n_x × n_u × (K-1)
%     f_bar, S_bar, z_bar  n_x × (K-1)

    n_x = params.n_x;
    n_u = params.n_u;
    K   = params.K;

    i0 = params.i0;  i1 = params.i1;  i2 = params.i2;  i3 = params.i3;
    i4 = params.i4;  i5 = params.i5;  i6 = params.i6;

    % --- boş rafları hazırla ---
    f_bar = zeros(n_x, K-1);
    A_bar = zeros(n_x, n_x, K-1);
    B_bar = zeros(n_x, n_u, K-1);
    C_bar = zeros(n_x, n_u, K-1);
    S_bar = zeros(n_x, K-1);
    z_bar = zeros(n_x, K-1);

    for k = 1:K-1
        % --- 1. bu aralığın girdileri ---
        x_k = X(:, k);
        u0  = U(:, k);
        u1  = U(:, k+1);

        % --- 2. zaman ayarları (Python'daki k 0'dan başlar, bizde k-1) ---
        if params.free_final_time
            if params.time_dil
                t_scp   = 1;
                tt      = cumsum([0, sigma(1, :)]);
                tt_k    = tt(k);
                sigma_k = sigma(1, k);
            else
                t_scp   = 1/(K - 1);
                tt_k    = (k - 1)*sigma/(K - 1);
                sigma_k = sigma;
            end
        else
            t_scp   = params.t_scp;
            tt_k    = (k - 1)*t_scp;
            sigma_k = 1;
        end

        % --- 3. boş tren: x_k, Phi = I, gerisi sıfır ---
        V0 = [x_k; reshape(eye(n_x), [], 1); zeros(i6 - i2, 1)];

        % --- 4. treni çalıştır, sadece son durağı al ---
        V = rk4(@dVdt, V0, t_scp, params.rk4_steps_J, u0, u1, sigma_k, tt_k, params);
        V = V(:, end);

        % --- 5. vagonları rafa diz ---
        f_bar(:, k)    = V(i0+1:i1);
        A_bar(:, :, k) = reshape(V(i1+1:i2), n_x, n_x);
        B_bar(:, :, k) = reshape(V(i2+1:i3), n_x, n_u);
        C_bar(:, :, k) = reshape(V(i3+1:i4), n_x, n_u);
        S_bar(:, k)    = V(i4+1:i5);
        z_bar(:, k)    = V(i5+1:i6);
    end

    % --- 6. Python'daki params_z sözlüğünün karşılığı ---
    params_z.f_bar = f_bar;
    params_z.A_bar = A_bar;
    params_z.B_bar = B_bar;
    params_z.C_bar = C_bar;
    params_z.S_bar = S_bar;
    params_z.z_bar = z_bar;
end
