%% test_discretization.m
% calculate_discretization.m (adım 7.4) beş yoldan sınanır:
%   (1) Python calculate_discretization'a karşı (ref_discretization.mat)
%   (2) f_bar = integrate_multiple : iki ayrı integrasyon yolu aynı yere varmalı
%   (3) Taylor özdeşliği          : cetvel dayandığı noktada tam doğru
%   (4) Sapma testi               : eps yarıya inince doğrusallaştırma hatası 4 kat küçülmeli
%   (5) Kütle satırları           : elle bilinen tam sonuçlar
% Göreli hata: |a-b|/max(|b|,1)

clear; format long g
params    = rl_params_fcn();
params    = initialize_trajectory(params);
params_sc = rl_scale_params(params);
params_sc = rl_dynamics(params_sc);
p   = params_sc;
n_x = p.n_x;  n_u = p.n_u;  K = p.K;
X   = p.X_last;  U = p.U_last;  sig = p.sigma_last;
rel = @(a, b) max(abs(a(:) - b(:)) ./ max(abs(b(:)), 1));

tic
Z = calculate_discretization(X, U, sig, p);
fprintf('\ncalculate_discretization süresi: %.3f s\n', toc);

% ---------- (1) Python'a karşı ----------
R    = load('ref_discretization.mat');
A_py = reshape(R.A_bar, n_x, n_x, K-1);     % Python: her sütun sütun-sütun düzleştirilmiş bir matris
B_py = reshape(R.B_bar, n_x, n_u, K-1);
C_py = reshape(R.C_bar, n_x, n_u, K-1);
e1 = [rel(Z.f_bar, R.f_bar); rel(Z.A_bar, A_py); rel(Z.B_bar, B_py); ...
      rel(Z.C_bar, C_py);    rel(Z.S_bar, R.S_bar); rel(Z.z_bar, R.z_bar)];
ad1 = {'f_bar', 'A_bar', 'B_bar', 'C_bar', 'S_bar', 'z_bar'};
fprintf('\n(1) Python calculate_discretization''a karşı (14 aralığın hepsi)\n');
for i = 1:6, fprintf('    %-6s  hata = %.1e   %s\n', ad1{i}, e1(i), durum(e1(i), 1e-10)); end

% ---------- (2) f_bar = integrate_multiple ----------
x_next = integrate_multiple(X, U, sig, p);
e2 = rel(Z.f_bar, x_next);
fprintf('\n(2) f_bar ile integrate_multiple\n');
fprintf('    hata = %.1e   %s\n', e2, durum(e2, 1e-12));

% ---------- (3) Taylor özdeşliği ----------
e3 = 0;
for k = 1:K-1
    lhs = Z.A_bar(:,:,k)*X(:,k) + Z.B_bar(:,:,k)*U(:,k) + Z.C_bar(:,:,k)*U(:,k+1) ...
        + Z.S_bar(:,k)*sig(1,k) + Z.z_bar(:,k);
    e3 = max(e3, rel(lhs, Z.f_bar(:,k)));
end
fprintf('\n(3) Taylor özdeşliği: A*x + B*u_k + C*u_k+1 + S*sigma + z = f_bar\n');
fprintf('    hata = %.1e   %s\n', e3, durum(e3, 1e-12));

% ---------- (4) Sapma testi ----------
dX = 1e-2*cos((1:n_x)'*(1:K));      % rastgele görünen ama tekrarlanabilir yönler
dU = 1e-2*sin((1:n_u)'*(1:K));
dS = 1e-1*cos(1:K-1);
fprintf('\n(4) Sapma testi (ölçekli birimler, 14 aralığın en kötüsü)\n');
fprintf('    %7s  %11s  %11s  %6s  %11s  %6s\n', 'eps', 'değişim', 'hata 1-14', 'oran', 'hata y', 'oran');
prev14 = NaN;  prev15 = NaN;
for e = [1 0.5 0.25 0.125 0.0625]
    tru = integrate_multiple(X + e*dX, U + e*dU, sig + e*dS, p);
    lin = zeros(n_x, K-1);
    for k = 1:K-1
        lin(:,k) = Z.f_bar(:,k) + Z.A_bar(:,:,k)*(e*dX(:,k)) + Z.B_bar(:,:,k)*(e*dU(:,k)) ...
                 + Z.C_bar(:,:,k)*(e*dU(:,k+1)) + Z.S_bar(:,k)*(e*dS(k));
    end
    d     = abs(tru - lin);
    chg   = max(abs(tru(1:14,:) - Z.f_bar(1:14,:)), [], 'all');
    err14 = max(d(1:14,:), [], 'all');
    err15 = max(d(15,:));
    fprintf('    %7.4f  %11.3e  %11.3e  %6.2f  %11.3e  %6.2f\n', e, chg, err14, prev14/err14, err15, prev15/err15);
    prev14 = err14;  prev15 = err15;
end

% ---------- (5) Kütle satırları ----------
am = p.alpha_m;
e5 = zeros(4, 1);
for k = 1:K-1
    e5(1) = max(e5(1), max(abs(Z.A_bar(1,:,k) - [1 zeros(1, n_x-1)])));
    e5(2) = max(e5(2), max(abs([Z.B_bar(1,1,k) Z.C_bar(1,1,k)] + am*sig(1,k)/2)));
    e5(3) = max(e5(3), max(abs([Z.B_bar(1,2:end,k) Z.C_bar(1,2:end,k)])));
    e5(4) = max(e5(4), abs(Z.S_bar(1,k) + am*(U(1,k) + U(1,k+1))/2));
end
ad5 = {'A_bar(1,:) = [1 0 ... 0]', 'B_bar(1,1) = C_bar(1,1) = -am*sig/2', ...
       'B_bar(1,2:5) = C_bar(1,2:5) = 0', 'S_bar(1) = -am*(T_k + T_k+1)/2'};
fprintf('\n(5) Kütle satırları (14 aralığın hepsi)\n');
for i = 1:4, fprintf('    %-38s hata = %.1e   %s\n', ad5{i}, e5(i), durum(e5(i), 1e-14)); end

% ---------- yerel fonksiyon ----------
function s = durum(e, tol)
    if e < tol, s = 'TUTTU'; else, s = 'KALDI'; end
end
