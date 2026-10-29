%% test_dVdt.m
% dVdt.m (adım 7.3) üç yoldan sınanır:
%   (1) tau = 0, V = V0 : elle bilinen sonuçlar (Phi' = sigma*A, Bm' = sigma*B, Bp' = 0, ...)
%   (2) tau = 1, V = V0 : FOH ağırlıkları yer değiştirir (Bm' = 0, Bp' = sigma*B)
%   (3) Python dVdt'ye karşı: dolu bir V ile tau = 0.37
% Ölçüt: göreli hata |a-b|/max(|b|,1) < 1e-12

clear; format long g
params    = rl_params_fcn();
params    = initialize_trajectory(params);
params_sc = rl_scale_params(params);
params_sc = rl_dynamics(params_sc);
p   = params_sc;
n_x = p.n_x;  n_u = p.n_u;
rel = @(a, b) max(abs(a(:) - b(:)) ./ max(abs(b(:)), 1));

% --- ortak girdiler: ilk tahminin 1. aralığı, u1 bilerek farklı yapıldı ---
x_k = p.X_last(:,1);
u0  = p.U_last(:,1);
u1  = p.U_last(:,2) + [-0.1*p.U_last(1,2); 0.05; 0.3; 0.02; 0.4];
sig = p.sigma_last(1);
V0  = [x_k; reshape(eye(n_x), [], 1); zeros(p.i6 - p.i2, 1)];

% ---------- (1) tau = 0 : u = u0, alpha = 1, beta = 0 ----------
A0 = sig*full(p.A_func(x_k, u0));  B0 = sig*full(p.B_func(x_k, u0));  f0 = full(p.f_func(x_k, u0));
[dx, dPhi, dBm, dBp, dS, dz] = unpack_V(dVdt(V0, 0, u0, u1, sig, 0, p), p);
e1 = [rel(dx, sig*f0); rel(dPhi, A0); rel(dBm, B0); rel(dBp, zeros(n_x, n_u)); rel(dS, f0); rel(dz, -A0*x_k - B0*u0)];
ad1 = {'dx   = sigma*f', 'dPhi = sigma*A', 'dBm  = sigma*B', 'dBp  = 0', 'dS   = f', 'dz   = -A*x - B*u'};
fprintf('\n(1) tau = 0, V = V0\n');
for i = 1:6, fprintf('    %-20s hata = %.1e   %s\n', ad1{i}, e1(i), durum(e1(i))); end

% ---------- (2) tau = 1 : u = u1, alpha = 0, beta = 1 ----------
A1 = sig*full(p.A_func(x_k, u1));  B1 = sig*full(p.B_func(x_k, u1));  f1 = full(p.f_func(x_k, u1));
[dx, dPhi, dBm, dBp, dS, dz] = unpack_V(dVdt(V0, 1, u0, u1, sig, 0, p), p);
e2 = [rel(dx, sig*f1); rel(dPhi, A1); rel(dBm, zeros(n_x, n_u)); rel(dBp, B1); rel(dS, f1); rel(dz, -A1*x_k - B1*u1)];
ad2 = {'dx   = sigma*f', 'dPhi = sigma*A', 'dBm  = 0', 'dBp  = sigma*B', 'dS   = f', 'dz   = -A*x - B*u'};
fprintf('\n(2) tau = 1, V = V0\n');
for i = 1:6, fprintf('    %-20s hata = %.1e   %s\n', ad2{i}, e2(i), durum(e2(i))); end

% ---------- (3) Python'a karşı: dolu V, tau = 0.37 ----------
x2    = p.X_last(:,2);
Phi_t = eye(n_x) + 1e-3*reshape(1:n_x*n_x, n_x, n_x);
Bm_t  =  1e-3*reshape(1:n_x*n_u, n_x, n_u);
Bp_t  = -2e-3*reshape(1:n_x*n_u, n_x, n_u);
S_t   =  1e-2*(1:n_x)';
z_t0  = -1e-2*(1:n_x)';
V_t   = [x2; Phi_t(:); Bm_t(:); Bp_t(:); S_t; z_t0];
[dx, dPhi, dBm, dBp, dS, dz] = unpack_V(dVdt(V_t, 0.37, u0, u1, sig, 0, p), p);

WV = (1:n_x)';  WP = reshape(1:n_x*n_x, n_x, n_x);  WB = reshape(1:n_x*n_u, n_x, n_u);
ML = [norm(dx); norm(dPhi, 'fro'); norm(dBm, 'fro'); norm(dBp, 'fro'); norm(dS); norm(dz); ...
      sum(dx.*WV); sum(dPhi.*WP, 'all'); sum(dBm.*WB, 'all'); sum(dBp.*WB, 'all'); sum(dS.*WV); sum(dz.*WV)];
PY = [0.38085716562695088; 7.9447245907388533; 10.325627689381125; 6.0016051170511275; ...
      0.45986971352517003; 0.62360142674983954; -7.140740180919793; 2793.2142835815584; ...
      -195.9774464171914; -301.58895774006129; -0.19499276764284623; 9.1950271217932382];
ad3 = {'norm dx', 'norm dPhi', 'norm dBm', 'norm dBp', 'norm dS', 'norm dz', ...
       'ağırlıklı dx', 'ağırlıklı dPhi', 'ağırlıklı dBm', 'ağırlıklı dBp', 'ağırlıklı dS', 'ağırlıklı dz'};
fprintf('\n(3) Python dVdt''ye karşı (tau = 0.37)\n');
fprintf('    %-16s %22s %22s %9s\n', '', 'MATLAB', 'Python', 'hata');
for i = 1:12
    e = rel(ML(i), PY(i));
    fprintf('    %-16s %22.15g %22.15g %9.1e   %s\n', ad3{i}, ML(i), PY(i), e, durum(e));
end

% ---------- yerel fonksiyonlar ----------
function [dx, dPhi, dBm, dBp, dS, dz] = unpack_V(dV, p)
    dx   = dV(p.i0+1:p.i1);
    dPhi = reshape(dV(p.i1+1:p.i2), p.n_x, p.n_x);
    dBm  = reshape(dV(p.i2+1:p.i3), p.n_x, p.n_u);
    dBp  = reshape(dV(p.i3+1:p.i4), p.n_x, p.n_u);
    dS   = dV(p.i4+1:p.i5);
    dz   = dV(p.i5+1:p.i6);
end

function s = durum(e)
    if e < 1e-12, s = 'TUTTU'; else, s = 'KALDI'; end
end
