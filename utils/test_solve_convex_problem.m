% --- hazırlık: fonksiyonun girdilerini taklit et ---
params    = rl_params_fcn();
params    = initialize_trajectory(params);
params    = rl_scale_params(params);
params    = rl_dynamics(params);
params.vehicle_cost_fcn = @rl_cost_fcn;
params.vehicle_cons_fcn = @rl_cons_fcn;
X_last     = params.X_last;
U_last     = params.U_last;
sigma_last = params.sigma_last;
w_tr       = params.w_ptr;

Z = calculate_discretization(X_last, U_last, sigma_last, params);
params.A_bar = Z.A_bar;  params.B_bar = Z.B_bar;  params.C_bar = Z.C_bar;
params.S_bar = Z.S_bar;  params.z_bar = Z.z_bar;



%%



[X_new, U_new, sigma_new, nu_new] = solve_convex_problem(X_last, U_last, sigma_last, w_tr, params);

% --- kontrol ---
sum(sigma_new)
X_new(1, end) * params.m_scale
U_new(1, [1 end]) * params.m_scale * params.r_scale / 1e6
sum(abs(nu_new(:)))
%%

% %% --- Parça 1: bilinmeyenler ---
% n_x = params.n_x;
% n_u = params.n_u;
% 
% % nedne " full" bütün elemanlar birbirinden bağımsız demek. kare matris
% % varsa bunu simetrik hale getirmesini engelliyor. x(1,2) = değil x(2,1).
% X  = sdpvar(n_x, params.K, 'full');
% U  = sdpvar(n_u, params.K, 'full');
% nu = sdpvar(n_x, params.K-1, 'full'); % sanal itki 
% dx = X - X_last;
% du = U - U_last;
% 
% 
% 
% %% 2. bölüm 	Süreleri (σ) tanımla ve S̄·σ terimini hazırla
% 
%  % --- Parça 2: süreler (σ) ve S̄·σ ---
% 
% 
% % constrains kısıt matrisim sürekli buna yeni kısıtlar eklenecek
% % constraints = [constraints, sigma >= 0]; zaman negatif olamaz bu ilk kısıt.
% 
% constraints = []; 
% 
% if params.free_final_time
%     if params.time_dil
%         sigma    = sdpvar(1, params.K - 1);  % 1x node sayısı-1 tane aralık var 
%         constraints = [constraints, sigma >= 0];        %  zaman negatif olamaz.
%         sig_time = sigma;
%         ds       = (sigma - sigma_last) / params.t_f * params.w_ds; %  bir önceki zaman tahmini ile fark al , lçeklendiriliyor ve ağırlıkla çarpılıyor
%         cp_sum   = sum(ds(1, :).^2); % tum ds  karesi toplanıyor
%     else% burara bütün aralıklardaki aynı süre harcansın durumunda gireceği döngü
%         sigma    = sdpvar(1, 1); % 1x1 lik optimize edilecek değişken tanımlama. 1x14 değil artık
%         constraints = [constraints, sigma >= 0]; 
%         sig_time = sigma * ones(1, params.K - 1);
%         ds       = (sigma - sigma_last) / params.t_f * params.w_ds;
%         cp_sum   = ds^2;
%     end
% 
%     % S_sigma her bir aralık için süreyi ne kadar değiştirirsem statem nasıl
%     % değişir params.S_bar da jacobian dF/dsigma
%     S_sigma = params.S_bar .* (ones(n_x, 1) * sig_time); 
% 
% 
% else % sabit uçuş süresi.
% 
%     sigma   = zeros(1);
%     cp_sum  = 0;
%     S_sigma = zeros(n_x, params.K - 1);
% end
% 
% 
% %% 3.bölüm Maliyeti kur: tasma + ν cezası
% 
% % biz bu maliyeti min yapmak istiyoruz.
% 
%     cost = w_tr * (sum(dx(:).^2) + sum(du(:).^2) + cp_sum); % Trust region penalization weight for the trust-region algorithm 1 de ağırlıklar adaptif ayarlanacak.
%     cost = cost + params.w_con_dyn * sum(abs(nu(:)));
% 
% %     test_cons = [X == X_last + 0.01, U == U_last, sigma == sigma_last + 0.21, nu == 0.001];
% % optimize(test_cons, cost, sdpsettings('solver', 'mosek', 'verbose', 0));
% % value(cost)
% 
% 
% 
% %% 4.bölüm 14 dinamik halkasını kısıt olarak yaz her bir adımı birbirine eşitle
% 
% 
% for k = 1:params.K - 1
%     x_lin = params.A_bar(:,:,k) * X(:, k)... %durumun doğrusal ötelenmiş hali
%           + params.B_bar(:,:,k) * U(:, k)... % kontrol girdisi 
%           + params.C_bar(:,:,k) * U(:, k+1)... % kontrol girdisi adım sonu
%           + S_sigma(:, k) ...% ne kadar süreceği
%           + params.z_bar(:, k)... % taylordan gelen sabit terimler.
%           + nu(:, k); % sanal itki                                
%     constraints = [constraints, X(:, k+1) == x_lin];  %  bir sonraki düğüm oraya denk gelmeli kısıtı da EKLENDİ artık.
% end
% 
% 
% % % --- kontrol (Parça 4) ---
% % test_cons = [constraints, X == X_last, U == U_last, sigma == sigma_last];
% % optimize(test_cons, cost, sdpsettings('solver', 'mosek', 'verbose', 0));
% % nu_beklenen = X_last(:, 2:end) - Z.f_bar;
% % max(abs(value(nu) - nu_beklenen), [], 'all')
% % value(cost)
% 
% 
% 
% %% 5.bölüm Araç maliyetini ve kısıtlarını ekle, MOSEK'le çöz, sonucu döndür
% 
% cost        = cost + params.vehicle_cost_fcn(X, U, sigma, X_last, U_last, sigma_last, nu, params,false);   %  toplam süre eklendi.
% constraints = [constraints, params.vehicle_cons_fcn(X, U, sigma, X_last, U_last, sigma_last, params,false)];  % + araç kısıtlarını  ekledik 
% 
% %sol = optimize(constraints, cost, sdpsettings('solver', 'mosek', 'verbose', 1));
% 
% ops = sdpsettings('solver', 'mosek', 'verbose', 0, ...
%     'mosek.MSK_DPAR_INTPNT_CO_TOL_REL_GAP', 1e-12, ...
%     'mosek.MSK_DPAR_INTPNT_CO_TOL_PFEAS',   1e-12, ...
%     'mosek.MSK_DPAR_INTPNT_CO_TOL_DFEAS',   1e-12, ...
%     'mosek.MSK_DPAR_INTPNT_QO_TOL_REL_GAP', 1e-12, ...
%     'mosek.MSK_DPAR_INTPNT_QO_TOL_PFEAS',   1e-12, ...
%     'mosek.MSK_DPAR_INTPNT_QO_TOL_DFEAS',   1e-12);
% sol = optimize(constraints, cost, ops);
% 
% if sol.problem ~= 0
%     disp(repmat('*', 1, 50));
%     disp([repmat('-', 1, 20) 'WARNING' repmat('-', 1, 20)]);
%     disp(['Problem status: ' sol.info]);
%     disp([repmat('-', 1, 20) 'WARNING' repmat('-', 1, 20)]);
%     disp(repmat('*', 1, 50));
% end
% 
% %YALMIP'te değer çekmek için value() kullanıyoruz.
% X_new  = value(X);
% U_new  = value(U);
% if params.free_final_time
%     sigma_new = value(sigma); % değişken YALMIP değişkeni classdan bakarsın (14 sayı içeren vektör)
% else
%     sigma_new = sigma; % burda sabit değer (0 atanıyor yukarıda tanımlamıştık ) 
% end
% nu_new = value(nu);
% 
% 
% 
% 
% % --- kontrol (Parça 5) ---
% sol.info
% sum(sigma_new)                                              % toplam uçuş süresi [s]
% X_new(1, end) * params.m_scale                              % son kütle [kg]
% U_new(1, [1 end]) * params.m_scale * params.r_scale / 1e6   % ilk ve son düğümde itki [MN]
% sum(abs(nu_new(:)))                                         % toplam hayali itici
% value(cost)                                                 % maliyet