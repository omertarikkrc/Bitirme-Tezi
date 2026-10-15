function f = dynamics(x,u,params)
% durum vektörü içinden parçaları ayıkla

    m   = x(1);            % kütle
    r_I = x(2:4);          % konum 
    v_I = x(5:7);          % hız 
    q   = x(8:11);         % quaternion 
    w_B = x(12:14);        % açısal hız 

 % --- kontrol vektörünü parçalarına ayır ---


    T       = u(1);     % itki büyüklüğü
    delta_e = u(2);     % motorun eğilme açısı
    phi_e   = u(3);     % motorun eğilme yönü



 % --- çerçeveler arası çevirmenler ---
    CBI = CBI_fcn(q);                 % yer -> roket
    CIB = transpose(CBI) ;            % roket -> yer

   f = zeros(params.n_x, 1);          % params.nx = 15 parametre sayısı 
   f(1)=-params.alpha_m*T;            % kütle değişimi
   f(2:4)= v_I;                       % konumun değişimi yani hız.



    v_norm = (v_I(1)^2+v_I(2)^2+v_I(3)^2+1e-8)^0.5;                                 % hızın büyüklüğü 1e-8 türevinin patlamaması için
    A_B    = -0.5*params.rho_air*v_norm*params.S_area*params.C_aero*CBI*v_I;        % hava kuvveti, roket çerçevesinde (3x1)
    T_v    = T*[sin(delta_e)*cos(phi_e);sin(delta_e)*sin(phi_e);cos(delta_e)] ;     % itki vektörü, roket çerçevesinde (3x1)
    f(5:7) = 1/m *CIB*(A_B+T_v)+params.g_I;   % kuvvet                                       % ivme, yer çerçevesinde
    f(8:11) = 0.5*omega(w_B)*q;               % quarterniyon değişim hızı
    J_B_m =   m*params.J_B;                   %kütle ile ölçeklendirilmiş atalet
    J_B_inv_m =params.J_B_inv/m;              % yukardaki matrisin tersi
    

    %torklar
    tau_thrust=skew(params.r_cm) * T_v ;     %itkiden kaynaklı tork
    tau_aero=skew(params.r_cp)*A_B;       % aerodinamik kuvveterden gelen tork
    tau_gyro=skew(w_B)*(J_B_m*w_B) ;% kendi etrafında dönmekten gelen tork terimi


    %açısal ivmeleri ekledik 
    f(12:14)=J_B_inv_m*(tau_thrust+tau_aero-tau_gyro);

    % --- normal kısıtlar ---
    tilt_ang_cons_pre = params.w_stc_1*(sqrt(q(2)^2+q(3)^2+ 1e-8)-sqrt((1-cosd(params.theta_max))/2))
    tilt_ang_cons     = max(0,tilt_ang_cons_pre)^2
    ang_vel_cons_pre  = params.w_stc_1*(sqrt(w_B(1)^2+w_B(2)^2+w_B(3)^2+1e-8)-params.w_B_max)
    ang_vel_cons      = max(0,ang_vel_cons_pre)^2
    gs_cons_pre       = tand(params.gs_max)*sqrt(r_I(1)^2+r_I(2)^2+1e-8)-r_I(3)
    gs_cons           = params.w_stc_1*max(0,gs_cons_pre)^2
    % 
    f(15) = tilt_ang_cons+ang_vel_cons+gs_cons


end