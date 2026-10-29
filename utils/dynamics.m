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
    delta_b = u(4); 
    phi_b = u(5);



 % --- çerçeveler arası çevirmenler ---
    CBI = CBI_fcn(q);                 % yer -> roket
    CIB = transpose(CBI) ;            % roket -> yer


    %%casadi e göre değiştirdik tanımlamayı 

    if isa(x, 'casadi.SX')
        f = casadi.SX.zeros(params.n_x, 1);   % sembolik çağrı (Jacobian için) casadi 
    else
        f = zeros(params.n_x, 1);             % sayısal çağrı (eskisi gibi)  eski hali 
    end



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

    %tilt açısı 
    tilt_ang_cons_pre = params.w_stc_1*(sqrt(q(2)^2+q(3)^2+ 1e-8)-sqrt((1-cosd(params.theta_max))/2));
    tilt_ang_cons     = max(0,tilt_ang_cons_pre)^2;

    %açısal hız 
    ang_vel_cons_pre  = params.w_stc_1*(sqrt(w_B(1)^2+w_B(2)^2+w_B(3)^2+1e-8)-params.w_B_max);
    ang_vel_cons      = max(0,ang_vel_cons_pre)^2;

    %glide scope
    gs_cons_pre       = tand(params.gs_max)*sqrt(r_I(1)^2+r_I(2)^2+1e-8)-r_I(3);
    gs_cons           = params.w_stc_1*max(0,gs_cons_pre)^2;


    %irtifa tetiği: h < 100 m iken > 0
    alt_stc_trig_i = params.w_stc_1 *max(0,params.alt_stc_trig-r_I(3))^2;
    %marjlı 1.1 ile çarpılmış halini eklendi yani 110 metreden 
    alt_stc_trig_i_alpha = params.w_stc_1 *max(0,params.alt_stc_trig*params.alpha_alt_stc_trig-r_I(3))^2;
    
    %2m irtifa tetik
    alt_stc_trig_f=params.w_stc_1 *max(0,r_I(3)-params.alt_stc_trig_los_gs_done)^2;


    %motor açısı kısıt +- delta_stc_cons kadar açılcak
    gimbal_ang_stc_cons_pre_1 = delta_e-deg2rad(params.delta_stc_cons);
    gimbal_ang_stc_cons_pre_2 = -delta_e-deg2rad(params.delta_stc_cons);


    %motor açısı ihlal ceza
    gimbal_ang_stc_cons_1 =params.w_stc_1*max(0,gimbal_ang_stc_cons_pre_1)^2;
    gimbal_ang_stc_cons_2 =params.w_stc_1*max(0,gimbal_ang_stc_cons_pre_2)^2;
    gimbal_ang_stc_cons = gimbal_ang_stc_cons_1 + gimbal_ang_stc_cons_2;

    

    %gimbal stc 100 metre altında VE motorlar 1 dereceden fazla eğik mi ?
    gimbal_ang_stc=alt_stc_trig_i*gimbal_ang_stc_cons;

    
    %eğim kısıtı 5 derece stc bu 100 metre altında 
    tilt_ang_stc_cons_pre =(sqrt(q(2)^2+q(3)^2+ 1e-8)-sqrt((1-cosd(params.theta_stc_cons*params.alpha_theta_stc_cons))/2));
    tilt_ang_stc_cons = params.w_stc_2*max(0,tilt_ang_stc_cons_pre)^2;
    %VE bağlacı 100 metre altında 5 dereceyi geçmesin ()
    tilt_ang_stc = alt_stc_trig_i * tilt_ang_stc_cons;

    %glidescope kısıt ve ceza 
    gs_stc_cons_pre = tand(params.gs_stc_cons* params.alpha_gs_stc_cons)*sqrt(r_I(1)^2+r_I(2)^2+1e-8)-r_I(3);
    gs_stc_cons     = params.w_stc_1*max(0,gs_stc_cons_pre)^2 ;
    %100-2 metre arasında çalışcak ve ceza hepsi VE bağlacı ile bağlanıyor.
    gs_stc = alt_stc_trig_i * alt_stc_trig_f * gs_stc_cons;

    

    % açısal hız norm aldık max değerden çıkardık max aldık eğer
    % omega_stc_cons_pre + ise max da o çıkacak ceza olarak gelecek.
    omega_stc_cons_pre = (w_B(1)^2 + w_B(2)^2 + w_B(3)^2 + 1e-8)^(0.5) - deg2rad(params.omega_stc_cons) * params.alpha_omega_stc_cons;
    omega_stc_cons = params.w_stc_1 * max(0., omega_stc_cons_pre)^2;


    %speed açısal hıza benziyor

   
    spd_stc_cons_pre = (v_I(1)^2 + v_I(2)^2+ v_I(3)^2 + 1e-8)^(0.5) - params.alpha_spd_stc_cons * params.spd_stc_cons;
    spd_stc_cons = params.w_stc_3 * max(0., spd_stc_cons_pre)^2;


    %şimdi açısal hızı ve hızı stc formuna sokcaz. 110 metreden itibaren
    % neden topladık ?
    % sıfır olması ikisinin de sıfır olması lazım. bunu anla bi    
    % alt_stc_trig_i_alpha bizim 110 metre triggeri var yok. roket açısal
    % hızı az ama hızı çok ne olacaktı ? bu yüzden toplama var.
    
    spds_stcs = alt_stc_trig_i_alpha * (omega_stc_cons + spd_stc_cons);




 

    %%   % line of sight 
        % --- LOS: kameranın baktığı yön ---
    l_B = [sin(delta_b)*cos(phi_b);
           sin(delta_b)*sin(phi_b);
           cos(delta_b)];                    % gövde ekseninde yön (delta_b = 0 -> [0;0;1], burun)
    % eksi: kamera alttan aşağı bakar; CIB: gövde -> yer her şeyi inertial frame e göre yazıyoruz.
    los_I = -CIB*l_B    ;                     
    los_I_norm = sqrt(los_I(1)^2 + los_I(2)^2 + los_I(3)^2 + 1e-8);

    % --- LOS kuralı: iniş noktası görüş konisinde mi? ---
     % roketle iniş noktası arasındaki uzaklık konum norm
    los_pos_norm = sqrt(r_I(1)^2 + r_I(2)^2 + r_I(3)^2 + 1e-8);   


    %0.98*5 derecelik bir konini içinde mi ? 2 tane dot producty farkını
    %alıyor eğer dot(-r_I, los_I) büyükse - çıkacak max alındığında 0
    %çıkacak ceza yok aksi halde pozitif değer çıkacak ve ceza terimi
    %çıkacak. acos kullanmadık 0 derecede sonsuza gidiyor.

    los_stc_cons_pre = los_pos_norm*los_I_norm*cos(params.alpha_los_stc_cons*params.los_stc_cons*pi/180) - dot(-r_I, los_I);

    los_stc_cons = params.w_stc_3*max(0, los_stc_cons_pre)^2;

    alt_stc_trig_los = params.w_stc_1*max(0, params.alpha_alt_alt_stc_trig_los*params.alt_stc_trig_los - r_I(3))^2;

    los_stc = alt_stc_trig_los*alt_stc_trig_f*los_stc_cons;
%% Hız ve Eğim trigger
%roket yavaş VE dik ise itki dar bir aralıkta olsun
%hızlı VEYA eğik ise itki normal bir aralıkta olsun
    %hız norm
    spd_stc_trig_i = params.w_stc_2*max(0, sqrt(v_I(1)^2 + v_I(2)^2 + v_I(3)^2 + 1e-8) - params.spd_stc_trig)^2;
    spd_stc_trig_f = params.w_stc_2*max(0, params.spd_stc_trig - sqrt(v_I(1)^2 + v_I(2)^2 + v_I(3)^2 + 1e-8))^2;

    %eğiklik 
    theta_stc_trig_i = params.w_stc_2*max(0,  sqrt(q(2)^2 + q(3)^2 + 1e-8) - sqrt((1 - cosd(params.theta_stc_trig))/2))^2;
    theta_stc_trig_f = params.w_stc_2*max(0, -sqrt(q(2)^2 + q(3)^2 + 1e-8) + sqrt((1 - cosd(params.theta_stc_trig))/2))^2;


    % thrust stcler

% claude atı düz vadi analojisi yaptı alt sınırda  kısıt aşımı küçük
% olmasından meydana gelir üst sınırda ise fazla olmasından.
    thrust_min_f = params.w_stc_3*max(0, params.T_min_aft - T)^2;
    thrust_max_f = params.w_stc_3*max(0, T - params.T_max_aft)^2;

    thrust_min_i = params.w_stc_3*max(0, params.alpha_T_min*params.T_min - T)^2;
    thrust_max_i = params.w_stc_3*max(0, T - params.T_max)^2;


    %en bombastik bölüm de morgan var burada biraz anlaması zor VE ve VEYA
    %

    thrust_stc_f = (spd_stc_trig_f*theta_stc_trig_f)*(thrust_max_f + thrust_min_f);
    thrust_stc_i = (spd_stc_trig_i + theta_stc_trig_i)*(thrust_max_i + thrust_min_i);
    
    % şu an anladım ama ileride anlat derlerse anlatamayabilirim kalem
    % tahtada anlaman lazım.

    %




%%



    % 15. değşken kısıtların aşımını topluyor 
    
    f(15) = tilt_ang_cons + ang_vel_cons + gs_cons + gimbal_ang_stc + tilt_ang_stc + gs_stc + spds_stcs + los_stc + thrust_stc_f + thrust_stc_i;








end