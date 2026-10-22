
% bu fonksiyon kontrol girdilerinin 2 nokta arasında nasıl değiştiğini
% tanımlıyor.bunu sonra rk4 e gönderip sayısal integrasyon yapacağız.

function f = dxdt(x, t, tt, tf, u_k, u_kp1, params)
% DXDT  RK4'e verilen türev (Python: dxdt_jax, hücre 17)
%   t     : aralığın içindeki zaman (0 ... tf)
%   tt    : aralığın uçuştaki başlangıç zamanı (Python'da da kullanılmıyor)
%   tf    : aralığın süresi (sigma_k)
%   u_k, u_kp1 : aralığın başındaki ve sonundaki kontrol

    if strcmp(params.inp_param, 'FOH')
        u = u_k + (t/tf)*(u_kp1 - u_k);      % doğrusal geçiş (rampa) 
    elseif strcmp(params.inp_param, 'ZOH')
        u = u_k;                              % sabit tut (merdiven)
    end
    f = dynamics(x, u, params);
end

