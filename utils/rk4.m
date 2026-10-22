function y = rk4(func, y0, tf, steps, varargin)

%   func  : türev fonksiyonu, func(y, t, ek argümanlar...) -> dy/dt (sütun)
%   y0    : başlangıç değeri (sütun)
%   tf    : integrasyon süresi
%   steps : NOKTA sayısı (adım sayısı = steps - 1)
%   y     : n x steps, her sütun bir zaman noktası

    h = tf/(steps - 1);                  % sabit adım boyu
    y = zeros(numel(y0), steps);
    y(:, 1) = y0;

    for i = 1:steps-1
        t  = (i-1)*h;                    
        yi = y(:, i);
        k1 = h*func(yi,          t,         varargin{:});
        k2 = h*func(yi + 0.5*k1, t + 0.5*h, varargin{:});
        k3 = h*func(yi + 0.5*k2, t + 0.5*h, varargin{:});
        k4 = h*func(yi + k3,     t + h,     varargin{:});
        y(:, i+1) = yi + (k1 + 2*k2 + 2*k3 + k4)/6;
    end
end