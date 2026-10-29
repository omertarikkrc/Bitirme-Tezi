function params = rl_dynamics(params)
% RL_DYNAMICS  Dinamiği ve Jacobian'larını CasADi fonksiyonu olarak params'a ekler


    % import ettik
    import casadi.*
    

    %durum, kontrol ve dinamiğin sembolik olarak tanımlanması

    xs = SX.sym('x', params.n_x);              % sembolik durum
    us = SX.sym('u', params.n_u);              % sembolik kontrol
    fs = dynamics(xs, us, params);             % dinamik
%%Function() sembolik fonksiyon yazmıyor, tersini yapıyor: elimizdeki sembolik ifadeyi sayı verip sayı alınabilen bir fonksiyona çeviriyor.

    %fs förmülünü yani sembolik ifadeyyi 
    % sayısal bir fonksiyona çevrilmiş girdileri ise
    %{xs, us} adı da f_func bu da structure koymuş.
    params.f_func = Function('f_func', {xs, us}, {fs});

    % burada da aynı mantık jacobian(fs, xs) bu çıktımız ama förmül halde
    % olan şeyi fonksiyona çeviriyoruz.
    %girdiler {xs, us}
    % A_func adı params structuna kaydediliyor
    params.A_func = Function('A_func', {xs, us}, {jacobian(fs, xs)});   % 15x15 = df/dx
    % yukarıdaki ile aynı mantık.
    params.B_func = Function('B_func', {xs, us}, {jacobian(fs, us)});   % 15x5  = df/du
end