function cost_dict = scp_non_linear_cost(X_new, U_new, sigma_new, X_last, U_last, sigma_last, nu_new, nl_nu_new, w_tr, params, cost_dict)
% SCP_NON_LINEAR_COST  Doğrusal (MOSEK'in vaat ettiği) ve gerçek (RK4) maliyet (Python hücre 22).
%   10. adımda rho = gerçek iyileşme / vaat edilen iyileşme hesabı için kullanılır.
%   Sonuçlar cost_dict'e eklenir: ptr_cost, lcd, nlcd, lc, nlc.

    dx = X_new - X_last;
    du = U_new - U_last;

    if params.free_final_time
        ds = (sigma_new - sigma_last) / params.t_f * params.w_ds;
        if params.time_dil
            cp_sum = norm(ds(1, :))^2;
        else
            cp_sum = abs(ds)^2;
        end
    else
        cp_sum = 0;
    end

    % tasma: sütun normlarının kareleri toplamı
    ptr_cost = w_tr * (sum(vecnorm(dx).^2) + sum(vecnorm(du).^2) + sum(cp_sum));

    % hayali itici cezası: doğrusal (nu) ve gerçek (nl_nu)
    lcd  = params.w_con_dyn * norm(nu_new(:), 1);
    nlcd = params.w_con_dyn * norm(nl_nu_new(:), 1);

    % araç maliyeti
    lin_vehicle_cost = params.vehicle_cost_fcn(X_new, U_new, sigma_new, X_last, U_last, sigma_last, nu_new, params, true);
    [n_lin_vehicle_cost, cost_dict] = params.vehicle_cost_fcn(X_new, U_new, sigma_new, X_new, U_new, sigma_new, nl_nu_new, params, true, cost_dict);

    lin_cost   = lcd  + lin_vehicle_cost + ptr_cost;
    n_lin_cost = nlcd + n_lin_vehicle_cost;

    cost_dict = dict_append(cost_dict, 'ptr_cost', ptr_cost);
    cost_dict = dict_append(cost_dict, 'lcd',      lcd);
    cost_dict = dict_append(cost_dict, 'nlcd',     nlcd);
    cost_dict = dict_append(cost_dict, 'lc',       lin_cost);
    cost_dict = dict_append(cost_dict, 'nlc',      n_lin_cost);
end