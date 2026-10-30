function [vehicle_cost, cost_dict] = rl_cost_fcn(X, U, sigma, X_last, U_last, sigma_last, nu, params, npy, cost_dict)
% RL_COST_FCN  Araç maliyeti (Python hücre 41, rl_cost_fcn).
%   params.cost_type:
%     'time'  : w_sigma * sum(sigma) / t_f               (orijinal: en kısa süre)
%     'fuel'  : w_m * (m_wet - m_K) / (m_wet - m_dry)    (deney: en az yakıt)
%     'mixed' : ikisinin toplamı                         (deney)
%   X, U, sigma sayı (double) da olabilir, YALMIP değişkeni (sdpvar) da.
%   MATLAB'de norm ikisinde de çalıştığı için npy'ye göre ayrı satır gerekmiyor
%   (Python numpy ve cvxpy için iki ayrı satır yazıyor); npy imza uyumu için duruyor.
%   cost_dict verilirse (10. girdi) maliyet parçaları kayda eklenir.

    log_on = (nargin >= 10);

    % --- süre maliyeti (orijinal) ---
    sigma_cost = 0;
    if params.free_final_time
        if params.time_dil
            sigma_cost = params.w_sigma * norm(sigma(1, :), 1) / params.t_f;
        else
            sigma_cost = params.w_sigma * norm(sigma, 1) / params.t_f;
        end
    end

    % --- yakıt maliyeti (deney): harcanan yakıt / toplam yakıt ---
    fuel_cost = params.w_m * (params.m_wet - X(1, end)) / (params.m_wet - params.m_dry);

    switch params.cost_type
        case 'time'
            vehicle_cost = sigma_cost;
        case 'fuel'
            vehicle_cost = fuel_cost;
        case 'mixed'
            vehicle_cost = sigma_cost + fuel_cost;
        otherwise
            error('rl_cost_fcn: bilinmeyen cost_type ''%s''', params.cost_type);
    end

    % --- kayıt (Python'daki cost_dict) ---
    if log_on
        if params.free_final_time
            cost_dict = dict_append(cost_dict, 'sigma_cost', sigma_cost);
        end
        if ~strcmp(params.cost_type, 'time')
            cost_dict = dict_append(cost_dict, 'fuel_cost', fuel_cost);
        end
        dyn_cost  = params.w_con_dyn * norm(nu(:), 1);
        cost_dict = dict_append(cost_dict, 'dyn_cost', dyn_cost);
    end
end
