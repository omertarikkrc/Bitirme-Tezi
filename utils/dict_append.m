function d = dict_append(d, key, value)
% DICT_APPEND  Python'daki dict_append (hücre 13): d.(key) listesinin sonuna value ekler.
%   Sayılar satır vektöründe, yazılar (örneğin 'Note') hücre dizisinde tutulur.
%   Not: MATLAB alan adlarında '-' olamaz; Python'daki 'T-Ite' gibi anahtarlar
%   MATLAB'de 'T_Ite' olarak yazılmalı.

    if ischar(value) || isstring(value)
        value = {char(value)};
    end

    if ~isfield(d, key)
        d.(key) = value;
    else
        d.(key) = [d.(key), value];
    end
end
