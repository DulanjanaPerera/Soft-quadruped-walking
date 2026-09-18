function p_pos_norm = m20250911_2_PositivePressures(p)

minP = min(p);
maxP = max(p);

p_pos = p-minP;
% p_pos_norm = p_pos / (maxP - minP) * maxP; % Normalize the positive pressures

p_pos_norm = p_pos/max(p_pos) * maxP;

end