function blend = quintic_blend(fraction)
%QUINTIC_BLEND Smoothly map a normalized fraction to [0, 1].

t = max(0, min(1, fraction));
blend = 10 .* t.^3 - 15 .* t.^4 + 6 .* t.^5;
end
