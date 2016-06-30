function vsat = sat( v, lower, upper )

vsat = v;
for k=1:length(v)
    if v(k) > upper(k)
        vsat(k) = upper(k);
    elseif v(k) < lower(k)
        vsat(k) = lower(k);
    end
end
end

