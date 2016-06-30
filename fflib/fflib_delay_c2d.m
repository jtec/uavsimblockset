% Computes the delay in samples for a given continuous time delay
% and a given sample time. The discrete delay is always larger than or
% equals the contiuous delay.
function samples = fflib_delay_c2d(tdelay, tsample)
    samples = ceil(tdelay/tsample);
end
