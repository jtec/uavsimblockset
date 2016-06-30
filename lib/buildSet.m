% Recursively samples (grid-style) a set of parameters definded by nominal value, upper and lower
% bounds.
% \param nominal - vector of nominal parameter values
% \param lbounds - vector of lower bounds parameter
% \param ubounds - vector of upper bounds on parameters
% \param nSamples - vector, indicates how many times each parameters should
% be sampled.
%
% Jan Bolting, ONERA/ISAE, 2014
function set = buildSet( nominal, lbounds, ubounds, nSamples )

nTotal = prod(nSamples);
set = zeros(nTotal, length(nominal));

theta = zeros(size(nominal));
for i=1:length(nominal)
    if nSamples(i) == 1
        % If the parameter is supposed to be sampled only once, take the
        % nominal value.
        theta(i) = nominal(i);
    else
        % If the parameter is supposed to be sampled only once, work your
        % way up from the lower bound.
        theta(i) = lbounds(i);
    end
end
total = 0;
sample(1);

% Samples the parameter indicated by iParam. Function recursively calls
% itself to sample all parameters.
    function sample(iParam)
        % Iterate over grid points
        for iS=1:nSamples(iParam)
            % Compute sample value
            if nSamples(iParam) > 1
                delta = (ubounds(iParam) - lbounds(iParam)) / (nSamples(iParam)-1);
                theta(iParam) = lbounds(iParam) + (iS-1) * delta;
            else
                theta(iParam) = nominal(iParam);
            end
            % Sample next parameter:
            if iParam < length(nominal)
                sample(iParam+1);
            end
            % The last parameter saves the sample vector:
            if iParam == length(nominal)
                total = total + 1;
                set(total, :) = theta;
            end
        end
        % disp([mfilename '>> done sampling parameter ' num2str(iParam)]);
    end

end
