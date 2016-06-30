% Makes one or more states of a state space model inputs of the model.
function ssmu = x2u(ssm, ix)
    % Remaining states:
    otherx = 1:length(ssm.a);
    otherx(ix) = [];
    % Impact of states on other states:
    xonx = ssm.a(otherx, ix);
    b = [ssm.b(otherx, :) xonx];
    a = ssm.a(otherx, otherx);
    c = ssm.c(:, otherx);
    d = [ssm.d ssm.c(:, ix)];
    ssmu = ss(a, b, c, d);
    ssmu.StateName = ssm.StateName(otherx);
    ssmu.InputName = [ssm.InputName; ssm.StateName(ix)];
    ssmu.OutputName = ssm.OutputName;
end