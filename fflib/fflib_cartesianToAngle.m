% Returns the angle that would correspond to a given position vector in a
% cartesian frame after converting to polar coordinates (positive clockwise
% from the x axis, in radians, [0, 2*pi]. Helpful e.g. to compute the
% trajectory azimuth chi from a given trajectory speed in an NED frame.
function angle = fflib_cartesianToAngle(x, y)
    angle = atan2(y, x);
end

