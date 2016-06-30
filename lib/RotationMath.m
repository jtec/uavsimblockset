% Static class that holds methods for everything related to spatial
% rotation.
% \author Jan Bolting
%
classdef RotationMath
    properties
    end
    
    methods (Static)
        % Computes quaternion fom given rotation axis and roation angle [radians].
        % Quaternion order: qw, qx, qy, qz.
        function q = angleaxis2quat( axis, angle )
            qw = cos(angle/2);
            qxyz = sin(angle/2) * fflib_normalize(axis);
            q = [qw; qxyz];
            q = quatnormalize(q')';
        end
        
        % Returns the matrix representation of a given quaternion for
        % quaternion multiplication. Quaternion order: [qw, qx, qy, qz]'.
        function m = quat2matrix(q)
            m = [q(1) -q(2:4)';
                q(2:4) q(1)*eye(3) + crossproductmatrix(q(2:4))];
            
            function c = crossproductmatrix(x)
                c = [0 -x(3) x(2);
                    x(3) 0 -x(1);
                    -x(2) x(1) 0];
            end
            
        end
        
        % Returns the equivalent euler angles (3 x n; order [psi, theta, phi]) for
        % a given rotation quaternion (4xn, order [w, x, y, z]).
        function eulers = quat2euler( q )
            eulers = zeros(3, size(q, 2));
            [yaw, pitch, roll] = quat2angle(q', 'ZYX');
            eulers = [yaw pitch roll]';
        end
        
        % Returns the equivalent unit quaternion (4xn) for a given rotation
        % vector (3xn).
        function q = rotv2quat( rotv )
            q = zeros(4, size(rotv, 2));
            for i=1:size(rotv, 2)
                length = norm(rotv(:,i));
                if(length < eps)
                    q(1:4, i) = [1 0 0 0]';
                else
                    q(1:4, i) = [cos(length/2); (rotv(:,i) ./ length) * sin(length/2)];
                end
            end
        end
        
        % Returns the equivalent rotation vector (3 x n; order [rx, ry, rz]) for
        % a given rotation quaternion (4xn, order [qw, qx, qy, qz]).
        function r = quat2rotv( q )
            r = zeros(3, size(q, 2));
            for i=1:size(q, 2)
                vectorPartNorm = norm(q(2:4, i));
                if(vectorPartNorm < eps)
                    r(1:3, i) = [0 0 0]';
                else
                    r(1:3, i) = 2 * acos (q(1, i)) * q(2:4, i)/vectorPartNorm;
                end
            end
        end
        
        % Computes rotation vector rates from body rotation rates.
        % See "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation
        % Vectors", Diebel, Stanford, 2006, section 7.11.
        function rDot = rotRates2rvRates( omega, rotv )
            %  The factors and matrices used here are defined in equations
            % a: eq. 210
            % G: eq. 212
            % W: eq. 237
            % V: eq. 261
            v = norm(rotv);
            if (v < eps)
                G = (1/4) * [-rotv'; 2*eye(3,3)];
                W = (1/2) * [...
                    -rotv(1) 2 -rotv(3) rotv(2); ...
                    -rotv(2) rotv(3) 2 -rotv(1); ...
                    -rotv(3) -rotv(2) rotv(1) 2];
            else
                a = cos(v/2)*v - 2*sin(v/2);
                G = ((sin(v/2)/(2*v)) .* [-rotv'; 2*eye(3,3)] )...
                    + ((a/(2*v^3)) .* [0 0 0; rotv*rotv']);
                W = (1/v) .* ...
                    [-rotv(1)*sin(v/2)  v*cos(v/2)       -rotv(3)*sin(v/2)   rotv(2)*sin(v/2); ...
                    -rotv(2)*sin(v/2)  rotv(3)*sin(v/2)  v*cos(v/2)        -rotv(1)*sin(v/2); ...
                    -rotv(3)*sin(v/2) -rotv(2)*sin(v/2)  rotv(1)*sin(v/2)   v*cos(v/2)];
            end
            V = W * G;
            if rcond(V) < 1e-3
                stopHere = 0;
            end
            rDot =  1/2 .* (V \ omega);
        end
        
    end
end