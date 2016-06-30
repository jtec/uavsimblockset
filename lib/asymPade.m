% Computes a Padé approximation of the given time delay T. The numerator will
% be of order N-1, the denominator of order N, as proposed in "SOME REMARKS
% ON PADÉ-APPROXIMATIONS", M.Vajita.
function [ num, den ] = asymPade( T, N )
num_n = N-1; 
den_n = N;

if nargin==1
    N = 1;
elseif N<0 || T<0
    disp([mfilename '>> Nochmal.'])
end
N = round(N);

num = fliplr(doit(num_n, den_n, -1));
den = fliplr(doit(den_n, num_n, 1));

if nargout == 0
   step(tf(num, den)); 
end
    function poly = doit(m, n, sign)
        poly = [];
       for k=0:m 
           poly(end+1) = (sign * T)^k * (factorial(m+n-k) * factorial(m)) / ...
                         (factorial(m+n) * factorial(k) * factorial(m-k));
      end
    end

end