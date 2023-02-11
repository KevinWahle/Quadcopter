%% Integrando
s = 0:0.01:20;
f = @(t) sqrt(2*t)
plot(s, f(s))
I_SIMPSONS = simpsons(f, 0, 20, 5)

I_MIDPOINT =  midpoint_rule(f, 0, 20, 10)

REAL = integral(f, 0, 20)
%%

function integral = simpsons(f, a, b, n)
%SIMPSONS Simpson's rule for numerical integration.
%   INTEGRAL = SIMPSONS(F, A, B, N) returns the approximation of the definite
%   integral of function F over the interval [A, B], using Simpson's rule with
%   N subintervals.
%   The function F should be written as a MATLAB inline function or an anonymous
%   function, or be a string that can be evaluated using the eval function.

% Check input arguments
if nargin < 4
    error('Not enough input arguments.');
end
if ~isa(f, 'function_handle') && ~ischar(f)
    error('F must be a function handle or a string.');
end

% Evaluate the function at the sample points
h = (b - a) / (2 * n);
x = a:h:b;
y = zeros(size(x));
for i = 1:length(x)
    if isa(f, 'function_handle')
        y(i) = feval(f, x(i));
    else
        y(i) = eval(f);
    end
end

% Compute the approximation of the integral
integral = (b - a) / (6 * n) * (y(1) + 4 * sum(y(2:2:end-1)) + 2 * sum(y(3:2:end-2)) + y(end));
end


function result = midpoint_rule(f, a, b, n)
% MIDPOINT_RULE Approximates the definite integral of f over [a,b] using the midpoint rule
% with n subintervals.
%
%   result = MIDPOINT_RULE(f, a, b, n)
%
% Inputs:
%   f - function handle for the function to be integrated
%   a - lower limit of the integral
%   b - upper limit of the integral
%   n - number of subintervals
%
% Outputs:
%   result - approximation of the definite integral of f over [a,b]
%

% Define step size h
h = (b - a) / n;

% Define the points x_i
x = a + (0:n) * h + h/2;

% Evaluate the function at x_i
y = f(x);

% Approximate the integral using the midpoint rule
result = h * sum(y);
end


function result = euler_rule(f, a, b, n)
% MIDPOINT_RULE Approximates the definite integral of f over [a,b] using the midpoint rule
% with n subintervals.
%
%   result = MIDPOINT_RULE(f, a, b, n)
%
% Inputs:
%   f - function handle for the function to be integrated
%   a - lower limit of the integral
%   b - upper limit of the integral
%   n - number of subintervals
%
% Outputs:
%   result - approximation of the definite integral of f over [a,b]
%

% Define step size h
h = (b - a) / n;

% Define the points x_i
x = a + (0:n-1) * h + h/2;

% Evaluate the function at x_i
y = f(x);

% Approximate the integral using the midpoint rule
result = h * sum(y);
end
