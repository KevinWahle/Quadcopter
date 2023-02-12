%% Integrando
s = 0:0.01:20;
f = @(t) (sin(t/2)).^2
plot(s, f(s))
I_SIMPSONS = simpsons(f, 0, 5) + simpsons(f, 5, 10) + simpsons(f, 10, 15) + simpsons(f, 15, 20)
I_EULER = eulerChad(f, 0, 20, 8)

REAL = integral(f, 0, 10)
%%
function integral = eulerChad(f, a, b, n)
h = (b - a) / n;
t = a:h:b;
integral = h*sum(f(t(2:end)));
end
function integral = simpsons(f, a, b)
% Evaluate the function at the sample points
h = (b - a) / 2;
t = a:h:b;

% Compute the approximation of the integral
integral = h * (f(t(1)) + 4 * sum(f(t(2))) + f(t(3)))/3;
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


function result = euler_rule(f, a, b)
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
