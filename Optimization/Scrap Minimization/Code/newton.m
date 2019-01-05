function [x_val,f_val,iter_n] = newton(func,grad,n,tol,x0,maxiter,lambda)
%Calculate gradient and initialize some values (e.g. iteration number,gradient vector)
iter_n = 0; grad_norm = inf;

xstart = -1;
xend = 1;
if size(x0,1) == 1
x0 = x0';
end
dir = inv(hess_f(x0,lambda))*grad(x0,lambda)';
if isinf(inv(hess_f(x0,lambda))) 
dir = eye(size(dir,1))*grad(x0,lambda)';
end
alpha_func = @(alpha)func((x0+alpha*dir),lambda);
[~,alpha] = golden_section(alpha_func,xstart,xend,tol);
x_old = x0;

while (iter_n<maxiter) && norm(alpha*dir) > tol
    
    x_new = x_old + alpha*dir;
    x_old = x_new;
    iter_n = iter_n + 1;
    x0 = x_new;
    
    dir = inv(hess_f(x_old,lambda))*grad(x0,lambda)';
    alpha_func = @(alpha)func((x_old+alpha*dir),lambda);
    [~,alpha] = golden_section(alpha_func,xstart,xend,tol);

    grad_norm = norm(inv(hess_f(x0,lambda))*grad(x0,lambda)');
end
f_val = func(x_new,lambda);
x_val = x_new;
if isinf(x_new) | isnan(x_new)
x_val = x0;
end

end