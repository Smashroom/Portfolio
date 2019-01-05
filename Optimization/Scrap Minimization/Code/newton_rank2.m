function [x_val,f_val,iter_n] = newton_rank2(func,grad,n,tol,x0,maxiter,lambda)
%Calculate gradient and initialize some values (e.g. iteration number,gradient vector)
iter_n = 0; grad_norm = inf;

xstart = -1;
xend = 1;

grad_old = grad(x0,lambda)';
if size(x0,1) == 1
    x0 = x0';
end
x_old = x0;
p = x0;
q = grad_old;
inv_hess = eye(n);

while (iter_n<maxiter) && (norm(p) >= sqrt(tol))
    dir = inv_hess*grad_old;
    alpha_func = @(alpha)func((x_old+alpha*dir),lambda);
    [~,alpha] = golden_section(alpha_func,xstart,xend,tol);

    x_new = x_old + alpha*dir;
    
    inv_hess = inv_hess + (p*p')/(p'*p)+-(inv_hess*(q*q')*inv_hess)/(q'*inv_hess*q);
    
    grad_new = grad(x_new,lambda)';
    q = grad_new - grad_old;
    p = x_new - x_old;
    x_old = x_new;
    
    grad_old = grad_new;
    iter_n = iter_n + 1;
    grad_norm = norm(grad_old);
end
f_val = func(x_new,lambda);
x_val = x_new;
if isinf(x_new) | isnan(x_new)
    x_val = x0;
end
end