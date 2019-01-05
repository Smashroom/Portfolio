%JeanPiere Demir
%1936632

function [x_val,f_val,iter_n] = conjugate_gradient(func,grad,n,tol,x0,maxiter,lambda)
%Calculate gradient and initialize some values (e.g. iteration number,gradient vector)
iter_n = 0; grad_norm = inf;

xstart = -2 ;
xend = 2;
if size(x0,1) == 1
    x0 = x0';
end
xold = x0;

grad_norm = -grad(x0,lambda)';
alpha_func = @(alpha)func((x0 +grad_norm*alpha),lambda);
[~,alpha] = golden_section(alpha_func,xstart,xend,tol);

p_old = grad_norm;
grad_old = grad_norm;
x_old = x0;
xnew = x0;
while norm(grad_norm)>tol && (iter_n<maxiter)

    xnew = x_old +alpha*p_old;
    grad_new = -grad(xnew,lambda)';
	beta = (norm(grad_new)^2)/(norm(grad_old)^2);
	p = grad_new + beta*p_old;

	grad_old = grad_new;
    p_old = p;
	x_old = xnew;
    alpha_func = @(alpha)func((x_old+alpha*p_old),lambda);
    [~,alpha] = golden_section(alpha_func,xstart,xend,tol);

    grad_norm = norm(grad(xnew,lambda));
	iter_n = iter_n + 1;
end
f_val = func(xnew,lambda);
x_val = xnew;
if isinf(x_new) | isnan(x_new)
    x_val = x0;
end
end
