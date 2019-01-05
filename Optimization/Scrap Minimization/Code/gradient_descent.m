function [x_val,f_val, iter_n] = gradient_descent(func,grad,n,tol,x0,maxiter,lambda)


%Calculate gradient and initialize some values (e.g. iteration number,gradient vector)
iter_n = 0; grad_norm = inf;
if size(x0,1) == 1
    x0 = x0';
end
xstart = -2 ;
xend = 2;
x_old = x0;

grad_norm = grad(x0,lambda);
alpha_func = @(alpha)func((x0 -grad_norm*alpha)',lambda);
[~,alpha] = golden_section(alpha_func,xstart,xend,tol);

while norm(grad_norm)>tol && (iter_n<maxiter)
	xnew = x_old -alpha*grad(x_old,lambda)';
	
	x_old = xnew; 
    grad_norm = grad(x_old,lambda)';
    
    alpha_func = @(alpha)func((x_old -grad_norm*alpha)',lambda);
	[~,alpha] = golden_section(alpha_func,xstart,xend,tol);
	
	iter_n = iter_n +1;	
end
f_val = func(xnew,lambda);
x_val = xnew;
if isinf(xnew) | isnan(xnew)
    x_val = x0;
end
end
