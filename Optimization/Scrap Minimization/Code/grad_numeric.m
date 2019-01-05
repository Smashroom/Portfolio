function grad = grad_numeric( x ,lambda)

    i = 1:10;
    for i = 1:10
        cost_1 = cost_f(x,lambda);
        x(i) = x(i) - 1e-10;
        cost_2 = cost_f(x,lambda);
        grad(i) = (cost_2-cost_1)/(1e-10); 
    end
end

