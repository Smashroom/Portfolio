% JeanPiere Demir
% 1936632

function [fval,x,cntr] = golden_section(func,x0,xend,tol);

error = tol;                    % uncertainty value 

iter= 100;                     % maximum number of iterations

tau=double((sqrt(5)-1)/2);      % golden proportion coefficient, around 0.618

cntr = 0;                            % Initialization


x_1=x0+(1-tau)*(xend-x0);             % computing initial x values
x_2=x0+tau*(xend-x0);

f_x1=func(x_1);                     % computing results for initial x values
f_x2=func(x_2);

while ((xend-x0)>error) && ( cntr <iter)
    cntr=cntr+1;
    if(f_x1<f_x2)
        xend=x_2;
        x_2=x_1;
        x_1=x0+(1-tau)*(xend-x0);
        
        f_x1=func(x_1);
        f_x2=func(x_2);
        
    else
        x0=x_1;
        x_1=x_2;
        x_2=x0+tau*(xend-x0);
        
        f_x1=func(x_1);
        f_x2=func(x_2);
        
    end
    
    cntr=cntr+1;
end

if f_x1 >= f_x2
    fval = f_x2;
    x = x_2;
else
    fval = f_x1;
    x = x_1;
end

