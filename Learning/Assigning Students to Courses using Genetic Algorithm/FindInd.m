function [ idx ] = FindInd( ind,Population )
for idx =  1: length(Population)
    if(ind == Population{idx})
        return;
    end
end



end

