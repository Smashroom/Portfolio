function c = insert(c,ins,idx)
    c = [c(1:idx-1) {ins} c(idx:end)];
end