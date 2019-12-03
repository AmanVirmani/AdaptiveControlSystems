function M = IntegrateMatrix(T,MD,MD_prev)

[~,m] = size(MD);
for col = 1:m
    I(:,col) = trapz(T,[MD_prev(:,col)'; MD(:,col)'])';
end

M = I;

end