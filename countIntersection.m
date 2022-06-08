function count = countIntersection(S)
count = 0;
for idx1 = 1:size(S,1)-1
    for idx2 = (idx1+2):size(S,1)-1
        denominator = (S(idx2+1,2)-S(idx2,2))*(S(idx1,1)-S(idx1+1,1)) - (S(idx2+1,1)-S(idx2,1))*(S(idx1,2)-S(idx1+1,2));
        num_a = (S(idx2+1,1)-S(idx1+1,1))*(S(idx2+1,2)-S(idx2,2)) - (S(idx2+1,1)-S(idx2,1))*(S(idx2+1,2)-S(idx1+1,2));
        num_b = (S(idx1,1)-S(idx1+1,1))*(S(idx2+1,2)-S(idx1+1,2)) - (S(idx2+1,1)-S(idx1+1,1))*(S(idx1,2)-S(idx1+1,2));
        Ua = num_a / denominator;
        Ub = num_b / denominator;
        if (Ua >= 0 && Ua <= 1 && Ub >= 0 && Ub <=1)
            count = count + 1;
        end
    end
end
end