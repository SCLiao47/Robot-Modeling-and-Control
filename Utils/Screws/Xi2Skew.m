function XI_h = Xi2Skew(xi)
    v = xi(1:3);
    w = xi(4:6);

    XI_h = zeros(4);
    
    if(norm(w) == 0)
        XI_h(1:3,3) = v;
    else
        XI_h = [W2Skew(w) v;
                0 0 0 0];
    end
end