function EXP = TwistExp(xi, theta)
    v = xi(1:3);
    w = xi(4:6);
    
    if(norm(w) == 0)
        EXP = [eye(3) v*theta;
               0 0 0 1];
           
    else
        exp_w = expm(W2Skew(w)*theta);
        
        EXP = [exp_w (eye(3)-exp_w)*cross(w,v)+w*w'*v*theta;
               0 0 0 1];
    end
    
    EXP = rewrite(EXP, 'sincos');
end