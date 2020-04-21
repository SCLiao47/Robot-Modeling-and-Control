function Ad = Adjoint(gst)
    R = gst(1:3,1:3);
    p = gst(1:3,4);
    
    Ad = [R, W2Skew(p)*R;
          zeros(3), R];
end