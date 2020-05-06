function [q_ref,v_ref,a_ref] = CubicPath(t)
    % Cubic trajectory reference 
    %   start at        (t0,q0,v0)
    %   pass through    (tm,qm,vm)
    %   end at          (tf,qf,vf)


    t0 = 0;     q0 = 0;     v0 = 0;
    tm = 1;     qm = pi/2;  vm = 0;
    tf = 2;     qf = 0;     vf = 0;
    
    alpha_0m = CubicCoefficient(t0,tm,q0,v0,qm,vm);
    alpha_mf = CubicCoefficient(tm,tf,qm,vm,qf,vf);
    
    num = length(t);
    q_ref = zeros(num,1);
    v_ref = zeros(num,1);
    a_ref = zeros(num,1);
    
    
    for i = 1:num
        ti = t(i);
    
        if ti>=t0 && ti<=tm
            alpha = alpha_0m;
        elseif ti>tm && ti<=tf
            alpha = alpha_mf;
        else
            alpha = zeros(4,1);
        end

        q_ref(i) = alpha'*[1, ti, ti^2, ti^3]';
        v_ref(i) = alpha'*[0, 1, 2*ti, 3*ti^2]';
        a_ref(i) = alpha'*[0, 0, 2, 6*ti]';
    end
end

function alpha = CubicCoefficient(t0,tf,q0,v0,qf,vf)
    M = [1 t0 t0^2 t0^3;
         0 1 2*t0 3*t0^2;
         1 tf tf^2 tf^3;
         0 1 2*tf 3*tf^2];
     
    alpha = M^-1*[q0 v0 qf vf]';
end