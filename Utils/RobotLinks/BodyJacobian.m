function Jb = BodyJacobian(XiTh, g_st0)
% Xi_th is a nx2 cell array
%       1st column is Xi
%       2nd column is th
% can be constructed by {Xi1, th1; Xi2 th2; ...}

    num = size(XiTh,1);
    
    Jb = sym(zeros(6, num));
    
    for j = 1:num
        g_ji = g_st0;
        for k = num:-1:j
            g_ji = TwistExp(XiTh{k,1}, XiTh{k,2})*g_ji;
            g_ji = simplify(g_ji);
        end
        
        Jb(:,j) = simplify(Adjoint(g_ji)^-1*XiTh{j,1});   
    end
end