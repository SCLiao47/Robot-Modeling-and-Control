function [M]=Lagrange(Lag,V)
% Lag = Lagrange of the system (symbolic).
% V   = System Variables (symbolic) [q1 dq1 ddq1 q2 dq2 ddq2... qn dqn
% ddqn].
% Equations   = [1 X DOF] (Degrees of freedom of the system).

% http://youngmok.com/lagrange-equation-by-matlab-with-examples/

    syms t;
    Var=length(V)/3;
    Vt=V;

%     syms dpLpqd_dt [1 Var]
    dpLpqd_dt = sym(zeros(2,1));
    for i = 1:1:Var
        pLpqd = diff(Lag, V((i-1)*3+2));

        % by chain rule 
        for j = 1:1:Var
            q = V((j-1)*3+1);
            qd = V((j-1)*3+2);
            qdd = V((j-1)*3+3);

            dpLpqd_dt(i) = dpLpqd_dt(i) + diff(pLpqd, q)*qd + diff(pLpqd, qd)*qdd;
        end
    end

    syms pLpq [Var 1]
    for i = 1:1:Var
        q = V((i-1)*3+1);
        pLpq(i) = diff(Lag, q);
    end

    M = dpLpqd_dt - pLpq;

end