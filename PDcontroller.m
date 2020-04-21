function [tau, e] = PDcontroller(t,x,refFun)
    % x = [q1; q1dot; q2; q2dot];
    %     x(1) is position of 1st joint
    %     x(2) is velocity of 1st joint
    %     x(3) is position of 2nd joint
    %     x(4) is velocity of 2nd joint

    kp = 100;
    kd = 20;
    
    [q_ref,v_ref,a_ref] = refFun(t);

    e1 = x(1) - q_ref;
    e2 = x(3) - q_ref;
    
    e = [e1,e2];

    tau1 = -kp*e1 - kd*x(2);
    tau2 = -kp*e2 - kd*x(4);

    tau = [tau1, tau2];
    tau = max(min(tau,10),-10);

end