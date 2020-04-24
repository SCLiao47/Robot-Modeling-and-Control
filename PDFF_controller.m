function [tau, e] = PDFF_controller(t,x,refFun)
    % PD plus FeedForward controller    
    % x = [q1; q1dot; q2; q2dot];

    kp1 = 100; kd1 = 20;
    kp2 = 100; kd2 = 20;
    
    [q_ref,v_ref,a_ref] = refFun(t);

    % error
    e1 = x(1) - q_ref;
    e1dot = x(2) - v_ref;
    
    e2 = x(3) - q_ref;
    e2dot = x(4) - v_ref;
    
    e = [e1,e2];

    % controller
    tau1 = a_ref - kp1*e1 - kd1*e1dot;
    tau2 = a_ref - kp2*e2 - kd2*e2dot;
    tau = [tau1, tau2];
    
    % actuator satuation
    tau = max(min(tau,10),-10);
end