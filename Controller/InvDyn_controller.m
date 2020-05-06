function [tau, e] = InvDyn_controller(t,x,refFun)
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
    
    aq = zeros(2,1);
    aq(1) = a_ref - kp1*e1 - kd1*e1dot;
    aq(2) = a_ref - kp2*e2 - kd2*e2dot;

    % parameter matrix of robot
    global g m1 l1 lc1 I1 m2 lc2 I2
    [D,C,P] = RR_ManEqnCoeff(I1,I2,g,l1,lc1,lc2,m1,m2,x(1),x(3),x(2),x(4));
    
    % controller
    tau = D*aq + C*[x(2);x(4)] + P;
    
    % actuator satuation
    tau = max(min(tau,10),-10);
end