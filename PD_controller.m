function [tau, e] = PD_controller(t,x,refFun)
    % PD controller
    % x = [q1; q1dot; q2; q2dot];

    kp1 = 100; kd1 = 20;
    kp2 = 100; kd2 = 20;
    
    [q_ref,~,~] = refFun(t);
    
    % error
    e1 = x(1) - q_ref;
    e2 = x(3) - q_ref; 
    e = [e1,e2];

    % controller
    tau1 = -kp1*e1 - kd1*x(2);
    tau2 = -kp2*e2 - kd2*x(4);
    tau = [tau1, tau2];
    
    % actuator satuation    
    tau = max(min(tau,10),-10);
end