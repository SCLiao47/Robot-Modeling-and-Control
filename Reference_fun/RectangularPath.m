function [q_ref,v_ref,a_ref] = RectangularPath(t)
    % Rectangular trajectory
    %   q_ref = q_desire        for 0 < t < tm 
    %         = 0               for else

    t0 = 0;     q0 = 0;     v0 = 0;
    tm = 1;     qm = pi/2;  vm = 0;
    tf = 2;     qf = 0;     vf = 0;

    q_ref = (t<tm)*qm;
    v_ref = 0;
    a_ref = 0;
end