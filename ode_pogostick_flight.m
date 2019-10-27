function dxdt = ode_pogostick_flight(t,q,robot)
    t
    g = [0
        -9.8
         0];
    dxdt = zeros(6,1);

    dxdt(1:3) = q(4:6);
    dxdt(4:6) = [g(1:2)
                 0];
end

