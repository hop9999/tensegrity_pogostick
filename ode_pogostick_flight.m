function dxdt = ode_pogostick_flight(t,q,robot)
    t
    g = [0
        -9.8
         0];
    dxdt = zeros(7,1);
    
    dxdt(1:3) = q(5:7);
    dxdt(5:7) = [g(1:2)
                 0];
end

