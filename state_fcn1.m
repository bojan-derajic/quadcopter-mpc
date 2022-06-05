function xd = state_fcn(x, u)
    cT = 3.13*1e-4;    
    cM = 7.5*1e-6;    
    d = 0.225;             
    m = 0.6;
    JR = 1e-5;
    Jxx = 0.0075;
    Jyy = 0.0075;
    Jzz = 0.0075;
    J = diag([Jxx, Jyy, Jzz]);

    S = [       cT              cT              cT              cT;
          d*cT*sqrt(2)/2  -d*cT*sqrt(2)/2  -d*cT*sqrt(2)/2  d*cT*sqrt(2)/2;
         -d*cT*sqrt(2)/2  -d*cT*sqrt(2)/2   d*cT*sqrt(2)/2  d*cT*sqrt(2)/2;
               -cM              cM             -cM              cM];

    g = [0; 0; 9.81];

    p = x(1:3);
    v = x(4:6);
    eta = x(7:9);
    omega = x(10:12);

    Ft = [0; 0; u(1)];
    Min = u(2:4);

    R = eul2rotm(eta(end:-1:1)', 'ZYX');

    T = [1  tan(eta(2))*sin(eta(1))   tan(eta(2))*cos(eta(1));
         0          cos(eta(1))                -sin(eta(1));
         0  sin(eta(1))/cos(eta(2))   cos(eta(1))/cos(eta(2))];

    OMEGA = sqrt(S^(-1)*u);

    Mgr = cross(omega, [0; 0; JR*(OMEGA(1)-OMEGA(2)+OMEGA(3)-OMEGA(4))]);

    xd = [v;
          -g + 1/m*R*Ft;
          T*omega;
          J^(-1)*(-cross(omega, J*omega) + Mgr + Min)];

    if p(3) <= 0 && xd(3) <= 0
        xd(3) = 0;
    end

end