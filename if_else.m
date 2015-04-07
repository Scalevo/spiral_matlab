% Initialize parameters out of input vector
        v = [.17,.28,0,-.2,-.6];
        h     = v(1);
        t     = v(2);
        theta = v(3);
        dx    = v(4);
        dz    = v(5);
        
        % Initialize helping parameters out of input paramters
        %xi = xi + dx;
        eta = atan2(t,h);
        a = h*cos(eta);
        b = t*sin(eta);
        L = 2*(a + b);
        x1 = a;
        x2 = a + b;
        
        xi = linspace(0,2,10000);
        %
        z = zeros(1,length(xi));
        for it = 1:length(xi)
            n = floor(xi(it)/x2);
            if mod(xi(it),x2) < x1
                z(it) = xi(it)*tan(eta) - (n)*t/(cos(eta)) - h*sin(eta);
            else
            z(it) = -xi(it)/(tan(eta)) + (n+1)*h/(sin(eta)) - h*sin(eta);
            end
        end
        
        plot(xi,z,'x');
        axis equal;