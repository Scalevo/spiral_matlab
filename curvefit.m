    function [ v_r,se_r,z_r,x0,zi ] = curvefit(x0,z0,v0_)
% Returns the result of the fminserach using start values as well as the
% pointcloud vectors.
%     
se_r = 0;
xi = 0;

%% Minimize delta for v

handle = @delta;
% options = struct('MaxFunEvals',1000,'MaxIter',1000,'TolX',1000,'TolFun',0.00001,); % 'OutputFcn', @outfun,'PlotFcns',@optimplotfval
options = struct('PlotFcns',@optimplotfval,'TolX',1000,'TolFun',0.001);
[v_r,z_r,residual,exitflag] = lsqcurvefit(handle,v0_,x0,z0,[],[],options);
%disp(v_r);
disp(exitflag);
disp(length(x0));
disp(length(z_r));


%% Calculate z_r for a given x0
    function [z_r] = delta(v,x0)
        
    %  Initialize parameters out of input vector

        h = sqrt(v(1)^2);
        t = sqrt(v(2)^2);
        dx    = sqrt(v(3)^2);
        dz    = v(4);
        theta = v(5);

        eta = atan2(t,h);

        a = h*cos(eta);           % Parameters used for if_else
        b = t*sin(eta);
        x1 = a;
        x2 = a + b;      
        
%%       If_else parametrisation
       
        zi = zeros(1,length(x0));        
        for it = 1:length(x0)
            n = floor(x0(it)/x2);
            if mod(x0(it),x2) < x1
                zi(it) = x0(it)*tan(eta) - n*t/(cos(eta)) - h*sin(eta);
            else
            zi(it) = -x0(it)/(tan(eta)) + (n+1)*h/(sin(eta)) - h*sin(eta);
            end
        end
        
       %% 
        zi = zi + dz;
        x0 = x0 + dx;
        
        x0 = cos(theta)*x0 - sin(theta)*zi;
        z_r = cos(theta)*zi + sin(theta)*x0;
        
%     plot(xi, zi, 'x');
%     axis equal tight
%     hold on;
%     plot(xi, z_r, 'o')
%     axis equal tight
%     hold off;
%     ylabel('z [m]','FontSize',20);
%     xlabel('x [m]','FontSize',20);
%     h_legend = legend('Transformed PointCloud', 'Matched Template');
%     set(h_legend,'FontSize',15);
        
    end
end

