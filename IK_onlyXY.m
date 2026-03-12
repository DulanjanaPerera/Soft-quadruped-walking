function [l, z] = IK_onlyXY(u, r, L)
X = u(1);
Y = u(2);
% Z = u(3);
% if (-4e-3<Y)>8e-3 || (-4e-3>X)>8e-3
if abs(Y)<8e-4
    Y = 0;
end
    
if abs(X)<8e-4
    X = 0;
end
% if abs(Y)>1e-3 && abs(X)>1e-3
    
    theta=atan2(Y,X);


%     f1 = L*sin(theta);
%     f2 = (Z/L)^2 + (Y/f1)^2;
%     phi = (2*Y)/(f1*f2); 
%     F = @(x) cos(x) + ((Y*x)/f1) - 1;
    F = @(x) sqrt(X.^2+Y.^2)-sqrt(L.^2*(1-cos(x)).^2./(x.^2));
    x=fsolve(F,[0.000001;pi]);
    phi=x(1);
    out = [theta, phi];

% Compute actuator lengths
    l = zeros(3, 1);
    l(1) = -r * cos(theta) * phi;
    l(2) = (0.5 * r * cos(theta) - sqrt(3) * r * sin(theta) / 2) * phi;
    l(3) = (0.5 * r * cos(theta) + sqrt(3) * r * sin(theta) / 2) * phi;
    
    % Compute z coordinate (handle small phi)
    if phi < 1e-6
        z = L; % straight configuration
    else
        z = (L/phi) * sin(phi);
    end

end
