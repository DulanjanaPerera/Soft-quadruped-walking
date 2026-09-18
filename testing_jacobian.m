J_Omega = zeros(3,3);
for i=1:6

    switch i
        case 1
            [~,R,~,R_dq] = Tx(1);
            J_Omega = R.' * R_dq;
        case 2
            [~,R,~,R_dq] = Ty(1);
            J_Omega = R.' * [J_Omega * R, R_dq];
        case 3
            [~,R,~,R_dq] = Tz(1);
            J_Omega = R.' * [J_Omega * R, R_dq];
        case 4
            [~,R,~,R_dq] = Rx(pi/3);
            J_Omega = R.' * [J_Omega * R, R_dq];
        case 5
            [~,R,~,R_dq] = Ry(pi/3);
            J_Omega = R.' * [J_Omega * R, R_dq];
        case 6
            [~,R,~,R_dq] = Rz(pi/3);
            J_Omega = R.' * [J_Omega * R, R_dq];
        otherwise
            
    end


end

function  res = blockMulti(A, B, len)

    [r, c] = size(A);
    blocks = floor(c/len);
    
    vec = 0;

    if isvector(B)
        res = zeros(blocks, r);
        vec = 1;
    else
        res = zeros(size(A));
    end
    
    for i=1:blocks
        if vec==1
            res(:,i) = A(:,3*i-2:3*i) * B;
        else
            res(:,3*i-2:3*i) = A(:,3*i-2:3*i) * B;
        end
        
    end

end