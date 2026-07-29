function R = Rz(psi)
% fuction to get Rotation Matrix for psi
R = [cos(psi) -sin(psi) 0;
     sin(psi)  cos(psi) 0;
            0         0 1];
end