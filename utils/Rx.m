function R = Rx(phi)
% function to get Rotation Matrix for phi (Rotation about x-axis)
R = [1          0           0;
     0    cos(phi)  -sin(phi);
     0    sin(phi)   cos(phi)];
end