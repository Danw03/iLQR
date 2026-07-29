function R = Ry(theta)
% function to get Rotation Matrix for theta (Rotation about y-axis)
R = [ cos(theta)  0  sin(theta);
               0  1           0;
     -sin(theta)  0  cos(theta)];
end