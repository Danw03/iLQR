function R = get_R(Xc, Xref, contact, p, Rc)
%% function to get foot placement
% Rc: current R

pos_c = Xc(4:6, 1);
hip_offset = [ 0.365;  0.12; 0;
               0.365; -0.12; 0;
              -0.365;  0.12; 0;
              -0.365; -0.12; 0];
R = zeros(12*p.k, 1);
t_stance = p.t_stance;

foot_pos_world = zeros(12, 1);
for i = 1:4
    index = 3*(i-1) + 1 : 3*i;
    foot_pos_world(index, 1) = pos_c + Rc(index, 1);
end

for j = 1:p.k
    ref_index = 13*(j-1) + 1 : 13*j;
    Xref_i = Xref(ref_index);
    yaw_ref = Xref_i(3);
    pos_ref = Xref_i(4:6);
    w_ref = Xref_i(9);
    v_ref = Xref_i(10:12);
    v_ref(3) = 0;

    for k = 1:4
        r_index = 3*(k-1) + 1 : 3*k;
        current_contact = contact(k, j);
        hip_body = hip_offset(r_index);

        hip_world = pos_ref + Rz(yaw_ref)*hip_body;
        v_hip = v_ref + cross([0; 0; w_ref], Rz(yaw_ref)*hip_body);

        if current_contact == 0
            foot_pos_world(r_index, 1) = hip_world + 0.5 * v_hip * t_stance;
        end
        
        foot_pos_world(r_index(3), 1) = 0; 
        r = foot_pos_world(r_index, 1);
        R(12*(j-1) + r_index) = r - pos_ref; 
    end
end
end