function history = genHistory(history, step, sim_steps, Xseries, R, U_full_current, Xref, current_contact)
    history.X(:, (step-1)*sim_steps + 1 : step*sim_steps) = Xseries; 
    history.R(:, step) = R(1:12, 1);
    history.F(:, step) = U_full_current;
    history.Xref(:, step) = Xref;
    history.F_total(:, step) = sum(U_full_current);
    history.contact(:, step) = sum(current_contact);
end