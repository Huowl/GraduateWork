if actuatorType == 1
    k_p_fem =  185.2563;
    k_d_fem =  0.0977;
    k_i_fem = 0;
    k_p_tib =  185.2563;
    k_d_tib =  0.0977;
    k_i_tib = 0;
    ctrl_name = "Motor";
elseif actuatorType == 2
    k_p_fem = 80;
    k_d_fem = 1;
    k_i_fem = 0;
    k_p_tib = 80;
    k_d_tib = 1;
    k_i_tib = 0;
    ctrl_name = "PD_ctrl";
elseif actuatorType == 4
    k_p_fem = 80;
    k_d_fem = 1;
    k_i_fem = 0;
    k_p_tib = 80;
    k_d_tib = 1;
    k_i_tib = 0;
    ctrl_name = "Motion";
end