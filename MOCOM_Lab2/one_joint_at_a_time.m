function [joint_vector_matrix] = one_joint_at_a_time(init_final_config, linknumber)

    joint_vector_matrix = zeros(2, linknumber, linknumber);
    
    qi = init_final_config(1, :);
    qf = qi;

for i = 1:linknumber
    for j = 1:i
        qf(1, j) =  init_final_config(2, j, 1);
    end
    joint_vector_matrix(1, :, i) = qi;
    joint_vector_matrix(2, :, i) = qf;

    qi = qf;
end
end