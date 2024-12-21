function [Jacobian, biTei, bTi] = compute_Jacobian(geom_model, q, linkType, numberOfLinks)
    % Compute direct geometry

    biTei = GetDirectGeometry(q, geom_model, linkType, numberOfLinks);
    
    % initialization of the bTi matrices vector
    bTi = zeros(4, 4, numberOfLinks);
    % Compute the transformation w.r.t. the base
    for i = 1:numberOfLinks
        bTi(:, :, i) = GetTransformationWrtBase(biTei, i);
    end

    % Computing the end effector jacobian 
    Jacobian = GetJacobian(bTi, linkType); 
end