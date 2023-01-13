% computes the adjoint matrix transferring elements from the lie algebra to
% the local tangent space of M in SE(3)
% for derivative of the formula applied see Example 6, p.7 in
% https://arxiv.org/pdf/1812.01537.pdf

% NOTE: use the Adjoint Matrix at M to transfer tangent vectors such as
% twists from the identity to the group element M
% let vt be a 6x1 vector at se(3), then AdM * vt is in Tau_M_SE(3)

function AdM = adjointSE3(M)
    R = M(1:3, 1:3); % the rotation matrix of the transform
    t = M(1:3, 4); % translation vector of the transform
    % skew-symmetric form of the translation vector
    t_x = [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0];
    % construct the 6x6 adjoint matrix
    AdM = [ R           t_x * R; 
            0 * eye(3)  R        ];
end