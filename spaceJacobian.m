function J = spaceJacobian(rigidBodyTree, currArticulation, tcpName)

    % set articulation input vector format to column
    rigidBodyTree.DataFormat = "column";
    thetaZero = homeConfiguration(rigidBodyTree);
    [numJoints, ~] = size(thetaZero);
    Sxs = zeros([4 4 numJoints]); % screw matrix [se(3)] storage
    Svecs = zeros([6 numJoints]); % screw vector storage
    % home pose of the endeffector body "n"
    hTn = getTransform(rigidBodyTree, thetaZero, tcpName);
    % the output Jacobian matrix
    J = zeros([6 numJoints]);

    % compute space jacobian
    for i = 1:numJoints
        delta = zeros(numJoints,1);
        delta(i) = 1; % NOTE: the value of the delta angle does not matter
        dTn = getTransform(rigidBodyTree, thetaZero + delta, tcpName);
        % compute screw matrix
        SxQ = logm(dTn / hTn);
        % obtain screw vector
        wxQ = [SxQ(3,2); SxQ(1,3); SxQ(2,1)];
        vxQ = SxQ(1:3, 4);
        % normalize screw vector and screw matrix
        if norm(wxQ) ~= 0
            Svec = [vxQ; wxQ] / norm(wxQ);
            Sx = SxQ / norm(wxQ);
        elseif norm(vxQ) ~= 0
            Svec = [vxQ; wxQ] / norm(vxQ);
            Sx = SxQ / norm(vxQ);
        else
            Svec = zeros(6,1); % edge case (e.g. incorrect tcpName)
            Sx = zeros(4);
        end

        Sxs(:,:, i) = Sx; % store for computation of adjoint matrices
        Svecs(:, i) = Svec; % store to generate global screw

        if i == 1
            J1 = Svecs(:,i);
            J(:, 1) = J1;
        elseif i == 2
            Ad1 = adjointSE3(expm(Sxs(:,:,1) * currArticulation(1)));
            J2 = Ad1 * Svecs(:,2);
            J(:, 2) = J2;
        else
            % compute first exponential independently
            PoE = expm(Sxs(:,:,1) * currArticulation(1));
            for j = 2:i-1 % then compute product of exponentials
                PoE = PoE * expm(Sxs(:,:,j) * currArticulation(j));
            end
            AdMn = adjointSE3(PoE);
            Jn = AdMn * Svecs(:,j);
            J(:, i) = Jn;
        end
    end
end

