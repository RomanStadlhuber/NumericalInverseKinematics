% function that obtains an initial guess for inverse kinematics using
% monte-carlo sampling

function bestGuess = monteCarloInitialGuess(rbTree, tcpName, targetPoseTransform)

rbTree.DataFormat = 'column';
numJoints = rbTree.NumBodies;

% how many random samples to create in total
initialRandomSamleCount = numJoints * 2000;
% the threshold at which the linearized pose-delta norm needs to lie in
% order to be accepted as a guess
acceptanceThreshold = 0.5;
% convert the target transform matrix to an element in SE(3)
goalPose = se3(targetPoseTransform);
% initialize a list to store the random guesses
guesses = zeros([6 initialRandomSamleCount], "double");
distances = zeros(initialRandomSamleCount, "double");
% a counter used to shrink the pre-allocated array after sampling
acceptanceCount = 0;
% generate a heap of random samples and select those below the threshold
for k=1:initialRandomSamleCount
    guess = randomConfiguration(rbTree);
    tcpPose = getTransform(rbTree, guess, tcpName);
    distance = dist(se3(tcpPose), goalPose);
    % add guess if the error is below
    if distance <= acceptanceThreshold
        acceptanceCount = acceptanceCount + 1;
        guesses(:, acceptanceCount) = guess;
        distances(acceptanceCount) = distance;
    end
end
% shrink to size of actually stored elements
guesses = guesses(:, 1:acceptanceCount);
distances = distances(1:acceptanceCount);
% obtain the index of the guess with the lowest distance
[~, idxBestGuess] = min(distances);
% read the best guess configuration
bestGuess = guesses(:, idxBestGuess);
end