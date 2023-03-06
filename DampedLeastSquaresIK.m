function DampedLeastSquaresIK(block)
%
% The setup method is used to setup the basic attributes of the
% S-function such as ports, parameters, etc. Do not add any other
% calls to the main body of the function.  
%   
setup(block);
  
%endfunction

% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
% 
%   Required         : Yes
%   C MEX counterpart: mdlInitializeSizes
%
function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 3; % Pose, Weights, InitialGuess
  block.NumOutputPorts = 1; % Config
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;


  % NOTE: see this page for documentation on block-data settings:
  % https://de.mathworks.com/help/simulink/slref/simulink.blockdata.html

  % Override the input port properties.

  % ---- Input "Pose": the current pose of the manipulator 
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(1).Dimensions = [4,4];
  % ---- Input "Weights": coefficients for weighting the individual
  % components of the error vector during optimization
  block.InputPort(2).DataTypeID = 0; % double
  block.InputPort(2).Complexity = 'Real';
  block.InputPort(2).Dimensions = [6,1];
  % ---- Input "InitialGuess": a self-fed-back joint-configuration vector
  % of the starting point for the optimization routine
  block.InputPort(3).DataTypeID = 0; % double
  block.InputPort(3).Complexity = 'Real';
  block.InputPort(3).Dimensions = [6,1];

  % Register the parameters.
  block.NumDialogPrms     = 6;
  block.DialogPrmsTunable = {'Nontunable', 'Nontunable', 'Tunable', 'Tunable', 'Tunable', 'Nontunable'}; 
  % Set up the continuous states.
  block.NumContStates = 0;
  block.SampleTimes = [0.1 0]; % first value is block sample time in seconds
  
  % 
  % CheckParameters:
  %   Functionality    : Called in order to allow validation of the
  %                      block dialog parameters. You are 
  %                      responsible for calling this method
  %                      explicitly at the start of the setup method.
  %   C MEX counterpart: mdlCheckParameters
  %
  block.RegBlockMethod('CheckParameters', @CheckPrms);
  
  %
  % SetInputPortDimensions:
  %   Functionality    : Check and set the input and optionally the output
  %                      port dimensions.
  %   C MEX counterpart: mdlSetInputPortDimensionInfo
  %
  block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);

  %
  % SetOutputPortDimensions:
  %   Functionality    : Check and set the output and optionally the input
  %                      port dimensions.
  %   C MEX counterpart: mdlSetOutputPortDimensionInfo
  %
  block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  
  %
  % SetInputPortDatatype:
  %   Functionality    : Check and set the input and optionally the output
  %                      port datatypes.
  %   C MEX counterpart: mdlSetInputPortDataType
  %
  block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  
  %
  % SetOutputPortDatatype:
  %   Functionality    : Check and set the output and optionally the input
  %                      port datatypes.
  %   C MEX counterpart: mdlSetOutputPortDataType
  %
  block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  

  % 
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  %   C MEX counterpart: mdlOutputs
  %
  block.RegBlockMethod('Outputs', @Outputs);
  
  % 
  % SimStatusChange:
  %   Functionality    : Call when simulation enters pause mode
  %                      or leaves pause mode.
  %   C MEX counterpart: mdlSimStatusChange
  %
  block.RegBlockMethod('SimStatusChange', @SimStatusChange);
  
  % 
  % Terminate:
  %   Functionality    : Call at the end of a simulation for cleanup.
  %   C MEX counterpart: mdlTerminate
  %
  block.RegBlockMethod('Terminate', @Terminate);

%endfunction

% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

function CheckPrms(block)
% load rigidbody tree
configuration = block.DialogPrm(1).Data;
tcpName = block.DialogPrm(2).Data;
dampingfactor = block.DialogPrm(3).Data;
maxIterations = block.DialogPrm(4).Data;
acceptanceThreshold = block.DialogPrm(5).Data;
computeInitialGuess = block.DialogPrm(6).Data;
% throw exception if the provided parameter is no rigidbody tree
if ~isa(configuration, 'rigidBodyTree')
    me = MSLException(block.BlockHandle, message('Requires a RigidBodyTree object'));
    throw(me)
end
% throw if tcp frame name is not a string
if ~isa(tcpName, "string")
    me = MSLException(block.BlockHandle, message('Requires a string of the TCP frames name.'));
    trhow(me)
end
% throw if damping coefficient is not a real double scalar
if ~isa(dampingfactor, "double")
    me = MSLException(block.BlockHandle, message('DampingFactor needs to be a positive real scalar.'));
    trhow(me)
end
% throw if max number of iterations is not an integer value or equal to or
% less than zero
if ~isa(maxIterations, "int32") || int32(maxIterations) <= 0
    me = MLSException(block.BlockHandle, message("Maximum number of iterations needs to be a positive integer."));
    throw(me)
end
if ~isa(acceptanceThreshold, "double") || double(acceptanceThreshold) <= 0.0
    me = MSLException(block.BlockHandle, message('Threshold needs to be a positive real scalar.'));
    trhow(me)
end
if size(logical(computeInitialGuess), 2) > 1
    me = MSLException(block.BlockHandle, message('Initial guess indicator needs to be a logical value.'));
    trhow(me)
end

function SetInpPortDims(block, idx, di)
  
  block.InputPort(idx).Dimensions = di;
  %  block.OutputPort(portid).Dimensions=dim, where dim=[m,n]
  block.OutputPort(1).Dimensions  = di;

%endfunction

function SetOutPortDims(block, idx, di)
  
  % NOTE: since the dimensions of the first input and output are not the
  % same, special logic is required to set the correct dimensions
  % this is required to compile the block
  block.OutputPort(idx).Dimensions = di;
  if idx == 1
      block.OutputPort(idx).Dimensions = [6,1];
  else
    block.InputPort(1).Dimensions    = di;
  end
%endfunction

function SetInpPortDataType(block, idx, dt)
  
  block.InputPort(idx).DataTypeID = dt;
  block.OutputPort(1).DataTypeID  = dt;

%endfunction
  
function SetOutPortDataType(block, idx, dt)

  block.OutputPort(idx).DataTypeID  = dt;
  block.InputPort(1).DataTypeID     = dt;


function Outputs(block)
  
  % obtain Pose (4x4), Weights (6x1) and InitialGuess (6x1)
  targetPose = block.InputPort(1).Data;
  weights = block.InputPort(2).Data;
  % switch angular and linear compoments of the weights
  % since they, similar to the geometric jacobian, put rotation before
  % translation
  weights = [weights(4:6); weights(1:3)];
  l = block.DialogPrm(3).Data; % damping constant
  initialGuess = block.InputPort(3).Data; % output of last iteration

  % obtain the rigidbody tree from the dialog parameter
  configuration = block.DialogPrm(1).Data;
  % set the datatype of the articulation ("q") to be a column vector
  configuration.DataFormat = 'column';

  % get current pose of the endeffector and compute pose delta to target
  robotTCPName = block.DialogPrm(2).Data;
  maxIterations = block.DialogPrm(4).Data;
  % whether the block should compute its own initial guess
  computeInitialGuess = block.DialogPrm(6).Data;

  % ---------------- ITERATIONS --------------------------------
  distanceInitialGuessFromHome = norm(initialGuess - homeConfiguration(configuration));
  if distanceInitialGuessFromHome < 0.01 && computeInitialGuess
      disp("Computing initial guess from random samples (this may take a moment).")
      initialGuess = monteCarloInitialGuess(configuration, robotTCPName, targetPose);
  
  elseif distanceInitialGuessFromHome < 0.01 && ~computeInitialGuess 
      disp("Using home-pose as initial guess.");
  end
  articulation = initialGuess;
  % compute initial error before performing optimization
  tcpPose = getTransform(configuration, articulation, robotTCPName);
  % compute the weighted distance in the space frame: ||W * Ad_T * delta||
  currDistance = norm(diag(weights) * adjointSE3(tcpPose) * errorTwist(tcpPose, targetPose));
  numIterations = 0;
  minDistance = block.DialogPrm(5).Data;

  while numIterations < maxIterations && currDistance > minDistance
      deltaArticulation = iterateIK(l, configuration, articulation, robotTCPName, targetPose);
      articulation = articulation + deltaArticulation;
      tcpPose = getTransform(configuration, articulation, robotTCPName);
      % compute the weighted distance in the space frame: ||W * Ad_T * delta||
      currDistance = norm(diag(weights) * adjointSE3(tcpPose) * errorTwist(tcpPose, targetPose));
      numIterations = numIterations + 1;
  end

  if numIterations >= maxIterations
      disp("Failed to satisfy constraint after " + maxIterations + " iterations. Aborting.");
  end

  block.OutputPort(1).Data = articulation;
  

function SimStatusChange(block, s)
  
  block.Dwork(2).Data = block.Dwork(2).Data+1;    

  if s == 0
    disp('Pause in simulation.');
  elseif s == 1
    disp('Resume simulation.');
  end
  
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

%% custom function definitions

% compute error twist in body (=TCP frame)
% see "Modern Robotics", Ch. 6.2.2 p. 230, where the numerical inverse
% kinematics algorithm is explained step by step
function err = errorTwist(Ta, Tb)
   % pose delta as seen from tangent at point a
   deltaA =  logm(Ta \ Tb);
   err_tvec = deltaA(1:3, 4); % tangent translation vector
  w_x = deltaA(1:3, 1:3); % tangent rotation vector
  err_rvec = [w_x(3,2); w_x(1,3); w_x(2,1)];
  % weighted error vector in twist coordinates at the local
  % tangent-space
  err = [err_tvec; err_rvec];


% iteration step of damped least squares IK algorithm
% for more information on "damped least squares" (a Leveberg-Marquardt optimization strategy), see
% http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
% this particular algorithm uses Eq. (11) which is found on p. 10
function deltaArticulation = iterateIK(dampingFactor, robot, articulation, tcpName, targetPose)
    l = dampingFactor;
    tcpPose = getTransform(robot, articulation, tcpName);
    localError = errorTwist(tcpPose, targetPose);
    globalError = adjointSE3(tcpPose) * localError;
    J = spaceJacobian(robot, articulation);
    deltaArticulation = J.' / (J * J.' + l^2 * eye(6)) * globalError;
