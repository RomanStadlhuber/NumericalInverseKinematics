% NOTES TO SELF:

% Tutorial used to create the code that interfaces with the s-function
% block:
% https://de.mathworks.com/help/simulink/ug/tutorial-creating-a-custom-block.html#bq60ehy

% How to correctly set the 'Tunability' of the Block Dialog Parameters
% https://de.mathworks.com/matlabcentral/answers/29446-integration-using-s-functions-in-simulink

% How to name the input and output ports of a custom S-Function block
% https://de.mathworks.com/matlabcentral/answers/49553-how-to-display-port-names-in-s-function-block-just-like-they-show-up-for-simulink-subsystems


function DampedLeastSquaresIK(block)
%MSFUNTMPL A Template for a MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl' with the name
%   of your S-function.  

%   Copyright 2003-2018 The MathWorks, Inc.
  
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
  block.NumDialogPrms     = 3;
  block.DialogPrmsTunable = {'Nontunable', 'Nontunable', 'Tunable'}; 
  
  % Set up the continuous states.
  block.NumContStates = 1; % TODO: find out how many are needed

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  block.SampleTimes = [0 0];
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back to the 
  % MATLAB file
  block.SetAccelRunOnTLC(false);
  
  % Specify the block's operating point compliance. The block operating 
  % point is used during the containing model's operating point save/restore)
  % The allowed values are:
  %   'Default' : Same the block's operating point as of a built-in block
  %   'UseEmpty': No data to save/restore in the block operating point
  %   'Custom'  : Has custom methods for operating point save/restore
  %                 (see GetOperatingPoint/SetOperatingPoint below)
  %   'Disallow': Error out when saving or restoring the block operating point.
  block.OperatingPointCompliance = 'Default';
  
  % -----------------------------------------------------------------
  % The MATLAB S-function uses an internal registry for all
  % block methods. You should register all relevant methods
  % (optional and required) as illustrated below. You may choose
  % any suitable name for the methods and implement these methods
  % as local functions within the same file.
  % -----------------------------------------------------------------
   
  % -----------------------------------------------------------------
  % Register the methods called during update diagram/compilation.
  % -----------------------------------------------------------------
  
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
  % SetInputPortSamplingMode:
  %   Functionality    : Check and set input and output port 
  %                      attributes and specify whether the port is operating 
  %                      in sample-based or frame-based mode
  %   C MEX counterpart: mdlSetInputPortFrameData.
  %   (The DSP System Toolbox is required to set a port as frame-based)
  %
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
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
  % SetInputPortComplexSignal:
  %   Functionality    : Check and set the input and optionally the output
  %                      port complexity attributes.
  %   C MEX counterpart: mdlSetInputPortComplexSignal
  %
  block.RegBlockMethod('SetInputPortComplexSignal', @SetInpPortComplexSig);
  
  %
  % SetOutputPortComplexSignal:
  %   Functionality    : Check and set the output and optionally the input
  %                      port complexity attributes.
  %   C MEX counterpart: mdlSetOutputPortComplexSignal
  %
  block.RegBlockMethod('SetOutputPortComplexSignal', @SetOutPortComplexSig);
  
  %
  % PostPropagationSetup:
  %   Functionality    : Set up the work areas and the state variables. You can
  %                      also register run-time methods here.
  %   C MEX counterpart: mdlSetWorkWidths
  %
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  % -----------------------------------------------------------------
  % Register methods called at run-time
  % -----------------------------------------------------------------
  
  % 
  % ProcessParameters:
  %   Functionality    : Call to allow an update of run-time parameters.
  %   C MEX counterpart: mdlProcessParameters
  %  
  block.RegBlockMethod('ProcessParameters', @ProcessPrms);

  % 
  % InitializeConditions:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C MEX counterpart: mdlInitializeConditions
  % 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
  % 
  % Start:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C MEX counterpart: mdlStart
  %
  block.RegBlockMethod('Start', @Start);

  % 
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  %   C MEX counterpart: mdlOutputs
  %
  block.RegBlockMethod('Outputs', @Outputs);

  % 
  % Update:
  %   Functionality    : Call to update the discrete states
  %                      during a simulation step.
  %   C MEX counterpart: mdlUpdate
  %
  % block.RegBlockMethod('Update', @Update);

  % 
  % Derivatives:
  %   Functionality    : Call to update the derivatives of the
  %                      continuous states during a simulation step.
  %   C MEX counterpart: mdlDerivatives
  %
  block.RegBlockMethod('Derivatives', @Derivatives);
  
  % 
  % Projection:
  %   Functionality    : Call to update the projections during a
  %                      simulation step.
  %   C MEX counterpart: mdlProjections
  %
  block.RegBlockMethod('Projection', @Projection);
  
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

  %
  % GetOperatingPoint:
  %   Functionality    : Return the operating point of the block.
  %   C MEX counterpart: mdlGetOperatingPoint
  %
  block.RegBlockMethod('GetOperatingPoint', @GetOperatingPoint);
  
  %
  % SetOperatingPoint:
  %   Functionality    : Set the operating point data of the block using
  %                       from the given value.
  %   C MEX counterpart: mdlSetOperatingPoint
  %
  block.RegBlockMethod('SetOperatingPoint', @SetOperatingPoint);

  % -----------------------------------------------------------------
  % Register the methods called during code generation.
  % -----------------------------------------------------------------
  
  %
  % WriteRTW:
  %   Functionality    : Write specific information to model.rtw file.
  %   C MEX counterpart: mdlRTW
  %
  block.RegBlockMethod('WriteRTW', @WriteRTW);
%endfunction

% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

function CheckPrms(block)
% TODO: the tutorial uses a try catch block in the case that no parameter
% was provided, see if that's useful..


% load rigidbody tree
configuration = block.DialogPrm(1).Data;
tcpName = block.DialogPrm(2).Data;
dampingfactor = block.DialogPrm(3).Data;
% throw exception if the provided parameter is no rigidbody tree
if ~isa(configuration, 'rigidBodyTree')
    me = MSLException(block.BlockHandle, message('Requires a RigidBodyTree object'));
    throw(me)
end
% throw if tcp frame name is not a string
if ~isa(tcpName, "string")
    me = MSLException(block.BlockHandle, message('Requires a string of the TCP frames name'));
    trhow(me)
end
% throw if damping coefficient is not a real double scalar
if ~isa(dampingfactor, "double")
    me = MSLException(block.BlockHandle, message('DampingFactor needs to be a real scalar'));
    trhow(me)
end
%endfunction

function ProcessPrms(block)

  block.AutoUpdateRuntimePrms;
 
%endfunction

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  
%endfunction

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

%endfunction  

function SetInpPortComplexSig(block, idx, c)
  
  block.InputPort(idx).Complexity = c;
  block.OutputPort(1).Complexity  = c;

%endfunction 
  
function SetOutPortComplexSig(block, idx, c)

  block.OutputPort(idx).Complexity = c;
  block.InputPort(1).Complexity    = c;

%endfunction 
    
function DoPostPropSetup(block)
  block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'numPause';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 7;      % uint32
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%endfunction

function InitializeConditions(block)

block.ContStates.Data = 1;

%endfunction

function Start(block)

  block.Dwork(1).Data = 0;
  block.Dwork(2).Data = uint32(1); 
   
%endfunction

function WriteRTW(block)
  
   block.WriteRTWParam('matrix', 'M',    [1 2; 3 4]);
   block.WriteRTWParam('string', 'Mode', 'Auto');
   
%endfunction

function Outputs(block)
  
  % obtain Pose (4x4), Weights (6x1) and InitialGuess (6x1)
  targetPose = block.InputPort(1).Data;
  weights = block.InputPort(2).Data;
  % TODO: find out how to check if this is empty before loading it
  initialGuess = block.InputPort(3).Data;

  % obtain the rigidbody tree from the dialog parameter
  configuration = block.DialogPrm(1).Data;
  % set the datatype of the articulation ("q") to be a column vector
  configuration.DataFormat = 'column';

  % get current pose of the endeffector and compute pose delta to target
  robotBaseName = configuration.BaseName;
  robotTCPName = block.DialogPrm(2).Data;

  % ---------------- ITERATIONS --------------------------------
  maxIterations = 10;
  if norm(initialGuess) < 0.01
      initialGuess = monteCarloInitialGuess(configuration, robotTCPName, targetPose);
  end
  articulation = initialGuess;
  for k=1:maxIterations
      tcpPose = getTransform(configuration, articulation, robotBaseName, robotTCPName);
      % delta = SE3(0_T_E^-1 * T_goal)
      deltaPoseSE3 = se3(tcpPose/targetPose);
      % obtain tangent elements of the delta pose
      err_tvec = trvec(deltaPoseSE3);
      err_Rmat = rotm(deltaPoseSE3);
      w_x = logm(err_Rmat); % the corresponding tangent element of the rotation matrix
      err_rvec = w_x([8, 3, 4]);
      % weighted error vector in twist coordinates
      err = diag(weights) * ([err_tvec, err_rvec].');

      % ------------- DAMPED LEAST SQUARES LOGIC ---------------------------

      % clamp the error vector to a maximum distance, let's say 100
      d_max = 1;
      if norm(err) > d_max
          err = err/norm(err) * d_max;
      end
      % obtain the jacobian and formulate the dls constraint
      l = block.DialogPrm(3).Data; % damping constant
      J = geometricJacobian(configuration, articulation, robotTCPName);
      w_s = J(1:3, :);
      v_s = J(4:6, :);
      J = [v_s; w_s];
      [~, n] = size(J); % obtain columns of the jacobian, rows don't matter
      delta_articulation = (J.' * J + l^2 * eye(n)) \ J.' * err;
      articulation = articulation + delta_articulation;
  end
 
  block.OutputPort(1).Data = articulation;
  
%endfunction

% function Update(block)
%   
%   block.Dwork(1).Data = block.InputPort(1).Data;
  
%endfunction

function Derivatives(block)
%NOTE: unused
block.Derivatives.Data = 0*block.ContStates.Data;

%endfunction

function Projection(block)

states = block.ContStates.Data;
block.ContStates.Data = states+eps; 

%endfunction

function SimStatusChange(block, s)
  
  block.Dwork(2).Data = block.Dwork(2).Data+1;    

  if s == 0
    disp('Pause in simulation.');
  elseif s == 1
    disp('Resume simulation.');
  end
  
%endfunction
    
function Terminate(block)

disp(['Terminating the block with handle ' num2str(block.BlockHandle) '.']);

%endfunction
 
function operPointData = GetOperatingPoint(block)
% package the Dwork data as the entire operating point of this block
operPointData = block.Dwork(1).Data;

%endfunction

function SetOperatingPoint(block, operPointData)
% the operating point of this block is the Dwork data (this method 
% typically performs the inverse actions of the method GetOperatingPoint)
block.Dwork(1).Data = operPointData;

%endfunction
