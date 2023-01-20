function viz(robot, outTrajectory, targetPositions, outJointStates)

ax = show(robot,outJointStates(:,end));

% Font
ax.FontName = "Arial Narrow";
ax.FontSize = 12;
ax.FontWeight = "bold";

% Ticks
ax.XTick = [-1.5 -1 -0.5 0 0.5 1 1.5];
ax.XTickLabel = [-1.5 -1 -0.5 0 0.5 1 1.5];
ax.XTickMode = "manual";
ax.XTickLabelMode = "auto";

ax.YTick = [-1.5 -1 -0.5 0 0.5 1 1.5];
ax.YTickLabel = [-1.5 -1 -0.5 0 0.5 1 1.5];
ax.YTickMode = "manual";
ax.YTickLabelMode = "auto";

ax.ZTick = [0 0.5 1 1.5];
ax.ZTickLabel = [0 0.5 1 1.5];
ax.ZTickMode = "manual";
ax.ZTickLabelMode = "auto";

% Rulers
ax.XLim = [-1.0 1.0];
ax.XLimMode = "manual";
ax.XAxisLocation = "origin";
ax.XColor = [1 0 0];
ax.XColorMode = "manual";

ax.YLim = [-1.0 1.0];
ax.YLimMode = "manual";
ax.YAxisLocation = "origin";
ax.YColor = [0 1 0];
ax.YColorMode = "manual";

ax.ZLim = [0 1.0];
ax.ZLimMode = "manual";
ax.ZColor = [0 0 1];
ax.ZColorMode = "manual";

% Grids
ax.XGrid = "on";
ax.YGrid = "on";
ax.ZGrid = "on";
ax.GridColor = [0.15 0.15 0.15];
ax.GridColorMode = "manual";

% Position
ax.OuterPosition = [0 0.023619631901841 1 0.785276073619632];
ax.InnerPosition = [0.13 0.11 0.775 0.815];
ax.Position = [0.13 0.11 0.775 0.64];
ax.PlotBoxAspectRatio = [2 2 1];
ax.PlotBoxAspectRatioMode = "manual";

% View
ax.View = [45 45];
ax.Projection = "orthographic";
ax.CameraPosition = [10 -10 10];
ax.CameraPositionMode = "manual";
ax.CameraTarget = [0 0 0];
ax.CameraTargetMode = "manual";
ax.CameraUpVector = [0 0 1];
ax.CameraUpVectorMode = "auto";
ax.CameraViewAngle = 10;

[~, numWP] = size(targetPositions);

xyz = zeros(numWP, 3);
for i = 1:size(outTrajectory,3)
    xyz(i,:) = tform2trvec(outTrajectory(:,:,i));
end

hold on
plot3(ax, xyz(:,1),xyz(:,2),xyz(:,3),'-k','LineWidth',2);
plot3(ax, targetPositions(1,:),targetPositions(2,:),targetPositions(3,:),'--r','LineWidth',2)
hold off