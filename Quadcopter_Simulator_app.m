classdef Quadcopter_Simulator_app < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                matlab.ui.Figure
        RecordButton            matlab.ui.control.StateButton
        SouravParidaLabel       matlab.ui.control.Label
        MotorSpeedsinradsLabel  matlab.ui.control.Label
        AttitudesLabel          matlab.ui.control.Label
        PositionLabel           matlab.ui.control.Label
        w4EditField             matlab.ui.control.NumericEditField
        w4EditFieldLabel        matlab.ui.control.Label
        w2EditField             matlab.ui.control.NumericEditField
        w2EditFieldLabel        matlab.ui.control.Label
        w3EditField             matlab.ui.control.NumericEditField
        w3EditFieldLabel        matlab.ui.control.Label
        w1EditField             matlab.ui.control.NumericEditField
        w1EditFieldLabel        matlab.ui.control.Label
        EditField_3             matlab.ui.control.NumericEditField
        EditField_3Label        matlab.ui.control.Label
        EditField_2             matlab.ui.control.NumericEditField
        Label                   matlab.ui.control.Label
        EditField               matlab.ui.control.NumericEditField
        Label_2                 matlab.ui.control.Label
        zEditField              matlab.ui.control.NumericEditField
        zEditFieldLabel         matlab.ui.control.Label
        yEditField              matlab.ui.control.NumericEditField
        yEditFieldLabel         matlab.ui.control.Label
        xEditField              matlab.ui.control.NumericEditField
        xEditFieldLabel         matlab.ui.control.Label
        CurrentStateLabel       matlab.ui.control.Label
        StatusLamp              matlab.ui.control.Lamp
        StatusLampLabel         matlab.ui.control.Label
        StopSimulationButton    matlab.ui.control.Button
        DownArrowLabel          matlab.ui.control.Label
        UpArrowLabel            matlab.ui.control.Label
        HoverLabel              matlab.ui.control.Label
        ALabel                  matlab.ui.control.Label
        DLabel                  matlab.ui.control.Label
        SLabel                  matlab.ui.control.Label
        WLabel                  matlab.ui.control.Label
        LeftArrowLabel          matlab.ui.control.Label
        RightArrowLabel         matlab.ui.control.Label
        RollLabel               matlab.ui.control.Label
        PitchLabel              matlab.ui.control.Label
        YawLabel                matlab.ui.control.Label
        DownLabel               matlab.ui.control.Label
        UpLabel                 matlab.ui.control.Label
        ControlsLabel           matlab.ui.control.Label
        ResetButton             matlab.ui.control.Button
        StartSimulationButton   matlab.ui.control.Button
        UIAxes                  matlab.ui.control.UIAxes
    end

    
    properties (Access = public)
            m = 0.5 ;             % Mass of drone (Kg)
            l = 0.25;            % Length of drone arms, from the center (m)
            b = 7.5e-7;          % Drag constant
            L = 3e-6;            % Lift constant
            g = 9.81;
            Ixx = 0.0125;
            Iyy = 0.0125;
            Izz = 0.025;
           
            w = zeros(4,1);     % w is the matrix of angular velocities of the 4 motors x Time interval
            x = zeros(3,2);     % Relative position (x,y,z) of drone, relative to Earth Frame
            x_dot = zeros(3,2); % Derivate of x (linear velocity)
            eta = zeros(3,2);   % Attitudes of drone (roll, pitch, yaw)(Tait-Bryant Angles)
            tau = zeros(3,1);   % Torques of drone, in body frame
            T = zeros(3,1);     % Thrust of drone on the z axis in body frame
            vel = zeros(3, 2);  % Angular velocitiy vector in body Frame
            Winv = zeros(3,3);  % Angular velocity vector to rate of change of Tait-Bryant Angles Matrix
            Rx = zeros(3,3);    % x Rotation matrix
            Ry = zeros(3,3);    % y Rotation matrix
            Rz = zeros(3,3);    % z Rotation matrix
            R = zeros(3,3);     % Combined Rotation Transformation Matrix
            
            %% Simulation Preparation
            %Initial states
            wBase = 640;        % Base value for motor's speeds
            deltaW = 0.5;       % Step size for controlling the motors
            delta_time = 0.01;
            counter = 0;    
            sim = false;
            animLine = 0;
            Drone = [0.1768 -0.1768 0 -0.1768 0.1768; 0.1768 -0.1768 0 0.1768 -0.1768; 0 0 0 0 0];
            recordVideo = false;
            VW = VideoWriter('drone_motion.mp4','MPEG-4');
            end


    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: StartSimulationButton
        function StartSimulationButtonPushed(app, event)
            app.StatusLamp.Color = 'g';
            
            I = [app.Ixx 0 0; 0 app.Iyy 0; 0 0 app.Izz]; % Inertia Tensor
            A = 0.25/app.m;          % Aerodynamic effects
            app.sim = true;
            
            RotatedDrone = app.Drone;                   
            % Variables(Initialized to zero)
            
            app.w(1,1) =  app.wBase;
            app.w(2,1) =  app.wBase;
            app.w(3,1) =  app.wBase;
            app.w(4,1) =  app.wBase;
            
            % Main plot for 3D simulation
            %f = figure('units','normalized','outerposition',[0 0 1 1]);
            set(app.UIFigure,'WindowKeyPressFcn',@KeyDown, 'WindowKeyReleaseFcn', @KeyUp);
            p = plot3(app.UIAxes,RotatedDrone(1,:),RotatedDrone(2,:), RotatedDrone(3,:),'o','LineStyle','-', 'MarkerFaceColor',[0.2 0.2 0.2],'Color',[0.2 0.2 0.2]);
            xlabel(app.UIAxes,'x');ylabel(app.UIAxes,'y');zlabel(app.UIAxes,'z')
            app.animLine = animatedline(app.UIAxes,'MaximumNumPoints',1000,'Color','g');
            axislim = 2;
            axis(app.UIAxes,[-axislim axislim -axislim axislim -axislim axislim]);
            
            
            while app.sim
                
                app.counter = app.counter+1;
                phi = app.eta(1,1);
                theta = app.eta(2,1);
                psi = app.eta(3,1);
                
                w1squared = app.w(1)^2;
                w2squared = app.w(2)^2;
                w3squared = app.w(3)^2;
                w4squared = app.w(4)^2;
                % Angular momentum
                app.tau = [0.1768*app.L*(w1squared+w2squared-w3squared-w4squared);
                       0.1768*app.L*(w2squared+w3squared-w1squared-w4squared);
                       app.b*(w1squared - w2squared + w3squared - w4squared)];
                % Thrust on drone's z axis
                app.T = [0 0 app.L*(w1squared + w2squared + w3squared + w4squared)];
                
                % Angular velocities
                app.vel(:,2) = app.delta_time*( I\(-cross( app.vel(:,1), I*app.vel(:,1)) +app.tau) ) + app.vel(:,1);
                % Transformation matrix from Body to Earth frame
                app.Winv = [ 1 sin(phi)*tan(theta) cos(phi)*tan(theta);
                         0 cos(phi) -sin(phi);
                         0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
                % Attitudes
                app.eta(:,2) = app.delta_time*( app.Winv*app.vel(:,1)) + app.eta(:,1);
                
                % Transformation matrices
                
                app.R = [cos(psi)*cos(theta) cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
                     cos(theta)*sin(psi) cos(phi)*cos(psi)+sin(psi)*sin(phi)*sin(theta) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
                     -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];
                % Defining the three rotation matrices for 3d plotting
                app.Rx = [ 1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];            
                app.Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];          
                app.Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
                   
                % Final Position & Velocity w.r.t inertial frame
                app.x_dot(:,2) = app.delta_time*( -[0;0;app.g] + app.R*(1/app.m)*app.T' - A*app.x_dot(:,1)) + app.x_dot(:,1);
                app.x(:,2) = (app.delta_time*app.x_dot(:,1)) + app.x(:,1);
            
                if mod(app.counter, 2) == 0
                    % Updated drone plot
                    % Orientation of drone
                    RotatedDrone = app.Rz*app.Ry*app.Rx*(app.Drone) + app.x(:,2);
                    set(p,'XData', RotatedDrone(1,:),'YData', RotatedDrone(2,:),'ZData', RotatedDrone(3,:));
                    axis(app.UIAxes,[-axislim+app.x(1,2) axislim+app.x(1,2) -axislim+app.x(2,2) axislim+app.x(2,2) -axislim+app.x(3,2) axislim+app.x(3,2)]);
                    
                    % Drone's path tracing
                    addpoints(app.animLine,app.x(1,1),app.x(2,1),app.x(3,1));
                    drawnow;
                
                end
                if mod(app.counter, 20) == 0
                    app.xEditField.Value = app.x(1,1);
                    app.yEditField.Value = app.x(2,1);
                    app.zEditField.Value = app.x(3,1);
    
                    app.EditField.Value = app.eta(1,1);
                    app.EditField_2.Value = app.eta(2,1);
                    app.EditField_3.Value = app.eta(3,1);
                    
                    app.w1EditField.Value = app.w(1);
                    app.w2EditField.Value = app.w(2);
                    app.w3EditField.Value = app.w(3);
                    app.w4EditField.Value = app.w(4);
                end
                if mod(app.counter,2) == 0
                    if app.recordVideo
                        
                        writeVideo(app.VW,getframe(app.UIFigure));
                    end
                end
                
                %% Setting values of next iteration
                app.vel(:,1) = app.vel(:,2);
                app.eta(:,1) = app.eta(:,2);
                app.x_dot(:,1) = app.x_dot(:,2);
                app.x(:,1) = app.x(:,2);
            
            end  

             %% Key functions
            
            function KeyDown(~,key)
                switch key.Key
                    case 'w'
                        app.w(2) = app.w(2) + app.deltaW;
                        app.w(3) = app.w(3) +app.deltaW;
                        app.w(1) = app.w(1) -app.deltaW;
                        app.w(4) = app.w(4) -app.deltaW;
                    case 's'
                        app.w(2) = app.w(2) -app.deltaW;
                        app.w(3) = app.w(3) -app.deltaW;
                        app.w(1) = app.w(1) +app.deltaW;
                        app.w(4) = app.w(4) +app.deltaW;
                    case 'd'
                        app.w(1) = app.w(2) +app.deltaW;
                        app.w(2) = app.w(2) +app.deltaW;
                        app.w(3) = app.w(3) -app.deltaW;
                        app.w(4) = app.w(4) -app.deltaW; 
                    case 'a'
                        app.w(1) = app.w(2) -app.deltaW;
                        app.w(2) = app.w(2) -app.deltaW;
                        app.w(3) = app.w(3) +app.deltaW;
                        app.w(4) = app.w(4) +app.deltaW;
                    case 'uparrow'
                        app.w = app.w +app.deltaW*10;
                    case 'downarrow'
                        app.w = app.w -app.deltaW*10;
                    case 'leftarrow'
                        app.w = [app.w(1)-app.deltaW;app.w(2)+app.deltaW;app.w(3)-app.deltaW;app.w(4)+app.deltaW];
                    case 'rightarrow'
                        app.w = [app.w(1)+app.deltaW;app.w(2)-app.deltaW;app.w(3)+app.deltaW;app.w(4)-app.deltaW]';  
                    end
            end
            
            function KeyUp(~,~)
                app.w = [app.wBase;app.wBase;app.wBase;app.wBase];
            end
        end

        % Button pushed function: ResetButton
        function ResetButtonPushed(app, event)
            
            app.StatusLamp.Color = [0.65 0.65 0.65];
            pause(0.01)
            app.StatusLamp.Color = 'g';
            app.wBase = sqrt(app.m*app.g/(4*app.L));
            app.w = [app.wBase;app.wBase;app.wBase;app.wBase];
            app.x = zeros(3, 2);
            app.x_dot = zeros(3,2);
            app.eta = zeros(3, 2);
            app.tau = zeros(3,1);
            app.T = zeros(3,1);
            app.vel = zeros(3, 2);
            app.Winv = zeros(3,3);
            app.R = zeros(3,3);
            app.Rx = zeros(3,3);
            app.Ry = zeros(3,3);
            app.Rz = zeros(3,3);
            clearpoints(app.animLine);
        end

        % Button pushed function: StopSimulationButton
        function StopSimulationButtonPushed(app, event)
            app.sim = false;
            app.StatusLamp.Color = 'r';
        end

        % Value changed function: RecordButton
        function RecordButtonValueChanged(app, event)
            if ~app.recordVideo 
                
                open(app.VW);
                app.recordVideo = true;
                app.RecordButton.Text = 'Recording';
            else
               
               app.recordVideo = false;
               close(app.VW);
               app.RecordButton.Text = 'Record';
               app.VW = VideoWriter('drone_motion.mp4','MPEG-4');
            end
         
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Color = [0.651 0.651 0.651];
            app.UIFigure.Position = [100 100 1134 682];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.WindowStyle = 'alwaysontop';

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'X-Config Quadcopter Simulation')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.MinorGridLineStyle = '-';
            app.UIAxes.XColor = [0 0 0];
            app.UIAxes.XMinorTick = 'on';
            app.UIAxes.YColor = [0 0 0];
            app.UIAxes.YMinorTick = 'on';
            app.UIAxes.ZColor = [0 0 0];
            app.UIAxes.ZMinorTick = 'on';
            app.UIAxes.Color = [0.502 0.502 0.502];
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.ZGrid = 'on';
            app.UIAxes.TickDir = 'both';
            app.UIAxes.GridColor = [0.0588 1 1];
            app.UIAxes.MinorGridColor = [0.0588 1 1];
            app.UIAxes.Position = [30 52 773 603];

            % Create StartSimulationButton
            app.StartSimulationButton = uibutton(app.UIFigure, 'push');
            app.StartSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @StartSimulationButtonPushed, true);
            app.StartSimulationButton.BackgroundColor = [0.4118 0.4118 0.4118];
            app.StartSimulationButton.FontColor = [1 1 1];
            app.StartSimulationButton.Position = [67 10 100 22];
            app.StartSimulationButton.Text = 'Start Simulation';

            % Create ResetButton
            app.ResetButton = uibutton(app.UIFigure, 'push');
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.ResetButton.BackgroundColor = [0.4118 0.4118 0.4118];
            app.ResetButton.FontColor = [1 1 1];
            app.ResetButton.Position = [310 10 100 22];
            app.ResetButton.Text = 'Reset';

            % Create ControlsLabel
            app.ControlsLabel = uilabel(app.UIFigure);
            app.ControlsLabel.FontSize = 20;
            app.ControlsLabel.FontWeight = 'bold';
            app.ControlsLabel.Position = [930 609 88 24];
            app.ControlsLabel.Text = 'Controls';

            % Create UpLabel
            app.UpLabel = uilabel(app.UIFigure);
            app.UpLabel.FontWeight = 'bold';
            app.UpLabel.Position = [931 581 25 22];
            app.UpLabel.Text = 'Up';

            % Create DownLabel
            app.DownLabel = uilabel(app.UIFigure);
            app.DownLabel.FontWeight = 'bold';
            app.DownLabel.Position = [1012 581 38 22];
            app.DownLabel.Text = 'Down';

            % Create YawLabel
            app.YawLabel = uilabel(app.UIFigure);
            app.YawLabel.FontWeight = 'bold';
            app.YawLabel.Position = [868 548 29 22];
            app.YawLabel.Text = 'Yaw';

            % Create PitchLabel
            app.PitchLabel = uilabel(app.UIFigure);
            app.PitchLabel.FontWeight = 'bold';
            app.PitchLabel.Position = [868 515 35 22];
            app.PitchLabel.Text = 'Pitch';

            % Create RollLabel
            app.RollLabel = uilabel(app.UIFigure);
            app.RollLabel.FontWeight = 'bold';
            app.RollLabel.Position = [868 484 28 22];
            app.RollLabel.Text = 'Roll';

            % Create RightArrowLabel
            app.RightArrowLabel = uilabel(app.UIFigure);
            app.RightArrowLabel.FontWeight = 'bold';
            app.RightArrowLabel.Position = [928 548 74 22];
            app.RightArrowLabel.Text = 'Right Arrow';

            % Create LeftArrowLabel
            app.LeftArrowLabel = uilabel(app.UIFigure);
            app.LeftArrowLabel.FontWeight = 'bold';
            app.LeftArrowLabel.Position = [1013 548 65 22];
            app.LeftArrowLabel.Text = 'Left Arrow';

            % Create WLabel
            app.WLabel = uilabel(app.UIFigure);
            app.WLabel.FontWeight = 'bold';
            app.WLabel.Position = [943 515 25 22];
            app.WLabel.Text = 'W';

            % Create SLabel
            app.SLabel = uilabel(app.UIFigure);
            app.SLabel.FontWeight = 'bold';
            app.SLabel.Position = [1030 515 25 22];
            app.SLabel.Text = 'S';

            % Create DLabel
            app.DLabel = uilabel(app.UIFigure);
            app.DLabel.FontWeight = 'bold';
            app.DLabel.Position = [941 484 25 22];
            app.DLabel.Text = 'D';

            % Create ALabel
            app.ALabel = uilabel(app.UIFigure);
            app.ALabel.FontWeight = 'bold';
            app.ALabel.Position = [1030 484 25 22];
            app.ALabel.Text = 'A';

            % Create HoverLabel
            app.HoverLabel = uilabel(app.UIFigure);
            app.HoverLabel.FontWeight = 'bold';
            app.HoverLabel.Position = [862 454 40 22];
            app.HoverLabel.Text = 'Hover';

            % Create UpArrowLabel
            app.UpArrowLabel = uilabel(app.UIFigure);
            app.UpArrowLabel.FontWeight = 'bold';
            app.UpArrowLabel.Position = [923 454 59 22];
            app.UpArrowLabel.Text = 'Up Arrow';

            % Create DownArrowLabel
            app.DownArrowLabel = uilabel(app.UIFigure);
            app.DownArrowLabel.FontWeight = 'bold';
            app.DownArrowLabel.Position = [1011 454 76 22];
            app.DownArrowLabel.Text = 'Down Arrow';

            % Create StopSimulationButton
            app.StopSimulationButton = uibutton(app.UIFigure, 'push');
            app.StopSimulationButton.ButtonPushedFcn = createCallbackFcn(app, @StopSimulationButtonPushed, true);
            app.StopSimulationButton.BackgroundColor = [0.4118 0.4118 0.4118];
            app.StopSimulationButton.FontColor = [1 1 1];
            app.StopSimulationButton.Position = [188 10 100 22];
            app.StopSimulationButton.Text = 'Stop Simulation';

            % Create StatusLampLabel
            app.StatusLampLabel = uilabel(app.UIFigure);
            app.StatusLampLabel.HorizontalAlignment = 'right';
            app.StatusLampLabel.Position = [62 643 40 22];
            app.StatusLampLabel.Text = 'Status';

            % Create StatusLamp
            app.StatusLamp = uilamp(app.UIFigure);
            app.StatusLamp.HandleVisibility = 'callback';
            app.StatusLamp.Position = [117 643 20 20];
            app.StatusLamp.Color = [0.651 0.651 0.651];

            % Create CurrentStateLabel
            app.CurrentStateLabel = uilabel(app.UIFigure);
            app.CurrentStateLabel.FontSize = 20;
            app.CurrentStateLabel.FontWeight = 'bold';
            app.CurrentStateLabel.Position = [916 406 132 24];
            app.CurrentStateLabel.Text = 'Current State';

            % Create xEditFieldLabel
            app.xEditFieldLabel = uilabel(app.UIFigure);
            app.xEditFieldLabel.HorizontalAlignment = 'right';
            app.xEditFieldLabel.Position = [816 342 25 22];
            app.xEditFieldLabel.Text = 'x';

            % Create xEditField
            app.xEditField = uieditfield(app.UIFigure, 'numeric');
            app.xEditField.Position = [856 342 100 22];

            % Create yEditFieldLabel
            app.yEditFieldLabel = uilabel(app.UIFigure);
            app.yEditFieldLabel.HorizontalAlignment = 'right';
            app.yEditFieldLabel.Position = [816 289 25 22];
            app.yEditFieldLabel.Text = 'y';

            % Create yEditField
            app.yEditField = uieditfield(app.UIFigure, 'numeric');
            app.yEditField.Position = [856 289 100 22];

            % Create zEditFieldLabel
            app.zEditFieldLabel = uilabel(app.UIFigure);
            app.zEditFieldLabel.HorizontalAlignment = 'right';
            app.zEditFieldLabel.Position = [816 232 25 22];
            app.zEditFieldLabel.Text = 'z';

            % Create zEditField
            app.zEditField = uieditfield(app.UIFigure, 'numeric');
            app.zEditField.Position = [856 232 100 22];

            % Create Label_2
            app.Label_2 = uilabel(app.UIFigure);
            app.Label_2.HorizontalAlignment = 'right';
            app.Label_2.Position = [969 342 25 22];
            app.Label_2.Text = 'φ';

            % Create EditField
            app.EditField = uieditfield(app.UIFigure, 'numeric');
            app.EditField.Position = [1009 342 100 22];

            % Create Label
            app.Label = uilabel(app.UIFigure);
            app.Label.HorizontalAlignment = 'right';
            app.Label.Position = [968 289 25 22];
            app.Label.Text = 'θ';

            % Create EditField_2
            app.EditField_2 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_2.Position = [1009 289 100 22];

            % Create EditField_3Label
            app.EditField_3Label = uilabel(app.UIFigure);
            app.EditField_3Label.HorizontalAlignment = 'right';
            app.EditField_3Label.Position = [969 232 25 22];
            app.EditField_3Label.Text = 'ψ';

            % Create EditField_3
            app.EditField_3 = uieditfield(app.UIFigure, 'numeric');
            app.EditField_3.Position = [1009 232 100 22];

            % Create w1EditFieldLabel
            app.w1EditFieldLabel = uilabel(app.UIFigure);
            app.w1EditFieldLabel.HorizontalAlignment = 'right';
            app.w1EditFieldLabel.Position = [816 139 25 22];
            app.w1EditFieldLabel.Text = 'w1';

            % Create w1EditField
            app.w1EditField = uieditfield(app.UIFigure, 'numeric');
            app.w1EditField.Position = [856 139 100 22];

            % Create w3EditFieldLabel
            app.w3EditFieldLabel = uilabel(app.UIFigure);
            app.w3EditFieldLabel.HorizontalAlignment = 'right';
            app.w3EditFieldLabel.Position = [818 90 25 22];
            app.w3EditFieldLabel.Text = 'w3';

            % Create w3EditField
            app.w3EditField = uieditfield(app.UIFigure, 'numeric');
            app.w3EditField.Position = [858 90 100 22];

            % Create w2EditFieldLabel
            app.w2EditFieldLabel = uilabel(app.UIFigure);
            app.w2EditFieldLabel.HorizontalAlignment = 'right';
            app.w2EditFieldLabel.Position = [969 139 25 22];
            app.w2EditFieldLabel.Text = 'w2';

            % Create w2EditField
            app.w2EditField = uieditfield(app.UIFigure, 'numeric');
            app.w2EditField.Position = [1009 139 100 22];

            % Create w4EditFieldLabel
            app.w4EditFieldLabel = uilabel(app.UIFigure);
            app.w4EditFieldLabel.HorizontalAlignment = 'right';
            app.w4EditFieldLabel.Position = [969 90 25 22];
            app.w4EditFieldLabel.Text = 'w4';

            % Create w4EditField
            app.w4EditField = uieditfield(app.UIFigure, 'numeric');
            app.w4EditField.Position = [1009 90 100 22];

            % Create PositionLabel
            app.PositionLabel = uilabel(app.UIFigure);
            app.PositionLabel.FontWeight = 'bold';
            app.PositionLabel.Position = [886 374 53 22];
            app.PositionLabel.Text = 'Position';

            % Create AttitudesLabel
            app.AttitudesLabel = uilabel(app.UIFigure);
            app.AttitudesLabel.FontWeight = 'bold';
            app.AttitudesLabel.Position = [1030 374 57 22];
            app.AttitudesLabel.Text = 'Attitudes';

            % Create MotorSpeedsinradsLabel
            app.MotorSpeedsinradsLabel = uilabel(app.UIFigure);
            app.MotorSpeedsinradsLabel.FontWeight = 'bold';
            app.MotorSpeedsinradsLabel.Position = [920 174 131 22];
            app.MotorSpeedsinradsLabel.Text = 'Motor Speeds in rad/s';

            % Create SouravParidaLabel
            app.SouravParidaLabel = uilabel(app.UIFigure);
            app.SouravParidaLabel.Position = [1009 10 118 22];
            app.SouravParidaLabel.Text = '2022 | Sourav Parida';

            % Create RecordButton
            app.RecordButton = uibutton(app.UIFigure, 'state');
            app.RecordButton.ValueChangedFcn = createCallbackFcn(app, @RecordButtonValueChanged, true);
            app.RecordButton.Text = 'Record';
            app.RecordButton.BackgroundColor = [0.9412 0.9412 0.9412];
            app.RecordButton.FontColor = [1 0 0];
            app.RecordButton.Position = [444 10 100 22];

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Quadcopter_Simulator_app

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end