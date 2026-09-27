classdef GridWorld < matlab.System
% GridWorld represents a deterministic area-coverage grid world with one to
% three robots.
%
% Actions (one per robot):
%   0 wait
%   1 up (+row)        2 down (-row)      3 left (-col)      4 right (+col)
%   5 up-right         6 up-left          7 down-right       8 down-left

% Copyright 2020-2022 The MathWorks, Inc.

    % Public, tunable properties
    properties
        % Initial position of robots, one [row col] per robot. Only the
        % first NumRobots rows are used.
        InitialStates (:,2) double = [2 2; 11 4; 3 12]
    end

    % Public, non-tunable properties
    properties(Nontunable)
        % Obstacle matrix, one [row col] per obstacle (-1 for none)
        Obstacles double = -1
        % Max step count
        MaxStepCount (1,1) double = 500
        % Grid size [numrows numcols]
        GridSize (1,2) double = [64 64]
        % Number of robots moving in the grid (1 to 3)
        NumRobots (1,1) double = 1
        % Plot the grid every step (slow, leave off for training)
        PlotEnvironment (1,1) logical = false
    end

    properties(DiscreteState)
        % Discretised XY space with cells containing
        % 0:    unexplored
        % 0.25: explored by robot A
        % 0.50: explored by robot B
        % 0.75: explored by robot C
        % 1.0:  obstacle
        Grid
        % States of robots: [rowA colA; rowB colB; ...]
        States
        % Step count
        StepCount
        % Individual cell exploration count
        NumExploredCells
    end

    % Pre-computed constants
    properties(Access = private)
        % Obstacle cells are 1, rest 0
        ObstacleMask
        % Number of cells that are not obstacles
        NumFreeCells
    end

    properties(Constant, Access = private)
        % [row col] offset of each move; row k is action k
        Moves = [1 0; -1 0; 0 -1; 0 1; 1 1; 1 -1; -1 1; -1 -1]
    end

    methods
        % Constructor
        function this = GridWorld(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function validatePropertiesImpl(obj)
            if ~any(obj.NumRobots == [1 2 3])
                error('GridWorld:NumRobots','NumRobots must be 1, 2 or 3.');
            end
            if size(obj.InitialStates,1) < obj.NumRobots
                error('GridWorld:InitialStates', ...
                    'InitialStates needs one [row col] row per robot.');
            end
            s = obj.InitialStates(1:obj.NumRobots,:);
            if any(s(:) < 1) || any(s(:,1) > obj.GridSize(1)) || any(s(:,2) > obj.GridSize(2))
                error('GridWorld:InitialStates', ...
                    'InitialStates must lie inside the %dx%d grid.',obj.GridSize(1),obj.GridSize(2));
            end
        end

        function setupImpl(obj)
            % The obstacle layout is fixed, so work it out once
            obj.ObstacleMask = zeros(obj.GridSize);
            if ~isequal(obj.Obstacles,-1)
                for idx = 1:size(obj.Obstacles,1)
                    obj.ObstacleMask(obj.Obstacles(idx,1),obj.Obstacles(idx,2)) = 1.0;
                end
            end
            obj.NumFreeCells = numel(obj.ObstacleMask) - nnz(obj.ObstacleMask);
        end

        function [observations,rewards,isdone] = stepImpl(obj,actions)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.

            numRobots = obj.NumRobots;

            % Rewards are:
            % Agent moves to unexplored cell: +20
            % Agent moves to explored cell: 0
            % Agent tries to move out of grid: -10
            % Agent collides with another agent: -10
            % Agent collides with obstacle: -10
            % Movement penalty: -1
            % Lazy penalty: -10
            % On full coverage: +4000 * coverage contribution

            % move robots to their next state
            rewards = zeros(numRobots,1);
            isdone = 0;
            next_states = obj.States;
            for idx = 1:numRobots
                state = obj.States(idx,:);
                action = actions(idx);
                if action == 0
                    % Wait
                    rewards(idx) = rewards(idx) - 10;  % lazy penalty
                elseif action >= 1 && action <= size(obj.Moves,1) && ...
                        isFreeCell(obj,state + obj.Moves(action,:),next_states((1:numRobots) ~= idx,:))
                    next_states(idx,:) = state + obj.Moves(action,:);
                    rewards(idx) = rewards(idx) - 1;
                else
                    % Off the grid, blocked, or not a valid action: stay put
                    rewards(idx) = rewards(idx) - 10;
                end
            end

            % update grid and reward agents for new exploration
            for idx = 1:numRobots
                r = next_states(idx,1);
                c = next_states(idx,2);
                if obj.Grid(r,c) == 0.0
                    % robot explores an unexplored cell
                    rewards(idx) = rewards(idx) + 20;
                    obj.Grid(r,c) = 0.25*idx;  % explored by A, B or C
                    obj.NumExploredCells(idx) = obj.NumExploredCells(idx) + 1;
                end
            end

            % coverage metrics
            if sum(obj.NumExploredCells) >= obj.NumFreeCells
                isdone = 1;
                rewards = rewards + 4000 * (obj.NumExploredCells(:)/obj.NumFreeCells);
            end

            % Observation for each agent is a GridSize 4-channel image. The
            % channels are:
            % 1. Obstacle channel - cells with obstacles are 1, rest 0
            % 2. Self channel - cell with the agent's state is 1, rest 0
            % 3. Friend channel - cells with other agents' states are 1, rest 0
            % 4. Coverage channel - cells that are unexplored are 1, rest 0
            observations = zeros(obj.GridSize(1),obj.GridSize(2),4,numRobots);
            coverageChannel = 1.0 * (obj.Grid == 0);
            for idx = 1:numRobots
                observations(:,:,1,idx) = obj.ObstacleMask;
                observations(:,:,4,idx) = coverageChannel;
                for k = 1:numRobots
                    if k == idx
                        channel = 2;  % self
                    else
                        channel = 3;  % friend
                    end
                    observations(next_states(k,1),next_states(k,2),channel,idx) = 1.0;
                end
            end

            % Scale down rewards
            rewards = rewards./20;

            % DEBUG
            for i = 1:numRobots-1
                for j = i+1:numRobots
                    if all(next_states(i,:) == next_states(j,:))
                        fprintf('Assertion: Invalid state.\n');
                    end
                end
            end

            % Update states
            obj.States = next_states;
            obj.StepCount = obj.StepCount + 1;

            % plot the environment
            if obj.PlotEnvironment && obj.StepCount <= obj.MaxStepCount
                plot(obj);
            end

        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties

            % set unexplored and obstacle cells
            obj.Grid = obj.ObstacleMask;

            % set step count
            obj.StepCount = 0;

            % set robot cells; each start cell counts as explored
            obj.States = obj.InitialStates(1:obj.NumRobots,:);
            obj.NumExploredCells = ones(1,obj.NumRobots);
            for idx = 1:obj.NumRobots
                obj.Grid(obj.States(idx,1),obj.States(idx,2)) = 0.25*idx;
            end
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            if isLocked(obj)
                s.ObstacleMask = obj.ObstacleMask;
                s.NumFreeCells = obj.NumFreeCells;
            end
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            if wasLocked
                obj.ObstacleMask = s.ObstacleMask;
                obj.NumFreeCells = s.NumFreeCells;
            end

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Simulink functions
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds.Grid = obj.Grid;
            ds.States = obj.States;
            ds.StepCount = obj.StepCount;
            ds.NumExploredCells = obj.NumExploredCells;
        end

        function flag = isInputSizeMutableImpl(obj,index) %#ok<INUSD>
            % Return false if input size cannot change
            % between calls to the System object
            flag = false;
        end

        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            if obj.NumRobots == 1
                out1 = [obj.GridSize 4];                 % observation
            else
                out1 = [obj.GridSize 4 obj.NumRobots];   % observation
            end
            out2 = [obj.NumRobots 1];   % reward
            out3 = [1 1];               % isdone
        end

        function [out1,out2,out3] = getOutputDataTypeImpl(obj) %#ok<MANU>
            % Return data type for each output port
            out1 = "double";
            out2 = "double";
            out3 = "double";
        end

        function [out1,out2,out3] = isOutputComplexImpl(obj) %#ok<MANU>
            % Return true for each output port with complex data
            out1 = false;
            out2 = false;
            out3 = false;
        end

        function [out1,out2,out3] = isOutputFixedSizeImpl(obj) %#ok<MANU>
            % Return true for each output port with fixed size
            out1 = true;
            out2 = true;
            out3 = true;
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            if strcmpi(name,'Grid')
                sz = obj.GridSize;
            elseif strcmpi(name,'States')
                sz = [obj.NumRobots 2];
            elseif strcmpi(name,'StepCount')
                sz = [1 1];
            elseif strcmpi(name,'NumExploredCells')
                sz = [1 obj.NumRobots];
            else
                error('GridWorld:StateName','Incorrect state name: %s',name);
            end
            dt = "double";
            cp = false;
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = class(obj); % Use class name, also for subclasses
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename("class"));
        end

    end

    methods(Access=private)
        function free = isFreeCell(obj,new_state,other_states)
            % True if new_state is inside the grid, not an obstacle and not
            % occupied by another robot
            free = all(new_state >= 1) && all(new_state <= obj.GridSize) && ...
                obj.ObstacleMask(new_state(1),new_state(2)) == 0 && ...
                ~any(all(new_state == other_states,2));
        end

        function plot(obj)
            persistent ax cells robots

            if isempty(ax) || ~isvalid(ax) || numel(robots) ~= obj.NumRobots
                % build figure
                f = figure;
                f.Position = [195 120 400 300];
                %f.Visible = 'on';  % force external figure
                ax = gca(f);
                hold(ax,'on');
                % With CLim fixed to [0 1] the five colours line up with the
                % cell values 0, 0.25, 0.50, 0.75 and 1.0
                cmap = [255 255 255; ...    % white  (unexplored)
                        255 140 105; ...    % light red (explored by A)
                        152 251 152; ...    % light green (explored by B)
                        176 226 255; ...    % light blue (explored by C)
                        0 0 0]./255;        % black (obstacles)
                colormap(ax,cmap);
                ax.CLim = [0 1];

                % plot cells
                numRows = obj.GridSize(1);
                numCols = obj.GridSize(2);
                cells = imagesc(ax,[0.5 numCols-0.5],[0.5 numRows-0.5],obj.Grid);

                % plot grid lines
                [X,Y] = meshgrid(0:numCols,0:numRows);
                plot(ax,X,Y,'Color',[0.94 0.94 0.94]);    % vertical
                plot(ax,X',Y','Color',[0.94 0.94 0.94]);  % horizontal

                % plot robots
                colours = 'rgb';
                robots = gobjects(1,obj.NumRobots);
                for idx = 1:obj.NumRobots
                    robots(idx) = rectangle(ax,'Position',[0 0 1 1], ...
                        'FaceColor',colours(idx),'Curvature',1);
                end

                ax.XTick = 0:numCols;
                ax.YTick = 0:numRows;
                ax.XTickLabel = {};
                ax.YTickLabel = {};
                axis(ax,'equal');
                ax.XLim = [0 numCols];
                ax.YLim = [0 numRows];
                ax.Box = 'on';
                grid(ax,'on');
            end

            % update cell colors
            cells.CData = obj.Grid;

            % update robot positions
            for idx = 1:obj.NumRobots
                s = obj.States(idx,:);
                robots(idx).Position = [s(2)-1 s(1)-1 1 1];
            end

            % update info text
            coverage = sum(obj.NumExploredCells) / obj.NumFreeCells * 100;
            ax.Title.String = sprintf('Steps = %d, Coverage = %.1f%%',obj.StepCount,coverage);

            drawnow limitrate;
        end
    end
end
