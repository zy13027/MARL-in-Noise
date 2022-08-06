classdef CustomGridWorldV0 < matlab.System
   % GridWorld represents a 12x12 deterministic grid world with 3 robots.

% Copyright 2020-2022 The MathWorks, Inc.
    
    % Public, tunable properties
%     properties
%         % Initial position of robots
%         InitialStates (1,2) double 
% 
%        
%     end
    
    properties(Dependent)
        % Initial position of robots
        InitialStates (1,2) double 

        % CurrentState
        % String with name of current state
%         CurrentState string
        
        % States
        % String vector of the state names
        States string
        
        % Actions
        % String vector with the action names
        Actions string

         % ObstacleStates
        % String vector with the state names which can not be reached in
        % the grid world.
        ObstacleStates string
        
        % TerminalStates
        % String vector with the state names which are terminal in the grid
        % world.
        TerminalStates string


          % add terminal 
    end

    % Public, non-tunable properties
    properties (Nontunable)
        % Obstacle matrix
        Obstacles double = -1
        % Max step count
        MaxStepCount (1,1) double = 500

        %%%&&&&&&&&&&&&&&&&&&&
%         TerminalStates (1,2) double   % add terminal 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    properties(DiscreteState)
        % Discretized XY space with cells containing 0.5 or 1.0
        % 0:   unexplored
        % 0.25: explored by robot A
        % 0.50: explored by robot B
        % 0.75: explored by robot B
        % 1.0: obstacle
        Grid
        % States of robot A,B: [rowA colA; rowB colB]
%         States
        % Step count
        StepCount
        % Individual cell exploration count
%         NumExploredCells
    end

    % Pre-computed constants
    properties(Access = private)
        % Handle to figure
%         Figure
        % Grid size [numrows numcols]
        Size (1,2) double 
    end

    methods
        % Constructor
        function this = CustomGridWorldV0(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj) %#ok<MANU>
            % Perform one-time calculations, such as computing constants
        end

        function [observations,rewards,isdone] = stepImpl(obj,actions)
                numRobots = 1;
            
            % Rewards are:
           
            % Agent moves to Terminal State: 10
            % Agent tries to move out of grid: -10
            % Agent collides with another agent: -10
            % Agent collides with obstacle: -10
            % Movement penalty: -1
            % Lazy penalty: -2
            % On full coverage: +4000 * coverage contribution
            
            % move robots to their next state
            %next_states = zeros(numRobots,2);
            rewards = zeros(numRobots,1);
            isdone = 0;
            next_states = obj.States;
            for idx = 1:numRobots
                state = obj.States(idx,:);
                action = actions(idx);
                switch(action)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%                     case 0
%                         % Wait
%                         next_states(idx,:) = state;
%                         rewards(idx) = rewards(idx) - 10;  % lazy penalty

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


                    case 1
                        % Move up North
                        if state(1) < obj.Size(1) && ~checkCollision(obj,state+[1 0],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [1 0];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go up
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                    case 2
                        % Move down South
                        if state(1) > 1 && ~checkCollision(obj,state+[-1 0],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [-1 0];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go down
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                    case 4
                        % Move left WEst
                        if state(2) > 1 && ~checkCollision(obj,state+[0 -1],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [0 -1];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go left
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                    case 3
                        % Move right East
                        if state(2) < obj.Size(2) && ~checkCollision(obj,state+[0 1],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [0 1];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go right
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                     case 5
                        % Move right-up  NorthEast
                        if state(2) < obj.Size(2) && ~checkCollision(obj,state+[1 1],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [1 1];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go right
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                
                       case 6
                        % Move Left up NorthWest 
                        if state(2) < obj.Size(2) && ~checkCollision(obj,state+[1 -1],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [1 -1];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go right
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                        case 7
                        % Move Right Down  South East
                        if state(2) < obj.Size(2) && ~checkCollision(obj,state+[-1 1],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [-1 1];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go right
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                        case 8
                        % Move Left Down South West
                        if state(2) < obj.Size(2) && ~checkCollision(obj,state+[-1 -1],next_states(idx~=1:numRobots,:))
                            next_states(idx,:) = state + [-1 -1];
                            rewards(idx) = rewards(idx) - 1;
                        else
                            % add a negative reward for trying to go right
                            next_states(idx,:) = state;
                            rewards(idx) = rewards(idx) - 10;
                        end
                end
                end
        end
    end
end