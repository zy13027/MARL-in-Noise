classdef CustomGridWorld < GridWorld
% CustomGridWorld is a GridWorld with terminal (goal) cells. A robot that
% reaches a terminal cell earns an extra reward and ends the episode.
%
% Rewards are those of GridWorld, plus:
%   Agent moves to Terminal State: +10

% Copyright 2020-2022 The MathWorks, Inc.

    % Public, tunable properties
    properties
        % Terminal cells, one [row col] per row (-1 for none)
        TerminalStates double = -1
    end

    methods
        % Constructor
        function this = CustomGridWorld(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function [observations,rewards,isdone] = stepImpl(obj,actions)
            [observations,rewards,isdone] = stepImpl@GridWorld(obj,actions);

            if ~isequal(obj.TerminalStates,-1)
                for idx = 1:obj.NumRobots
                    if any(all(obj.States(idx,:) == obj.TerminalStates,2))
                        % +10, scaled down like the GridWorld rewards
                        rewards(idx) = rewards(idx) + 10/20;
                        isdone = 1;
                    end
                end
            end
        end
    end

    methods(Static, Access = protected)
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"));
        end

        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog,
            % including TerminalStates
            group = matlab.system.display.Section(mfilename("class"));
        end
    end
end
