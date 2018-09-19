% executive( ConfigFile, mm_ip)
%
% ConfigFile is the file name of the main config file that should be
% loaded from the config/ directory
%
% mm_ip is the network address of the MessageManager
%
% Ivana Stevens 8/21/2018
function executive( ConfigFile, mm_ip )

NOTE_TEXT = 'Subject 12345'; % Will Be "Sonic" eventually
NOTE_EPOC = 'Tick';

MessageTypes = {...
    'MT_FORCE_FEEDBACK' ...
    'MT_POSITION_FEEDBACK' ...
    'MT_TRIAL_STATUS_FEEDBACK' ...
    'MT_RT_POSITION_FEEDBACK' ...
    'MT_TASK_STATE_CONFIG' ...
    'MT_BURT_STATUS' ...
    'EXIT' ...
    'PING' ...
    'GIVE_REWARD' ...
    };

Dragonfly_BaseDir = getenv('DRAGONFLY');

addpath([Dragonfly_BaseDir '/lang/matlab']);
App_SourceDir = getenv('BCI_MODULES');
App_IncludeDir = getenv('BCI_INCLUDE');

MessageConfigFile = [App_IncludeDir '/Dragonfly_config.mat'];
ModuleID = 'EXEC_MOD';

ConnectArgs = {ModuleID, '', MessageConfigFile};
if exist('mm_ip','var') && ~isempty(mm_ip)
    ConnectArgs{end+1} = ['-server_name ' mm_ip];
end

ConnectToMMM(ConnectArgs{:});
Subscribe( MessageTypes{:})

disp 'RewardModule running...'

% TODO Read in yaml file
directions = [2, 2, 2, 2];
forces = [0.01, 0.02, 0.03, 0.04];
widths = [3, 5, 10];
distances = [0.4, 0.6, 0.8];

% Initialize targetList
targetList = initTargetList(directions, forces, widths, distances);
targetList.begin();
currentTarget = targetIter;
nextTarget = targetIter;

% Set next Target
trial_input_data.data.state = currentState;
trial_input_data.data.direction = currentTarget.direction;
trial_input_data.data.force = currentTarget.force;
trial_input_data.data.distance = currentTarget.distance;
trial_input_data.data.target_width = currentTarget.width;

while(true)
    fprintf('\nWaiting for message\n');
    M = ReadMessage( 'blocking');
    
    switch(M.msg_type)
        % This should probably be a different message
        case 'MT_TASK_STATE_CONFIG'
            break;
            
            % BURT sent an msg about state termination.
        case 'MT_BURT_STATUS'
            burt_status_data = M.Data;
            % if there is a success for given state
            if (burt_status_data.task_complete)
                % move to the next state
                if (burt_status_data.task_success)
                    shouldReset = true;
                    if (~userDefState)
                        % progress next state as usual if not set else where
                        nextState = RESET;
                    end
                    % Subject error occurred, start next trial, no reward
                else
                    rewardable = false;
                    shouldReset = true;
                end
                % pull up next TARGET parameters
                if ( nextTarget == currentTarget)
                    currentTarget = ++targetIter;
                else
                    currentTarget = nextTarget;
                end
                
                % Determine next state
                if (true) % nextState ==  RESET || shouldReset)
                    % Determine next target (send out with state)
                    
                    % Send out RESET messages and next target parameters
                    
                    % Set next Target
                    trial_input_data.direction = currentTarget.direction;
                    trial_input_data.force = currentTarget.force;
                    trial_input_data.distance = currentTarget.distance;
                    trial_input_data.target_width = currentTarget.width;
                    
                    % Print target definition
                    print 'Sending out next trial data...'
                    
                    % If this is a rewardable transition, do so.
                    if (rewardable)
                        print 'REWARD!!!';
                    end
                    
                    % Reset variables
                    shouldReset = false;
                    userDefState = false;
                    userDefTarget = false;
                    nextTarget = currentTarget;
                    
                    % TODO: THIS SHOULD BE REMOVED LATER
                    trial_input_data.state =  START;
                    %Send message
                    task_state_config_M.Data = trial_input_data;
                    mod.SendMessageDF( task_state_config_M);
                else
                    % Send message
                    trial_input_data.state = REST;
                    task_state_config_M.Data = trial_input_data;
                    mod.SendMessageDF( task_state_config_M);
                end
            end
            break;
            
        case 'GIVE_REWARD'
            break;
            
        case 'EXIT'
            break;
    end
    
end
DisconnectFromMMM

end


% blah blah blah. documentation
function [ targetList ] = initTargetList( dirs, forces, widths, distances )

end


% blah blah blah. documentation
function [ targetChanged ] = modifyTask(edits, targetIter, nextTarget, rewardable)
bool targetChanged = false;
switch (edits.type)    
    case 'CONTROL' % Manually control the robot position. TODO
    case 'FREEZE' % Freeze the robot in place. TODO
    case 'HOME' % Move the robot back home. TODO
    case 'REPEAT' % Repeat current trial
        
        % Automatically used nextTrial which already
        % is the same as the current
        targetChanged = true;
    case 'REWARD' % Give a reward now
    case 'REWARD_SET' % Set reward level
    case 'REWIND' % Do last trial again
        nextTarget = --targetIter;
        targetChanged = true;
    case 'SKIP' % Skip next trial
        ++targetIter;
        nextTarget = ++targetIter;
        targetChanged = true;
    case 'TARGET' % Set the target
        dir = edits.direction;
        dist = edits.distance;
        force = edits.force;
        wid = edits.target_width;
        node = TargetNode(dist, dir, wid, force);
        nextTarget = node;
        targetChanged = true;
        
    otherwise % Bad messgae sent. Error (?)
end
end

function [ out ] = readJson( fname )
% readJson:  Read in json config file
fid = fopen(fname);
raw = fread(fid,inf);
str = char(raw');
fclose(fid);
out = jsondecode(str);

end  % function