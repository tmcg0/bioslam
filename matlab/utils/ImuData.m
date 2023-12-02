% -------------------------------------------------------------------- %
%    (c) Copyright 2021 Massachusetts Institute of Technology          %
%    Author: Tim McGrath                                               %
%    All rights reserved. See LICENSE file for license information.    %
% -------------------------------------------------------------------- %

classdef ImuData < handle
    properties (Hidden)
        ax; ay; az % (m/s^2) | acceleration in x, y, z
        gx; gy; gz % (rad/s) | angular velocity x, y, z
        mx; my; mz % [uT]    | magnetic field x, y, z
        button % binary vector of button pushed (1) or not (0)
        fullfile % full path to h5 file
        unixTime % unix time in microseconds
        datenumVec % vector time in MATLAB datenum format
        ID % XI-###### (string)
        num % parsed numbers from ID (double)
        accelEnabled, gyroEnabled, magEnabled
    end
    properties % public properties
        name % string for dataset name
        label % string label (set in MotionStudio)
        numSamples % number of samples
        data % sampled IMU Data (all, Nx9)
        time % time vector
        sampleRate % Sample rate (Hz)
        filename % just end file title (with .h5)
        qAPDM % filtered quaternion dataset from APDM
        startTime % string for global start time
        endTime % string for global data end time
        temperature % temp in deg C
    end
    properties (Constant, Hidden)
        gc=9.80665; % accel due to gravity, m/s^2
    end
    methods
        function obj=ImuData(varargin) % construct from h5 file
            %% handle inputs
            switch nargin
                case 0
                    % warning('ImuData() needs to be passed a string which represents a valid .h5 file. Creating blank object.')
                    return
                case 1
                    FILE=varargin{1};
                    if ~exist(FILE,'file') % file doesn't exist
                        error('file does not exist.');
                    end
                otherwise
                    error('right now can''t support more than one .h5 file');
            end % handling of inputs
            obj=construct_v5(FILE,obj);
        end
        
        function g=gyro(obj)
            % return gyroscope measurements as an Nx3 matrix in rad/s
            g=[obj.gx obj.gy obj.gz];
        end
        
        function a=accel(obj)
            % return accelerometer measurements as an Nx3 matrix in m/s^2
            a=[obj.ax obj.ay obj.az];
        end
        
        function m=mag(obj)
            % return magnetometer measurements as an Nx3 matrix in uT
            m=[obj.mx obj.my obj.mz];
        end
        
    end
end

function obj=construct_v5(FILE,obj)
% Construct ImuData from APDM .h5 file format version 5
% There seems to be an ambiguity. Sometimes it's Sensors/999/..,
% Somtimes it's Sensors/XI-000999/...
% create a switch in get_sensor_list to handle two types
%     h5disp('v5_one_sensor.h5')  % for example
all_str=strsplit(FILE,'\');
filename=all_str{end}; % just end file name
[ID_list,num_list,stringMode]=get_sensor_list_v5(FILE);
% ID_list - > cell array of strings XI-######
% num_list - > array of doubles that are the ID number
% stringMode=1 wants ID_list, 2 wants num_list
numSensors=length(num_list);
textprogressbar(sprintf('Constructing %d IMU(s): ',numSensors)); textprogressbar(0); % initialize textprogressbar
for n=1:numSensors % number of sensors in the file
    obj(n).fullfile=FILE;
    obj(n).filename=filename;
    if stringMode==1 % use ID_list (XI-######)
        monstr=ID_list{n};
    elseif stringMode==2 % use num_list
        monstr=num2str(num_list(n));
    end
    obj(n).ID=ID_list{n};
    obj(n).name=obj(n).ID;
    obj(n).num=num_list(n);
    
    % setting all data
    % label
    label0=strrep(h5readatt(FILE,strcat('/Sensors/',monstr,'/Configuration/'),'Label 0'),char(0),'');
    label1=strrep(h5readatt(FILE,strcat('/Sensors/',monstr,'/Configuration/'),'Label 1'),char(0),'');
    label2=strrep(h5readatt(FILE,strcat('/Sensors/',monstr,'/Configuration/'),'Label 2'),char(0),''); % you can add up to three labels here
    allLabels={label0,label1,label2};
    allLabels(strcmp(allLabels,''))=[];
    obj(n).label=strjoin(allLabels,'/');
    % button
    try
        button = h5read(FILE,'/Annotations');
        if ~isempty(button.Time)
            obj(n).buttonEnabled = 1;
            obj(n).button = double(button.Time);
        else
            obj(n).buttonEnabled = 0;
        end
    catch % couldn't read button data
    end
    % accelerometer data:
    obj(n).accelEnabled=double(h5readatt(FILE,strcat('/Sensors/',monstr,'/Configuration/'),'Accelerometer Enabled'));
    if obj(n).accelEnabled
        accels=h5read(FILE,strcat('/Sensors/',monstr,'/Accelerometer'));
        obj(n).ax=accels(1,:)'; % (m/s^2) | acceleration in x
        obj(n).ay=accels(2,:)'; % (m/s^2) | acceleration in y
        obj(n).az=accels(3,:)'; % (m/s^2) | acceleration in z
    else; obj(n).ax=nan; obj(n).ay=nan; obj(n).az=nan;
    end
    
    % gyro data (angular velocities):
    obj(n).gyroEnabled=double(h5readatt(FILE,strcat('/Sensors/',monstr,'/Configuration/'),'Gyroscope Enabled'));
    if obj(n).gyroEnabled
        gyro=h5read(FILE,strcat('/Sensors/',monstr,'/Gyroscope'));
        obj(n).gx=gyro(1,:)'; % (rad/s) | angular velocity x
        obj(n).gy=gyro(2,:)'; % (rad/s) | angular velocity y
        obj(n).gz=gyro(3,:)'; % (rad/s) | angular velocity z
        %         gyroCal=h5read(FILE,strcat('/',monstr,'/Calibrated/','Gyroscopes'));
        %         obj(n).gxCal=gyroCal(1,:)'; % (rad/s) | angular velocity x
        %         obj(n).gyCal=gyroCal(2,:)'; % (rad/s) | angular velocity y
        %         obj(n).gzCal=gyroCal(3,:)'; % (rad/s) | angular velocity z
    else; obj(n).gx=nan; obj(n).gy=nan; obj(n).gz=nan;
    end
    
    % magnetometer data:
    obj(n).magEnabled=double(h5readatt(FILE,strcat('/Sensors/',monstr,'/Configuration/'),'Magnetometer Enabled'));
    if obj(n).magEnabled
        mags=h5read(FILE,strcat('/Sensors/',monstr,'/Magnetometer'));
        convmultiplier=1; % so stay in uT
        obj(n).mx=mags(1,:)'.*convmultiplier; % [uT] -> [uT] | magnetic field x
        obj(n).my=mags(2,:)'.*convmultiplier; % [uT] -> [uT] | magnetic field y
        obj(n).mz=mags(3,:)'.*convmultiplier; % [uT] -> [uT] | magnetic field z
    else; obj(n).mx=nan; obj(n).my=nan; obj(n).mz=nan;
    end
    
    obj(n).numSamples=length(obj(n).ax);
    obj(n).qAPDM=h5read(FILE,strcat('/Processed/',monstr,'/Orientation'))';
    
    % timevector
    obj(n).unixTime=double(h5read(FILE,strcat('/Sensors/',monstr,'/Time'))); % unix time in microseconds
    obj(n).datenumVec=obj(n).unixTime./86400./1E6 + datenum(1970,1,1)-datenum(0,0,0,5,0,0); % subtract 5 hours to put in EST
    obj(n).startTime = datestr(obj(n).datenumVec(1));
    obj(n).endTime = datestr(obj(n).datenumVec(end));
    timevec=obj(n).unixTime-obj(n).unixTime(1);
    obj(n).time=timevec./1E6; % convert to seconds to store in obj(n)
    obj(n).sampleRate=1/(obj(n).time(3)-obj(n).time(2)); % Hz
    
    obj(n).temperature=h5read(FILE,strcat('/Sensors/',monstr,'/Temperature')); % temperature in deg C
    
    textprogressbar(n/numSensors*100); % update textprogressbar
end
textprogressbar(' Complete!');
end 

function [ID_list,num_list,stringMode]=get_sensor_list_v5(FILE)
% retrieves sensor list from v5 files
% makes a switch to see if it wants ID_list or num_list
info=h5info(FILE,'/Sensors/');
numSensors=length(info.Groups);
for k=1:numSensors
    sensor_paths{k}=info.Groups(k).Name;
    trash=strsplit(sensor_paths{k},'/');
    endstring=trash{end};
    if strcmp(endstring(1:2),'XI') % pull last 4 numbers, convert to num
        ID_list{k}=endstring;
        num_list(k)=str2num(endstring(end-3:end));
        stringMode=1; % wants ID list (XI-######)
    else
        num_list(k)=str2num(endstring); % straight number in list
        s='XI-000000'; % lets put numbers in there
        s(end-(length(num2str(num_list(k)))-1):end)=num2str(num_list(k));
        ID_list{k}=s;
        stringMode=2; % wants num_list
    end
end
end % retrieve sensor list v5

function textprogressbar(c)
% This function creates a text progress bar. It should be called with a
% STRING argument to initialize and terminate. Otherwise the number correspoding
% to progress in % should be supplied.
% INPUTS:   C   Either: Text string to initialize or terminate
%                       Percentage number to show progress (from 0 to 100)
% OUTPUTS:  N/A
% Example:  Please refer to demo_textprogressbar.m

% Author: Paul Proteus (e-mail: proteus.paul (at) yahoo (dot) com)
% Version: 1.0
% Changes tracker:  29.06.2010  - First version

% Inspired by: http://blogs.mathworks.com/loren/2007/08/01/monitoring-progress-of-a-calculation/

%% Initialization
persistent strCR;           %   Carriage return pesistent variable

% Vizualization parameters
strPercentageLength = 10;   %   Length of percentage string (must be >5)
strDotsMaximum      = 10;   %   The total number of dots in a progress bar

%% Main

if isempty(strCR) && ~ischar(c),
    % Progress bar must be initialized with a string
    error('The text progress must be initialized with a string');
elseif isempty(strCR) && ischar(c),
    % Progress bar - initialization
    fprintf('%s',c);
    strCR = -1;
elseif ~isempty(strCR) && ischar(c),
    % Progress bar  - termination
    strCR = [];
    fprintf([c '\n']);
elseif isnumeric(c)
    % Progress bar - normal progress
    c = floor(c);
    percentageOut = [num2str(c) '%%'];
    percentageOut = [percentageOut repmat(' ',1,strPercentageLength-length(percentageOut)-1)];
    nDots = floor(c/100*strDotsMaximum);
    dotOut = ['[' repmat('.',1,nDots) repmat(' ',1,strDotsMaximum-nDots) ']'];
    strOut = [percentageOut dotOut];
    
    % Print it on the screen
    if strCR == -1,
        % Don't do carriage return during first run
        fprintf(strOut);
    else
        % Do it during all the other runs
        fprintf([strCR strOut]);
    end
    
    % Update carriage return
    strCR = repmat('\b',1,length(strOut)-1);
    
else
    % Any other unexpected input
    error('Unsupported argument type');
end
end % text progress bar
