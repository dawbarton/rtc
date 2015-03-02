classdef (ConstructOnLoad) rtc_interface < handle
    %RTC_INTERFACE Interface to the real-time control device. As far as
    %  possible this object is designed to be state-less. (The exception
    %  being the parameter list.)

    properties (Hidden)
        MAX_PACKET_SIZE = 64;
        CMD_GET_PAR_NAMES = 10;
        CMD_GET_PAR_SIZES = 11;
        CMD_GET_PAR_TYPES = 12;
        CMD_GET_PAR_VALUE = 20;
        CMD_SET_PAR_VALUE = 25;
        CMD_GET_STREAM    = 30;
        CMD_SOFT_RESET    = 90;
        CMD_ACK           = uint32(4291668991);
        STREAM_STATE_INACTIVE = 0;
        STREAM_STATE_ACTIVE   = 1;
        STREAM_STATE_FINISHED = 2;
        dev;
        libusb_notfound;
        libusb_warnings;
        timeout;
        intf;
        ep_in;
        ep_out;
        par_info;
        par_idx;
    end

    properties
        par; % Parameters accessible to the user. 
        opt; % Default options for the rig.
    end
    
    methods (Hidden)
        function send_raw(obj, data)
            % SEND_RAW  Send raw data to the RTC device (internal).
            transferred = libpointer('int32Ptr', 0);
            dataptr = libpointer('uint8Ptr', data);
            r = calllib('libusb','libusb_bulk_transfer', obj.dev, obj.ep_out, dataptr, length(data), transferred, obj.timeout);
            if r ~= 0
                error('Error in bulk transfer send');
            end
            if transferred.value ~= length(data)
                error('Failed to write all the data');
            end
        end
        
        function send_cmd(obj, cmd, data)
            % SEND_CMD  Send a command to the RTC device (internal).
            cmd = typecast(uint16(cmd), 'uint8');
            datalen = typecast(uint32(length(data)), 'uint8');
            if isempty(data)
                packet = horzcat(cmd, datalen);
                obj.send_raw(packet);
            else
                crc = typecast(crc32(data), 'uint8');
                packet = horzcat(cmd, datalen, data, crc);
                obj.send_raw(packet);
            end
        end
        
        function output = recv_raw(obj, nbytes)
            % RECV_RAW  Receive raw data from the RTC device (internal).
            if mod(nbytes, obj.MAX_PACKET_SIZE)
                warning('rtc_interface:recv_raw', 'recv_raw should be used with a multiple of MAX_PACKET_SIZE bytes to avoid potential data corruption');
            end
            if nbytes > 104857600
                % Check that we aren't trying to read too much data
                error('Trying to receive an insane amount of data');
            end
            dataptr = libpointer('uint8Ptr', zeros(1, nbytes, 'uint8'));
            transferred = libpointer('int32Ptr', 0);
            r = calllib('libusb','libusb_bulk_transfer', obj.dev, obj.ep_in, dataptr, nbytes, transferred, obj.timeout);
            if r ~= 0
                error('Error in bulk transfer read');
            end
            output = dataptr.value(1:transferred.value);
        end
        
        function output = recv_cmd(obj)
            % RECV_CMD  Receive the results of a command from the RTC device (internal).
            data = obj.recv_raw(obj.MAX_PACKET_SIZE);
            if length(data) < 4
                error('No intelligible response from RTC device');
            end
            data_len = typecast(data(1:4), 'uint32');
            % Check for a simple acknowledgement response
            if data_len == obj.CMD_ACK
                output = 1;
                return;
            end
            % Check how much data is needed
            while data_len + 8 > length(data)
                data_needed = data_len + 8 - length(data);
                if mod(data_needed, obj.MAX_PACKET_SIZE) ~= 0
                    data_needed = data_needed + obj.MAX_PACKET_SIZE - mod(data_needed, obj.MAX_PACKET_SIZE);
                end
                new_data = obj.recv_raw(data_needed);
                if isempty(new_data)
                    error('Timed out while waiting for data');
                end
                data = horzcat(data, new_data); %#ok<AGROW>
            end
            % Process the data
            output = data(5:data_len+4);
            output_crc = typecast(data(data_len+5:data_len+8), 'uint32');
            crc = crc32(output);
            if crc ~= output_crc
                error('Error in data stream returned from RTC device - CRC failed');
            end            
        end
        
        function get_par_list(obj)
            % GET_PAR_LIST Get the parameters available to access on the RTC device (internal).
            
            % Types that we might have to deal with
            type_sizes = containers.Map();
            type_sizes('b') = 1; type_sizes('h') = 2; type_sizes('i') = 4;
            type_sizes('B') = 1; type_sizes('H') = 2; type_sizes('I') = 4; 
            type_sizes('c') = 1; type_sizes('f') = 4;
            type_conv = containers.Map();
            type_conv('b') = 'int8'; type_conv('h') = 'int16'; type_conv('i') = 'int32';
            type_conv('B') = 'uint8'; type_conv('H') = 'uint16'; type_conv('I') = 'uint32';
            type_conv('f') = 'single'; type_conv('c') = 'char';
            % Get the parameter names
            obj.send_cmd(obj.CMD_GET_PAR_NAMES, []);
            names = char(obj.recv_cmd());
            % Get the parameter sizes
            obj.send_cmd(obj.CMD_GET_PAR_SIZES, []);
            sizes = typecast(obj.recv_cmd(), 'uint32');
            % Get the parameter types
            obj.send_cmd(obj.CMD_GET_PAR_TYPES, []);
            types = char(obj.recv_cmd());
            % Find the delimiters between names
            idx = [0, find(names == 0)];
            % Construct an appropriate structures to store all the information
            obj.par_info = struct('id', cell(1, length(idx) - 1), ...
                                  'name', '', ...
                                  'size', 0, ...
                                  'type', '', ...
                                  'count', 0);
            obj.par_idx = containers.Map();
            % Store all the information
            for i = 1:length(idx)-1
                obj.par_info(i).id = uint32(i - 1);
                obj.par_info(i).name = names(idx(i)+1:idx(i+1)-1);
                obj.par_info(i).size = sizes(i);
                obj.par_info(i).type = type_conv(types(i));
                obj.par_info(i).count = sizes(i)/type_sizes(types(i));
                obj.par_idx(obj.par_info(i).name) = i;
            end
        end
    end
    
    methods
        function obj = rtc_interface()
            %RTC_INTERFACE Interface to the real-time control device. As far as
            %  possible this object is designed to be state-less. (The exception
            %  being the parameter list.)
            
            % Load the library
            if ~libisloaded('libusb')
                [obj.libusb_notfound, obj.libusb_warnings] = loadlibrary('libusb','libusb.h');
            end
            calllib('libusb', 'libusb_init', libpointer);
            obj.dev = calllib('libusb', 'libusb_open_device_with_vid_pid', libpointer, hex2dec('0123'), hex2dec('4567'));
            if obj.dev.isNull()
                error('Unable to open the RTC device');
            else
                fprintf('Connected to RTC device\n');
            end
            r = calllib('libusb','libusb_claim_interface', obj.dev, 0);
            if r < 0
                error('Failed to claim the RTC device');
            end
            
            % Set the end points and interface of interest
            obj.intf = 0;
            obj.ep_out = 1;
            obj.ep_in = 129;

            % The timeout for any USB communications
            obj.timeout = 5000;

            % Get the parameter names
            obj.get_par_list();

            % A helper class for getting/setting parameters
            obj.par = rtc_parameters(obj);
            
            % Set the underlying device name
            obj.opt.device = 'BBB-RTC';
        end
        
        function delete(obj)
            % DELETE  Destroy the interface to the real-time controller.
            if ~obj.dev.isNull()
                calllib('libusb', 'libusb_close', obj.dev);
                fprintf('Released RTC device\n');
            end
            calllib('libusb', 'libusb_exit', libpointer);
        end
        
        function set_par(obj, names, values)
            % SET_PAR  Set the values of the specified parameters.
            %
            % OBJ.SET_PAR(NAME, VALUE) sets the value of the parameter NAME to VALUE.
            % Both NAME and VALUE can be cell arrays in the case of setting multiple
            % parameter values simultaneously.

            if ~iscell(names)
                names = {names};
                values = {values};
            end
            % Pack the data appropriately
            payload = uint8([]);
            for i = 1:length(names)
                idx = obj.par_idx(names{i});
                par_id = obj.par_info(idx).id;
                par_type = obj.par_info(idx).type;
                par_count = obj.par_info(idx).count;
                if length(values{i}) ~= par_count
                    error('Incorrect number of values supplied for parameter %s', names{i});
                end
                value = cast(values{i}, par_type);
                if strcmp(par_type, 'char')
                    payload = horzcat(payload, typecast(par_id, 'uint8'), ...
                                      cast(value, 'uint8')); %#ok<AGROW>
                else
                    payload = horzcat(payload, typecast(par_id, 'uint8'), ...
                                      typecast(value, 'uint8')); %#ok<AGROW>
                end
            end
            % Send the command
            obj.send_cmd(obj.CMD_SET_PAR_VALUE, payload);
            % Get the response
            if obj.recv_cmd() ~= 1
                error('Failed to set parameter values');
            end
        end
        
        function values = get_par(obj, names)
            % GET_PAR  Get the values of the specified parameters.
            %
            % OBJ.GET_PAR(NAME) gets the value of the parameter NAME. NAME can be a
            % cell array to get multiple parameter values simultaneously.
            
            if ~iscell(names)
                names = {names};
                islist = 0;
            else
                islist = 1;
            end
            % Get the ids associated with the names provided
            par_ids = zeros(1, length(names), 'uint32');
            par_idxs = zeros(1, length(names));
            for i = 1:length(names)
                par_idxs(i) = obj.par_idx(names{i});
                par_ids(i) = obj.par_info(par_idxs(i)).id;
            end
            % Request the parameter values
            obj.send_cmd(obj.CMD_GET_PAR_VALUE, typecast(par_ids, 'uint8'));
            data = obj.recv_cmd();
            % Reformat the data as appropriate
            idx = 1;
            data_list = cell(1, length(names));
            for i = 1:length(names)
                par_type = obj.par_info(par_idxs(i)).type;
                par_size = obj.par_info(par_idxs(i)).size;
                if strcmp(par_type, 'char')
                    data_list{i} = cast(data(idx:idx + par_size - 1), ...
                                        par_type);
                else
                    data_list{i} = typecast(data(idx:idx + par_size - 1), ...
                                            par_type);
                    if strcmp(par_type, 'single')
                        data_list{i} = cast(data_list{i}, 'double');
                    end
                end
                idx = idx + par_size;
            end
            % Return as a list or raw data depending on what was passed originally
            if islist
                values = data_list;
            else
                values = data_list{1};
            end
        end
        
        function set_stream(obj, stream, parameters, samples, downsample)
            % SET_STREAM  Set stream recording properties.
            %
            % OBJ.SET_STREAM(ID, NAMES, SAMPLES, DOWNSAMPLE) sets the stream with
            % identifier ID (where multiple streams are available) to record the
            % parameters given by the cell array NAMES. SAMPLES data points are
            % recorded and DOWNSAMPLE data points are discarded between each recorded
            % sample.
            %
            % Example
            %
            %   rtc.set_stream(0, {'x', 'out'}, 1000, 0);
            %
            % will set stream id 0 to record the parameters x and out. 1000 samples
            % will be returned with no data discarded.
            
            stream_name = sprintf('S%d', stream);
            if exist('samples', 'var') && ~isempty(samples)
                if samples > 0
                    obj.set_par([stream_name 'samples'], samples);
                end
            end
            if exist('downsample', 'var') && ~isempty(downsample)
                if downsample >= 0
                    obj.set_par([stream_name 'downsample'], downsample);
                end
            end
            if exist('parameters', 'var') && ~isempty(parameters)
                npars = obj.par_info(obj.par_idx([stream_name 'params'])).count;
                par_ids = zeros(1, npars, 'uint32');
                par_ids(:) = 4294967295;
                if length(parameters) > npars
                    error('Too many parameters provided');
                end
                for i = 1:length(parameters)
                    stream_par = obj.par_info(obj.par_idx(parameters{i}));
                    par_ids(i) = stream_par.id;
                    if ~(strcmp(stream_par.type, 'int32') || strcmp(stream_par.type, 'uint32') || strcmp(stream_par.type, 'single'))
                        error('Stream parameters must be either floats or ints (%s is %s)', stream_par.name, stream_par.type);
                    end
                end
                obj.set_par([stream_name 'params'], par_ids);
            end
        end
        
        function data = get_stream(obj, stream, return_struct)
            % GET_STREAM  Get the data from a particular stream.
            %
            % OBJ.GET_STREAM(ID) returns an array of data recorded in the stream given
            % by ID. If the stream is not ready, no data is returned.
            %
            % OBJ.GET_STREAM(ID, true) returns a structure with named fields containing
            % the data recorded in the stream given by ID.
            %
            % See also START_STREAM.
            stream_name = sprintf('S%d', stream);
            if obj.get_par([stream_name 'state']) ~= obj.STREAM_STATE_FINISHED
                data = [];
                return;
            end
            par_ids = obj.get_par([stream_name 'params']);
            par_count = find(par_ids >= length(obj.par_info), 1) - 1;
            par_ids = par_ids(1:par_count);
            obj.send_cmd(obj.CMD_GET_STREAM, typecast(uint32(stream), 'uint8'));
            raw_data = typecast(obj.recv_cmd(), 'uint32');
            raw_data = reshape(raw_data, [par_count, length(raw_data)/par_count]);
            if exist('return_struct', 'var') && return_struct
                for i = 1:par_count
                    par_type = obj.par_info(par_ids(i) + 1).type;
                    par_name = obj.par_info(par_ids(i) + 1).name;
                    data.(par_name) = typecast(raw_data(i, :), par_type);
                    if strcmp(par_type, 'single')
                        data.(par_name) = cast(data.(par_name), 'double');
                    end
                end
            else
                data = zeros(size(raw_data));
                for i = 1:par_count
                    par_type = obj.par_info(par_ids(i) + 1).type;
                    data(i, :) = cast(typecast(raw_data(i, :), par_type), 'double');
                end
            end
        end
        
        function result = start_stream(obj, stream)
            % START_STREAM  Start a stream recording.
            %
            % OBJ.START_STREAM(ID) starts the stream given by ID recording data with
            % the current parameters from SET_STREAM.
            %
            % See also SET_STREAM.
            stream_name = sprintf('S%d', stream);
            obj.set_par([stream_name 'state'], obj.STREAM_STATE_ACTIVE);
            if obj.get_par([stream_name 'state']) ~= obj.STREAM_STATE_INACTIVE
                result = 1;
            else
                result = 0;
            end
        end
        
        function data = run_stream(obj, stream, varargin)
            % RUN_STREAM  Start a stream recording and then return the captured data.
            %
            % OBJ.RUN_STREAM(ID) starts the stream given by ID and then returns the
            % captured data.
            %
            % OBJ.RUN_STREAM(ID, Name, Value) overrides the default options for running
            % the stream.
            %
            % Options
            %
            %     start: allowed values are true or false. Default true.
            %         Whether or not to start the stream running before waiting for
            %         available captured data.
            %
            %     wait_period: allowed values are a > 0. Default 0.1.
            %         The period of time the function should pause before checking if
            %         there is captured data available.
            %
            %     struct: allowed values are true or false. Default false.
            %         Whether or not to return the data as a structure.
            %
            % See also START_STREAM, GET_STREAM.
            p = inputParser();
            if ismethod(p, 'addParameter')
                % New versions of Matlab
                add_par = @p.addParameter;
            else
                % Old versions of Matlab
                add_par = @p.addParamValue;
            end
            add_par('start', true, @islogical);
            add_par('wait_period', 0.1, @(x)(x > 0));
            add_par('struct', false, @islogical);
            p.parse(varargin{:});
            stream_name = sprintf('S%d', stream);
            if p.Results.start
                if ~obj.start_stream(stream)
                    error('Failed to start stream - perhaps bad parameters');
                end
            elseif obj.get_par([stream_name 'state']) == obj.STREAM_STATE_INACTIVE
                error('Stream not already started');
            end
            while obj.get_par([stream_name 'state']) == obj.STREAM_STATE_ACTIVE
                pause(p.Results.wait_period);
            end
            data = obj.get_stream(stream, p.Results.struct);
        end
        
        function b = saveobj(~)
            b = [];
        end
    end
    
    methods (Static = true)
        function a = loadobj(~)
            a = [];
        end
    end
    
end
