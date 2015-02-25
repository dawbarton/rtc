classdef (ConstructOnLoad) rtc_interface < rtc_generic_interface
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
    end

    properties
        dev;
        libusb_notfound;
        libusb_warnings;
        timeout;
        intf;
        ep_in;
        ep_out;
        par_info;
        par_idx;
        par;
    end
    
    methods
        function obj = rtc_interface()
            % function obj = rtc_interface()
            %
            % Construct an interface to the RTC device.
            
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
            
        end
        
        function delete(obj)
            if ~obj.dev.isNull()
                calllib('libusb', 'libusb_close', obj.dev);
                fprintf('Released RTC device\n');
            end
            calllib('libusb', 'libusb_exit', libpointer);
        end
        
        function send_raw(obj, data)
            % function send_raw(data)
            %
            % Send raw data to the RTC device
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
            % function send_cmd(cmd, data)
            %
            % Send a command to the RTC device
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
            % function recv_raw(nbytes)
            %
            % Read raw data from the RTC device
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
            % function recv_cmd()
            %
            % Read the return value from a command from the RTC device
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
            % function get_par_list()
            %
            % Get the parameters available to access on the RTC device
            
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
        
        function set_par(obj, names, values)
            % function set_par(names, values)
            %
            % Set the values of particular parameters on the RTC device

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
            % function get_par(names)
            %
            % Get the values of particular parameters from the RTC device
            
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
            % function set_stream(stream, parameters, samples, downsample)
            %
            % Set stream parameters, sample count and downsample rate
            
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
        
        function [data, data_struct] = get_stream(obj, stream)
            % function get_stream(stream)
            %
            % Get stream data from a finished stream
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
            data = zeros(size(raw_data));
            for i = 1:par_count
                par_type = obj.par_info(par_ids(i) + 1).type;
                par_name = obj.par_info(par_ids(i) + 1).name;
                data_struct.(par_name) = typecast(raw_data(i, :), par_type);
                data(i, :) = cast(data_struct.(par_name), 'double');
            end
        end
        
        function result = start_stream(obj, stream)
            % function start_stream(stream)
            %
            % Start a stream recording data
            stream_name = sprintf('S%d', stream);
            obj.set_par([stream_name 'state'], obj.STREAM_STATE_ACTIVE);
            if obj.get_par([stream_name 'state']) ~= obj.STREAM_STATE_INACTIVE
                result = 1;
            else
                result = 0;
            end
        end
        
        function [data, data_struct] = run_stream(obj, stream, start, wait_period)
            % function run_stream(stream, start, wait_period)
            %
            % Start a stream recording data and return the resulting data
            stream_name = sprintf('S%d', stream);
            if ~exist('start', 'var') || start
                if ~obj.start_stream(stream)
                    error('Failed to start stream - perhaps bad parameters');
                end
            elseif obj.get_par([stream_name 'state']) == obj.STREAM_STATE_INACTIVE
                error('Stream not already started');
            end
            if ~exist('wait_period', 'var')
                wait_period = 0.1;
            end
            while obj.get_par([stream_name 'state']) == obj.STREAM_STATE_ACTIVE
                pause(wait_period);
            end
            [data, data_struct] = obj.get_stream(stream);
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
