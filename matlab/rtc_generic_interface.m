classdef (Abstract) rtc_generic_interface < handle
    %RTC_GENERIC_INTERFACE Interface to a real-time control device. It
    %  exposes basic functionality to the user (parameter setting/getting
    %  and stream capture functions.) This class should be inherited to
    %  implement actual functionality.

    properties (Abstract)
        par;
    end
    
    methods (Abstract)
        set_par(obj, names, values);
        values = get_par(obj, names);
        set_stream(obj, stream, parameters, samples, downsample);
        [data, data_struct] = get_stream(obj, stream);
        result = start_stream(obj, stream);
        [data, data_struct] = run_stream(obj, stream, start, wait_period);
    end
end
