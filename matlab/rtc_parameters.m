classdef rtc_parameters < dynamicprops
    %rtc_interface: Interface to the real-time control device. As far as
    %  possible this object is designed to be state-less. (The exception
    %  being the parameter list.)

    properties (Hidden)
        rtc_;
        str_;
        names_;
    end
    
    methods
        function disp(obj)
            fprintf(obj.str_);
        end
    end

    methods
        function obj = rtc_parameters(rtc)
            % function obj = rtc_parameters(rtc)
            % 
            % Construct a helper object to simplify access to parameters
            
            obj.rtc_ = rtc;
            obj.str_ = 'Accessible RTC parameters:\n';
            obj.names_ = cell(1, length(rtc.par_info));
            for i = 1:length(rtc.par_info)
                obj.names_{i} = rtc.par_info(i).name;
                obj.str_ = [obj.str_, '\t', rtc.par_info(i).name, '\n']; 
            end
            obj.add_properties();
        end
        
        function add_properties(obj)
            for i = 1:length(obj.names_)
                name = obj.names_{i};
                prop = obj.addprop(name);
                prop.GetMethod = @(obj)obj.rtc_.get_par(name);
                prop.SetMethod = @(obj, value)obj.rtc_.set_par(name, value);
                prop.Dependent = true;
                prop.SetObservable = true;
                prop.Transient = false;
            end
        end
    end        
    
    methods (Static = true)
        function obj = loadobj(obj)
            % Add the dynamic properties that will have been lost through
            % saving/loading in Matlab
            obj.add_properties();
        end
    end
    
end
