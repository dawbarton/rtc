classdef rtc_parameters < dynamicprops
    %RTC_PARAMETERS A helper class to easily access device parameters.

    properties (Hidden)
        rtc_;
        str_;
    end
    
    methods
        function disp(obj)
            fprintf(obj.str_);
        end
    end

    methods
        function obj = rtc_parameters(rtc)
            obj.rtc_ = rtc;
            obj.str_ = 'Accessible RTC parameters:\n';
            for i = 1:length(rtc.par_info)
                obj.add_property(rtc.par_info(i).name);
            end
        end
        
        function add_property(obj, names)
            if ~iscell(names)
                names = {names};
            end
            for i = 1:length(names)
                name = names{i};
                obj.str_ = [obj.str_, '\t', name, '\n']; 
                prop = obj.addprop(name);
                prop.GetMethod = @(obj)obj.rtc_.get_par(name);
                prop.SetMethod = @(obj, value)obj.rtc_.set_par(name, value);
                prop.Dependent = true;
                prop.SetObservable = true;
                prop.Transient = false;
            end
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
