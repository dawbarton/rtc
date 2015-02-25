classdef ntmd_interface < rtc_interface
    %NTMD_INTERFACE  Interface to the real-time control device that controls
    %  the nonlinear tuned mass damper experiment.
    
    % Written by David A.W. Barton (david.barton@bristol.ac.uk) 2015
    
    properties
        fourier;
        opt;
        datafields;
    end
    
    methods
        function obj = ntmd_interface()
            %NTMD_INTERFACE  Construct an NTMD_INTERFACE object.
            
            % Indices into the array of Fourier variables
            n_coeff = length(rtc.par.x_coeffs);
            obj.fourier.n_modes = n_coeff/2 - 1;
            obj.fourier.n_ave = obj.par_info(rtc.par_idx('x_coeffs_arr')).count/n_coeff; % Number of periods that are averaged to get the result
            obj.fourier.idx_DC = n_coeff/2 + 1;
            obj.fourier.idx_AC = setdiff(2:n_coeff, obj.fourier.idx_DC);
            obj.fourier.idx_fund = [2, obj.fourier.idx_DC + 1];
            obj.fourier.idx_higher = [obj.fourier.idx_fund(1) + (1:obj.fourier.n_modes - 1), ...
                                      obj.fourier.idx_fund(2) + (1:obj.fourier.n_modes - 1)];
            obj.fourier.idx_sin = 1 + (1:obj.fourier.n_modes);
            obj.fourier.idx_cos = n_coeff/2 + 1 + (1:obj.fourier.n_modes);
            
            % Default options for the experiment
            obj.opt.wait_time = 1; % Time (in secs) to wait for Fourier coefficients to settle
            obj.opt.max_waits = 10; % Maximum number of times to wait
            obj.opt.max_picard_iter = 5; % Maximum number of Picard iterations to do
            obj.opt.x_coeffs_var_tol = 1e-3; % Maximum (normalised) variance of Fourier coefficients for steady-state behaviour
            obj.opt.x_coeffs_tol = 1e-1; % Maximum tolerance for difference between two Fourier coefficients (mm)
            
            % Data recording fields
            obj.datafields.stream_id = 1; % The stream to use for data recording
            obj.datafields.static_fields = {'x_Kp', 'x_Kd', 'x_control', 'base_Kp', 'base_Kd', 'base_control', 'input_filter_freq', 'rand_filter_freq', 'sample_freq'};
            obj.datafields.dynamic_fields = {'forcing_freq', 'forcing_coeffs', 'rand_ampl', 'x_coeffs_ave', 'x_coeffs_var', 'x_target_coeffs', 'out_coeffs_ave'};
            obj.datafields.stream_fields = {'time_mod_2pi', 'x', 'x_target', 'base', 'mass', 'force', 'out'};
            
            % Default control gains (that work!)
            obj.par.base_Kp = -0.04;
            obj.par.base_Kd = -0.004;
            obj.par.base_control = 1;
            obj.par.x_Kp = 0.005;
            obj.par.x_Kd = 0.0008;
            
            % Set the filter to 50 Hz
            obj.par.input_filter_freq = 50/obj.par.sample_freq;
        end
    end
    
end

