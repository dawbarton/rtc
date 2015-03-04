classdef eh_interface < rtc_interface
    %EH_INTERFACE  Interface to the real-time control device that controls
    %numerical Duffing oscillator experiment. 
    
    % Written by David A.W. Barton (david.barton@bristol.ac.uk) 2015
    
    properties
        fourier;
        datafields;
    end
    
    methods
        function obj = eh_interface()
            %EH_INTERFACE  Construct an EH_INTERFACE object.
            
            % Indices into the array of Fourier variables
            n_coeff = length(obj.par.x_coeffs);
            obj.fourier.n_modes = n_coeff/2 - 1;
            obj.fourier.n_ave = obj.par_info(obj.par_idx('x_coeffs_arr')).count/n_coeff; % Number of periods that are averaged to get the result
            obj.fourier.idx_DC = n_coeff/2 + 1;
            obj.fourier.idx_AC = setdiff(2:n_coeff, obj.fourier.idx_DC);
            obj.fourier.idx_fund = [2, obj.fourier.idx_DC + 1];
            obj.fourier.idx_higher = [obj.fourier.idx_fund(1) + (1:obj.fourier.n_modes - 1), ...
                                      obj.fourier.idx_fund(2) + (1:obj.fourier.n_modes - 1)];
            obj.fourier.idx_sin = 1 + (1:obj.fourier.n_modes);
            obj.fourier.idx_cos = n_coeff/2 + 1 + (1:obj.fourier.n_modes);
            
            % Default options for the experiment
            obj.opt.samples = 2000; % Number of samples to record
            obj.opt.downsample = 0; % Number of samples to ignore for every sample recorded
            obj.opt.wait_time = 0.1; % Time (in secs) to wait for Fourier coefficients to settle
            obj.opt.max_waits = 10; % Maximum number of times to wait
            obj.opt.max_picard_iter = 5; % Maximum number of Picard iterations to do
            obj.opt.x_coeffs_var_tol_rel = 1e-5; % Maximum (normalised) variance of Fourier coefficients for steady-state behaviour
            obj.opt.x_coeffs_var_tol_abs = 1e-5; % Maximum (absolute) variance of Fourier coefficients for steady-state behaviour
            obj.opt.x_coeffs_tol = 1e-3; % Maximum tolerance for difference between two Fourier coefficients
            
            % Data recording fields
            obj.datafields.stream_id = 1; % The stream to use for data recording
            obj.datafields.static_fields = {'x_Kp', 'x_Kd', 'x_control', ...
                                'input_filter_freq', 'rand_filter_freq', ...
                                'sample_freq', 'firmware'};
            obj.datafields.dynamic_fields = {'forcing_freq', 'forcing_coeffs', ...
                                'rand_ampl', 'x_coeffs_ave', 'x_coeffs_var', ...
                                'x_target_coeffs', 'out_coeffs_ave', 'out_coeffs_var'};
            obj.datafields.stream_fields = {'time_mod_2pi', 'x', 'x_target', 'out', 'coil', 'x_accn', 'base_accn'};

            % Default control gains (that work!)
            obj.par.input_filter_freq = 0.01; % Cut-off at 50Hz
            obj.par.x_Kp =  2.000;
            obj.par.x_Kd = -0.005;
            
            % Shift the zero point of the shaker somewhere sensible
            obj.par.forcing_coeffs(obj.fourier.idx_DC) = -1.5;
        end
    end
    
end

