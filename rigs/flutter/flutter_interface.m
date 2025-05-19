classdef flutter_interface < rtc_interface
    %FLUTTER_INTERFACE  Interface to the real-time control device that controls
    %Flutter experiment. 
    
    % Written by Irene Tartaruga (irene.tartaruga@bristol.ac.uk) 2018
    
    properties
        fourier;
        datafields;
    end
    
    methods
        function obj = flutter_interface()
            %FLUTTER_INTERFACE  Construct an EH_INTERFACE object.
            
            % Indices into the array of Fourier variables
            n_coeff = length(obj.par.x1_coeffs);
            obj.fourier.n_modes = n_coeff/2 - 1;
            obj.fourier.n_ave = obj.par_info(obj.par_idx('x1_coeffs_arr')).count/n_coeff; % Number of periods that are averaged to get the result
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

            obj.opt.x1_coeffs_var_tol_abs = 1e-2; % Maximum (absolute) variance of Fourier coefficients for steady-state behaviour
            obj.opt.x1_coeffs_var_tol_rel = 0.1; % Maximum (normalised) variance of Fourier coefficients for steady-state behaviour
            obj.opt.x1_coeffs_tol = 1e-2; % Maximum tolerance for difference between two Fourier coefficients
            
            
            obj.opt.frequency = 0.05; % tolerance for the frequency in the Picard iteration 
            
            obj.opt.x1_lim=10^5;%0.20; %m - maximum allowed heave motion - not adopted yet
            
            % Data recording fields
            obj.datafields.stream_id = 1; % The stream to use for data recording
            obj.datafields.static_fields = {'x_Kp', 'x_Kd', 'x1_control', ...
                                'input_filter_freq', 'rand_filter_freq', ...
                                'sample_freq', 'firmware','x1d', ...
								'phi','phi_bis','Fshaker','mean_h'};
            obj.datafields.dynamic_fields = {'forcing_freq', 'forcing_coeffs', ...
                                'rand_ampl', 'x1_coeffs_ave', 'x1_coeffs_var', ...
                                'x1_target_coeffs','out_coeffs_ave', 'out_coeffs_var',...
								'x1d', 'phi','phi_bis','Fshaker','mean_h'};
            obj.datafields.stream_fields = {'time_mod_2pi', 'x1', 'x1_target','x1d', 'phi','phi_bis','Fshaker','out'};
            
            % Default control gains (that work!)
            obj.par.x_Kp = -0.5;
            obj.par.x_Kd = 0;
        end
    end
    
end

