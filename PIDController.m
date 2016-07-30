function [ms,Ts] = PIDController(control,ref,time,kp,ki,kd,port)


%% read serial from AURDINO
%  for Drawing the output response from RC cricuit
% and implement the PID Control

%% initial varaible
    clc; %close all  
    global S
    delete(instrfindall)  % delete  residual port assignment from previous sessions

    %time = 40;          % total time for control loop
    T = 0.1;            % step sample time
    %control = 1;        % control on (1)  or off (0) open loop
    %ref= 1;             % referance value (step responce 1)
    %kp = 2.1;             %control paramter values kp ki kd
    %ki = 1.38;
    %kd = .1;
    
    N =time/T;          % total number of sample
    ms = [];            % Array for saving ouput responce value
    Ts = [];            % Array for saving Time value at each sample
    Us = [];            % Array for saving control output sample
    
    
    eP1  = 0;            % previous error sample e(T-1)
    eP2  = 0 ;          % previous two sample error e(T-2)
    
    K0 = kp+ki*T + kd/T;
    K1 = (-kp - 2*kd/T);
    K2 = kd/T;
    U = 0;
    
 %%  open serial communcation between MATLAB and AURDINO
    %fclose(S);
    %port ='COM3';
    S = serial(port,'BaudRate',9600,'timeOut',0.1);
    fclose(S);
    fopen(S);
%% Open Figure and add two line ( output response , output of control)
    %figure
    set(gca,'NextPlot','add',...
            'XLabel',xlabel('Time'),...
            'YLabel',ylabel('VOLT'),...
            'GridLineStyle',':')

    hr = line(0,0,...
            'Color','blue',...
            'Tag','Response',...
            'Marker','.',...
            'LineStyle','--');
    hc = line(0,0,...
            'Color','red',...
            'Tag','Control',...
            'Marker','.',...
            'LineStyle','--');
    
    axis ([0 time 0 5])
    grid on
    
    pause(2)
%%  Send Refance value to AURDINO

        fprintf(S,'2')              % send command 2 before sending Ref value  ;
        pause(0.001)
        pwmRef = ceil(ref*255/5);   %convert it to range from  ( 0 - 255)
        fprintf(S,num2str(pwmRef))  %send value of Ref;
        pause(0.01)
        
%% start control loop by reading sample from output 
        for i = 1 : 1 : N
             % read sample from ouput by send command 1 to AURDINO
             fprintf(S,'1\n');
             pause(T)       
             y = str2num(fscanf(S))*5/255;      % convert it to 0 - 5 volt

            if ~isempty(y)
                    ms = [ms y];                %save the sample
                    Ts = [Ts (i-1)*T];          %save the time of sample
                                                % draw the response
                      set(hr,...
                         'XData',Ts,...
                         'YData',ms)
                     
                 % if controller is on
                if control == 1
                         
                         eT = ref- y;           % cal error
                         Du = K0 * eT + K1 * eP1 + K2 * eP2;
                         
                         %cal the control output U
                         U = U + Du;
                         
                         % check the value of control is in the range of 0
                         % to 5
                         if U > 5
                             Uo = 5;
                         elseif U < 0
                             Uo = 0;
                         else
                             Uo = U;
                         end
                         
                         % save the error for next interation
                         eP2 = eP1;
                         eP1 = eT;
                         
                         
                        
                        % convert control signal U to range 0 -255 and send
                        % it to AURDINO
                        Upwm = Uo*255/5;
                        fprintf(S,'2')
                        pause(0.001)
                        fprintf(S,num2str(Upwm))
                        
                        % draw the control signal U over time
                        Us = [Us  U];
                        set(hc,...
                             'XData',Ts,...
                             'YData',Us)
                end



            end
         %pause(T)
        end 

        fclose(S)

end

