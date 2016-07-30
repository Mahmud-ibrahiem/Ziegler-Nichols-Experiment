function varargout = PIDGUI(varargin)
% PIDGUI MATLAB code for PIDGUI.fig
%      PIDGUI, by itself, creates a new PIDGUI or raises the existing
%      singleton*.
%
%      H = PIDGUI returns the handle to a new PIDGUI or the handle to
%      the existing singleton*.
%
%      PIDGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PIDGUI.M with the given input arguments.
%
%      PIDGUI('Property','Value',...) creates a new PIDGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PIDGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PIDGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PIDGUI

% Last Modified by GUIDE v2.5 13-May-2016 21:51:18

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PIDGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @PIDGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before PIDGUI is made visible.
function PIDGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PIDGUI (see VARARGIN)
set(handles.kpSlider,'Visible','on'); %make the slider visible
% Choose default command line output for PIDGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PIDGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PIDGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function kpSlider_Callback(hObject, eventdata, handles)
% hObject    handle to kpSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.sliderKp,'String',get(handles.kpSlider,'Value')) % copy the slider value to the label
 
 


function kpSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kpSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
MinValue=0; % the initial min value of the slider 
MaxValue=50; % The initial max value of the slider
NumberOfSteps=1000; % the number of presses for the scroll to go from the slider start to End
avg=(MaxValue-MinValue)/2; % slider Avg value
set(hObject,'Min',MinValue) % copy the value from variable Min value to slider
set(hObject,'Value',avg) % set the start initial position value of the slider to be equal to average.
set(hObject,'Max',MaxValue) % set the max value of the slider to be equal to the 'Max Value' variable
set(hObject,'SliderStep',[.5/NumberOfSteps,1/NumberOfSteps]) % set the step length for the slider
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



% --- Executes on selection change in popupmenu.
function popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp(get(handles.popupmenu,'Value'))
% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu


% --- Executes during object creation, after setting all properties.
function popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in startZigglerNichole.
function startZigglerNichole_Callback(hObject, eventdata, handles)
% hObject    handle to startZigglerNichole (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
   cla reset % deletes graphics objects from the current axes regardless of their handle visibility. It also resets axes properties to their default values, with the exception of the Position and Units properties.
   set(handles.text5,'String','Charging open Loop') % Show the status on its label
   disp(get(handles.MaxTime,'String'))
   [ms,Ts]=PIDController(0,1,str2num(get(handles.MaxTime,'String')),2.1,1.38,.1,getport(handles));   delayindex=find(abs(ms-.01)==min(abs(ms-.01)),1); % call the PID Control script
   ind63=find(abs(ms-.63*max(ms))==min(abs(ms-.63*max(ms))),1); % gets the index of the time const value
   ind28=find(abs(ms-.28*max(ms))==min(abs(ms-.28*max(ms))),1); % gets the index of the value equal to .28 of steady state value.
   timeConst=1.5*(Ts(ind63)-Ts(ind28)); % calculates the time const
   delaytime=timeConst-Ts(delayindex);  % calculates the delay time
   set(handles.TimeConst,'String',timeConst) % shows the value of the time const on its label
   set(handles.DelayTime,'String',delaytime) % shows the value of the delay time on its label
   kp=1.2*timeConst/delaytime; % calculat the value of proportional controller using Ziegler–Nichols 1st method 
   ki=kp/(2*delaytime); % calculat the value of integral controller using Ziegler–Nichols 1st method 
   kd=kp/(.5*delaytime); % calculat the value of differential controller using Ziegler–Nichols 1st method 
   set(handles.kp,'String',kp) % display the value of the proportional controller on the label.
   set(handles.ki,'String',ki) % display the value of the integral controller on the label.
   set(handles.kd,'String',kd) % display the value of the differential controller on the label.
   set(handles.sliderKp,'String',kp)% set the value of the
   set(handles.kpSlider,'Value',kp) % set the value of the kp slider
   set(handles.sliderKi,'String',ki) % set the value of the ki slider label
   set(handles.KiSkider,'Value',ki) % set the value of the ki slider
   set(handles.sliderKd,'String',kd) % set the value of the kd slider label
   set(handles.kdSlider,'Value',kd)  % set the value of the kd slider
   set(handles.text5,'String','Discharging') % show the discharging status
   [ms,Ts]=PIDController(0,0,str2num(get(handles.dischargeTimeText,'String')),kp,ki,kd,getport(handles)); % start discharging process
   set(handles.text5,'String','Ziggler Nichole Response') % show the 'Ziggler Nichole Response' status
   cla reset
   [ms,Ts]=PIDController(1,1,str2num(get(handles.MaxTime,'String')),kp,ki,kd,getport(handles)); % start the control process

function [port]=getport(handles)

 switch get(handles.popupmenu,'Value')
      case 1
          port='COM1';
      case 2
          port='COM2';
      case 3
          port='COM3';
      case 4
          port='COM4';
      case 5
          port='COM5';
      case 6
          port='COM6';
      case 7
          port='COM7';
      case 8
          port='COM8';
 end

% --- Executes on button press in Discharge.
function Discharge_Callback(hObject, eventdata, handles)
% hObject    handle to Discharge (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  delete(instrfindall) % to clear any previous serial connection
  cla reset % reset the drawing
  set(handles.text5,'String','Discharging') % show the discharging status
  [ms,Ts]=PIDController(0,0,str2num(get(handles.dischargeTimeText,'String')),2.1,1.38,.1,getport(handles));
   set(handles.text5,'String','Status') % return the status label to show the word status



function MaxTime_Callback(hObject, eventdata, handles)
% hObject    handle to MaxTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of MaxTime as text
%        str2double(get(hObject,'String')) returns contents of MaxTime as a double


% --- Executes during object creation, after setting all properties.
function MaxTime_CreateFcn(hObject, eventdata, handles) % the value of the response time
% hObject    handle to MaxTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles) 
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear  % clear all variables
 delete(instrfindall) % delete any open serial connection 


% --- Executes on slider movement.
function KiSkider_Callback(hObject, eventdata, handles)
% hObject    handle to KiSkider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.sliderKi,'String',get(handles.KiSkider,'Value')) % execute the value of the slider on the label

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function KiSkider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to KiSkider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
MinValue=0; % the initial min value of the slider 
MaxValue=50; % The initial max value of the slider
NumberOfSteps=10000;  % the number of presses for the scroll to go from the slider start to End
avg=(MaxValue-MinValue)/2; % slider Avg value
set(hObject,'Min',MinValue) % copy the value from variable Min value to slider
set(hObject,'Value',avg) % set the start initial position value of the slider to be equal to average.
set(hObject,'Max',MaxValue) % set the max value of the slider to be equal to the 'Max Value' variable
set(hObject,'SliderStep',[.5/NumberOfSteps,1/NumberOfSteps])  % set the step length for the slider
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function kdSlider_Callback(hObject, eventdata, handles)
% hObject    handle to kdSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.sliderKd,'String',get(handles.kdSlider,'Value')) % show the values of the slider on the label

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function kdSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kdSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
MinValue=0; % the initial min value of the slider 
MaxValue=50; % The initial max value of the slider
NumberOfSteps=10000; % the number of presses for the scroll to go from the slider start to End
avg=(MaxValue-MinValue)/2;  % slider Avg value
set(hObject,'Min',MinValue) % copy the value from variable Min value to slider
set(hObject,'Value',avg) % set the start initial position value of the slider to be equal to average.
set(hObject,'Max',MaxValue) % set the max value of the slider to be equal to the 'Max Value' variable
set(hObject,'SliderStep',[.5/NumberOfSteps,1/NumberOfSteps]) % set the step length for the slider
% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in Tune.
function Tune_Callback(hObject, eventdata, handles)
% hObject    handle to Tune (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla reset % reset the drawing
[ms,Ts]=PIDController(1,1,str2num(get(handles.MaxTime,'String')),get(handles.kpSlider,'Value'),get(handles.KiSkider,'Value'),get(handles.kdSlider,'Value'),getport(handles)); % Apply the tuned values


% --- Executes on button press in Clear.
function Clear_Callback(hObject, eventdata, handles)
% hObject    handle to Clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla reset % clears the drawing



function dischargeTimeText_Callback(hObject, eventdata, handles)
% hObject    handle to dischargeTimeText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dischargeTimeText as text
%        str2double(get(hObject,'String')) returns contents of dischargeTimeText as a double


% --- Executes during object creation, after setting all properties.
function dischargeTimeText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dischargeTimeText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in GeneticButton.
function GeneticButton_Callback(hObject, eventdata, handles)
% hObject    handle to GeneticButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  
  % Executes a function Equivalent to the One done by the Ziegler Nicholes
  % button on line 128
   cla reset
   set(handles.text5,'String','Charging open Loop Father')
   disp(get(handles.MaxTime,'String'))
   [ms,Ts]=PIDController(0,1,str2num(get(handles.MaxTime,'String')),2.1,1.38,.1,getport(handles));   delayindex=find(abs(ms-.01)==min(abs(ms-.01)),1);
   ind63=find(abs(ms-.63*max(ms))==min(abs(ms-.63*max(ms))),1);
   ind28=find(abs(ms-.28*max(ms))==min(abs(ms-.28*max(ms))),1);
   timeConst=1.5*(Ts(ind63)-Ts(ind28));
   delaytime=timeConst-Ts(delayindex);
   set(handles.TimeConst,'String',timeConst)
   set(handles.DelayTime,'String',delaytime)
   kp(1)=1.2*timeConst/delaytime;
   ki(1)=kp/(2*delaytime);
   kd(1)=kp/(.5*delaytime);
   set(handles.kp,'String',kp(1))
   set(handles.ki,'String',ki(1))
   set(handles.kd,'String',kd(1))
   set(handles.sliderKp,'String',kp(1))
   set(handles.kpSlider,'Value',kp(1))
   set(handles.sliderKi,'String',ki(1))
   set(handles.KiSkider,'Value',ki(1))
   set(handles.sliderKd,'String',kd(1))
   set(handles.kdSlider,'Value',kd(1))
   set(handles.text5,'String','Discharging')
   [ms,Ts]=PIDController(0,0,str2num(get(handles.dischargeTimeText,'String')),kp(1),ki(1),kd(1),getport(handles));
   set(handles.text5,'String','Ziggler Nichole Response Father')
   cla reset
   [ms,Ts]=PIDController(1,1,str2num(get(handles.MaxTime,'String')),kp(1),ki(1),kd(1),getport(handles));
   
   %2nd iteration ( Mother iteration)
     % Executes a function Equivalent to the One done by the Ziegler Nicholes
     % button on line 128
   cla reset
   set(handles.text5,'String','Discharging')
   [ms,Ts]=PIDController(0,0,str2num(get(handles.dischargeTimeText,'String')),kp,ki,kd,getport(handles));
   set(handles.text5,'String','Charging open Loop Mother')
   disp(get(handles.MaxTime,'String'))
  [ms,Ts]=PIDController(0,1,str2num(get(handles.MaxTime,'String')),2.1,1.38,.1,getport(handles));   delayindex=find(abs(ms-.01)==min(abs(ms-.01)),1);
   ind63=find(abs(ms-.63*max(ms))==min(abs(ms-.63*max(ms))),1);
   ind28=find(abs(ms-.28*max(ms))==min(abs(ms-.28*max(ms))),1);
   timeConst=1.5*(Ts(ind63)-Ts(ind28));
   delaytime=timeConst-Ts(delayindex);
   set(handles.TimeConst,'String',timeConst)
   set(handles.DelayTime,'String',delaytime)
   kp(2)=1.2*timeConst/delaytime;
   ki(2)=kp(2)/(2*delaytime);
   kd(2)=kp(2)/(.5*delaytime);
   set(handles.kp,'String',kp(2))
   set(handles.ki,'String',ki(2))
   set(handles.kd,'String',kd(2))
   set(handles.sliderKp,'String',kp(2))
   set(handles.kpSlider,'Value',kp(2))
   set(handles.sliderKi,'String',ki(2))
   set(handles.KiSkider,'Value',ki(2))
   set(handles.sliderKd,'String',kd(2))
   set(handles.kdSlider,'Value',kd(2))
    set(handles.text5,'String','Discharging')
   [ms,Ts]=PIDController(0,0,str2num(get(handles.dischargeTimeText,'String')),kp(2),ki(2),kd(2),getport(handles));
   set(handles.text5,'String','Ziggler Nichole Response Mother')
   cla reset
   [ms,Ts]=PIDController(1,1,str2num(get(handles.MaxTime,'String')),kp(2),ki(2),kd(2),getport(handles));
  
% in case we want to test the Genetic search implemetation alone uncomment
% the next lines and comment all the lines up to the beginning of this call
% back function

%  timeConst=8.5;
%  delaytime=8.2;
%  kp(1)=1.2436;
%  ki(1)=.0700;
%  kd(1)=.301;
%  kp(2)=1.200;
%  ki(2)=.0700;
%  kd(2)=.2800;

 % Genetic Algorithm  
 
 % the values of the controller converted to binary after it was multiplied
 % by 10000 to save the 1st four decimals
 binaryKp_1=de2bi(round(kp(1)*10000));
 binaryKi_1=de2bi(round(ki(1)*10000));
 binaryKd_1=de2bi(round(kd(1)*10000));
 binaryKp_2=de2bi(round(kp(2)*10000));
 binaryKi_2=de2bi(round(ki(2)*10000));
 binaryKd_2=de2bi(round(kd(2)*10000));

 % make sure that all the binary numbers have the same length
binaryKp_1=[binaryKp_1,zeros(1,16-length(binaryKp_1))];
binaryKi_1=[binaryKi_1,zeros(1,16-length(binaryKi_1))];
binaryKd_1=[binaryKd_1,zeros(1,16-length(binaryKd_1))];
binaryKp_2=[binaryKp_2,zeros(1,16-length(binaryKp_2))];
binaryKi_2=[binaryKi_2,zeros(1,16-length(binaryKi_2))];
binaryKd_2=[binaryKd_2,zeros(1,16-length(binaryKd_2))];

% combine the solutions of the controllers to get a combined solution
 totalSolution_1=[binaryKp_1,binaryKi_1,binaryKd_1];
 totalSolution_2=[binaryKp_2,binaryKi_2,binaryKd_2];
 totalSolution=[totalSolution_1;totalSolution_2];

% Cross Over Step
totsize=size(totalSolution); % get the size of the combined solution

 for i=1:(get(handles.RepeatSlider,'Value')-2)
     
     for j=1:totsize(2)
     newsolution(j)= totalSolution(1+round(rand*(totsize(1)-1)),j); % this step gets a random value of each binary digit in the new solution from the previous ones
     end
     totalSolution=[totalSolution;newsolution]; % add the newly generated solution to the matrix of solutions
 end
totsize=size(totalSolution); % get the dimension of the total solution
 % mutation chooses random 8 digits and changes them
  for i=1:totsize(1)
     for j=1:8 % only 8 mutations
     totalSolution(i,round(1+(totsize(2)-1)*rand))=not( totalSolution(i,round(1+(totsize(2)-1)*rand)));
     end
  end
 
 % recover the values transfere from binary to decimal
   for i=1:totsize(1)
    kp(i)=0;
    kd(i)=0;
    ki(i)=0;
       for j=1:totsize(2)/3
    kp(i)=kp(i)+totalSolution(i,j)*2^(j-1);
    kd(i)=kd(i)+totalSolution(i,totsize(2)/3+j)*2^(j-1);
    ki(i)=ki(i)+totalSolution(i,2*totsize(2)/3+j)*2^(j-1);
       end
   end
  % retrieve the decimal values
 kp=kp/10000;
 kd=kd/10000;
 ki=ki/10000;

 % choosing min steady state value 
 num = 1; % the numenator of the open loop system transfere function
 den = [timeConst 1]; % the denumenator of the open loop system transfere function
 opensys= tf(num,den); % form the transfere function
 delaysys=tf([delaytime/-2,1],[delaytime/2,1]); % an approximation of the delay transfere function  where exp(x) = (1+.5x)/(1-.5x)
 opensys= series(opensys,delaysys); % create the "FOPDT" first order plus delay time system
 for i= 1 :totsize(1) % form the transfere function off all possible systems from all possible kp,kd,ki
     % that were acheived from the cross over process
 controller=pid(kp(i),ki(i),kd(i),1);
 [num,den] =tfdata(controller);
 num=cell2mat(num);
 den=cell2mat(den);
 forward=series(opensys,tf(num,den));
 closedsystem=feedback(forward,1);
  info=stepinfo(closedsystem);
  settle(i)=info.SettlingTime;
 end
     indexOfSettleMin=find(settle==min(settle),1); % get the index of the minimum settling time
     set(handles.KpSettle,'String',kp(indexOfSettleMin)) % show the kp corresponding to the minimum settling time
     set(handles.KiSettle,'String',ki(indexOfSettleMin)) % show the ki corresponding to the minimum settling time
     set(handles.KdSettle,'String',kd(indexOfSettleMin)) % show the kd corresponding to the minimum settling time
     set(handles.settletime,'String',settle(indexOfSettleMin)) % show the settling time










% --- Executes on slider movement.
function RepeatSlider_Callback(hObject, eventdata, handles)
% hObject    handle to RepeatSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(hObject,'Value',round(get(handles.RepeatSlider,'Value')));
set(handles.RepeatNo,'String',get(handles.RepeatSlider,'Value'))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function RepeatSlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RepeatSlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
MinValue=500;
MaxValue=5000;
NumberOfSteps=2000;
set(hObject,'Min',MinValue);
set(hObject,'Value',MinValue);
set(hObject,'Max',MaxValue);
set(hObject,'SliderStep',[1/NumberOfSteps,2/NumberOfSteps]);

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
