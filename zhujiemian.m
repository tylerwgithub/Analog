function varargout = zhujiemian(varargin)
% ZHUJIEMIAN MATLAB code for zhujiemian.fig
%      ZHUJIEMIAN, by itself, creates a new ZHUJIEMIAN or raises the existing
%      singleton*.
%
%      H = ZHUJIEMIAN returns the handle to a new ZHUJIEMIAN or the handle to
%      the existing singleton*.
%
%      ZHUJIEMIAN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ZHUJIEMIAN.M with the given input arguments.
%
%      ZHUJIEMIAN('Property','Value',...) creates a new ZHUJIEMIAN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before zhujiemian_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to zhujiemian_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help zhujiemian

% Last Modified by GUIDE v2.5 17-Apr-2018 23:52:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @zhujiemian_OpeningFcn, ...
                   'gui_OutputFcn',  @zhujiemian_OutputFcn, ...
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


% --- Executes just before zhujiemian is made visible.
function zhujiemian_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to zhujiemian (see VARARGIN)

% Choose default command line output for zhujiemian
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes zhujiemian wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = zhujiemian_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
AMS(handles.figure1);
close zhujiemian;

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ExitStr = questdlg('请确认是否退出系统？','退出系统','确认','取消','取消');
if strcmp(ExitStr,'确认')
    close;
end


% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2


% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1
set(handles.text2,'string','       幅度调制是用调制信号去控制高频载波的幅度，使之随调制信号做线性变化的过程。标准调幅就是常规双边带调制，简称调幅。将基带信号m(t)与一个直流分量A叠加后再与载波相乘，便可产生调幅信号。解调是调制的逆过程，其作用是从接收的已调制信号中恢复出原基带信号，   解调的方法一般有两种：            (1)相干解调；(2)非相干解调，又名包络检波。本系统采用包络检波法。');
set(handles.radiobutton1,'value',1);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.radiobutton4,'value',0);
set(handles.radiobutton5,'value',0);
set(handles.radiobutton6,'value',0);

% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2
set(handles.text2,'string','在幅度调制的一般模型中，若假设滤波器为全通网络，调制信号m(t)中无直流分量，则输出的已调信号就是无载波分量的双边带调制信号，或称抑制载波双边带调制信号，简称双边带信号。与AM信号相比，因为不存在载波分量，所以它的调制效率是100%。');
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',1);
set(handles.radiobutton3,'value',0);
set(handles.radiobutton4,'value',0);
set(handles.radiobutton5,'value',0);
set(handles.radiobutton6,'value',0);

% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3
set(handles.text2,'string','由于DSB信号的上边带和下边带是完全对称的，皆携带了调制信号的全部信息，因此，仅传输其中一个边带就即可，这就演变出另一种新的调制方式――SSB调制。这样既节省了发送功率，又可以节省一半传输频带。');
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',1);
set(handles.radiobutton4,'value',0);
set(handles.radiobutton5,'value',0);
set(handles.radiobutton6,'value',0);

% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4
set(handles.text2,'string','所谓频率调制，是指瞬时频率偏移随调制信号m(t)成比例变化。它是一种使受调波瞬时频率随调制信号而变的调制方法。实现这种调制方法的电路称调频器,广泛用于调频广播、电视伴音、微波通信、锁相电路和扫频仪等方面。');
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.radiobutton4,'value',1);
set(handles.radiobutton5,'value',0);
set(handles.radiobutton6,'value',0);

% --- Executes on button press in radiobutton5.
function radiobutton5_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton5
set(handles.text2,'string','所谓相位调制，是指瞬时相位偏移随调制信号m(t)作线性变化。调相和调频有密切的关系。调相时，同时有调频伴随发生；调频时，也同时有调相伴随发生，不过两者的变化规律不同。');
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.radiobutton4,'value',0);
set(handles.radiobutton5,'value',1);
set(handles.radiobutton6,'value',0);

% --- Executes on button press in radiobutton6.
function radiobutton6_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton6
set(handles.text2,'string','频分复用(FDM，Frequency Division Multiplexing)就是将用于传输信道的总带宽划分成若干个子信道，每一个子信道传输1路信号。频分复用要求总频率宽度大于各个子信道频率之和，同时为了保证各子信道中所传输的信号互不干扰，应在各子信道之间设立隔离带，这样就保证了各路信号互不干扰。');
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.radiobutton4,'value',0);
set(handles.radiobutton5,'value',0);
set(handles.radiobutton6,'value',1);
