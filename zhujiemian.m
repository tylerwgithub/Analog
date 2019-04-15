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
ExitStr = questdlg('��ȷ���Ƿ��˳�ϵͳ��','�˳�ϵͳ','ȷ��','ȡ��','ȡ��');
if strcmp(ExitStr,'ȷ��')
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
set(handles.text2,'string','       ���ȵ������õ����ź�ȥ���Ƹ�Ƶ�ز��ķ��ȣ�ʹ֮������ź������Ա仯�Ĺ��̡���׼�������ǳ���˫�ߴ����ƣ���Ƶ������������ź�m(t)��һ��ֱ������A���Ӻ������ز���ˣ���ɲ��������źš�����ǵ��Ƶ�����̣��������Ǵӽ��յ��ѵ����ź��лָ���ԭ�����źţ�   ����ķ���һ�������֣�            (1)��ɽ����(2)����ɽ������������첨����ϵͳ���ð���첨����');
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
set(handles.text2,'string','�ڷ��ȵ��Ƶ�һ��ģ���У��������˲���Ϊȫͨ���磬�����ź�m(t)����ֱ����������������ѵ��źž������ز�������˫�ߴ������źţ���������ز�˫�ߴ������źţ����˫�ߴ��źš���AM�ź���ȣ���Ϊ�������ز��������������ĵ���Ч����100%��');
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
set(handles.text2,'string','����DSB�źŵ��ϱߴ����±ߴ�����ȫ�ԳƵģ���Я���˵����źŵ�ȫ����Ϣ����ˣ�����������һ���ߴ��ͼ��ɣ�����ݱ����һ���µĵ��Ʒ�ʽ�D�DSSB���ơ������Ƚ�ʡ�˷��͹��ʣ��ֿ��Խ�ʡһ�봫��Ƶ����');
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
set(handles.text2,'string','��νƵ�ʵ��ƣ���ָ˲ʱƵ��ƫ��������ź�m(t)�ɱ����仯������һ��ʹ�ܵ���˲ʱƵ��������źŶ���ĵ��Ʒ�����ʵ�����ֵ��Ʒ����ĵ�·�Ƶ�Ƶ��,�㷺���ڵ�Ƶ�㲥�����Ӱ�����΢��ͨ�š������·��ɨƵ�ǵȷ��档');
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
set(handles.text2,'string','��ν��λ���ƣ���ָ˲ʱ��λƫ��������ź�m(t)�����Ա仯������͵�Ƶ�����еĹ�ϵ������ʱ��ͬʱ�е�Ƶ���淢������Ƶʱ��Ҳͬʱ�е�����淢�����������ߵı仯���ɲ�ͬ��');
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
set(handles.text2,'string','Ƶ�ָ���(FDM��Frequency Division Multiplexing)���ǽ����ڴ����ŵ����ܴ����ֳ����ɸ����ŵ���ÿһ�����ŵ�����1·�źš�Ƶ�ָ���Ҫ����Ƶ�ʿ�ȴ��ڸ������ŵ�Ƶ��֮�ͣ�ͬʱΪ�˱�֤�����ŵ�����������źŻ������ţ�Ӧ�ڸ����ŵ�֮������������������ͱ�֤�˸�·�źŻ������š�');
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.radiobutton4,'value',0);
set(handles.radiobutton5,'value',0);
set(handles.radiobutton6,'value',1);
