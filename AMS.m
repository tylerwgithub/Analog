function varargout = AMS(varargin)
% AMS MATLAB code for AMS.fig
%      AMS, by itself, creates a new AMS or raises the existing
%      singleton*.
%
%      H = AMS returns the handle to a new AMS or the handle to
%      the existing singleton*.
%
%      AMS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AMS.M with the given input arguments.
%
%      AMS('Property','Value',...) creates a new AMS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AMS_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AMS_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AMS

% Last Modified by GUIDE v2.5 20-Apr-2018 01:11:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AMS_OpeningFcn, ...
                   'gui_OutputFcn',  @AMS_OutputFcn, ...
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


% --- Executes just before AMS is made visible.
function AMS_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AMS (see VARARGIN)

% Choose default command line output for AMS
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AMS wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = AMS_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
zs=get(handles.edit4,'string');
zs=str2num(zs);
spe=get(handles.radiobutton1,'value');
if spe==1
    spe1=1;
    spe=0;
else spe=get(handles.radiobutton2,'value');
    if spe==1
    spe1=2;
    spe=0;
    else
        spe1=3;
        spe=0;
    end
end
global sw;
if sw==1%AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
    %AM
t=0.01:0.001:1;
aa=get(handles.edit1,'string');
bb=get(handles.edit2,'string');
zz=get(handles.edit3,'string');
aa=str2num(aa);
bb=str2num(bb);
zz=str2num(zz);
if spe1==1
a=cos(aa*t);
else if spe1==2
        a=sawtooth(aa*t);
    else
        a=square(aa*t);
    end
end
% a=awgn(a,zs);
b=cos(bb*t);
c=(zz+a).*b;
c=awgn(c,zs);


e=c.*b;
Ft=2000;    %����Ƶ��
fpts=[0 aa*2]; %ͨ���߽�Ƶ��fp=100Hz�������ֹƵ��fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %ͨ������1%���������5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord���Ʋ��ÿ�������Ƶ�FIR�� �����Ĳ���
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %��fir1����˲���
e=fftfilt(b21,e);                %FIR��ͨ�˲�

% e=abs(hilbert(c));%hilbert�任�������ֵ�õ�˲ʱ���ȣ�����첨��
% zero=(max(c1)-min(c1))/2;
% c2=c1-zero;

axes(handles.axes1);
plot(t,a);
axes(handles.axes3);
plot(b);
axes(handles.axes5);
plot(c);
axes(handles.axes7);
plot(t,e,'r');

ts=0.001;                                    %�������
fs=1/ts;                                      %����Ƶ��
df=0.25;                                     %�����Ƶ�ʷֱ��ʣ���������Ҷ�任
%ʱ������ʾFFT����СƵ�ʼ��
%*****�Ե����ź�m(t)����Ҷ�任*****
                         %ԭ�����ź�
if spe1==1
a=cos(aa*5*t);
% a=awgn(a,zs);
else if spe1==2
        a=sawtooth(aa*5*t);
    else
        a=square(aa*5*t);
    end
end
n1=0;
n2=length(a);
n=2^(max(nextpow2(0),nextpow2(n2)));
A=fft(a,n);
a=[a,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(a)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes2);
plot(f,abs(fftshift(A)));

% axes(handles.axes8);
% plot(f,abs(fftshift(A)));
%*****���ز��ź�m(t)����Ҷ�任*****
b=cos(bb*5*t);                         %ԭ�����ź�
n1=0;
n2=length(b);
n=2^(max(nextpow2(0),nextpow2(n2)));
B=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(b)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes4);
plot(f,abs(fftshift(B)));

%*****���ѵ��ź�m(t)����Ҷ�任*****
c=(zz+a).*b;                        %ԭ�����ź�
c=awgn(c,zs);
n1=0;
n2=length(c);
n=2^(max(nextpow2(0),nextpow2(n2)));
C=fft(c,n);
c=[c,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(c)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes6);
plot(f,abs(fftshift(C)));

%*****���ѵ��ź�m(t)����Ҷ�任*****
c=(zz+a).*b;                        %ԭ�����ź�
c=awgn(c,zs);
d= abs(hilbert(c));
n1=0;
n2=length(d);
n=2^(max(nextpow2(0),nextpow2(n2)));
D=fft(d,n);
d=[d,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(d)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes8);
plot(f,abs(fftshift(D)),'r');

    va=0;

else
    if sw==2%DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        %DSB
        t=0.01:0.001:1;
aa=get(handles.edit1,'string');
bb=get(handles.edit2,'string');
zz=get(handles.edit3,'string');
aa=str2num(aa);
bb=str2num(bb);
zz=str2num(zz);
if spe1==1
a=cos(aa*t);
else if spe1==2
        a=sawtooth(aa*t);
    else
        a=square(aa*t);
    end
end
% a=awgn(a,zs);
b=cos(bb*t);
c=a.*b;
c=awgn(c,zs);
e=c.*b;
Ft=2000;    %����Ƶ��
fpts=[0 aa*2]; %ͨ���߽�Ƶ��fp=100Hz�������ֹƵ��fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %ͨ������1%���������5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord���Ʋ��ÿ�������Ƶ�FIR�� �����Ĳ���
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %��fir1����˲���
e=fftfilt(b21,e);                %FIR��ͨ�˲�



axes(handles.axes1);
plot(t,a);
axes(handles.axes3);
plot(b);
axes(handles.axes5);
plot(c);
axes(handles.axes7);
plot(t,e,'r');

ts=0.001;                                    %�������
fs=1/ts;                                      %����Ƶ��
df=0.25;                                     %�����Ƶ�ʷֱ��ʣ���������Ҷ�任
%ʱ������ʾFFT����СƵ�ʼ��
%*****�Ե����ź�m(t)����Ҷ�任*****
if spe1==1
a=cos(aa*5*t);
else if spe1==2
        a=sawtooth(aa*5*t);
    else
        a=square(aa*5*t);
    end
end
% a=awgn(a,zs);
                         %ԭ�����ź�
n1=0;
n2=length(a);
n=2^(max(nextpow2(0),nextpow2(n2)));
A=fft(a,n);
a=[a,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(a)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes2);
plot(f,abs(fftshift(A)));

%*****���ز��ź�m(t)����Ҷ�任*****
b=cos(bb*5*t);                         %ԭ�����ź�
n1=0;
n2=length(b);
n=2^(max(nextpow2(0),nextpow2(n2)));
B=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(b)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes4);
plot(f,abs(fftshift(B)));

%*****���ѵ��ź�m(t)����Ҷ�任*****
c=a.*b;     %ԭ�����ź�
c=awgn(c,zs);
n1=0;
n2=length(c);
n=2^(max(nextpow2(0),nextpow2(n2)));
C=fft(c,n);
c=[c,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(c)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes6);
plot(f,abs(fftshift(C)));

% %*****�Խ���ź�m(t)����Ҷ�任*****
% d=c.*b;
% e=c.*b;
% e = filter(y1,x1,d);                        %ԭ�����ź�

c=a.*b;
c=awgn(c,zs);
e=c.*b;
Ft=2000;    %����Ƶ��
fpts=[0 aa*10]; %ͨ���߽�Ƶ��fp=100Hz�������ֹƵ��fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %ͨ������1%���������5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord���Ʋ��ÿ�������Ƶ�FIR�� �����Ĳ���
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %��fir1����˲���
e=fftfilt(b21,e);                %FIR��ͨ�˲�

n1=0;
n2=length(e);
n=2^(max(nextpow2(0),nextpow2(n2)));
E=fft(e,n);
e=[e,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(e)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������


axes(handles.axes8);
plot(f,abs(fftshift(E)),'r');
        va=0;
    else
            if sw==3%SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                %SSB
                
t=0.01:0.001:1;
aa=get(handles.edit1,'string');
bb=get(handles.edit2,'string');
zz=get(handles.edit3,'string');
aa=str2num(aa);
bb=str2num(bb);
zz=str2num(zz);
if spe1==1
a=cos(aa*t);
else if spe1==2
        a=sawtooth(aa*t);
    else
        a=square(aa*t);
    end
end
% a=awgn(a,zs);
b=cos(bb*t);
c=a.*b;
c=awgn(c,zs);

Rp=0.1;
Rs=80;
Wp=49*2*pi*0.001;
Ws=73*2*pi*0.001;
[n,Wp] = ellipord(Wp,Ws,Rp,Rs); %���˲�������С����
[x,y] = ellip(n,Rp,Rs,Wp); %�����Բ�˲���
d = filter(x,y,c);
e=d.*b;

% 
Ft=2000;    %����Ƶ��
fpts=[0 aa*2]; %ͨ���߽�Ƶ��fp=100Hz�������ֹƵ��fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %ͨ������1%���������5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord���Ʋ��ÿ�������Ƶ�FIR�� �����Ĳ���
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %��fir1����˲���
e=fftfilt(b21,e);                %FIR��ͨ�˲�


axes(handles.axes1);
plot(t,a);
axes(handles.axes3);
plot(b);
axes(handles.axes5);
plot(c);
axes(handles.axes7);
plot(t,e,'r');

ts=0.001;                                    %�������
fs=1/ts;                                      %����Ƶ��
df=0.25;                                     %�����Ƶ�ʷֱ��ʣ���������Ҷ�任
%ʱ������ʾFFT����СƵ�ʼ��
%*****�Ե����ź�m(t)����Ҷ�任*****
if spe1==1
a=cos(aa*5*t);
else if spe1==2
        a=sawtooth(aa*5*t);
    else
        a=square(aa*5*t);
    end
end
% a=awgn(a,zs);
                         %ԭ�����ź�
n1=0;
n2=length(a);
n=2^(max(nextpow2(0),nextpow2(n2)));
A=fft(a,n);
a=[a,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(a)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes2);
plot(f,abs(fftshift(A)));

%*****���ز��ź�m(t)����Ҷ�任*****
b=cos(bb*5*t);                         %ԭ�����ź�
n1=0;
n2=length(b);
n=2^(max(nextpow2(0),nextpow2(n2)));
B=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(b)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes4);
plot(f,abs(fftshift(B)));

%*****���ѵ��ź�m(t)����Ҷ�任*****
c=a.*b;                        %ԭ�����ź�
c=awgn(c,zs);
Rp=0.1;
Rs=80;
Wp=49*2*pi*0.001;
Ws=73*2*pi*0.001;
[n,Wp] = ellipord(Wp,Ws,Rp,Rs); %���˲�������С����
[x,y] = ellip(n,Rp,Rs,Wp); %�����Բ�˲���
c = filter(x,y,c);
n1=0;
n2=length(c);
n=2^(max(nextpow2(0),nextpow2(n2)));
C=fft(c,n);
c=[c,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(c)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes6);
plot(f,abs(fftshift(C)));

%*****�Խ���ź�m(t)����Ҷ�任*****
c=a.*b;
c=awgn(c,zs);
d=c.*b;
e=c.*b;
Ft=2000;    %����Ƶ��
fpts=[0 aa*10]; %ͨ���߽�Ƶ��fp=100Hz�������ֹƵ��fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %ͨ������1%���������5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord���Ʋ��ÿ�������Ƶ�FIR�� �����Ĳ���
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %��fir1����˲���
e=fftfilt(b21,e);                %FIR��ͨ�˲�


n1=0;
n2=length(e);
n=2^(max(nextpow2(0),nextpow2(n2)));
E=fft(e,n);
e=[e,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����Ƕ��ź�����任
f=[0:df1:df1*(length(e)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

axes(handles.axes8);
plot(f,abs(fftshift(E)),'r');
                va=0;
            else
                     if sw==4%FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
                         %FM
%****************FM����*******************
dt=0.001;                                   %�趨ʱ�䲽��
t=0.01:0.001:1;                    	           %����ʱ������
fm=get(handles.edit1,'string');        %�趨�����ź�Ƶ��
fc=get(handles.edit2,'string');        %�趨�ز�Ƶ��
am=get(handles.edit3,'string');        %�趨�����źŷ���
fm=str2num(fm);
fc=str2num(fc);
am=str2num(am);
                        %���ɵ����ź�
if spe1==1
mt=am*cos(2*pi*fm*t);
else if spe1==2
 mt=sawtooth(2*pi*fm*t);
    else
 mt=square(2*pi*fm*t);
    end
end
% mt=awgn(mt,zs);
ct=cos(2*pi*fc*t);                             %�����ز�
kf=get(handles.edit5,'string');
kf=str2num(kf);%�趨��Ƶָ��

int_mt(1)=0;
for i=1:length(t)-1  
    int_mt(i+1)=int_mt(i)+mt(i)*dt;               %���ź�m(t)�Ļ���
end                                          %���ƣ������ѵ��ź�
sfm=am*cos(2*pi*fc*t+2*pi*kf*int_mt);            %�����ź�
sfm=awgn(sfm,zs);
%****************FM���*******************
for i=1:length(t)-1                             %�����ź�ͨ��΢��������
    diff_sfm(i)=(sfm(i+1)-sfm(i))./dt;
end
diff_sfmn = abs(hilbert(diff_sfm));     %hilbert�任�������ֵ�õ�˲ʱ���ȣ�����첨��
zero=(max(diff_sfmn)-min(diff_sfmn))/2;
diff_sfmn1=diff_sfmn-zero;
%*****************************************
%��*��*��*��*��*��*��*��*��*��*��*��*��*��*��*��
%**************ʱ��Ƶ��ת��**************
ts=0.001;                                    %�������
fs=1/ts;                                      %����Ƶ��
df=0.25;                                     %�����Ƶ�ʷֱ��ʣ���������Ҷ�任
%ʱ������ʾFFT����СƵ�ʼ��
%*****�Ե����ź�m(t)����Ҷ�任*****
if spe1==1
m=am*cos(2*pi*fm*t);
else if spe1==2
 m=sawtooth(2*pi*fm*t);
    else
 m=square(2*pi*fm*t);
    end
end                         %ԭ���ź�
% m=awgn(m,zs);
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(m);
n=2^(max(nextpow2(n1),nextpow2(n2)));
M=fft(m,n);
m=[m,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
M=M/fs;                                   %���ţ�������Ƶ��ͼ������۲�
bb=[0:df1:df1*(length(m)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

%************���ѵ��ź�u����任**********
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(sfm);
n=2^(max(nextpow2(n1),nextpow2(n2)));
U=fft(sfm,n);
u=[sfm,zeros(1,n-n2)];
df1=fs/n;                                   %�����Ƕ��ѵ��ź�u����任
U=U/fs;                                    %����


%************���ز��ź�c����任**********
b=cos(2*pi*fc*t);                            %�ز��ź�
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(b);
n=2^(max(nextpow2(n1),nextpow2(n2)));
C=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
C=C/fs;                                   %���ţ�������Ƶ��ͼ������۲�
%f=[0:df1:df1*(length(c)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������


%

nsfm=sfm;                              
for i=1:length(t)-1                           %�����ź�ͨ��΢��������
    diff_nsfm(i)=(nsfm(i+1)-nsfm(i))./dt;
end
diff_nsfmn = abs(hilbert(diff_nsfm));     %hilbert�任�������ֵ�õ�˲ʱ���ȣ�����첨��
zero=(max(diff_nsfmn)-min(diff_nsfmn))/2;
diff_nsfmn1=diff_nsfmn-zero;


%

axes(handles.axes1);
plot(t,mt);		
axes(handles.axes2);
plot(bb,abs(fftshift(M)));
axes(handles.axes3);
plot(t,ct);
axes(handles.axes6);
plot(bb,abs(fftshift(U)))
axes(handles.axes5);
plot(t,sfm);
axes(handles.axes4);
plot(bb,abs(fftshift(C)))
axes(handles.axes7);
plot((1:length(diff_nsfmn1))./1000,diff_nsfmn1./400,'r');
%
%************�Խ���ź�z����任**********
%%%%%%%%%%%%%%%%%%%%%%%%%
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(diff_nsfmn1);
n=2^(max(nextpow2(n1),nextpow2(n2)));
G=fft(diff_nsfmn1,n);
diff_nsfmn1=[diff_nsfmn1,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
G=G/fs;                                   %���ţ�������Ƶ��ͼ������۲�
%%%%%%%%%%%%%%%%%%%%%
axes(handles.axes8);
plot(bb,abs(fftshift(G)),'r');

%************�Խ���ź�z����任**********
%c=cos(2*pi*fc*t);                            %�ز��ź�
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(diff_nsfmn1);
n=2^(max(nextpow2(n1),nextpow2(n2)));
diff_nsfmn1=fft(diff_nsfmn1,n);
diff_nsfmn1=[diff_nsfmn1,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
diff_nsfmn1=diff_nsfmn1/fs;                                   %���ţ�������Ƶ��ͼ������۲�
bb=[0:df1:df1*(length(diff_nsfmn1)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������
%

%axes(handles.axes8);
% plot(f,abs(fftshift(diff_nsfmn1)))
%             else if va5==1
%                     none;
%                 end
va=0;
                     else
                         if sw==5%PM
                         %PM
                         %PM
                         %PM
                         %PM
                         %PM
                         %PM
                         %PM
                         %PM
                         %PM
                         %PM
                         %PM
 %****************PM����*******************
dt=0.001;                                   %�趨ʱ�䲽��
t=0.01:0.001:1;                    	           %����ʱ������
fm=get(handles.edit1,'string');        %�趨�����ź�Ƶ��
fc=get(handles.edit2,'string');        %�趨�ز�Ƶ��
am=get(handles.edit3,'string');        %�趨�����źŷ���
fm=str2num(fm);
fc=str2num(fc);
am=str2num(am);

if spe1==1
mt=am*cos(2*pi*fm*t);
else if spe1==2
 mt=sawtooth(2*pi*fm*t);
    else
 mt=square(2*pi*fm*t);
    end
end                        %���ɵ����ź�
% mt=awgn(mt,zs);
ct=-sin(2*pi*fc*t);                             %�����ز�
kf=get(handles.edit5,'string');
kf=str2num(kf);%�趨��Ƶָ��
int_mt(1)=0;
for i=1:length(t)-1  
    int_mt(i+1)=int_mt(i)+mt(i)*dt;               %���ź�m(t)�Ļ���
end                                          %���ƣ������ѵ��ź�
sfm=am*(-1)*sin(2*pi*fc*t+2*pi*kf*int_mt);            %�����ź�
sfm=awgn(sfm,zs);
%****************FM���*******************
for i=1:length(t)-1                             %�����ź�ͨ��΢��������
    diff_sfm(i)=(sfm(i+1)-sfm(i))./dt;
end
diff_sfmn = abs(hilbert(diff_sfm));     %hilbert�任�������ֵ�õ�˲ʱ���ȣ�����첨��
zero=(max(diff_sfmn)-min(diff_sfmn))/2;
diff_sfmn1=diff_sfmn-zero;
%*****************************************
%��*��*��*��*��*��*��*��*��*��*��*��*��*��*��*��
%**************ʱ��Ƶ��ת��**************
ts=0.001;                                    %�������
fs=1/ts;                                      %����Ƶ��
df=0.25;                                     %�����Ƶ�ʷֱ��ʣ���������Ҷ�任
%ʱ������ʾFFT����СƵ�ʼ��
%*****�Ե����ź�m(t)����Ҷ�任*****

if spe1==1
m=am*cos(2*pi*fm*t);
else if spe1==2
 m=sawtooth(2*pi*fm*t);
    else
 m=square(2*pi*fm*t);
    end
    end                         %ԭ���ź�
%     m=awgn(m,zs);
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(m);
n=2^(max(nextpow2(n1),nextpow2(n2)));
M=fft(m,n);
m=[m,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
M=M/fs;                                   %���ţ�������Ƶ��ͼ������۲�
bb=[0:df1:df1*(length(m)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������

%************���ѵ��ź�u����任**********
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(sfm);
n=2^(max(nextpow2(n1),nextpow2(n2)));
U=fft(sfm,n);
u=[sfm,zeros(1,n-n2)];
df1=fs/n;                                   %�����Ƕ��ѵ��ź�u����任
U=U/fs;                                    %����


%************���ز��ź�c����任**********
b=cos(2*pi*fc*t);                            %�ز��ź�
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(b);
n=2^(max(nextpow2(n1),nextpow2(n2)));
C=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
C=C/fs;                                   %���ţ�������Ƶ��ͼ������۲�
%f=[0:df1:df1*(length(c)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������


%

nsfm=sfm;                              
for i=1:length(t)-1                           %�����ź�ͨ��΢��������
    diff_nsfm(i)=(nsfm(i+1)-nsfm(i))./dt;
end
diff_nsfmn = abs(hilbert(diff_nsfm));     %hilbert�任�������ֵ�õ�˲ʱ���ȣ�����첨��
zero=(max(diff_nsfmn)-min(diff_nsfmn))/2;
diff_nsfmn1=diff_nsfmn-zero;



axes(handles.axes1);
plot(t,mt);		
axes(handles.axes2);
plot(bb,abs(fftshift(M)));
axes(handles.axes3);
plot(t,ct);
axes(handles.axes6);
plot(bb,abs(fftshift(U)))
axes(handles.axes5);
plot(t,sfm);
axes(handles.axes4);
plot(bb,abs(fftshift(C)))
axes(handles.axes7);
plot((1:length(diff_nsfmn1))./1000,diff_nsfmn1./400,'r');

%************�Խ���ź�z����任**********
%%%%%%%%%%%%%%%%%%%%%%%%%
fs=1/ts;
if nargin==2
    n1=0;
else
    n1=fs/df;
end
n2=length(diff_nsfmn1);
n=2^(max(nextpow2(n1),nextpow2(n2)));
G=fft(diff_nsfmn1,n);
diff_nsfmn1=[diff_nsfmn1,zeros(1,n-n2)];
df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
G=G/fs;                                   %���ţ�������Ƶ��ͼ������۲�
%%%%%%%%%%%%%%%%%%%%%
axes(handles.axes8);
plot(bb,abs(fftshift(G)),'r');
%************�Խ���ź�z����任**********
%c=cos(2*pi*fc*t);                            %�ز��ź�
% fs=1/ts;
% if nargin==2
%     n1=0;
% else
%     n1=fs/df;
% end
% n2=length(diff_nsfmn1);
% n=2^(max(nextpow2(n1),nextpow2(n2)));
% diff_nsfmn1=fft(diff_nsfmn1,n);
% diff_nsfmn1=[diff_nsfmn1,zeros(1,n-n2)];
% df1=fs/n;                                   %���ϳ����ǶԵ��ƺ���ź�u����任
% diff_nsfmn1=diff_nsfmn1/fs;                                   %���ţ�������Ƶ��ͼ������۲�
% bb=[0:df1:df1*(length(diff_nsfmn1)-1)]-fs/2;                %ʱ��������Ӧ��Ƶ������
%
                         va=0;
                         else
                             
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                             %Ƶ�ָ���
                            %%%�����źŵĲ���
fs=30000;%����Ƶ��
t=(0:1/fs:0.01);%ͼ�����������

f1=get(handles.edit1,'string');        
f2=get(handles.edit2,'string');        
f3=get(handles.edit4,'string');        
f1=str2num(f1);
f2=str2num(f2);
f3=str2num(f3);
% f1=1000;f2=2000;f3=3000;%�����ź�Ƶ��
s1=sin(2*pi*f1*t);%���������ź�
s2=sin(2*pi*f2*t);
s3=sin(2*pi*f3*t);
% s1=sawtooth(2*pi*f2*t);
% s3=square(2*pi*f3*t);
% plot(t,s1);xlabel('��λ��s');ylabel('����');title('�����ź�1');
% figure(1)
% subplot(3,2,1);
axes(handles.axes1);
plot(t,s1);
% xlabel('��λ��s');ylabel('����');title('�����ź�1');%��ʾ�����ź�ͼ��
% subplot(3,2,3);
axes(handles.axes3);
plot(t,s2);
% xlabel('��λ��s');ylabel('����');title('�����ź�2');
% subplot(3,2,5);
axes(handles.axes5);
plot(t,s3);
% xlabel('��λ��s');ylabel('����');title('�����ź�3');
%%%Ƶ�����
N=1024;
Y1=fft(s1,N);Y1=fftshift(Y1);%���ٸ���Ҷ�任�ó�Ƶ�׺���
Y2=fft(s2,N);Y2=fftshift(Y2);
Y3=fft(s3,N);Y3=fftshift(Y3);   
f=(0:N-1)*fs/N-fs/2;
% subplot(5,2,1);plot(f,abs(Y1));xlabel('��λ��HZ');ylabel('����');title('����Ƶ��ͼ1');
% subplot(5,2,3);plot(f,abs(Y2));xlabel('��λ��HZ');ylabel('����');title('����Ƶ��ͼ2');
% subplot(5,2,5);plot(f,abs(Y3));xlabel('��λ��HZ');ylabel('����');title('����Ƶ��ͼ3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%SSB����
fc1=10000;fc2=20000;fc3=30000;fs1=100000;
sm1 = modulate(s1,fc1,fs1,'amssb');   %���źŽ���SSB����
sm2 = modulate(s2,fc2,fs1,'amssb');   %���źŽ���SSB����
% sfm3 = modulate(s3,fc3,fs1,'fm');   %���źŽ���FM����
sfm3 = modulate(s3,fc3,fs1,'amssb');   %���źŽ���FM����
% figure(2)
% subplot(3,2,1);plot(t,sm1);xlabel('��λ��s');ylabel('����');title('SSB1');
% subplot(3,2,3);plot(t,sm2);xlabel('��λ��s');ylabel('����');title('SSB2');
% subplot(3,2,5);plot(t,sfm3);xlabel('��λ��s');ylabel('����');title('FM3');
%%SSB���Ƶ�Ƶ�����
F1=fft(sm1,N);F1=fftshift(F1); 
F2=fft(sm2,N);F2=fftshift(F2);      
FM3=fft(sfm3,N);FM3=fftshift(FM3); 
f1=(0:N-1)*fs1/N-fs1/2;
axes(handles.axes7);
% subplot(5,2,7);
plot(f1,abs(F1));
% xlabel('��λ��HZ');ylabel('����');title('SSBƵ��ͼ1');
% subplot(5,2,8);
% 
axes(handles.axes8);
plot(f1,abs(F2));
% xlabel('��λ��HZ');ylabel('����');title('SSBƵ��ͼ2');
% % subplot(5,2,9);
% 
axes(handles.axes9);
plot(f1,abs(FM3));
% xlabel('��λ��HZ');ylabel('����');title('FMƵ��ͼ3');
% grid;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%����·�źŵ��Ӻϳ�Ϊһ·���ŵ��д���%%%%%%%%%%%%%%%%%%
dj1=sm1+sm2+sfm3;%��·�źŵ���
zss=get(handles.edit3,'string');        
zss=str2num(zss);
dj3=awgn(dj1,zss);%��·SSB+G�ĵ���
zss=zss-5
s1=0.8*s1;
s2=0.7*s2;
s3=0.62*s3;
s1=fft(s1);
s2=fft(s2);
s3=fft(s3);
s1=awgn(s1,zss);
s2=awgn(s2,zss);
s3=awgn(s3,zss);
s1=ifft(s1);
s2=ifft(s2);
s3=ifft(s3);
% figure(3)   
% subplot(2,2,1);plot(t,dj1);xlabel('��λ��s');ylabel('����');title('��·�źŵ���');
% subplot(5,2,10);plot(t,dj3);xlabel('��λ��s');ylabel('����');title('��·�źŵ���');
%%%��·�źŵ��Ӻϳ�Ϊһ·�źŵ�Ƶ�׷���
DJ1=fft(dj1,N);DJ1=fftshift(DJ1);
DJ3=fft(dj3,N);DJ3=fftshift(DJ3);
% subplot(2,2,2);plot(f1,abs(DJ1));xlabel('��λ��HZ');ylabel('����');title('��·�źŵ���Ƶ��ͼ');
% subplot(5,2,10);
axes(handles.axes10);
plot(f1,abs(DJ3));
% xlabel('��λ��HZ');ylabel('����');title('��·�źŵ���Ƶ��ͼ');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%��ͨ�˲��������
fs1=100000;
Rp=0.5;Rs=40;  %���б�ѩ��2����ƴ�ͨ�˲���1��               
% Wp1=[9000 12000]/50000;   %����Ƶ�� fs1/2=50000         
% Ws1=[8000 13000]/50000;
Wp1=[9000 11000]/50000;   %����Ƶ�� fs1/2=50000         
Ws1=[8000 12000]/50000;
[n1,Wn1]=cheb2ord(Wp1,Ws1,Rp,Rs);
[b1,a1]=cheby2(n1,Rs,Wn1);
[h1,w1]=freqz(b1,a1);%��ʾ�б�ѩ��2����ƴ�ͨ�˲���Ƶ����Ӧ

% Wp2=[19000 23000]/50000;  %���б�ѩ��2����ƴ�ͨ�˲���2��
% Ws2=[18000 24000]/50000;
Wp2=[19000 21000]/50000;  %���б�ѩ��2����ƴ�ͨ�˲���2��
Ws2=[18000 22000]/50000;
[n2,Wn2]=cheb2ord(Wp2,Ws2,Rp,Rs);
[b2,a2]=cheby2(n2,Rs,Wn2);
[h2,w2]=freqz(b2,a2);
% Wp3=[31000 35000]/50000;  %���б�ѩ��2����ƴ�ͨ�˲���3��
% Ws3=[30000 36000]/50000;

Wp3=[29000 31000]/50000;  %���б�ѩ��2����ƴ�ͨ�˲���3��
Ws3=[28000 32000]/50000;
[n3,Wn3]=cheb2ord(Wp3,Ws3,Rp,Rs);
[b3,a3]=cheby2(n3,Rs,Wn3);
[h3,w3]=freqz(b3,a3);
% figure(4);
% subplot(3,1,1);plot(w1*fs1/(2*pi),abs(h1));xlabel('Ƶ��');ylabel('����');title('�б�ѩ��2�ʹ�ͨ�˲���1');
% subplot(3,1,2);plot(w2*fs1/(2*pi),abs(h2));xlabel('Ƶ��');ylabel('����');title('�б�ѩ��2�ʹ�ͨ�˲���2');
% subplot(3,1,3);plot(w3*fs1/(2*pi),abs(h3));xlabel('Ƶ��');ylabel('����');title('�б�ѩ��2�ʹ�ͨ�˲���3');
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���������˳�
lvn11=filter(b1,a1,dj3);
lvn12=filter(b2,a2,dj3);
lvn13=filter(b3,a3,dj3);
% figure(7)
% subplot(3,2,1);plot(t,lvn11);xlabel('��λ��s');ylabel('����');title('��·ssb+n���˳�1')
% subplot(3,2,3);plot(t,lvn12);xlabel('��λ��s');ylabel('����');title('��·ssb+n���˳�2');
% subplot(3,2,5);plot(t,lvn13);xlabel('��λ��s');ylabel('����');title('��·fm+n���˳�3');
FN11=fft(lvn11,N);FN11=fftshift(FN11); 
FN22=fft(lvn12,N);FN22=fftshift(FN22);        
FN33=fft(lvn12,N);FN33=fftshift(FN33); 
% subplot(3,2,2);plot(f1,abs(FN11));xlabel('��λ��HZ');ylabel('����');title('��·�˳�SSB+NƵ��ͼ1');
% subplot(3,2,4);plot(f1,abs(FN22));xlabel('��λ��HZ');ylabel('����');title('��·�˳�SSB+NƵ��ͼ2');
% subplot(3,2,6);plot(f1,abs(FN33));xlabel('��λ��HZ');ylabel('����');title('��·�˳�fm+NƵ��ͼ3');
%%%�Լ������˳�����·�źŽ��
sn10=lvn11.*cos(2*pi*10000*t/fs);   %�����ѵ��źŷֱ���Ը��Եĸ�Ƶ�ز��ź�
sn20=lvn12.*cos(2*pi*20000*t/fs);
sn30=lvn13.*cos(2*pi*30000*t/fs);
ZN10=fft(sn10,N);ZN10=fftshift(ZN10); %����ź�Ƶ�� 
ZN20=fft(sn20,N);ZN20=fftshift(ZN20);   
ZN30=fft(sn30,N);ZN30=fftshift(ZN30);
% figure(8)
% subplot(3,2,1);plot(t,sn10);xlabel('��λ��s');ylabel('����');title('ssb+n����źŲ���1')
% subplot(3,2,2);plot(f,abs(ZN10));xlabel('��λ��HZ');ylabel('����'); title('ssb+n����ź�Ƶ��1') 
% subplot(3,2,3);plot(t,sn20);xlabel('��λ��s');ylabel('����');title('ssb+n����źŲ���2')
% subplot(3,2,4);plot(f,abs(ZN20));xlabel('��λ��HZ');ylabel('����'); title('ssb+n����ź�Ƶ��2') 
% subplot(3,2,5);plot(t,sn30);xlabel('��λ��s');ylabel('����');title('fm+n����źŲ���2')
% subplot(3,2,6);plot(f,abs(ZN30));xlabel('��λ��HZ');ylabel('����'); title('fm+n����ź�Ƶ��2') 
grid;     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
Rp=0.5;Rs=40;   %��ͨ�˲�������ѡ��
fs=30000;
%%%��ͨ�˲���1
Wp11=[900 1100]/15000
Ws11=[800 1200]/15000;
[n11,Wn11]=cheb2ord(Wp11,Ws11,Rp,Rs);
[b11,a11]=cheby2(n11,Rs,Wn11);
[h11,w11]=freqz(b11,a11);
%%%��ͨ�˲���2
Wp12=[1900 2100]/15000;
Ws12=[1800 2200]/15000;
[n12,Wn12]=cheb2ord(Wp12,Ws12,Rp,Rs);
[b12,a12]=cheby2(n12,Rs,Wn12);
[h12,w12]=freqz(b12,a12);
%%%��ͨ�˲���3
Wp13=[2900 3100]/15000;
Ws13=[2800 3200]/15000;
[n13,Wn13]=cheb2ord(Wp13,Ws13,Rp,Rs);
[b13,a13]=cheby2(n13,Rs,Wn1);
[h13,w13]=freqz(b13,a13);
% figure(9)   %ͼΪ��ͨ�˲�����Ƶ����Ӧ
% subplot(3,1,1);plot(w11*fs/(2*pi),abs(h11));xlabel('Ƶ��');ylabel('����');title('��ͨ�˲�����Ƶ����Ӧ1');
% subplot(3,1,2);plot(w12*fs/(2*pi),abs(h12));xlabel('Ƶ��');ylabel('����');title('��ͨ�˲�����Ƶ����Ӧ2');
% subplot(3,1,3);plot(w13*fs/(2*pi),abs(h13));xlabel('Ƶ��');ylabel('����');title('��ͨ�˲�����Ƶ����Ӧ3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%�ָ��������źŵ�ʱ���κ�Ƶ�׷���
yy11=filter(b11,a11,sn10);
yy21=filter(b12,a12,sn20);
yy31=filter(b13,a13,sn30);
% figure(11)  
YY11=fft(yy11,N);YY11=fftshift(YY11); 
YY21=fft(yy21,N);YY21=fftshift(YY21);  
YY31=fft(yy31,N);YY31=fftshift(YY31);
% t=(0:1/fs:0.01);%ͼ�����������
% subplot(5,2,2);

axes(handles.axes2);
% plot(t,yy11);
plot(t,s1,'m');
% xlabel('��λ��s');ylabel('����');title('��������ssb�ָ��źŵ�ʱ����1');%ͼΪ�ָ��źŵ�ʱ����
% subplot(5,2,4);
axes(handles.axes4);
% plot(t,yy21);
plot(t,s2,'k');
% xlabel('��λ��s');ylabel('����');title('��������ssb�ָ��źŵ�ʱ����2');
% subplot(5,2,6);
axes(handles.axes6);
% plot(t,yy31);
plot(t,s3,'r');
% xlabel('��λ��s');ylabel('����'); title('��������fm�ָ��źŵ�ʱ����3'); 
% subplot(3,2,2);plot(f,abs(YY11));xlabel('��λ:Hz');ylabel('����');title('��������ssb�ָ��źŵ�Ƶ�׷���1');%ͼΪ�ָ��źŵ�Ƶ�׷���
% subplot(3,2,4);plot(f,abs(YY21));xlabel('��λ:Hz');ylabel('����');title('��������ssb�ָ��źŵ�Ƶ�׷���2');
% subplot(3,2,6);plot(f,abs(YY31));xlabel('��λ:Hz');ylabel('����');title('���������ָ��źŵ�Ƶ�׷���3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
                         end
                     end
            end
    end
end


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1
set(handles.radiobutton1,'value',1);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',1);
set(handles.radiobutton3,'value',0);

% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3
set(handles.radiobutton1,'value',0);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',1);

% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4

% --- Executes on button press in radiobutton5.
function radiobutton5_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton5



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sw;
if sw==1
open('AM.slx');
va1=0;
else
    if sw==2
open('DSB.slx');
va1=0;
    else
    if sw==3
open('SSB.slx');
va1=0;
    else
    if sw==4
open('FM.slx');
va1=0;
    else
    if sw==5
        open('PM.slx');
        va1=0;
    else
        open('PFFY.slx');
        va1=0;
    end
    end
    end
    end
end


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'value',1);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.edit1,'string',20);
set(handles.edit2,'string',200);
set(handles.edit3,'string',1.2);
set(handles.text4,'string','ֱ���źŷ���');
set(handles.text4,'visible','on');
set(handles.edit3,'visible','on');
set(handles.text6,'visible','off');
set(handles.edit5,'visible','off');
global sw;
sw=1;
axes(handles.axes1);
cla reset;
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;
axes(handles.axes5);
cla reset;
axes(handles.axes6);
cla reset;
axes(handles.axes7);
cla reset;
axes(handles.axes8);
cla reset;
axes(handles.axes9);
cla reset;
axes(handles.axes10);
cla reset;
set(handles.radiobutton1,'visible','on');
set(handles.radiobutton2,'visible','on');
set(handles.radiobutton3,'visible','on');
set(handles.text2,'string','�����ź�Ƶ��');
set(handles.text3,'string','�ز��ź�Ƶ��');
set(handles.text5,'string','������');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','AM(����)');
set(handles.text11,'string','dB');
set(handles.text12,'string','V');
set(handles.text12,'visible','on');
% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'value',1);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.edit1,'string',20);
set(handles.edit2,'string',200);
set(handles.edit3,'string',0);
set(handles.text4,'visible','off');
set(handles.edit3,'visible','off');
set(handles.text6,'visible','off');
set(handles.edit5,'visible','off');
global sw;
sw=2;
axes(handles.axes1);
cla reset;
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;
axes(handles.axes5);
cla reset;
axes(handles.axes6);
cla reset;
axes(handles.axes7);
cla reset;
axes(handles.axes8);
cla reset;
axes(handles.axes9);
cla reset;
axes(handles.axes10);
cla reset;
set(handles.radiobutton1,'visible','on');
set(handles.radiobutton2,'visible','on');
set(handles.radiobutton3,'visible','on');
set(handles.text2,'string','�����ź�Ƶ��');
set(handles.text3,'string','�ز��ź�Ƶ��');
set(handles.text5,'string','������');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','DSB(˫�ߴ�)');
set(handles.text11,'string','dB');
set(handles.text12,'string','V');
set(handles.text12,'visible','off');

% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'value',1);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.edit1,'string',20);
set(handles.edit2,'string',200);
set(handles.edit3,'string',0);
set(handles.text4,'visible','off');
set(handles.edit3,'visible','off');
set(handles.text6,'visible','off');
set(handles.edit5,'visible','off');
global sw;
sw=3;
axes(handles.axes1);
cla reset;
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;
axes(handles.axes5);
cla reset;
axes(handles.axes6);
cla reset;
axes(handles.axes7);
cla reset;
axes(handles.axes8);
cla reset;
axes(handles.axes9);
cla reset;
axes(handles.axes10);
cla reset;
set(handles.radiobutton1,'visible','on');
set(handles.radiobutton2,'visible','on');
set(handles.radiobutton3,'visible','on');
set(handles.text2,'string','�����ź�Ƶ��');
set(handles.text3,'string','�ز��ź�Ƶ��');
set(handles.text5,'string','������');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','SSB(���ߴ�)');
set(handles.text11,'string','dB');
set(handles.text12,'string','V');
set(handles.text12,'visible','off');

% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'value',1);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.edit1,'string',5);
set(handles.edit2,'string',50);
set(handles.edit3,'string',5);
set(handles.text4,'string','�����źŷ���');
set(handles.text4,'visible','on');
set(handles.edit3,'visible','on');
set(handles.text6,'string','��Ƶָ��');
set(handles.text6,'visible','on');
set(handles.edit5,'visible','on');
set(handles.edit5,'string',10);
global sw;
sw=4;
axes(handles.axes1);
cla reset;
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;
axes(handles.axes5);
cla reset;
axes(handles.axes6);
cla reset;
axes(handles.axes7);
cla reset;
axes(handles.axes8);
cla reset;
axes(handles.axes9);
cla reset;
axes(handles.axes10);
cla reset;
set(handles.radiobutton1,'visible','on');
set(handles.radiobutton2,'visible','on');
set(handles.radiobutton3,'visible','on');
set(handles.text2,'string','�����ź�Ƶ��');
set(handles.text3,'string','�ز��ź�Ƶ��');
set(handles.text5,'string','������');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','FM(��Ƶ)');
set(handles.text11,'string','dB');
set(handles.text12,'string','V');
set(handles.text12,'visible','on');

% --------------------------------------------------------------------
function Untitled_6_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.radiobutton1,'value',1);
set(handles.radiobutton2,'value',0);
set(handles.radiobutton3,'value',0);
set(handles.edit1,'string',5);
set(handles.edit2,'string',50);
set(handles.edit3,'string',5);
set(handles.text4,'string','�����źŷ���');
set(handles.text4,'visible','on');
set(handles.edit3,'visible','on');
set(handles.text6,'string','����ָ��');
set(handles.text6,'visible','on');
set(handles.edit5,'visible','on');
set(handles.edit5,'string',10);
global sw;
sw=5;
axes(handles.axes1);
cla reset;
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;
axes(handles.axes5);
cla reset;
axes(handles.axes6);
cla reset;
axes(handles.axes7);
cla reset;
axes(handles.axes8);
cla reset;
axes(handles.axes9);
cla reset;
axes(handles.axes10);
cla reset;
set(handles.radiobutton1,'visible','on');
set(handles.radiobutton2,'visible','on');
set(handles.radiobutton3,'visible','on');
set(handles.text2,'string','�����ź�Ƶ��');
set(handles.text3,'string','�ز��ź�Ƶ��');
set(handles.text5,'string','������');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','PM(����)');
set(handles.text11,'string','dB');
set(handles.text12,'string','V');
set(handles.text12,'visible','on');

% --------------------------------------------------------------------
function Untitled_7_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global sw;
sw=6;
set(handles.radiobutton1,'visible','off');
set(handles.radiobutton2,'visible','off');
set(handles.radiobutton3,'visible','off');

set(handles.edit1,'string',1000);
set(handles.edit2,'string',2000);
set(handles.edit4,'string',3000);
set(handles.edit3,'string',20);

set(handles.text2,'string','��1·�ź�Ƶ��');
set(handles.text3,'string','��2·�ź�Ƶ��');
set(handles.text5,'string','��3·�ź�Ƶ��');
set(handles.text4,'string','������');

set(handles.edit3,'visible','on');
set(handles.text4,'visible','on');

set(handles.text6,'visible','off');
set(handles.edit5,'visible','off');
axes(handles.axes1);
cla reset;
axes(handles.axes2);
cla reset;
axes(handles.axes3);
cla reset;
axes(handles.axes4);
cla reset;
axes(handles.axes5);
cla reset;
axes(handles.axes6);
cla reset;
axes(handles.axes7);
cla reset;
axes(handles.axes8);
cla reset;
axes(handles.axes9);
cla reset;
axes(handles.axes10);
cla reset;
set(handles.axes9,'visible','on');
set(handles.axes10,'visible','on');
set(handles.text8,'string','Ƶ�ָ���');
set(handles.text11,'string','Hz');
set(handles.text12,'string','dB');
set(handles.text12,'visible','on');

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
zhujiemian(handles.figure1);
close AMS;


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ExitStr = questdlg('��ȷ���Ƿ��˳�ϵͳ��','�˳�ϵͳ','ȷ��','ȡ��','ȡ��');
if strcmp(ExitStr,'ȷ��')
    close;
end
