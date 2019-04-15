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
Ft=2000;    %采样频率
fpts=[0 aa*2]; %通带边界频率fp=100Hz，阻带截止频率fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %通带波动1%，阻带波动5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord估计采用凯塞窗设计的FIR滤 波器的参数
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %由fir1设计滤波器
e=fftfilt(b21,e);                %FIR低通滤波

% e=abs(hilbert(c));%hilbert变换，求绝对值得到瞬时幅度（包络检波）
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

ts=0.001;                                    %抽样间隔
fs=1/ts;                                      %抽样频率
df=0.25;                                     %所需的频率分辨率，用在求傅里叶变换
%时，它表示FFT的最小频率间隔
%*****对调制信号m(t)求傅里叶变换*****
                         %原调制信号
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
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(a)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes2);
plot(f,abs(fftshift(A)));

% axes(handles.axes8);
% plot(f,abs(fftshift(A)));
%*****对载波信号m(t)求傅里叶变换*****
b=cos(bb*5*t);                         %原调制信号
n1=0;
n2=length(b);
n=2^(max(nextpow2(0),nextpow2(n2)));
B=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(b)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes4);
plot(f,abs(fftshift(B)));

%*****对已调信号m(t)求傅里叶变换*****
c=(zz+a).*b;                        %原调制信号
c=awgn(c,zs);
n1=0;
n2=length(c);
n=2^(max(nextpow2(0),nextpow2(n2)));
C=fft(c,n);
c=[c,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(c)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes6);
plot(f,abs(fftshift(C)));

%*****对已调信号m(t)求傅里叶变换*****
c=(zz+a).*b;                        %原调制信号
c=awgn(c,zs);
d= abs(hilbert(c));
n1=0;
n2=length(d);
n=2^(max(nextpow2(0),nextpow2(n2)));
D=fft(d,n);
d=[d,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(d)-1)]-fs/2;                %时间向量对应的频率向量

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
Ft=2000;    %采样频率
fpts=[0 aa*2]; %通带边界频率fp=100Hz，阻带截止频率fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %通带波动1%，阻带波动5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord估计采用凯塞窗设计的FIR滤 波器的参数
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %由fir1设计滤波器
e=fftfilt(b21,e);                %FIR低通滤波



axes(handles.axes1);
plot(t,a);
axes(handles.axes3);
plot(b);
axes(handles.axes5);
plot(c);
axes(handles.axes7);
plot(t,e,'r');

ts=0.001;                                    %抽样间隔
fs=1/ts;                                      %抽样频率
df=0.25;                                     %所需的频率分辨率，用在求傅里叶变换
%时，它表示FFT的最小频率间隔
%*****对调制信号m(t)求傅里叶变换*****
if spe1==1
a=cos(aa*5*t);
else if spe1==2
        a=sawtooth(aa*5*t);
    else
        a=square(aa*5*t);
    end
end
% a=awgn(a,zs);
                         %原调制信号
n1=0;
n2=length(a);
n=2^(max(nextpow2(0),nextpow2(n2)));
A=fft(a,n);
a=[a,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(a)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes2);
plot(f,abs(fftshift(A)));

%*****对载波信号m(t)求傅里叶变换*****
b=cos(bb*5*t);                         %原调制信号
n1=0;
n2=length(b);
n=2^(max(nextpow2(0),nextpow2(n2)));
B=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(b)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes4);
plot(f,abs(fftshift(B)));

%*****对已调信号m(t)求傅里叶变换*****
c=a.*b;     %原调制信号
c=awgn(c,zs);
n1=0;
n2=length(c);
n=2^(max(nextpow2(0),nextpow2(n2)));
C=fft(c,n);
c=[c,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(c)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes6);
plot(f,abs(fftshift(C)));

% %*****对解调信号m(t)求傅里叶变换*****
% d=c.*b;
% e=c.*b;
% e = filter(y1,x1,d);                        %原调制信号

c=a.*b;
c=awgn(c,zs);
e=c.*b;
Ft=2000;    %采样频率
fpts=[0 aa*10]; %通带边界频率fp=100Hz，阻带截止频率fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %通带波动1%，阻带波动5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord估计采用凯塞窗设计的FIR滤 波器的参数
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %由fir1设计滤波器
e=fftfilt(b21,e);                %FIR低通滤波

n1=0;
n2=length(e);
n=2^(max(nextpow2(0),nextpow2(n2)));
E=fft(e,n);
e=[e,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(e)-1)]-fs/2;                %时间向量对应的频率向量


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
[n,Wp] = ellipord(Wp,Ws,Rp,Rs); %求滤波器的最小阶数
[x,y] = ellip(n,Rp,Rs,Wp); %设计椭圆滤波器
d = filter(x,y,c);
e=d.*b;

% 
Ft=2000;    %采样频率
fpts=[0 aa*2]; %通带边界频率fp=100Hz，阻带截止频率fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %通带波动1%，阻带波动5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord估计采用凯塞窗设计的FIR滤 波器的参数
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %由fir1设计滤波器
e=fftfilt(b21,e);                %FIR低通滤波


axes(handles.axes1);
plot(t,a);
axes(handles.axes3);
plot(b);
axes(handles.axes5);
plot(c);
axes(handles.axes7);
plot(t,e,'r');

ts=0.001;                                    %抽样间隔
fs=1/ts;                                      %抽样频率
df=0.25;                                     %所需的频率分辨率，用在求傅里叶变换
%时，它表示FFT的最小频率间隔
%*****对调制信号m(t)求傅里叶变换*****
if spe1==1
a=cos(aa*5*t);
else if spe1==2
        a=sawtooth(aa*5*t);
    else
        a=square(aa*5*t);
    end
end
% a=awgn(a,zs);
                         %原调制信号
n1=0;
n2=length(a);
n=2^(max(nextpow2(0),nextpow2(n2)));
A=fft(a,n);
a=[a,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(a)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes2);
plot(f,abs(fftshift(A)));

%*****对载波信号m(t)求傅里叶变换*****
b=cos(bb*5*t);                         %原调制信号
n1=0;
n2=length(b);
n=2^(max(nextpow2(0),nextpow2(n2)));
B=fft(b,n);
b=[b,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(b)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes4);
plot(f,abs(fftshift(B)));

%*****对已调信号m(t)求傅里叶变换*****
c=a.*b;                        %原调制信号
c=awgn(c,zs);
Rp=0.1;
Rs=80;
Wp=49*2*pi*0.001;
Ws=73*2*pi*0.001;
[n,Wp] = ellipord(Wp,Ws,Rp,Rs); %求滤波器的最小阶数
[x,y] = ellip(n,Rp,Rs,Wp); %设计椭圆滤波器
c = filter(x,y,c);
n1=0;
n2=length(c);
n=2^(max(nextpow2(0),nextpow2(n2)));
C=fft(c,n);
c=[c,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(c)-1)]-fs/2;                %时间向量对应的频率向量

axes(handles.axes6);
plot(f,abs(fftshift(C)));

%*****对解调信号m(t)求傅里叶变换*****
c=a.*b;
c=awgn(c,zs);
d=c.*b;
e=c.*b;
Ft=2000;    %采样频率
fpts=[0 aa*10]; %通带边界频率fp=100Hz，阻带截止频率fs=120Hz
mag=[1 0];      
dev=[0.01 0.05];  %通带波动1%，阻带波动5%
[n21,wn21,beta,ftype]=kaiserord(fpts,mag,dev,Ft);%kaiserord估计采用凯塞窗设计的FIR滤 波器的参数
b21=fir1(n21,wn21,kaiser(n21+1,beta));    %由fir1设计滤波器
e=fftfilt(b21,e);                %FIR低通滤波


n1=0;
n2=length(e);
n=2^(max(nextpow2(0),nextpow2(n2)));
E=fft(e,n);
e=[e,zeros(1,n-n2)];
df1=fs/n;                                   %以上程序是对信号求傅里变换
f=[0:df1:df1*(length(e)-1)]-fs/2;                %时间向量对应的频率向量

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
%****************FM调制*******************
dt=0.001;                                   %设定时间步长
t=0.01:0.001:1;                    	           %产生时间向量
fm=get(handles.edit1,'string');        %设定调制信号频率
fc=get(handles.edit2,'string');        %设定载波频率
am=get(handles.edit3,'string');        %设定调制信号幅度
fm=str2num(fm);
fc=str2num(fc);
am=str2num(am);
                        %生成调制信号
if spe1==1
mt=am*cos(2*pi*fm*t);
else if spe1==2
 mt=sawtooth(2*pi*fm*t);
    else
 mt=square(2*pi*fm*t);
    end
end
% mt=awgn(mt,zs);
ct=cos(2*pi*fc*t);                             %生成载波
kf=get(handles.edit5,'string');
kf=str2num(kf);%设定调频指数

int_mt(1)=0;
for i=1:length(t)-1  
    int_mt(i+1)=int_mt(i)+mt(i)*dt;               %求信号m(t)的积分
end                                          %调制，产生已调信号
sfm=am*cos(2*pi*fc*t+2*pi*kf*int_mt);            %调制信号
sfm=awgn(sfm,zs);
%****************FM解调*******************
for i=1:length(t)-1                             %接受信号通过微分器处理
    diff_sfm(i)=(sfm(i+1)-sfm(i))./dt;
end
diff_sfmn = abs(hilbert(diff_sfm));     %hilbert变换，求绝对值得到瞬时幅度（包络检波）
zero=(max(diff_sfmn)-min(diff_sfmn))/2;
diff_sfmn1=diff_sfmn-zero;
%*****************************************
%·*·*·*·*·*·*·*·*·*·*·*·*·*·*·*·
%**************时域到频域转换**************
ts=0.001;                                    %抽样间隔
fs=1/ts;                                      %抽样频率
df=0.25;                                     %所需的频率分辨率，用在求傅里叶变换
%时，它表示FFT的最小频率间隔
%*****对调制信号m(t)求傅里叶变换*****
if spe1==1
m=am*cos(2*pi*fm*t);
else if spe1==2
 m=sawtooth(2*pi*fm*t);
    else
 m=square(2*pi*fm*t);
    end
end                         %原调信号
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
df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
M=M/fs;                                   %缩放，便于在频铺图上整体观察
bb=[0:df1:df1*(length(m)-1)]-fs/2;                %时间向量对应的频率向量

%************对已调信号u求傅里变换**********
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
df1=fs/n;                                   %以上是对已调信号u求傅里变换
U=U/fs;                                    %缩放


%************对载波信号c求傅里变换**********
b=cos(2*pi*fc*t);                            %载波信号
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
df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
C=C/fs;                                   %缩放，便于在频铺图上整体观察
%f=[0:df1:df1*(length(c)-1)]-fs/2;                %时间向量对应的频率向量


%

nsfm=sfm;                              
for i=1:length(t)-1                           %接受信号通过微分器处理
    diff_nsfm(i)=(nsfm(i+1)-nsfm(i))./dt;
end
diff_nsfmn = abs(hilbert(diff_nsfm));     %hilbert变换，求绝对值得到瞬时幅度（包络检波）
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
%************对解调信号z求傅里变换**********
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
df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
G=G/fs;                                   %缩放，便于在频铺图上整体观察
%%%%%%%%%%%%%%%%%%%%%
axes(handles.axes8);
plot(bb,abs(fftshift(G)),'r');

%************对解调信号z求傅里变换**********
%c=cos(2*pi*fc*t);                            %载波信号
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
df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
diff_nsfmn1=diff_nsfmn1/fs;                                   %缩放，便于在频铺图上整体观察
bb=[0:df1:df1*(length(diff_nsfmn1)-1)]-fs/2;                %时间向量对应的频率向量
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
 %****************PM调制*******************
dt=0.001;                                   %设定时间步长
t=0.01:0.001:1;                    	           %产生时间向量
fm=get(handles.edit1,'string');        %设定调制信号频率
fc=get(handles.edit2,'string');        %设定载波频率
am=get(handles.edit3,'string');        %设定调制信号幅度
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
end                        %生成调制信号
% mt=awgn(mt,zs);
ct=-sin(2*pi*fc*t);                             %生成载波
kf=get(handles.edit5,'string');
kf=str2num(kf);%设定调频指数
int_mt(1)=0;
for i=1:length(t)-1  
    int_mt(i+1)=int_mt(i)+mt(i)*dt;               %求信号m(t)的积分
end                                          %调制，产生已调信号
sfm=am*(-1)*sin(2*pi*fc*t+2*pi*kf*int_mt);            %调制信号
sfm=awgn(sfm,zs);
%****************FM解调*******************
for i=1:length(t)-1                             %接受信号通过微分器处理
    diff_sfm(i)=(sfm(i+1)-sfm(i))./dt;
end
diff_sfmn = abs(hilbert(diff_sfm));     %hilbert变换，求绝对值得到瞬时幅度（包络检波）
zero=(max(diff_sfmn)-min(diff_sfmn))/2;
diff_sfmn1=diff_sfmn-zero;
%*****************************************
%·*·*·*·*·*·*·*·*·*·*·*·*·*·*·*·
%**************时域到频域转换**************
ts=0.001;                                    %抽样间隔
fs=1/ts;                                      %抽样频率
df=0.25;                                     %所需的频率分辨率，用在求傅里叶变换
%时，它表示FFT的最小频率间隔
%*****对调制信号m(t)求傅里叶变换*****

if spe1==1
m=am*cos(2*pi*fm*t);
else if spe1==2
 m=sawtooth(2*pi*fm*t);
    else
 m=square(2*pi*fm*t);
    end
    end                         %原调信号
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
df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
M=M/fs;                                   %缩放，便于在频铺图上整体观察
bb=[0:df1:df1*(length(m)-1)]-fs/2;                %时间向量对应的频率向量

%************对已调信号u求傅里变换**********
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
df1=fs/n;                                   %以上是对已调信号u求傅里变换
U=U/fs;                                    %缩放


%************对载波信号c求傅里变换**********
b=cos(2*pi*fc*t);                            %载波信号
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
df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
C=C/fs;                                   %缩放，便于在频铺图上整体观察
%f=[0:df1:df1*(length(c)-1)]-fs/2;                %时间向量对应的频率向量


%

nsfm=sfm;                              
for i=1:length(t)-1                           %接受信号通过微分器处理
    diff_nsfm(i)=(nsfm(i+1)-nsfm(i))./dt;
end
diff_nsfmn = abs(hilbert(diff_nsfm));     %hilbert变换，求绝对值得到瞬时幅度（包络检波）
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

%************对解调信号z求傅里变换**********
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
df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
G=G/fs;                                   %缩放，便于在频铺图上整体观察
%%%%%%%%%%%%%%%%%%%%%
axes(handles.axes8);
plot(bb,abs(fftshift(G)),'r');
%************对解调信号z求傅里变换**********
%c=cos(2*pi*fc*t);                            %载波信号
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
% df1=fs/n;                                   %以上程序是对调制后的信号u求傅里变换
% diff_nsfmn1=diff_nsfmn1/fs;                                   %缩放，便于在频铺图上整体观察
% bb=[0:df1:df1*(length(diff_nsfmn1)-1)]-fs/2;                %时间向量对应的频率向量
%
                         va=0;
                         else
                             
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                             %频分复用
                            %%%正弦信号的产生
fs=30000;%采样频率
t=(0:1/fs:0.01);%图形坐标轴设计

f1=get(handles.edit1,'string');        
f2=get(handles.edit2,'string');        
f3=get(handles.edit4,'string');        
f1=str2num(f1);
f2=str2num(f2);
f3=str2num(f3);
% f1=1000;f2=2000;f3=3000;%正弦信号频率
s1=sin(2*pi*f1*t);%产生正弦信号
s2=sin(2*pi*f2*t);
s3=sin(2*pi*f3*t);
% s1=sawtooth(2*pi*f2*t);
% s3=square(2*pi*f3*t);
% plot(t,s1);xlabel('单位：s');ylabel('幅度');title('正弦信号1');
% figure(1)
% subplot(3,2,1);
axes(handles.axes1);
plot(t,s1);
% xlabel('单位：s');ylabel('幅度');title('正弦信号1');%显示正弦信号图形
% subplot(3,2,3);
axes(handles.axes3);
plot(t,s2);
% xlabel('单位：s');ylabel('幅度');title('正弦信号2');
% subplot(3,2,5);
axes(handles.axes5);
plot(t,s3);
% xlabel('单位：s');ylabel('幅度');title('正弦信号3');
%%%频域分析
N=1024;
Y1=fft(s1,N);Y1=fftshift(Y1);%快速傅里叶变换得出频谱函数
Y2=fft(s2,N);Y2=fftshift(Y2);
Y3=fft(s3,N);Y3=fftshift(Y3);   
f=(0:N-1)*fs/N-fs/2;
% subplot(5,2,1);plot(f,abs(Y1));xlabel('单位：HZ');ylabel('幅度');title('函数频谱图1');
% subplot(5,2,3);plot(f,abs(Y2));xlabel('单位：HZ');ylabel('幅度');title('函数频谱图2');
% subplot(5,2,5);plot(f,abs(Y3));xlabel('单位：HZ');ylabel('幅度');title('函数频谱图3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%SSB调制
fc1=10000;fc2=20000;fc3=30000;fs1=100000;
sm1 = modulate(s1,fc1,fs1,'amssb');   %对信号进行SSB调制
sm2 = modulate(s2,fc2,fs1,'amssb');   %对信号进行SSB调制
% sfm3 = modulate(s3,fc3,fs1,'fm');   %对信号进行FM调制
sfm3 = modulate(s3,fc3,fs1,'amssb');   %对信号进行FM调制
% figure(2)
% subplot(3,2,1);plot(t,sm1);xlabel('单位：s');ylabel('幅度');title('SSB1');
% subplot(3,2,3);plot(t,sm2);xlabel('单位：s');ylabel('幅度');title('SSB2');
% subplot(3,2,5);plot(t,sfm3);xlabel('单位：s');ylabel('幅度');title('FM3');
%%SSB调制的频域分析
F1=fft(sm1,N);F1=fftshift(F1); 
F2=fft(sm2,N);F2=fftshift(F2);      
FM3=fft(sfm3,N);FM3=fftshift(FM3); 
f1=(0:N-1)*fs1/N-fs1/2;
axes(handles.axes7);
% subplot(5,2,7);
plot(f1,abs(F1));
% xlabel('单位：HZ');ylabel('幅度');title('SSB频谱图1');
% subplot(5,2,8);
% 
axes(handles.axes8);
plot(f1,abs(F2));
% xlabel('单位：HZ');ylabel('幅度');title('SSB频谱图2');
% % subplot(5,2,9);
% 
axes(handles.axes9);
plot(f1,abs(FM3));
% xlabel('单位：HZ');ylabel('幅度');title('FM频谱图3');
% grid;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%将三路信号叠加合成为一路在信道中传输%%%%%%%%%%%%%%%%%%
dj1=sm1+sm2+sfm3;%三路信号叠加
zss=get(handles.edit3,'string');        
zss=str2num(zss);
dj3=awgn(dj1,zss);%三路SSB+G的叠加
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
% subplot(2,2,1);plot(t,dj1);xlabel('单位：s');ylabel('幅度');title('三路信号叠加');
% subplot(5,2,10);plot(t,dj3);xlabel('单位：s');ylabel('幅度');title('三路信号叠加');
%%%三路信号叠加合成为一路信号的频谱分析
DJ1=fft(dj1,N);DJ1=fftshift(DJ1);
DJ3=fft(dj3,N);DJ3=fftshift(DJ3);
% subplot(2,2,2);plot(f1,abs(DJ1));xlabel('单位：HZ');ylabel('幅度');title('三路信号叠加频谱图');
% subplot(5,2,10);
axes(handles.axes10);
plot(f1,abs(DJ3));
% xlabel('单位：HZ');ylabel('幅度');title('三路信号叠加频谱图');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%带通滤波器的设计
fs1=100000;
Rp=0.5;Rs=40;  %用切比雪夫2型设计带通滤波器1；               
% Wp1=[9000 12000]/50000;   %数字频率 fs1/2=50000         
% Ws1=[8000 13000]/50000;
Wp1=[9000 11000]/50000;   %数字频率 fs1/2=50000         
Ws1=[8000 12000]/50000;
[n1,Wn1]=cheb2ord(Wp1,Ws1,Rp,Rs);
[b1,a1]=cheby2(n1,Rs,Wn1);
[h1,w1]=freqz(b1,a1);%显示切比雪夫2型设计带通滤波器频率响应

% Wp2=[19000 23000]/50000;  %用切比雪夫2型设计带通滤波器2；
% Ws2=[18000 24000]/50000;
Wp2=[19000 21000]/50000;  %用切比雪夫2型设计带通滤波器2；
Ws2=[18000 22000]/50000;
[n2,Wn2]=cheb2ord(Wp2,Ws2,Rp,Rs);
[b2,a2]=cheby2(n2,Rs,Wn2);
[h2,w2]=freqz(b2,a2);
% Wp3=[31000 35000]/50000;  %用切比雪夫2型设计带通滤波器3；
% Ws3=[30000 36000]/50000;

Wp3=[29000 31000]/50000;  %用切比雪夫2型设计带通滤波器3；
Ws3=[28000 32000]/50000;
[n3,Wn3]=cheb2ord(Wp3,Ws3,Rp,Rs);
[b3,a3]=cheby2(n3,Rs,Wn3);
[h3,w3]=freqz(b3,a3);
% figure(4);
% subplot(3,1,1);plot(w1*fs1/(2*pi),abs(h1));xlabel('频率');ylabel('幅度');title('切比雪夫2型带通滤波器1');
% subplot(3,1,2);plot(w2*fs1/(2*pi),abs(h2));xlabel('频率');ylabel('幅度');title('切比雪夫2型带通滤波器2');
% subplot(3,1,3);plot(w3*fs1/(2*pi),abs(h3));xlabel('频率');ylabel('幅度');title('切比雪夫2型带通滤波器3');
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%加噪声的滤出
lvn11=filter(b1,a1,dj3);
lvn12=filter(b2,a2,dj3);
lvn13=filter(b3,a3,dj3);
% figure(7)
% subplot(3,2,1);plot(t,lvn11);xlabel('单位：s');ylabel('幅度');title('三路ssb+n的滤出1')
% subplot(3,2,3);plot(t,lvn12);xlabel('单位：s');ylabel('幅度');title('三路ssb+n的滤出2');
% subplot(3,2,5);plot(t,lvn13);xlabel('单位：s');ylabel('幅度');title('三路fm+n的滤出3');
FN11=fft(lvn11,N);FN11=fftshift(FN11); 
FN22=fft(lvn12,N);FN22=fftshift(FN22);        
FN33=fft(lvn12,N);FN33=fftshift(FN33); 
% subplot(3,2,2);plot(f1,abs(FN11));xlabel('单位：HZ');ylabel('幅度');title('三路滤出SSB+N频谱图1');
% subplot(3,2,4);plot(f1,abs(FN22));xlabel('单位：HZ');ylabel('幅度');title('三路滤出SSB+N频谱图2');
% subplot(3,2,6);plot(f1,abs(FN33));xlabel('单位：HZ');ylabel('幅度');title('三路滤出fm+N频谱图3');
%%%对加噪声滤出的三路信号解调
sn10=lvn11.*cos(2*pi*10000*t/fs);   %各个已调信号分别乘以各自的高频载波信号
sn20=lvn12.*cos(2*pi*20000*t/fs);
sn30=lvn13.*cos(2*pi*30000*t/fs);
ZN10=fft(sn10,N);ZN10=fftshift(ZN10); %解调信号频谱 
ZN20=fft(sn20,N);ZN20=fftshift(ZN20);   
ZN30=fft(sn30,N);ZN30=fftshift(ZN30);
% figure(8)
% subplot(3,2,1);plot(t,sn10);xlabel('单位：s');ylabel('幅度');title('ssb+n解调信号波形1')
% subplot(3,2,2);plot(f,abs(ZN10));xlabel('单位：HZ');ylabel('幅度'); title('ssb+n解调信号频谱1') 
% subplot(3,2,3);plot(t,sn20);xlabel('单位：s');ylabel('幅度');title('ssb+n解调信号波形2')
% subplot(3,2,4);plot(f,abs(ZN20));xlabel('单位：HZ');ylabel('幅度'); title('ssb+n解调信号频谱2') 
% subplot(3,2,5);plot(t,sn30);xlabel('单位：s');ylabel('幅度');title('fm+n解调信号波形2')
% subplot(3,2,6);plot(f,abs(ZN30));xlabel('单位：HZ');ylabel('幅度'); title('fm+n解调信号频谱2') 
grid;     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
Rp=0.5;Rs=40;   %低通滤波器参数选择
fs=30000;
%%%低通滤波器1
Wp11=[900 1100]/15000
Ws11=[800 1200]/15000;
[n11,Wn11]=cheb2ord(Wp11,Ws11,Rp,Rs);
[b11,a11]=cheby2(n11,Rs,Wn11);
[h11,w11]=freqz(b11,a11);
%%%低通滤波器2
Wp12=[1900 2100]/15000;
Ws12=[1800 2200]/15000;
[n12,Wn12]=cheb2ord(Wp12,Ws12,Rp,Rs);
[b12,a12]=cheby2(n12,Rs,Wn12);
[h12,w12]=freqz(b12,a12);
%%%低通滤波器3
Wp13=[2900 3100]/15000;
Ws13=[2800 3200]/15000;
[n13,Wn13]=cheb2ord(Wp13,Ws13,Rp,Rs);
[b13,a13]=cheby2(n13,Rs,Wn1);
[h13,w13]=freqz(b13,a13);
% figure(9)   %图为低通滤波器的频率响应
% subplot(3,1,1);plot(w11*fs/(2*pi),abs(h11));xlabel('频率');ylabel('幅度');title('低通滤波器的频率响应1');
% subplot(3,1,2);plot(w12*fs/(2*pi),abs(h12));xlabel('频率');ylabel('幅度');title('低通滤波器的频率响应2');
% subplot(3,1,3);plot(w13*fs/(2*pi),abs(h13));xlabel('频率');ylabel('幅度');title('低通滤波器的频率响应3');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%恢复有噪声信号的时域波形和频谱分析
yy11=filter(b11,a11,sn10);
yy21=filter(b12,a12,sn20);
yy31=filter(b13,a13,sn30);
% figure(11)  
YY11=fft(yy11,N);YY11=fftshift(YY11); 
YY21=fft(yy21,N);YY21=fftshift(YY21);  
YY31=fft(yy31,N);YY31=fftshift(YY31);
% t=(0:1/fs:0.01);%图形坐标轴设计
% subplot(5,2,2);

axes(handles.axes2);
% plot(t,yy11);
plot(t,s1,'m');
% xlabel('单位：s');ylabel('幅度');title('由有噪声ssb恢复信号的时域波形1');%图为恢复信号的时域波形
% subplot(5,2,4);
axes(handles.axes4);
% plot(t,yy21);
plot(t,s2,'k');
% xlabel('单位：s');ylabel('幅度');title('由有噪声ssb恢复信号的时域波形2');
% subplot(5,2,6);
axes(handles.axes6);
% plot(t,yy31);
plot(t,s3,'r');
% xlabel('单位：s');ylabel('幅度'); title('由有噪声fm恢复信号的时域波形3'); 
% subplot(3,2,2);plot(f,abs(YY11));xlabel('单位:Hz');ylabel('幅度');title('由有噪声ssb恢复信号的频谱分析1');%图为恢复信号的频谱分析
% subplot(3,2,4);plot(f,abs(YY21));xlabel('单位:Hz');ylabel('幅度');title('由有噪声ssb恢复信号的频谱分析2');
% subplot(3,2,6);plot(f,abs(YY31));xlabel('单位:Hz');ylabel('幅度');title('由有噪声恢复信号的频谱分析3');
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
set(handles.text4,'string','直流信号幅度');
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
set(handles.text2,'string','调制信号频率');
set(handles.text3,'string','载波信号频率');
set(handles.text5,'string','白噪声');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','AM(调幅)');
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
set(handles.text2,'string','调制信号频率');
set(handles.text3,'string','载波信号频率');
set(handles.text5,'string','白噪声');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','DSB(双边带)');
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
set(handles.text2,'string','调制信号频率');
set(handles.text3,'string','载波信号频率');
set(handles.text5,'string','白噪声');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','SSB(单边带)');
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
set(handles.text4,'string','调制信号幅度');
set(handles.text4,'visible','on');
set(handles.edit3,'visible','on');
set(handles.text6,'string','调频指数');
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
set(handles.text2,'string','调制信号频率');
set(handles.text3,'string','载波信号频率');
set(handles.text5,'string','白噪声');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','FM(调频)');
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
set(handles.text4,'string','调制信号幅度');
set(handles.text4,'visible','on');
set(handles.edit3,'visible','on');
set(handles.text6,'string','调相指数');
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
set(handles.text2,'string','调制信号频率');
set(handles.text3,'string','载波信号频率');
set(handles.text5,'string','白噪声');
set(handles.edit4,'string','20');
set(handles.axes9,'visible','off');
set(handles.axes10,'visible','off');
set(handles.text8,'string','PM(调相)');
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

set(handles.text2,'string','第1路信号频率');
set(handles.text3,'string','第2路信号频率');
set(handles.text5,'string','第3路信号频率');
set(handles.text4,'string','白噪声');

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
set(handles.text8,'string','频分复用');
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
ExitStr = questdlg('请确认是否退出系统？','退出系统','确认','取消','取消');
if strcmp(ExitStr,'确认')
    close;
end
