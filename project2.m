%Author: Tanit Pan-anuruk, Z5224642

clc(); close all;

% Create global variables struct
global ABCD;            % I use a global variable, to easily share it, in many functions. You may obtain similar functionality using other ways.
ABCD=[]; ABCD.flagPause=0; ABCD.exit=0;

global landmark;
landmark.coor = [];
landmark.id = [];
landmark.detected = 0;
landmark.DAcoor = [];
landmark.localOOIs = [];

% Setting up loading data
% In case the caller does not specify the input argument, we propose a
% default one, assumed to be in the same folder where we run this example from.
if ~exist('file','var'), file ='data_project1/Laser__2.mat'; end;
if ~exist('file2','var'), file2 ='data_project1/IMU_dataC.mat'; end;
if ~exist('file3','var'), file3 ='data_project1/Speed_dataC.mat'; end;

load(file);
load(file2);
load(file3);
% now, after loading, the data is in a variable named "dataL";
% --------------------------------------
% Create graphical object for refreshing data during program execution.
% figure(1) ; clf();
% global MyGUIHandles;
% MyGUIHandles.handle1 = plot(0,0,'b.');      % to be used for showing the laser points
% hold on;
% MyGUIHandles.handle3 = plot(0,0,'g+','MarkerSize',15,'LineWidth',3);
% MyGUIHandles.handle4 = plot(0,0,'r*','MarkerSize',5,'LineWidth',1);
% hold off;
% 
% 
% axis([-10,10,0,20]);
% %     axis([0,180,0,20]);                         % focuses the plot on this region (of interest, close to the robot)
% xlabel('angle (degrees)');
% ylabel('range (meters)');
% 
% MyGUIHandles.handle2 = title('');           % create an empty title..
% zoom on ;  grid on;
% 
% figure(2); clf();
% %     MyGUIHandles.handle5 = title('');
% axis([0   45000 -600 100]);
% %     axis([0 250 -15 15])
% %     MyGUIHandles.handle5 = plot(0,0,'b.');
% 
% %     MyGUIHandles.handle3 = plot(1,1,'r.');
% % If you do not understand these functions ( figure() ,plot(),
% % axis(),.....) ===> Read Matlab's Help.
% % (In MTRN2500 you used them)
% 
% %---------------------------------
% 
% disp('Showing laser scans, in POLAR representation');
% disp('Yo need to modify this program for showing the data in Cartesian.');
% disp('(then the images will make sense, for our brains)');
% 
% fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,file);
% 
% figure(1);
% 

% Now, loop through the avaialable scans..

N = dataL.N;                        %number of scans in this squence.

figure('visible','on');
% figure(6);
clf();
hold on;
axis([-5 5 0 10]);                %focuses plot on this region ( of interest in L220)
xlabel('x (meters)');
ylabel('y (meters)');
global MyGUIHandles;
MyGUIHandles.handle5 = title('');
MyGUIHandles.handle6 = plot(0,0,'DisplayName','Landmark Estimates(EKF)');       %handle for reflective OOIs
% MyGUIHandles.handle7 = plot(0,0);       %handle for non-reflective OOIs
MyGUIHandles.handle8 = plot(0,0,'DisplayName','DR Pose');       %handle for DR
MyGUIHandles.real = plot(0,0);

% Landmark and currently detected OOIs handles
landmark.handle = plot(0,0,'linestyle','none','DisplayName','Landmark(MAP)');
landmark.text = text(zeros(1,5),zeros(1,5),'');
landmark.DA = text(zeros(1,5),zeros(1,5),'');


% Create robot body
global robot;
robot.body = plot(0,0);
robot.heading = plot(0,0);
robot.trace = plot(0,0);
robot.traceData = [];
legend([MyGUIHandles.handle6,MyGUIHandles.handle8,landmark.handle]);
zoom on; grid on;
uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});

%%

% Assume noises condtion as given
stdDevGyro = 1.5*pi/180; % degree/s
stdDevSpeed = 0.4; % m/s
sdev_rangeMeasurement = 0.2; % m
sdev_bearingMeasurement = 1.5*pi/180; % degree/s

% Matrices
P = zeros(3,3); 
Pu = diag([stdDevSpeed^2,stdDevGyro^2]);
Q_state = diag( [ (0.01)^2 ,(0.01)^2 , (1*pi/180)^2]) ;

% Removing bias
time_corr = double(IMU.times - IMU.times(1))/10000;
Laser_time_corr = double(dataL.times - dataL.times(1))/10000;
yaw = IMU.DATAf(6,:);

start = length(find(time_corr < 20)); % 20s of stationary
noise = mean(IMU.DATAf(6,1:start)); % Bias

yaw_corr = yaw - noise; % corrected wk
max_length = length(yaw_corr);

% Buffer for storage of intermediate values
Xdr = [ 0; 0;pi/2 ];
Xe = [ 0; 0;pi/2 ];
XeHistory= zeros(3,max_length) ;
XdrHistory= zeros(3,max_length) ;

L_time = length(Laser_time_corr);
curr_scan = 1;

for i = 2:length(time_corr)
    dt = time_corr(i) - time_corr(i-1); % find dt
    speed = Vel.speeds(i);
    imu_gyro = yaw_corr(i);
    Xdr = processModel(Xdr,speed,imu_gyro,dt);
    XdrHistory(:,i) = Xdr;
end

% Obtaining robot pose(position& heading) at time scan i
for i = 2:length(time_corr)
    dt = time_corr(i) - time_corr(i-1);
    speed = Vel.speeds(i);
    imu_gyro = yaw_corr(i);
    OOIs_detected = 0;
    
    Xdr = processModel(Xdr,speed,imu_gyro,dt);
    % Check availability of laser data. If no data,skip processing of scan.
    if(curr_scan <= L_time && Laser_time_corr(curr_scan) - time_corr(i-1) < dt)
        % Obtaining X,Y to process scan
        dtL = Laser_time_corr(curr_scan)- time_corr(i-1);
        
        X_temp = processModel(Xe,speed,imu_gyro,dtL);
        Local_OOIs = MyProcessingOfScan(dataL.Scans(:,curr_scan));
        Global_OOIs = ToGlobalCoordinateFrame(Local_OOIs,X_temp(1),X_temp(2),X_temp(3)); % convert OOIs to global coor
        PlotOOIs(Global_OOIs); % Plot only the estimated OOIs location
        IdentifyAssociation(Global_OOIs,Local_OOIs); % Perform identification(LandMark) and Data Association
        OOIs_detected = landmark.detected;
        curr_scan = curr_scan + 1;
    end

    J = [ [1,0,-dt*speed*sin(Xe(3))  ]  ; [0,1,dt*speed*cos(Xe(3))] ;    [ 0,0,1 ] ] ; % 3x3 Jacobian of process model
    Fu = [dt*cos(Xe(3)), 0;         
          dt*sin(Xe(3)), 0;
          0, dt];               % 3x2 Jacobian of our input to process model
   
    Q = Fu*Pu*Fu' + Q_state;
    P = J*P*J'+ Q ;
    Xe = processModel(Xe,speed,imu_gyro,dt);
    
    if(OOIs_detected > 0)
        for j=1:OOIs_detected
            ID = landmark.DAcoor(3,j);
            d = 0.46;
            
            eDX = (landmark.coor(1,ID)-Xe(1)-d*cos(Xe(3))) ;      % (xu-x)
            eDY = (landmark.coor(2,ID)-Xe(2)-d*sin(Xe(3))) ;      % (yu-y)
            eDD = sqrt( eDX*eDX + eDY*eDY ) ; %   so : sqrt( (xu-x)^2+(yu-y)^2 )
            H = [  [-eDX/eDD , -eDY/eDD , 0]; [eDY/(eDD*eDD), -eDX/(eDD*eDD),-1] ]; % Jacobian of h(X); size 1x3
            
            % The "Expected output" h(Xe)
            ExpectedRange = eDD;
            ExpectedBearing = atan2(eDY,eDX)-Xe(3)+pi/2;
           
            MX = (landmark.localOOIs(1,j));
            MY = (landmark.localOOIs(2,j));
            MD = sqrt(MX*MX + MY*MY);
            MeasuredRange = MD;
            MeasuredBearing = atan2(MY,MX);
            
            % Evaluate residual (innovation)  "Y-h(Xe)" 
            %(measured output value - expected output value)
            z  = [MeasuredRange - ExpectedRange;
                  wrapToPi(MeasuredBearing-ExpectedBearing)] ;
            % ------ covariance of the noise/uncetainty in the measurements
            R = [sdev_rangeMeasurement*sdev_rangeMeasurement*4, 0; 0,sdev_bearingMeasurement*sdev_bearingMeasurement*4];
            % Multiply by 4 because I want to be conservative and assume
            % twice the standard deviation that I believe does happen.
            S = R + H*P*H' ;
            iS=inv(S);                 % iS = inv(S) ;   % in this case S is 1x1 so inv(S) is just 1/S
            K = P*H'*iS ;           % Kalman gain
%             K = P*H'/S ;
            % ----- finally, we do it...We obtain  X(k+1|k+1) and P(k+1|k+1)
            
            Xe = Xe+K*z ;       % update the  expected value
            P = P-K*H*P ;       % update the Covariance % i.e. "P = P-P*H'*iS*H*P"  )
            % -----  individual EKF update done ...
        end
    end
    % History
    XeHistory(:,i) = Xe;
    if ABCD.exit==1, clf(); close all; clear; break; end;
    while (ABCD.flagPause == 1) 
        pause(0.15);
        if ABCD.exit==1, clf(); close all; clear; break; end;
    end
    s = sprintf('Showing scan #[%d]/[%d]\r',i,max_length);
    set(MyGUIHandles.handle5,'string',s);
    set(MyGUIHandles.handle8,'xdata',XdrHistory(1,1:i),'ydata',XdrHistory(2,1:i),'LineStyle','none','marker','.');
    plotRobot(Xe(1),Xe(2),Xe(3));
    pause(0.03);
end

function Xnew = processModel(Xprior,speed,w,dt)
    Xnew = zeros(3,1);
    Xnew = Xprior + dt*[speed*cos(Xprior(3)); speed*sin(Xprior(3)); w];
end

function OOIs = MyProcessingOfScan(scan)
    % I made this function, to receive the following parameters/variables:
    % 'scan' : scan measurements to be shown.
    % 't':  associated time.
    % 'i' : scan number.
    % 'mh'  : struct contaning handles of necessary graphical objects.

    angles = [0:360]'*0.5*pi/180 ;         % Associated angle for each range of scan
    % same as in "dataL.angles".
    %     angles = [0:360]'*0.5* pi/180 ;         % associated angle, for each individual range in a scan


    % scan data is provided as a array of class uint16, which encodes range
    % and intensity (that is the way the sensor provides the data, in that
    % mode of operation)


    MaskLow13Bits = uint16(2^13-1); % mask for extracting the range bits.
    % the lower 13 bits are for indicating the range data (as provided by this sensor)
    MaskHighE000 = bitshift(uint16(7),13)  ;

    intensities = bitshift(bitand(scan,MaskHighE000),-13);

    rangesA = bitand(scan,MaskLow13Bits) ;
    % rangesA now contains the range data of the scan, expressed in CM, having uint16 format.

    % now I convert ranges to meters, and also to floating point format
    ranges    = 0.01*double(rangesA);
    X = cos(angles).*ranges;
    Y = sin(angles).*ranges;



    OOIdata = [X,Y,single(intensities)];

    OOIs = ExtractOOIs(OOIdata);
    pause(0.01); % Pause to allow enough refresh time
    return;
end

function r = ExtractOOIs(OOIdata)
    threshold = 0.15;
    cluster_num = 1;

    distance = [0; sqrt(diff(OOIdata(:,1)).^2 + diff(OOIdata(:,2)).^2)];
    cluster_vector = zeros(length(distance),1);

    for i = 1:length(distance)

        if (distance(i) >= threshold)
            cluster_num = cluster_num + 1;
        end

        cluster_vector(i) = cluster_num;
    end

    r.N = cluster_num;
    r.Centers = zeros(2,r.N);
    r.Diameters = zeros(1,r.N);
    r.Colors = zeros(1,r.N);

    % Filling r struct using circfit function from Izhak bucher 25/oct /1991
    % https://au.mathworks.com/matlabcentral/fileexchange/5557-circle-fit

    for i = 1:r.N
        cluster_i = OOIdata(cluster_vector == i,:);

        % might need to implement check_size

        [xc,yc,R] = circfit(cluster_i(:,1),cluster_i(:,2));
        r.Centers(:,i) = [xc; yc];
        r.Diameters(i) = R*2;
        r.Colors(i) = max(cluster_i(:,3)) > 0;

    end


    Filter = r.Diameters < 0.03 | r.Diameters > 0.25;
    r.Centers(:,Filter) = [];
    r.Diameters(Filter) = [];
    r.Colors(Filter) = [];
    r.N = length(r.Diameters);

end


function PlotOOIs(OOIs)
    global MyGUIHandles;
if OOIs.N<1, return ; end;
    MyGUIHandles.handle6.LineStyle = 'none';
    MyGUIHandles.handle6.LineWidth = 2;
%     MyGUIHandles.handle7.LineStyle = 'none';
%     MyGUIHandles.handle7.LineWidth = 1.5;
    set(MyGUIHandles.handle6,'xdata',OOIs.Centers(1,OOIs.Colors>0),'ydata',OOIs.Centers(2,OOIs.Colors>0),'color','g','marker','*','markersize',10);
    
    return;
end

function IdentifyAssociation(OOI,localOOI)
global landmark;
OOIglobal = OOI.Centers(:,OOI.Colors>0);
OOIlocal = localOOI.Centers(:,localOOI.Colors>0);

% %negate Xaxis in local OOI
%     OOIlocal(1,:) = -OOIlocal(1,:);
[~,n] = size(OOIglobal);
landmark.detected = 0;
measured=[];
DA = []; % data association
if isempty(landmark.coor)
    % add all ooi and give unique id
    landmark.coor = OOIglobal;
    landmark.id = 1:length(landmark.coor);
%     landmark.text = text(landmark.coor(1,:)+0.1,landmark.coor(2,:)-0.1,string(landmark.id));
end
for i = 1:n
    for j = landmark.id
        distance = norm(OOIglobal(:,i)-landmark.coor(:,j));
        if (distance <= 1)
            temp = [OOIglobal(:,i);j];
            DA = [DA,temp];
            temp = [OOIlocal(:,i)];
            measured = [measured,temp];
        end
    end
    landmark.DAcoor = DA;
    landmark.localOOIs = measured;
    if ~isempty(DA)
        landmark.detected = length(DA(3,:));
    end
end

set(landmark.handle,'xdata',landmark.coor(1,:),'ydata',landmark.coor(2,:),'color','k','marker','+','markersize',10);

for i = 1:5
    set(landmark.DA(i),'String','');
    if ~isempty(DA)
        if i<=length(DA(3,:))
            set(landmark.DA(i),'Position',DA(1:2,i),'String',string(DA(3,i)));
        end
    end
end
end

function OOIs = ToGlobalCoordinateFrame(OOIs,x,y,theta)

d = 0.46; % from project specs
% negate x-coordinate
OOIs.Centers(1,:) = OOIs.Centers(1,:);
% adding d
OOIs.Centers(2,:) = d + OOIs.Centers(2,:);
a = theta - pi/2;
R = [ cos(a), -sin(a);
    sin(a), cos(a)];
%OOI
OOIs.Centers = R * OOIs.Centers + [x;y];

end

function plotRobot(x,y,theta)
global robot;
robot.traceData = [robot.traceData,[x;y]];
set(robot.body,'xdata',x,'ydata',y,'markersize',5,'marker','diamond');
R = [ cos(theta), -sin(theta);
    sin(theta), cos(theta)];
coor = [0 0.2 0.4 0.8 1; 0 0 0 0 0];
coor = R * coor + [x;y];
set(robot.heading,'xdata',coor(1,:),'ydata',coor(2,:),'markersize',2,'color','r');
set(robot.trace,'xdata',robot.traceData(1,:),'ydata',robot.traceData(2,:),'color','b');
end



% Callback function. I defined it, and associated it to certain GUI button,
function MyCallBackA(~,~,x)
global ABCD;

if (x==1)
    ABCD.flagPause = ~ABCD.flagPause; %Switch ON->OFF->ON -> and so on.
    return;
end;
if (x==2)
    
    disp('you pressed "END NOW"');
    ABCD.exit = 1;
    %          clf();
    %          close all
    %          clear
    %         uiwait(msgbox('Ooops, you still need to implement this command!','?','modal'));
    
    % students complete this.
    return;
end;
return;
end




