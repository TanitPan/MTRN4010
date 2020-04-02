%Author: Tanit Pan-anuruk, Z5224642

%Program: Solution for AAS, T1.2020, Project1.part 1....

function project1(file)

    global ABCD;            % I use a global variable, to easily share it, in many functions. You may obtain similar functionality using other ways.
    ABCD=[]; ABCD.flagPause=0; ABCD.exit=0;

    global landmark;

    landmark.coor = [];
    landmark.id = [];
    landmark.detected = 0;
    landmark.DAcoor = [];

    % In case the caller does not specify the input argument, we propose a
    % default one, assumed to be in the same folder where we run this example from.
    if ~exist('file','var'), file ='Laser__2.mat'; end;
    if ~exist('file2','var'), file2 ='IMU_dataC.mat'; end;
    if ~exist('file3','var'), file3 ='Speed_dataC.mat'; end;

    load(file); 
    load(file2);
    load(file3);
    % now, after loading, the data is in a variable named "dataL";



    % --------------------------------------
    % Create graphical object for refreshing data during program execution.
    figure(1) ; clf(); 

    MyGUIHandles.handle1 = plot(0,0,'b.');      % to be used for showing the laser points
    hold on;
    MyGUIHandles.handle3 = plot(0,0,'g+','MarkerSize',15,'LineWidth',3);
    MyGUIHandles.handle4 = plot(0,0,'r*','MarkerSize',5,'LineWidth',1);
    hold off;


    axis([-10,10,0,20]); 
%     axis([0,180,0,20]);                         % focuses the plot on this region (of interest, close to the robot)
    xlabel('angle (degrees)');
    ylabel('range (meters)');
    
    MyGUIHandles.handle2 = title('');           % create an empty title..
    zoom on ;  grid on;
    
    figure(2); clf();
%     MyGUIHandles.handle5 = title('');
    axis([0   45000 -600 100]);
%     axis([0 250 -15 15])
%     MyGUIHandles.handle5 = plot(0,0,'b.'); 
    
%     MyGUIHandles.handle3 = plot(1,1,'r.'); 
    % If you do not understand these functions ( figure() ,plot(),
    % axis(),.....) ===> Read Matlab's Help.
    % (In MTRN2500 you used them)
    
    %---------------------------------
   
    disp('Showing laser scans, in POLAR representation');
    disp('Yo need to modify this program for showing the data in Cartesian.');
    disp('(then the images will make sense, for our brains)');
    
    fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,file);
    
    figure(1);
    
    uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
    uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});


    
% Now, loop through the avaialable scans..
% dt = double(IMU.times(2) - IMU.times(1))/10000;

    [x,y,theta] = GetPose(IMU,dataL,Vel);

    N = dataL.N;                        %number of scans in this squence.

    figure('visible','on');
    % figure(6);
    clf();
    hold on;
    axis([-5 5 0 10]);                %focuses plot on this region ( of interest in L220)
    xlabel('x (meters)');
    ylabel('y (meters)');
    MyGUIHandles.handle5 = title('');
    MyGUIHandles.handle6 = plot(0,0);       %handle for reflective OOIs
    MyGUIHandles.handle7 = plot(0,0);       %handle for non-reflective OOIs

    landmark.handle = plot(0,0,'linestyle','none');
    landmark.text = text(0,0,'');
    landmark.DA = text(zeros(1,5),zeros(1,5),'');

    % Create robot body
    global robot;
    robot.body = plot(0,0);
    robot.heading = plot(0,0);
    robot.trace = plot(0,0);
    robot.traceData = [];
    zoom on; grid on;

    %%
    % N = dataL.N; 
    skip=1;     % in this example I skip some of the laser scans.
    i=1;


    while 1,             

        if (ABCD.flagPause), pause(0.2) ; continue ; end;
        if i>N, break ;  end;
        if ABCD.exit==1, clf(); close all; clear; break; end;


        % Native time expressed via uint32 numbers, where 1 unint means
        % [1/10,000]second (i.e. 0.1 millisecond)
        % (BTW: we do not use the time, in this task)
        t =  double(dataL.times(i)-dataL.times(1))/10000;
        % t: time expressed in seconds, relative to the time of the first scan.



        scan_i = dataL.Scans(:,i);
        MyProcessingOfScan(scan_i,t,MyGUIHandles,i,x(i),y(i),theta(i));   % some function to use the data...

        s=sprintf('Showing scan #[%d]/[%d]\r',i,N);
        set(MyGUIHandles.handle5,'string',s);
        plotRobot(x(i),y(i),theta(i));

        pause(0.01) ;                   % wait for ~10ms (approx.)
        i=i+skip;
    end;

    fprintf('\nDONE BYE!\n');

    % Read Matlab Help for explanation of FOR loops, and function double( ) and pause()


    return;
end
%-----------------------------------------
function MyProcessingOfScan(scan,t,mh,i,x,y,theta)
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
    PlotOOIs(OOIs,OOIdata,mh,i,t);
    OOIs = ToGlobalCoordinateFrame(OOIs,x,y,theta); % convert OOIs to global coor
    PlotOOIsGlobal(OOIs,OOIdata,mh,i,t);
    IdentifyOOIs(OOIs); % identify landmark && data association
    
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


function PlotOOIs(OOIs,OOIdata,mh,i,t)
    if OOIs.N<1, return ; end;
    % your part....
    s= sprintf('Laser scan # [%d] at time [%.3f] secs',i,t);
    set(mh.handle2,'string',s);
    
    set(mh.handle1,'xdata',OOIdata(:,1),'ydata',OOIdata(:,2));
    set(mh.handle3,'xdata',OOIs.Centers(1,OOIs.Colors>0),'ydata',OOIs.Centers(2,OOIs.Colors>0));
    set(mh.handle4,'xdata',OOIdata(OOIdata(:,3) > 0,1),'ydata',OOIdata(OOIdata(:,3) > 0,2));
    
return;
end


function PlotOOIsGlobal(OOIs,OOIdata,mh,i,t)
    if OOIs.N<1, return ; end;
    mh.handle6.LineStyle = 'none';
    mh.handle6.LineWidth = 2;
    mh.handle7.LineStyle = 'none';
    mh.handle7.LineWidth = 1.5;
    set(mh.handle6,'xdata',OOIs.Centers(1,OOIs.Colors>0),'ydata',OOIs.Centers(2,OOIs.Colors>0),'color','g','marker','*','markersize',10);
    %set(myHandle.handle5,'xdata',OOIs.Centers(1,OOIs.Color==0),'ydata',OOIs.Centers(2,OOIs.Color==0),'color','k','marker','+','markersize',10);
return;
end


% ---------------------------------------
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

function [xL,yL,thetaL] = GetPose(IMU, dataL, Vel)
    % removing bias
     time_corr = double(IMU.times - IMU.times(1))/10000;
     Laser_time_corr = double(dataL.times - dataL.times(1))/10000;
     yaw = IMU.DATAf(6,:);

     start = length(find(time_corr < 20)); % 20s of stationary
     noise = mean(IMU.DATAf(6,1:start)); % Bias

     yaw_corr = yaw - noise; % corrected wk
     N = length(yaw_corr);

     CurrAlt = zeros(3,44578);
    CurrAlt(:,1) = [0;0;pi/2];
    % CurrAlt(:,2) = [0;0;pi/2];

    % s = sprintf('Yaw Rate Integrated');
     corr_data = IMU.DATAf(4:6,:)-IMU.DATAf(4:6,1);
     corr_data(3,:)= yaw_corr;

   
    speed = Vel.speeds';
     speed = speed*-1;
    wz = corr_data(3,:);

    Pose = zeros(3,N);
    Pose(:,1) = [0;0;pi/2];
    X0 = [0;0;pi/2];
    X= X0;



    PoseL = zeros(3,length(Laser_time_corr));
    PoseL(:,1) = [0;0;pi/2];
    XL0 = [0;0;pi/2];
    XL = XL0;

% obtain the robot pose (position and heading) at time scan[i]
    j=1;
    for ii= 1:N-1
        dt = time_corr(ii+1) - time_corr(ii);
        X = predictVehiclePose(X,wz(ii),speed(ii),dt); % euler approx
        Pose(:,ii+1) = X;

         % handling laser scan and position frequency difference
          if (j < length(Laser_time_corr) && Laser_time_corr(j) -  time_corr(ii)< dt)
                dtL = Laser_time_corr(j+1) -  time_corr(ii);
                XL = predictVehiclePose(XL,wz(ii),speed(ii),dtL); % euler approx
                PoseL(:,j+1) = XL;
                j = j + 1;
          end

    end
%     Pose(2,:) = -Pose(2,:);
%     PoseL(2,:) = -PoseL(2,:);

    xL = PoseL(1,:);
    yL = PoseL(2,:);
    thetaL = PoseL(3,:);

    % change to degree and store value in data buffer
    theta = Pose(3,:)*180/pi;
    x = Pose(1,:);
    y = Pose(2,:);
    figure(2);
    hold on;
    plot(1:44578,theta)
    title('Yaw Rate Integrated')
    grid on;
    xlabel('Sample');
    ylabel('heading(degrees)');
    % Plotting graph  
    figure(3)
    wz_new = corr_data(3,:)*180/pi;
    plot(time_corr,wz_new);
    title('Yaw Rate');
    xlabel('time (s)');
    ylabel('Wz (deg/s)');
    grid on;

     figure(4)
     plot(time_corr,Vel.speeds);
     title('Longitudinal Velocity');
     xlabel('time (s)');
     ylabel('Speed (m/s)');
     grid on;
    
    figure(5); clf();
    plot(Pose(1,:),Pose(2,:));
    title('Position')
    ylabel('Y (m)')
    xlabel('X (m)')
    grid on;
 
    return;

end


function X = predictVehiclePose(X0,wz,speed,dt)
    X = X0;
    X(1:2) = X0(1:2) + dt*speed*[cos(X0(3)); -sin(X0(3))];
    X(3) = X0(3) + dt*wz;
    return;
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

function IdentifyOOIs(r)
    global landmark;
    OOIarray = r.Centers(:,r.Colors>0);
    [~,n] = size(OOIarray);
    landmark.detected = n;
    DA = []; %data association
    if isempty(landmark.coor)
        % add all ooi and give unique id
        landmark.coor = OOIarray;
        landmark.id = 1:length(landmark.coor);
        landmark.text = text(landmark.coor(1,:)+0.1,landmark.coor(2,:)-0.1,string(landmark.id));
    end
    for i = 1:n 
        for j = landmark.id
            distance = norm(OOIarray(:,i)-landmark.coor(:,j));
            if (distance <= 0.4)
                temp = [OOIarray(:,i);j];
                DA = [DA,temp];
            end
        end
        landmark.DAcoor = DA;
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