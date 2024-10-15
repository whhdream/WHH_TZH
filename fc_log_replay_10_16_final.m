clear variables; close all; clc;
%% 10.16版本
% 加入了用dir_out选择是否直接输出
% 其中包括了xyz的观测，速度V的期望与实际比较，姿态期望与实际的比较
%%加入了对四元数的观测(new)
%%加入经纬度
%% 选择ulog

ulog_file = "ffc_data_log.ulg";

START_TIME = 60;%% 开始时间
DURATION = 100; %%持续时间

%% 选择输出模式
dir_out  = 0;  %% 0为按时间截，1为直接输出

%% 加载文件
END_TIME = START_TIME + DURATION; %结束时间
disp("--> 开始读取ulog文件")
tic   %计时器时间
ulog = ulogreader(ulog_file); %存储相应的一些数据的开始时间、结束时间、数据量、无人机名字

% 时间段
if isempty(START_TIME)  %如果没有设定值则以日志开始时间作为开始，注意创建的是（以时分秒为表示的时间量一种新时间表示,但是没有会报错没有用）
    T1 = ulog.StartTime;
else
    T1 = duration(0,0,START_TIME);
end

if isempty(END_TIME)
    T2 = ulog.EndTime;
else
    T2 = duration(0,0,END_TIME);
end


ulog_data = parseAllTopic(ulog, T1,T2); %得到ulog所需时间的数据
disp("--> 读取完成")
toc  %结束计时并输出时间


%% 绘制动画
VideoFrameRate = 10;  %设置帧率
outputVideoFile = 'fc_log_video.mp4'; %设置输出mp4
outputVideo = VideoWriter(outputVideoFile, 'MPEG-4'); %创建mp4对象
outputVideo.FrameRate = VideoFrameRate; %设置该对象帧率
open(outputVideo); %打开这个对象

% 创建一个figure窗口用于绘图
%% 设置图形窗口大小
    screen_size = get(groot, 'ScreenSize');%获取屏幕尺寸
    % 设置图片大小，当然也可以设置为固定值
    fig = figure('Position',screen_size);

%% 两种输出方式
if dir_out
     disp("开始直接输出模式")
     %...............................................................................
     subplot(3,3,1)
     plot3(ulog_data.drone_fusion.position_x, ...
                ulog_data.drone_fusion.position_y, ...
                ulog_data.drone_fusion.position_z, ...
                'r', 'linewidth',2)
     grid on
%    set(gca,'ZLim',[0 0.5]);%设置z轴显示范围
%    set(gca,'ZTick',0:0.1:0.5);%设置z显示坐标刻度
     xlabel("\it x"),ylabel("\it y"),zlabel("\it z"),
     ax = gca;
     ax.DataAspectRatio = [1 1 ax.DataAspectRatio(3)];
     ax.YDir = 'reverse';
     ax.ZDir = 'reverse';
    % ------------------------------------------------------------------------------------
     subplot(3,3,2)
     plot(ulog_data.sensor_gps.lon, ulog_data.sensor_gps.lat,'r', 'linewidth',2)
     grid on
     xlabel("\it lon"),ylabel("\it lat")
     title("经纬度")
    % ....................................................................................
     subplot(3,3,3)
     plot3(ulog_data.sensor_magnet.mag_x,...
                ulog_data.sensor_magnet.mag_y,...
                ulog_data.sensor_magnet.mag_z,'b', 'linewidth',2)
     grid on, axis equal
     title("三轴磁力计")
     % ------------------------------------------------------------------------------------
     subplot(3,3,4)  %VX
     plot(ulog_data.control_horizontal.timestamp, ...
                ulog_data.control_horizontal.vel_ned_sp_x, 'b','linewidth',2)
     hold on
     plot(ulog_data.drone_fusion.timestamp, ...
                ulog_data.drone_fusion.velocity_x, 'r', 'linewidth',2)
     grid on
     h =legend({'\it Vx_{sp}','\it Vx'},'Location','bestoutside');
%      h.Location = 'southoutside';
%      h.Orientation = 'horizontal';   
     set(h,'box','off'); %不显示方框
     title("X轴速度")
     % ------------------------------------------------------------------------------------
     subplot(3,3,5)  %VY
     plot(ulog_data.control_horizontal.timestamp, ...
                ulog_data.control_horizontal.vel_ned_sp_y, 'b','linewidth',2)
     hold on
     plot(ulog_data.drone_fusion.timestamp, ...
                ulog_data.drone_fusion.velocity_y, 'r', 'linewidth',2)
     grid on
     h =legend({'\it Vy_{sp}','\it Vy'},'Location','bestoutside');
     set(h,'box','off'); %不显示方框
     title("Y轴速度")
     % ------------------------------------------------------------------------------------
     subplot(3,3,6)  %VZ
     plot(ulog_data.control_vertical.timestamp, ...
                ulog_data.control_vertical.vel_z_sp, 'b', 'linewidth',2)
     hold on
     plot(ulog_data.drone_fusion.timestamp, ...
                ulog_data.drone_fusion.velocity_z, 'r','linewidth',2)
     grid on
     h =legend({'\it Vz_{sp}','\it Vz'},'Location','bestoutside');
     set(h,'box','off'); %不显示方框
     title("Z轴速度")
     % ------------------------------------------------------------------------------------
     subplot(3,3,7) %roll
     plot(ulog_data.control_attitude.timestamp, ...
                ulog_data.control_attitude.raw_roll_sp, 'g', 'linewidth',2)
     hold on
     plot(ulog_data.drone_attitude.timestamp, ...
                ulog_data.drone_attitude.roll, 'r', 'linewidth',2)
     grid on
     h =legend({'\it roll_{sp}','\it roll'},'Location','bestoutside');
     set(h,'box','off'); %不显示方框
     title("\it roll")
     % ------------------------------------------------------------------------------------
     subplot(3,3,8) %pitch
     plot(ulog_data.control_attitude.timestamp, ...
                ulog_data.control_attitude.raw_pitch_sp, 'g', 'linewidth',2)
     hold on
     plot(ulog_data.drone_attitude.timestamp, ...
                ulog_data.drone_attitude.pitch, 'r', 'linewidth',2)
     grid on
     h =legend({'\it pitch_{sp}','\it pitch'},'Location','bestoutside');
     set(h,'box','off'); %不显示方框
     title("\it pitch")
     % ------------------------------------------------------------------------------------
     subplot(3,3,9) %yaw
     plot(ulog_data.control_attitude.timestamp, ...
                ulog_data.control_attitude.raw_yaw_sp, 'g', 'linewidth',2)
     hold on
     plot(ulog_data.drone_attitude.timestamp, ...
                ulog_data.drone_attitude.yaw, 'r','linewidth',2 )
     grid on
     h =legend({'\it yaw_{sp}','\it yaw'},'Location','bestoutside');
     set(h,'box','off'); %不显示方框
     title("\it yaw")        
else
    disp("开始逐点输出模式")
    T1 = seconds(T1);
    T2 = seconds(T2);
    dt = 1/VideoFrameRate;%采样时间
    
    for time = T1:dt:T2
       % tic
    % ------------------------------------------------------------------------------------
        subplot(3,4,1)
        idx = find(ulog_data.drone_fusion.timestamp >= time, 1, 'first');
        if (~isempty(idx))  %找不到就不干了
            plot3(ulog_data.drone_fusion.position_x(idx), ...
                ulog_data.drone_fusion.position_y(idx), ...
                ulog_data.drone_fusion.position_z(idx), ...
                'r.', MarkerSize=8)
            hold on, grid on
            xlabel("\it x"),ylabel("\it y"),zlabel("\it z"),
            ax = gca;
            ax.DataAspectRatio = [1 1 ax.DataAspectRatio(3)];
            ax.YDir = 'reverse';
            ax.ZDir = 'reverse';
            %title(sprintf(' t_{system}: %.2f sec', time));
        end
        title(['$t_{\it{system}}$: ' num2str(time, '%.2f') ' sec'], 'Interpreter', 'latex');%优化原来字体
        %且放在if下可能导致显示丢失放出来 
    % ------------------------------------------------------------------------------------
        subplot(3,4,2)
        %期望的值
        idx1 = find(ulog_data.control_attitude.timestamp >= time, 1, 'first');
        if (~isempty(idx1))
            Qd = quaternion([ulog_data.control_attitude.raw_yaw_sp(idx1), ...
                                 ulog_data.control_attitude.raw_pitch_sp(idx1), ...
                                 ulog_data.control_attitude.raw_roll_sp(idx1)],...
                                 'eulerD','ZYX','frame');
            plot_qua(Qd,1); 
        end
        idx = find(ulog_data.drone_fusion.timestamp >= time, 1, 'first');
        if (~isempty(idx))
             Q = quaternion(ulog_data.drone_fusion.quaternion_w(idx), ...
                 ulog_data.drone_fusion.quaternion_x(idx), ...
                 ulog_data.drone_fusion.quaternion_y(idx), ...
                 ulog_data.drone_fusion.quaternion_z(idx) );
             plot_qua(Q,0);  
        end
        grid on
        hold off
        title("四元数")
    % ------------------------------------------------------------------------------------
        subplot(3,4,3)
        idx = find(ulog_data.sensor_magnet.timestamp >= time, 1, 'first');
        if (~isempty(idx))
            plot3(ulog_data.sensor_magnet.mag_x(idx),...
                ulog_data.sensor_magnet.mag_y(idx),...
                ulog_data.sensor_magnet.mag_z(idx),'r.', MarkerSize=8)
            hold on, grid on, axis equal
        end
        title("三轴磁力计")
    % ------------------------------------------------------------------------------------
        subplot(3,4,4)  %VX
        idx =  find(ulog_data.drone_fusion.timestamp >= time, 1, 'first');
        %idx1 = find(ulog_data.control_vertical.timestamp >= time, 1, 'first');%做高度的判断
        idx1 = find(ulog_data.control_horizontal.timestamp >= time, 1, 'first');%做水平的判断
        if ((~isempty(idx)) && (~isempty(idx1)))
            plot(ulog_data.drone_fusion.timestamp(idx), ...
                ulog_data.drone_fusion.velocity_x(idx), 'r.', MarkerSize=8)
            hold on
            plot(ulog_data.control_horizontal.timestamp(idx1), ...
                ulog_data.control_horizontal.vel_ned_sp_x(idx1), 'b.', MarkerSize=8)
            grid on
            h =legend({'\it Vx_{sp}','\it Vx'});
            h.Location = 'southoutside';
            h.Orientation = 'horizontal'; 
            set(h,'box','off'); %不显示方框
            title("X轴速度")
        end
     % ------------------------------------------------------------------------------------
         subplot(3,4,5)  %VY
        idx =  find(ulog_data.drone_fusion.timestamp >= time, 1, 'first');
        %idx1 = find(ulog_data.control_vertical.timestamp >= time, 1, 'first');%做高度的判断
        idx1 = find(ulog_data.control_horizontal.timestamp >= time, 1, 'first');%做水平的判断
        if ((~isempty(idx)) && (~isempty(idx1)))
            plot(ulog_data.drone_fusion.timestamp(idx), ...
                ulog_data.drone_fusion.velocity_y(idx), 'r.', MarkerSize=8)
            hold on
            plot(ulog_data.control_horizontal.timestamp(idx1), ...
                ulog_data.control_horizontal.vel_ned_sp_y(idx1), 'b.', MarkerSize=8)
            grid on
            h =legend({'\it Vy','\it Vy_{sp}'});
            h.Location = 'southoutside';
            h.Orientation = 'horizontal'; 
            set(h,'box','off'); %不显示方框
            title("Y轴速度")
        end
     % ------------------------------------------------------------------------------------
        subplot(3,4,6)  %VZ
        idx =  find(ulog_data.drone_fusion.timestamp >= time, 1, 'first');
        idx1 = find(ulog_data.control_vertical.timestamp >= time, 1, 'first');%做高度的判断
        %idx1 = find(ulog_data.control_horizontal.timestamp >= time, 1, 'first');%做水平的判断
        if ((~isempty(idx)) && (~isempty(idx1)))
            plot(ulog_data.drone_fusion.timestamp(idx), ...
                ulog_data.drone_fusion.velocity_z(idx), 'r.', MarkerSize=8)
            hold on
            plot(ulog_data.control_vertical.timestamp(idx1), ...
                ulog_data.control_vertical.vel_z_sp(idx1), 'b.', MarkerSize=8)
            grid on
            h =legend({'\it Vz','\it Vz_{sp}'});
            h.Location = 'southoutside';
            h.Orientation = 'horizontal'; 
            set(h,'box','off'); %不显示方框
            title("Z轴速度")
        end 
     % ------------------------------------------------------------------------------------
        subplot(3,4,7) %roll
        idx = find(ulog_data.drone_attitude.timestamp >= time, 1, 'first');
        idx1 = find(ulog_data.control_attitude.timestamp >= time, 1, 'first');%姿态期望值
        if ((~isempty(idx)) && (~isempty(idx1)) )
            plot(ulog_data.drone_attitude.timestamp(idx), ...
                ulog_data.drone_attitude.roll(idx), 'r.', MarkerSize=8)
            hold on
            plot(ulog_data.control_attitude.timestamp(idx1), ...
                ulog_data.control_attitude.raw_roll_sp(idx1), 'g.', MarkerSize=8)
            grid on
            h =legend({'\it roll','\it roll_{sp}'});
            h.Location = 'southoutside';
            h.Orientation = 'horizontal'; 
            set(h,'box','off'); %不显示方框
            title("\it roll")
        end
     % ------------------------------------------------------------------------------------
        subplot(3,4,8) %pitch
        idx = find(ulog_data.drone_attitude.timestamp >= time, 1, 'first');
        idx1 = find(ulog_data.control_attitude.timestamp >= time, 1, 'first');%姿态期望值
        if ((~isempty(idx)) && (~isempty(idx1)) )
            plot(ulog_data.drone_attitude.timestamp(idx), ...
                ulog_data.drone_attitude.pitch(idx), 'r.', MarkerSize=8)
            hold on
            plot(ulog_data.control_attitude.timestamp(idx1), ...
                ulog_data.control_attitude.raw_pitch_sp(idx1), 'g.', MarkerSize=8)
            grid on
            h =legend({'\it pitch','\it pitch_{sp}'});
            h.Location = 'southoutside';
            h.Orientation = 'horizontal'; 
            set(h,'box','off'); %不显示方框
            title("\it pitch")
        end
     % ------------------------------------------------------------------------------------
        subplot(3,4,9) %yaw
        idx = find(ulog_data.drone_attitude.timestamp >= time, 1, 'first');
        idx1 = find(ulog_data.control_attitude.timestamp >= time, 1, 'first');%姿态期望值
        if ((~isempty(idx)) && (~isempty(idx1)) )
            plot(ulog_data.drone_attitude.timestamp(idx), ...
                ulog_data.drone_attitude.yaw(idx), 'r.', MarkerSize=8)
            hold on
            plot(ulog_data.control_attitude.timestamp(idx1), ...
                ulog_data.control_attitude.raw_yaw_sp(idx1), 'g.', MarkerSize=8)
            grid on
            h =legend({'\it yaw','\it yaw_{sp}'});
            h.Location = 'southoutside';
            h.Orientation = 'horizontal'; 
            set(h,'box','off'); %不显示方框
            title("\it yaw")
        end
    %....................................................................................
        subplot(3,4,10)
        idx = find(ulog_data.sensor_gps.timestamp >= time, 1, 'first');
        if (~isempty(idx))
            plot(ulog_data.sensor_gps.lon(idx), ulog_data.sensor_gps.lat(idx),'r.', 'linewidth',2)
        hold on
        grid on
        xlabel("\it lon"),ylabel("\it lat")
        title("经纬度")
        end


    % ------------------------------------------------------------------------------------
        % 获取当前帧的图像数据
        frameWithPlot = getframe(fig);
        % 写入新的视频帧
        writeVideo(outputVideo, frameWithPlot);
       %toc
    end
end
% 关闭视频写入对象
close(outputVideo);
% close(fig)
% 姿态
