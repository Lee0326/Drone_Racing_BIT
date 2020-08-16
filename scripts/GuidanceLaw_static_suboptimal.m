clear all;    	  %清除所有内存变量
%--------初始化制导系统参数------------------
pi=3.141592653;
g=9.8;
Rmx=0;Rmy=2;                                 %无人机的位置
Rtx=3; Rty=3;                                  %门的位置
Vm=3;                                          %无人机的速度
HeadError=70/180*pi;                           %指向角误差

AtMay=0*g;                                     %目标不机动
AmMay=5*g;                                     %无人机的最大机动能力为5G

M_= 10;
N_= 20;
SightAngle_desired = 0/180*pi;

%弹目几何运动学解算
Rtmx=Rtx-Rmx;                                  %无人机和门x轴相对距离
Rtmy=Rty-Rmy;                                  %无人机和门y轴相对距离
Rtm=sqrt(Rtmx^2+Rtmy^2);                       %无人机和门相对距离
SightAngle=atan(Rtmy/Rtmx);                    %视线角
PathAngle=SightAngle+HeadError;
Vmx=Vm*cos(SightAngle+HeadError);              %无人机的x轴速度分量
Vmy=Vm*sin(SightAngle+HeadError);              %无人机的y轴速度分量
Vtmx=-Vmx;                                     %无人机和门相对运动的x轴速度分量
Vtmy=-Vmy;                                     %无人机和门相对运动的y轴速度分量
Vc=-(Rtmx*Vtmx+Rtmy*Vtmy)/Rtm;                 %无人机和门相对运动速度
SignVc=sign(Vc);                               %无人机和门相对运动速度的符号

Am=0;

%仿真解算
Time=0;                                        %仿真时间
TimeStep=0.01;                                 %仿真步长
q=0;
% file=fopen('output.tyt','w');                %将数据写入文件
%循环
it= 0;
while(1)
    it=it+1;
   %若Vc改变符号则仿真结束
   if(sign(Vc) ~= SignVc)          
         break;
   else
         if(Rtm<500)
             TimeStep=0.0005;
         end
    
SignVc=sign(Vc);%Vc的符号

dSightAngle=(Rtmx*Vtmy-Rtmy*Vtmx)/(Rtm^2);%视线角速率

invb=Rtm;

kd=3;
%%%%%%%%%%%%%%%%%%无人机加速度，无人机加速度矢量垂直于视线%%%%%%%%%%%%%%%%%%
W=-Vm*cos(HeadError)/Rtm;
Am=(2+sqrt(4+N_+2*M_))*Vm*dSightAngle-M_*Vm*W*(SightAngle-SightAngle_desired);     %导引律
dPathAngle=Am/Vm;
dHeadError=dPathAngle-dSightAngle;
%---------------------------------------------------------------
% 限制机动能力
if Am>AmMay
    Am=AmMay;
end
if Am<-AmMay
    Am=-AmMay;
end
%无人机加速度分量，无人机加速度矢量垂直于视线
Amx=-Am*sin(SightAngle);
Amy=Am*cos(SightAngle);

%--------------------------状态更新-----------------------------
Time=Time+TimeStep;




%无人机
%无人机加速度矢量垂直于视线
Rmx=Rmx+TimeStep*Vmx;
Rmy=Rmy+TimeStep*Vmy;
Vmx=Vmx+TimeStep*Amx;
Vmy=Vmy+TimeStep*Amy;
Vm=sqrt(Vmy^2+Vmx^2);
HeadError=HeadError+dHeadError*TimeStep;
%无人机和门相对
%无人机和门相对位移
Rtmx=Rtx-Rmx;
Rtmy=Rty-Rmy;
Rtm0=Rtm;%上一步的脱靶量
Rtm=sqrt(Rtmx^2+Rtmy^2);
SightAngle=atan(Rtmy/Rtmx); %视线角
%无人机和门相对速度
Vtmy=-Vmy;
Vtmx=-Vmx;
Vc=-(Rtmy*Vtmy+Rtmx*Vtmx)/Rtm;
%数据写文件
% fprintf(file,'%f %f %f %f %f %f %f \n',Time,Rmy,Rmx,Rty,Rtx,sqrt(Amy^2+Amx^2), Rtm);

q=q+1;

rmx2(1,q)=Rmx;
rmy2(1,q)=Rmy;
rtx2(1,q)=Rtx;
rty2(1,q)=Rty;

a1(1,q)=0;
a2(1,q)=Am;
time2(1,q)=Time;

dq1(1,q)=SightAngle;
dq2(1,q)=dSightAngle;

vm_sq(1,q)=Vm;

b_1(1,q)=invb;


end
end

disp('脱靶量');
Rtm

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=q-200;

%点线——无人机运动轨迹
%实线——目标运动轨迹
figure(1)%穿越曲线
plot(rmx2,rmy2,'b:');
hold on;
plot(rtx2,rty2,'b');
plot(rtx2(end),rty2(end),'k+')
xlabel('水平距离(米)');
ylabel('垂直距离(米)');
legend('无人机','门','穿越点');
hold off;
title('无人机与目标拦截曲线');

figure(2)%无人机过载曲线
plot(time2,a2/g,'b');
xlabel('时间(秒)');
ylabel('过载(重力加速度)');
title('无人机与目标法向加速度曲线');

figure(3)%视线角速度
plot(time2(1:q),dq2(1:q)*57.3,'b');
xlabel('时间(秒)');
ylabel('视线角速度(度每秒)');
title('无人机与目标视线角速度曲线');

figure(4)%无人机速度
plot(time2(1:q),vm_sq(1:q),'b');
xlabel('时间(秒)');
ylabel('无人机速度(米每秒)');
title('无人机速度曲线');

