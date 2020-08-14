clear all;    	  %��������ڴ����
%--------��ʼ���Ƶ�ϵͳ����------------------
pi=3.141592653;
g=9.8;
Rmx=0;Rmy=0;                                   %���˻���λ��
Rtx=50; Rty=50;                                %�ŵ�λ��
Vm=3.5;                                          %���˻����ٶ�
Vt=0;                                          %�ŵ��ٶ�
HeadError=10/180*pi;                            %ָ������

AtMay=0*g;                                     %Ŀ�겻����
AmMay=5*g;                                     %����������������Ϊ20G

M_= 5;
N_= 2;
SightAngle_desired = 0/180*pi;

%��Ŀ�����˶�ѧ����
Rtmx=Rtx-Rmx;                                  %���˻�����x����Ծ���
Rtmy=Rty-Rmy;                                  %���˻�����y����Ծ���
Rtm=sqrt(Rtmx^2+Rtmy^2);                       %���˻�������Ծ���
SightAngle=atan(Rtmy/Rtmx);                    %���߽�
PathAngle=SightAngle+HeadError;
Vmx=Vm*cos(SightAngle+HeadError);              %���˻���x���ٶȷ���
Vmy=Vm*sin(SightAngle+HeadError);              %���˻���y���ٶȷ���
Vtx=0;                                         %�ŵ��ٶ�x�����
Vty=0;                                         %�ŵ��ٶ�y�����
Vtmx=-Vmx;                                     %���˻���������˶���x���ٶȷ���
Vtmy=-Vmy;                                     %���˻���������˶���y���ٶȷ���
Vc=-(Rtmx*Vtmx+Rtmy*Vtmy)/Rtm;                 %���˻���������˶��ٶ�
SignVc=sign(Vc);                               %���˻���������˶��ٶȵķ���

Am=0;

%�������
Time=0;                                        %����ʱ��
TimeStep=0.01;                                 %���沽��
q=0;
% file=fopen('output.tyt','w');                %������д���ļ�
%ѭ��
it= 0;
while(1)
    it=it+1;
   %��Vc�ı������������
   if(sign(Vc) ~= SignVc)          
         break;
   else
         if(Rtm<500)
             TimeStep=0.0005;
         end
    
SignVc=sign(Vc);%Vc�ķ���

dSightAngle=(Rtmx*Vtmy-Rtmy*Vtmx)/(Rtm^2);%���߽�����

invb=Rtm;

kd=3;
%%%%%%%%%%%%%%%%%%���˻����ٶȣ����˻����ٶ�ʸ����ֱ������%%%%%%%%%%%%%%%%%%
W=-Vm*cos(HeadError)/Rtm;
Am=(2+sqrt(4+N_+2*M_))*Vm*dSightAngle-M_*Vm*W*(SightAngle-SightAngle_desired);     %������
dPathAngle=Am/Vm;
dHeadError=dPathAngle-dSightAngle;
%---------------------------------------------------------------
% ���ƻ�������
if Am>AmMay
    Am=AmMay;
end
if Am<-AmMay
    Am=-AmMay;
end
%���˻����ٶȷ��������˻����ٶ�ʸ����ֱ������
Amx=-Am*sin(SightAngle);
Amy=Am*cos(SightAngle);

%--------------------------״̬����-----------------------------
Time=Time+TimeStep;


if Time<2
    At=-0*AtMay;
elseif Time<7
    At=AtMay;
else
    At=-AtMay;
end

At=-AtMay*sin(Time*0.25*pi);


%���˻�
%���˻����ٶ�ʸ����ֱ������
Rmx=Rmx+TimeStep*Vmx;
Rmy=Rmy+TimeStep*Vmy;
Vmx=Vmx+TimeStep*Amx;
Vmy=Vmy+TimeStep*Amy;
Vm=sqrt(Vmy^2+Vmx^2);
HeadError=HeadError+dHeadError*TimeStep;
%���˻��������
%���˻��������λ��
Rtmx=Rtx-Rmx;
Rtmy=Rty-Rmy;
Rtm0=Rtm;%��һ�����Ѱ���
Rtm=sqrt(Rtmx^2+Rtmy^2);
SightAngle=atan(Rtmy/Rtmx); %���߽�
%���˻���������ٶ�
Vtmy=Vty-Vmy;
Vtmx=Vtx-Vmx;
Vc=-(Rtmy*Vtmy+Rtmx*Vtmx)/Rtm;
%����д�ļ�
% fprintf(file,'%f %f %f %f %f %f %f \n',Time,Rmy,Rmx,Rty,Rtx,sqrt(Amy^2+Amx^2), Rtm);

q=q+1;

rmx2(1,q)=Rmx;
rmy2(1,q)=Rmy;
rtx2(1,q)=Rtx;
rty2(1,q)=Rty;

a1(1,q)=At;
a2(1,q)=Am;
time2(1,q)=Time;

dq1(1,q)=SightAngle;
dq2(1,q)=dSightAngle;

vm_sq(1,q)=Vm;

b_1(1,q)=invb;


end
end

disp('�Ѱ���');
Rtm

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q=q-200;

%���ߡ������˻��˶��켣
%ʵ�ߡ���Ŀ���˶��켣
figure(1)%��Խ����
plot(rmx2,rmy2,'b:');
hold on;
plot(rtx2,rty2,'b');
plot(rtx2(end),rty2(end),'k+')
xlabel('ˮƽ����(��)');
ylabel('��ֱ����(��)');
legend('���˻�','��','��Խ��');
hold off;
title('���˻���Ŀ����������');

figure(2)%���˻���������
plot(time2,a2/g,'b');
xlabel('ʱ��(��)');
ylabel('����(�������ٶ�)');
title('���˻���Ŀ�귨����ٶ�����');

figure(3)%���߽��ٶ�
plot(time2(1:q),dq2(1:q)*57.3,'b');
xlabel('ʱ��(��)');
ylabel('���߽��ٶ�(��ÿ��)');
title('���˻���Ŀ�����߽��ٶ�����');

figure(4)%���˻��ٶ�
plot(time2(1:q),vm_sq(1:q),'b');
xlabel('ʱ��(��)');
ylabel('���˻��ٶ�(��ÿ��)');
title('���˻��ٶ�����');

