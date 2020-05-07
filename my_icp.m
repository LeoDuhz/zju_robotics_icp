function [tform,movingReg]=my_icp(moving,fixed,tform_init,max_iteration,tolerance_t,tolerance_r)
% my_icp solves the problem of pointclouds matching and generating the map
% and robot track
iteration=0;
pp=moving.Location;%����תƽ�Ƶľ���
p=fixed.Location;  %Ŀ�����

H=tform_init';

R=H(1:3,1:3);
T=H(1:3,4);
pp=(R*pp'+T)';%������tform_init����Ԥ�������ٵ�������

n=size(pp,1);

d=10;%Ԥ��һ���ܴ�ľ������

while d>tolerance_t %|| r>tolerance_r
    if iteration>max_iteration
        break
    end
    
    for i=1:n                    %Ѱ��pp�еĵ���p�еĶ�Ӧ�㣨��ŷʽ�����жϣ�
        delta_p=p-pp(i,:);
        delta_psquare=delta_p.^2;
        delta=sum(delta_psquare,2);
        [~,min_index]=min(delta);
        ppp(i,:)=p(min_index,:); %ppp���󱣴���pp��p�е�ÿһ���Ӧ��
    end
    
    %Ѱ������
    masscenter_pp=sum(pp)/size(pp,1);
    masscenter_ppp=sum(ppp)/size(ppp,1);
    
    %ȥ���Ļ�
    qq=pp-masscenter_pp;
    qqq=ppp-masscenter_ppp;

    W=zeros(3,3);
    for i=1:n
        W=W+qq(i,:)'*qqq(i,:);%�����W
    end
    

    [U,~,V]=svd(W);%svd�ֽ�

    R=V*U';%�����ת����R
    T=masscenter_ppp'-R*masscenter_pp';%�����������ƽ�ƾ���T

    pp=R*pp'+T;%��pp���б任
    pp=pp';
    
    h=[R,T;0,0,0,1];
    H=h*H;%������תƽ�ƾ���H
    
    delta_p=pp-ppp;
    m=size(delta_p,1);
    delta_psquare=sum(delta_p.^2,2);
    d=sum(delta_psquare)/n;  %���������
    
    iteration=iteration+1;%����������1�������ж��Ƿ��˳�
end
H=H';%�����ﴦ���ʱ��һֱ��H��ת��ȥ����Ϊ�˺ͱ�׼matlab�������գ��ҽ�����ת��
tform=affine3d(H);%����ά����任��ʽ���
movingReg=pointCloud(pp);%�Ե�����ʽ���

        
        
        
