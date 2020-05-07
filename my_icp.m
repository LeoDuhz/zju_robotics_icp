function [tform,movingReg]=my_icp(moving,fixed,tform_init,max_iteration,tolerance_t,tolerance_r)
% my_icp solves the problem of pointclouds matching and generating the map
% and robot track
iteration=0;
pp=moving.Location;%待旋转平移的矩阵
p=fixed.Location;  %目标矩阵

H=tform_init';

R=H(1:3,1:3);
T=H(1:3,4);
pp=(R*pp'+T)';%事先用tform_init进行预处理，减少迭代次数

n=size(pp,1);

d=10;%预设一个很大的均方误差

while d>tolerance_t %|| r>tolerance_r
    if iteration>max_iteration
        break
    end
    
    for i=1:n                    %寻找pp中的点在p中的对应点（用欧式距离判断）
        delta_p=p-pp(i,:);
        delta_psquare=delta_p.^2;
        delta=sum(delta_psquare,2);
        [~,min_index]=min(delta);
        ppp(i,:)=p(min_index,:); %ppp矩阵保存了pp在p中的每一组对应点
    end
    
    %寻找质心
    masscenter_pp=sum(pp)/size(pp,1);
    masscenter_ppp=sum(ppp)/size(ppp,1);
    
    %去质心化
    qq=pp-masscenter_pp;
    qqq=ppp-masscenter_ppp;

    W=zeros(3,3);
    for i=1:n
        W=W+qq(i,:)'*qqq(i,:);%求矩阵W
    end
    

    [U,~,V]=svd(W);%svd分解

    R=V*U';%求出旋转矩阵R
    T=masscenter_ppp'-R*masscenter_pp';%利用质心求出平移矩阵T

    pp=R*pp'+T;%对pp进行变换
    pp=pp';
    
    h=[R,T;0,0,0,1];
    H=h*H;%更新旋转平移矩阵H
    
    delta_p=pp-ppp;
    m=size(delta_p,1);
    delta_psquare=sum(delta_p.^2,2);
    d=sum(delta_psquare)/n;  %计算均方差
    
    iteration=iteration+1;%迭代次数加1，用于判断是否退出
end
H=H';%我这里处理的时候一直用H的转置去做，为了和标准matlab函数对照，我进行了转置
tform=affine3d(H);%以三维仿射变换形式输出
movingReg=pointCloud(pp);%以点云形式输出

        
        
        
