% Simulating state of nodes
%
% Implements the displacement-based formation controller from Section V-B
% of the cited survey: each node moves to correct the error between its
% actual relative position to its neighbors and the target relative
% position encoded in `Shape`, plus a sigma-norm-based repulsive term
% (Auxiliar.sigma_norm / rho_h / sigma_1, see Auxiliar.m in this same
% folder — an Olfati-Saber-style flocking potential function) to avoid
% node-to-node collisions while not yet "assembling" into the final
% formation.
clear
%controller Section V-B
%A survey of multi-agent formation control: Position-, displacement-, and distance-based approaches
%Kwang-Kyo Oh, Myoung-Chul Park, and Hyo-Sung Ahn


Shape = [
    0  1  0  0;
    0  0  1 -1;
    1  1  1  1];

% Shape = [
%     0  1  1  1;
%     0  0  1 -1;
%     1  1  1  1];
% 
% Shape = [
%     0  1  0  0;
%     0  0  1 -1;
%     1  1  1  1];
% 
% Shape = [
%     -1  0  0  0;
%      0  0  1 -1;
%      1  1  1  1];


%% 1D - 2 nodes
% Gamma=1;   % Inter state relationship matrix
% A=[0 1; 1 0];       % Ajacency Matrix
% A_deg=A;       % Ajacency Matrix version with degree
% for i=1:size(A,1); A_deg(i,i) = -sum(A(i,:)); end
% N=2;                % Nodes in the network, cardinallity of vertex set
% n=1;                % Variables of the state of each node dimensions to consider
% x=[0.25,0.75;-0.5,0.5];
% %u=[0;0.1]*0;

%% 2D - 2 nodes
% Gamma=[1 0; 0 1];   % Inter state relationship matrix
% A=[0 1; 1 0];       % Ajacency Matrix
% A_deg=A;       % Ajacency Matrix version with degree
% for i=1:size(A,1); A_deg(i,i) = -sum(A(i,:)); end
% N=2;                % Nodes in the network, cardinallity of vertex set
% n=2;                % Variables of the state of each node
% x=[ [0.1;0.55;0;0] , [0.7;0.8;0;0] ] ;


%% 3D - 3 nodes
% N=4;                % Nodes in the network, cardinallity of vertex set
% n=3;                % Variables of the state of each node
% Gamma=[1 0; 0 1];   % Inter state relationship matrix
% A=zeros(N,N);       % Ajacency Matrix
% A(1,2)=1; A(1,3)=1; A(1,4)=1; 
%           A(2,3)=1; A(2,4)=1; 
%                     A(3,4)=1; 
% A = A + A';
% A_deg=A;       % Ajacency Matrix version with degree
% % D=zeros(N,N);       % Ajacency Matrix
% % d=0.25;
% % rd=sqrt(2)*d;
% % D(1,2)=2*d; D(1,3)=2*rd; D(1,4)=2*d;
% %             D(2,3)=2*d;  D(2,4)=2*rd;
% %                          D(3,4)=2*rd;
% % D = D+D';
% x=zeros(n*2,N);
% for i=1:N x(:,i)=[i/1;0;0;0;0;0];  end
% % for i=1:N; x(:,i)=[i/2;((-1)^i)/2;0;0;0;0]+[0;0.5;0;0;0;0]; end

%% 3D - 9 nodes
N=4;                % Nodes in the network, cardinallity of vertex set
n=3;                % Variables of the state of each node
Gamma=[1 0; 0 1];   % Inter state relationship matrix
A=zeros(N,N);       % Ajacency Matrix
A(1,2)=1; A(1,3)=1; A(1,4)=1; 
          A(2,3)=0; A(2,4)=0; 
                    A(3,4)=0; 
                              
A = A + A';
x=zeros(n,N);
for i=1:N; x(:,i)=[i/10;rand;1]+[0;0;0]; end

%% WP navigation of the ensemble
%2D
% WP = [2.5;2.5;0;0];
% for j=2:5; WP(:,j) = WP(:,j-1)+[1;1;0;0]; end
% %3D
WP = [0;0;0;0;0;0];
for j=2:5; WP(:,j) = WP(:,j-1)+[0.2;0.2;0;0;0;0]; end
nextwp=1;
WP(3,nextwp)=Shape(3,1);

close all
figure

%% Flocking simulation

% Discretizing the adjacency matrix to simulate the network behaviour
%Ad = ssdata(c2d(ss(A_deg,zeros(N*n,1),zeros(1,N*n),zeros(N*n,1)),dt))
% `i` here is whatever value the "for i=1:N" initialization loop above
% left it at (i.e. N) — this just plots that last node's starting position
% as a one-off priming point before the main loop below takes over.
plot3(x(1,i,1),x(2,i,1), x(3,i,1), 'xr');
axis([-2 2 -2 2 0 2])
% axis([-1 5 -1 5])
grid on
hold on
epslon = 1;
dt=0.1;

int_range_r = 1.33;
sigma_int_range = Auxiliar.sigma_norm(int_range_r);

now=1
kp=0.5
kv=1
assembling=0;



for t=dt:dt:200
    tic
    k=round(t/dt);
    cla
    hold on
    clc;
    for i=1:N
        % Figure plot
        % --------------- 
%          plot(x(1,i,k), 0.5, 'xr'); %1D
%          text(x(1,i,k)+0.01, 0.5+0.01, num2str(i)); %1D
        %2D
%         plot(x(1,i,k),x(2,i,k), 'xr');
%         text(x(1,i,k)+0.01,x(2,i,k)+0.01, num2str(i)); 
        %3D        
        plot3(x(1,i,k),x(2,i,k), x(3,i,k), 'xr');  
        text(x(1,i,k)+0.01,x(2,i,k)+0.01,x(3,i,k)+0.01, num2str(i)); 
        plot3([x(1,i,k),x(1,i,k)],[x(2,i,k),x(2,i,k)],[x(3,i,k),0], 'k--');
        
        posi = x(1:n,i,k);
%         veli = x(n+1:n+n,i,k);
        u = x(1:n,1,1)*0;


        posi_s = Shape(1:n,i);
        plot3(posi_s(1),posi_s(2),posi_s(3), 'xm');  
        text(posi_s(1,1)+0.01,posi_s(2,1)+0.01,posi_s(3,1)+0.01,num2str(i));

        for j=1:N
            posj = x(1:n,j,k);
            if i~=j && A(i,j)~=0
%                 posj = x(1:n,j,k);
%                 velj = x(n+1:n+n,j,k);
                posj_s = Shape(1:n,j);
                u = u + kp*(posi - posj - posi_s + posj_s);%+ kv*(veli-velj);
                if i>j
                    plot3([x(1,i,k) x(1,j,k)],[x(2,i,k) x(2,j,k)],[x(3,i,k) x(3,j,k)], 'k');
                    %text(mean([x(1,i,k) x(1,j,k)]),mean([x(2,i,k) x(2,j,k)]), strcat('d=',num2str(sqrt((x(1,i,k)-x(1,j,k))^2+(x(2,i,k)-x(2,j,k))^2),3)));
                    plot3([posi_s(1),posj_s(1)]+WP(1,nextwp),[posi_s(2),posj_s(2)]+WP(2,nextwp),[posi_s(3),posj_s(3)], 'm--');
                end
            end
            if i~=j
                %trying to avoid colisions
                diff = posj-posi;
                sigma_diff = Auxiliar.sigma_norm(diff);
                sigma_d = Auxiliar.sigma_norm(1);
                phi_alpha = Auxiliar.rho_h(sigma_diff/sigma_int_range)*Auxiliar.sigma_1(sigma_diff-sigma_d);
                nij= diff/sqrt(1+epslon*norm(diff,2)^2);
%                 if norm(phi_alpha*nij,2)>0.001 || norm(diff) < 0.2
%                     %just for debug
%                     [i,j, norm(phi_alpha*nij,2), norm(diff)]
%                 end
                %do not avoid colisions if it is an assembly stage
                if assembling==0; u = u - phi_alpha*nij; end
                if norm(diff)<0.6
                    disp(norm(phi_alpha*nij))
                end
            end
        end
        
        u=-1*u;
        
        %Pinning controller
        if i==1
%             u = u + 1e0*(WP(1:n,nextwp)-mean(x(1:n,:,k),2)) + 1e1*(WP(n+1:n+n,nextwp)-mean(x(n+1:n+n,:,k),2));
            u = u + 1e0*(WP(1:n,nextwp)-x(1:n,i,k));% + 1e1*(WP(n+1:n+n,nextwp)-x(n+1:n+n,i,k));
        end
        
        x(1:n,i,k+1)  = posi + dt*u;
    end
    
    
    plot(mean(x(1,:,k),2),mean(x(2,:,k)), 'ok');
    text(mean(x(1,:,k))+0.01,mean(x(2,:,k))+0.01, 'CM');
    plot(WP(1,nextwp),WP(2,nextwp), 'vk');
    text(WP(1,nextwp)+0.02,WP(2,nextwp)+0.02, 'goal');
    title(strcat('t = ',num2str(t),' s'));
    
    %shirink the structure, assembly stage
    %if norm((WP(1:n,nextwp)-mean(x(1:n,:,k),2)), 2) < 0.2 &&...
    if norm(WP(1:n,nextwp)-x(1:n,1,k),2)  < 0.05 &&...
       norm((Shape(:,1)-Shape(:,2)), 2) > 0.2
        disp([norm((WP(1:n,nextwp)-mean(x(1:n,:,k),2)), 2), norm((Shape(:,1)-Shape(:,2)), 2), WP(3,nextwp)])
        Shape=Shape*0.999;
        WP(3,nextwp)=Shape(3,1);%WP(3,nextwp)*0.999;
%         WP(3,nextwp)=norm((End_POS_pyramid(:,1)-End_POS_pyramid(:,2)), 2)-0.2+0.135;%WP(3,nextwp)*0.999;
        assembling=1;
%         axis tight
    end
        
%     if abs(WP(1:n,nextwp)-mean(x(1:n,:,k),2)) + abs(WP(n+1:n+n,nextwp)-mean(x(n+1:n+n,:,k),2)) < 0.05
%         nextwp=mod(nextwp+1,size(WP,2));
%         if nextwp==0;nextwp=1;end
%     end

    hold off
    % ---------------

   proc_time=toc
   drawnow
   pause(0.1)
   if dt>proc_time pause(abs(dt-proc_time)); end

%     if t>=now
% %         print(strcat(num2str(now),'s'),'-dpng')
%         now=t+1;
%         pause(0.05);
%     end
%    pause(0.02);
end

figure
hold on
grid on


for node=1:N
    subplot(3,ceil(N/3),node); hold on; grid on;
    for j=1:n
        state_node=zeros(k,0);
        state_name='';
        for time=1:k; state_node(time)=x(j,node,time); end; 
        switch j; case 1, state_name='x' ; case 2, state_name='y' ; case 3, state_name='z' ; end
        plot(state_node, 'displayname', state_name)
    end
    title(strcat('Node ',num2str(node)));
    legend show
    axis tight
    hold off
end



%% Not used
%                 zed = posj-posi;
%                 sigma_norm_zed = Auxiliar.sigma_norm(zed);
%                 delta_sigma = zed / (1+epslon*sigma_norm_zed);
                %d_sig_norm=sqrt(1+norm(d,2)^2)-1;
                %phi_function_minus=(sigma_norm^2 -2*d_sig_norm*sigma_norm + d_sig_norm^2);
                %delta = (phi_function_plus-phi_function_minus)/0.001;
                %posji = posj-posi;
                %sigma_norm=sqrt(1+norm(posji,2)^2)-1;
                %phi_function=(sigma_norm^2 -2*d_sig_norm*sigma_norm + d_sig_norm^2);