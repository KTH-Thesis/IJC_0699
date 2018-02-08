close all
clear
load('variables.mat');

%plot(tT_1, uU_1);
%hold on;
%plot(tT_2, uU_2);
%plot(tT_3, uU_3);
%hold off;

tT_1 = tT_1(2:end);
tT_2 = tT_2(2:end);
tT_3 = tT_3(2:end);

xX_1 = xX_1(2:end,:);
xX_2 = xX_2(2:end,:);
xX_3 = xX_3(2:end,:);

len = length(tT_1);
norm_error = zeros(len,3);


for i = 1:len
   norm_error(i,1) = norm(xX_1(i,:));
   norm_error(i,2) = norm(xX_2(i,:));
   norm_error(i,3) = norm(xX_3(i,:));
end

fig1 = figure(1);
h1 = plot(tT_1, norm_error(:,1), 'LineWidth', 2, 'color', 'r');
title('$\rm Error \ signals \ of \ the \ 3 \ agents$','interpreter','latex','fontsize',16);
hold on;
h2 = plot(tT_2, norm_error(:,2), 'LineWidth', 2, 'color', 'b');
h3 = plot(tT_3, norm_error(:,3), 'LineWidth', 2, 'color', 'c');
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$\|e_i(t)\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
axis([0.0 10 -1 14]);
legend([h1, h2, h3], {'$\|e_1(t)\|$','$\|e_2(t)\|$', '$\|e_3(t)\|$'},'interpreter','latex','fontsize',16);
grid on;
%%
len = length(tT_1);
norm_control = zeros(len,2);

for i = 1:len
   norm_control(i,1) = norm(uU_1(:,i), 2);
   norm_control(i,2) = norm(uU_2(:,i), 2);
   norm_control(i,3) = norm(uU_3(:,i), 2);
end

fig2 = figure(2);
h1 = plot(tT_1, norm_control(:,1), 'LineWidth', 2, 'color', 'g');
title('$\rm Control \ inputs $','interpreter','latex','fontsize',16);
hold on;
h2 = plot(tT_2, norm_control(:,2), 'LineWidth', 2, 'color', 'k');
h3 = plot(tT_3, norm_control(:,3), 'LineWidth', 2, 'color', 'm');
xlabel('$t \ [sec]$','interpreter','latex','fontsize',16);
ylabel({'$\|u_i(t)\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
axis([0.0 10 -1 sqrt(2)*u_abs+1]);
h4 = plot(tT_1, 8*sqrt(2)*ones(len,1), 'LineWidth', 2, 'color', 'y');
legend([h1 h2 h3 h4], {'$\|u_1(t)\|$','$\|u_2(t)\|$', '$\|u_3(t)\|$', '$\overline{u}_i$'},'interpreter','latex','fontsize',16);
grid on;
%% print results

% print('error_plots', '-depsc', '-r500');
% print('control_bounds', '-depsc', '-r500');

%% 

fig3 = figure(3);

dist_ag1_obs_01 = sqrt((xX_1(:,1)+(des_1(1)-obs_0(1,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs_0(1,2))*ones(size(tT_1))).^2);
dist_ag2_obs_01 = sqrt((xX_2(:,1)+(des_2(1)-obs_0(1,1))*ones(size(tT_1))).^2 + (xX_2(:,2)+(des_2(2)-obs_0(1,2))*ones(size(tT_2))).^2);
dist_ag3_obs_01 = sqrt((xX_3(:,1)+(des_3(1)-obs_0(1,1))*ones(size(tT_1))).^2 + (xX_3(:,2)+(des_3(2)-obs_0(1,2))*ones(size(tT_2))).^2);
h1 = plot(tT_1, dist_ag1_obs_01, 'LineWidth', 2, 'color', 'r');
title('$\rm Distance \ between \ agents \ and \ obstacle \ 1$','interpreter','latex','fontsize',16);
ylabel({'$\|p_i(t)-p_{\scriptscriptstyle O_1}\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
hold on;
h2 = plot(tT_2, dist_ag2_obs_01, 'LineWidth', 2, 'color', 'g');
h3 = plot(tT_3, dist_ag3_obs_01, 'LineWidth', 2, 'color', 'm');
h4 = plot(tT_1, 1.46*ones(len,1), 'color', 'y');
legend([h1 h2 h3 h4], {'$\|p_1(t)-p_{\scriptscriptstyle O_1}\|$','$\|p_2(t)-p_{\scriptscriptstyle O_1}\|$', '$\|p_3(t)-p_{\scriptscriptstyle O_1}\|$', '$r_i+r_{\scriptscriptstyle O_1}+\varepsilon$'},'interpreter','latex','fontsize',16);
grid on;

%%

fig4 = figure(4);

dist_ag1_obs_02 = sqrt((xX_1(:,1)+(des_1(1)-obs_0(2,1))*ones(size(tT_1))).^2 + (xX_1(:,2)+(des_1(2)-obs_0(2,2))*ones(size(tT_1))).^2);
dist_ag2_obs_02 = sqrt((xX_2(:,1)+(des_2(1)-obs_0(2,1))*ones(size(tT_1))).^2 + (xX_2(:,2)+(des_2(2)-obs_0(2,2))*ones(size(tT_2))).^2);
dist_ag3_obs_02 = sqrt((xX_3(:,1)+(des_3(1)-obs_0(2,1))*ones(size(tT_1))).^2 + (xX_3(:,2)+(des_3(2)-obs_0(2,2))*ones(size(tT_2))).^2);
h1 = plot(tT_1, dist_ag1_obs_02, 'LineWidth', 2, 'color', 'c');
title('$\rm Distance \ between \ agents \ and \ obstacle \ 2$','interpreter','latex','fontsize',16);
ylabel({'$\|p_i(t)-p_{\scriptscriptstyle O_2}\|, i \in \{1,2,3\}$'},'interpreter','latex','fontsize',16);
hold on;
h2 = plot(tT_2, dist_ag2_obs_02, 'LineWidth', 2, 'color', 'r');
h3 = plot(tT_3, dist_ag3_obs_02, 'LineWidth', 2, 'color', 'g');
h4 = plot(tT_1, 1.46*ones(len,1), 'color', 'y');
legend([h1 h2 h3 h4], {'$\|p_1(t)-p_{\scriptscriptstyle O_2}\|$','$\|p_2(t)-p_{\scriptscriptstyle O_2}\|$', '$\|p_3(t)-p_{\scriptscriptstyle O_2}\|$', '$r_i+r_{\scriptscriptstyle O_2}+\varepsilon$'},'interpreter','latex','fontsize',16);
axis([0 10 1.0 7]);
grid on;

%%

% Distance of agents
figure5 = figure(5);
dist_ag12 = sqrt( (xX_1(:,1) - xX_2(:,1)+ (des_1(1) - des_2(1))*ones(size(tT_1))).^2 +(xX_1(:,2) - xX_2(:,2)+ (des_1(2) - des_2(2))*ones(size(tT_1))).^2);
dist_ag23 = sqrt((xX_1(:,1) - xX_3(:,1)+ (des_1(1) - des_3(1))*ones(size(tT_1))).^2 + (xX_1(:,2) - xX_3(:,2)+ (des_1(2) - des_3(2))*ones(size(tT_1))).^2 );

h1 = plot(tT_1, dist_ag12, 'LineWidth', 2, 'color', 'c');
title('$\rm Distance \ between \ agents \ 1-2 \ and \ 1-3$','interpreter','latex','fontsize',16);
ylabel({'$\|p_i(t)-p_{j}(t)\|, i \in \{1,2,3\}, j \in \mathcal{N}_i$'},'interpreter','latex','fontsize',16);
hold on;
h2 = plot(tT_1, dist_ag23, 'LineWidth', 2, 'color', 'r');
h3 = plot(tT_1, 1.0*ones(len,1), 'color', 'y');
h4 = plot(tT_1, 2.0*ones(len,1), 'color', 'b');
%h4 = plot(tT_1, 1.46*ones(len,1), 'color', 'y');
legend([h1 h2 h3 h4], {'$\|p_1(t)-p_{2}(t)\|$','$\|p_1(t)-p_{3}(t)\|$', '$r_i+r_j+\varepsilon$', '$d_i - \varepsilon$'},'interpreter','latex','fontsize',16);
grid on;
axis([0 10 0.9 2.1]);


