close all;
%% UPPERARM
figure1 = figure;
axes1 = axes('Parent',gcf,'FontSize',18);
box(axes1,'on');
hold(axes1,'on');
plot(tRobotran,qRobotran(:,2)*180/pi,'DisplayName','Résultats de Robotran','LineWidth',2,'Color',[0 0 1]);
hold on;
plot(tSimu,qSimu(:,2)*180/pi,'DisplayName','Résultats du projet','LineWidth',2,'LineStyle','--','Color',[1 0 0]);
xlabel('Time [s]','FontSize',18);
ylabel('Angle du bras supérieur [°]','LineWidth',2,'FontSize',18);
legend1 = legend(axes1,'show');
set(legend1,'Location','SouthEast');
set(figure1, 'Position', [0 0 1500 600]);
set(figure1, 'PaperPositionMode', 'auto');
ylim([-30 -12]);
print -depsc -r300 Plots1.eps 
%% ZOOM
figure1 = figure;
axes1 = axes('Parent',gcf,'FontSize',18);
box(axes1,'on');
hold(axes1,'on');
plot(tRobotran,qRobotran(:,2)*180/pi,'DisplayName','Résultats de Robotran','LineWidth',2,'Color',[0 0 1]);
hold on;
plot(tSimu,qSimu(:,2)*180/pi,'DisplayName','Résultats du projet','LineWidth',2,'Color',[1 0 0]);
xlabel('Time [s]','FontSize',18);
ylabel('Angle du bras supérieur [°]','LineWidth',2,'FontSize',18);
legend1 = legend(axes1,'show');
set(legend1,'Location','SouthEast');
set(figure1, 'Position', [0 0 1500 600]);
set(figure1, 'PaperPositionMode', 'auto');
ylim([-31 -12]);
xlim([0 0.5]);
print -depsc -r300 Plots2.eps 

%% Hauteur du chassis
figure1 = figure;
axes1 = axes('Parent',gcf,'FontSize',18);
box(axes1,'on');
hold(axes1,'on');
plot(tRobotran,qRobotran(:,1),'DisplayName','Résultats de Robotran','LineWidth',2,'Color',[0 0 1]);
hold on;
plot(tSimu,qSimu(:,1),'DisplayName','Résultats du projet','LineWidth',2,'LineStyle','--','Color',[1 0 0]);
xlabel('Time [s]','FontSize',18);
ylabel('Hauteur du chassis [m]','LineWidth',2,'FontSize',18);
legend1 = legend(axes1,'show');
set(legend1,'Location','SouthEast');
set(figure1, 'Position', [0 0 1500 600]);
set(figure1, 'PaperPositionMode', 'auto');
print -depsc -r300 Plots3.eps 