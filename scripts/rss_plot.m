%% test RSS with distance

load('sto_rss.dat');
dat_ = sto_rss;
dat = [];
max_dis = 199;
for r=1:max_dis
    row = r;
   
    for i=1:length(dat_)
        row_val = dat_(i,:);
        if(row_val(1) == r) 
            row = [row row_val(2)];
        end
    end
    dat = [dat;row];
end

% calculate the error bars
means = mean(dat(:,2:end), 2);
stds = [];
[sets, ~] = size(dat);

for i=1:sets
    stds = [stds; std(dat(i,2:end))];
end

% 
% for s=1:100
% f = [f normrnd(0,sqrt(32))];
% end

% figure

% x = dat(:,1);
% yu = means + stds;
% yl = means - stds;
% fill([x fliplr(x)], [yu fliplr(yl)], [.9 .9 .9], 'linestyle', 'none')
% errorbar(x, means, stds); 
% hold on
% 
% plot(x, means, 'LineWidth',2);


% 
% hold off
figure
data = dat(:,2:end);
plot_areaErrorBar(data');
hold on

ylabel("RSS (dBm)")
xlabel("Distance (m)")
% without nakagami fading
load('sto_rss_log.dat');
dat_ = sto_rss_log;
plot(dat_(1:max_dis,1),dat_(1:max_dis,2),'--r','LineWidth',2);
legend({'Std. Dev Error','Avg. RSS (Log + Stochastic Noise)','RSS (Log Fading)'});
ylim([-105,-30]);
grid on
set(axes1,'FontSize',13,'GridAlpha',0.4,'GridLineStyle','--','LineWidth',...
    1.5,'XGrid','on','YGrid','on');
hold off