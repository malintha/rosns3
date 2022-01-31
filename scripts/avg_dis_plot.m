clear;
load('avg_entrees.txt');
dat_ = avg_entrees;

dis = dat_(:,1);
errors = [];
means = [];
stds = [];

[idx,c] = kmeans(dis,20);
for i=1:length(dat_)
    index = idx(i);
    dat_(i,1) = c(index,1);
end
c = sort(c(:,1));

for i=1:length(c)
    x = dat_(i,1);
    data = dat_(any(dat_==c(i,1),2),2:end);
    data = data(:);
    outliers = any(data == 8,2);
    data(outliers) = 7;
    means = [means; mean(data)];
    stds = [stds; std(data)];
end

errorbar(c(:,1),means,stds, 'LineWidth',2);

hold on

% avg distance to a node

errors = [];
means = [];
errors = [];

load('hops.txt');
dat_ = hops;
dis = dat_(:,1);
[idx,c] = kmeans(dis,20);

for i=1:length(dis)
    index = idx(i);
    dat_(i,1) = c(index,1);
end
c = sort(c(:,1));

for i=1:length(c)
    x = dat_(i,1);
    data = dat_(any(dat_==c(i,1),2),2:end);
    data = data(:);
    outliers = any(data == 0,2);
    data(outliers) = [];
    means = [means; mean(data)];
    errors = [errors; std(data)];
end

errorbar(c(:,1),means,errors, 'LineWidth',2);
legend('Avg. Routing Nodes','Avg. Hops to a Node');
xlabel('Average Distance Between Robots (m)');
ylabel('Average #');
% yu = means + errors;
% yl = means - errors;
% fill([c fliplr(c)], [yu fliplr(yl)], [.9 .9 .9], 'linestyle', 'none')
% hold on


