%% test RSS with distance
% epsilon = 1;
x = linspace(1,200,200);
Txp = 16.6;
L0 = 46;
n = 3;
w = 0;
var_f = 32;
f = [];

for s=1:100
f = [f normrnd(0,sqrt(32))];
end

P_r = (Txp) - L0 - 10*n*log(x);

% figure
% yu = P_r + sqrt(32);
% yl = P_r - sqrt(32);
% fill([x fliplr(x)], [yu fliplr(yl)], [.9 .9 .9], 'linestyle', 'none')
hold on

plot(x, P_r, 'LineWidth',2);
ylabel("RSS (dBm)")
xlabel("Distance (m)")
xlim([1 20])
hold off
%%
% rss_2d();
%% function for calculating the expected rss
function[pr] = rss_mu(pos_a, pos_b)
    L0 = 46;
    n = 2.1;
    w = 0;
    x = norm(pos_a - pos_b);

    pr = L0 - 10*n*log(x) - 10;
end

%% calculate communication rate
function[cr] = crate(pr)
    k = 10e-3;
    pn0 = -10;
    R0 = 250;
    pe = erf(real(sqrt((k*pr)/pn0)));
    cr = R0*(1-pe);
end

%% in 2 dimensions
function[] = rss_2d()
    st_sp1 = [];
    for i=-10:10
        for j=-10:10
            st_sp1(i+11,j+11) = rss_mu([i,j],[0,0]);
        end
    end
    cr = crate(st_sp1);
    figure('Name','RSS');
    im1 = imagesc(st_sp1);
    im1.Interpolation = 'bilinear';
    hold on
    colorbar;
    h0 = colorbar;
    ylabel(h0, "dBm")
%     title('RSS ')
    xlabel('Distance(m)');
    ylabel('Distance (m)');
    hold off
    
    figure('Name','Communication Rate');
    im = imagesc(cr);
    hold on
    title('Communication Rate')
    xlabel('Distance');
    ylabel('Distance');
    im.Interpolation = 'bilinear';
    h1 = colorbar;
    ylabel(h1, "kbps")
    hold off
end
