dist1 = dist;
for i = 2:313
    if dist(i) == 0 
        dist1(i) = (dist(i+1) + dist(i-1))./2;
    end
end
x = linspace(1,10,313);
figure
plot(x,dist),title('Speed Calculation')
% figure
% plot(x,dist1)
fffff = dist~=0;

% [pks,locs] = findpeaks(dist1);
% plot(x,dist1,'or')
%%
a = fit( transpose(x),dist1,'nearestinterp');
%%
figure
plot(a, transpose(x), dist1)