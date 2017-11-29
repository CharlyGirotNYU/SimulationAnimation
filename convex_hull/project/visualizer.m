clear;
clf;

out;

plot(V(1,:),V(2,:),'.');
hold on
plot(hull(1,:),hull(2,:),'r-','LineWidth',3);
plot(hull(1,:),hull(2,:),'k+','LineWidth',3);
