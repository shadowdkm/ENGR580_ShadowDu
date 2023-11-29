A1=[0,1;-6,-5];
A2=[0,1;-30,-11];
B=[0;1];
C=[1,1];
D=[0];
s1=ss(A1,B,C,D);
s2=ss(A2,B,C,D);

[y1,tOut,x1] = impulse(s1,3);
[y2,tOut,x2] = impulse(s2,tOut);
figure(1), clf, hold on

plot(tOut,y1,"r","linewidth",2)
plot(tOut,x1(:,1),"r--")
plot(tOut,x1(:,2),"r.")
plot(tOut,y2,"g","linewidth",2)
plot(tOut,x2(:,1),"g--")
plot(tOut,x2(:,2),"g.")

xlabel("Time (s)")
ylabel("Impulse Responce")

legend("Output Y, pole@ -2,-3","X1, pole@ -2,-3","X2, pole@ -2,-3","Output Y, pole@ -5,-6","X1, pole@ -5,-6","X2, pole@ -5,-6")