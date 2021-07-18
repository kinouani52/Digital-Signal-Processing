
x0=load('confondre.sig');
figure (); plot (x0); title ("x");



N0=5000;
y0 = abs((fft(x0,N0)));
t0 =  linspace(-500,500,N0);
figure;
plot(t0,fftshift(y0));


fs=1000;
N=200;
fk=100;
Wn= fk/(fs/2);

%%Low pass filter:
b = fir1(200,Wn);
[A,~] = freqz(b, 1, 200, "whole");
f = linspace(-500,500, 200);
figure;
plot(f, abs(fftshift(A)));


y=filter(b,1,y0);
figure;
plot(y);

%%High pass filter:
b2 = fir1(200,Wn,'high');
 [A2,~] = freqz(b2, 1, 200, "whole");
figure;
plot(f, abs(fftshift(A2)));


%%Applied high pass to initial signal:
 
 %%SERGE and Roland Master EES%%
 y2=filter(b2,1,y0);
plot(y2);

%%Apply both low pass and then high pass to our signal:
s=filter(b2,1,y);
figure;
 plot(s);


%%Use commands butter or ellip;
[C,D]= butter(4,100/(1000/2));
 fqc = linspace(-500, 500, 1000);
 [H,~] = freqz(C, D, 1000, "whole");
 figure;
 plot(fqc,fftshift(abs(H)));


 z=filter(C,D,y0);
 figure;
plot(z);


%%High pass butter
 [C2,D2]= butter(4,100/(1000/2),"high")
 [H2,~] = freqz(C2, D2, 1000, "whole")
 plot(fqc,fftshift(abs(H2)));
 
 %%Applied high pass butter 
z2=filter(C2,D2,y0);
figure;
plot(z2)

%%Applied both filters to x:
z3=filter(C2,D2,z);
figure;
plot(z3);




%%All the signal subplotted
 subplot (431);
 plot(x0);title("raw signal");
plot(y0);title("raw signal sampled");
subplot(432);
plot(y0);title("raw signal sampled");
subplot(433);
 plot(f, abs(fftshift(A))); title("FIR low pass filter");
subplot(434);
plot(y);title("low pass filter applied ");
subplot(435);
plot(f, abs(fftshift(A2))); title ("high pass filter");
subplot(436)
plot(s);title("final signal with low pass and high pass filters applied");
subplot(437);
 plot(fqc,fftshift(abs(H)));title("IIR low pass filter");
 subplot(438);
plot(z);title("IIR low pass applied to initial signal");
 subplot(439);
 plot(fqc,fftshift(abs(H2)));
 title("IIR high pass filter");
 subplot(441);
 plot(z3);title("Final IIR filtered signal with low pass and high pass");


