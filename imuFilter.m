		%Acc[0] = LPF_ACC_ALPHA * AccPrev[0] + (1.0f - LPF_ACC_ALPHA) * Acc[0];
alpha = 0.7
b = [1-alpha];
a = [1 -alpha];

[h,f] = freqz(b,a ,1024 , 1000);

plot(f, 20*log10(abs(h)))
xlabel('Frecuencia (Hz)')
ylabel('Magnitud (dB)')
%%
syms theta phi psi

Rx = [1         0                0    ;
      0         cos(phi)    -sin(phi) ;
      0         sin(phi)     cos(phi) ];
  
Ry = [cos(theta)    0       sin(theta);
      0             1           0     ;
     -sin(theta)    0       cos(theta)];
  
Rz = [cos(psi)   -sin(psi)      0     ;
      sin(psi)    cos(psi)      0     ;
        0           0           1    ];
 
R = (Rz*Ry*Rx)



%%

function  angles = ImuImplementation(freqInput,AccMatrix,GyroMatrix)
%#codegen
FUSE = imufilter('SampleRate',freqInput, 'OrientationFormat','Rotation matrix');
rotm = FUSE(AccMatrix,GyroMatrix);
angles = rotm2eul(rotm)*180/pi;
end



