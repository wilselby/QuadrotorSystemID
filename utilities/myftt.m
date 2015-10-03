
function [f, amp] = myftt(data, tsamp, plot)

Fs = 1/tsamp;
T = tsamp;
L = length(data);
x = data;

NFFT = 2^nextpow2(L);
Y = fft(x,NFFT)/L;
f = Fs/2*linspace(0,1,NFFT/2+1);

amp = 2*abs(Y(1:NFFT/2+1));
if(plot)
%     figure();
    loglog(f,amp);
    title('Single-Sided Amplitude Spectrum');
    xlabel('Frequency (Hz)');
    ylabel('Acceleration (m/s^2)');
end