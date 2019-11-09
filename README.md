# Bat-Sonar
Bat like sonar sensor that can track multiple targets and estimate angle of arrival using chirps and cross correlation in near real time.  Code for a ARM Cortex signal processing project I completed and published on hackaday.io. 


<img src="https://github.com/filipmu/Bat-Sonar/blob/master/images/Summary%20image.png" width="900" height="auto" />

## Video
This video shows the sensor in action.  The left graph shows the identified targets.  The size of the circles indicates the strength, and the location on the graph is their measured location.  The graph on the right shows the cross correlation signal vs distance.  The first step is pressing a switch on the board to calibrate and baseline the sensor.  Then the video shows the sensor identifying various targets.:https://vimeo.com/250041155


## How it works
The approach works natively on the 1-bit PDM output of MEMS microphones.  Since the sample rate is 4MHz, very accurate phase comparison can be done. This approach takes advantage of how the cross correlation problem simplifies when its matching a simple square wave chirp with a 1-bit PDM stream. In this case its a sum of delays with terms reflecting the signal level changes in the chirp  and delays reflecting the length of each pulse in the chirp.  This reduces the computational complexity significantly.  Note that its done in the time domain, not using the FFT as is typical with higher resolution digital samples.  The Cortex M4 has SIMD (Single Instruction Multiple Data) and DSP instructions that I carefully tuned to maximize computation speed. The theory is in this paper: :http://www.kurosawa.ip.titech.ac.jp/publications/papers/ieice08sh.pdf
 
 A description of how it works can be found at :https://hackaday.io/project/29512-bat-sonar/details


