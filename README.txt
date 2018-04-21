Design Project: Speech Number Recognition on an Embedded system

- Open the NumRecDP Keil project file in the MDK-ARM folder
- Make sure the target settings are for the correct board (STM32f407VG) and clock is set to 168MHz
- Compile program and upload to target (press the reset button on board to start)

Mic & UART setup:
					UART - TX line should be connected to PA2. No need to connect the RX line as program doesn’t receive any input via UART.
					MIC - Should be connected to 3V pin on board and have the output connected to the ADC input on PA1.
					Our microphone had an offset of 2000. This means that our average energy per frame of ambient noise (40 samples) was set_thresh = 160000000 (40*2000^2).
					The threshold for the VAD detection should therefor be higher than that. Our experimentally determined value was {set_thresh + 2000000} on line 127 of main.c
					Different microphones will require different values above set_thresh but a good offset to start at would be (0.5 * offset^2) this gave us the 2000000 number.

Program execution: 
					1) Wait until RED led lights up on the board. This indicates that the threshold energy of the room is calculated and voice is ready to be captured
					2) Speak loudly/firmly about 2-4 inches from the microphone. The remaining LEDs should light up indicating that speech was captured
					3) After 3 seconds the results should display via UART (TX line on discovery board)


Program Structure:
					main.c - Speech parsing for high energy (VAD) to fill speech buffer to be sent through the following functions

					mfccFunc.c - Contains the mfcc code to compute the mfcc coefficients of the captured speech (CMSIS fft should be implemented here for improvement)

					classification.c - Contains the implemented neural net weights and calculations to determine the final output (A different classification technique could be used here such as HMMs)
					New weights for a similar NN of 312 -> 12 -> 10 can just be replaced with the array found in classification.c
					Note our network is decently biased in recognizing certain numbers a certain way but that is solely due to the lack of training data and can be improved with more training.

