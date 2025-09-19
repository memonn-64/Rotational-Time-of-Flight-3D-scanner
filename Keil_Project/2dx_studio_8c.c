
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"





#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

// initialization for LED ports (according to student id number)
void PortF_Init(void){
  
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; 					// Activate the clock for Port F
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){}; // Wait for the clock to stabilize 
  GPIO_PORTF_DIR_R=0b00010001;											// Set PF0 and PF4 as digital outputs
  GPIO_PORTF_DEN_R=0b00010001;											 // Enable digital functionality on PF0, PF4
  return;
}

void PortN_Init(void){
  
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Activate the clock for Port N
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){}; // Wait for the clock to stabilize 
  GPIO_PORTN_DIR_R=0b00000110;// Set PN1 and PN2 as digital outputs
  GPIO_PORTN_DEN_R=0b00000110; // Enable digital functionality on PN1
  // Return from the function
  return;
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//Stepper motor port
void PortH_Init (void){
  
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;							// Activate the clock for Port H
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; 		// Wait for the clock to stabilize 
  GPIO_PORTH_DIR_R = 0b00001111;												// Set PH0, PH1, PH2, and PH3 as outputs
  GPIO_PORTH_DEN_R = 0b00001111;												// Enable digital functionality on PH0, PH1, PH2, and PH3
  // Return from the function
  return;
}

// Global variable number of falling edges when button is pressed.
volatile unsigned long Falling_Edges = 0;

// Creating an onboard LED button Port J1
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor
}

// This function shuts down the VL53L1X sensor by driving its XSHUT pin low.
// XSHUT is an active-low shutdown input, which the board pulls up to VDD to enable the sensor by default.
// Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
  
    GPIO_PORTG_DIR_R |= 0x01;					// Make PG0 an output
    GPIO_PORTG_DATA_R &= 0b11111110;	// Set PG0 to 0 to drive XSHUT to low)
    FlashAllLEDs();										
		SysTick_Wait10ms(10);							
    GPIO_PORTG_DIR_R &= ~0x01;				// Make PG0 an input (HiZ)
}


// Spinning stepper motor for the scan
void CW_spin(void)
	{ //// does a full 360 turn (512 steps for mottor)
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1); 
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110; 
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
	//Four steps to do full step rotation
}

//Function to rotate stepper motor to home
void CCW_spin (void) {
	GPIO_PORTF_DATA_R ^= 0b00000001; // PortF0 (LED4) for additional status (led is lit while returning to home position)
	for (int i = 0; i < 512; i++)
	{	// does a full 360 turn (512 steps for mottor)
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);
	}
	GPIO_PORTF_DATA_R ^= 0b00000001; //Toggle Led4 (off) when returned back to home position 
}
				

// parameters positions and status of motor
int position  = 0;
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;
int flag = 0; 						// to know to call CW_spin or CCW_spin 


int main(void)
	{
	uint8_t byteDate = 0x00;
	uint8_t id, name = 0x00;
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize fucntions 
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	PortH_Init ();
	I2C_Init();
	UART_Init();
	PortJ_Init();	
	PortF_Init();
	PortN_Init();
		
	GPIO_PORTF_DATA_R = 0b00000000;

	// initializes the ToF sensor by booting it up and clearing any interrupts
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }

	UART_printf("ToF sensor booted.\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); // clear interrupt has to be called to enable next interrupt
	
  // This function must to be called to initialize the sensor with the default setting  
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	int delay = 1; 
	int position  = 0;
	int step = 32; // angle = 11.25 , steps = 512/(360/angle) , 
	double angle = 0;
	

	// Get the Distance Measures 
	// While loop to consistenly get measuremnt from ToF
	while(1)
		{
		SysTick_Wait10ms(1);
		GPIO_PORTN_DATA_R ^= 0b00000100;
		
		//check if onboard button is pressed
		if((GPIO_PORTJ_DATA_R&0x1)==0)
		{
			SysTick_Wait10ms(30);
			flag ^= 1; //toggle the flag 
		}
		
		//wait until the ToF sensor's data is ready 
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
      
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0; // makes sure to read data from the sensor until it is available.
	  
		if (flag == 1)
			{ 
			// if flag is one, does the rotation	
			CW_spin(); 
			
			// Read data values from Tof Sensor
			if (position %step == 0 && position !=0){ 
				status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance position ue
				status = VL53L1X_GetSignalRate(dev, &SignalRate);
				status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
				status = VL53L1X_GetSpadNb(dev, &SpadNum);
			
			
				// FlashLED1 (since second least significatn digit is 0) every 11.25 degrees
				FlashLED1(delay);
				
				//FlashLED3 (since second least significatn digit is 0)
				FlashLED3(delay);
				
				// clear interrupt has to be called to enable next interrupt
				status = VL53L1X_ClearInterrupt(dev); 
				
				// increment the angle
				angle+= 11.25; 
				
				// print the resulted readings to UART
				sprintf(printf_buffer,"%u\r\n", Distance);
        UART_printf(printf_buffer);
				SysTick_Wait10ms(50);
			}
			
			// after one 360 degree rotation, sets position back to 
			//home and sets parameters back to 0
			if (position  == 512)
			{				
				flag ^=1;  
				position  = 0;
				angle=0; 			
				CCW_spin ();
			}
			// steps increase as mottor moves 
			position ++; 
		}
  }
}
