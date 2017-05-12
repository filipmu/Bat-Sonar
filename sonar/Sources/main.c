/* ###################################################################
**     Filename    : main.c
**     Project     : sonar
**     Processor   : MK64FN1M0VLL12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2017-01-17, 22:40, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "Pins1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "PDD_Includes.h"
#include "Init_Config.h"
/* User includes (#include below this line is not maintained by Processor Expert) */
#include <stdio.h>

  #define __ASM            __asm                                      /*!< asm keyword for GNU Compiler */
  #define __INLINE         inline                                     /*!< inline keyword for GNU Compiler */
  #define __STATIC_INLINE  static inline

__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __QADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __ASM volatile ("qadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __MUL(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __ASM volatile ("mul %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __MLA(uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result;

  __ASM volatile ("mla %0, %1, %2 %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  return(result);
}


__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __SADD16(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __ASM volatile ("sadd16 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __UQADD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __ASM volatile ("uqadd8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}


__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __USAD8(uint32_t op1, uint32_t op2)
{
  uint32_t result;

  __ASM volatile ("usad8 %0, %1, %2" : "=r" (result) : "r" (op1), "r" (op2) );
  return(result);
}

__attribute__( ( always_inline ) ) __STATIC_INLINE uint32_t __USADA8(uint32_t op1, uint32_t op2, uint32_t op3)
{
  uint32_t result;

  __ASM volatile ("usada8 %0, %1, %2, %3" : "=r" (result) : "r" (op1), "r" (op2), "r" (op3) );
  return(result);
}


//RBIT
//Reverse bit order of value

__attribute__((always_inline)) __STATIC_INLINE uint32_t __RBIT(uint32_t value)
{
  uint32_t result;
   __ASM volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}

/*
 * Notes
 *
 * Be sure to set compiler optimization to a high setting for performance of code
 *


*/


//#define CHIRP_COUNT 483
//#define CHIRP_ADD 16
#define CHIRP_COUNT 600 //should be 30MHz/freq 42khz is 714, 40khz is 750, 50khz is 600, 60khz is 500
#define CHIRP_ADD 1 //was 1  controls the amount of period length after each half wave 1=.0166 microsec
#define CHIRP_PULSES 200  //was 64
#define CHIRP_PULSES_DECAY 200  //used for the matched filter pattern.  Make it slightly larger than CHIRP_PULSES to model the decay of 40kHZ pulses in the piezo
#define CHIRP_PULSES_DECAY_MAX 1000  //max number of pulses ever expected.
#define PULSE_AMP 2047 //controls the chirp pulse amplitude 0 is 0 and 2047 is 3.3v (the max)

#define BLOCK_SIZE 7000  //30ft x 12 in = 360 in.  360in / .053 in/sample ~ 7000.  7000
#define CORR_SIZE 7000
#define TARGET_MAX 50 //size of target buffer - max number of targets to store
#define TARGET_BLANK 300 //number of initial correlation samples to ignore  - at some point make this a function of the pulse amp and type of chirp
#define TARGET_THRESHOLD 100
uint32_t memsbuffer0[BLOCK_SIZE];
uint32_t memsbuffer1[BLOCK_SIZE];
uint32_t memsbuffer2[BLOCK_SIZE];
uint16_t *memsbuffer2_16 = &memsbuffer2;  //pointer to memsbuffer location as a 16 bit for SPI
volatile uint16_t memsbuffer_i, memsbuffer_i16;
volatile uint16_t chirp_pulses_counter=CHIRP_PULSES;

uint8_t state=0;



//Variables that control the chirp
uint8_t chirp_add=CHIRP_ADD;
uint32_t chirp_count=CHIRP_COUNT;
uint16_t chirp_pulses=CHIRP_PULSES;
uint16_t chirp_pulses_decay=CHIRP_PULSES_DECAY;
uint16_t chirp_len; //size of chirp in 32bit words
int16_t corr_size_max; //maximum real size of correlation


//variables that can be changed at run time and will regenerate the chirp and matched filter
volatile uint8_t new_chirp_flag=1;
volatile uint8_t new_chirp_add=CHIRP_ADD;
volatile uint32_t new_chirp_count=CHIRP_COUNT;
volatile uint16_t new_chirp_pulses=CHIRP_PULSES;
volatile uint16_t new_chirp_pulses_decay=CHIRP_PULSES_DECAY;
volatile uint16_t new_pulse_amp=PULSE_AMP;//controls amplitude of chirp 2048 to 4095 is 0 to 3.3v at input of amp
volatile int16_t target_threshold=TARGET_THRESHOLD;


//__attribute__((section (".m_data_1FFF0000")))

uint16_t timestamp=0; //time stamp used to indicate the frame
int16_t  corr_calc0,corr_calc1, corr_calc2, corr_calc3;
int16_t corr0[CORR_SIZE] __attribute__((section (".m_data_1FFF0000")));
int16_t corr1[CORR_SIZE] __attribute__((section (".m_data_1FFF0000")));
int32_t corr_DC;
uint16_t corr_i;



int32_t cs0,cs1, cs2, cs3;
int32_t cplus0,cplus1,cplus2,cplus3;
int32_t cminus0,cminus1,cminus2,cminus3;
uint32_t a0,a1,a2,a3;



uint8_t count;

int16_t target_d[TARGET_MAX],target_s[TARGET_MAX], target_s1[TARGET_MAX],target_a[TARGET_MAX],target_score,target_score0,target_score1;
int8_t target_i,target_i_max;

int32_t fine_corr0[TARGET_MAX][64];	//microphone 0 target correlation
int32_t fine_corr1[TARGET_MAX][64];  //microphone 1 target correlation

int32_t inda;
int32_t ind_pos[CHIRP_PULSES_DECAY_MAX];
int32_t ind_neg[CHIRP_PULSES_DECAY_MAX];

int16_t i;  //general index into filter terms
uint16_t i_neg, i_pos;
int32_t j,ind,indl,indr;
int8_t sign,sign_last,delta_sign;
uint32_t ct, cum_ct,loop;



int16_t m0_max,m1_max, m0_max_loc, m1_max_loc;


volatile uint8_t output_flag=1;

#define PIN_PTC2 0x0004
#define PIN_PTC3 0x0008
#define PIN_PTC7 0x0080



void PIT0_Int(void)
{
	//Handle interrupt for timer 0
	PIT_TFLG0=1;




	if(chirp_pulses_counter != 0)
	{
	//Advance buffer points for DAC so that chirp output changes state
	DAC0_C0 = (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK | DAC_C0_DACTRGSEL_MASK | DAC_C0_DACSWTRG_MASK);

	//set max buffer pointer to 1 so that we don't output the 50% value
	DAC0_C2 = (DAC0_C2 & 0xF0) | 0x01;

	//update the period of the PIT0
	//each tick is 0.016667 usec
	//start at 61.98khz so for 2 pulses in that cycle so 483+1
	//want to finish around 20khz so 50 usec so 1499+1
	//so increment by 16

	PIT_LDVAL0 = PIT_LDVAL0 +chirp_add;
	chirp_pulses_counter = chirp_pulses_counter -1;
	}
	else
	{
		//disable the PIT0
		PIT_TCTRL0 = PIT_TCTRL0 & (~PIT_TCTRL_TEN_MASK);
		//set buffer read pointer value to 2 to output 50%
		//set max buffer pointer to 2
		DAC0_C2 = (DAC_C2_DACBFRP(2) | DAC_C2_DACBFUP(2));
		//Advance buffer points for DAC so that chirp output changes state
		//DAC0_C0 = (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK | DAC_C0_DACTRGSEL_MASK | DAC_C0_DACSWTRG_MASK);
	}


}


void SPI0_Int(void)
{
uint16_t buff;

	//clear interrupt flag
	//Clear the end of queue flag
	SPI0_SR=SPI0_SR | SPI_SR_EOQF_MASK;



if(SPI0_SR | SPI_SR_RFDF_MASK) //check for read buffer flag
{
	buff=(uint16_t) SPI0_POPR;
	if(memsbuffer_i16 < BLOCK_SIZE*2)
		{
		memsbuffer2_16[memsbuffer_i16]=buff;


		//add a loop to load more than one in case - need to check size of FIFO
		//SPI0_SR bits 7-4
		memsbuffer_i16 = memsbuffer_i16 +1;
		}
	else
	{

		//flush buffers and halt SPI
		//SPI0_MCR = SPI0_MCR | (SPI_MCR_CLR_RXF_MASK | SPI_MCR_HALT_MASK);
	}
	SPI0_SR=SPI0_SR | SPI_SR_RFDF_MASK;

	if(SPI0_SR | SPI_SR_TFFF_MASK) //check for read buffer flag
		{
		//fill output buffer
			//indicate that not at end of transfer
			//indicate continuous chip select enable for continuous transfer
			SPI0_PUSHR = SPI_PUSHR_CONT_MASK;

			SPI0_SR=SPI0_SR | SPI_SR_TFFF_MASK;
		}

	}




}


void SPI0_slave_Int(void)
{
uint32_t buff;

	//clear interrupt flag
	//Clear the end of queue flag
	SPI0_SR=SPI0_SR | SPI_SR_EOQF_MASK;

	//fill output buffer
		//indicate that not at end of transfer
		//indicate continuous chip select enable for continuous transfer
		//SPI0_PUSHR = SPI_PUSHR_CONT_MASK;

if(SPI0_SR | SPI_SR_RFDF_MASK) //check for read buffer flag
{
	buff=(uint16_t) SPI0_POPR;
	if(memsbuffer_i16 < BLOCK_SIZE*2)
		{
		memsbuffer2_16[memsbuffer_i16]=__RBIT(buff) >> 16;  //flip order of bits since can only load MSB first with slave SPI


		//add a loop to load more than one in case - need to check size of FIFO
		//SPI0_SR bits 7-4
		memsbuffer_i16 = memsbuffer_i16 +1;
		}
	else
	{

		//flush buffers and halt SPI
		SPI0_MCR = SPI0_MCR | (SPI_MCR_CLR_RXF_MASK | SPI_MCR_HALT_MASK);
	}
	SPI0_SR=SPI0_SR | SPI_SR_RFDF_MASK;

	//if(SPI0_SR | SPI_SR_TFFF_MASK) //check for read buffer flag
	//	{
		//fill output buffer
			//indicate that not at end of transfer
			//indicate continuous chip select enable for continuous transfer
	//		SPI0_PUSHR = SPI_PUSHR_CONT_MASK;

	//		SPI0_SR=SPI0_SR | SPI_SR_TFFF_MASK;
	//	}

	}




}



void I2S_Int(void)
{

	uint32_t buff0,buff1;
	//GPIOC_PSOR = PIN_PTC7; //set pin 7
	I2S0_RCSR|=0x40000; //clear error flag


	buff0 = I2S0_RDR0;
	buff1 = I2S0_RDR1;

	if(memsbuffer_i < BLOCK_SIZE)
	{
		memsbuffer0[memsbuffer_i]=buff0;
		memsbuffer1[memsbuffer_i]=buff1;
		memsbuffer_i = memsbuffer_i +1;

	}
	else
	{
		//disable, reset fifos, turn off interrupts, but keeps bit clock on
		I2S0_RCSR =  I2S_RCSR_FR_MASK| I2S_RCSR_WSF_MASK| I2S_RCSR_SEF_MASK| I2S_RCSR_FEF_MASK| I2S_RCSR_FWF_MASK| I2S_RCSR_BCE_MASK;
	}

	//GPIOC_PCOR = PIN_PTC7; //clear pin 7
}








const uint32_t masklt[32] =
     {0x00, 0x1, 0x3, 0x7, 0xF, 0x1F, 0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF,
      0x1FFFF, 0x3FFFF, 0x7FFFF, 0xFFFFF, 0x1FFFFF, 0x3FFFFF, 0x7FFFFF, 0xFFFFFF, 0x1FFFFFF, 0x3FFFFFF, 0x7FFFFFF, 0xFFFFFFF,
      0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF};

// const uint32_t maskx[32] =
//      {0xFFFFFFFF,0x7FFFFFFF,0x3FFFFFFF,0x1FFFFFFF,0xFFFFFFF,0x7FFFFFF,0x3FFFFFF,0x1FFFFFF,
//      0xFFFFFF, 0x7FFFFF,0x3FFFFF,0x1FFFFF,0xFFFFF,0x7FFFF,0x3FFFF,0x1FFFF,
//      0xFFFF,0x7FFF,0x3FFF,0x1FFF,0xFFF,0x7FF,0x3FF,0x1FF,0xFF,0x7F,0x3F,0x1F,0xF,0x7,0x3,0x1};


const uint32_t maskrt[32] =
     {0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFC, 0xFFFFFFF8, 0xFFFFFFF0, 0xFFFFFFE0, 0xFFFFFFC0, 0xFFFFFF80, 0xFFFFFF00,
      0xFFFFFE00,0xFFFFFC00, 0xFFFFF800, 0xFFFFF000, 0xFFFFE000,0xFFFFC000, 0xFFFF8000, 0xFFFF0000,
      0xFFFE0000, 0xFFFC0000, 0xFFF80000, 0xFFF00000, 0xFFE00000, 0xFFC00000, 0xFF800000, 0xFF000000,
     0xFE000000, 0xFC000000, 0xF8000000, 0xF0000000, 0xE0000000, 0xC0000000, 0x80000000};

uint8_t SWAR(uint32_t i)
{
 i = i - ((i >> 1) & 0x55555555);
 i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
 return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}







int32_t SWARquadaccum(uint32_t acc,uint32_t i,uint32_t j,uint32_t k,uint32_t l)
{
 i = i - ((i >> 1) & 0x55555555);
 j = j - ((j >> 1) & 0x55555555);
 k = k - ((j >> 1) & 0x55555555);
 l = l - ((j >> 1) & 0x55555555);
 i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
 j = (j & 0x33333333) + ((j >> 2) & 0x33333333);
 k = (k & 0x33333333) + ((k >> 2) & 0x33333333);
 l = (l & 0x33333333) + ((l >> 2) & 0x33333333);


//each byte already has sum of set bits in the 8 bits

i= (i + (i >> 4)) & 0x0F0F0F0F;
j= (j + (j >> 4)) & 0x0F0F0F0F;
k= (k + (k >> 4)) & 0x0F0F0F0F;
l= (l + (l >> 4)) & 0x0F0F0F0F;

return(   __UQADD8(__UQADD8(__UQADD8(i, j),k),l)   );   //sum using a simd instruction


}


uint32_t SWARsimd(uint32_t i)
{
//GPIOC_PSOR = PIN_PTC7; //set pin 7
 //i = i - ((i >> 1) & 0x55555555);
	i = i - ((i >> 1) & 0x55555555);
 i = (i & 0x33333333) + ((i >> 2) & 0x33333333);

//each byte already has sum of set bits in the 8 bits
 //sum using a simd instruction

 //i=(i + (i >> 4)) & 0x0F0F0F0F;

 //GPIOC_PCOR = PIN_PTC7; //clear pin 7
return((i + (i >> 4)) & 0x0F0F0F0F);



}











uint8_t SWARx2(uint32_t i)
{
 i = i - ((i >> 1) & 0x55555555);
 i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
 //return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 23;

 return __MUL( ((i + (i >> 4)) & 0x0F0F0F0F) , 0x01010101) >> 23;

}


/*

int32_t SWARx2dual(uint32_t i,j)
{
 i = i - ((i >> 1) & 0x55555555);
 j = j - ((j >> 1) & 0x55555555);
 i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
 j = (j & 0x33333333) + ((j >> 2) & 0x33333333);
 //return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 23;

//need to use packing instruction and create a positive and neg sign function
 return (__MUL( ((i + (i >> 4)) & 0x0F0F0F0F) , 0x01010101) >> 7) | (__MUL( ((j + (j >> 4)) & 0x0F0F0F0F) , 0x01010101) >> 23);
}

*/



int8_t SWAR_signed(uint32_t i)
{
 i = i - ((i >> 1) & 0x55555555);
 i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
 return ((((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 23)-32;
}

int8_t SWARx2_signed(uint32_t i)
{
 i = i - ((i >> 1) & 0x55555555);
 i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
 return ((((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 22)-64;
}





 uint8_t b_to_c[256] = {
        0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
        4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
    };

/*
uint8_t SWARx2(uint32_t i)
{
 return( (b_to_c[i & 0xff]+b_to_c[(i >> 8) & 0xff]+b_to_c[(i >> 16) & 0xff]+b_to_c[(i >> 24) & 0xff])>>1);
}

*/





uint32_t bit_lookup(uint32_t * a, uint32_t t)
{

 uint32_t wd=t >> 5;  //get starting word


uint32_t bt=t&31; //get bit for mask

uint32_t templ,tempr;

    // tempr=(a[wd] & maskrt[bt]) >> bt;

    // templ=(a[wd+1] & masklt[bt])<< (32-bt);

if (bt==0)
	return(a[wd]);
else
{
     tempr=(a[wd] ) >> bt;

     templ=((a[wd+1] )<< (!bt))<<1;

     return(tempr | templ);
}




}



void debug_call(int16_t num)
{
	if(j==0 || j ==32)
	{
		printf("%d, %d, %d , %d, %d, %d, %dl, %dl\r\n",num,target_i,j,ind,indl,indr,fine_corr0[target_i][j],fine_corr1[target_i][j]  );
	}


}




/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/

	PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/



  //Enable the DAC

    SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK;


    //enable DAC and use VDDA as reference (DACREF_2)
    //use a software trigger
    DAC0_C0 = (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK | DAC_C0_DACTRGSEL_MASK);

    //define buffer work mode as normal mode
    //use buffer read pointer
    DAC0_C1 = (DAC_C1_DACBFMD(0) | DAC_C1_DACBFEN_MASK );

    //set buffer read pointer value to 2 to output 50%
    //set max buffer pointer to 2
    DAC0_C2 = (DAC_C2_DACBFRP(2) | DAC_C2_DACBFUP(2));






  /* Write your code here */
  /* For example: for(;;) { } */

    //enable the SPI
    //SPI0_Init();
    //SPI1_Init();
    //SPI2_Init();

    I2S0_Init();
    PIT_Init();





state=0;
count=0;

GPIOC_PDDR= PIN_PTC2 | PIN_PTC3 | PIN_PTC7;  //set port data direction to output.



  for(;;){







	  if (state==0)
	  {

		  //Zero the target array
		  for(target_i=TARGET_MAX-1; target_i>=0 ;--target_i)
		  {
			  target_d[target_i]=0;
			  target_s[target_i]=0;
		  }

		  target_i=0;


		  if(new_chirp_flag)  //reset chirp data
		  {
			  chirp_add=new_chirp_add;
			  chirp_count=new_chirp_count;
			  chirp_pulses=new_chirp_pulses;
			  chirp_pulses_decay=new_chirp_pulses_decay;

			  //populate output buffer

			  //let output swing from 50% level



			  //Set the max level of the DAC
				DAC0_DAT0H = (uint8_t) ((new_pulse_amp+2048) >> 8)  & 0x0F;
				DAC0_DAT0L = (uint8_t) (new_pulse_amp+2048) & 0xFF;

				//Set the min level of the DAC
				DAC0_DAT1H = (uint8_t) ((4095-2048-new_pulse_amp) >> 8)  & 0x0F;  //set to inverse of max value
				DAC0_DAT1L = (uint8_t) (4095-2048-new_pulse_amp) & 0xFF;

				//Set the mid level
				//set second word to 1.65v (50%)
				DAC0_DAT2H = 0x08;
				DAC0_DAT2L = 0x00;



			  //calculate new chirp length with buffer to ensure no overflow - buffer should be >4 due to loop with corr_i incrementing 4 times



			  /*
			   * initialize the indexing array for the matched filter
			   * see paper "Cross-Correlation by Single-bit Signal Processing for Ultrasonic Distance Measurement" by
			   * Shinnosuke HIRATA, Minoru Kuribayashi KUROSAWA, and Takashi KATAGIRI
			   * IEICE TRANS. FUNDAMENTALS, VOL.E91–A, NO.4 APRIL 2008
			   *
			   *We want 2 indexes, one for positive terms and one for negative terms to speed math
			   */

			  //Simulate the PIT counter that is used to generate the linear period chirp, it is a periodic counter that increases in period.
			  //Clock rate is 60MHz of the counter
			  //  Input samples from the mems microphones come in at 4MhZ bit rate
			  ct=chirp_count;  //start with the number of counts in the first pulse
			  cum_ct=0;  //keep track of cumulative counts of simulated PIT counter

			  i_pos=0; // term index for positive terms - we won't store the first term
			  i_neg=0; // term index for negative terms
			  sign_last=0;  //sign of initial zero signal level
			  sign=1;	//sign of first pulse
			  delta_sign=sign_last-sign;  //First term is at (t+0) and the sign of first term is based on the transition
			  i=0;  //keep track of terms will not store first term
			  for (loop=(chirp_pulses_decay); loop>0 ; --loop)  //loop through and calculate terms for pulses 2,...,chirp_pulses_decay
			  	{
					cum_ct=cum_ct+ct; //accumulate the time duration adding the next pulse

					//Calculate the next pulse width and signs
					sign_last = sign;
					sign = - sign;  //alternate the sign to simulate the pulse phase change
					delta_sign=sign_last-sign; //calculate the sign of the term (+2 or -2)

				  	ind=(cum_ct+7)/15;  //calculate the index into the memsbuffer - 60MHz clocked chirp counters and 4MHz input clock yields 60/4 = 15 chirp counts per input bit
				  	inda = (ind << 16)  |  ((ind & 31) << 8) | (32-(ind & 31));  //encode the index to the 32 bit word as well as bit shift amounts into the array

					if(delta_sign>0)
					{
						ind_pos[i_pos]=inda;
						i_pos=i_pos+1;
					}
					else
					{
						ind_neg[i_neg]=inda;
						i_neg=i_neg+1;

					}

					ct = ct + chirp_add;  //increase the period of the next pulse by the chirp_add amount
					i=i+1;

			  	}

			  i_neg=i_neg-1;  //holds the max index for negative terms
			  i_pos=i_pos-1;  //holds the max index for positive terms

			  chirp_len=ind/32+10;
			  corr_size_max = CORR_SIZE-chirp_len;
			  new_chirp_flag=0;

		  }




		  //output one on timing pin for mems capture
		  GPIOC_PSOR= PIN_PTC3;

		  /*
		  //start mems microphone data collection on SPI for microphone #2
		  memsbuffer_i16=0;
		  SPI0_MCR = SPI0_MCR | SPI_MCR_CLR_RXF_MASK;  //clear input buffer
		  SPI0_SR = SPI0_SR | SPI_SR_EOQF_MASK; //Clear the end of queue flag
		  */





	  //start the chirp and mems microphone data collection

		  memsbuffer_i=0; //reset index used by mems data load interrupt (not needed for DMA)


		  //Interrupt driven data load settings
	      //reset I2S fifo, interrupts, errors, but keeps bit clock on
	      I2S0_RCSR =  I2S_RCSR_FR_MASK| I2S_RCSR_WSF_MASK| I2S_RCSR_SEF_MASK| I2S_RCSR_FEF_MASK| I2S_RCSR_FWF_MASK| I2S_RCSR_FRF_MASK | I2S_RCSR_BCE_MASK;
	      //enable I2S receive transfer, interrupts, bit clock on
	      I2S0_RCSR =  I2S_RCSR_RE_MASK | I2S_RCSR_BCE_MASK | I2S_RCSR_DBGE_MASK | I2S_RCSR_FRIE_MASK;

	      //start SPI read transfer
	      //SPI0_MCR = SPI0_MCR & (~SPI_MCR_HALT_MASK);


	      //reset counter
	      PIT_LDVAL0 = chirp_count;
	      //reset chirp pulses
	      chirp_pulses_counter = chirp_pulses;

	      //enable the PIT0
	      PIT_TCTRL0 = PIT_TCTRL0 | PIT_TCTRL_TEN_MASK;



	      corr_i=0;

	      target_i=0;


	      state=2;

	  }






	  //check if the data load is complete
	 // if(memsbuffer_i16 == BLOCK_SIZE*2)
		//  {
		// GPIOC_PCOR = PIN_PTC3; //clear pin 3
		// state=3; //debug

		//  }

	  //if(state < 4 && state > 1)
	  if(state ==2)
	  {



		  if (((corr_i+chirp_len) <= (memsbuffer_i)))  //start processing the correlation since we have at least a full chirp length plus buffer ahead of us to process

		  {

		  GPIOC_PSOR= PIN_PTC2;  //make pin 2 high to indicate starting
		  //check how much timing changes after data loaded
		  if(corr_i & 0x100)
			  GPIOC_PSOR = PIN_PTC7; //set pin 7
		  else
			  GPIOC_PCOR = PIN_PTC7; //clear pin 7





		  // Do mems channel 0


		  		  cminus0=0;
		  		  cminus1=0;
		  		  cminus2=0;
		  		  cminus3=0;

		  		  cplus0=0;
		  		  cplus1=0;
		  		  cplus2=0;
		  		  cplus3=0;


		  		  //ct=714;
		  		  //cum_ct=ct;
				  for (i=0; i<=i_pos ; ++i)  //Do positive 2x factor terms
				  {
					  ind=ind_pos[i] >> 21;
					  indl=(ind_pos[i] & 0xFF);
					  indr=(ind_pos[i] & 0xFF00)>>8;


					  //inda[i] = (ind << 16)  |  ((ind & 31) << 8) | (32-(ind & 31));


				  a0=(memsbuffer0[corr_i + 1+ ( ind)] << (indl)) | (memsbuffer0[corr_i + (ind)] >> (indr));
				  a1=(memsbuffer0[corr_i + 2+ ( ind)] << (indl)) | (memsbuffer0[corr_i + 1+(ind)] >> (indr));
				  a2=(memsbuffer0[corr_i + 3+ ( ind)] << (indl)) | (memsbuffer0[corr_i + 2+(ind)] >> (indr));
				  a3=(memsbuffer0[corr_i + 4+ ( ind)] << (indl)) | (memsbuffer0[corr_i +3+ (ind)] >> (indr));

		  		  cplus0 = __USADA8(SWARsimd(a0),0,cplus0);
		  		  cplus1 = __USADA8(SWARsimd(a1),0,cplus1);
		  		  cplus2 = __USADA8(SWARsimd(a2),0,cplus2);
		  		  cplus3 = __USADA8(SWARsimd(a3),0,cplus3);

				  }

				  for (i=0; i<i_neg ; ++i)  //Do negative 2x factor terms
				  {
				  ind=ind_neg[i] >> 21;
				  indl=(ind_neg[i] & 0xFF);
				  indr=(ind_neg[i] & 0xFF00)>>8;


				  a0=(memsbuffer0[corr_i + 1+ ( ind)] << (indl)) | (memsbuffer0[corr_i + (ind)] >> (indr));
				  a1=(memsbuffer0[corr_i + 2+ ( ind)] << (indl)) | (memsbuffer0[corr_i + 1+(ind)] >> (indr));
				  a2=(memsbuffer0[corr_i + 3+ ( ind)] << (indl)) | (memsbuffer0[corr_i + 2+(ind)] >> (indr));
				  a3=(memsbuffer0[corr_i + 4+ ( ind)] << (indl)) | (memsbuffer0[corr_i +3+ (ind)] >> (indr));


		  		  cminus0 = __USADA8(SWARsimd(a0),0,cminus0);
		  		  cminus1 = __USADA8(SWARsimd(a1),0,cminus1);
		  		  cminus2 = __USADA8(SWARsimd(a2),0,cminus2);
		  		  cminus3 = __USADA8(SWARsimd(a3),0,cminus3);

		  		  }




				  ind=ind_neg[i_neg] >> 21;
				  indl=(ind_neg[i_neg] & 0xFF);
				  indr=(ind_neg[i_neg] & 0xFF00)>>8;

				  //do a negative 1x term along with the 0 1x term
				  a0=(memsbuffer0[corr_i + 1+ ( ind)] << (indl)) | (memsbuffer0[corr_i + (ind)] >> (indr));
				  a1=(memsbuffer0[corr_i + 2+ ( ind)] << (indl)) | (memsbuffer0[corr_i + 1+(ind)] >> (indr));
				  a2=(memsbuffer0[corr_i + 3+ ( ind)] << (indl)) | (memsbuffer0[corr_i + 2+(ind)] >> (indr));
				  a3=(memsbuffer0[corr_i + 4+ ( ind)] << (indl)) | (memsbuffer0[corr_i +3+ (ind)] >> (indr));


		  		  cs0=__USADA8( __UQADD8(SWARsimd(memsbuffer0[corr_i]), SWARsimd(a0)),0,cminus0 <<1);
		  		  cs1=__USADA8( __UQADD8(SWARsimd(memsbuffer0[corr_i+1]), SWARsimd(a1)),0,cminus1 <<1);
		  		  cs2=__USADA8( __UQADD8(SWARsimd(memsbuffer0[corr_i+2]), SWARsimd(a2)),0,cminus2 <<1);
		  		  cs3=__USADA8( __UQADD8(SWARsimd(memsbuffer0[corr_i+3]), SWARsimd(a3)),0,cminus3 <<1);


		  		  corr_calc0 =(cplus0 <<1)-cs0; //multiply both first terms by 2 correction for using SWAR that is unsigned
		  		  corr_calc1 =(cplus1 <<1)-cs1; //multiply both first terms by 2 correction for using SWAR that is unsigned
		  		  corr_calc2 =(cplus2 <<1)-cs2; //multiply both first terms by 2 correction for using SWAR that is unsigned
		  		  corr_calc3 =(cplus3 <<1)-cs3; //multiply both first terms by 2 correction for using SWAR that is unsigned




		  		  corr0[corr_i]=corr_calc0;
		  		  corr0[corr_i+1]=corr_calc1;
		  		  corr0[corr_i+2]=corr_calc2;
		  		  corr0[corr_i+3]=corr_calc3;

		  		  /*
				  if (corr_i > TARGET_BLANK)
				  		  {

				  			  //target_score= 2*(corr_calc1-corr_calc0-corr_calc2)+corr[corr_i-6]+corr_calc4; //calculate sombrero fit
					  	  	  target_score= corr_calc0-corr0[corr_i-1];
				  			  if((target_score > target_threshold) && (target_i < TARGET_MAX))
				  		  		  {
				  				  target_d[target_i]=corr_i;
				  				  target_s[target_i]=target_score;
				  				  target_i=target_i + 1;

				  		  		  }
					  	  	  target_score= corr_calc1-corr_calc0;
				  			  if((target_score > target_threshold) && (target_i < TARGET_MAX))
				  		  		  {
				  				  target_d[target_i]=corr_i+1;
				  				  target_s[target_i]=target_score;
				  				  target_i=target_i + 1;

				  		  		  }
					  	  	  target_score= corr_calc2-corr_calc2;
				  			  if((target_score > target_threshold) && (target_i < TARGET_MAX))
				  		  		  {
				  				  target_d[target_i]=corr_i+2;
				  				  target_s[target_i]=target_score;
				  				  target_i=target_i + 1;

				  		  		  }
					  	  	  target_score= corr_calc3-corr_calc2;
				  			  if((target_score > target_threshold) && (target_i < TARGET_MAX))
				  		  		  {
				  				  target_d[target_i]=corr_i+3;
				  				  target_s[target_i]=target_score;
				  				  target_i=target_i + 1;

				  		  		  }
				  		  }

*/






				  // Do mems channel 1


				  cminus0=0;
				  cminus1=0;
				  cminus2=0;
				  cminus3=0;

				  cplus0=0;
				  cplus1=0;
				  cplus2=0;
				  cplus3=0;


				  //ct=714;
				  //cum_ct=ct;
				  i=0;

				  for (i=0; i<=i_pos ; ++i)  //Do positive 2x factor terms
				  {
					  ind=ind_pos[i] >> 21;
					  indl=(ind_pos[i] & 0xFF);
					  indr=(ind_pos[i] & 0xFF00)>>8;


					  //inda[i] = (ind << 16)  |  ((ind & 31) << 8) | (32-(ind & 31));


				  a0=(memsbuffer1[corr_i + 1+ ( ind)] << (indl)) | (memsbuffer1[corr_i + (ind)] >> (indr));
				  a1=(memsbuffer1[corr_i + 2+ ( ind)] << (indl)) | (memsbuffer1[corr_i + 1+(ind)] >> (indr));
				  a2=(memsbuffer1[corr_i + 3+ ( ind)] << (indl)) | (memsbuffer1[corr_i + 2+(ind)] >> (indr));
				  a3=(memsbuffer1[corr_i + 4+ ( ind)] << (indl)) | (memsbuffer1[corr_i +3+ (ind)] >> (indr));

				  cplus0 = __USADA8(SWARsimd(a0),0,cplus0);
				  cplus1 = __USADA8(SWARsimd(a1),0,cplus1);
				  cplus2 = __USADA8(SWARsimd(a2),0,cplus2);
				  cplus3 = __USADA8(SWARsimd(a3),0,cplus3);

				  }

				  for (i=0; i<i_neg ; ++i)  //Do negative 2x factor terms
				  {
				  ind=ind_neg[i] >> 21;
				  indl=(ind_neg[i] & 0xFF);
				  indr=(ind_neg[i] & 0xFF00)>>8;


				  a0=(memsbuffer1[corr_i + 1+ ( ind)] << (indl)) | (memsbuffer1[corr_i + (ind)] >> (indr));
				  a1=(memsbuffer1[corr_i + 2+ ( ind)] << (indl)) | (memsbuffer1[corr_i + 1+(ind)] >> (indr));
				  a2=(memsbuffer1[corr_i + 3+ ( ind)] << (indl)) | (memsbuffer1[corr_i + 2+(ind)] >> (indr));
				  a3=(memsbuffer1[corr_i + 4+ ( ind)] << (indl)) | (memsbuffer1[corr_i +3+ (ind)] >> (indr));


				  cminus0 = __USADA8(SWARsimd(a0),0,cminus0);
				  cminus1 = __USADA8(SWARsimd(a1),0,cminus1);
				  cminus2 = __USADA8(SWARsimd(a2),0,cminus2);
				  cminus3 = __USADA8(SWARsimd(a3),0,cminus3);


				  }

				  ind=ind_neg[i_neg] >> 21;
				  indl=(ind_neg[i_neg] & 0xFF);
				  indr=(ind_neg[i_neg] & 0xFF00)>>8;

				  //do a negative 1x term along with the 0 1x term
				  a0=(memsbuffer1[corr_i + 1+ ( ind)] << (indl)) | (memsbuffer1[corr_i + (ind)] >> (indr));
				  a1=(memsbuffer1[corr_i + 2+ ( ind)] << (indl)) | (memsbuffer1[corr_i + 1+(ind)] >> (indr));
				  a2=(memsbuffer1[corr_i + 3+ ( ind)] << (indl)) | (memsbuffer1[corr_i + 2+(ind)] >> (indr));
				  a3=(memsbuffer1[corr_i + 4+ ( ind)] << (indl)) | (memsbuffer1[corr_i +3+ (ind)] >> (indr));



		  		  cs0=__USADA8( __UQADD8(SWARsimd(memsbuffer1[corr_i]), SWARsimd(a0)),0,cminus0 <<1);
		  		  cs1=__USADA8( __UQADD8(SWARsimd(memsbuffer1[corr_i+1]), SWARsimd(a1)),0,cminus1 <<1);
		  		  cs2=__USADA8( __UQADD8(SWARsimd(memsbuffer1[corr_i+2]), SWARsimd(a2)),0,cminus2 <<1);
		  		  cs3=__USADA8( __UQADD8(SWARsimd(memsbuffer1[corr_i+3]), SWARsimd(a3)),0,cminus3 <<1);


		  		  corr_calc0 =(cplus0 <<1)-cs0; //multiply both first terms by 2 correction for using SWAR that is unsigned
		  		  corr_calc1 =(cplus1 <<1)-cs1; //multiply both first terms by 2 correction for using SWAR that is unsigned
		  		  corr_calc2 =(cplus2 <<1)-cs2; //multiply both first terms by 2 correction for using SWAR that is unsigned
		  		  corr_calc3 =(cplus3 <<1)-cs3; //multiply both first terms by 2 correction for using SWAR that is unsigned


				  corr1[corr_i]=corr_calc0;
				  corr1[corr_i+1]=corr_calc1;
				  corr1[corr_i+2]=corr_calc2;
				  corr1[corr_i+3]=corr_calc3;


				  if (corr_i > TARGET_BLANK)
				  		  {

				  			  //target_score= 2*(corr_calc1-corr_calc0-corr_calc2)+corr[corr_i-6]+corr_calc4; //calculate sombrero fit
					  	  	  //target_score0= corr0[corr_i]-corr0[corr_i-1];
					  	  	  target_score0= corr0[corr_i];
				  			  if(target_score0 > target_threshold)
				  			  {
				  				  //target_score1=corr1[corr_i]-corr1[corr_i-1];
				  				  target_score1=corr1[corr_i];
				  				  if(target_score1 > target_threshold)
				  				  {
				  					target_score0=target_score0+target_score1;
				  					if(target_s[target_i]<=target_score0)
				  					{
				  						target_s[target_i]=target_score0;
				  						//target_s1[target_i]=target_score1;  //keep track of channel 1 too
				  						target_d[target_i]=corr_i;


										for(i=TARGET_MAX-1;i>=0;--i)
										{
										if(target_s[i]<target_score0)
											{
											target_score0=target_s[i];
											target_i=i;

											}
										}

				  					}
				  				  }
				  			  }
				  		  }


				  corr_i=corr_i+1;
				  if (corr_i > TARGET_BLANK)
				  		  {

				  			  //target_score= 2*(corr_calc1-corr_calc0-corr_calc2)+corr[corr_i-6]+corr_calc4; //calculate sombrero fit
			  	         	  target_score0= corr0[corr_i];
				  			  if(target_score0 > target_threshold)
				  			  {
				  				  target_score1=corr1[corr_i];
				  				  if(target_score1 > target_threshold)
				  				  {
				  					target_score0=target_score0+target_score1;
				  					if(target_s[target_i]<=target_score0)
				  					{
				  						target_s[target_i]=target_score0;
				  						//target_s1[target_i]=target_score1;  //keep track of channel 1 too
				  						target_d[target_i]=corr_i;


										for(i=TARGET_MAX-1;i>=0;--i)
										{
										if(target_s[i]<target_score0)
											{
											target_score0=target_s[i];
											target_i=i;

											}
										}

				  					}
				  				  }
				  			  }
				  		  }




				  corr_i=corr_i+1;
				  if (corr_i > TARGET_BLANK)
				  		  {

				  			  //target_score= 2*(corr_calc1-corr_calc0-corr_calc2)+corr[corr_i-6]+corr_calc4; //calculate sombrero fit

					  	  	  target_score0= corr0[corr_i];
					  	  	  if(target_score0 > target_threshold)
				  			  {
				  				  target_score1=corr1[corr_i];
				  				  if(target_score1 > target_threshold)
				  				  {
				  					target_score0=target_score0+target_score1;
				  					if(target_s[target_i]<=target_score0)
				  					{
				  						target_s[target_i]=target_score0;
				  						//target_s1[target_i]=target_score1;  //keep track of channel 1 too
				  						target_d[target_i]=corr_i;


										for(i=TARGET_MAX-1;i>=0;--i)
										{
										if(target_s[i]<target_score0)
											{
											target_score0=target_s[i];
											target_i=i;

											}
										}

				  					}
				  				  }
				  			  }
				  		  }




				  corr_i=corr_i+1;
				  if (corr_i > TARGET_BLANK)
				  		  {

				  			  //target_score= 2*(corr_calc1-corr_calc0-corr_calc2)+corr[corr_i-6]+corr_calc4; //calculate sombrero fit
					  	  	  target_score0= corr0[corr_i];
				  			  if(target_score0 > target_threshold)
				  			  {
				  				  target_score1=corr1[corr_i];
				  				  if(target_score1 > target_threshold)
				  				  {
				  					target_score0=target_score0+target_score1;
				  					if(target_s[target_i]<=target_score0)
				  					{
				  						target_s[target_i]=target_score0;
				  						//target_s1[target_i]=target_score1;  //keep track of channel 1 too
				  						target_d[target_i]=corr_i;


										for(i=TARGET_MAX-1;i>=0;--i)
										{
										if(target_s[i]<target_score0)
											{
											target_score0=target_s[i];
											target_i=i;

											}
										}

				  					}
				  				  }
				  			  }
				  		  }




				  corr_i=corr_i+1;









				  if ((corr_i) >= (corr_size_max))  //check if we have processed the whole mems buffer
					  {
					  GPIOC_PCOR = PIN_PTC2; //clear pin 2
					  state = 4;  //go to next state
					  }




  	  	  }

	}


	  if (state == 4)
		  {

		  //do some post processing
		  //fine grain correlation for microphone 0

		 // GPIOC_PSOR = PIN_PTC7; //set pin 7
		  target_i=TARGET_MAX-1;

		  while(target_i>=0 && target_d[target_i]>0)
		  {

				  i=0;
				  j=0;
				  corr_i=target_d[target_i] - 1 ; //take the next one and scan from one ahead
				  for(j=0; j<64 ; ++j)
					{
					fine_corr0[target_i][j]=0;
					fine_corr1[target_i][j]=0;

					}
				  for (i=0; i<=i_pos ; ++i)  //Do positive 2x factor terms
				  {
					  ind=ind_pos[i] >> 21;
					  indl=(ind_pos[i] & 0xFF);
					  indr=(ind_pos[i] & 0xFF00)>>8;
					  for(j=0; j<64; ++j)
						  {
						  a0=(memsbuffer0[corr_i + 1+ ( ind)] << (indl))| (memsbuffer0[corr_i + (ind)] >> (indr));
						  a1=(memsbuffer1[corr_i + 1+ ( ind)] << (indl))| (memsbuffer1[corr_i + (ind)] >> (indr));

						  //fine_corr0[target_i][j] = __USADA8(SWARsimd(a0),0,fine_corr0[target_i][j]);
						  //fine_corr1[target_i][j] = __USADA8(SWARsimd(a1),0,fine_corr1[target_i][j]);

						  fine_corr0[target_i][j] =fine_corr0[target_i][j] + SWARx2(a0);
						  fine_corr1[target_i][j] =fine_corr1[target_i][j] + SWARx2(a1);


						  //debug_call(1);

						  //shift to the next bit
						  indr = indr+1;
						  indl = indl-1;
						  if(indr==33)
							  {
								  indr=1;
								  indl=31;
								  ind=ind+1;
							  }
						  }
				  }

				  for (i=0; i<i_neg ; ++i)  //Do negative 2x factor terms
				  {
				  ind=ind_neg[i] >> 21;
				  indl=(ind_neg[i] & 0xFF);
				  indr=(ind_neg[i] & 0xFF00)>>8;
				  for(j=0; j<64; ++j)
					  {
					  a0=(memsbuffer0[corr_i + 1+ ( ind)] << (indl)) |(memsbuffer0[corr_i + (ind)] >> (indr));
					  a1=(memsbuffer1[corr_i + 1+ ( ind)] << (indl))| (memsbuffer1[corr_i + (ind)] >> (indr));
					  fine_corr0[target_i][j] =fine_corr0[target_i][j] - SWARx2(a0);
					  fine_corr1[target_i][j] =fine_corr1[target_i][j] - SWARx2(a1);
					  //debug_call(2);
					  //shift to the next bit
					  indr = indr+1;
					  indl = indl-1;
					  if(indr==33)
						  {
							  indr=1;
							  indl=31;
							  ind=ind+1;
						  }
					  }
				  }

				  //do a negative 1x term along with the 0 1x term

				  ind=ind_neg[i_neg] >> 21;
				  indl=(ind_neg[i_neg] & 0xFF);
				  indr=(ind_neg[i_neg] & 0xFF00)>>8;
				  for(j=0; j<64; ++j)
					  {

					  a0=(memsbuffer0[corr_i + 1+ ( ind)] << (indl))| (memsbuffer0[corr_i + (ind)] >> (indr));
					  a1=(memsbuffer1[corr_i + 1+ ( ind)] << (indl))| (memsbuffer1[corr_i + (ind)] >> (indr));
					  fine_corr0[target_i][j] =fine_corr0[target_i][j] - SWAR(a0);
					  fine_corr1[target_i][j] =fine_corr1[target_i][j] - SWAR(a1);
					  //debug_call(3);
					  //shift to the next bit
					  indr = indr+1;
					  indl = indl-1;
					  if(indr==33)
						  {
							  indr=1;
							  indl=31;
							  ind=ind+1;
						  }
					  }



				  ind=0;
				  indl = 32;
				  indr = 0;
				  for(j=0; j<64; ++j)
					  {
					  a0=(memsbuffer0[corr_i + 1+ ( ind)] << (indl)) |(memsbuffer0[corr_i + (ind)] >> (indr));
					  a1=(memsbuffer1[corr_i + 1+ ( ind)] << (indl))| (memsbuffer1[corr_i + (ind)] >> (indr));
					  fine_corr0[target_i][j] =fine_corr0[target_i][j] -SWAR(a0);
					  fine_corr1[target_i][j] =fine_corr1[target_i][j] -SWAR(a1);
					  //shift to the next bit
					  //debug_call(4);

					  indr = indr+1;
					  indl = indl-1;
					  if(indr==33)
						  {
							  indr=1;
							  indl=31;
							  ind=ind+1;
						  }
					  }

				  //Find the peak for each microphone and then the delta between them
				  m0_max=-32767;
				  m1_max=m0_max;
				  m0_max_loc=-1;
				  m1_max_loc=m0_max_loc;
				  a0=0;
				  a1=0;
				  a2=0;
				  a3=0;



				  for(j=0; j<64; ++j)
					{
					  //a0=(fine_corr0[target_i][j-1]+fine_corr0[target_i][j]+fine_corr0[target_i][j+1]);
					  //a1=(fine_corr1[target_i][j-1]+fine_corr1[target_i][j]+fine_corr1[target_i][j+1]);

					  if(fine_corr0[target_i][j]>m0_max)
					  {
						  m0_max=fine_corr0[target_i][j];
						  m0_max_loc=j;
					  }

					  if(fine_corr1[target_i][j]>m1_max)
					  {
						  m1_max=fine_corr1[target_i][j];
						  m1_max_loc=j;
					  }


					}

				  //log the delta delay between the peaks-future take arcsin (or lookup table) to get angle
				  // a positive number means to the right from perspective of sensor, negative is to the left
				  // so looking down from the top, clockwise direction increases angle
				  target_a[target_i]=m1_max_loc-m0_max_loc;
				  //target_a[target_i]=((m1_max-m0_max)<<5)/(m1_max+ m0_max);
				  target_s[target_i]=m0_max;
				  target_s1[target_i]=m1_max;

				  target_i=target_i-1;

		  }  //while loop

		  GPIOC_PCOR = PIN_PTC7; //clear pin 7

		  timestamp=timestamp+1;  //increment the timestamp tracking the frame rate

		 //create output data for audacity

		  //for(corr_i=0; corr_i<(CORR_SIZE-10); ++corr_i)

		  //{
		//	corr[corr_i]=(SWAR(memsbuffer0[corr_i]) + SWAR(memsbuffer0[corr_i+1])+ SWAR(memsbuffer0[corr_i+2])+ SWAR(memsbuffer0[corr_i+3])+SWAR(memsbuffer0[corr_i+4])+SWAR(memsbuffer0[corr_i+5])+SWAR(memsbuffer0[corr_i+6])+SWAR(memsbuffer0[corr_i+7])+SWAR(memsbuffer0[corr_i+8])+SWAR(memsbuffer0[corr_i+9]))-160;



		  //}




		  if(output_flag==1)
		  {

			  if(target_i_max<255)
			  {
			  printf("\r\n\r\nTarget data \r\n");
			  for(target_i=0; target_i<TARGET_MAX; ++target_i)
			  	{
				  if(target_d[target_i]>0)
				  printf("%d, %d , %d, %d, %d\r\n",timestamp,target_d[target_i],target_a[target_i],target_s[target_i], target_s1[target_i] );
			  	}
			  }
			  else
			  {
				  printf("\r\n\r\nNo Targets \r\n");
			  }
			 // printf("Correlation data \r\n");
			  //for(i=0;i<CORR_SIZE;++i)
			 // {
				//  printf("%ld\r\n",corr0[i] );
			 // }


		  }
		  else if(output_flag==2)
		  {
			  //printf("testcount0: %ld testcount1: %ld\r\n",testcount0,testcount1);

		  }


		  if (count<2)
			  {
			  count=count+1;
			  state=0; //restart process
			  }
		  else
			  {count = 0;
			  state=0;  //done doing it 10 times

			  }


		  }


  }


  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
