/*Ing. Bonavolontà
Esame 25 Febbraio Emanuele Di Maio N46004384*/


#include <stm32f30x.h>
#include <stdio.h>
#include <math.h>

//Registri RCC
#define RCC_APB1ENR (unsigned int*) 0x4002101C
#define RCC_AHBENR (unsigned int *) 0x40021014
#define RCC_APB2ENR (unsigned int *) 0x40021018

#define GPIOA_MODER (unsigned int *) 0x48000000
#define GPIOE_MODER (unsigned int *) 0x48001000
#define GPIOE_ODR (unsigned int *) 0x48001014

#define TIM2_CR1 (unsigned int*) 0x40000000
#define TIM2_CR2 (unsigned int*) 0x40000004
#define TIM2_CNT (unsigned int*) 0x40000024
#define TIM2_ARR (unsigned int*) 0x4000002C
#define TIM2_SR (unsigned int*) 0x40000010

#define TIM3_CR1 (unsigned int*) 0x40000400
#define TIM3_CR2 (unsigned int*) 0x40000404
#define TIM3_CNT (unsigned int*) 0x40000424
#define TIM3_ARR (unsigned int*) 0x4000042C
#define TIM3_SR (unsigned int*) 0x40000410

//Registri ADC

#define ADC12_CCR (unsigned int*) 0x50000308
#define ADC1_CR (unsigned int*) 0x50000008
#define ADC1_DIFSEL (unsigned int*) 0x500000B0
#define ADC1_ISR (unsigned int*) 0x50000000
#define ADC1_SMPR1 (unsigned int*) 0x50000014
#define ADC1_SQR1 (unsigned int*) 0x50000030
#define ADC1_DR (unsigned int*) 0x50000040
#define ADC1_CFGR (unsigned int*) 0x5000000C

#define CKMODE_1 (unsigned int) 1<<17
#define CKMODE_0 (unsigned int) 1<<16

//Bit ADC1_CR
#define ADVREGEN_0 (unsigned int) 1<<28
#define ADVREGEN_1 (unsigned int) 1<<29

#define ADEN (unsigned int) 1<<0
#define DIFSEL_3 (unsigned int) (1<<3)
#define ADCAL (unsigned int) 1<<31
#define ADSTART (unsigned int) (1<<2)

//Bit ADC1_ISR
#define EOC (unsigned int) 1<<2
#define ADRDY (unsigned int) 1<<0

#define L_0 (unsigned int) 1<<0
#define L_1 (unsigned int) 1<<1
#define L_2 (unsigned int) 1<<2
#define L_3 (unsigned int) 1<<3

#define SQ1_0 (unsigned int) 1<<6
#define SQ1_1 (unsigned int) 1<<7
#define SQ1_2 (unsigned int) 1<<8
#define SQ1_3 (unsigned int) 1<<9
#define SQ1_4 (unsigned int) 1<<10

#define SMP3_0 (unsigned int) (1<<9)
#define SMP3_1 (unsigned int) (1<<10)
#define SMP3_2 (unsigned int) (1<<11)

//Registri DAC
#define DAC_CR (unsigned int*) 0x40007400
#define DAC_SWTRIGR (unsigned int*) 0x40007404
#define DAC_DHR12R1 (unsigned int*) 0x40007408

#define EN1 (unsigned int) 1<<0
#define BOFF1 (unsigned int) 1<<1
#define TEN1 (unsigned int) 1<<2
#define TSEL1_0 (unsigned int) 1<<3
#define TSEL1_1 (unsigned int) 1<<4
#define TSEL1_2 (unsigned int) 1<<5
#define DMAEN1 (unsigned int) 1<<12


#define MMS_0 (unsigned int) (1<<4)
#define MMS_1 (unsigned int) (1<<5)
#define MMS_2 (unsigned int) (1<<6)


//Define DMA2 e DMA1
#define DMA2_CCR3 (unsigned int*) 0x40020430
#define DMA2_CNDTR3 (unsigned int*) 0x40020434
#define DMA2_CPAR3 (unsigned int*) 0x40020438
#define DMA2_CMAR3 (unsigned int*) 0x4002043C

#define DMA1_ISR (unsigned int*) 0x40020000
#define DMA1_IFCR (unsigned int*) 0x40020004
#define DMA1_CCR1 (unsigned int*) 0x40020008
#define DMA1_CNDTR1 (unsigned int*) 0x4002000C
#define DMA1_CPAR1 (unsigned int*) 0x40020010
#define DMA1_CMAR1 (unsigned int*) 0x40020014

#define PL_0 (unsigned int) 1<<12
#define PL_1 (unsigned int) 1<<13
#define MSIZE_0 (unsigned int) 1<<10
#define MSIZE_1 (unsigned int) 1<<11
#define PSIZE_0 (unsigned int) 1<<8
#define PSIZE_1 (unsigned int) 1<<9
#define MINC (unsigned int) 1<<7
#define CIRC (unsigned int) 1<<5
#define DIR (unsigned int) 1<<4
#define EN (unsigned int) 1<<0
#define TCIF1 (unsigned int) (1<<1)
#define CTCIF1 (unsigned int) (1<<1)

//ADC1
#define DMAEN (unsigned int) (1<<0)
#define DMACFG (unsigned int) (1<<1)
#define EXTSEL_0 (unsigned int) (1<<6)
#define EXTSEL_1 (unsigned int) (1<<7)
#define EXTSEL_2 (unsigned int) (1<<8)
#define EXTSEL_3 (unsigned int) (1<<9)
#define EXTEN_0 (unsigned int) (1<<10)
#define EXTEN_1 (unsigned int) (1<<11)

//Portare da 40 khz a 4 khz
//S = del Segnale (DAC)
//C = del Campionamento (ADC1)

#define PI 3.14159265358979
#define NPPs 100
#define NPPc 128
#define N_ARR_S 180
#define N_ARR_C 140

//0.384 V = X/4095 * 3 -> X = 524
//#define MATRICOLA 524
#define MATRICOLA 100

unsigned int* punt;
unsigned int y[NPPs];
unsigned int v[NPPc];
int i;
float x,dx;
float vrms;

unsigned int t;
unsigned int Voff;
unsigned int Vp;

//LED ROSSO PE9
//Timer2 per DAC
//Timer3 per ADC

void main(){
  	///Init var
	i = 1;
	x = 0.0;
	dx=(2*PI)/100;
	t = 0;
	Vp = (unsigned int)MATRICOLA*i; //524 è 0.384 Volt in unsigned int [0-4095]
	Voff = (unsigned int)2047; //1.5 V
	
	///Configurazione RCC
	punt = RCC_AHBENR;
	*punt |= (1<<0) | (1<<1) | (1<<17) | (1<<21) | (1<<28); //Abilito DMA1, DMA2, IOPA, IOPE, ADC12
	punt=RCC_APB1ENR;
	*punt|= 1|(1<<29) | (1<<1); //abilito timer2// ABILITO IL dac // Abilito timer3

	///Configurazione I/O
	punt = GPIOE_MODER;
	*punt |= (1<<18); //Output PE9
	*punt &= ~(1<<19); 
	
	punt = GPIOA_MODER;
	*punt &= ~(1<<0); //Bottone PA0
	*punt &= ~(1<<0); 
	
	*punt |= (1<<4); //Analog ADC1_CH3 in PA2
	*punt |= (1<<5); 
	
	*punt |= (1<<8); //Analog DAC1 in PA4
	*punt |= (1<<9); 


	///Sinusoide
	for(int i=0; i<NPPs;i++){
		y[i]=(unsigned int)(Vp*sin(x)+Voff);
		x=x+dx;
	}

	///Configurazione ADC12
	punt=ADC12_CCR;
	*punt|=CKMODE_0;
	*punt&=~(CKMODE_1);

	//Voltage Regulator
	punt=ADC1_CR;
	*punt &= ~(ADVREGEN_1);
	*punt |= ADVREGEN_0;
	for(int i=0;i<1000000;i++);

	//Single-Ended
	*punt &= ~(ADEN);
	punt = ADC1_DIFSEL;
	*punt&=~(DIFSEL_3); //Canale 3 in Single ended mode

	//Autocalibrazione
	punt=ADC1_CR;
	*punt|=ADCAL;
	while(((*punt)&ADCAL)==ADCAL);

	//Accensione ADC
	*punt|=ADEN;
	punt=ADC1_ISR;
	while(((*punt)&ADRDY)!=ADRDY);

	//Tempo di Sample
	punt=ADC1_SMPR1;
	*punt|=SMP3_2|SMP3_1|SMP3_0;

	//Sequenza regolare
	punt=ADC1_SQR1;
	*punt&=~(L_0); //1 Conversione
	*punt&=~(L_1);
	*punt&=~(L_2);
	*punt&=~(L_3);

	//Selezione del Canale 3 per PA2
	*punt|=SQ1_0;
	*punt|=SQ1_1;
	*punt&=~(SQ1_2);
	*punt&=~(SQ1_3);
	*punt&=~(SQ1_4);
	
	//AGGIUNGERE SYSCFG
	punt = ADC1_CFGR;
	*punt |= DMACFG;
	*punt &= ~(EXTSEL_0); //EXT4. TIM3_TRGO
	*punt &= ~(EXTSEL_1);
	*punt |= (EXTSEL_2);
	*punt &= ~(EXTSEL_3);
	
	*punt &= ~EXTEN_0;
	*punt |= EXTEN_1;
	
	*punt |= DMAEN;
	
	///CONFIGURAZIONE DAC
	punt=DAC_CR;
	*punt|= (BOFF1);// Amplificatore di uscita disattivato
	*punt|= TEN1; 
	*punt&=~(TSEL1_0);//TRIGGER DA TIMER2 100
	*punt&=~(TSEL1_1);
	*punt|=TSEL1_2;
	*punt|=DMAEN1; //ABILITAZIONE DEL DMA
	*punt|=EN1;

	///Configurazione DMA2_Channel3 DAC
	// Scrivere l'indirizzo del DHR in CPAR
	punt=DMA2_CPAR3;
	*punt=(unsigned int)DAC_DHR12R1;
	// Scrivere &y[0] in MPAR
	punt=DMA2_CMAR3;
	*punt=(unsigned int)&y[0];
	// Scrivere in CNDTR
	punt=DMA2_CNDTR3;
	*punt=NPPs;
	// Configurare Priorità PL// Configurare DIR, INCREMENTO, CIRCULAR MODE, ...
	punt=DMA2_CCR3;
	*punt|=PL_0|PL_1;

	*punt&=~(MSIZE_0);//32 BIT MSIZE=10
	*punt|=MSIZE_1;

	*punt&=~(PSIZE_0);//32 BIT MSIZE=10
	*punt|=PSIZE_1;

	*punt|=MINC;// INCREMENTO DELLA MEMORIA
	*punt|=DIR;// DIREZIONE DELLA MEMORIA
	*punt|=CIRC;// BUFFER CIRCOLARE DELLA MEMORIA
	*punt|=EN;// ABILITAZIONE DMA
	
	///Configurazione DMA1_Channel1 ADC1
	punt=DMA1_CPAR1;
	*punt=(unsigned int)ADC1_DR;
	
	punt=DMA1_CMAR1;
	*punt = (unsigned int) &v[0];
	
	punt=DMA1_CNDTR1;
	*punt=NPPc;
	// Configurare Priorità PL// Configurare DIR, INCREMENTO, CIRCULAR MODE, ...
	punt=DMA1_CCR1;
	*punt|=PL_0|PL_1;

	*punt&=~(MSIZE_0);//32 BIT MSIZE=10
	*punt|=MSIZE_1;

	*punt&=~(PSIZE_0);//32 BIT MSIZE=10
	*punt|=PSIZE_1;

	*punt|=MINC;// INCREMENTO DELLA MEMORIA
	*punt &= ~(DIR);// DIREZIONE DALLA PERIFERICA
	*punt |= (CIRC);
	*punt|=EN;// ABILITAZIONE DMA

	///Interrupt
	NVIC->ISER[0] |= (1<<6); //Abilito EXTI0
	EXTI->IMR |= (1<<0); //Abilito Interrupt Mask register 0
	EXTI->RTSR |= (1<<0); //Abilito RTSR su EXTI0
	
	///Timer2 e Timer3
	//Configurazione TIMER2 
	punt = TIM2_ARR;
	*punt = N_ARR_S;

	punt=TIM2_CR2;
	*punt=~(MMS_0); //Update Event 010
	*punt|=MMS_1; 
	*punt=~(MMS_2);

	//Configurazione TIMER3 
	punt=TIM3_ARR;
	*punt=N_ARR_C;

	punt=TIM3_CR2;
	*punt=~(MMS_0); //Update Event 010
	*punt|=MMS_1; 
	*punt=~(MMS_2);
	
	punt=TIM2_CR1;
	*punt |= 1;//Avvio il Timer
	punt=TIM3_CR1;
	*punt |= 1;//Avvio il Timer

	punt = ADC1_CR;
	*punt |= ADSTART;
	
	while(1){
	 	punt = DMA1_ISR;
	  	do{
	  		t = (unsigned int)(*punt);
	  	}while((t&TCIF1) != TCIF1);
		punt = TIM3_CR1;
		*punt &= ~(1<<0); //Spengo il Timer3 (ADC1)
		
		for(int i=0; i<NPPc; i++){
			vrms += pow((v[i]*3.0/4095.0),2);
		}
		vrms = vrms/NPPc;
		vrms = sqrt(vrms);
		printf("Vrms: %.3f\n", vrms);
		
		if(vrms >= 1.8){ ///Pare che non viene mai superato
			punt = GPIOE_ODR;
			*punt |= (1<<9); //PE9 acceso
		}else{
			punt = GPIOE_ODR;
			*punt &= ~(1<<9); //PE9 spento
		}
		
		
		punt = DMA1_IFCR;
		*punt |= CTCIF1; //Resetto il flag
		
		punt = TIM3_CR1;
		*punt |= (1<<0); //Accendo il Timer3 (ADC1)
	}
}

void EXTI0_IRQHandler(){
	EXTI->PR |= (1<<0);
	i = i+1;
	Vp = (unsigned int) (i * MATRICOLA);
	if(Vp >= 2047){
		i = 1;
		Vp = (unsigned int) (i*MATRICOLA);
	}		
	
	for(int j=0; j<NPPs; j++){
		y[j] = (unsigned int)(Vp*sin(2*PI*j/NPPs)+Voff);
	}
	
	printf("Variazione indice: %d\n", i);
}