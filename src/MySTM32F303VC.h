/*FILE MySTM32F303VC*/

/*---------------------------------------*/
/***********STRUTTURA DATI RCC**********/

typedef struct{

   unsigned int CR;
   unsigned int CFGR;
   unsigned int CIR;
   unsigned int APB2RSTR;
   unsigned int APB1RSTR;
   
   union{unsigned int AHBENR;
     struct{
       unsigned DMA1EN:1;   
       unsigned DMA2EN:1;  
       unsigned SRAMEN:1;
       unsigned Res:1;
       unsigned FLITFEN:1;   
       unsigned Res1:1;
       unsigned CRCEN:1;
       unsigned Res2:10;
       unsigned IOPAEN:1;
       unsigned IOPBEN:1;
       unsigned IOPCEN:1;
       unsigned IOPDEN:1;
       unsigned IOPEEN:1;
       unsigned IOPFEN:1;
       unsigned Res3:1;
       unsigned TSCEN:1;
       unsigned Res4:3;
       unsigned ADC12:1;
       unsigned ADC34:1;
       unsigned Res5:2;
                     };//END STRUCT ahbenr
            };//end union AHBENR
   
   
   unsigned int APB2ENR;
   unsigned int APB1ENR;
   unsigned int BDCR;
   unsigned int CSR;
   unsigned int AHBRSTR;
   unsigned int CFGR2;
   unsigned int CFGR3;

}RCC_Type;



/*---------------------------------------*/
/***********STRUTTURA DATI GPIO**********/

typedef struct{
  
  union{unsigned int MODER;
        struct{
          unsigned MODER0:2;
          unsigned MODER1:2;
          unsigned MODER2:2;
          unsigned MODER3:2;
          unsigned MODER4:2;
          unsigned MODER5:2;
          unsigned MODER6:2;
          unsigned MODER7:2;
          unsigned MODER8:2;
          unsigned MODER9:2;
          unsigned MODER10:2;
          unsigned MODER11:2;
          unsigned MODER12:2;
          unsigned MODER13:2;
          unsigned MODER14:2;
          unsigned MODER15:2;
                };
        };
  
       
       unsigned int OTYPER;
       unsigned int OSPEEDR;
       unsigned int PUPDR;
       
       
       union{   unsigned int IDR;
                struct{
                  unsigned IDR0:1;
                  unsigned IDR1:1;
                  unsigned IDR2:1;
                  unsigned IDR3:1;
                  unsigned IDR4:1;
                  unsigned IDR5:1;
                  unsigned IDR6:1;
                  unsigned IDR7:1;
                  unsigned IDR8:1;
                  unsigned IDR9:1;
                  unsigned IDR10:1;
                  unsigned IDR11:1;
                  unsigned IDR12:1;
                  unsigned IDR13:1;
                  unsigned IDR14:1;
                  unsigned IDR15:1;
                  };
             };
       
       
       
       union{     unsigned int ODR;
       
                  struct{
                  unsigned OR0:1;
                  unsigned ODR1:1;
                  unsigned ODR2:1;
                  unsigned ODR3:1;
                  unsigned ODR4:1;
                  unsigned ODR5:1;
                  unsigned ODR6:1;
                  unsigned ODR7:1;
                  unsigned ODR8:1;
                  unsigned ODR9:1;
                  unsigned ODR10:1;
                  unsigned ODR11:1;
                  unsigned ODR12:1;
                  unsigned ODR13:1;
                  unsigned ODR14:1;
                  unsigned ODR15:1;
                  };
                  
       };
       /*
       unsigned int BSRR;
       unsigned int LCKR;
       unsigned int AFRL;
       unsigned int AFRH;
       unsigned int BRR;
       */

}GPIO_Type;

#define Input           0
#define Output          1
#define Alternate       2
#define Analog          3

#define RCC ((RCC_Type*) 0x40021000)
#define GPIOA ((GPIO_Type*) 0x48000000)
#define GPIOE ((GPIO_Type*) 0x48001000)
