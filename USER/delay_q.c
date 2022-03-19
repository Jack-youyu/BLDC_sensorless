//******************************************************************************
// delay_q.c
//******************************************************************************

#define _48MHz  1
#define _72MHz  0

#define NOP1    __nop();
#define NOP2    NOP1 NOP1
#define NOP5    NOP2 NOP2 NOP1
#define NOP10   NOP5 NOP5
#define NOP20   NOP10 NOP10
#define NOP50   NOP20 NOP20 NOP10
#define NOP100  NOP50 NOP50

//==============================================================================
void delay_1us(void)
{
  #if _48MHz
	NOP20; NOP10; NOP1;
  #endif
  #if _72MHz
	NOP50; NOP1;
  #endif
}

//------------------------------------------------------------------------------
void delay_2us(void)
{
  #if _48MHz
	NOP50; NOP20; NOP5;
  #endif
  #if _72MHz
	NOP100; NOP20; NOP5;
  #endif
}

//------------------------------------------------------------------------------
void _delay_5us(void) //!!! 把'_delay_5us();'調準確,因為它是下面各種delay的基礎
{
  #if _48MHz
	NOP100; NOP100; NOP10;
  #endif
  #if _72MHz
	NOP100; NOP100; NOP50; NOP10;
  #endif
}

//
void delay_5us(void)
{
	_delay_5us();
}

//------------------------------------------------------------------------------
void delay_10us(unsigned int n)
{
	unsigned int i;
	for(i=0;i<n;i++){
		_delay_5us(); _delay_5us();
	}
}

//------------------------------------------------------------------------------
void delay_100us(unsigned int n)
{
	unsigned int i,j;
	for(i=0;i<n;i++){
		for(j=0;j<10;j++){ _delay_5us(); _delay_5us(); NOP20;}
	}
}

//------------------------------------------------------------------------------
void delay_ms(unsigned int n)
{
	unsigned int i,j;
	for(i=0;i<n;i++){
		for(j=0;j<105;j++){ _delay_5us(); _delay_5us();}
	}
}

//==============================================================================
