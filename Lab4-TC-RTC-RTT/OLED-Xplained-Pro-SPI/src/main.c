#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */



// LED da placa
#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        ID_PIOC              // ID do periférico PIOC (controla LED) 
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_IDX_MASK  (1 << LED_PIO_IDX) 

// LED 1 do OLED
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO PIOA
#define LED1_PIN 0
#define LED1_IDX_MASK (1 << LED1_PIN)

// LED 2 do OLED
#define LED2_PIO_ID ID_PIOC
#define LED2_PIO PIOC
#define LED2_PIN 30
#define LED2_IDX_MASK (1 << LED2_PIN)

// LED 3 do OLED
#define LED3_PIO_ID ID_PIOB
#define LED3_PIO PIOB
#define LED3_PIN 2
#define LED3_IDX_MASK (1 << LED3_PIN)

// Butt 1 do OLED
#define BUT1_PIO_ID ID_PIOD
#define BUT1_PIO PIOD
#define BUT1_PIN 28
#define BUT1_IDX_MASK (1 << BUT1_PIN)

/*  Rising edge interrupt is active */
#define PIO_IT_RISE_EDGE        (PIO_IT_RE_OR_HL | PIO_IT_EDGE | PIO_IT_AIME)

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

uint32_t h, m, s;

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/

volatile char flag_tc = 0;
volatile char flag_placa = 0;
volatile Bool f_rtt = false;
volatile char flag_rtc = 0;
volatile char but1_flag;
volatile char flag_sec = 0;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void LED_init(int estado);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void pin_toggle(Pio *pio, uint32_t mask);
void pisca_led(int n, int t);
void pisca_led_p(int n, int t);
void pisca_led3(int n, int t);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

/**
*  Handle Interrupcao botao 1
*/
// static void Button1_Handler(uint32_t id, uint32_t mask)
// {

// }

void but1_callback(void)
{
	but1_flag = 1;
}

/**
*  Interrupt handler for TC1 interrupt.
*/
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc = 1;
}

void TC0_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_placa = 1;
}

void RTT_Handler(void)
{
  uint32_t ul_status;

  /* Get RTT status - ACK */
  ul_status = rtt_get_status(RTT);

  /* IRQ due to Time has changed */
  if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
    // f_rtt = false;  
	pin_toggle(LED2_PIO, LED2_IDX_MASK);  
    }

  /* IRQ due to Alarm */
  if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
    // pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
      f_rtt = true;                  // flag RTT alarme
   }  
}

/**
* \brief Interrupt handler for the RTC. Refresh the display.
*/
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* seccond tick	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		//rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		flag_sec = 1; 
		
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		flag_rtc = 1;
	}


	
	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask){
  if(pio_get_output_data_status(pio, mask)){
	  pio_clear(pio, mask);
  }
    
  else{
	  pio_set(pio,mask);
  }
    
}


/*
 * @Brief Pisca LED placa
 */
void pisca_led(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED1_PIO, LED1_IDX_MASK);
    delay_ms(t);
    pio_set(LED1_PIO, LED1_IDX_MASK);
    delay_ms(t);
  }
}

void pisca_led_p(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED_PIO, LED_IDX_MASK);
    delay_ms(t);
    pio_set(LED_PIO, LED_IDX_MASK);
    delay_ms(t);
  }
}

void pisca_led3(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED3_PIO, LED3_IDX_MASK);
    delay_ms(t);
    pio_set(LED3_PIO, LED3_IDX_MASK);
    delay_ms(t);
  }
}




void init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	
	// Inicializa PIO do led do Xplained
	
	pmc_enable_periph_clk(LED1_PIO_ID);
	pio_set_output(LED1_PIO, LED1_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(LED2_PIO_ID);
	pio_set_output(LED2_PIO, LED2_IDX_MASK, 0, 0, 0);
	
	pmc_enable_periph_clk(LED3_PIO_ID);
	pio_set_output(LED3_PIO, LED3_IDX_MASK, 0, 0, 0);
	
	// Ativa interrupção
	pio_enable_interrupt(BUT1_PIO, BUT1_IDX_MASK);
	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais proximo de 0 maior)
	NVIC_EnableIRQ(BUT1_PIO_ID);
	NVIC_SetPriority(BUT1_PIO_ID, 4); // Prioridade 4
	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback  a: but_callback()
	pio_handler_set(BUT1_PIO,
	BUT1_PIO_ID,
	BUT1_IDX_MASK,
	PIO_IT_RISE_EDGE,
	but1_callback);
	
}

static float get_time_rtt(){
  uint ul_previous_time = rtt_read_timer_value(RTT); 
}

/**
* Configura TimerCounter (TC) para gerar uma interrupcao no canal (ID_TC e TC_CHANNEL)
* na taxa de especificada em freq.
*/
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	/* O TimerCounter é meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
  	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}


static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
  uint32_t ul_previous_time;

  /* Configure RTT for a 1 second tick interrupt */
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  ul_previous_time = rtt_read_timer_value(RTT);
  while (ul_previous_time == rtt_read_timer_value(RTT));
  
  rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

  /* Enable RTT interrupt */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);
  rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

/**
* Configura o RTC para funcionar com interrupcao de alarme
*/
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	init();
	// Init OLED
	gfx_mono_ssd1306_init();

	/** Configura timer TC0, canal 1 */
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC0, ID_TC0, 0, 5);
	
	// Inicializa RTT com IRQ no alarme.
	f_rtt = true;
	
	pio_set(LED1_PIO, LED1_IDX_MASK);
	pio_set(LED2_PIO, LED2_IDX_MASK);
	pio_set(LED3_PIO, LED3_IDX_MASK);
	
	/** Configura RTC */
	calendar rtc_initial = {2021, 9, 36, 10, 10, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);
  
	char buffer [50];
	while (1) {
		if(but1_flag){
			rtc_get_time(RTC,&h,&m,&s);
			/* configura alarme do RTC */
			rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
			rtc_set_time_alarm(RTC, 1, h, 1, m, 1, s + 20);
			but1_flag=0;
		}
		if(flag_rtc){
			pisca_led3(5, 200);
			flag_rtc = 0;
		}
		
		if (f_rtt){
		  uint16_t pllPreScale = (int) (((float) 32768) / 0.125);
		  uint32_t irqRTTvalue = 16;
      
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);         
      
		  f_rtt = false;
		}
		
		if(flag_tc){
			pisca_led(1,10);
			flag_tc = 0;
		}
		
		if (flag_placa){
			pisca_led_p(1, 10);
			flag_placa = 0;
		}	
		
		if (flag_sec){
			rtc_get_time(RTC,&h,&m,&s);

			sprintf(buffer, "%d : %d : %d", h, m, s);		
			gfx_mono_draw_string(buffer, 0, 0, &sysfont);
			flag_sec= 0;
			
		}	
		
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}