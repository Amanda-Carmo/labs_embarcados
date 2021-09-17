#include <asf.h>
#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// TRIG
#define TRIG_PIO      PIOD
#define TRIG_PIO_ID   ID_PIOD
#define TRIG_IDX      30
#define TRIG_IDX_MASK (1 << TRIG_IDX)

// ECHO
#define ECHO_PIO      PIOA
#define ECHO_PIO_ID   ID_PIOA
#define ECHO_IDX      6
#define ECHO_IDX_MASK (1 << ECHO_IDX)

volatile char echo_flag;
// volatile Bool f_rtt = false;
volatile uint32_t rtt_status = 0;

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void pin_toggle(Pio *pio, uint32_t mask);

void echo_callback(void){

	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_IDX_MASK))
	{
		uint16_t pllPreScale = (int) (((float) 32768) / 8000);
		uint32_t irqRTTvalue = 8000; // define quantidade de tempo
      
		// reinicia RTT para gerar um novo IRQ
		RTT_init(pllPreScale, irqRTTvalue);       
	}
	else{
		rtt_status = rtt_read_timer_value(RTT);
	}
}

void pin_toggle(Pio *pio, uint32_t mask){
  if(pio_get_output_data_status(pio, mask)){
	  pio_clear(pio, mask);
  }
    
  else{
	  pio_set(pio,mask);
  }
    
}

void gera_pulso(){
    pio_set(TRIG_PIO, TRIG_IDX_MASK);
    delay_us(10);
    pio_clear(TRIG_PIO, TRIG_IDX_MASK);
}


// void RTT_Handler(void)
// {
//   uint32_t ul_status;

//   /* Get RTT status - ACK */
//   ul_status = rtt_get_status(RTT);

//   /* IRQ due to Time has changed */
//   if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
//     // f_rtt = false;  
// 	pin_toggle(ECHO_PIO, ECHO_IDX_MASK);  
//     }

//   /* IRQ due to Alarm */
//   if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
//     // pin_toggle(LED_PIO, LED_IDX_MASK);    // BLINK Led
//     //   f_rtt = true;                  // flag RTT alarme
//    }  
// }

void io_init(void){
	// Initialize the board clock
	sysclk_init();

	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;

	// Configura trig
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pio_configure(TRIG_PIO, PIO_OUTPUT_0, TRIG_IDX_MASK, PIO_PULLUP);

	// Configura echo
	pmc_enable_periph_clk(ECHO_PIO_ID);
	pio_configure(ECHO_PIO, PIO_INPUT, ECHO_IDX_MASK, PIO_PULLUP);

	// Ativa interrupção
	//pio_enable_interrupt(TRIG_PIO, TRIG_IDX_MASK);
	pio_enable_interrupt(ECHO_PIO, ECHO_IDX_MASK);

	// Configura interrupção no pino referente ao echo e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: echo_callback()
	pio_handler_set(ECHO_PIO,
					ECHO_PIO_ID,
					ECHO_IDX_MASK,
					PIO_IT_EDGE,
					echo_callback);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)	
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4); // Prioridade 4
}

static float get_time_rtt(){
  uint ul_previous_time = rtt_read_timer_value(RTT); 
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
  uint32_t ul_previous_time;

  /* Configure RTT for a 1 second tick interrupt */
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
//   ul_previous_time = rtt_read_timer_value(RTT);
//   while (ul_previous_time == rtt_read_timer_value(RTT));
  
//   rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);

//   /* Enable RTT interrupt */
//   NVIC_DisableIRQ(RTT_IRQn);
//   NVIC_ClearPendingIRQ(RTT_IRQn);
//   NVIC_SetPriority(RTT_IRQn, 4);
//   NVIC_EnableIRQ(RTT_IRQn);
//   rtt_enable_interrupt(RTT, RTT_MR_ALMIEN | RTT_MR_RTTINCIEN);
}

int main (void)
{
	io_init();
	board_init();
	sysclk_init();
	delay_init();

  // Init OLED
	gfx_mono_ssd1306_init();
	float buffer [100];
  
  // Escreve na tela um circulo e um texto
	// gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
	// Inicializa RTT com IRQ no alarme.
	// f_rtt = true;

  /* Insert application code here, after the board has been initialized. */
	while(1) {
		gera_pulso();
		while (rtt_status == 0){}
		
		float t = (float)rtt_status/8000.0;
		float dis = (340.0 * t * 100.0)/2.0;
		
		sprintf(buffer, "dist: %f", dis);
		// gfx_mono_draw_string("ola", 0, 0, &sysfont);	
		gfx_mono_draw_string(buffer, 0, 0, &sysfont);	
		
		rtt_status = 0;
	}
	
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
}
