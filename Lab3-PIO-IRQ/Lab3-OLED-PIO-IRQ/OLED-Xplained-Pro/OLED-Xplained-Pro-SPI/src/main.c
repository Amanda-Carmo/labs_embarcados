#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include <math.h>

#include "asf.h"

//#include <stdio.io> 

// LED
#define LED_PIO      PIOC
#define LED_PIO_ID   ID_PIOC
#define LED_IDX      8
#define LED_IDX_MASK (1 << LED_IDX)

// Botão 1
#define BUT_PIO      PIOD
#define BUT_PIO_ID   ID_PIOD
#define BUT_IDX  28
#define BUT_IDX_MASK (1 << BUT_IDX)

// Botão 2
#define BUT2_PIO      PIOC
#define BUT2_PIO_ID   ID_PIOC
#define BUT2_IDX  31
#define BUT2_IDX_MASK (1 << BUT2_IDX)

// Botão 3
#define BUT3_PIO      PIOA
#define BUT3_PIO_ID   ID_PIOA
#define BUT3_IDX  19
#define BUT3_IDX_MASK (1 << BUT3_IDX)

/************************************************************************/
/*  Interrupt Edge detection is active. */
#define PIO_IT_EDGE             (1u << 6)

/*  Rising edge interrupt is active */
#define PIO_IT_RISE_EDGE        (PIO_IT_RE_OR_HL | PIO_IT_EDGE | PIO_IT_AIME)
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
volatile char but_flag;
volatile char but_flag1;
volatile char but2_flag;
volatile char led_flag;
volatile char but3_flag;
/************************************************************************/

void io_init(void);
void pisca_led(int n, int t);

void but_callback(void)
{
	if(but_flag == 0 && but_flag1 == 0){
		but_flag1 = 1;
	}
	else if(but_flag1 == 1 && but_flag == 0){
		but_flag1 = 0;
		but_flag  = 1;
	}
}

void but2_callback(void)
{
	if(led_flag){
		but2_flag = 1;
	}
}


void but3_callback(void)
{
	but3_flag = 1;
}

/************************************************************************/
/* funções                                                              */
/************************************************************************/

// pisca led N vez no periodo T
void pisca_led(int n, int t){
	led_flag = 1;
	
	// freq = (1/(t*10^-3));
  	for (int i=0;i<n;i++){
		pio_clear(LED_PIO, LED_IDX_MASK);
		delay_ms(t);
		pio_set(LED_PIO, LED_IDX_MASK);
		delay_ms(t);

		if (but2_flag){
			i = n;
		}
  }
}

// Inicializa botao SW0 do kit com interrupcao
void io_init(void)
{

  	// Configura led
	pmc_enable_periph_clk(LED_PIO_ID);
	pio_configure(LED_PIO, PIO_OUTPUT_0, LED_IDX_MASK, PIO_DEFAULT);

	// Inicializa clock do periférico PIO responsavel pelo botao
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);


  	// Configura PIO para lidar com o pino do botão como entrada
  	// com pull-up
  	pio_configure(BUT_PIO, PIO_INPUT, BUT_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);    
  	pio_set_debounce_filter(BUT_PIO, BUT_IDX_MASK, 60);

	pio_configure(BUT2_PIO, PIO_INPUT, BUT2_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);    
  	pio_set_debounce_filter(BUT2_PIO, BUT2_IDX_MASK, 60);

	pio_configure(BUT3_PIO, PIO_INPUT, BUT3_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);    
  	pio_set_debounce_filter(BUT3_PIO, BUT3_IDX_MASK, 60);

	  
	// Ativa interrupção
	pio_enable_interrupt(BUT_PIO, BUT_IDX_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO, BUT3_IDX_MASK);


	// Configura interrupção no pino referente ao botao e associa
	// função de callback caso uma interrupção for gerada
	// a função de callback é a: but_callback()
	pio_handler_set(BUT_PIO,
					BUT_PIO_ID,
					BUT_IDX_MASK,
					PIO_IT_EDGE,
					but_callback);

	pio_handler_set(BUT2_PIO,
					BUT2_PIO_ID,
					BUT2_IDX_MASK,
					PIO_IT_RISE_EDGE,
					but2_callback);
	
	pio_handler_set(BUT3_PIO,
				BUT3_PIO_ID,
				BUT3_IDX_MASK,
				PIO_IT_RISE_EDGE,
				but3_callback);

	// Configura NVIC para receber interrupcoes do PIO do botao
	// com prioridade 4 (quanto mais próximo de 0 maior)
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4); // Prioridade 4

	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 4); // Prioridade 4

	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID, 4); // Prioridade 4

}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	
	char buffer [50];
	volatile float freq;
	volatile int t = 200;
	int count = 0;
	
	freq = 1.0/(t*pow(10,-3));
	
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;

  	// Init OLED
	  
	gfx_mono_ssd1306_init();

	sprintf(buffer, "f: %f", freq);
	gfx_mono_draw_string(buffer, 0, 0, &sysfont);

  	// configura botao com interrupcao
  	io_init();

  /* Insert application code here, after the board has been initialized. */

	but_flag1 = 0;
	while(1) {
		if(but_flag){

			if (count >= 10){
				t = t + 100;
			}
			
			else if (t != 100){
				t = t - 100;
			}
			
			freq = 1.0/(t*pow(10,-3));

			sprintf(buffer, "f: %f", freq);
			gfx_mono_draw_string(buffer, 0, 0, &sysfont);

			count = 0;

			pisca_led(5, t);
			but2_flag = 0;
			led_flag = 0;
			but_flag = 0;	


			// Entra em sleep mode    
			// Código 'trava' aqui até ser 'acordado' 
			pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);     
		}

		if (but_flag1){
			delay_ms(100);
			count ++;
			
		}

		if (but3_flag){
			t = t + 100;
			freq = 1.0/(t*pow(10,-3));

			sprintf(buffer, "f: %f", freq);
			gfx_mono_draw_string(buffer, 0, 0, &sysfont);

			but3_flag = 0;

		}

	}
}
