/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"


/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define LED_PIO           PIOC                 // periferico que controla o LED
#define LED_PIO_ID        ID_PIOC              // ID do perif�rico PIOC (controla LED)
#define LED_PIO_IDX       8                    // ID do LED no PIO
#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

#define BUT_PIO           PIOA                 // periferico que controla o BUTTON
#define BUT_PIO_ID        ID_PIOA              // ID do perif�rico PIOC (controla BUTTON)
#define BUT_PIO_IDX       11                   // ID do BUTTON no PIO
#define BUT_PIO_IDX_MASK  (1 << BUT_PIO_IDX)   // Mascara para CONTROLARMOS o LED

//LED 1
#define OLED_LED1_PIO	  PIOA
#define OLED_LED1_PIO_ID  ID_PIOA
#define OLED_LED1_PIO_IDX 0
#define OLED_LED1_PIO_IDX_MASK (1 << OLED_LED1_PIO_IDX)

//LED2
#define OLED_LED2_PIO	  PIOC
#define OLED_LED2_PIO_ID  ID_PIOC
#define OLED_LED2_PIO_IDX 30
#define OLED_LED2_PIO_IDX_MASK (1 << OLED_LED2_PIO_IDX)

//LED3
#define OLED_LED3_PIO	  PIOB
#define OLED_LED3_PIO_ID  ID_PIOB
#define OLED_LED3_PIO_IDX 2
#define OLED_LED3_PIO_IDX_MASK (1 << OLED_LED3_PIO_IDX)

/************************************************************************/
/* constants                                                            */
/************************************************************************/
typedef struct {
	Pio *pio;
	int pio_id;
	int id;
	int mask;
	int isOn;
} component;

typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t seccond;
} calendar;

component ledPlaca = {
	LED_PIO,
	LED_PIO_ID,
	LED_PIO_IDX,
	LED_PIO_IDX_MASK,
};

component led1 = {
	OLED_LED1_PIO,
	OLED_LED1_PIO_ID,
	OLED_LED1_PIO_IDX,
	OLED_LED1_PIO_IDX_MASK
};
component led2 = {
	OLED_LED2_PIO,
	OLED_LED2_PIO_ID,
	OLED_LED2_PIO_IDX,
	OLED_LED2_PIO_IDX_MASK
};
component led3 = {
	OLED_LED3_PIO,
	OLED_LED3_PIO_ID,
	OLED_LED3_PIO_IDX,
	OLED_LED3_PIO_IDX_MASK
};

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

volatile char flag_tc_4hz = 0;
volatile char flag_tc_5hz = 0;
volatile char flag_rtc = 0;
volatile char flag_rtt = 0;
volatile char string[10];
volatile int seconds = 0;
volatile int secondsDozen = 0;

volatile int minutes = 0;
volatile int minutesDozen = 0;

volatile int hours = 0;
volatile int hoursDozen = 0;


/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);
void configureLeds(component leds[], int size);
void eneblePioPeriph(int periphIdsList[], int size);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses);
void RTC_init(Rtc *rtc, uint32_t id_rtc, uint32_t irq_type);
void turnOffLED(component led);
void turnOnLED(component led);
void writeLCD();
void pisca_led(int n, int t, component led);
void pin_toggle(component led);
void updateTime(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/
void TC1_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc_4hz = 1;
}

void TC2_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	flag_tc_5hz = 1;
}

void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
      flag_rtc = 1;
	}
	
	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		updateTime();
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}

void RTT_Handler(void)
{
	uint32_t ul_status;

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {  }

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		pin_toggle(led2);    // BLINK Led
		flag_rtt = 1;                  // flag RTT alarme
	}
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void turnOnLED(component led){
	pio_clear(led.pio, led.mask);
}

void turnOffLED(component led){
	pio_set(led.pio, led.mask);
}
void writeLCD(){
	gfx_mono_draw_string(string, 0,16, &sysfont);
}

void updateTime(void){
	seconds += 1;
	if (seconds == 10){
		seconds = 0;
		secondsDozen += 1;
		if (secondsDozen == 6){
			secondsDozen = 0;
			minutes += 1;
			if (minutes == 10){
				minutes = 0;
				minutesDozen += 1;
				if (minutesDozen == 6){
					minutesDozen = 0;
					hours += 1;
					if (hours == 10){
						hours = 0;
						hoursDozen += 1;
						if (hoursDozen == 24)
						{
							hoursDozen = 0;
						}
					}
				}
			}
		}
	}
	sprintf(string, "%d%d:%d%d:%d%d ", hoursDozen, hours, minutesDozen, minutes, secondsDozen, seconds);
	writeLCD();
}


void eneblePioPeriph(int periphIdsList[], int size){
	for (int i = 0; i <size; i++)
	{
		pmc_enable_periph_clk(periphIdsList[i]);
	}
}

void pisca_led(int n, int t, component led){
	for (int i=0;i<n;i++){
		turnOnLED(led);
		delay_ms(t);
		turnOffLED(led);
		delay_ms(t);
	}
}

void pin_toggle(component led){
	if(pio_get_output_data_status(led.pio, led.mask))
	pio_clear(led.pio, led.mask);
	else
	pio_set(led.pio,led.mask);
}

void configureLeds(component leds[], int size){

	for (int i = 0; i < size; i++)
	{
		component led = leds[i];
		pio_configure(led.pio, PIO_OUTPUT_1, led.mask, PIO_DEFAULT);
	}
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

	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrupçcão no TC canal 0 */
	/* Interrupção no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}

void RTC_init(Rtc *rtc, uint32_t id_rtc, uint32_t irq_type){
	calendar t = {2018, 3, 19, 12, 15, 45, 1};
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 0);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
	
		/* configura alarme do RTC */
	//rtc_set_date_alarm(RTC, 1, t.month, 1, t.day);
	rtc_set_time_alarm(RTC, 1, t.hour, 1, t.minute, 1, t.seccond + 20);
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
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
}

// Função de inicialização do uC
void init(void){
	board_init();
	// Initialize the board clock
	sysclk_init();
	
	// Desativa WatchDog Timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	int periphIdsList[4] = {
		ID_PIOA,
		ID_PIOB,
		ID_PIOC
	};
	eneblePioPeriph(periphIdsList, 4);
	component leds[4] = {
		ledPlaca,
		led1,
		led2,
		led3
	};
	//Configura os LEDs
	configureLeds(leds, 4);

	/** Configura timer TC0, canal 1 com 4Hz */
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC0, ID_TC2, 2, 5);
	

	/** Configura RTC */
	RTC_init(RTC, ID_RTC, RTC_IER_ALREN | RTC_IER_SECEN);
	
	flag_tc_4hz = 0;
	flag_tc_5hz = 0;
	flag_rtc = 0;
	flag_rtt = 0;

	delay_init();
	//Init OLED Screen
	gfx_mono_ssd1306_init();

}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main (void) {
	init();
	
	 // Inicializa RTT com IRQ no alarme.
	 flag_rtt = 1;

	sprintf(string, "        ");
	writeLCD();
	
	while(1) {
		
		if (flag_tc_4hz) {
			pisca_led(1,1, led1);
			flag_tc_4hz = 0;
		};
		if (flag_tc_5hz) {
			pisca_led(1,1, led3);
			flag_tc_5hz = 0;
		};
		if (flag_rtt){
		  /*
		   * IRQ apos 4s -> 8*0.5
		   */
		  uint16_t pllPreScale = (int) (((float) 32768) / 4.0);
		  uint32_t irqRTTvalue = 8;
      
		  // reinicia RTT para gerar um novo IRQ
		  RTT_init(pllPreScale, irqRTTvalue);         
      
		  flag_rtt = 0;
		}
		if (flag_rtc) {
			RTC_init(RTC, ID_RTC, RTC_IER_ALREN | RTC_IER_SECEN);
			pin_toggle(ledPlaca);
			flag_rtc = 0;

		};
	};
	return 0;
}
