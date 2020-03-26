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

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);
void configureLeds(component leds[], int size);
void eneblePioPeriph(int periphIdsList[], int size);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);
void turnOffLED(component led);
void turnOnLED(component led);
void writeLCD(char string[128]);
void pisca_led(int n, int t, component led);

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
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
      flag_rtc = 1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
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
void writeLCD(char string[128]){
	gfx_mono_draw_string(string, 0,16, &sysfont);
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

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type){
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

// 	//Configura os botões
// 	configureButtons(myBoardComponents.buttons, 3);
 	
	/** Configura timer TC0, canal 1 com 4Hz */
	TC_init(TC0, ID_TC1, 1, 4);
	TC_init(TC0, ID_TC2, 2, 5);
	

	calendar rtc_initial = {2018, 3, 19, 12, 15, 45, 1};
	/** Configura RTC */
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);
	
	/* configura alarme do RTC */
	rtc_set_date_alarm(RTC, 1, rtc_initial.month, 1, rtc_initial.day);
	rtc_set_time_alarm(RTC, 1, rtc_initial.hour, 1, rtc_initial.minute, 1, rtc_initial.seccond + 20);
	
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
	
	char string[128];
	sprintf(string, "       ");
	writeLCD(string);

	while(1) {
		if (flag_tc_4hz) {
			pisca_led(1,1, led1);
			flag_tc_4hz = 0;
		};
		if (flag_rtt) {
			pisca_led(1,1, led2);
			flag_rtt = 0;
		};
		if (flag_tc_5hz) {
			pisca_led(1,1, led3);
			flag_tc_5hz = 0;
		};
		if (flag_rtc) {
			pisca_led(5, 200, ledPlaca);
			flag_rtc = 0;
		};
	};
	return 0;
}
