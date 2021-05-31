/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include <asf.h>
#include <string.h>
#include "ili9341.h"
#include "lvgl.h"
#include "touch/touch.h"

/************************************************************************/
/* STATIC                                                               */
/************************************************************************/

/*A static or global variable to store the buffers*/
static lv_disp_buf_t disp_buf;

/*Static or global buffer(s). The second buffer is optional*/
static lv_color_t buf_1[LV_HOR_RES_MAX * LV_VER_RES_MAX];

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/

#define TASK_LCD_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
  printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
  for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
  configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* globals                                                              */
/************************************************************************/

#define AFEC_POT AFEC1
#define AFEC_POT_ID ID_AFEC1
#define AFEC_POT_CHANNEL 6 // Canal do pino PD30

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t second;
} calendar;

#define LIST_LEN 5
lv_obj_t * label_list[LIST_LEN];
lv_obj_t * label_butOn;
lv_obj_t * label_alarmValue;
lv_obj_t * mbox_alarm;

QueueHandle_t xQueueRx;
QueueHandle_t xQueueAdc;
TimerHandle_t xTimerAdc;


/************************************************************************/
/* prototype                                                            */
/************************************************************************/

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel);
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type);

/************************************************************************/
/* hw handlers                                                          */
/************************************************************************/

void USART1_Handler(void){
  uint32_t ret = usart_get_status(CONSOLE_UART);

  BaseType_t xHigherPriorityTaskWoken = pdTRUE;
  char c;

  // Verifica por qual motivo entrou na interrup?cao
  //  - Dadodispon?vel para leitura
  if(ret & US_IER_RXRDY){
    usart_serial_getchar(CONSOLE_UART, &c);
    xQueueSendFromISR(xQueueRx, (void *) &c, &xHigherPriorityTaskWoken);

    // -  Transmissoa finalizada
    } else if(ret & US_IER_TXRDY){

  }
}

/************************************************************************/
/* lvgl                                                                 */
/************************************************************************/

static void btn_on_handler(lv_obj_t * obj, lv_event_t event) {
  if(event == LV_EVENT_CLICKED) {
    printf("Clicked\n");
  }
}

static void btn_alarmValue_handler(lv_obj_t * obj, lv_event_t event) {
  if(event == LV_EVENT_CLICKED) {
    printf("Clicked\n");
  }
}

void lv_create_head(void) {
  lv_obj_t * btn_on = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_event_cb(btn_on, btn_on_handler);
  lv_obj_align(btn_on, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);
  lv_obj_set_size(btn_on, 105, 105);
  lv_obj_set_style_local_radius(btn_on, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
  
  label_butOn = lv_label_create(btn_on, NULL);
  lv_label_set_text(label_butOn, "ON");
  lv_obj_set_style_local_text_font(label_butOn, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_22);
  
  lv_obj_t * btn_alarmValur = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_event_cb(btn_alarmValur, btn_alarmValue_handler);
  lv_obj_align(btn_alarmValur, NULL, LV_ALIGN_IN_TOP_RIGHT, +50, 0);
  lv_obj_set_size(btn_alarmValur, 105, 105);
  lv_obj_set_style_local_radius(btn_alarmValur, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, 0);
  lv_obj_set_style_local_bg_opa(btn_alarmValur, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_30);

  label_alarmValue = lv_label_create(btn_alarmValur, NULL);
  lv_obj_set_style_local_text_font(label_alarmValue, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_22);
  lv_obj_set_style_local_text_color(label_alarmValue, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE);

  lv_label_set_text(label_alarmValue, "ALARME \n < 2000");
}

void lv_create_alarm(void) {
  mbox_alarm = lv_msgbox_create(lv_scr_act(), NULL);
  lv_msgbox_set_text(mbox_alarm, "Alarme ativado!");
  lv_obj_set_size(mbox_alarm, 200, 300);
  lv_obj_align(mbox_alarm, NULL, LV_ALIGN_CENTER, 0, 0);
}

void lv_create_list(void) {
  lv_obj_t * cont;
  cont = lv_cont_create(lv_scr_act(), NULL);

  lv_obj_align_origo(cont, NULL, LV_ALIGN_CENTER, 0, 0);  /*This parametrs will be sued when realigned*/
  lv_cont_set_fit2(cont, LV_FIT_NONE, LV_FIT_NONE);
  lv_obj_set_size(cont, 240, 190);
  lv_cont_set_layout(cont, LV_LAYOUT_COLUMN_MID);
  lv_obj_align_origo(cont, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_obj_align(cont, NULL, LV_ALIGN_IN_TOP_MID, 0, 120);

  for (int i = 0; i<LIST_LEN; i++ ){
    label_list[i] = lv_label_create(cont, NULL);
    lv_label_set_text_fmt(label_list[i], "%d: HH:MM:SS - valor", i);
    lv_obj_set_style_local_text_font(label_list[i], LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, &lv_font_montserrat_18);
    lv_obj_set_size(label_list[i], 240, 40);
    lv_obj_set_style_local_bg_color(label_list[i], LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_RED);
    lv_obj_set_style_local_bg_opa(label_list[i], LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_OPA_100);
  }
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_lcd(void *pvParameters) {
  for (;;)  {
    lv_tick_inc(50);
    lv_task_handler();
    vTaskDelay(50);
  }
}

static void task_uart(void *pvParameters) {
  char p_data[32];
  char p_cnt = 0;
  char p_do = 0;
  
  for (;;)  {
    char c;
    if (xQueueReceive( xQueueRx, &c, 1000 )) {
      if( p_cnt >= 32 ) {
        p_cnt = 0;
      }          
      p_data[p_cnt++] = c; 
      if (c == 'X'){
        p_do = 1;
      }
    }
    
    if(p_do == 1) {
      p_cnt = 0;
      p_do = 0;
    }

  }
}

void vTimerAfecCallback( TimerHandle_t xTimer ) {
  afec_channel_enable(AFEC_POT, AFEC_POT_CHANNEL);
  afec_start_software_conversion(AFEC_POT);
  vTaskDelay(10);
  int adc = afec_channel_get_value(AFEC_POT, AFEC_POT_CHANNEL);
  xQueueSend(xQueueAdc, &adc, 0);
}

static void task_process(void *pvParameters) {
  xQueueAdc = xQueueCreate(32, sizeof(int));
  config_AFEC_pot(AFEC_POT, AFEC_POT_ID, AFEC_POT_CHANNEL);
  xTimerAdc = xTimerCreate("afec", 4000, pdTRUE, (void *) 0, vTimerAfecCallback);
  xTimerStart( xTimerAdc, 0 );
  
  calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
  RTC_init(RTC, ID_RTC, rtc_initial, NULL);

  while(1) {

  }
}

static void task_main(void *pvParameters) {
  // cria head e lista
  lv_create_head();
  lv_create_list();
  
  // TODO exemplo de alarme, deve ser removido
  vTaskDelay(3000);
  lv_create_alarm();
  vTaskDelay(1000);
  lv_obj_del(mbox_alarm);
  
  for (;;) {

  }
}

/************************************************************************/
/* configs                                                              */
/************************************************************************/

static void configure_lcd(void) {
  /**LCD pin configure on SPI*/
  pio_configure_pin(LCD_SPI_MISO_PIO, LCD_SPI_MISO_FLAGS);  //
  pio_configure_pin(LCD_SPI_MOSI_PIO, LCD_SPI_MOSI_FLAGS);
  pio_configure_pin(LCD_SPI_SPCK_PIO, LCD_SPI_SPCK_FLAGS);
  pio_configure_pin(LCD_SPI_NPCS_PIO, LCD_SPI_NPCS_FLAGS);
  pio_configure_pin(LCD_SPI_RESET_PIO, LCD_SPI_RESET_FLAGS);
  pio_configure_pin(LCD_SPI_CDS_PIO, LCD_SPI_CDS_FLAGS);
  
}

static void configure_console(void) {
  const usart_serial_options_t uart_serial_options = {
    .baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
    .charlength = USART_SERIAL_CHAR_LENGTH,
    .paritytype = USART_SERIAL_PARITY,
    .stopbits = USART_SERIAL_STOP_BIT,
  };

  /* Configure console UART. */
  stdio_serial_init(CONSOLE_UART, &uart_serial_options);

  /* Specify that stdout should not be buffered. */
  setbuf(stdout, NULL);
}

static void USART1_init(void) {
  /* Configura USART1 Pinos */
  sysclk_enable_peripheral_clock(ID_PIOB);
  sysclk_enable_peripheral_clock(ID_PIOA);
  pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4); // RX
  pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
  MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;

  /* Configura opcoes USART */
  const sam_usart_opt_t usart_settings = {
    .baudrate       = 115200,
    .char_length    = US_MR_CHRL_8_BIT,
    .parity_type    = US_MR_PAR_NO,
    .stop_bits   	= US_MR_NBSTOP_1_BIT	,
    .channel_mode   = US_MR_CHMODE_NORMAL
  };

  /* Ativa Clock periferico USART0 */
  sysclk_enable_peripheral_clock(CONSOLE_UART_ID);

  /* Configura USART para operar em modo RS232 */
  usart_init_rs232(CONSOLE_UART, &usart_settings, sysclk_get_peripheral_hz());

  /* Enable the receiver and transmitter. */
  usart_enable_tx(CONSOLE_UART);
  usart_enable_rx(CONSOLE_UART);

  /* map printf to usart */
  ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
  ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;

  /* ativando interrupcao */
  usart_enable_interrupt(CONSOLE_UART, US_IER_RXRDY);
  NVIC_SetPriority(CONSOLE_UART_ID, 4);
  NVIC_EnableIRQ(CONSOLE_UART_ID);

}

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

static void config_AFEC_pot(Afec *afec, uint32_t afec_id, uint32_t afec_channel) {
  /*** Configuracao espec?fica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);
  afec_channel_set_analog_offset(afec, afec_channel, 0x200);
}

/************************************************************************/
/* port lvgl                                                            */
/************************************************************************/

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
  ili9341_set_top_left_limit(area->x1, area->y1);   ili9341_set_bottom_right_limit(area->x2, area->y2);
  ili9341_copy_pixels_to_screen(color_p,  (area->x2 - area->x1) * (area->y2 - area->y1));
  
  /* IMPORTANT!!!
  * Inform the graphics library that you are ready with the flushing*/
  lv_disp_flush_ready(disp_drv);
}

bool my_input_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
  int px, py, pressed;
  
  taskENTER_CRITICAL();
  if (readPoint(&py, &px)) {
    data->state = LV_INDEV_STATE_PR;
  }
  else {
    data->state = LV_INDEV_STATE_REL;
  }
  taskEXIT_CRITICAL();
  data->point.x = px ;
  data->point.y = 320 - py;
  return false; /*No buffering now so no more data read*/
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
  /* board and sys init */
  board_init();
  sysclk_init();
  
  /* uart */
  xQueueRx = xQueueCreate(32, sizeof(char));
  USART1_init();
  configure_console();

  /* LCd int */
  configure_lcd();
  ili9341_init();
  ili9341_set_orientation(ILI9341_FLIP_Y | ILI9341_SWITCH_XY);
  configure_touch();
  ili9341_backlight_on();
  
  /*LittlevGL init*/
  lv_init();
  lv_disp_drv_t disp_drv;                 /*A variable to hold the drivers. Can be local variable*/
  lv_disp_drv_init(&disp_drv);            /*Basic initialization*/
  lv_disp_buf_init(&disp_buf, buf_1, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX);  /*Initialize `disp_buf` with the buffer(s) */
  disp_drv.buffer = &disp_buf;            /*Set an initialized buffer*/
  disp_drv.flush_cb = my_flush_cb;        /*Set a flush callback to draw to the display*/
  lv_disp_t * disp;
  disp = lv_disp_drv_register(&disp_drv); /*Register the driver and save the created display objects*/
  
  /* Init input on LVGL */
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);      /*Basic initialization*/
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_input_read;
  /*Register the driver in LVGL and save the created input device object*/
  lv_indev_t * my_indev = lv_indev_drv_register(&indev_drv);
  
  if (xTaskCreate(task_lcd, "LCD", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY + 1, NULL) != pdPASS) {
    printf("Failed to create lcd task\r\n");
  }
  
  if (xTaskCreate(task_main, "main", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create main task\r\n");
  }
  
  if (xTaskCreate(task_uart, "uart", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create uart task\r\n");
  }
  
  if (xTaskCreate(task_process, "process", TASK_LCD_STACK_SIZE, NULL, TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create process task\r\n");
  }
  
  /* Start the scheduler. */
  vTaskStartScheduler();

  /* RTOS n?o deve chegar aqui !! */
  while(1){ }
}
