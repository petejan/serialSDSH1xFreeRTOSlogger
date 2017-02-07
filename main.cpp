#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "ff/ff.h"
#include "ff/diskio.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "SHT1x.h"
#include "ds3234.h"
#include "xatoi.h"

static void TaskBlinkGreenLED(void* pvParameters);
static void TaskCommand(void* pvParameters);
static void TaskReadSHT(void* pvParameters);

extern xComPortHandle xSerialPort;

/* References to the private heap variables */
// These are system provided variables. The don't need to be further defined.
extern char  __heap_start;
extern char  __heap_end;
extern char *__malloc_heap_start;
extern char *__malloc_heap_end;
extern void *__brkval;
extern void *__flp;

#define DD_MOSI 	3
#define DD_MISO 	4
#define DD_SCK 		5
#define DD_SS 		2

#define DDR_SPI 	DDRB

#define DDR_DS3234_SS  	DDRB
#define PORT_DS3234_SS 	PORTB
#define DS_3234_SS 		2

void spi_init()
{
	avrSerialPrint_P(PSTR("spi_init\r\n"));

    DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK) | (1<<DD_SS);
    DDR_SPI &= ~(1<<DD_MISO);

    DDR_DS3234_SS |= (1<<DS_3234_SS);

    SPCR = (1<<MSTR) | (1<<CPOL) | (1<<CPHA);
    SPCR |= (1<<SPE);

    PORT_DS3234_SS &= ~(1<<DS_3234_SS);
    PORT_DS3234_SS |= (1<<DS_3234_SS);
}

uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while(!(SPSR & (1 <<SPIF)));
    return SPDR;
}

void ds3234_spi_slave_select()
{
    PORT_DS3234_SS &= ~(1<<DS_3234_SS);
}

void ds3234_spi_slave_unselect()
{
    PORT_DS3234_SS |= (1<<DS_3234_SS);
}


//-----------------------------------------------------------

int main()
{
  DDR_DS3234_SS |= (1<<DS_3234_SS);
  PORT_DS3234_SS &= ~(1<<DS_3234_SS);
  PORT_DS3234_SS |= (1<<DS_3234_SS);

  DDRC = 0x0f; // indicator outputs
  PORTC = 0x00; // all low

  xSerialPort = xSerialPortInitMinimal(USART0, 38400, 32, 8);
  ds3234_init(spi_init, spi_transfer, ds3234_spi_slave_select, ds3234_spi_slave_unselect);

//  /* Start 100Hz system timer with TC0 */
//  OCR0A = F_CPU / 1024 / 100 - 1;
//  TCCR0A = _BV(WGM01);
//  TCCR0B = 0b101;
//  TIMSK0 = _BV(OCIE0A);

  avrSerialPrint_P(PSTR("FreeRTOS SD SHT1x DS3234 "__TIME__"\r\n"));
  avrSerialPrint_P(PSTR("Starting scheduler\r\n"));

  xTaskCreate(TaskBlinkGreenLED, (const portCHAR*) "GreenLED", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
  xTaskCreate(TaskCommand, (const portCHAR*) "Command", 128, NULL, 3, NULL);
  xTaskCreate(TaskReadSHT, (const portCHAR*) "ReadSHT", 300, NULL, 2, NULL);

//  avrSerialPrintf_P(PSTR("Heap start addr:        %x\r\n"),  (int16_t) __heap_start);
//  avrSerialPrintf_P(PSTR("Heap end   addr:        %x\r\n"),  (int16_t) __heap_end);
//  avrSerialPrintf_P(PSTR("malloc Heap start addr: %x\r\n"),  (int16_t) __malloc_heap_start);
//  avrSerialPrintf_P(PSTR("malloc Heap end   addr: %x\r\n"),  (int16_t) __malloc_heap_end);
//  avrSerialPrintf_P(PSTR("brkval addr:            %x\r\n"),  (int16_t) __brkval);
  avrSerialPrintf_P(PSTR("\r\nFree Heap Size: %u\r\n"), xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.

  vTaskStartScheduler();

  for (;;)
    ;;

  return 0;
}

//-----------------------------------------------------------

// Main Green LED Flash
static void TaskBlinkGreenLED(void* pvParameters)
{
  // set pin 0 of PORTC for output
  DDRC |= _BV(DDB0);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    // LED on
    PORTC |= _BV(PORTC0);
    vTaskDelayUntil(&xLastWakeTime, (100 / portTICK_PERIOD_MS));

    // LED off
    PORTC &= ~_BV(PORTC0);
    vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_PERIOD_MS));
  }

  vTaskDelete(NULL);
}

//-----------------------------------------------------------
SHT1x sh1 = SHT1x();
static TaskHandle_t xTaskToNotify = NULL;
TaskHandle_t xUSARTTaskToNotify = NULL;
DS3234_TIME time;
DS3234_DATE date;

uint8_t readCount;

BYTE Buff[64];	/* Working buffer */
FATFS *fs;

/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/

DWORD get_fattime (void)
{
	/* Pack date and time into a DWORD variable */
	return	  ((DWORD)(date.year + 2000 - 1980) << 25)
			| ((DWORD)date.month << 21)
			| ((DWORD)date.day_of_month << 16)
			| ((DWORD)time.hours << 11)
			| ((DWORD)time.minutes << 5)
			| ((DWORD)time.seconds >> 1);
}

uint8_t canSleep;

static void TaskReadSHT(void* pvParameters)
{
	float temp, humid;
	uint32_t ulNotificationValue;
	uint8_t i;
	uint8_t htr;
    FRESULT fr;
    FIL *file;

	// set pin 5 of PORTB for output
	DDRB |= _BV(DDB5);
	// LED off
    PORTB &= ~_BV(PORTB5);

    canSleep = 3;

    ds3234_read_time(&time);
    time.seconds = 0; // create an alarm at 0 seconds
    ds3234_read_date(&date);

    ds3234_write_alarm1(&date, &time, DS3234_ALARM1_SECONDS_MATCH);

    uint8_t constat_register; // Short for control-status register
    constat_register = ds3234_read_register(DS3234_REG_CONTROL);
    constat_register |= (1 << DS3234_CONTROL_INTCN) | (1 << DS3234_CONTROL_A1IE);
    ds3234_write_register(DS3234_REG_CONTROL, constat_register);
    // clear INT0 bit
    constat_register = ds3234_read_register(DS3234_REG_CTRL_STATUS);
    constat_register &= ~(1 << DS3234_CONSTAT_A1F);
    ds3234_write_register(DS3234_REG_CTRL_STATUS, constat_register);

    //EICRA = 0b10; // falling edge on INT0
    EICRA = 0b0; // low level on INT0
    EIMSK = _BV(INT0);

	vTaskDelay(100 / portTICK_PERIOD_MS);

    xSerialPrint_P(PSTR("ReadSHT task\r\n"));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    fs = (FATFS *)pvPortMalloc(sizeof(FATFS));
    if (fs == NULL)
    {
  	  xSerialPrint_P(PSTR("Could not allocate FATFS\r\n"));
    }

    xSerialPrintf_P(PSTR("mount rc=%d\r\n"), f_mount(fs, "", 1));

    file = (FIL *)pvPortMalloc(sizeof(FIL));
    if (file == NULL)
    {
  	  xSerialPrintf_P(PSTR("Could not allocate FIL %d\r\n"), sizeof(FIL));
    }

    fr = f_open(file, "SHT15.LOG", FA_OPEN_APPEND | FA_WRITE);
    xSerialPrintf_P(PSTR("file open rc=%d\r\n"), fr);

    readCount = 1;
    htr = 0;
	while(true)
	{
	    /* Store the handle of the calling task. */
	    xTaskToNotify = xTaskGetCurrentTaskHandle();

		PORTC &= ~_BV(PORTC3);

	    if (canSleep == 0)
	    {
			portENTER_CRITICAL();
		    SPCR &= ~(1<<SPE);
		    PORTB &= ~_BV(PORTB5); // LED off

			/* power down, wake by RTC alarm on INT0 */
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			/* We can sleep in critical section because on exit real
			   I-bit of SREG is restored and prvSleep() do SEI on its own. */
			/* Re-enabling interrupts to awake and go to sleep*/
			sleep_enable();
			sei();
			sleep_cpu();
			/* Sleeps here until awaken, then continues */
			sleep_disable();
			cli();
		    SPCR |= (1<<SPE);
			portEXIT_CRITICAL();
	    }

		ulNotificationValue = ulTaskNotifyTake( pdTRUE, 62000 / portTICK_PERIOD_MS );
		PORTC |= _BV(PORTC3);

		uint8_t constat_register; // byte for control-status register
	    constat_register = ds3234_read_register(DS3234_REG_CTRL_STATUS);
	    constat_register &= ~((1 << DS3234_CONSTAT_A1F) | (1 << DS3234_CONSTAT_A2F));
	    ds3234_write_register(DS3234_REG_CTRL_STATUS, constat_register);

	    if( ulNotificationValue == 1 )
		{
		    //xSerialPrint_P(PSTR("notify\r\n"));

			// LED on
			PORTB |= _BV(PORTB5);

			for(i=0;i<readCount;i++)
			{
				if (i == 3)
				{
				    xSerialPrint_P(PSTR("HTR On\r\n"));
					sh1.setHeater(1);
					htr = 1;
				}
				else if (i == 20)
				{
					sh1.setHeater(0);
				    xSerialPrint_P(PSTR("HTR Off\r\n"));
				    htr = 0;
				}

				ds3234_read_time(&time);
				ds3234_read_date(&date);

				xSerialPrintf_P(PSTR("%02d-%02d-%02d"), date.year + 2000, date.month, date.day_of_month);
				xSerialPrintf_P(PSTR(" %02d:%02d:%02d"), time.hours, time.minutes, time.seconds);

				xSerialPrintf_P(PSTR(" %2d %d "), i, htr);

				temp = sh1.readTemperatureC();
				humid = sh1.readHumidity();

				xSerialPrintf_P(PSTR(" th : %5.2f C %5.2f %%\r\n"), temp, humid);

				snprintf_P((char *)Buff, sizeof(Buff),
						PSTR("%02d-%02d-%02d %02d:%02d:%02d %2d %d  th : %5.2f C %5.2f %%\r\n"),
						date.year + 2000, date.month, date.day_of_month,
						time.hours, time.minutes, time.seconds, i, htr, temp, humid );

//				snprintf_P((char *)Buff, sizeof(Buff), "TEST\n");

				unsigned int n = strlen((char *)Buff);

				unsigned int write;
				fr = f_write(file, Buff, n, &write);
				//xSerialPrintf_P(PSTR("write rc=%d written %d\r\n"), fr, write);
			}
			readCount = 1;

			if (time.minutes == 10) // at 10 mins every hour, to a heater cycle
			{
			    xSerialPrint_P(PSTR("HTR Auto Next\r\n"));
				readCount = 25;
			}
			if (time.minutes == 59)
			{
			    xSerialPrintf_P(PSTR("file sync rc=%d\r\n"), f_sync(file));
			}

			if (canSleep > 0)
			{
				canSleep--;
			}

			vTaskDelay(100 / portTICK_PERIOD_MS);

			// LED off
			PORTB &= ~_BV(PORTB5);
		}
		else
		{
		    xSerialPrint_P(PSTR("timeout\r\n"));
		    vTaskDelay(100);
		}
	}
}

static void put_dump (const BYTE *buff, DWORD ofs, BYTE cnt)
{
	BYTE i;

	xSerialPrintf_P(PSTR("%08lX:"), ofs);

	for(i = 0; i < cnt; i++)
		xSerialPrintf_P(PSTR(" %02X"), buff[i]);

	xSerialPutChar(&xSerialPort, ' ');
	for(i = 0; i < cnt; i++)
		xSerialPutChar(&xSerialPort, (buff[i] >= ' ' && buff[i] <= '~') ? buff[i] : '.');

	xSerialPrint_P(PSTR("\r\n"));
}


static void get_line (uint8_t *buff, uint8_t len)
{
	uint8_t c;
	uint8_t i = 0;
	uint32_t ulNotificationValue;

	for (;;)
	{
	    /* Store the handle of the calling task. */
	    xUSARTTaskToNotify = xTaskGetCurrentTaskHandle();

		while ( ! xSerialGetChar( &xSerialPort, &c ))
		{
			ulNotificationValue = ulTaskNotifyTake( pdTRUE, 100 / portTICK_PERIOD_MS );
		}

		if (c == '\r') break;
		if ((c == '\b') && i)
		{
			--i;
			xSerialPutChar( &xSerialPort, c );
			continue;
		}
		if (c >= ' ' && i < len - 1)
		{	/* Visible chars */
			buff[i++] = c;
			xSerialPutChar( &xSerialPort, c );
		}
	}
	buff[i] = 0;
	xSerialPrint_P(PSTR("\r\n"));
}

static uint8_t LineBuffer[25]; // enough for t yyyy mm dd hh mm ss

static void put_rc(FRESULT fr)
{
	xSerialPrintf_P(PSTR("rc=%d\r\n"), fr);
}

//DIR Dir;			/* Directory object */
//FILINFO Finfo;
//DWORD AccSize;				/* Work register for fs command */
//WORD AccFiles, AccDirs;
//
//static
//FRESULT scan_files (
//	char* path		/* Pointer to the working buffer with start path */
//)
//{
//	DIR dirs;
//	FRESULT fr;
//	int i;
//
//	fr = f_opendir(&dirs, path);
//	if (fr == FR_OK)
//	{
//		while (((fr = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0])
//		{
//			if (Finfo.fattrib & AM_DIR)
//			{
//				AccDirs++;
//				i = strlen(path);
//				path[i] = '/'; strcpy(path+i+1, Finfo.fname);
//				fr = scan_files(path);
//				path[i] = 0;
//				if (fr != FR_OK) break;
//			}
//			else
//			{
////				xprintf(PSTR("%s/%s\n"), path, Finfo.fname);
//				AccFiles++;
//				AccSize += Finfo.fsize;
//			}
//		}
//	}
//
//	return fr;
//}

static void TaskCommand(void* pvParameters)
{
  uint8_t *ptr;

  long p2;
  int32_t p1;
  BYTE b1;
  int16_t s1;
  float temp;
  FRESULT fr;

  xSerialPrint_P(PSTR("Command task\r\n"));

  while (true)
  {
	  ptr = LineBuffer;

	  get_line(ptr, (sizeof(LineBuffer)));
	  //SPCR |= (1<<SPE);

	  //xSerialPrintf_P(PSTR("line : %s\r\n"), LineBuffer);

	  switch (*ptr++)
	  {
		case 'r':
			temp = sh1.readTemperatureC();
			xSerialPrintf_P(PSTR("read : %5.2f C\r\n"), temp);
			break;
		  case 'n':
			  xSerialPrintf_P(PSTR("disk i rc=%d\r\n"), disk_initialize(DRV_MMC));
			  break;
		  case 'f':
			  xSerialPrintf_P(PSTR("mount rc=%d\r\n"), f_mount(fs, "", 1));
			  break;
		  case 'h':
			  xSerialPrintf_P(PSTR("Free Heap Size: %u\r\n"), xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.
			  break;
		  case 's' :	/* s - Show disk status */
			if (disk_ioctl(DRV_MMC, GET_SECTOR_COUNT, &p2) == RES_OK)
				{ xSerialPrintf_P(PSTR("Drive size: %lu sectors\r\n"), p2); }
			if (disk_ioctl(DRV_MMC, GET_BLOCK_SIZE, &p2) == RES_OK)
				{ xSerialPrintf_P(PSTR("Erase block: %lu sectors\r\n"), p2); }
			if (disk_ioctl(DRV_MMC, MMC_GET_TYPE, &b1) == RES_OK)
				{ xSerialPrintf_P(PSTR("Card type: %u\r\n"), b1); }
			if (disk_ioctl(DRV_MMC, MMC_GET_CSD, Buff) == RES_OK)
				{ xSerialPrint_P(PSTR("CSD:\r\n")); put_dump(Buff, 0, 16); }
			if (disk_ioctl(DRV_MMC, MMC_GET_CID, Buff) == RES_OK)
				{ xSerialPrint_P(PSTR("CID:\r\n")); put_dump(Buff, 0, 16); }
			if (disk_ioctl(DRV_MMC, MMC_GET_OCR, Buff) == RES_OK)
				{ xSerialPrint_P(PSTR("OCR:\r\n")); put_dump(Buff, 0, 4); }
			if (disk_ioctl(DRV_MMC, MMC_GET_SDSTAT, Buff) == RES_OK) {
				xSerialPrint_P(PSTR("SD Status:\r\n"));
				for (s1 = 0; s1 < 64; s1 += 16) put_dump(Buff+s1, s1, 16);
			}
			if (disk_ioctl(DRV_MMC, ATA_GET_MODEL, Buff) == RES_OK)
				{ Buff[40] = '\0'; xSerialPrintf_P(PSTR("Model: %s\r\n"), Buff); }
			if (disk_ioctl(DRV_MMC, ATA_GET_SN, Buff) == RES_OK)
				{ Buff[20] = '\0'; xSerialPrintf_P(PSTR("S/N: %s\r\n"), Buff); }
			break;
		  case 't' :	/* t [<year yy> <month mm> <date dd> <hour hh> <minute mm> <second ss>] */
			if (xatoi(&ptr, &p1))
			{
				date.year = (uint8_t)p1 - 2000;
				xatoi(&ptr, &p1); date.month = (uint8_t)p1;
				xatoi(&ptr, &p1); date.day_of_month = (uint8_t)p1;
				xatoi(&ptr, &p1); time.hours = (uint8_t)p1;
				xatoi(&ptr, &p1); time.minutes = (uint8_t)p1;
				if (!xatoi(&ptr, &p1))
					break;
				time.seconds = (uint8_t)p1;

				xSerialPrintf_P(PSTR("set %02d-%02d-%02d"), date.year + 2000, date.month, date.day_of_month);
				xSerialPrintf_P(PSTR(" %02d:%02d:%02d\r\n"), time.hours, time.minutes, time.seconds);

				ds3234_write_date(&date);
				ds3234_write_time(&time);
			}
			else
			{
				ds3234_read_time(&time);
				ds3234_read_date(&date);

				xSerialPrintf_P(PSTR("%02d-%02d-%02d"), date.year + 2000, date.month, date.day_of_month);
				xSerialPrintf_P(PSTR(" %02d:%02d:%02d\r\n"), time.hours, time.minutes, time.seconds);
				break;
			}
			break;
		  case 'l' :	/* l - Show logical drive status */
			fr = f_getfree("/", (DWORD*)&p2, &fs);
			if (fr) { put_rc(fr); break; }
			xSerialPrintf_P(PSTR("FAT type = %u\r\n"), fs->fs_type);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("Bytes/Cluster = %lu\r\n"), (DWORD)fs->csize*512);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("Number of FATs = %u\r\n"), fs->n_fats);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("Root DIR entries = %u\r\n"), fs->n_rootdir);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("Sectors/FAT = %lu\r\n"), fs->fsize);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("Number of clusters = %lu\r\n"), fs->n_fatent);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("FAT start (lba) = %lu\r\n"), fs->fatbase);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("DIR start (lba,clustor) = %lu\r\n"), fs->dirbase);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			xSerialPrintf_P(PSTR("Data start (lba) = %lu\r\n"), fs->database);
			vTaskDelay(100 / portTICK_PERIOD_MS);
#if _USE_LABEL
				fr = f_getlabel(ptr2, (char*)Buff, (DWORD*)&p1);
				if (fr) { put_rc(fr); break; }
				xprintf(Buff[0] ? PSTR("Volume name is %s\n") : PSTR("No volume label\n"), Buff);
				xprintf(PSTR("Volume S/N is %04X-%04X\n"), (WORD)((DWORD)p1 >> 16), (WORD)(p1 & 0xFFFF));
#endif
//			xSerialPrint_P(PSTR("scan_files..."));
//			AccSize = AccFiles = AccDirs = 0;
//			fr = scan_files((char *)"0:");
//			if (fr) { put_rc(fr); break; }
//			xSerialPrintf_P(PSTR("\r%u files, %lu bytes.\r\n"), AccFiles, AccSize);
//			vTaskDelay(100 / portTICK_PERIOD_MS);
//			xSerialPrintf_P(PSTR("%u folders.\r\n"), AccDirs);
//			vTaskDelay(100 / portTICK_PERIOD_MS);
//			xSerialPrintf_P(PSTR("%lu KB total disk space.\n\r"), (fs->n_fatent - 2) * fs->csize / 2);
//			vTaskDelay(100 / portTICK_PERIOD_MS);
//			xSerialPrintf_P(PSTR("%lu KB available.\r\n"), p2 * fs->csize / 2);
//			vTaskDelay(100 / portTICK_PERIOD_MS);
			break;
//		  case 'd' :	/* fl [<path>] - Directory listing */
//			fr = f_opendir(&Dir, (char *)ptr);
//			if (fr) { put_rc(fr); break; }
//			p1 = s1 = s2 = 0;
//			for(;;)
//			{
//				fr = f_readdir(&Dir, &Finfo);
//				if ((fr != FR_OK) || !Finfo.fname[0]) break;
//				if (Finfo.fattrib & AM_DIR)
//				{
//					s2++;
//				}
//				else
//				{
//					s1++; p1 += Finfo.fsize;
//				}
//				xSerialPrintf_P(PSTR("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s\n"),
//							(Finfo.fattrib & AM_DIR) ? 'D' : '-',
//							(Finfo.fattrib & AM_RDO) ? 'R' : '-',
//							(Finfo.fattrib & AM_HID) ? 'H' : '-',
//							(Finfo.fattrib & AM_SYS) ? 'S' : '-',
//							(Finfo.fattrib & AM_ARC) ? 'A' : '-',
//							(Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
//							(Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
//							(DWORD)Finfo.fsize,
//							Finfo.fname);
//			}
//			if (fr == FR_OK)
//			{
//				xSerialPrintf_P(PSTR("%4u File(s),%10lu bytes total\n%4u Dir(s)"), s1, p1, s2);
//				if (f_getfree("/", (DWORD*)&p1, &fs) == FR_OK)
//				{
//					xSerialPrintf_P(PSTR(", %10luKiB free\n"), p1 * fs->csize / 2);
//				}
//			}
//			if (fr) put_rc(fr);
//			break;
		  case 'o':
			  xSerialPrintf_P(PSTR("OS Ticks %u\r\n"), xTaskGetTickCount());
			  break;
		  case 'k':
			  	unsigned char alarm_mask;
			  	ds3234_read_alarm1(&date, &time, &alarm_mask);

				xSerialPrintf_P(PSTR("ALM: %02d-%02d-%02d"), date.year + 2000, date.month, date.day_of_month);
				xSerialPrintf_P(PSTR(" %02d:%02d:%02d MSK 0x%02x\r\n"), time.hours, time.minutes, time.seconds, alarm_mask);

				vTaskDelay(100);

				portENTER_CRITICAL();
				/* power down, wake by RTC alarm on INT0 */
				set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				/* We can sleep in critical section because on exit real
				   I-bit of SREG is restored and prvSleep() do SEI on its own. */
				/* Re-enabling interrupts to awake and go to sleep*/
				sleep_enable();
				sei();
				sleep_cpu();
				/* Sleeps here until awaken, then continues */
				sleep_disable();
				cli();
				portEXIT_CRITICAL();
				break;
		  case 'c':
			    xSerialPrintf_P(PSTR("can sleep OS Ticks %u\r\n"), xTaskGetTickCount());
			    canSleep = 0;
			    break;
		  default:
			xSerialPrint((const unsigned char *)"unknown command\r\n");
			break;
	  }
  }
}

/* Interrupt from RTC alarm */

ISR( INT0_vect ) __attribute__ ((hot, flatten));
ISR( INT0_vect )
{
	static signed char xHigherPriorityTaskWoken;

	vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
	xTaskToNotify = NULL;

    SPCR |= (1<<SPE);

	// clear RTC alarm output
	uint8_t constat_register; // byte for control-status register
    constat_register = ds3234_read_register(DS3234_REG_CTRL_STATUS);
    constat_register &= ~((1 << DS3234_CONSTAT_A1F) | (1 << DS3234_CONSTAT_A2F));
    ds3234_write_register(DS3234_REG_CTRL_STATUS, constat_register);

    EIFR |= _BV(INTF0); // clear the interrupt flag

	if( xHigherPriorityTaskWoken != pdFALSE )
		taskYIELD();
}

//-----------------------------------------------------------

extern "C"
{
uint8_t disk_tick;

void vApplicationIdleHook( void )
{
	//PORTC ^= _BV(PORTC1); // we're IDLE
}

void vApplicationTickHook( void )
{
	//PORTC ^= _BV(PORTC2); // tick-toc
	disk_tick++;
	if (disk_tick > 10)
	{
		disk_timerproc();	/* Drive timer procedure of low level disk I/O module */
		disk_tick = 0;
	}
}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, portCHAR* pcTaskName)
{
  SPCR &= ~(1<<SPE); // disable the SCLK output on PORTB5

  // main LED on
  DDRB |= _BV(DDB5);
  PORTB |= _BV(PORTB5);

  // die
  while (true)
  {
    PORTB |= _BV(PORTB5);
    _delay_ms(250);
    PORTB &= ~_BV(PORTB5);
    _delay_ms(250);
  }
}
