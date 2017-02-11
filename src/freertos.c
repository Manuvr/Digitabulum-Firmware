#if defined(STM32F7xx)

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId filesystemTaskHandle;
osThreadId usbTaskHandle;
osMessageQId myQueue01Handle;

uint8_t retSD;    /* Return value for SD */
char SD_Path[4];  /* SD logical drive path */

/* Function prototypes -------------------------------------------------------*/
void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);


void MX_FREERTOS_Init(); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init() {
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartTask01, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of filesystemTask */
  osThreadDef(filesystemTask, StartTask02, osPriorityLow, 0, 128);
  filesystemTaskHandle = osThreadCreate(osThread(filesystemTask), NULL);

  /* definition and creation of usbTask */
  osThreadDef(usbTask, StartTask03, osPriorityIdle, 0, 128);
  usbTaskHandle = osThreadCreate(osThread(usbTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartTask01 function */
void StartTask01(void const * argument) {
  /* init code for FATFS */
  //MX_FATFS_Init();

  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop */
  for(;;) {
    osDelay(20);
  }
}

/* StartTask02 function */
void StartTask02(void const * argument) {
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}

/* StartTask03 function */
void StartTask03(void const * argument) {
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}


void MX_FATFS_Init() {
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SD_Path);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void) {
  /* USER CODE BEGIN get_fattime */
  return 0;
}

#endif   //STM32F7xx
