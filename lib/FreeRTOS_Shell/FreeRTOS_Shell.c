/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* Standard includes. */
#include <cmsis_gcc.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* custom includes. */
#include "FreeRTOS_CLI.h"
#include "FreeRTOS_Shell.h"
#include "FreeRTOS_Shell_port.h"

/* Private variables ---------------------------------------------------------*/
QueueHandle_t   FreeRTOS_ShellRecvQueue;
static uint8_t  inputBuffer[FREERTOS_SHELL_INPUT_BUFFER_LENGTH];
static uint8_t* inputBuffer_ptr;
static uint8_t  outputBuffer[FREERTOS_CLI_OUTPUT_MAX_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
__WEAK void FreeRTOS_Shell_init(void) {}

/**
 * @brief A FreeRTOS thread, it will handle msg from a msgqueue, and output to UART
 *
 * @note  when there is no input, the thread will suspend and take no CPU time.
 */
void FreeRTOS_Shell(void* argument)
{
    FreeRTOS_Shell_init();
    /* a shell task */
    inputBuffer_ptr         = inputBuffer;
    FreeRTOS_ShellRecvQueue = xQueueCreate(FREERTOS_SHELL_RECV_QUEUE_LENGTH, sizeof(uint8_t));
    configASSERT(FreeRTOS_ShellRecvQueue);

    FreeRTOS_ShellOutput(FREERTOS_SHELL_START_LOGO, strlen(FREERTOS_SHELL_START_LOGO));
    FreeRTOS_ShellOutput("\r\n", 2);
    FreeRTOS_ShellOutput(FREERTOS_SHELL_USER_INFO, strlen(FREERTOS_SHELL_USER_INFO));

    /* regist all cmd using link symbol */
#ifdef __CC_ARM /* ARM C Compiler */
    extern uint8_t SHELL_SECTION$$Base;
    extern uint8_t SHELL_SECTION$$Limit;

    uint8_t* start = &SHELL_SECTION$$Base;
    uint8_t* end   = &SHELL_SECTION$$Limit;
#elif defined __GNUC__
    /* GNU GCC Compiler */
    typedef void (*init_call)(void);
    extern init_call __shell_section_start;
    extern init_call __shell_section_end;

    uint8_t* start = (uint8_t*)&__shell_section_start;
    uint8_t* end   = (uint8_t*)&__shell_section_end;
#endif

    int size = sizeof(CLI_Command_Definition_t);
    for (uint8_t* i = start; i < end; i += size) {
        CLI_Command_Definition_t* cli_command = (CLI_Command_Definition_t*)i;
        FreeRTOS_CLIRegisterCommand(cli_command);
    }

    while (1) {
        /* always wait a queue */
        char recvChar = 0;
        xQueueReceive(FreeRTOS_ShellRecvQueue, &recvChar, portMAX_DELAY);

        bool lineOver           = false;
        bool isInputBufferEmpty = inputBuffer_ptr == inputBuffer;
        if (recvChar == '\r')
            lineOver = true;

        if (lineOver) {
            BaseType_t ret = pdTRUE;
            FreeRTOS_ShellOutput("\r\n", 2);
            if (!isInputBufferEmpty) {
                while (ret == pdTRUE) {
                    ret = FreeRTOS_CLIProcessCommand((const char*)inputBuffer, (char*)outputBuffer, FREERTOS_CLI_OUTPUT_MAX_BUFFER_SIZE);
                    FreeRTOS_ShellOutput((const char*)outputBuffer, strlen((const char*)outputBuffer));
                }
                memset(outputBuffer, 0, FREERTOS_CLI_OUTPUT_MAX_BUFFER_SIZE);
                memset(inputBuffer, 0, FREERTOS_SHELL_INPUT_BUFFER_LENGTH);
                inputBuffer_ptr = inputBuffer;
                FreeRTOS_ShellOutput("\r\n", 2);
            }
            FreeRTOS_ShellOutput(FREERTOS_SHELL_USER_INFO, strlen(FREERTOS_SHELL_USER_INFO));
        }
        else if (!lineOver) {
            /* backspace */
            if (recvChar == 0x7f && isInputBufferEmpty) {
                __NOP();
            }
            /* backspace*/
            else if (recvChar == 0x7f && !isInputBufferEmpty) {
                *--inputBuffer_ptr = 0;
                FreeRTOS_ShellOutput(&recvChar, 1);
            }
            /* tab */
            /* TODO: when double click tab, trigger auto complete */
            else if (recvChar == 0x08) {
                __NOP();
            }
            else {
                *inputBuffer_ptr++ = recvChar;
                FreeRTOS_ShellOutput(&recvChar, 1);
            }
        }
    }
}

void FreeRTOS_ShellIRQHandle(uint8_t recvData)
{
    /* send data to FreeRTOS queue */
    // xQueueSendToBack(FreeRTOS_ShellRecvQueue, &recvData, NULL);
    xQueueSendToBack(FreeRTOS_ShellRecvQueue, &recvData, 0);
}

static int        listAllThreadCallCount = 0;
static BaseType_t listAllThread(char* pcWriteBuffer, size_t xWriteBufferLen, const char* pcCommandString)
{
    BaseType_t  ret     = pdTRUE;
    UBaseType_t taskNum = uxTaskGetNumberOfTasks();
    int         len     = taskNum * FREERTOS_SHELL_EACH_TASKINFO_MAX_SIZE;

    char  stack_malloc[len * sizeof(char)];
    char* _buffer = (char*)&stack_malloc;
    vTaskList(_buffer);
    if (len > xWriteBufferLen * (listAllThreadCallCount + 1)) {
        /* Insufficient buffer size, should be called more than once */
        memcpy(pcWriteBuffer, _buffer + xWriteBufferLen * listAllThreadCallCount, xWriteBufferLen);
        listAllThreadCallCount++;
        ret = pdTRUE;
    }
    else {
        /* sufficient buffer size */
        memcpy(pcWriteBuffer, _buffer + xWriteBufferLen * listAllThreadCallCount, xWriteBufferLen * (listAllThreadCallCount + 1) - len);
        listAllThreadCallCount = 0;
        ret                    = pdFALSE;
    }

    return ret;
}

FREERTOS_SHELL_CMD_REGISTER("ps", "list all thread", listAllThread, 0);
