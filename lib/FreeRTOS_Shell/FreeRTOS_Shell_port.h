#ifndef __FREERTOS_SHELL_PORT_H
#define __FREERTOS_SHELL_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#define USERCOMM_MAX_OUTPUT 128

void FreeRTOS_ShellOutput(const char * buffer, int length);

#ifdef __cplusplus
}
#endif

#endif /* __FREERTOS_SHELL_PORT_H */
