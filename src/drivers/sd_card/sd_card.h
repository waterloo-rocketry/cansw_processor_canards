#ifndef SD_CARD_H
#define SD_CARD_H

#include <fatfs.h>

FRESULT sdCardInit(void);

FRESULT fileRead(char *fileName, void *buffer, UINT bufferSize, UINT *bytesRead);

FRESULT fileWrite(char *fileName, void *buffer, UINT bufferSize, UINT *bytesWritten);

FRESULT fileCreate(char *fileName);

FRESULT fileDelete(char *fileName);

FRESULT sdCardStatus(void);

#endif // SD_CARD_H