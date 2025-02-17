#include "sd_card.h"

// Global static objects for the FATFS file system and the mutex to ensure thread safety.
static SemaphoreHandle_t sd_mutex = NULL;

w_status_t sd_card_init(void) {
    /*
     * Mount the filesystem.
     * The f_mount() function links the FATFS file system object (SDFatFS) with the logical drive.
     * The second parameter (an empty string "") indicates the default drive.
     * The third parameter (1) forces the mount.
     */
    if (f_mount(&SDFatFS, "", 1) != FR_OK) {
        return W_FAILURE;
    }

    /*
     * Create a mutex for thread-safe SD card operations.
     * This mutex will ensure that concurrent tasks do not access the SD card simultaneously,
     * which helps prevent data corruption.
     */
    sd_mutex = xSemaphoreCreateMutex();
    if (sd_mutex == NULL) {
        return W_FAILURE;
    }

    return W_SUCCESS;
}

w_status_t file_read(char *fileName, void *buffer, uint32_t bufferSize, uint32_t *bytesRead) {
    /* Ensure thread-safe access to the SD card. */
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) != pdTRUE) {
        return W_FAILURE;
    }

    FIL file;
    FRESULT res;

    /* Open the file in read mode. */
    res = f_open(&file, fileName, FA_READ);
    if (res != FR_OK) {
        xSemaphoreGive(sd_mutex);
        return W_FAILURE;
    }

    /* Read data into buffer. f_read expects a pointer to a UINT variable, that's why I have
     * declared UINT localBytesRead*/
    {
        UINT localBytesRead;
        res = f_read(&file, buffer, bufferSize, &localBytesRead);
        if (res != FR_OK) {
            f_close(&file);
            xSemaphoreGive(sd_mutex);
            return W_FAILURE;
        }
        *bytesRead = (uint32_t)localBytesRead;
    }

    /* Close the file and release the mutex. */
    f_close(&file);
    xSemaphoreGive(sd_mutex);
    return W_SUCCESS;
}

w_status_t file_write(char *fileName, void *buffer, uint32_t bufferSize, uint32_t *bytesWritten) {
    /* Acquire the mutex */
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) != pdTRUE) {
        return W_FAILURE;
    }

    FIL file;
    FRESULT res;

    /* Open the file in write mode.
     * Use FA_WRITE | FA_OPEN_EXISTING to ensure the file exists; if it does not, the operation
     * fails.
     */
    res = f_open(&file, fileName, FA_WRITE | FA_OPEN_EXISTING);
    if (res != FR_OK) {
        xSemaphoreGive(sd_mutex);
        return W_FAILURE;
    }

    /* Write data from buffer to file. */
    UINT localBytesWritten;
    res = f_write(&file, buffer, bufferSize, &localBytesWritten);
    if (res != FR_OK) {
        f_close(&file);
        xSemaphoreGive(sd_mutex);
        return W_FAILURE;
    }

    *bytesWritten = (uint32_t)localBytesWritten;

    /* Close the file and release the mutex. */
    f_close(&file);
    xSemaphoreGive(sd_mutex);
    return W_SUCCESS;
}

w_status_t file_create(char *fileName) {
    /* Acquire the mutex  */
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) != pdTRUE) {
        return W_FAILURE;
    }

    FIL file;
    FRESULT res;

    /* Create a new file. The FA_CREATE_NEW flag causes the function to fail if the file already
     * exists. */
    res = f_open(&file, fileName, FA_WRITE | FA_CREATE_NEW);
    if (res != FR_OK) {
        xSemaphoreGive(sd_mutex);
        return W_FAILURE;
    }

    /* Close the file immediately since we just want to create it. */
    f_close(&file);
    xSemaphoreGive(sd_mutex);
    return W_SUCCESS;
}

w_status_t file_delete(char *fileName) {
    /* Acquire the mutex. */
    if (xSemaphoreTake(sd_mutex, portMAX_DELAY) != pdTRUE) {
        return W_FAILURE;
    }

    FRESULT res;
    res = f_unlink(fileName);
    xSemaphoreGive(sd_mutex);

    if (res != FR_OK) {
        return W_FAILURE;
    }
    return W_SUCCESS;
}
