#include "drivers/sd_card/sd_card.h"
#include "FreeRTOS.h"
#include "application/logger/log.h"
#include "fatfs.h"
#include "semphr.h"

FATFS g_fs_obj;

sd_card_health_t sd_card_health = {0};

// Only 1 SD card mutex is needed because only 1 sd card exists
SemaphoreHandle_t sd_mutex = NULL;

w_status_t sd_card_init(void) {
    // attempting to init the module >1 time is an error
    if (sd_card_health.is_init) {
        log_text(1, "SDCard", "WARN: Attempted to re-initialize SD card driver.");
        return W_FAILURE;
    }
    /*
     * Mount the filesystem.
     * The f_mount() function links the FATFS file system object (SDFatFS) with the logical drive.
     * The second parameter (an empty string "") indicates the default drive.
     * The third parameter (1) forces the mount. Decided to force mount here so this init
     * function can check if the volume mount actually worked or not.
     */
    // NOTE: f_mount internally calls a HAL sd init, which uses HAL_Delay(), which
    // depends on the systick timer, which is disabled if all interrupts are masked.
    // Freertos masks all interrupts before scheduler starts. Thus this function
    // must only be called AFTER scheduler starts.
    if (f_mount(&g_fs_obj, "", 1) != FR_OK) {
        log_text(1, "SDCard", "ERROR: f_mount failed during init.");
        return W_FAILURE;
    }

    /*
     * Create a mutex for thread-safe SD card operations.
     * This mutex will ensure that concurrent tasks do not access the SD card simultaneously,
     * which helps prevent data corruption.
     */
    sd_mutex = xSemaphoreCreateMutex();
    if (sd_mutex == NULL) {
        log_text(1, "SDCard", "ERROR: Failed to create SD card mutex.");
        f_mount(NULL, "", 0); // Unmount
        return W_FAILURE;
    }

    sd_card_health.is_init = true;
    return W_SUCCESS;
}

w_status_t sd_card_file_read(
    const char *file_name, char *buffer, uint32_t bytes_to_read, uint32_t *bytes_read
) {
    // validate args
    if (!sd_card_health.is_init || file_name == NULL || buffer == NULL || bytes_read == NULL) {
        log_text(1, "SDCard", "ERROR: sd_card_file_read invalid parameters.");
        return W_INVALID_PARAM;
    }

    /* Ensure thread-safe access to the SD card. */
    // use timeout 0 to avoid blocking
    if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
        log_text(1, "SDCard", "WARN: Failed to take SD mutex for read (%s).", file_name);
        return W_FAILURE;
    }

    FIL file;
    FRESULT res;

    /* Open the file in read mode. */
    res = f_open(&file, file_name, FA_READ);
    if (res != FR_OK) {
        xSemaphoreGive(sd_mutex);
        sd_card_health.err_count++;
        log_text(1, "SDCard", "ERROR: f_open failed for read (%s, code: %d).", file_name, res);
        return W_FAILURE;
    }

    // read into provided buffer
    res = f_read(&file, buffer, bytes_to_read, (UINT *)bytes_read);
    if (res != FR_OK) {
        f_close(&file);
        xSemaphoreGive(sd_mutex);
        sd_card_health.err_count++;
        log_text(1, "SDCard", "ERROR: f_read failed (%s, code: %d).", file_name, res);
        return W_FAILURE;
    }

    /* Close the file and release the mutex. */
    f_close(&file);
    xSemaphoreGive(sd_mutex);
    sd_card_health.read_count++;
    return W_SUCCESS;
}

w_status_t sd_card_file_write(
    const char *file_name, const char *buffer, uint32_t bytes_to_write, bool append,
    uint32_t *bytes_written
) {
    // validate args
    if (!sd_card_health.is_init || file_name == NULL || buffer == NULL || bytes_written == NULL) {
        log_text(1, "SDCard", "ERROR: sd_card_file_write invalid parameters.");
        return W_INVALID_PARAM;
    }

    /* Acquire the mutex */
    if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
        log_text(1, "SDCard", "WARN: Failed to take SD mutex for write (%s).", file_name);
        return W_FAILURE;
    }

    FIL file;
    FRESULT res;

    /* Open the file in write mode.
     * Use FA_WRITE | FA_OPEN_EXISTING to ensure the file exists; if it does not, the operation
     * fails.
     */
    res = f_open(&file, file_name, FA_WRITE | FA_OPEN_EXISTING);
    if (res != FR_OK) {
        xSemaphoreGive(sd_mutex);
        sd_card_health.err_count++;
        log_text(1, "SDCard", "ERROR: f_open failed for write (%s, code: %d).", file_name, res);
        return W_FAILURE;
    }

    // move the file r/w ptr to either start or end of file depending on `append`
    if (true == append) {
        res = f_lseek(&file, f_size(&file));
    } else {
        res = f_lseek(&file, 0);
    }
    if (res != FR_OK) {
        f_close(&file);
        xSemaphoreGive(sd_mutex);
        sd_card_health.err_count++;
        log_text(1, "SDCard", "ERROR: f_lseek failed (%s, code: %d).", file_name, res);
        return W_FAILURE;
    }

    /* Write data from buffer to file. */
    res = f_write(&file, buffer, bytes_to_write, (UINT *)bytes_written);
    if (res != FR_OK) {
        f_close(&file);
        xSemaphoreGive(sd_mutex);
        sd_card_health.err_count++;
        log_text(1, "SDCard", "ERROR: f_write failed (%s, code: %d).", file_name, res);
        return W_FAILURE;
    }

    /* Close the file and release the mutex. */
    f_close(&file);
    xSemaphoreGive(sd_mutex);
    sd_card_health.write_count++;
    return W_SUCCESS;
}

w_status_t sd_card_file_create(const char *file_name) {
    // validate args
    if (!sd_card_health.is_init || file_name == NULL) {
        log_text(1, "SDCard", "ERROR: sd_card_file_create invalid parameters.");
        return W_INVALID_PARAM;
    }

    /* Acquire the mutex  */
    if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
        log_text(1, "SDCard", "WARN: Failed to take SD mutex for create (%s).", file_name);
        return W_FAILURE;
    }

    FIL file;
    FRESULT res;

    /* Create a new file. The FA_CREATE_NEW flag causes the function to fail if the file already
     * exists. */
    res = f_open(&file, file_name, FA_WRITE | FA_CREATE_NEW);
    if (res != FR_OK) {
        xSemaphoreGive(sd_mutex);
        sd_card_health.err_count++;
        // Don't log FR_EXIST as an error, it's expected sometimes
        if (res != FR_EXIST) {
            log_text(
                1, "SDCard", "ERROR: f_open failed for create (%s, code: %d).", file_name, res
            );
        }
        return W_FAILURE;
    }

    /* Close the file immediately since we just want to create it. */
    f_close(&file);
    xSemaphoreGive(sd_mutex);
    sd_card_health.file_create_count++;
    return W_SUCCESS;
}

// commenting this out to prevent accidentally deleting files somehow
// w_status_t sd_card_file_delete(char *file_name) {
//     /* Acquire the mutex. */
//     if (xSemaphoreTake(sd_mutex, 0) != pdTRUE) {
//         return W_FAILURE;
//     }

//     FRESULT res;
//     res = f_unlink(file_name);
//     xSemaphoreGive(sd_mutex);

//     if (res != FR_OK) {
//         return W_FAILURE;
//     }
//     return W_SUCCESS;
// }

w_status_t sd_card_is_writable(SD_HandleTypeDef *sd_handle) {
    /*
     * It uses HAL_SD_GetCardState() on the SD handle (&hsd1) to check if the card is in the
     * transfer state (HAL_SD_CARD_TRANSFER). If the card is not ready—due to being busy,
     * ejected, or in an error state—the function returns W_FAILURE; otherwise, it returns
     * W_SUCCESS.
     * NOTE: same hal_delay issue as sd_card_init(). (see comment in sd_card_init for details)
     */
    HAL_SD_CardStateTypeDef resp = HAL_SD_GetCardState(sd_handle);

    // HAL transfer is the correct state to be ready for r/w. also module must be initialized
    if ((resp == HAL_SD_CARD_TRANSFER) && (true == sd_card_health.is_init)) {
        return W_SUCCESS;
    } else {
        log_text(
            1,
            "SDCard",
            "WARN: SD card not writable (State: %d, Init: %d).",
            resp,
            sd_card_health.is_init
        );
        return W_FAILURE;
    }
}
