#ifndef SD_CARD_H
#define SD_CARD_H

/* C Standard Library */
#include <stdint.h>

/* Project-local headers */
#include "rocketlib/include/common.h"

/**
 * @brief Initialize the SD card hardware and create the mutex for thread safety.
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t sd_card_init(void);

/**
 * @brief Read a file from the SD card.
 *
 * @param fileName - The name of the file to read.
 * @param buffer - The buffer to read the file into.
 * @param bufferSize - The size of the buffer.
 * @param bytesRead - The number of bytes read from the file.
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t file_read(char *fileName, void *buffer, uint32_t bufferSize, uint32_t *bytesRead);

/**
 * @brief Write data to a file on the SD card.
 *
 * Acquires the SD card mutex, opens the file, writes data from buffer,
 * closes the file, then releases the mutex.
 *
 * @param[in]  file_name    Name/path of the file to write to.
 * @param[in]  buffer       Pointer to the data to be written.
 * @param[in]  buffer_size  Number of bytes from buffer to write.
 * @param[out] bytes_written Actual number of bytes written (if successful).
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t file_write(char *fileName, void *buffer, uint32_t bufferSize, uint32_t *bytesWritten);

/**
 * @brief Create a new file on the SD card.
 *
 * Acquires mutex, creates the file (if it does not exist),
 * closes the file, then releases the mutex.
 *
 * @param[in] file_name  Name/path of the file to create.
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t file_create(char *fileName);

/**
 * @brief Delete a file from the SD card.
 *
 * Acquires mutex, deletes the file, then releases the mutex.
 *
 * @param[in] file_name  Name/path of the file to delete.
 *
 * @return w_status_t - W_SUCCESS on success, W_FAILURE on failure.
 */
w_status_t file_delete(char *fileName);

/**
 * @brief Check SD card status or re-initialize as needed.
 *
 * Acquires mutex, performs any status checks or re-mounting,
 * then releases the mutex.
 *
 * @return w_status_t - W_SUCCESS if the SD card is OK, W_FAILURE if not.
 */
w_status_t sd_card_status(void);

#endif // SD_CARD_H