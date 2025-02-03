#ifndef MOCK_I2C_HAL_H
#define MOCK_I2C_HAL_H

#include "fff.h"
#include <stdint.h>

// mock hal i2c types minimally - only the parts we need
typedef struct __I2C_HandleTypeDef
{
  uint8_t *pBuffPtr;       /*!< Pointer to I2C transfer buffer            */
  uint16_t XferSize;       /*!< I2C transfer size                         */
  uint16_t XferCount;      /*!< I2C transfer counter                      */
  uint32_t XferOptions;    /*!< I2C sequantial transfer options, this parameter can
uint32_t              PreviousState;  /*!< I2C communication Previous state          */
  uint32_t ErrorCode;      /*!< I2C Error code                            */
  uint32_t AddrEventCount; /*!< I2C Address Event counter                 */
  uint32_t Devaddress;     /*!< I2C Target device address                 */
  uint32_t Memaddress;     /*!< I2C Target memory address                 */
  void (*MasterTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*MasterRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*SlaveTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*SlaveRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*ListenCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*MemTxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*MemRxCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*ErrorCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*AbortCpltCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*AddrCallback)(struct __I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
  void (*MspInitCallback)(struct __I2C_HandleTypeDef *hi2c);
  void (*MspDeInitCallback)(struct __I2C_HandleTypeDef *hi2c);
} I2C_HandleTypeDef;

typedef enum
{
  HAL_I2C_MASTER_TX_COMPLETE_CB_ID = 0x00U, /*!< I2C Master Tx Transfer completed callback ID  */
  HAL_I2C_MASTER_RX_COMPLETE_CB_ID = 0x01U, /*!< I2C Master Rx Transfer completed callback ID  */
  HAL_I2C_SLAVE_TX_COMPLETE_CB_ID = 0x02U,  /*!< I2C Slave Tx Transfer completed callback ID   */
  HAL_I2C_SLAVE_RX_COMPLETE_CB_ID = 0x03U,  /*!< I2C Slave Rx Transfer completed callback ID   */
  HAL_I2C_LISTEN_COMPLETE_CB_ID = 0x04U,    /*!< I2C Listen Complete callback ID               */
  HAL_I2C_MEM_TX_COMPLETE_CB_ID = 0x05U,    /*!< I2C Memory Tx Transfer callback ID            */
  HAL_I2C_MEM_RX_COMPLETE_CB_ID = 0x06U,    /*!< I2C Memory Rx Transfer completed callback ID  */
  HAL_I2C_ERROR_CB_ID = 0x07U,              /*!< I2C Error callback ID                         */
  HAL_I2C_ABORT_CB_ID = 0x08U,              /*!< I2C Abort callback ID                         */

  HAL_I2C_MSPINIT_CB_ID = 0x09U,  /*!< I2C Msp Init callback ID                      */
  HAL_I2C_MSPDEINIT_CB_ID = 0x0AU /*!< I2C Msp DeInit callback ID                    */

} HAL_I2C_CallbackIDTypeDef;

typedef void (*pI2C_CallbackTypeDef)(I2C_HandleTypeDef *hi2c);

typedef enum
{
  HAL_OK = 0x00,
  HAL_ERROR = 0x01,
  HAL_BUSY = 0x02,
  HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

#define I2C_MEMADD_SIZE_8BIT 1

#define HAL_I2C_ERROR_NONE (0x00000000U)             /*!< No error              */
#define HAL_I2C_ERROR_BERR (0x00000001U)             /*!< BERR error            */
#define HAL_I2C_ERROR_ARLO (0x00000002U)             /*!< ARLO error            */
#define HAL_I2C_ERROR_AF (0x00000004U)               /*!< ACKF error            */
#define HAL_I2C_ERROR_OVR (0x00000008U)              /*!< OVR error             */
#define HAL_I2C_ERROR_DMA (0x00000010U)              /*!< DMA transfer error    */
#define HAL_I2C_ERROR_TIMEOUT (0x00000020U)          /*!< Timeout error         */
#define HAL_I2C_ERROR_SIZE (0x00000040U)             /*!< Size Management error */
#define HAL_I2C_ERROR_DMA_PARAM (0x00000080U)        /*!< DMA Parameter Error   */
#define HAL_I2C_ERROR_INVALID_CALLBACK (0x00000100U) /*!< Invalid Callback error */
#define HAL_I2C_ERROR_INVALID_PARAM (0x00000200U)    /*!< Invalid Parameters error  */

// Mock functions!!!!!!! -------------------------------

// Fake function for non-blocking memory read (Interrupt-driven)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Read_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                        uint8_t *, uint16_t);

// Fake function for non-blocking memory write (Interrupt-driven)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Mem_Write_IT,
                        I2C_HandleTypeDef *, uint16_t, uint16_t, uint16_t,
                        const uint8_t *, uint16_t);

// Fake function for registering a callback.
// If your driver uses this, even if it’s a no-op in test, you still want to override it.
DECLARE_FAKE_VOID_FUNC(HAL_I2C_RegisterCallback,
                       I2C_HandleTypeDef *, uint32_t, void *);

// Fake function for aborting a master I2C process (Interrupt-driven)
DECLARE_FAKE_VALUE_FUNC(HAL_StatusTypeDef, HAL_I2C_Master_Abort_IT,
                        I2C_HandleTypeDef *, uint16_t);

void hal_i2c_mocks_init(void);
void hal_i2c_mocks_reset(void);

#endif // MOCK_I2C_HAL_H
