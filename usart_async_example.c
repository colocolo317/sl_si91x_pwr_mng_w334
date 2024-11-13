/***************************************************************************/ /**
 * @file usart_async_example.c
 * @brief USART Asynchronous examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2023 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_si91x_usart.h"
#include "rsi_debug.h"
#include "usart_async_example.h"

/*******************************************************************************
 ***************************  Defines / Macros  ********************************
 ******************************************************************************/
#define USART_BUFFER_SIZE 4   // Data send and receive length
#define USART_BAUDRATE    115200 // Baud rate <9600-7372800>
#define NON_UC_DEFAULT_CONFIG \
  0 //  Enable this macro to set the default configurations in non_uc case, this is useful when someone don't want to use UC configuration

/*******************************************************************************
 *************************** LOCAL VARIABLES   *******************************
 ******************************************************************************/
static uint8_t usart_data_in[USART_BUFFER_SIZE];
//static uint8_t usart_data_out[USART_BUFFER_SIZE];

volatile boolean_t usart_send_complete = false, usart_transfer_complete = false, usart_receive_complete = false;
//static boolean_t usart_begin_transmission = true;

/*******************************************************************************
 **********************  Local Function prototypes   ***************************
 ******************************************************************************/
void usart_callback_event(uint32_t event);

/*******************************************************************************
 **************************   GLOBAL VARIABLES   *******************************
 ******************************************************************************/
sl_usart_handle_t usart_handle;
usart_mode_enum_t current_mode = SL_USART_SEND_DATA;

void usart_async_example_deinit(void)
{
  sl_si91x_usart_multiple_instance_unregister_event_callback(USART_0);
  sl_status_t status = sl_si91x_usart_deinit(usart_handle);
  DEBUGOUT("usart deinit: 0x%lX\r\n", status);
}
/*******************************************************************************
 * USART Example Initialization function
 ******************************************************************************/
void usart_async_example_init(void)
{
  sl_status_t status;
  sl_si91x_usart_control_config_t usart_config;

//#if NON_UC_DEFAULT_CONFIG
  usart_config.baudrate      = USART_BAUDRATE;
  usart_config.mode          = SL_USART_MODE_ASYNCHRONOUS;
  usart_config.parity        = SL_USART_NO_PARITY;
  usart_config.stopbits      = SL_USART_STOP_BITS_1;
  usart_config.hwflowcontrol = SL_USART_FLOW_CONTROL_NONE;
  usart_config.databits      = SL_USART_DATA_BITS_8;
  usart_config.misc_control  = SL_USART_MISC_CONTROL_NONE;
  usart_config.usart_module  = USART_0;
  usart_config.config_enable = ENABLE;
  usart_config.synch_mode    = DISABLE;
//#endif
  sl_si91x_usart_control_config_t get_config;

  usart_send_complete = false; usart_transfer_complete = false; usart_receive_complete = false;

  do {
    // Initialize the UART
    status = sl_si91x_usart_init(USART_0, &usart_handle);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_usart_initialize: Error Code : %lu \n", status);
      break;
    }
    DEBUGOUT("USART initialization is successful \n");
    // Configure the USART configurations
    status = sl_si91x_usart_set_configuration(usart_handle, &usart_config);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_usart_set_configuration: Error Code : %lu \n", status);
      break;
    }
    DEBUGOUT("USART configuration is successful \n");
    // Register user callback function
    status = sl_si91x_usart_multiple_instance_register_event_callback(USART_0, usart_callback_event);
    if (status != SL_STATUS_OK) {
      DEBUGOUT("sl_si91x_usart_register_event_callback: Error Code : %lu \n", status);
      break;
    }
    DEBUGOUT("USART user event callback registered successfully \n");
    sl_si91x_usart_get_configurations(USART_0, &get_config);
    DEBUGOUT("Baud Rate = %ld \n", get_config.baudrate);
  } while (false);
}


void amk_usart_async_send(void)
{
  sl_status_t status = sl_si91x_usart_send_data(usart_handle, "abcd\r\n", sizeof("abcd\r\n"));
  if (status != SL_STATUS_OK)
  {
    // If it fails to execute the API, it will not execute rest of the things
    DEBUGOUT("sl_si91x_usart_send_data: Error Code : %lu \n", status);
  }
  while(usart_send_complete == false){}
  usart_send_complete = false;
  DEBUGOUT("usart send done\r\n");
}

void amk_usart_async_receive(void)
{
  //sl_usart_status_t usart_status = sl_si91x_usart_get_status(usart_handle);
  memset(usart_data_in, 0, USART_BUFFER_SIZE);
  sl_status_t status = sl_si91x_usart_receive_data(usart_handle, usart_data_in, USART_BUFFER_SIZE);
  if (status != SL_STATUS_OK)
  {
    // If it fails to execute the API, it will not execute rest of the things
    DEBUGOUT("sl_si91x_usart_receive_data: Error Code : %lu \n", status);
  }
  while(usart_receive_complete == false){}
  usart_receive_complete = false;
  DEBUGOUT("receive data: %s\r\n", (char*) usart_data_in);
  amk_usart_async_send();
}
/*******************************************************************************
 * Example ticking function
 ******************************************************************************/
void usart_async_example_process_action(void)
{
  amk_usart_async_receive();
  usart_async_example_deinit();
  usart_async_example_init();
}


/*******************************************************************************
 * Callback function triggered on data Transfer and reception
 ******************************************************************************/
void usart_callback_event(uint32_t event)
{
  switch (event) {
    case SL_USART_EVENT_SEND_COMPLETE:
      usart_send_complete = true;
      break;
    case SL_USART_EVENT_RECEIVE_COMPLETE:
      usart_receive_complete = true;
      break;
    case SL_USART_EVENT_TRANSFER_COMPLETE:
      usart_transfer_complete = true;
      break;
  }
}
