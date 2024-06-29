#include "control.h"
#include "pwm.h"
#include "encoder.h"

/**
 * su dung hang doi queue cho trinh xu ly ngat 
*/
static QueueHandle_t uart2_queue;

void control::recdata(String message)
{
    Serial.print("UART2 Receive from pc:");
    Serial.println(message);
}

/**
 * function ngat 
 * 
*/

static void UART_ISR_ROUTINE(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    /**
     * dieu kien de thoat ra gap khi thuc hien ngat 
    */

    bool exit_condition = false;
    while(1)
    {

        if(xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY))
        {
            //Handle received event

            if (event.type == UART_DATA)
            {
                uint8_t UART2_data[128];
                int UART2_data_length = 0;
                /**
                 * esp_err_t uart_get_buffered_data_len(uart_port_t uart_num, size_t *size)
                */
                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&UART2_data_length));
                /**
                 * int uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait)
                */
                UART2_data_length = uart_read_bytes(UART_NUM_2, UART2_data, UART2_data_length, 100);
                _ctrl.data="";
                
                for(byte i=0; i<UART2_data_length;i++)
                {
                    _ctrl.data+=char(UART2_data[i]);
                }
                _ctrl.recdata(_ctrl.data);

            }
            //Handle frame error event
            else if (event.type == UART_FRAME_ERR) {
                //TODO...
            }
            //Final else statement to act as a default case
            else {
                //TODO...
            }    


        }

        
        if (exit_condition) {
            break;
        }

        
    }
    //Out side of loop now. Task needs to clean up and self terminate before returning
    vTaskDelete(NULL);
}

void control::init_driver_uart()
{
    uart_config_t Configurazione_UART2 = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    /***
     * esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t *uart_config)
    */
    uart_param_config(NUMERO_PORTA_SERIALE, &Configurazione_UART2);
    /**
     * esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num)
    */
    uart_set_pin(NUMERO_PORTA_SERIALE, U2TXD, U2RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    /**
     * esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size,
     *  int queue_size, QueueHandle_t *uart_queue, int intr_alloc_flags)
     * 
    */
    uart_driver_install(NUMERO_PORTA_SERIALE, BUF_SIZE, BUF_SIZE, 20, &uart2_queue, 0);
    //Create a task to handler UART event from ISR
    /**
     * extern "C" static inline BaseType_t xTaskCreate(TaskFunction_t pvTaskCode, 
     * const char *pcName, uint32_t usStackDepth, void *pvParameters, UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask)
    */
    xTaskCreate(UART_ISR_ROUTINE, "UART_ISR_ROUTINE", 2048, NULL, 12, NULL);
}
void control::processor(String data)
{
    int speed=0; // toc do pwm
    int state=0; // trang thai banh quay toi hay quay lui
    int target=0;
    const char* cdata = data.c_str();
    for(int i=0;i<strlen(cdata);i++)
    {
        if(cdata[i] == 'p')  
        {
            //Serial.println("da tim duoc p");
            sscanf(cdata,"p%d",&target);
            
            //_pwm.PIDCalculate(encoder2Count,count_pulse,pinDirOne,pinPwmOne,DI_TOI);
            
            break;
        }
        if(cdata[i] == 'r')
        {
            esp_restart();
            break;
        }
        /*
        if(cdata[i] == 'h')
        {
            Serial.println("da tim duoc h");
            sscanf(cdata,"h%d",&speed);
            Serial.printf("speed:= %d\n",speed);
            _pwm.ledPwm(pwmPinB,_pwm.speedB,speed);
        }
        */
        
    }
}

control _ctrl;