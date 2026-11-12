/*
xử lý việc đọc tín hiệu từ cảm biến Hall A3144 và đếm số lần phát hiện từ trường.
Cụ thể:
- Lưu chân GPIO của cảm biến khi khởi tạo.
- Đọc trạng thái chân OUT — thường là LOW khi A3144 phát hiện từ trường.
- Khi có ngắt ở chân cảm biến, đếm cạnh xuống (HIGH → LOW).
- Bỏ qua các tín hiệu đến quá sát nhau theo thời gian debounce_ms để hạn chế đếm nhiễu.
- Cho phép lấy hoặc đặt lại số lần đếm.
*/ 


#include "a3144.h"

static A3144_HandleTypeDef *a3144_instance = NULL;

// Gán chân GPIO và thời gian chống nhiễu cho cảm biến, đặt bộ đếm và thời điểm gần nhất về 0. 
// Lưu cảm biến để hàm ngắt có thể sử dụng.
void A3144_Init(A3144_HandleTypeDef *sensor,
                GPIO_TypeDef *port,
                uint16_t pin,
                uint32_t debounce_ms)
{
    if (sensor == NULL)
        return;

    sensor->port = port;
    sensor->pin = pin;
    sensor->count = 0U;
    sensor->last_tick = 0U;
    sensor->debounce_ms = debounce_ms;

    a3144_instance = sensor;
}

// Đọc mức logic ở chân cảm biến. 
// A3144 thường kéo chân OUT xuống LOW khi phát hiện từ trường. 
// Nếu truyền con trỏ NULL, hàm trả về
GPIO_PinState A3144_Read(A3144_HandleTypeDef *sensor)
{
    if (sensor == NULL)
        return GPIO_PIN_SET;

    /* A3144 thường kéo OUT xuống LOW khi phát hiện từ trường */
    return HAL_GPIO_ReadPin(sensor->port, sensor->pin);
}

// Trả về số lần phát hiện đã đếm; trả về 0 nếu nhận NULL.
uint32_t A3144_GetCount(A3144_HandleTypeDef *sensor)
{
    if (sensor == NULL)
        return 0U;

    return sensor->count;
}

// Đặt lại bộ đếm về 0; trả về nếu nhận NULL.
void A3144_ResetCount(A3144_HandleTypeDef *sensor)
{
    if (sensor != NULL)
        sensor->count = 0U;
}

// Hàm xử lý gọi từ callback ngắt GPIO. 
// Nếu đúng chân và tín hiệu đang LOW, 
// hàm kiểm tra thời gian chống nhiễu rồi tăng bộ đếm.
void A3144_EXTI_Callback(uint16_t GPIO_Pin)
{
    A3144_HandleTypeDef *sensor = a3144_instance;

    if (sensor == NULL || GPIO_Pin != sensor->pin)
        return;

    /* Đếm cạnh xuống: OUT chuyển HIGH -> LOW */
    if (HAL_GPIO_ReadPin(sensor->port, sensor->pin)
        != GPIO_PIN_RESET)
        return;

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - sensor->last_tick)
        < sensor->debounce_ms)
        return;

    sensor->last_tick = now;
    sensor->count++;
}
