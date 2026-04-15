/**
 * @file cmd.cpp
 * @brief UART ASCII command decoder with IRQ-backed RX ring buffer.
 */

#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "uart_driver.h"

extern "C" {
void SystemInit(void);
void _init(void);
}

namespace {

constexpr uint16_t kDebugMagic     = 0xD66DU;
constexpr uint32_t kDebugPeriodMs  = 100U;
constexpr uint32_t kUartTimeoutMs  = 700U;
constexpr uint8_t  kLineBufferSize = 64U;

struct ParseStats
{
    uint32_t rx_bytes{0U};
    uint32_t ok{0U};
    uint32_t fail_parse{0U};
    uint32_t fail_overflow{0U};
    uint8_t  line_len{0U};
};

enum class RxState : uint8_t
{
    kNoRx = 0U,
    kDecodeOk,
    kDecodeFail
};

enum class LedHealth : uint8_t
{
    kRed = 0U,
    kYellow,
    kGreen
};

#pragma pack(push, 1)
struct DebugFrame
{
    uint16_t magic;
    uint32_t ms;
    uint32_t seq;
    float    cmd_vx_m_s;
    float    cmd_yaw_deg_s;
    uint8_t  rx_state;
    uint8_t  line_len;
    uint16_t reserved;
    uint32_t parser_rx_bytes;
    uint32_t parser_ok;
    uint32_t parser_fail_parse;
    uint32_t parser_fail_overflow;
    uint32_t irq_rx_bytes;
    uint32_t ring_overflow;
    uint32_t err_overrun;
    uint32_t err_framing;
    uint32_t err_noise;
    uint32_t err_parity;
    uint16_t crc;
};
#pragma pack(pop)

static_assert(sizeof(DebugFrame) == 64U, "Unexpected DebugFrame size");

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t b = 0U; b < 8U; ++b) {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void leds_init_always_on(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

void set_led_health(LedHealth health)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, (health == LedHealth::kGreen) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, (health == LedHealth::kYellow) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (health == LedHealth::kRed) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool parse_ascii_cmd_line(const char *line, float *vx, float *yaw_rate)
{
    if (line == nullptr || vx == nullptr || yaw_rate == nullptr) {
        return false;
    }
    if (line[0] != 'V' && line[0] != 'v') {
        return false;
    }

    char *end = nullptr;
    const float parsed_vx = std::strtof(line + 1, &end);
    if (end == (line + 1) || *end != ',') {
        return false;
    }
    ++end;
    if (*end != 'Y' && *end != 'y') {
        return false;
    }
    ++end;

    const float parsed_yaw = std::strtof(end, &end);
    if (end == nullptr || *end != '\0') {
        return false;
    }

    *vx = parsed_vx;
    *yaw_rate = parsed_yaw;
    return true;
}

RxState drain_and_parse_rx(ParseStats *stats, float *vx_cmd, float *yaw_cmd)
{
    if (stats == nullptr || vx_cmd == nullptr || yaw_cmd == nullptr) {
        return RxState::kNoRx;
    }

    static char    line_buf[kLineBufferSize]{};
    static uint8_t idx = 0U;

    uint8_t byte = 0U;
    bool got_rx = false;
    bool decode_ok = false;
    bool decode_fail = false;

    while (uart_try_read_byte(&byte)) {
        got_rx = true;
        ++stats->rx_bytes;

        if (byte == '\r') {
            continue;
        }
        if (byte != '\n') {
            if (idx < static_cast<uint8_t>(kLineBufferSize - 1U)) {
                line_buf[idx++] = static_cast<char>(byte);
                stats->line_len = idx;
            } else {
                idx = 0U;
                stats->line_len = 0U;
                ++stats->fail_overflow;
                decode_fail = true;
            }
            continue;
        }

        line_buf[idx] = '\0';
        if (idx > 0U && parse_ascii_cmd_line(line_buf, vx_cmd, yaw_cmd)) {
            ++stats->ok;
            decode_ok = true;
        } else if (idx > 0U) {
            ++stats->fail_parse;
            decode_fail = true;
        }
        idx = 0U;
        stats->line_len = 0U;
    }

    if (decode_ok) {
        return RxState::kDecodeOk;
    }
    if (decode_fail) {
        return RxState::kDecodeFail;
    }
    return got_rx ? RxState::kDecodeFail : RxState::kNoRx;
}

} // namespace

int main(void)
{
    HAL_Init();
    uart_driver_init(&uart_driver_config_stlink_vcp);
    leds_init_always_on();

    ParseStats parse_stats{};
    float      cmd_vx_m_s = 0.0f;
    float      cmd_yaw_deg_s = 0.0f;
    uint32_t   last_debug_ms = HAL_GetTick();
    uint32_t   last_uart_rx_ms = last_debug_ms;
    uint32_t   seq = 0U;
    LedHealth  led_health = LedHealth::kRed;

    set_led_health(led_health);

    while (true) {
        const uint32_t now_ms = HAL_GetTick();
        const RxState rx_state = drain_and_parse_rx(&parse_stats, &cmd_vx_m_s, &cmd_yaw_deg_s);

        if (rx_state == RxState::kDecodeOk) {
            led_health = LedHealth::kGreen;
            last_uart_rx_ms = now_ms;
        } else if (rx_state == RxState::kDecodeFail) {
            led_health = LedHealth::kYellow;
            last_uart_rx_ms = now_ms;
        } else if ((now_ms - last_uart_rx_ms) > kUartTimeoutMs) {
            led_health = LedHealth::kRed;
        }
        set_led_health(led_health);

        if ((now_ms - last_debug_ms) >= kDebugPeriodMs) {
            uart_driver_rx_stats_t rx_stats{};
            uart_driver_get_rx_stats(&rx_stats);

            DebugFrame frame{};
            frame.magic = kDebugMagic;
            frame.ms = now_ms;
            frame.seq = seq++;
            frame.cmd_vx_m_s = cmd_vx_m_s;
            frame.cmd_yaw_deg_s = cmd_yaw_deg_s;
            frame.rx_state = static_cast<uint8_t>(rx_state);
            frame.line_len = parse_stats.line_len;
            frame.reserved = 0U;
            frame.parser_rx_bytes = parse_stats.rx_bytes;
            frame.parser_ok = parse_stats.ok;
            frame.parser_fail_parse = parse_stats.fail_parse;
            frame.parser_fail_overflow = parse_stats.fail_overflow;
            frame.irq_rx_bytes = rx_stats.irq_rx_bytes;
            frame.ring_overflow = rx_stats.ring_overflow;
            frame.err_overrun = rx_stats.err_overrun;
            frame.err_framing = rx_stats.err_framing;
            frame.err_noise = rx_stats.err_noise;
            frame.err_parity = rx_stats.err_parity;
            frame.crc = 0U;
            frame.crc = crc16_ccitt(reinterpret_cast<const uint8_t *>(&frame),
                                    sizeof(frame) - sizeof(frame.crc));

            (void)uart_write_bytes(reinterpret_cast<const uint8_t *>(&frame),
                                   static_cast<uint16_t>(sizeof(frame)),
                                   2U);
            last_debug_ms = now_ms;
        }
    }
}

extern "C" {
void _init(void) {}
}
