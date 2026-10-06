#include "task_eeprom.h"
#include "task_console.h"
#include "FreeRTOS_CLI.h"
#include <string.h>
#include <stdlib.h>

#define CMD_WREN   0x06u
#define CMD_RDSR   0x05u
#define CMD_READ   0x03u
#define CMD_WRITE  0x02u
#define CMD_RDID_STD 0x9Fu
#define CMD_RDID_ALT 0x83u
#define SR_WIP     0x01u

static inline void cs_assert(void)
{
#if !EEPROM_USE_HW_CS
    cyhal_gpio_write(EEPROM_CS_PIN, EEPROM_CS_ACTIVE_LOW ? 0 : 1);
#endif
    /* Guard time after CS# falling edge before first SCK edge */
    Cy_SysLib_DelayUs(2);
}

static inline void cs_deassert(void)
{
    while (cyhal_spi_is_busy(&mSPI)) {}
    Cy_SysLib_DelayUs(2);
#if !EEPROM_USE_HW_CS
    cyhal_gpio_write(EEPROM_CS_PIN, EEPROM_CS_ACTIVE_LOW ? 1 : 0);
#endif
    /* Inter-command deselect time */
    Cy_SysLib_DelayUs(2);
}

static uint8_t read_status(void)
{
    uint8_t tx[2] = {CMD_RDSR, 0xFF};
    uint8_t rx[2] = {0x00, 0x00};
    cs_assert();
    cyhal_spi_transfer(&mSPI, tx, 2, rx, 2, 0xFF);
    cs_deassert();
    return rx[1];
}

static cy_rslt_t wait_ready(void)
{
    for (uint32_t i = 0; i < 200; i++)
    {
        if (!(read_status() & SR_WIP))
            return CY_RSLT_SUCCESS;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return (cy_rslt_t)0xFFFF0002u; /* timeout */
}

/* Some EEPROMs may not reflect WIP immediately after WRITE CS rising edge.
 * Add a short guard delay before polling WIP to avoid false-ready reads.
 */
static cy_rslt_t wait_ready_after_write(void)
{
    vTaskDelay(pdMS_TO_TICKS(1));
    return wait_ready();
}

static uint8_t eeprom_read_raw1(uint16_t addr, uint8_t *rr, uint8_t rr_len)
{
    uint8_t rt[5] = {0};
    uint8_t rx_index = 0u;

    rt[0] = CMD_READ;
#if (EEPROM_ADDR_BYTES == 1u)
    rt[1] = (uint8_t)(addr & 0xFF);
    rt[2] = 0xFF;
    rx_index = 2u;
    if (rr && rr_len >= 3u)
    {
        cs_assert();
        cyhal_spi_transfer(&mSPI, rt, 3, rr, 3, 0xFF);
        cs_deassert();
    }
#elif (EEPROM_ADDR_BYTES == 2u)
    rt[1] = (uint8_t)(addr >> 8);
    rt[2] = (uint8_t)(addr & 0xFF);
    rt[3] = 0xFF;
    rx_index = 3u;
    if (rr && rr_len >= 4u)
    {
        cs_assert();
        cyhal_spi_transfer(&mSPI, rt, 4, rr, 4, 0xFF);
        cs_deassert();
    }
#else
    rt[1] = 0x00;
    rt[2] = (uint8_t)(addr >> 8);
    rt[3] = (uint8_t)(addr & 0xFF);
    rt[4] = 0xFF;
    rx_index = 4u;
    if (rr && rr_len >= 5u)
    {
        cs_assert();
        cyhal_spi_transfer(&mSPI, rt, 5, rr, 5, 0xFF);
        cs_deassert();
    }
#endif
    return rr ? rr[rx_index] : 0x00u;
}

static void write_enable(void)
{
    uint8_t tx[1] = {CMD_WREN};
    uint8_t rx[1];
    cs_assert();
    cyhal_spi_transfer(&mSPI, tx, 1, rx, 1, 0xFF);
    cs_deassert();
}

/* ── Public API ──────────────────────────────────────────────────────────── */

cy_rslt_t eeprom_init(void)
{
    cy_rslt_t rslt;
#if !EEPROM_USE_HW_CS
    rslt = cyhal_gpio_init(EEPROM_CS_PIN,
                           CYHAL_GPIO_DIR_OUTPUT,
                           CYHAL_GPIO_DRIVE_STRONG,
                           (EEPROM_CS_ACTIVE_LOW ? 1 : 0));
    if (rslt != CY_RSLT_SUCCESS) return rslt;
#endif

    /* Drive HOLD# high so the EEPROM is never paused */
    rslt = cyhal_gpio_init(EEPROM_HOLD_PIN,
                           CYHAL_GPIO_DIR_OUTPUT,
                           CYHAL_GPIO_DRIVE_STRONG,
                           1);
    if (rslt != CY_RSLT_SUCCESS) return rslt;

    /* Drive WP# high so array writes are never blocked by protection pin state */
    rslt = cyhal_gpio_init(EEPROM_WP_PIN,
                           CYHAL_GPIO_DIR_OUTPUT,
                           CYHAL_GPIO_DRIVE_STRONG,
                           1);
    return rslt;
}

cy_rslt_t eeprom_read(uint16_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t total = (uint16_t)(1u + EEPROM_ADDR_BYTES + len);
    uint8_t *tx = pvPortMalloc(total);
    uint8_t *rx = pvPortMalloc(total);
    if (tx == NULL || rx == NULL) { vPortFree(tx); vPortFree(rx); return (cy_rslt_t)0xFFFF0001u; }

    xSemaphoreTake(Semaphore_SPI, portMAX_DELAY);
    if (wait_ready() != CY_RSLT_SUCCESS) { vPortFree(tx); vPortFree(rx); xSemaphoreGive(Semaphore_SPI); return (cy_rslt_t)0xFFFF0003u; }

    tx[0] = CMD_READ;
#if (EEPROM_ADDR_BYTES == 1u)
    tx[1] = (uint8_t)(addr & 0xFF);
#elif (EEPROM_ADDR_BYTES == 2u)
    tx[1] = (uint8_t)(addr >> 8);
    tx[2] = (uint8_t)(addr & 0xFF);
#elif (EEPROM_ADDR_BYTES == 3u)
    tx[1] = 0x00;
    tx[2] = (uint8_t)(addr >> 8);
    tx[3] = (uint8_t)(addr & 0xFF);
#else
#error "EEPROM_ADDR_BYTES must be 1, 2 or 3"
#endif
    memset(&tx[1u + EEPROM_ADDR_BYTES], 0xFF, len);

    cs_assert();
    cyhal_spi_transfer(&mSPI, tx, total, rx, total, 0xFF);
    cs_deassert();

    memcpy(buf, &rx[1u + EEPROM_ADDR_BYTES], len);
    vPortFree(tx);
    vPortFree(rx);
    xSemaphoreGive(Semaphore_SPI);
    return CY_RSLT_SUCCESS;
}

cy_rslt_t eeprom_write(uint16_t addr, const uint8_t *data, uint16_t len)
{
    xSemaphoreTake(Semaphore_SPI, portMAX_DELAY);

    for (uint16_t i = 0; i < len; i++)
    {
        if (wait_ready() != CY_RSLT_SUCCESS) { xSemaphoreGive(Semaphore_SPI); return (cy_rslt_t)0xFFFF0004u; }
        write_enable();

        uint16_t a = addr + i;
        uint8_t tx[5] = {0};
        uint8_t rx[5] = {0};
        tx[0] = CMD_WRITE;
#if (EEPROM_ADDR_BYTES == 1u)
        tx[1] = (uint8_t)(a & 0xFF);
        tx[2] = data[i];
        uint16_t nbytes = 3u;
#elif (EEPROM_ADDR_BYTES == 2u)
        tx[1] = (uint8_t)(a >> 8);
        tx[2] = (uint8_t)(a & 0xFF);
        tx[3] = data[i];
        uint16_t nbytes = 4u;
#else
        tx[1] = 0x00;
        tx[2] = (uint8_t)(a >> 8);
        tx[3] = (uint8_t)(a & 0xFF);
        tx[4] = data[i];
        uint16_t nbytes = 5u;
#endif
        cs_assert();
        cyhal_spi_transfer(&mSPI, tx, nbytes, rx, nbytes, 0xFF);
        cs_deassert();

        if (wait_ready_after_write() != CY_RSLT_SUCCESS) { xSemaphoreGive(Semaphore_SPI); return (cy_rslt_t)0xFFFF0005u; }
    }

    xSemaphoreGive(Semaphore_SPI);
    return CY_RSLT_SUCCESS;
}

/* ── CLI commands ────────────────────────────────────────────────────────── */

/* er <hex_addr> <len> : 读取指定地址的若干字节并打印 */
static BaseType_t cli_er(char *buf, size_t buf_len, const char *cmd)
{
    BaseType_t plen;
    char tmp[16];

    const char *p = FreeRTOS_CLIGetParameter(cmd, 1, &plen);
    memset(tmp, 0, sizeof(tmp));
    if (p) memcpy(tmp, p, plen < (BaseType_t)sizeof(tmp)-1 ? plen : (BaseType_t)sizeof(tmp)-1);
    uint16_t addr = (uint16_t)strtol(tmp, NULL, 16);

    p = FreeRTOS_CLIGetParameter(cmd, 2, &plen);
    memset(tmp, 0, sizeof(tmp));
    if (p) memcpy(tmp, p, plen < (BaseType_t)sizeof(tmp)-1 ? plen : (BaseType_t)sizeof(tmp)-1);
    uint16_t len = (uint16_t)atoi(tmp);

    if (len == 0 || len > 32)
    {
        snprintf(buf, buf_len, "Usage: er <hex_addr> <len>  (len: 1-32)\r\n");
        return pdFALSE;
    }

    uint8_t data[32];
    cy_rslt_t rslt = eeprom_read(addr, data, len);
    if (rslt != CY_RSLT_SUCCESS)
    {
        snprintf(buf, buf_len, "Read failed: 0x%08lX\r\n", (unsigned long)rslt);
        return pdFALSE;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t v = data[i];
        task_print("0x%04X: 0x%02X = %u%u%u%u_%u%u%u%u\r\n",
                   addr + i, v,
                   (v >> 7) & 1, (v >> 6) & 1, (v >> 5) & 1, (v >> 4) & 1,
                   (v >> 3) & 1, (v >> 2) & 1, (v >> 1) & 1, v & 1);
    }
    buf[0] = '\0';
    return pdFALSE;
}
static const CLI_Command_Definition_t cmd_er = {
    "er", "\r\ner <hex_addr> <len>: Read bytes from EEPROM (len: 1-32)\r\n", cli_er, 2
};

/* ew <hex_addr> <b0> [b1 ...] : 写入最多16个字节 */
static BaseType_t cli_ew(char *buf, size_t buf_len, const char *cmd)
{
    BaseType_t plen;
    char tmp[16];

    const char *p = FreeRTOS_CLIGetParameter(cmd, 1, &plen);
    memset(tmp, 0, sizeof(tmp));
    if (p) memcpy(tmp, p, plen < (BaseType_t)sizeof(tmp)-1 ? plen : (BaseType_t)sizeof(tmp)-1);
    uint16_t addr = (uint16_t)strtol(tmp, NULL, 16);

    uint8_t data[16];
    uint8_t count = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        p = FreeRTOS_CLIGetParameter(cmd, (UBaseType_t)(2 + i), &plen);
        if (p == NULL) break;
        memset(tmp, 0, sizeof(tmp));
        memcpy(tmp, p, plen < (BaseType_t)sizeof(tmp)-1 ? plen : (BaseType_t)sizeof(tmp)-1);
        data[count++] = (uint8_t)strtol(tmp, NULL, 16);
    }

    if (count == 0)
    {
        snprintf(buf, buf_len, "Usage: ew <hex_addr> <b0> [b1 ...]\r\n");
        return pdFALSE;
    }

    cy_rslt_t rslt = eeprom_write(addr, data, count);
    if (rslt == CY_RSLT_SUCCESS)
        snprintf(buf, buf_len, "Wrote %u byte(s) to 0x%04X\r\n", count, addr);
    else
        snprintf(buf, buf_len, "Write failed: 0x%08lX\r\n", (unsigned long)rslt);
    return pdFALSE;
}
static const CLI_Command_Definition_t cmd_ew = {
    "ew", "\r\new <hex_addr> <b0> [b1 ...]: Write hex bytes to EEPROM\r\n", cli_ew, -1
};

/* eauto [rounds]: 自动化写读稳定性测试 */
static BaseType_t cli_eauto(char *buf, size_t buf_len, const char *cmd)
{
    BaseType_t plen;
    char tmp[16];
    uint32_t rounds = 64u;
    uint32_t total = 0u;
    uint32_t fail = 0u;
    uint8_t rd = 0u;
    cy_rslt_t rslt;
    uint8_t rr_dbg[5] = {0};

    const char *p = FreeRTOS_CLIGetParameter(cmd, 1, &plen);
    if (p != NULL)
    {
        memset(tmp, 0, sizeof(tmp));
        memcpy(tmp, p, plen < (BaseType_t)sizeof(tmp) - 1 ? plen : (BaseType_t)sizeof(tmp) - 1);
        rounds = (uint32_t)atoi(tmp);
        if (rounds == 0u) rounds = 64u;
        if (rounds > 2000u) rounds = 2000u;
    }

    buf[0] = '\0';
    task_print("[eauto] cfg: mode=%s, addr_bytes=%u, rounds=%lu\r\n",
               (SPI_MODE_EEPROM == CYHAL_SPI_MODE_00_MSB) ? "MODE0" : "MODE3",
               (unsigned)EEPROM_ADDR_BYTES,
               (unsigned long)rounds);
    task_print("[eauto] cs=%s, active=%s\r\n",
               EEPROM_USE_HW_CS ? "HW" : "GPIO",
               EEPROM_CS_ACTIVE_LOW ? "LOW" : "HIGH");

    /* Phase A: same-address toggle stress */
    task_print("[eauto] phase A: same-address toggle @0x0200\r\n");
    for (uint32_t i = 0; i < rounds; i++)
    {
        uint8_t wr = (i & 1u) ? 0x55u : 0xAAu;
        rslt = eeprom_write(0x0200u, &wr, 1u);
        total++;
        if (rslt != CY_RSLT_SUCCESS) { fail++; continue; }

        rslt = eeprom_read(0x0200u, &rd, 1u);
        total++;
        if (rslt != CY_RSLT_SUCCESS || rd != wr)
        {
            fail++;
            if (fail <= 12u)
            {
                memset(rr_dbg, 0, sizeof(rr_dbg));
                uint8_t rd_raw = eeprom_read_raw1(0x0200u, rr_dbg, sizeof(rr_dbg));
#if (EEPROM_ADDR_BYTES == 1u)
                task_print("[A.%lu] wr=%02X rd=%02X raw=%02X rr=[%02X %02X %02X] rslt=0x%08lX SR=%02X\r\n",
                           (unsigned long)i, wr, rd, rd_raw,
                           rr_dbg[0], rr_dbg[1], rr_dbg[2],
                           (unsigned long)rslt, read_status());
#elif (EEPROM_ADDR_BYTES == 2u)
                task_print("[A.%lu] wr=%02X rd=%02X raw=%02X rr=[%02X %02X %02X %02X] rslt=0x%08lX SR=%02X\r\n",
                           (unsigned long)i, wr, rd, rd_raw,
                           rr_dbg[0], rr_dbg[1], rr_dbg[2], rr_dbg[3],
                           (unsigned long)rslt, read_status());
#else
                task_print("[A.%lu] wr=%02X rd=%02X raw=%02X rr=[%02X %02X %02X %02X %02X] rslt=0x%08lX SR=%02X\r\n",
                           (unsigned long)i, wr, rd, rd_raw,
                           rr_dbg[0], rr_dbg[1], rr_dbg[2], rr_dbg[3], rr_dbg[4],
                           (unsigned long)rslt, read_status());
#endif
            }
        }
    }

    /* Phase B: sequential pattern */
    task_print("[eauto] phase B: sequential block @0x0300 len=64\r\n");
    for (uint16_t i = 0; i < 64u; i++)
    {
        uint16_t addr = (uint16_t)(0x0300u + i);
        uint8_t wr = (uint8_t)((i * 37u) ^ 0xA5u);
        rslt = eeprom_write(addr, &wr, 1u);
        total++;
        if (rslt != CY_RSLT_SUCCESS) { fail++; continue; }

        rslt = eeprom_read(addr, &rd, 1u);
        total++;
        if (rslt != CY_RSLT_SUCCESS || rd != wr)
        {
            fail++;
            if (fail <= 12u)
            {
                memset(rr_dbg, 0, sizeof(rr_dbg));
                uint8_t rd_raw = eeprom_read_raw1(addr, rr_dbg, sizeof(rr_dbg));
#if (EEPROM_ADDR_BYTES == 1u)
                task_print("[B.%u] addr=0x%04X wr=%02X rd=%02X raw=%02X rr=[%02X %02X %02X] rslt=0x%08lX SR=%02X\r\n",
                           i, addr, wr, rd, rd_raw,
                           rr_dbg[0], rr_dbg[1], rr_dbg[2],
                           (unsigned long)rslt, read_status());
#elif (EEPROM_ADDR_BYTES == 2u)
                task_print("[B.%u] addr=0x%04X wr=%02X rd=%02X raw=%02X rr=[%02X %02X %02X %02X] rslt=0x%08lX SR=%02X\r\n",
                           i, addr, wr, rd, rd_raw,
                           rr_dbg[0], rr_dbg[1], rr_dbg[2], rr_dbg[3],
                           (unsigned long)rslt, read_status());
#else
                task_print("[B.%u] addr=0x%04X wr=%02X rd=%02X raw=%02X rr=[%02X %02X %02X %02X %02X] rslt=0x%08lX SR=%02X\r\n",
                           i, addr, wr, rd, rd_raw,
                           rr_dbg[0], rr_dbg[1], rr_dbg[2], rr_dbg[3], rr_dbg[4],
                           (unsigned long)rslt, read_status());
#endif
            }
        }
    }

    uint32_t pass = total - fail;
    uint32_t pass_pct = (total == 0u) ? 0u : (pass * 100u) / total;
    task_print("[eauto] done: total=%lu fail=%lu pass=%lu (%lu%%)\r\n",
               (unsigned long)total,
               (unsigned long)fail,
               (unsigned long)pass,
               (unsigned long)pass_pct);

    snprintf(buf, buf_len, "eauto finished\r\n");
    return pdFALSE;
}
static const CLI_Command_Definition_t cmd_eauto = {
    "eauto", "\r\neauto [rounds]: Auto EEPROM stress test\r\n", cli_eauto, -1
};

/* ebit [rounds]: bit-level fault localization (walking 1/0) */
static BaseType_t cli_ebit(char *buf, size_t buf_len, const char *cmd)
{
    BaseType_t plen;
    char tmp[16];
    uint32_t rounds = 16u;
    uint32_t bit_err_set[8] = {0};
    uint32_t bit_err_clr[8] = {0};
    uint32_t total_cases = 0u;
    uint32_t fail_cases = 0u;

    const char *p = FreeRTOS_CLIGetParameter(cmd, 1, &plen);
    if (p != NULL)
    {
        memset(tmp, 0, sizeof(tmp));
        memcpy(tmp, p, plen < (BaseType_t)sizeof(tmp) - 1 ? plen : (BaseType_t)sizeof(tmp) - 1);
        rounds = (uint32_t)atoi(tmp);
        if (rounds == 0u) rounds = 16u;
        if (rounds > 512u) rounds = 512u;
    }

    buf[0] = '\0';
    task_print("[ebit] cfg: mode=%s, addr_bytes=%u, rounds=%lu\r\n",
               (SPI_MODE_EEPROM == CYHAL_SPI_MODE_00_MSB) ? "MODE0" : "MODE3",
               (unsigned)EEPROM_ADDR_BYTES,
               (unsigned long)rounds);

    for (uint32_t r = 0; r < rounds; r++)
    {
        uint16_t addr = (uint16_t)(0x0400u + (r & 0x1Fu));

        for (uint8_t b = 0; b < 8u; b++)
        {
            uint8_t wr1 = (uint8_t)(1u << b);
            uint8_t rd1 = 0u;
            uint8_t wr0 = (uint8_t)(~wr1);
            uint8_t rd0 = 0u;

            cy_rslt_t rslt = eeprom_write(addr, &wr1, 1u);
            if (rslt == CY_RSLT_SUCCESS) rslt = eeprom_read(addr, &rd1, 1u);
            total_cases++;
            if (rslt != CY_RSLT_SUCCESS || rd1 != wr1)
            {
                fail_cases++;
                uint8_t exp1_1 = (uint8_t)(1u << b);
                uint8_t miss_set = (uint8_t)(exp1_1 & (uint8_t)(~rd1));
                uint8_t extra_set = (uint8_t)((uint8_t)(~exp1_1) & rd1);
                for (uint8_t i = 0; i < 8u; i++)
                {
                    if (miss_set & (1u << i)) bit_err_set[i]++;
                    if (extra_set & (1u << i)) bit_err_clr[i]++;
                }
            }

            rslt = eeprom_write(addr, &wr0, 1u);
            if (rslt == CY_RSLT_SUCCESS) rslt = eeprom_read(addr, &rd0, 1u);
            total_cases++;
            if (rslt != CY_RSLT_SUCCESS || rd0 != wr0)
            {
                fail_cases++;
                uint8_t exp0_1 = wr0;
                uint8_t miss_set = (uint8_t)(exp0_1 & (uint8_t)(~rd0));
                uint8_t extra_set = (uint8_t)((uint8_t)(~exp0_1) & rd0);
                for (uint8_t i = 0; i < 8u; i++)
                {
                    if (miss_set & (1u << i)) bit_err_set[i]++;
                    if (extra_set & (1u << i)) bit_err_clr[i]++;
                }
            }
        }
    }

    task_print("[ebit] result: total=%lu fail=%lu pass=%lu\r\n",
               (unsigned long)total_cases,
               (unsigned long)fail_cases,
               (unsigned long)(total_cases - fail_cases));
    task_print("[ebit] bit miss-set count  (expected 1, got 0): b7..b0 = %lu %lu %lu %lu %lu %lu %lu %lu\r\n",
               (unsigned long)bit_err_set[7], (unsigned long)bit_err_set[6],
               (unsigned long)bit_err_set[5], (unsigned long)bit_err_set[4],
               (unsigned long)bit_err_set[3], (unsigned long)bit_err_set[2],
               (unsigned long)bit_err_set[1], (unsigned long)bit_err_set[0]);
    task_print("[ebit] bit extra-set count (expected 0, got 1): b7..b0 = %lu %lu %lu %lu %lu %lu %lu %lu\r\n",
               (unsigned long)bit_err_clr[7], (unsigned long)bit_err_clr[6],
               (unsigned long)bit_err_clr[5], (unsigned long)bit_err_clr[4],
               (unsigned long)bit_err_clr[3], (unsigned long)bit_err_clr[2],
               (unsigned long)bit_err_clr[1], (unsigned long)bit_err_clr[0]);

    snprintf(buf, buf_len, "ebit finished\r\n");
    return pdFALSE;
}
static const CLI_Command_Definition_t cmd_ebit = {
    "ebit", "\r\nebit [rounds]: Bit-level EEPROM fault localization\r\n", cli_ebit, -1
};

/* eid: read JEDEC/device identification bytes */
static BaseType_t cli_eid(char *buf, size_t buf_len, const char *cmd)
{
    (void)cmd;
    uint8_t tx_std[4] = {CMD_RDID_STD, 0xFF, 0xFF, 0xFF};
    uint8_t rx_std[4] = {0};
    uint8_t tx_alt[4] = {CMD_RDID_ALT, 0xFF, 0xFF, 0xFF};
    uint8_t rx_alt[4] = {0};
    uint8_t sr = 0;

    xSemaphoreTake(Semaphore_SPI, portMAX_DELAY);
    sr = read_status();

    cs_assert();
    cyhal_spi_transfer(&mSPI, tx_std, 4, rx_std, 4, 0xFF);
    cs_deassert();

    cs_assert();
    cyhal_spi_transfer(&mSPI, tx_alt, 4, rx_alt, 4, 0xFF);
    cs_deassert();

    xSemaphoreGive(Semaphore_SPI);

    snprintf(buf, buf_len,
             "eid: mode=%s addr_bytes=%u cs=%s SR=0x%02X\r\n"
             "  RDID(0x9F): %02X %02X %02X\r\n"
             "  RDID(0x83): %02X %02X %02X\r\n",
             (SPI_MODE_EEPROM == CYHAL_SPI_MODE_00_MSB) ? "MODE0" : "MODE3",
             (unsigned)EEPROM_ADDR_BYTES,
             EEPROM_USE_HW_CS ? "HW" : "GPIO",
             sr,
             rx_std[1], rx_std[2], rx_std[3],
             rx_alt[1], rx_alt[2], rx_alt[3]);
    return pdFALSE;
}
static const CLI_Command_Definition_t cmd_eid = {
    "eid", "\r\neid: Read EEPROM identification bytes\r\n", cli_eid, 0
};

/* ediag: 每步立即通过 task_print 发出，不等函数返回 */
static BaseType_t cli_ediag(char *buf, size_t buf_len, const char *cmd)
{
    (void)cmd; (void)buf_len;
    buf[0] = '\0';  /* CLI 框架要求 buf 有内容，返回空字符串即可 */
    task_print("[cfg] SPI mode=%s, addr_bytes=%u\r\n",
               (SPI_MODE_EEPROM == CYHAL_SPI_MODE_00_MSB) ? "MODE0" : "MODE3",
               (unsigned)EEPROM_ADDR_BYTES);

    /* Step 1 */
    uint8_t tx1[1]={0x55}, rx1[1]={0};
    cs_assert();
    cy_rslt_t rslt = cyhal_spi_transfer(&mSPI, tx1, 1, rx1, 1, 0xFF);
    cs_deassert();
    task_print("[1] SPI transfer: %s\r\n", rslt==CY_RSLT_SUCCESS ? "PASS" : "FAIL");
    if (rslt != CY_RSLT_SUCCESS) return pdFALSE;

    /* Step 2 */
    xSemaphoreTake(Semaphore_SPI, portMAX_DELAY);
    uint8_t sr = read_status();
    xSemaphoreGive(Semaphore_SPI);
    task_print("[2] Status: 0x%02X  WIP=%u WEL=%u%s\r\n",
        sr, sr&1u, (sr>>1)&1u, sr==0xFF ? "  FAIL-MISO悬空" : "");
    if (sr == 0xFF) return pdFALSE;

    /* Step 3 */
    xSemaphoreTake(Semaphore_SPI, portMAX_DELAY);
    write_enable();
    uint8_t sr2 = read_status();
    xSemaphoreGive(Semaphore_SPI);
    task_print("[3] After WREN: 0x%02X  WEL=%u  %s\r\n",
        sr2, (sr2>>1)&1u, (sr2&0x02u) ? "PASS" : "FAIL-W#未上拉");
    if (!(sr2 & 0x02u)) return pdFALSE;

    xSemaphoreTake(Semaphore_SPI, portMAX_DELAY);

    /* Step 4: 逐字节写入 0x0200 处的4个字节，每次只发4字节事务 */
    /* 写 AB 到 0x0200 */
    task_print("[4] single-byte write test to 0x0200...\r\n");
    static const uint8_t test_data[4] = {0xAB, 0xCD, 0xEF, 0x91};
    uint8_t ok_count = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        /* WREN */
        { uint8_t t[1]={CMD_WREN},r[1]={0}; cs_assert(); cyhal_spi_transfer(&mSPI,t,1,r,1,0xFF); cs_deassert(); }
        /* WRITE 1 byte */
        uint16_t a = 0x0200u + i;
        uint8_t wt[5]={0}, wr[5]={0};
        uint8_t rt[5]={0}, rr[5]={0};
        uint8_t rx_index = 0;
        wt[0] = CMD_WRITE;
        rt[0] = CMD_READ;
#if (EEPROM_ADDR_BYTES == 1u)
        wt[1]=(uint8_t)(a&0xFF); wt[2]=test_data[i];
        rt[1]=(uint8_t)(a&0xFF); rt[2]=0xFF;
        cs_assert(); cyhal_spi_transfer(&mSPI,wt,3,wr,3,0xFF); cs_deassert();
        rx_index = 2u;
#elif (EEPROM_ADDR_BYTES == 2u)
        wt[1]=(uint8_t)(a>>8); wt[2]=(uint8_t)(a&0xFF); wt[3]=test_data[i];
        rt[1]=(uint8_t)(a>>8); rt[2]=(uint8_t)(a&0xFF); rt[3]=0xFF;
        cs_assert(); cyhal_spi_transfer(&mSPI,wt,4,wr,4,0xFF); cs_deassert();
        rx_index = 3u;
#else
        wt[1]=0x00; wt[2]=(uint8_t)(a>>8); wt[3]=(uint8_t)(a&0xFF); wt[4]=test_data[i];
        rt[1]=0x00; rt[2]=(uint8_t)(a>>8); rt[3]=(uint8_t)(a&0xFF); rt[4]=0xFF;
        cs_assert(); cyhal_spi_transfer(&mSPI,wt,5,wr,5,0xFF); cs_deassert();
        rx_index = 4u;
#endif
        uint8_t sr_after_write = read_status();
        if (wait_ready_after_write() != CY_RSLT_SUCCESS) {
            task_print("[4.%u] wait_ready timeout\r\n", i);
            continue;
        }
        uint8_t sr_ready = read_status();
        /* READ back 1 byte */
        cs_assert();
#if (EEPROM_ADDR_BYTES == 1u)
        cyhal_spi_transfer(&mSPI,rt,3,rr,3,0xFF);
#elif (EEPROM_ADDR_BYTES == 2u)
        cyhal_spi_transfer(&mSPI,rt,4,rr,4,0xFF);
#else
        cyhal_spi_transfer(&mSPI,rt,5,rr,5,0xFF);
#endif
        cs_deassert();
        uint8_t got = rr[rx_index];
        task_print("[4.%u] addr=0x%04X wrote=%02X got=%02X SRw=%02X SRr=%02X  %s\r\n",
            i, a, test_data[i], got, sr_after_write, sr_ready, (got==test_data[i])?"PASS":"FAIL");
        if (got == test_data[i]) ok_count++;
    }
    xSemaphoreGive(Semaphore_SPI);
    task_print("[4] %u/4 bytes correct\r\n", ok_count);

    return pdFALSE;
}
static const CLI_Command_Definition_t cmd_ediag = {
    "ediag", "\r\nediag: EEPROM 分步诊断 (SPI / MISO / W# / 读写)\r\n", cli_ediag, 0
};

void task_eeprom_init(void)
{
    FreeRTOS_CLIRegisterCommand(&cmd_er);
    FreeRTOS_CLIRegisterCommand(&cmd_ew);
    FreeRTOS_CLIRegisterCommand(&cmd_eauto);
    FreeRTOS_CLIRegisterCommand(&cmd_ebit);
    FreeRTOS_CLIRegisterCommand(&cmd_eid);
    FreeRTOS_CLIRegisterCommand(&cmd_ediag);
}
