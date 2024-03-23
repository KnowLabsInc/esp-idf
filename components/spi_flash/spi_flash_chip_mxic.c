// Copyright 2015-2020 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include "spi_flash_chip_generic.h"
#include "spi_flash_defs.h"
#include "esp_log.h"
#include "hal/spi_flash_hal.h"

/* Driver for MXIC flash chip */
static const char TAG[] = "chip_mcix";

#define ADDR_32BIT(addr)            (addr >= (1<<24))

esp_err_t spi_flash_chip_mxic_probe(esp_flash_t *chip, uint32_t flash_id)
{
    /* Check manufacturer and product IDs match our desired masks */
    const uint8_t MFG_ID = 0xC2;
    if (flash_id >> 16 != MFG_ID) {
        return ESP_ERR_NOT_FOUND;
    }
    if (chip->read_mode >= SPI_FLASH_OPI_FLAG) {
        // The code here serve for ordinary mxic chip. If opi mode has been selected, go `spi_flash_chip_mxic_opi.c`
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t spi_flash_chip_issi_get_io_mode(esp_flash_t *chip, esp_flash_io_mode_t* out_io_mode);

// Use the same implementation as ISSI chips
#define spi_flash_chip_mxic_get_io_mode spi_flash_chip_issi_get_io_mode
#define spi_flash_chip_mxic_read_reg        spi_flash_chip_generic_read_reg

static const char chip_name[] = "mxic";

static spi_flash_caps_t spi_flash_chip_mxic_get_caps(esp_flash_t *chip)
{
    spi_flash_caps_t caps_flags = 0;
    if ((chip->chip_id & 0xFF) >= 0x19) {
        caps_flags |= SPI_FLASH_CHIP_CAP_32MB_SUPPORT;
    }
    // flash-suspend is not supported
    // reading unique id is not supported.
    return caps_flags;
}

static esp_err_t spi_flash_chip_mxic_detect_size(esp_flash_t *chip, uint32_t *size)
{
    *size = 1 << (chip->chip_id & 0xff);
    return ESP_OK;
}

static esp_err_t spi_flash_chip_mxic_setup_host_io(esp_flash_t *chip)
{
    const bool addr_32bit = chip->chip_drv->get_chip_caps(chip) & SPI_FLASH_CHIP_CAP_32MB_SUPPORT;
    chip->addr32 = addr_32bit;

    switch (chip->read_mode & 0xFFFF) {
    case SPI_FLASH_QIO:
	chip->read_cmd = addr_32bit ? CMD_FASTRD_QIO_4B : CMD_FASTRD_QIO;
	chip->dummylen = SPI_FLASH_QIO_DUMMY_BITLEN / 2;
        break;
    case SPI_FLASH_QOUT:
	chip->read_cmd = addr_32bit ? CMD_FASTRD_QUAD_4B : CMD_FASTRD_QUAD;
	chip->dummylen = SPI_FLASH_QOUT_DUMMY_BITLEN / 2;
        break;
    case SPI_FLASH_DIO:
	chip->read_cmd = addr_32bit ? CMD_FASTRD_DIO_4B : CMD_FASTRD_DIO;
	chip->dummylen = SPI_FLASH_DIO_DUMMY_BITLEN / 2;
        break;
    case SPI_FLASH_DOUT:
	chip->read_cmd = addr_32bit ? CMD_FASTRD_DUAL_4B : CMD_FASTRD_DUAL;
	chip->dummylen = SPI_FLASH_DOUT_DUMMY_BITLEN / 2;
        break;
    case SPI_FLASH_FASTRD:
	chip->read_cmd = addr_32bit ? CMD_FASTRD_4B : CMD_FASTRD;
	chip->dummylen = SPI_FLASH_FASTRD_DUMMY_BITLEN / 2;
        break;
    case SPI_FLASH_SLOWRD:
	chip->read_cmd = addr_32bit ? CMD_READ_4B : CMD_READ;
	chip->dummylen = SPI_FLASH_SLOWRD_DUMMY_BITLEN / 2;
        break;
    default:
        return ESP_ERR_FLASH_NOT_INITIALISED;
    }
    return ESP_OK;
}

static esp_err_t spi_flash_chip_mxic_set_io_mode(esp_flash_t *chip)
{
    /* MXIC uses bit 6 of "basic" SR as Quad Enable */
    //ESP_LOGI(TAG, "set io mode");
    const uint8_t BIT_QE = 1 << 6;
    esp_err_t ret = spi_flash_common_set_io_mode(chip,
                                                 spi_flash_common_write_status_8b_wrsr,
                                                 spi_flash_common_read_status_8b_rdsr,
                                                 BIT_QE);

    if (ret == ESP_OK) {
	// ESP_LOGI(TAG, "configure io mode values");
	ret = spi_flash_chip_mxic_setup_host_io(chip);
    }
    return ret;
}

static esp_err_t spi_flash_chip_mxic_config_host_io_mode(esp_flash_t *chip, uint32_t flags) {
    // ESP_LOGI(TAG, "configure host %p = %02x %d %u", chip, chip->read_cmd, chip->addr32, chip->dummylen * 2u);
    return chip->host->driver->configure_host_io_mode(chip->host, chip->read_cmd,
        chip->addr32 ? 32 : 24,
        chip->dummylen * 2u,
	chip->read_mode | SPI_FLASH_CONFIG_CONF_BITS);
}

static esp_err_t spi_flash_command_mxic_program_page(esp_flash_t *chip, const void *buffer, uint32_t address, uint32_t length)
{
    esp_err_t err;

    err = chip->chip_drv->wait_idle(chip, chip->chip_drv->timeout->idle_timeout);

    if (err == ESP_OK) {
        bool addr_4b = ADDR_32BIT(address);
        spi_flash_trans_t t = {
            .command = (addr_4b ? CMD_PROGRAM_PAGE_4B : CMD_PROGRAM_PAGE),
            .address_bitlen = (addr_4b ? 32 : 24),
            .address = address,
            .mosi_len = length,
            .mosi_data = buffer,
            .flags = SPI_FLASH_TRANS_FLAG_PE_CMD,
        };
        err = chip->host->driver->common_command(chip->host, &t);
        if (err == ESP_OK) {
            chip->busy = 1;
            err = chip->chip_drv->wait_idle(chip, chip->chip_drv->timeout->page_program_timeout);
            if (err == ESP_ERR_NOT_SUPPORTED) {
                ESP_LOGW(TAG, "Idle after program fail, cmd: %02x, address: %08lx", t.command, address);
                err = ESP_OK;
            }
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Program %02x @ %08lx = %d", ADDR_32BIT(address) ? CMD_PROGRAM_PAGE_4B : CMD_PROGRAM_PAGE_4B, address, err);
    }
    return err;
}

static esp_err_t spi_flash_command_mcix_erase(esp_flash_t *chip, uint32_t start_address, uint8_t cmd3, uint8_t cmd4, uint32_t timeout)
{
    esp_err_t err = chip->chip_drv->set_chip_write_protect(chip, false);
    if (err == ESP_OK || err == ESP_ERR_NOT_SUPPORTED) {
        err = chip->chip_drv->wait_idle(chip, chip->chip_drv->timeout->idle_timeout);
        if (err == ESP_OK) {
            //SET_FLASH_ERASE_STATUS(chip, SPI_FLASH_OS_IS_ERASING_STATUS_FLAG);

            bool addr_4b = ADDR_32BIT(start_address);
            spi_flash_trans_t t = {
                .command = (addr_4b ? cmd4 : cmd3),
                .address_bitlen = (addr_4b ? 32 : 24),
                .address = start_address,
                .flags = SPI_FLASH_TRANS_FLAG_PE_CMD,
            };
            err = chip->host->driver->common_command(chip->host, &t);
            if (err == ESP_OK) {
                chip->busy = 1;
                err = chip->chip_drv->wait_idle(chip, timeout);
                if (err == ESP_ERR_NOT_SUPPORTED) {
                    ESP_LOGW(TAG, "Idle after erase fail, cmd: %02x, address: %08lx", t.command, start_address);
                    err = ESP_OK;
                }
            }
            //SET_FLASH_ERASE_STATUS(chip, 0);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erase %02x @%08lx: Error %d", ADDR_32BIT(start_address) ? cmd4 : cmd3, start_address, err);
    }
    return err;
}

static esp_err_t spi_flash_command_mcix_erase_sector(esp_flash_t *chip, uint32_t start_address)
{
    return spi_flash_command_mcix_erase(chip, start_address, CMD_SECTOR_ERASE, CMD_SECTOR_ERASE_4B,
                                        chip->chip_drv->timeout->sector_erase_timeout);
}

static esp_err_t spi_flash_command_mcix_erase_block(esp_flash_t *chip, uint32_t start_address)
{
    return spi_flash_command_mcix_erase(chip, start_address, CMD_LARGE_BLOCK_ERASE, CMD_LARGE_BLOCK_ERASE_4B,
                                        chip->chip_drv->timeout->block_erase_timeout);
}

static esp_err_t spi_flash_chip_mcix_read(esp_flash_t *chip, void *buffer, uint32_t address, uint32_t length)
{
    esp_err_t err = ESP_OK;

    // There is no init function so we have to do this on every single read!
    err = spi_flash_chip_mxic_config_host_io_mode(chip, SPI_FLASH_CONFIG_IO_MODE_32B_ADDR);
    // chip->chip_drv->config_host_io_mode(chip, SPI_FLASH_CONFIG_IO_MODE_32B_ADDR);
    if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "configure host io mode failed - unsupported");
    }

    while (err == ESP_OK && length > 0) {
	uint32_t read_len = MIN(length, 64); // Some kind of 64 byte limit!
        err = chip->host->driver->read(chip->host, buffer, address, read_len);

	address += read_len;
	length -= read_len;
	buffer += read_len;
    }

    return err;
}

// The mxic chip can use the functions for generic chips except from set read mode and probe,
// So we only replace these two functions.
const spi_flash_chip_t esp_flash_chip_mxic = {
    .name = chip_name,
    .timeout = &spi_flash_chip_generic_timeout,
    .probe = spi_flash_chip_mxic_probe,
    .reset = spi_flash_chip_generic_reset,
    .detect_size = spi_flash_chip_mxic_detect_size,
    .erase_chip = spi_flash_chip_generic_erase_chip,
    .erase_sector = spi_flash_command_mcix_erase_sector,
    .erase_block = spi_flash_command_mcix_erase_block,
    .sector_size = 4 * 1024,
    .block_erase_size = 64 * 1024,

    .get_chip_write_protect = spi_flash_chip_generic_get_write_protect,
    .set_chip_write_protect = spi_flash_chip_generic_set_write_protect,

    .num_protectable_regions = 0,
    .protectable_regions = NULL,
    .get_protected_regions = NULL,
    .set_protected_regions = NULL,

    .read = spi_flash_chip_mcix_read,
    .write = spi_flash_chip_generic_write,
    .program_page = spi_flash_command_mxic_program_page,
    .page_size = 256,
    .write_encrypted = spi_flash_chip_generic_write_encrypted,

    .wait_idle = spi_flash_chip_generic_wait_idle,
    .set_io_mode = spi_flash_chip_mxic_set_io_mode,
    .get_io_mode = spi_flash_chip_mxic_get_io_mode,

    .read_reg = spi_flash_chip_mxic_read_reg,
    .yield = spi_flash_chip_generic_yield,
    .sus_setup = spi_flash_chip_generic_suspend_cmd_conf,
    .read_unique_id = spi_flash_chip_generic_read_unique_id_none,
    .get_chip_caps = spi_flash_chip_mxic_get_caps,
    .config_host_io_mode = spi_flash_chip_mxic_config_host_io_mode,
};
