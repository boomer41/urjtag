/*
 * $Id$
 *
 * Segger J-Link cable driver using libjaylink.
 *
 * Copyright (C) 2020 Stephan Brunner <s.brunner@stephan-brunner.net>
 *
 * The code was largely inspired by gpio.c.
 * Several magic values have been taken from jlink.c.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 */

#include <sysdep.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <urjtag/cable.h>
#include <urjtag/chain.h>

#include "generic.h"

#include <libjaylink/libjaylink.h>

struct urj_jaylink_params
{
    struct jaylink_context *ctx;
    struct jaylink_device_handle *handle;
    uint8_t last_tdo;
};

struct urj_jaylink_device_selection
{
    uint8_t use_serial_number;
    uint32_t serial_number;

    uint8_t use_index;
    unsigned long index;
};

static int urj_jaylink_setup_device(struct jaylink_device_handle *pHandle)
{
    int ret;

    ret = jaylink_select_interface(pHandle, JAYLINK_TIF_JTAG, NULL);

    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "Failed to switch device to JTAG mode: %d",
            ret);
        return ret;
    }

    return JAYLINK_OK;
}

static struct jaylink_device_handle *urj_jaylink_select_device(struct jaylink_context *pContext, const struct urj_jaylink_device_selection *selection)
{
    int ret;
    struct jaylink_device **pDevices;
    size_t count;

    ret = jaylink_discovery_scan(pContext, JAYLINK_HIF_USB);
    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "Failed to scan for devices: %u",
            ret);
        return NULL;
    }

    ret = jaylink_get_devices(pContext, &pDevices, &count);
    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "Failed to get devices: %u",
            ret);
        return NULL;
    }

    urj_log(URJ_LOG_LEVEL_DEBUG, _("Found %zu devices\n"), count);

    if (!selection->use_index && !selection->use_serial_number && count > 1)
    {
        urj_error_set(URJ_ERROR_NOTFOUND,
            "Multiple devices found, please use serial number or index parameters");
        return NULL;
    }

    struct jaylink_device_handle *handle = NULL;

    for (unsigned long i = 0; i < count; i++)
    {
        struct jaylink_device *pDevice = pDevices[i];

        if (selection->use_index && i != selection->index)
        {
            continue;
        }

        if (selection->use_serial_number)
        {
            uint32_t serial_number;
            ret = jaylink_device_get_serial_number(pDevice, &serial_number);
            if (ret != JAYLINK_OK)
            {
                urj_warning(_("Failed to get serial number of device with index %lu\n"), i);
                continue;
            }

            if (serial_number != selection->serial_number)
            {
                continue;
            }
        }

        ret = jaylink_open(pDevice, &handle);

        if (ret != JAYLINK_OK)
        {
            continue;
        }

        ret = urj_jaylink_setup_device(handle);

        if (ret == JAYLINK_OK)
        {
            break;
        }

        jaylink_close(handle);
        handle = NULL;
    }

    jaylink_free_devices(pDevices, 1);

    return handle;
}

static int urj_jaylink_check_hardware(struct jaylink_device_handle *pHandle)
{
    struct jaylink_hardware_status status;
    int ret;

    ret = jaylink_get_hardware_status(pHandle, &status);
    if (ret != JAYLINK_OK)
    {
        return ret;
    }

    // 1.5V taken from jlink.c
    if (status.target_voltage < 1500)
    {
        urj_error_set(URJ_ERROR_ILLEGAL_STATE,
            "Target voltage under 1500mV, currently is %umV\n",
            status.target_voltage);
        return JAYLINK_ERR;
    }

    return JAYLINK_OK;
}

static void urj_jaylink_parse_params(const urj_param_t *params[], struct urj_jaylink_device_selection *selection)
{
    selection->use_index = 0;
    selection->use_serial_number = 0;

    if (!params)
    {
        return;
    }

    int ret;

    for (int i = 0; params[i]; i++)
    {
        switch (params[i]->key)
        {
            case URJ_CABLE_PARAM_KEY_INDEX:
                selection->index = params[i]->value.lu;
                selection->use_index = 1;
                break;
            case URJ_CABLE_PARAM_KEY_DESC:
                ret = jaylink_parse_serial_number(params[i]->value.string, &selection->serial_number);
                if (ret == JAYLINK_OK)
                {
                    selection->use_serial_number = 1;
                }
                else
                {
                    urj_warning(_("Failed to parse serial number, ignoring\n"));
                }
                break;
            default:
                break;
        }
    }
}

static int urj_jaylink_connect(urj_cable_t *cable, const urj_param_t *params[])
{
    int ret;
    struct urj_jaylink_params *cable_params = calloc(1, sizeof(struct urj_jaylink_params));

    if (!cable_params)
    {
        urj_error_set(URJ_ERROR_OUT_OF_MEMORY, _("calloc(%zd) fails"),
            sizeof(*cable_params));
        return URJ_STATUS_FAIL;
    }

    struct urj_jaylink_device_selection selection_params;
    urj_jaylink_parse_params(params, &selection_params);

    ret = jaylink_init(&cable_params->ctx);
    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "Failed to initialize libjaylink context");
        return URJ_STATUS_FAIL;
    }

    struct jaylink_device_handle *deviceHandle = urj_jaylink_select_device(cable_params->ctx, &selection_params);

    if (!deviceHandle)
    {
        urj_error_set(URJ_ERROR_NOTFOUND,
            "Failed to find a device supported by libjaylink");
        return URJ_STATUS_FAIL;
    }

    ret = urj_jaylink_check_hardware(deviceHandle);

    if (ret != JAYLINK_OK)
    {
        return URJ_STATUS_FAIL;
    }

    cable_params->handle = deviceHandle;
    cable->params = cable_params;
    cable->delay = 1000;

    return JAYLINK_OK;
}

static void urj_jaylink_close(urj_cable_t *cable)
{
    struct urj_jaylink_params *params = cable->params;
    jaylink_close(params->handle);
    params->handle = NULL;

    jaylink_exit(params->ctx);
    params->ctx = NULL;
}

static void urj_jaylink_disconnect(urj_cable_t *cable)
{
    urj_tap_chain_disconnect(cable->chain);
    urj_jaylink_close(cable);
}

static void urj_jaylink_free(urj_cable_t *cable)
{
    free(cable->params);
    free(cable);
}

static int urj_jaylink_init(urj_cable_t *cable)
{
    (void) cable;
    return URJ_STATUS_OK;
}

static void urj_jaylink_done(urj_cable_t *cable)
{
    urj_jaylink_close(cable);
}

static void urj_jaylink_set_frequency(urj_cable_t *cable, uint32_t freq)
{
    int ret;
    struct urj_jaylink_params *params = cable->params;

    // Need kHz
    freq /= 1000;

    if (freq > UINT16_MAX)
    {
        freq = UINT16_MAX;
    }

    ret = jaylink_set_speed(params->handle, (uint16_t) freq);

    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "Failed to set frequency: %d",
            ret);
    }
}

static void urj_jaylink_clock(urj_cable_t *cable, int tms, int tdi, int n)
{
    int ret;
    struct urj_jaylink_params *params = cable->params;

    size_t bytes_required = n / 8;
    if (n % 8)
    {
        bytes_required++;
    }

    uint8_t *tdo8 = malloc(bytes_required);
    uint8_t *tdi8 = malloc(bytes_required);
    uint8_t *tms8 = malloc(bytes_required);

    if (!tdo8 || !tdi8 || !tms8)
    {
        urj_error_set(URJ_ERROR_OUT_OF_MEMORY, _("malloc(%zd) fails"),
            bytes_required);
        goto free;
    }

    memset(tms8, tms ? 0xFF : 0x00, bytes_required);
    memset(tdi8, tdi ? 0xFF : 0x00, bytes_required);

    ret = jaylink_jtag_io(params->handle, tms8, tdi8, tdo8, n, JAYLINK_JTAG_VERSION_3);
    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "JTAG I/O failed: %d",
            ret);
    }
    else
    {
        params->last_tdo = tdo8[bytes_required - 1] & (1u << ((n - 1u) % 8u)) ? 1 : 0;
    }

free:
    free(tdo8);
    free(tdi8);
    free(tms8);
}

static int urj_jaylink_get_tdo(urj_cable_t *cable)
{
    struct urj_jaylink_params *params = cable->params;
    return params->last_tdo;
}

static int urj_jaylink_transfer(urj_cable_t *cable, int len, const char *in, char *out)
{
    int ret;
    struct urj_jaylink_params *params = cable->params;

    size_t bytes_required = len / 8;
    if (len % 8)
    {
        bytes_required++;
    }

    uint8_t *tms = calloc(bytes_required, 1);
    uint8_t *tdi = calloc(bytes_required, 1);
    uint8_t *tdo = malloc(bytes_required);
    if (!tms || !tdi || !tdo)
    {
        ret = JAYLINK_ERR;
        goto free;
    }

    for (size_t i = 0; i < len; i++)
    {
        if (in[i])
        {
            tdi[i / 8u] |= 1u << (i % 8u);
        }
    }

    urj_tap_cable_wait(cable);
    ret = jaylink_jtag_io(params->handle, tms, tdi, tdo, len, JAYLINK_JTAG_VERSION_3);

    if (out && ret == JAYLINK_OK)
    {
        for (size_t i = 0; i < len; i++)
        {
            if (tdo[i / 8u] & (1u << (i % 8u)))
            {
                out[i] = 1;
            }
            else
            {
                out[i] = 0;
            }
        }
    }

    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "JTAG I/O failed: %d",
            ret);
    }

free:
    free(tms);
    free(tdi);
    free(tdo);

    return ret == JAYLINK_OK ? len : -1;
}

static int urj_jaylink_set_signal(urj_cable_t *cable, int mask, int val)
{
    int ret = JAYLINK_OK;
    struct urj_jaylink_params *params = cable->params;

    // Only TRST can be set
    if (mask & URJ_POD_CS_TRST)
    {
        if (val & URJ_POD_CS_TRST)
        {
            ret = jaylink_jtag_set_trst(params->handle);
        }
        else
        {
            ret = jaylink_jtag_clear_trst(params->handle);
        }
    }

    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "Failed to set signals: %d",
            ret);
    }

    return ret == JAYLINK_OK ? 0 : -1;
}

static int urj_jaylink_get_signal(urj_cable_t *cable, urj_pod_sigsel_t sig)
{
    struct urj_jaylink_params *params = cable->params;
    struct jaylink_hardware_status status;
    int ret;

    ret = jaylink_get_hardware_status(params->handle, &status);
    if (ret != JAYLINK_OK)
    {
        urj_error_set(URJ_ERROR_IO,
            "Failed to get hardware status: %d",
            ret);
        return -1;
    }

    uint8_t s;

    switch (sig)
    {
        case URJ_POD_CS_TCK:
            s = status.tck;
            break;
        case URJ_POD_CS_TDI:
            s = status.tdi;
            break;
        case URJ_POD_CS_TMS:
            s = status.tms;
            break;
        case URJ_POD_CS_TRST:
            s = status.trst;
            break;
        case URJ_POD_CS_RESET:
            s = status.tres;
            break;
        default:
            return -1;
    }

    return s ? 1 : 0;
}

static void urj_jaylink_help(urj_log_level_t ll, const char *cablename)
{
    urj_log(ll,
        _("Usage: cable %s [index=INDEX] [desc=SERIALNO]\n"
          "\n"
          "INDEX      Index of the device to use.\n"
          "SERIALNO   Find device by serial number.\n"),
        cablename);
}

const urj_cable_driver_t urj_tap_cable_jaylink_driver = {
    .name = "jaylink",
    .description = N_("SEGGER J-Link using libjaylink (EXPERIMENTAL)"),
    .device_type = URJ_CABLE_DEVICE_OTHER,
    .connect = {
        .other = &urj_jaylink_connect
    },
    .disconnect = &urj_jaylink_disconnect,
    .cable_free = &urj_jaylink_free,
    .init = &urj_jaylink_init,
    .done = &urj_jaylink_done,
    .set_frequency = &urj_jaylink_set_frequency,
    .clock = &urj_jaylink_clock,
    .get_tdo = &urj_jaylink_get_tdo,
    .transfer = &urj_jaylink_transfer,
    .set_signal = &urj_jaylink_set_signal,
    .get_signal = &urj_jaylink_get_signal,
    .flush = &urj_tap_cable_generic_flush_one_by_one,
    .help = &urj_jaylink_help
};
