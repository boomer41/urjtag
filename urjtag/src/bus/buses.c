/*
 * $Id$
 *
 * Copyright (C) 2003 ETC s.r.o.
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
 * Written by Marcel Telka <marcel@telka.sk>, 2003.
 *
 */

#include "sysdep.h"

#include <stdlib.h>

#include "bus.h"
#include "buses.h"

const urj_bus_driver_t *bus_drivers[] = {
#ifdef ENABLE_BUS_AU1500
    &au1500_bus,
#endif
#ifdef ENABLE_BUS_AVR32
    &avr32_bus_driver,
#endif
#ifdef ENABLE_BUS_BCM1250
    &bcm1250_bus,
#endif
#ifdef ENABLE_BUS_BF526_EZKIT
    &bf526_ezkit_bus,
#endif
#ifdef ENABLE_BUS_BF527_EZKIT
    &bf527_ezkit_bus,
#endif
#ifdef ENABLE_BUS_BF533_STAMP
    &bf533_stamp_bus,
#endif
#ifdef ENABLE_BUS_BF533_EZKIT
    &bf533_ezkit_bus,
#endif
#ifdef ENABLE_BUS_BF537_STAMP
    &bf537_stamp_bus,
#endif
#ifdef ENABLE_BUS_BF537_EZKIT
    &bf537_ezkit_bus,
#endif
#ifdef ENABLE_BUS_BF538F_EZKIT
    &bf538f_ezkit_bus,
#endif
#ifdef ENABLE_BUS_BF548_EZKIT
    &bf548_ezkit_bus,
#endif
#ifdef ENABLE_BUS_BF561_EZKIT
    &bf561_ezkit_bus,
#endif
#ifdef ENABLE_BUS_BSCOACH
    &bscoach_bus,
#endif
#ifdef ENABLE_BUS_EJTAG
    &ejtag_bus,
    &ejtag_dma_bus,
#endif
#ifdef ENABLE_BUS_FJMEM
    &fjmem_bus,
#endif
#ifdef ENABLE_BUS_IXP425
    &ixp425_bus,
#endif
#ifdef ENABLE_BUS_JOPCYC
    &jopcyc_bus,
#endif
#ifdef ENABLE_BUS_H7202
    &h7202_bus,
#endif
#ifdef ENABLE_BUS_LH7A400
    &lh7a400_bus,
#endif
#ifdef ENABLE_BUS_MPC5200
    &mpc5200_bus,
#endif
#ifdef ENABLE_BUS_MPC824X
    &mpc824x_bus,
#endif
#ifdef ENABLE_BUS_PPC405EP
    &ppc405ep_bus,
#endif
#ifdef ENABLE_BUS_PPC440GX_EBC8
    &ppc440gx_ebc8_bus,
#endif
#ifdef ENABLE_BUS_PROTOTYPE
    &prototype_bus,
#endif
#ifdef ENABLE_BUS_PXA2X0
    &pxa2x0_bus,
#endif
#ifdef ENABLE_BUS_PXA27X
    &pxa27x_bus,
#endif
#ifdef ENABLE_BUS_S3C4510
    &s3c4510_bus,
#endif
#ifdef ENABLE_BUS_SA1110
    &sa1110_bus,
#endif
#ifdef ENABLE_BUS_SH7727
    &sh7727_bus,
#endif
#ifdef ENABLE_BUS_SH7750R
    &sh7750r_bus,
#endif
#ifdef ENABLE_BUS_SH7751R
    &sh7751r_bus,
#endif
#ifdef ENABLE_BUS_SHARC_21065L
    &sharc_21065L_bus,
#endif
#ifdef ENABLE_BUS_SLSUP3
    &slsup3_bus,
#endif
#ifdef ENABLE_BUS_TX4925
    &tx4925_bus,
#endif
#ifdef ENABLE_BUS_ZEFANT_XS3
    &zefant_xs3_bus,
#endif
    NULL                        /* last must be NULL */
};

urj_bus_t *urj_bus = NULL;
urj_buses_t urj_buses = { 0, NULL };

void
urj_bus_buses_free (void)
{
    int i;

    for (i = 0; i < urj_buses.len; i++)
        URJ_BUS_FREE (urj_buses.buses[i]);

    free (urj_buses.buses);
    urj_buses.len = 0;
    urj_buses.buses = NULL;
    urj_bus = NULL;
}

void
urj_bus_buses_add (urj_bus_t *abus)
{
    urj_bus_t **b;

    if (abus == NULL)
        return;

    b = realloc (urj_buses.buses, (urj_buses.len + 1) * sizeof (urj_bus_t *));
    if (b == NULL)
    {
        printf (_("Out of memory\n"));
        return;
    }
    urj_buses.buses = b;
    urj_buses.buses[urj_buses.len++] = abus;
    if (urj_bus == NULL)
        urj_bus = abus;
}

void
urj_bus_buses_delete (urj_bus_t *abus)
{
    int i;
    urj_bus_t **b;

    for (i = 0; i < urj_buses.len; i++)
        if (abus == urj_buses.buses[i])
            break;
    if (i >= urj_buses.len)
        return;

    while (i + 1 < urj_buses.len)
    {
        urj_buses.buses[i] = urj_buses.buses[i + 1];
        i++;
    }
    urj_buses.len--;
    b = realloc (urj_buses.buses, urj_buses.len * sizeof (urj_bus_t *));
    if ((b != NULL) || (urj_buses.len == 0))
        urj_buses.buses = b;

    if (urj_bus != abus)
        return;

    if (urj_buses.len > 0)
        urj_bus = urj_buses.buses[0];
}