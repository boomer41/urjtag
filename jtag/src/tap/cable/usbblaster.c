/*
 * $Id: usbblaster.c,v 1.8 2003/08/22 22:42:02 telka Exp $
 *
 * Altera USB-Blaster<tm> Cable Driver
 * Copyright (C) 2006 K. Waschk
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
 * Written by Kolja Waschk, 2006; http://www.ixo.de
 *
 */

#include "sysdep.h"

#include "cable.h"
#include "parport.h"
#include "chain.h"

#include "generic.h"

#define TCK    0
#define TMS    1
#define TDI    4
#define READ   6
#define SHMODE 7
#define OTHERS ((1<<2)|(1<<3)|(1<<5))

#define TDO    0

static int
usbblaster_init( cable_t *cable )
{
	int i;

	if (parport_open( cable->port ))
		return -1;

	for(i=0;i<64;i++)
		parport_set_data( cable->port, 0 );

	parport_set_control( cable->port, 1 ); // flush
	parport_set_control( cable->port, 0 ); // noflush

	return 0;
}

static void
usbblaster_clock( cable_t *cable, int tms, int tdi, int n )
{
	int i;

	tms = tms ? 1 : 0;
	tdi = tdi ? 1 : 0;

	for (i = 0; i < n; i++) {
		parport_set_data( cable->port, OTHERS | (0 << TCK) | (tms << TMS) | (tdi << TDI) );
		parport_set_data( cable->port, OTHERS | (1 << TCK) | (tms << TMS) | (tdi << TDI) );
		parport_set_control( cable->port, 1 ); // flush
		parport_set_control( cable->port, 0 ); // noflush
	}
}

static int
usbblaster_get_tdo( cable_t *cable )
{
	parport_set_control( cable->port, 0 ); // noflush
	parport_set_data( cable->port, OTHERS ); /* TCK low */
	parport_set_data( cable->port, OTHERS | (1 << READ) ); /* TCK low */
	parport_set_control( cable->port, 1 ); // flush
	parport_set_control( cable->port, 0 ); // noflush
#if 0
    {
	  char x = ( parport_get_data( cable->port ) & (1 << TDO)) ? 1 : 0;
      printf("GetTDO %d\n", x);
      return x;
    }
#else
	return ( parport_get_data( cable->port ) & (1 << TDO)) ? 1 : 0;
#endif
}

static int
usbblaster_set_trst( cable_t *cable, int trst )
{
	return 1;
}

static int
usbblaster_transfer( cable_t *cable, int len, char *in, char *out )
{
	int in_offset = 0;
	int out_offset = 0;
	parport_set_control( cable->port, 0 );
	parport_set_data( cable->port, OTHERS ); /* TCK low */

	while(len - in_offset >= 8)
	{
		int i;
		int chunkbytes = ((len-in_offset)>>3);
		if(chunkbytes > 63) chunkbytes = 63;

		if(out)
			parport_set_data( cable->port,(1<<SHMODE)|(1<<READ)|chunkbytes);
		else
			parport_set_data( cable->port,(1<<SHMODE)|(0<<READ)|chunkbytes);

		for(i=0; i<chunkbytes; i++)
		{
			int j;
			unsigned char b = 0;
			for(j=1; j<256; j<<=1) if(in[in_offset++]) b |= j;
			parport_set_data( cable->port, b );
		};

		if(out) 
		{
			parport_set_control( cable->port, 1 ); // flush
			parport_set_control( cable->port, 0 ); 

			for(i=0; i<chunkbytes; i++)
			{
				int j;
				unsigned char b = parport_get_data( cable->port );
#if 0
                printf("read byte: %02X\n", b);
#endif
                 
				for(j=1; j<256; j<<=1) out[out_offset++] = (b & j) ? 1:0;
			};
		};
	};

	while(len > in_offset)
	{
		char tdi = in[in_offset++] ? 1 : 0;
		parport_set_data( cable->port, OTHERS ); /* TCK low */
		if(out) parport_set_data( cable->port, OTHERS | (1 << READ) | (tdi << TDI)); 
		parport_set_data( cable->port, OTHERS | (1 << TCK)  | (tdi << TDI));
	}

	if(out)
	{
		parport_set_control( cable->port, 1 ); // flush
		parport_set_control( cable->port, 0 );

		while(len > out_offset)
			out[out_offset++] = ( parport_get_data( cable->port ) & (1 << TDO)) ? 1 : 0;
	}

	return 0;
}

void
usbblaster_help( const char *cablename )
{
	printf( _(
		"Usage: cable %s ftdi VID:PID\n"
		"\n"
		"VID        vendor ID (hex, e.g. 9FB, or empty)\n"
		"PID        product ID (hex, e.g. 6001, or empty)\n"
		"\n"
	), cablename );
}

cable_driver_t usbblaster_cable_driver = {
	"UsbBlaster",
	N_("Altera USB-Blaster Cable"),
	generic_connect,
	generic_disconnect,
	generic_cable_free,
	usbblaster_init,
	generic_done,
	usbblaster_clock,
	usbblaster_get_tdo,
	usbblaster_transfer,
	usbblaster_set_trst,
	generic_get_trst,
	usbblaster_help,
};
