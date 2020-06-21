/** @file
    Decoder for Ambient Weather TX-3102.

    Copyright (C) 2020 Daniel J. Grinkevich

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
 */

/**
The device uses FSK_PCM encoding,

The device sends a transmission every 5 minutes or can be triggered with the refesh button.

A transmission starts with a preamble of 0xAAAAAAAAAA and codeword 0x2DD4

Data layout:
    ?? ?? ?? ?? ?? ?? ?C ?? ?? ?? ?? ?? TT TB MM ?? ?? ?? ?? ??	
    41 70 18 70 0c 31 44 ff ff ff 00 00 22 82 01 ff f2 24 00 00

- C: Channel (1-7)
- T: 12 bit temperature, 228 = 22.8C
- B: Battery status and sign for temperature (S?B?), ex: 0010 good voltage, 0000 voltage below 2.6V, 1010 negative temperature and good voltage, 1000 negative temperature and low voltage 
- M: 8 bit soil moisture, (values 01 - 16)

*/

#include "decoder.h"

#define TX3103_BITLEN      172
#define TX3102_STARTBYTE   0xAA

static int ambient_weather_tx3102_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    int r = 0; // a row index
    uint8_t *b; // bits of a row
    char value[12];  
    int channel;
    float temperature;
    int moisture_array[16] = { 0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99 };
    int moisture;
    int battery_ok;
    int sign;

    /*
     * Early debugging aid to see demodulated bits in buffer and
     * to determine if your limit settings are matched and firing
     * this decode callback.
     *
     * 1. Enable with -vvv (debug decoders)
     * 2. Delete this block when your decoder is working
     */
        if (decoder->verbose > 1) {
            bitbuffer_printf(bitbuffer, "%s: ", __func__);
        }

    b = bitbuffer->bb[r];

    if (b[0] != TX3102_STARTBYTE) {
        return 0;
    }

    if (b[5] != 0x2D && b[6] != 0xD4) { // check for codeword 2dd4
	    return 0;
    }
    
    sprintf(value, "%02x%02x%02x%02x%02x", b[18], b[19], b[20], b[21], b[22]);
    channel = (b[13] & 0xF);
    battery_ok = ((b[20] & 0x02) >> 1);
    sign = ((b[20] & 0x08) >> 3);
    if (sign == 1) {
    temperature = (b[18] & 0xF) * -10 + ((b[19] >> 4) & 0xF) * -1 + (b[19] & 0xF) * -0.1;

    }
    else {
    temperature = ((b[19] >> 4) & 0xF) * 10 + (b[19] & 0xF) + ((b[20] >> 4) & 0xF) * 0.1;
    }
    moisture = moisture_array[(((b[21] >> 4) & 0xF) * 10 + (b[21] & 0xF)) - 1];

    /* clang-format off */
    data = data_make(
            "model", "", DATA_STRING, "Ambient Weather TX-3102",
	    "channel", "", DATA_INT,	channel,
	    "temperature", "", DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
	    "moisture", "", DATA_FORMAT, "%.1i %%", DATA_INT, moisture,
	    "battery", "", DATA_STRING, battery_ok ? "OK" : "LOW",
            "data",  "", DATA_STRING,    value,
            NULL);
    /* clang-format on */
    decoder_output_data(decoder, data);

    // Return 1 if message successfully decoded
    return 1;
}

static char *output_fields[] = {
        "model",
	"channel",
	"temperature",
	"battery",
        "data",
        NULL,
};

r_device ambient_weather_tx3102 = {
        .name        = "Ambient Weather TX-2103",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 200,  // short gap is 132 us
        .long_width  = 200,  // long gap is 224 us
        .gap_limit   = 300,  // some distance above long
        .reset_limit = 5000, // a bit longer than packet gap
        .decode_fn   = &ambient_weather_tx3102_decode,
        .disabled    = 1, // disabled and hidden, use 0 if there is a MIC, 1 otherwise
        .fields      = output_fields,
};