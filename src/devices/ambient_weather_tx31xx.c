/** @file
    Decoder for Ambient Weather TX-3102 (FCC ID: 2ALZ7-3102C1708) and 
    Ambient Weather TX-3110 (FCC ID: 2ALZ7-3110B1706).

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

        00 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17
        II II FF DD ?? ?? ?C ?? ?? ?? ?? ?? TT TB MM ?? ?? SS	
TX-3102 1a 92 18 70 0c 31 41 ff ff ff 00 00 23 42 01 ff f0 a7
TX-3102 c5 f9 18 70 08 22 42 ff ff ff 00 00 26 82 01 ff f0 76
TX-3110 9c d3 18 30 15 32 27 00 00 00 00 00 27 26 39 00 00 c3

- I: CRC-16/X-MODEM (bytes 02 to 16)
- F: Probably a family code?
- D: Device type?
- C: Channel (1-7)
- T: 12 bit temperature, 234 = 23.4C (byte 13 might be used, couldn't get the probe below -9C)
- B: Battery status and sign for temperature (S?B?), ex: 0010 good voltage, 0000 voltage below 2.6V, 1010 negative temperature and good voltage, 1000 negative temperature and low voltage 
- M: 8 bit soil moisture, (values 01 - 16 for TX-3102 or 00 to 99 for TX-3110)
- S: SUM-8 of bytes 02 to 16 XOR with FF

Bytes 02 to 06 are probably the family, device type, and unique ID.

From the manufacturer, "The WS-8482 can receive up to 7 sensors, including thermo-hygrometer (TX-3110B), 
floating pool and spa thermometer (TX-3107) and soil temperature and moisture (TX-3102)."

*/

#include "decoder.h"

#define TX31XX_BITLEN  168

static int ambient_weather_tx31xx_decode(r_device *decoder, bitbuffer_t *bitbuffer)
{
    data_t *data;
    uint8_t *b;
    uint8_t sensor_id;
    float temperature;
    uint8_t moisture_array[16] = { 0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99 }; // TX-3102 has 16 values mapped from 0% to 99%
    uint8_t moisture;
    uint8_t battery_ok;
    uint8_t sign;
    uint16_t r_crc;
    uint16_t c_crc;
    uint8_t dataforcrc[15];

    uint8_t const preamble[] = {0xaa, 0x2d, 0xd4};

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

    b = bitbuffer->bb[0];
    unsigned start_pos = bitbuffer_search(bitbuffer, 0, 0, preamble, 24);
    if (start_pos == bitbuffer->bits_per_row[0]) {
    return DECODE_ABORT_EARLY;
    }
    if (decoder->verbose) {
        fprintf(stderr, "%s: TX-3102/TX-3108 detected, buffer is %d bits length\n", __func__, bitbuffer->bits_per_row[0]);
    }

    if (bitbuffer->bits_per_row[0] < TX31XX_BITLEN) {
        return DECODE_ABORT_LENGTH;
    }
    
    bitbuffer_extract_bytes(bitbuffer, 0, start_pos + 24, b, 18 * 8); //remove preamble and code word
    bitbuffer_extract_bytes(bitbuffer, 0, 16, dataforcrc, 15 * 8); //remove CRC

    if (b[2] != 0x18) { //check of family code of 0x18
    return 0;
    }

    if (b[3] == 0x70 || b[3] == 0x30) {
    }
    else {
    return 0;
    }	    

    r_crc = (b[0] << 8) | b[1];

    c_crc = crc16(dataforcrc, 15, 0x1021, 0x0000);

    uint8_t c_sum = (add_bytes(dataforcrc, 15) ^ 0xFF) - b[17];
    if (c_sum) {
       return DECODE_FAIL_MIC;
    }

    if (c_crc != r_crc) {
	return DECODE_FAIL_MIC;
    }

    sensor_id = (b[6] & 0xF);

    if (sensor_id > 7) { // channel can only be 1 to 7
	return DECODE_FAIL_SANITY;
    }

    battery_ok = ((b[13] & 0x02) >> 1);

    sign = ((b[13] & 0x08) >> 3);
    if (sign == 1) { // negative temperature
    	temperature = ((b[19] >> 4) & 0xF) * -1 + (b[19] & 0xF) * -0.1;
    }
    else {
    	temperature = ((b[12] >> 4) & 0xF) * 10 + (b[12] & 0xF) + ((b[13] >> 4) & 0xF) * 0.1;
    }

    if (b[3] == 0x70) { //TX-3102
    moisture = moisture_array[(((b[14] >> 4) & 0xF) * 10 + (b[14] & 0xF)) - 1];
    /* clang-format off */
    data = data_make(
            "model", "", DATA_STRING, "Ambient Weather TX-3102",
            "id", "", DATA_INT, sensor_id,
            "temperature", "", DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
            "humidity", "", DATA_FORMAT, "%.1i %%", DATA_INT, moisture,
            "battery", "", DATA_STRING, battery_ok ? "OK" : "LOW",
            "mic", "Integrity", DATA_STRING, "CRC",
             NULL);
    /* clang-format on */
    }
    else{ //TX-3110
    moisture = ((b[14] >> 4) & 0xF) * 10 + (b[14] & 0xF) ;
    /* clang-format off */
    data = data_make(
            "model", "", DATA_STRING, "Ambient Weather TX-3108",
	    "id", "", DATA_INT,	sensor_id,
	    "temperature", "", DATA_FORMAT, "%.1f C", DATA_DOUBLE, temperature,
	    "humidity", "", DATA_FORMAT, "%.1i %%", DATA_INT, moisture,
	    "battery", "", DATA_STRING, battery_ok ? "OK" : "LOW",
            "mic", "Integrity", DATA_STRING, "CRC",
             NULL);
    /* clang-format on */
    }
    decoder_output_data(decoder, data);

    // Return 1 if message successfully decoded
    return 1;
}

static char *output_fields[] = {
        "model",
	"channel",
	"temperature",
	"moisture",
	"humidity",
	"battery",
	"mic",
         NULL,
};

r_device ambient_weather_tx31xx = {
        .name        = "Ambient Weather TX-3102/TX-3110",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 130,  
        .long_width  = 130,  
        .reset_limit = 7000,
        .decode_fn   = &ambient_weather_tx31xx_decode,
        .disabled    = 0,
        .fields      = output_fields,
};
