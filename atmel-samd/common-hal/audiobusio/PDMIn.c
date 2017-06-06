/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>
#include <string.h>

#include "py/gc.h"
#include "py/mperrno.h"
#include "py/runtime.h"
#include "common-hal/audiobusio/PDMIn.h"
#include "shared-bindings/audiobusio/PDMIn.h"
#include "shared-bindings/microcontroller/Pin.h"

#include "asf/sam0/drivers/port/port.h"
#include "samd21_pins.h"

#include "shared_dma.h"
#include "tick.h"

void pdmin_reset(void) {
    while (I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE) {}
    I2S->INTENCLR.reg = I2S_INTENCLR_MASK;
    I2S->INTFLAG.reg = I2S_INTFLAG_MASK;
    I2S->CTRLA.reg &= ~I2S_SYNCBUSY_ENABLE;
    while (I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE) {}
    I2S->CTRLA.reg = I2S_CTRLA_SWRST;
}

void common_hal_audiobusio_pdmin_construct(audiobusio_pdmin_obj_t* self,
                                           const mcu_pin_obj_t* clock_pin,
                                           const mcu_pin_obj_t* data_pin,
                                           uint32_t frequency,
                                           uint8_t bit_depth,
                                           bool mono,
                                           uint8_t oversample) {
    self->clock_pin = clock_pin; // PA10, PA20 -> SCK0, PB11 -> SCK1
    if (clock_pin == &pin_PA10 || clock_pin == &pin_PA20) {
        self->clock_unit = 0;
    } else if (clock_pin == &pin_PB11) {
        self->clock_unit = 1;
    } else {
        mp_raise_ValueError("Invalid clock pin");
    }

    self->data_pin = data_pin; // PA07, PA19 -> SD0, PA08, PB16 -> SD1

    if (data_pin == &pin_PA07 || data_pin == &pin_PA19) {
        self->serializer = 0;
    } else if (data_pin == &pin_PA08
        #ifdef PB16
        || data_pin == &pin_PB16) {
        #else
        ) {
        #endif
        self->serializer = 1;
    } else {
        mp_raise_ValueError("Invalid data pin");
    }

    claim_pin(clock_pin);
    claim_pin(data_pin);

    if (MP_STATE_VM(audiodma_block_counter) == NULL &&
        !allocate_block_counter()) {
        mp_raise_RuntimeError("Unable to allocate audio DMA block counter.");
    }

    if (frequency != 8000 || bit_depth != 8 || !mono) {
        mp_raise_NotImplementedError("");
    }

    if (oversample % 32 != 0) {
        mp_raise_ValueError("Oversample must be multiple of 32.");
    }

    // TODO(tannewt): Use the DPLL to get a more precise sampling rate.
    // DFLL -> GCLK (/600 for 8khz, /300 for 16khz and /150 for 32khz) -> DPLL (*(63 + 1)) -> GCLK ( / 10) -> 512khz

    // For now sample at 8mhz (GCLK3) / 15 ~= 8333 khz

    i2s_init(&self->i2s_instance, I2S);
    struct i2s_clock_unit_config config_clock_unit;
    i2s_clock_unit_get_config_defaults(&config_clock_unit);
    config_clock_unit.clock.gclk_src = GCLK_GENERATOR_3;

    config_clock_unit.clock.mck_src = I2S_MASTER_CLOCK_SOURCE_GCLK;
    config_clock_unit.clock.mck_out_enable = false;

    config_clock_unit.clock.sck_src = I2S_SERIAL_CLOCK_SOURCE_MCKDIV;
    config_clock_unit.clock.sck_div = 7;

    config_clock_unit.frame.number_slots = 2;
    config_clock_unit.frame.slot_size = I2S_SLOT_SIZE_32_BIT;
    config_clock_unit.frame.data_delay = I2S_DATA_DELAY_0;

    config_clock_unit.mck_pin.enable = false;
    config_clock_unit.sck_pin.enable = true;
    config_clock_unit.sck_pin.gpio = self->clock_pin->pin;
    // Mux is always the same.
    config_clock_unit.sck_pin.mux = 6L;
    config_clock_unit.fs_pin.enable = false;
    i2s_clock_unit_set_config(&self->i2s_instance, self->clock_unit, &config_clock_unit);

    struct i2s_serializer_config config_serializer;
    i2s_serializer_get_config_defaults(&config_serializer);
    config_serializer.clock_unit = self->clock_unit;
    config_serializer.mode = I2S_SERIALIZER_RECEIVE;
    config_serializer.data_size = I2S_DATA_SIZE_32BIT;
    config_serializer.data_pin.gpio = self->data_pin->pin;
    // Mux is always the same.
    config_serializer.data_pin.mux = 6L;
    config_serializer.data_pin.enable = true;
    i2s_serializer_set_config(&self->i2s_instance, self->serializer, &config_serializer);
    i2s_enable(&self->i2s_instance);

    self->bytes_per_sample = oversample >> 3;
}

void common_hal_audiobusio_pdmin_deinit(audiobusio_pdmin_obj_t* self) {
    i2s_disable(&self->i2s_instance);
    i2s_reset(&self->i2s_instance);
}

// Algorithm from https://en.wikipedia.org/wiki/Hamming_weight
static uint8_t hamming_weight(uint8_t word) {
    uint8_t b0 = (word >> 0) & 0x55;
    uint8_t b1 = (word >> 1) & 0x55;
    uint8_t c = b0 + b1;
    uint8_t d0 = (c >> 0) & 0x33;
    uint8_t d2 = (c >> 2) & 0x33;
    uint8_t e = d0 + d2;
    uint8_t f0 = (e >> 0) & 0x0f;
    uint8_t f4 = (e >> 4) & 0x0f;
    return f0 + f4;
}

uint32_t common_hal_audiobusio_pdmin_record_to_buffer(audiobusio_pdmin_obj_t* self, uint8_t* output_buffer, uint32_t length) {
    // Write the wave file header.

    // We allocate two 256 byte buffers on the stack to use for double buffering.
    // Our oversample rate is 64 (bits) so each buffer produces 32 samples.
    // TODO(tannewt): Can the compiler optimize better if we fix the size of
    // these buffers?
    uint8_t samples_per_buffer = 32;
    uint16_t bytes_per_buffer = samples_per_buffer * self->bytes_per_sample;
    uint8_t first_buffer[bytes_per_buffer];
    uint8_t second_buffer[bytes_per_buffer];

    COMPILER_ALIGNED(16) DmacDescriptor second_descriptor;

    uint8_t words_per_sample = self->bytes_per_sample / 4;
    uint8_t words_per_buffer = bytes_per_buffer / 4;

    // Set up the DMA
    struct dma_descriptor_config descriptor_config;
    dma_descriptor_get_config_defaults(&descriptor_config);
    descriptor_config.beat_size = DMA_BEAT_SIZE_WORD;
    descriptor_config.step_selection = DMA_STEPSEL_SRC;
    descriptor_config.source_address = (uint32_t)&I2S->DATA[self->serializer];
    descriptor_config.src_increment_enable = false;
    // Block transfer count is the number of beats per block (aka descriptor).
    // In this case there are two bytes per beat so divide the length by two.
    uint16_t block_transfer_count = words_per_buffer;
    if (length * words_per_sample < words_per_buffer) {
        block_transfer_count = length * words_per_sample;
    }
    descriptor_config.block_transfer_count = block_transfer_count;
    descriptor_config.destination_address = ((uint32_t) first_buffer + sizeof(uint32_t) * block_transfer_count);
    descriptor_config.event_output_selection = DMA_EVENT_OUTPUT_BLOCK;
    descriptor_config.next_descriptor_address = 0;
    if (length * words_per_sample > words_per_buffer) {
        descriptor_config.next_descriptor_address = ((uint32_t)&second_descriptor);
    }
    dma_descriptor_create(audio_dma.descriptor, &descriptor_config);

    if (length * words_per_sample > words_per_buffer) {
        block_transfer_count = words_per_buffer;
        descriptor_config.next_descriptor_address = ((uint32_t)audio_dma.descriptor);
        if (length * words_per_sample < 2 * words_per_buffer) {
            block_transfer_count = 2 * words_per_buffer - length * words_per_sample;
            descriptor_config.next_descriptor_address = 0;
        }
        descriptor_config.block_transfer_count = block_transfer_count;
        descriptor_config.destination_address = ((uint32_t) second_buffer + sizeof(uint32_t) * block_transfer_count);
        dma_descriptor_create(&second_descriptor, &descriptor_config);
    }

    switch_audiodma_trigger(I2S_DMAC_ID_RX_0 + self->serializer);

    dma_start_transfer_job(&audio_dma);
    tc_start_counter(MP_STATE_VM(audiodma_block_counter));
    i2s_clock_unit_enable(&self->i2s_instance, self->clock_unit);
    i2s_serializer_enable(&self->i2s_instance, self->serializer);
    I2S->DATA[1].reg = I2S->DATA[1].reg;

    // Record
    uint32_t buffers_processed = 0;
    uint32_t total_bytes = 0;

    int32_t sum1 = 0;
    int32_t sum2 = 0;
    int32_t comb1_1 = 0;
    int32_t comb1_2 = 0;
    int32_t comb2_1 = 0;
    int32_t comb2_2 = 0;
    int32_t sample_average = 0;
    uint64_t start_ticks = ticks_ms;
    while (total_bytes < length) {
        // Wait for the next buffer to fill
        while (tc_get_count_value(MP_STATE_VM(audiodma_block_counter)) == buffers_processed) {
            #ifdef MICROPY_VM_HOOK_LOOP
                MICROPY_VM_HOOK_LOOP
            #endif
        }
        if (tc_get_count_value(MP_STATE_VM(audiodma_block_counter)) != (buffers_processed + 1)) {
            break;
        }
        // Throw away the first ~20ms of data because thats during mic start up.
        if (ticks_ms - start_ticks < 21) {
            mp_printf(&mp_plat_print, "skipping buffer\n");
            buffers_processed++;
            continue;
        }
        uint8_t* buffer = first_buffer;
        DmacDescriptor* descriptor = audio_dma.descriptor;
        if (buffers_processed % 2 == 1) {
            buffer = second_buffer;
            descriptor = &second_descriptor;
        }
        // Decimate the last buffer
        // A CIC filter based on: https://curiouser.cheshireeng.com/2015/01/16/pdm-in-a-tiny-cpu/
        int32_t buffer_sum = 0;
        int32_t samples_gathered = descriptor->BTCNT.reg / words_per_sample;
        for (uint16_t i = 0; i < samples_gathered; i++) {
            for (uint8_t j = 0; j < self->bytes_per_sample; j++) {
                // We use hamming weight to determine the value of each byte
                // rather than a look up table to save memory. This is
                // considered the first stage.
                int16_t one_count = hamming_weight(buffer[i * self->bytes_per_sample + j]);
                sum1 += one_count - (8 - one_count);
                sum2 += sum1;
            }
            int tmp = sum2 - comb1_2;
            comb1_2 = comb1_1;
            comb1_1 = sum2;

            int16_t sample = tmp - comb2_2;
            comb2_2 = comb2_1;
            comb2_1 = tmp;

            buffer_sum += sample;
            int16_t adjusted_sample = sample - sample_average;
            if (i == 0 || i == samples_gathered - 1) {
                mp_printf(&mp_plat_print, "sample: %d\n", adjusted_sample);
            }

            // This filter gives ~12 bits of significance, four from the hamming
            // weight sum and nine (maybe) from the second stage. So, shift away
            // the low bits to pack it into a single unsigned byte.
            output_buffer[total_bytes] = (adjusted_sample >> 2) + 128;
            total_bytes++;
        }
        mp_printf(&mp_plat_print, "%d sum %d num samples\n", buffer_sum, samples_gathered);
        sample_average = buffer_sum / samples_gathered;
        mp_printf(&mp_plat_print, "sample average: %d\n", sample_average);


        buffers_processed++;

        if (length - total_bytes < samples_per_buffer) {
            descriptor->BTCNT.reg = (length - total_bytes) * words_per_sample;
            descriptor->DSTADDR.reg = ((uint32_t) buffer) + (length - total_bytes) * self->bytes_per_sample;
            descriptor->DESCADDR.reg = 0;
        }
        // Write it to the file (may or may not cause an actual write)
    }

    // Turn off the I2S clock and serializer. Peripheral is still enabled.
    i2s_serializer_disable(&self->i2s_instance, self->serializer);
    i2s_clock_unit_disable(&self->i2s_instance, self->clock_unit);

    // Shutdown the DMA
    tc_stop_counter(MP_STATE_VM(audiodma_block_counter));
    dma_abort_job(&audio_dma);

    // Flush the file
    return total_bytes;
}

void common_hal_audiobusio_pdmin_record_to_file(audiobusio_pdmin_obj_t* self, uint8_t* buffer, uint32_t length) {

}
