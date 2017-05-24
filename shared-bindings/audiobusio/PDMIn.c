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

#include "lib/utils/context_manager_helpers.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared-bindings/audio_bus_io/AudioOut.h"

//| .. currentmodule:: audio_bus_io
//|
//| :class:`PDMIn` -- Record an input PDM audio stream
//| ========================================================
//|
//| PDMIn can be used to record an input audio signal on a given set of pins.
//|
//| .. class:: PDMIn(clock_pin, data_pin, \*, frequency=8000, bit_depth=8, mono=True, oversample=64)
//|
//|   Create a PDMIn object associated with the given pins. This allows you to
//|   record audio signals from the given pins.
//|
//|   :param ~microcontroller.Pin clock_pin: The pin to output the clock to
//|   :param ~microcontroller.Pin data_pin: The pin to read the data from
//|   :param int sample_source: Frequency of the resulting samples
//|   :param int bit_depth: Final number of bits per sample. Must be divisible by 8
//|   :param bool mono: True when capturing a single channel of audio, captures two channels otherwise
//|   :param int oversample: Number of single bit samples to decimate into a final sample
//|
//|   Simple record to buffer::
//|
//|     import audio_bus_io
//|     import board
//|
//|     # Prep a buffer to record into
//|     b = bytearray(200)
//|     with audio_bus_io.PDMIn(board.MICROPHONE_DATA, board.MICROPHONE_CLOCK) as mic:
//|         mic.record(b, len(b))
//|
STATIC mp_obj_t audio_bus_io_pdmin_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 2, 2, true);
    mp_obj_t pin_obj = args[0];
    assert_pin(pin_obj, false);
    const mcu_pin_obj_t *pin = MP_OBJ_TO_PTR(pin_obj);

    // create PDMIn object from the given pin
    audio_bus_io_pdmin_obj_t *self = m_new_obj(audio_bus_io_pdmin_obj_t);
    self->base.type = &audio_bus_io_pdmin_type;

    return MP_OBJ_FROM_PTR(self);
}

//|   .. method:: deinit()
//|
//|      Deinitialises the PWMOut and releases any hardware resources for reuse.
//|
STATIC mp_obj_t audio_bus_io_pdmin_deinit(mp_obj_t self_in) {
    audio_bus_io_pdmin_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_audio_bus_io_pdmin_deinit(self);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_bus_io_pdmin_deinit_obj, audio_bus_io_pdmin_deinit);

//|   .. method:: __enter__()
//|
//|      No-op used by Context Managers.
//|
//  Provided by context manager helper.

//|   .. method:: __exit__()
//|
//|      Automatically deinitializes the hardware when exiting a context.
//|
STATIC mp_obj_t audio_bus_io_pdmin_obj___exit__(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    common_hal_audio_bus_io_pdmin_deinit(args[0]);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(audio_bus_io_pdmin___exit___obj, 4, 4, audio_bus_io_pdmin_obj___exit__);


//|   .. method:: record(destination, destination_length)
//|
//|     Records destination_length bytes of samples to destination. This is
//|     blocking. It does not filter the input samples, it only decimates them
//|     to produce a sample.
//|
STATIC mp_obj_t audio_bus_io_pdmin_obj_play(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_loop };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_loop,      MP_ARG_BOOL, {.u_bool = false} },
    };
    audio_bus_io_pdmin_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    common_hal_audio_bus_io_pdmin_play(self, args[ARG_loop].u_bool);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(audio_bus_io_pdmin_play_obj, 1, audio_bus_io_pdmin_obj_play);

STATIC const mp_rom_map_elem_t audio_bus_io_pdmin_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&audio_bus_io_pdmin_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&audio_bus_io_pdmin___exit___obj) },
    { MP_ROM_QSTR(MP_QSTR_record), MP_ROM_PTR(&audio_bus_io_pdmin_record_obj) },
};
STATIC MP_DEFINE_CONST_DICT(audio_bus_io_pdmin_locals_dict, audio_bus_io_pdmin_locals_dict_table);

const mp_obj_type_t audio_bus_io_pdmin_type = {
    { &mp_type_type },
    .name = MP_QSTR_PDMIn,
    .make_new = audio_bus_io_pdmin_make_new,
    .locals_dict = (mp_obj_dict_t*)&audio_bus_io_pdmin_locals_dict,
};
