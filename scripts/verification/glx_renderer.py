#!/usr/bin/env python3
"""Report the actual NVIDIA OpenGL renderer using a hidden 1x1 GLX pbuffer."""
import ctypes as ct
import json
import sys


def bind(library, name, result, *arguments):
    function = getattr(library, name)
    function.restype, function.argtypes = result, arguments
    return function


def main():
    x11, gl = ct.CDLL('libX11.so.6'), ct.CDLL('libGL.so.1')
    pointer, integer, drawable = ct.c_void_p, ct.c_int, ct.c_ulong
    integers = ct.POINTER(integer)
    open_display = bind(x11, 'XOpenDisplay', pointer, ct.c_char_p)
    screen = bind(x11, 'XDefaultScreen', integer, pointer)
    free = bind(x11, 'XFree', integer, pointer)
    close_display = bind(x11, 'XCloseDisplay', integer, pointer)
    choose = bind(gl, 'glXChooseFBConfig', ct.POINTER(pointer),
                  pointer, integer, integers, integers)
    create_context = bind(gl, 'glXCreateNewContext', pointer,
                          pointer, pointer, integer, pointer, integer)
    create_pbuffer = bind(gl, 'glXCreatePbuffer', drawable,
                          pointer, pointer, integers)
    make_current = bind(gl, 'glXMakeContextCurrent', integer,
                        pointer, drawable, drawable, pointer)
    destroy_context = bind(gl, 'glXDestroyContext', None, pointer, pointer)
    destroy_pbuffer = bind(gl, 'glXDestroyPbuffer', None, pointer, drawable)
    get_string = bind(gl, 'glGetString', ct.c_char_p, ct.c_uint)
    display = open_display(None)
    if not display:
        raise RuntimeError('XOpenDisplay failed; check DISPLAY and Xauthority')
    configs = context = pbuffer = None
    try:
        # GLX_DRAWABLE_TYPE=PBUFFER_BIT, GLX_RENDER_TYPE=RGBA_BIT.
        attributes = (integer * 5)(0x8010, 0x0004, 0x8011, 0x0001, 0)
        count = integer()
        configs = choose(display, screen(display), attributes, ct.byref(count))
        if not configs or count.value == 0:
            raise RuntimeError('glXChooseFBConfig found no RGBA pbuffer config')
        context = create_context(display, configs[0], 0x8014, None, 1)
        if not context:
            raise RuntimeError('glXCreateNewContext failed')
        # GLX_PBUFFER_WIDTH=1, GLX_PBUFFER_HEIGHT=1.
        dimensions = (integer * 5)(0x8041, 1, 0x8040, 1, 0)
        pbuffer = create_pbuffer(display, configs[0], dimensions)
        if not pbuffer or not make_current(display, pbuffer, pbuffer, context):
            raise RuntimeError('Could not make the GLX pbuffer context current')
        report = {}
        for name, token in (('vendor', 0x1F00), ('renderer', 0x1F01),
                            ('version', 0x1F02)):
            value = get_string(token)
            if not value:
                raise RuntimeError('glGetString(%s) returned no value' % name)
            report[name] = value.decode('utf-8', errors='replace')
        print(json.dumps(report), flush=True)
        if 'nvidia' not in report['vendor'].lower():
            raise RuntimeError('NVIDIA renderer required; got %s' % report['renderer'])
    finally:
        make_current(display, 0, 0, None)
        if pbuffer:
            destroy_pbuffer(display, pbuffer)
        if context:
            destroy_context(display, context)
        if configs:
            free(configs)
        close_display(display)


if __name__ == '__main__':
    try:
        main()
    except (OSError, AttributeError, RuntimeError) as error:
        print('GLX renderer check failed: %s' % error, file=sys.stderr)
        sys.exit(1)
