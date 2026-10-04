#!/usr/bin/env python3
"""Rewrites #include <apm32/f4/...> for the layout where apm32/f4/<module>.hpp
is the module's base and apm32/f4/<module>/ holds its concepts and drivers.
A former umbrella header becomes the base plus the concept headers whose names
the file uses.

usage: migrate_includes.py [--dry-run] PATH...
"""

import argparse
import difflib
import os
import re
import sys

F4 = 'apm32/f4/'
SOURCES = ('.h', '.hh', '.hpp', '.c', '.cc', '.cpp', '.ipp')

RENAMED = {
    'adc/adc_sequence.hpp': 'adc/sequence.hpp',
    'adc/common_adc.hpp': 'adc.hpp',
    'adc/low_layer/adc_channels.hpp': 'adc/channels.hpp',
    'adc/low_layer/adc_instances.hpp': 'adc.hpp',
    'adc/low_layer/adc_utils.hpp': 'adc.hpp',
    'can/low_layer/can_instances.hpp': 'can.hpp',
    'chrono/chrono.hpp': 'chrono.hpp',
    'core/core.hpp': 'core.hpp',
    'crc/crc.hpp': 'crc.hpp',
    'dac/dac.hpp': 'dac.hpp',
    'dbg/dbg.hpp': 'dbg.hpp',
    'dma/low_layer/dma_buffer.hpp': 'dma/buffer.hpp',
    'dma/low_layer/dma_channels.hpp': 'dma.hpp',
    'dma/low_layer/dma_controllers.hpp': 'dma.hpp',
    'dma/low_layer/dma_streams.hpp': 'dma.hpp',
    'exti/exti.hpp': 'exti.hpp',
    'flash/flash.hpp': 'flash.hpp',
    'gpio/low_layer/gpio_pin_base.hpp': 'gpio/pin_base.hpp',
    'gpio/low_layer/gpio_pins.hpp': 'gpio.hpp',
    'gpio/low_layer/gpio_ports.hpp': 'gpio.hpp',
    'i2c/i2c.hpp': 'i2c.hpp',
    'nvic/nvic.hpp': 'nvic.hpp',
    'rcc/rcc.hpp': 'rcc.hpp',
    'rcc/rcc_limits.hpp': 'rcc.hpp',
    'rcc/rcc_types.hpp': 'rcc/clock_tree.hpp',
    'spi/low_layer/spi_instances.hpp': 'spi.hpp',
    'spi/low_layer/spi_types.hpp': 'spi.hpp',
    'spi/low_layer/spi_utils.hpp': 'spi.hpp',
    'spi/spi.hpp': 'spi.hpp',
    'tim/low_layer/tim_channels.hpp': 'tim/channels.hpp',
    'usart/usart.hpp': 'usart.hpp',
}


def words(*names):
    return re.compile(r'\b(?:' + '|'.join(names) + r')\b')


ADC = ('adc.hpp',
       words(r'adc[123]', r'(?:some|is)_adc_instance', r'common_registers',
             r'max_clock_frequency', r'powerup_time', r'vref', r'resolution',
             r'full_scale', r'max_code', r'lsb', r'codes_per_volt',
             r'is_compatible_dma_(?:stream|channel)', r'trigger_edge',
             r'(?:inj|reg)_trigger(?:_event)?', r'start_(?:injected|regular)',
             r'j?eoc_flag', r'acknowledge_j?eoc', r'clock_prescalers',
             r'calculate_prescaler', r'prescaler_to_field',
             r'common_irq_priority', r'init_common'),
       [('adc/channels.hpp',
         words(r'adc\d+_in\d+', r'channel_type', r'sampletime',
               r'(?:some|is)_adc_channel',
               r'(?:injected|regular)_rank_sequence',
               r'is_(?:injected|regular)_sequence', r'adc::channel(?=\s*<)'))])

CAN = ('can.hpp',
       words(r'can[12]', r'(?:some|is)_can_instance', r'irq_handler',
             r'[rt]x_pin_config', r'rx_fifo', r'mode', r'error'),
       [('can/bit_timing.hpp',
         words(r'(?:calculate_|find_)?bit_timing', r'with_sync_jump_width',
               r'default_sample_point', r'sample_point_tolerance',
               r'default_sync_jump_width', r'tq_per_bit_(?:min|max)')),
        ('can/filter.hpp',
         words(r'filter_(?:scale|mode|init_session)',
               r'filter_(?:16|32)_(?:mask|list)', r'setup_filter_bank',
               r'encode_(?:16|32)bit_(?:id|mask)'))])

DMA = ('dma.hpp',
       words(r'dma[12]', r'dma[12]_stream[0-7]', r'channel[0-7]',
             r'(?:some|is)_dma_(?:controller|stream|channel)_instance',
             r'(?:controller|stream)_(?:registers|count)'),
       [('dma/buffer.hpp',
         words(r'dma_data_type', r'double_buffer', r'owned_storage',
               r'static_storage', r'dma::buffer(?=\s*<)'))])

GPIO = ('gpio.hpp',
        words(r'port', r'pin', r'pull', r'speed', r'output_type', r'altfunc',
              r'mode', r'ports', r'port_(?:registers|count)',
              r'(?:input|output|alternate|analog)_pin_config'),
        [('gpio/pin_base.hpp', words(r'pin_base'))])

TIMERS = (r'(?:timer_instance|advanced_timer|general_purpose_timer'
          r'|32bit_timer|basic_timer|master_timer_instance)')
TIM = ('tim.hpp',
       words(r'tim(?:1[0-4]|[1-9])', r'(?:some|is)_' + TIMERS,
             r'timer_count', r'clock_division', r'count_direction',
             r'counter_mode', r'trigger_output', r'(?:enable|disable)_counter',
             r'update_flag', r'acknowledge_update', r'break_flag',
             r'acknowledge_break', r'calculate_prescaler'),
       [('tim/channels.hpp',
         words(r'channel[1-4]', r'channel_at', r'channel_idx',
               r'(?:some|is)_timer_channel_instance', r'capture_compare_flag',
               r'acknowledge_capture_compare', r'capture_filter'))])

SPLIT = {
    'adc/adc.hpp': ADC,
    'adc/low_layer/adc_types.hpp': ADC,
    'can/can.hpp': CAN,
    'can/low_layer/can_types.hpp': CAN,
    'can/low_layer/can_utils.hpp': CAN,
    'dma/dma.hpp': DMA,
    'gpio/gpio.hpp': GPIO,
    'tim/tim.hpp': TIM,
    'tim/low_layer/tim_instances.hpp': TIM,
    'tim/low_layer/tim_types.hpp': TIM,
    'tim/low_layer/tim_utils.hpp': TIM,
}

INCLUDE = re.compile(r'^(\s*#\s*include\s*)<(apm32/f4/[^>]+)>(.*)$')
ANY_INCLUDE = re.compile(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]')


def resolve(split, code):
    base, base_names, parts = split
    used = [part for part, names in parts if names.search(code)]
    return ([base] if base_names.search(code) or not used else []) + used


def sort_key(line):
    m = ANY_INCLUDE.match(line)
    return (m.group(1) == '<', m.group(2))


def tidy(lines, fresh):
    seen, kept = set(), []
    for i, line in enumerate(lines):
        m = ANY_INCLUDE.match(line)
        if m:
            if m.group(2) in seen:
                continue
            seen.add(m.group(2))
        kept.append((line, i in fresh))

    out, k = [], 0
    while k < len(kept):
        j = k
        while j < len(kept) and ANY_INCLUDE.match(kept[j][0]):
            j += 1
        if j == k:
            out.append(kept[k][0])
            k += 1
            continue
        block = kept[k:j]
        if any(is_fresh for _, is_fresh in block):
            block.sort(key=lambda entry: sort_key(entry[0]))
        out.extend(line for line, _ in block)
        k = j

    last = max((i for i, l in enumerate(out) if ANY_INCLUDE.match(l)),
               default=-1)
    return [l for i, l in enumerate(out)
            if not (0 < i <= last + 1 and not l.strip()
                    and not out[i - 1].strip())]


def rewrite(path, text):
    lines = text.split('\n')
    code = '\n'.join(l for l in lines if not ANY_INCLUDE.match(l))
    code = re.sub(r'//[^\n]*', '', code)
    me = os.path.abspath(path).replace(os.sep, '/')

    out, fresh = [], set()
    for line in lines:
        m = INCLUDE.match(line)
        old = m.group(2)[len(F4):] if m else None
        if old in RENAMED:
            new = [RENAMED[old]]
        elif old in SPLIT:
            new = resolve(SPLIT[old], code)
        else:
            out.append(line)
            continue
        for header in new:
            if me.endswith('/' + F4 + header):
                continue
            fresh.add(len(out))
            out.append(f'{m.group(1)}<{F4}{header}>{m.group(3)}')

    if not fresh and out == lines:
        return text
    return '\n'.join(tidy(out, fresh))


def sources(paths):
    for path in paths:
        if os.path.isfile(path):
            yield path
            continue
        for root, dirs, files in os.walk(path):
            dirs[:] = sorted(d for d in dirs
                             if d not in ('.git', 'external')
                             and not d.startswith('build'))
            for name in sorted(files):
                if name.endswith(SOURCES):
                    yield os.path.join(root, name)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--dry-run', action='store_true',
                        help='print the diff instead of writing the files')
    parser.add_argument('paths', nargs='+', metavar='PATH',
                        help='files or directories (external/ and build*/ '
                             'are skipped)')
    args = parser.parse_args()

    changed = 0
    for path in sources(args.paths):
        with open(path, encoding='utf-8') as f:
            text = f.read()
        new = rewrite(path, text)
        if new == text:
            continue
        changed += 1
        if args.dry_run:
            sys.stdout.writelines(difflib.unified_diff(
                text.splitlines(True), new.splitlines(True), path, path))
        else:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(new)
            print(path)
    verb = 'would change' if args.dry_run else 'changed'
    print(f'{changed} file(s) {verb}', file=sys.stderr)


if __name__ == '__main__':
    main()
