const NOW = new Date();
const FREQUENCY_C0 = 16.35;
const SAMPLE_RATE = 1 << 16;
const OCTAVES = [2, 6];
const PERIODS = 2;

const C = 0;
const C_Sharp = 1; const D_Flat = C_Sharp;
const D = 2;
const D_Sharp = 3; const E_Flat = D_Sharp;
const E = 4;
const F = 5;
const F_Sharp  = 6; const G_Flat = F_Sharp;
const G = 7;
const G_Sharp = 8; const A_Flat = G_Sharp;
const A = 9;
const A_Sharp = 10; const B_Flat = A_Sharp;
const B = 11;

const NOTE_SYMBOLS = [
    Object.keys({C}),
    Object.keys({C_Sharp, D_Flat}),
    Object.keys({D}),
    Object.keys({D_Sharp, E_Flat}),
    Object.keys({E}),
    Object.keys({F}),
    Object.keys({F_Sharp, G_Flat}),
    Object.keys({G}),
    Object.keys({G_Sharp, A_Flat}),
    Object.keys({A}),
    Object.keys({A_Sharp, B_Flat}),
    Object.keys({B}),
];

const NOTE_NAMES = [
    ["C"],
    ["C#", "Db"],
    ["D"],
    ["D#", "Eb"],
    ["E"],
    ["F"],
    ["F#", "Gb"],
    ["G"],
    ["G#", "Ab"],
    ["A"],
    ["A#", "Bb"],
    ["B"]
];

/**
 *
 * @param {string[]} argv
 * @returns {{}}
 * @constructor
 */
function ParseArgs(argv) {
    let dictionary = {};

    for(let i = 2; i < argv.length; i++) {
        let arg = argv[i];

        if(arg.startsWith('--')) {
            arg = arg.substring(2);

            if(arg === '')
                continue;

            let value = 'true';

            if(i + 1 < argv.length) {
                let maybe_value = argv[i + 1];

                if(!value.startsWith('-')) {
                    value = maybe_value;
                    i++;
                }
            }

            dictionary[arg] = value;
        }
    }

    return dictionary;
}

/**
 * Get the frequency of the given note + octave
 * @param {number} note Ordinal into NOTE_SYMBOLS
 * @param {number} octave Octave to calculate frequency for
 * @returns {number}
 */
function musical_note_freq_hz(note, octave)
{
    return FREQUENCY_C0 * Math.pow(2, (note + (octave * NOTE_SYMBOLS.length)) / NOTE_SYMBOLS.length);
}

/**
 * Map the given value between 0 and 1 to a byte from 0 to 255
 * @param {number} value from 0-1
 * @returns {number} [0-255]
 */
function map_to_byte(value) {
    if(value < 0 || value >= 1)
        return 255;

    return Math.round(255 * value);
}

/**
 *
 * @param note
 * @param octave
 * @returns The least number of elements that is required to populate the table
 * with {PERIODS} periods of the given tone
 */
function get_table_length(note, octave) {
    const frequency = musical_note_freq_hz(note, octave);
    const period = 1.0 / frequency;
    const samples_per_period = SAMPLE_RATE * period;
    return Math.ceil(PERIODS * samples_per_period);
}

/**
 * @param {number} note Ordinal into NOTE_SYMBOLS
 * @param {number} octave Octave to calculate frequency for
 * @returns An array of values from [0, 255], each representing a single sample of
 * sound intensity, at SAMPLE_RATE hz. Length of array is for one period (rounded up).
 */
function generate_table(note, octave) {
    const frequency = musical_note_freq_hz(note, octave);
    const table_len = get_table_length(note, octave);

    let table = [];

    for(let i = 0; i < table_len; i++) {
        let t = i / SAMPLE_RATE;

        //Calculate sine wave value at t, translating and scaling to lie
        //between [0, 1]
        const value = (1 + Math.sin(2 * Math.PI * t * frequency)) / 2;

        table.push(map_to_byte(value));
    }

    return table;
}

function print_table(note, octave) {
    generate_table(note, octave).forEach((value, ndx, arr) => {
        let s = '';

        let start_at = Math.min(
            value,
            ndx === 0? arr[arr.length - 1] : arr[ndx - 1],
            (ndx + 1) === arr.length? arr[0] : arr[ndx + 1]);

        for(let i = 0; i <= 0xFF; i++)
        {
            if(i === 127)
                s += '|';
            else if(i < start_at)
                s += '.';
            else if(i <= value)
                s += '*';
            else
                s += ' ';
        }

        console.log(`[0x${value.toString(16).toUpperCase().padStart(2, '0')}] | ${s}`);
    });
}

const args = ParseArgs(process.argv);

if(!args.output)
    throw new Error(`Output file (--output) is required.`);

if(typeof args.save == 'undefined')
    args.save = 'true';

if(typeof args.print == 'undefined' || args.print === 'false')
{
    args.print = 0;
}
else if(args.print === 'true')
{
    args.print = 1;
}
else
{
    try {
        args.print = parseInt(args.print);
    }
    catch {
        args.print = 0;
    }

    if(isNaN(args.print) || !isFinite(args.print))
        args.print = 0;
}

//Generate table for all notes/octaves with the length that is necessary to contain
//{PERIODS} periods of the lowest frequency tone
let note_tables = [];

for(let i = 0; i < NOTE_SYMBOLS.length; i++) {
    let octave_tables = [];

    for(let j = OCTAVES[0]; j <= OCTAVES[1]; j++) {
        octave_tables.push(generate_table(i, j));
    }

    note_tables.push(octave_tables);
}

const file_content = `
//
// File generated by script '${__filename}' on ${(NOW.getMonth() + 1).toString().padStart(2, '0')}/${NOW.getDate().toString().padStart(2, '0')}/${NOW.getFullYear().toString().padStart(4, '0')} at ${NOW.getHours().toString().padStart(2, '0')}:${NOW.getMinutes().toString().padStart(2, '0')}.
// Contains samples of sine waves for all notes${PERIODS === 0? '' : ` (${PERIODS} periods minimum)`}, octaves ${OCTAVES[0]}-${OCTAVES[1]}, mapped from [0, 1] => [0, 255], at a sample rate of ${SAMPLE_RATE < 1000? `${SAMPLE_RATE} Hz` : (SAMPLE_RATE < 1000000? `~${Math.round(10 * (SAMPLE_RATE / 1000))/ 10} KHz` : `~${Math.round(100 * (SAMPLE_RATE / 1000000))/ 100} MHz`)}.
//

#ifndef AUDIO_CONTROLLER_MUSICAL_NOTE_DATA_H
#define AUDIO_CONTROLLER_MUSICAL_NOTE_DATA_H
#include <cmath>

#include "sine_table.h"
#include "musical_note.h"

namespace musical_note_data {
    namespace data {   
        //Note names
        constexpr auto MUSICAL_NOTE_NAME_NONE = "";
        
        ${NOTE_SYMBOLS.flatMap((symbols, symbols_ndx) => {
            const note_names = NOTE_NAMES[symbols_ndx];
            
            return symbols.map((symbol, ndx) => `constexpr auto MUSICAL_NOTE_NAME_${symbol} = "${note_names[ndx]}";`);
        }).reduce((a, b) => `${a}\r\n\t\t${b}`)}
        
        ${(() => {
            let lines = [];
            
            for(let octave = OCTAVES[0]; octave <= OCTAVES[1]; octave++) {
                if(octave !== OCTAVES[0])
                    lines.push('');
                
                lines.push(`//Note names, octave ${octave}`);
                
                lines.push(...NOTE_SYMBOLS.flatMap((symbols, symbols_ndx) => {
                    const note_names = NOTE_NAMES[symbols_ndx];

                    return symbols.map((symbol, ndx) => `constexpr auto MUSICAL_NOTE_NAME_${symbol}_OCTAVE_${octave} = "${note_names[ndx]}${octave}";`);
                }));
            }
            
            return lines;
        })().reduce((a, b) => `${a}\r\n\t\t${b}`)}
    
        ${note_tables.map((note_table, note_ndx) => {
            const names = NOTE_SYMBOLS[note_ndx];
            const primary_name = names[0];

            return note_table.map((table, octave) => {
                octave = octave + OCTAVES[0];
                return `
        /**
         * Sine wave samples for note ${primary_name}, octave ${octave}
         */
        constexpr uint8_t SINE_TABLE_DATA_${primary_name}_${octave}[] { ${table.map(value => `0x${value.toString(16).toUpperCase().padStart(2, '0')}`).reduce((a, b) => `${a}, ${b}`)} };
                `.trim();
            }).reduce((a, b) => `${a}\r\n\r\n\t\t${b}`);
        }).reduce((a, b) => `${a}\r\n\r\n\t\t${b}`)}
    }

    /**
     * Number of samples processed per second for each sine wave table 
     */
    constexpr auto SAMPLE_RATE = ${SAMPLE_RATE};

    /**
     * The minimum octave present in the table
     */
    constexpr size_t MIN_OCTAVE = ${OCTAVES[0]};

    /**
     * The maximum octave present in the table
     */
    constexpr size_t MAX_OCTAVE = ${OCTAVES[1]};
    
    /**
     * The maximum length of the tables
     */
     constexpr size_t MAX_LENGTH = ${Math.max(...note_tables.flatMap(n => n.map(n => n.length)))};
    
    constexpr size_t ALL_SINE_TABLES_LENGTH_0 = ${NOTE_SYMBOLS.length};
    constexpr size_t ALL_SINE_TABLES_LENGTH_1 = ${1 + OCTAVES[1] - OCTAVES[0]};

    /**
     * 2D array mapping musical note + octave to a table sampling its sine wave
     */
    constexpr SineTable ALL_SINE_TABLES[ALL_SINE_TABLES_LENGTH_0][ALL_SINE_TABLES_LENGTH_1]
	{
        ${note_tables.map((note_table, note_ndx) => {
            const names = NOTE_SYMBOLS[note_ndx];
            const primary_name = names[0];

            return `
                { ${note_table.map((_, octave) => {
                    octave = octave + OCTAVES[0];
                    return `{ .length = sizeof(data::SINE_TABLE_DATA_${primary_name}_${octave}), .data = data::SINE_TABLE_DATA_${primary_name}_${octave} }`;
                }).reduce((a, b) => `${a}, ${b}`)} }
            `.trim();
        }).reduce((a, b) => `${a},\r\n\t\t${b}`)}
	};
}

inline const char* get_note_name(const musical_note note, const bool prefer_flat = false)
{
    switch (note)
    {
        ${NOTE_SYMBOLS.map(symbols => {
            if(symbols.length > 0) {
                if(symbols.length > 1) {
                    return `case musical_note::${symbols[0]}: return prefer_flat? musical_note_data::data::MUSICAL_NOTE_NAME_${symbols[1]} : musical_note_data::data::MUSICAL_NOTE_NAME_${symbols[0]};`;
                }
                else {
                    return `case musical_note::${symbols[0]}: return musical_note_data::data::MUSICAL_NOTE_NAME_${symbols[0]};`;
                }
            }
        }).reduce((a, b) => `${a}\r\n\t\t${b}`)}
        default: return musical_note_data::data::MUSICAL_NOTE_NAME_NONE;
    }
}

inline const char* get_note_name(const musical_note note, const uint8_t octave, const bool prefer_flat = false)
{
    switch (octave)
    {
        ${(() => {
            let lines = [];
        
            for(let octave = OCTAVES[0]; octave <= OCTAVES[1]; octave++) {
                if(octave !== OCTAVES[0])
                    lines.push('');
        
                lines.push(`//Octave ${octave}`);
        
                lines.push(...`case ${octave}:
            switch (note)
            {
                ${NOTE_SYMBOLS.map(symbols => {
                    if(symbols.length > 0) {
                        if(symbols.length > 1) {
                            return `case musical_note::${symbols[0]}: return prefer_flat? musical_note_data::data::MUSICAL_NOTE_NAME_${symbols[1]}_OCTAVE_${octave} : musical_note_data::data::MUSICAL_NOTE_NAME_${symbols[0]}_OCTAVE_${octave};`;
                        }
                        else {
                            return `case musical_note::${symbols[0]}: return musical_note_data::data::MUSICAL_NOTE_NAME_${symbols[0]}_OCTAVE_${octave};`;
                        }
                    }
                }).reduce((a, b) => `${a}\r\n\t\t${b}`)}
                default: return musical_note_data::data::MUSICAL_NOTE_NAME_NONE;
            }`.split('\r\n'));
            }
        
            return lines;
        })().reduce((a, b) => `${a}\r\n\t\t${b}`)}
        default: return musical_note_data::data::MUSICAL_NOTE_NAME_NONE;
    }
}

#endif //AUDIO_CONTROLLER_MUSICAL_NOTE_DATA_H
`.trim();

console.log(file_content);

for(let i = 0; i < args.print; i++) {
    print_table(A, 4);
}

console.log(`Total bytes: ${note_tables.flatMap(n => n.map(n => n.length)).reduce((a, b) => a + b)}`)

if(args.save === 'true') {
    console.log(`Saving file to ${args.output}`);
    require('fs').writeFileSync(args.output, file_content);
}
else {
    console.log('Will not save.')
}