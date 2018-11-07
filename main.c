/*

File    : main.c
Author  : Stefano Morgani
Date    : 10 October 2018
REV 1.0

This example shows how to record the data from the microphones and save them in the micro sd.
Moreover it applies a pitch scaling algorithm to the date before playing it from the micro sd.
If you want to get an higher pitch, then change the "TIME_SCALE" in "sola.c" to a value > 1.0.
If you want to get a lower pitch, then change the "TIME_SCALE" in "sola.c" to a value < 1.0.
It was noticed that a maximum of about 1.8 seconds can be recorded in the micro sd without
loosing samples, this is probably due to the micro sd file system that does some job after
a number of write cycles.

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include "sola.h"
#include <main.h>

#include "audio/audio_thread.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "sensors/proximity.h"
#include "button.h"
#include <fat.h>
#include "leds.h"
#include "sdio.h"
#include "spi_comm.h"

static THD_WORKING_AREA(processing_thd_wa, 4096);
static THD_WORKING_AREA(record_thd_wa, 2048);

typedef struct wav_header {
    // RIFF Header
    char riff_header[4]; // Contains "RIFF"
    int32_t wav_size; // Size of the wav portion of the file, which follows the first 8 bytes. File size - 8
    char wave_header[4]; // Contains "WAVE"

    // Format Header
    char fmt_header[4]; // Contains "fmt " (includes trailing space)
    int32_t fmt_chunk_size; // Should be 16 for PCM
    int16_t audio_format; // Should be 1 for PCM. 3 for IEEE Float
    int16_t num_channels;
    int32_t sample_rate;
    int32_t byte_rate; // Number of bytes per second. sample_rate * num_channels * Bytes Per Sample
    int16_t sample_alignment; // num_channels * Bytes Per Sample
    int16_t bit_depth; // Number of bits per sample

    // Data
    char data_header[4]; // Contains "data"
    int32_t data_bytes; // Number of bytes in data. Number of samples * num_channels * sample byte size
    // uint8_t bytes[]; // Remainder of wave file is bytes
} wav_header;
wav_header my_wav, wav_sola;

// Microphones recording variables.
static int16_t *rec_data;
static uint16_t rec_num_samples;
static BSEMAPHORE_DECL(rec_buffer_filled, true);

// File I/O variables.
static FIL file, file_w;   /* file object */
static FRESULT err;
static uint16_t num_write = 0;
UINT bytesWrite = 0;

// Global variables.
uint8_t rec_state = 0;

// Filtering variables.
static SAMPLE sola_input[32750]; // The input wav is 28960 samples, but give some more space in case you want to get an higher pitch.
static SAMPLE sola_output[32750] __attribute__ ((section(".ram4"))); // Put this array in CCM RAM otherwise doesn't fit in memory.
static int32_t num_out_samples = 0;

// This callback is called every 10 ms.
static void mic_callback(int16_t *data, uint16_t num_samples) {
	rec_data = data;
	rec_num_samples = num_samples;
	if(rec_state == 3) { // Save the data only when in "recording state".
		chBSemSignal(&rec_buffer_filled);
	}
	return;
}

static THD_FUNCTION(record_thd, arg) {
	(void) arg;

	while(1) {
		chBSemWait(&rec_buffer_filled);

		set_led(LED3, 1);
		err = f_write(&file, rec_data, rec_num_samples*2, &bytesWrite); // All the 4 microphones data will be saved in the micro sd ("mic.dat" file).
		if (err != FR_OK) {
			//f_close(&file);
			//rec_state = 0;
		}
		set_led(LED3, 0);
		num_write++;
		if(num_write == (181)) { // 181 writes every 10 ms = 1.81 seconds of length for the wav file. Pass to the next state to process the data.
			rec_state = 4;
		}

	}
}

static THD_FUNCTION(processing_thd, arg)
{
    (void) arg;
    chRegSetThreadName(__FUNCTION__);

	uint32_t index = 0, index_out = 0;
	int32_t samples_sum = 0;
	int16_t temp[4];
	int16_t temp_filt[5];
	uint8_t filter_order = 2;
	UINT bytesRead;

	// Fill the wav header that will be fix.
	// We assume a length of 1.81 seconds for the wav file, mono, 16 KHz, 2 bytes per sample.
	strcpy(my_wav.riff_header, "RIFF");
	my_wav.wav_size = 36 + 57920; //Given in bytes: 36 bytes for header + 57920 bytes of audio data (16000 KHz * 2 bytes per sample * 1.81 seconds)
	strcpy(my_wav.wave_header, "WAVE");
	strcpy(my_wav.fmt_header, "fmt ");
	my_wav.fmt_chunk_size = 16;
	my_wav.audio_format = 1;
	my_wav.num_channels = 1;
	my_wav.sample_rate = 16000;
	my_wav.byte_rate = 16000*2;
	my_wav.sample_alignment = 2;
	my_wav.bit_depth = 16;
	strcpy(my_wav.data_header, "data");
	my_wav.data_bytes = 57920;

	// We don't have all the information to fill the header for the wav created after the "sola algorithm" (time scaling).
	// In particular the wav size is unknown, it will be filled after running the "sola algorithm".
	strcpy(wav_sola.riff_header, "RIFF");
	//wav_sola.wav_size = 36 + ...; // To be filled after.
	strcpy(wav_sola.wave_header, "WAVE");
	strcpy(wav_sola.fmt_header, "fmt ");
	wav_sola.fmt_chunk_size = 16;
	wav_sola.audio_format = 1;
	wav_sola.num_channels = 1;
	wav_sola.sample_rate = 16000;
	wav_sola.byte_rate = 16000*2;
	wav_sola.sample_alignment = 2;
	wav_sola.bit_depth = 16;
	strcpy(wav_sola.data_header, "data");
	//wav_sola.data_bytes = ...; // To be filled after.

	setSoundFileVolume(40);

	//Attempt to mount the drive.
	if(!mountSDCard()) {
		return;
	}

    while(1) {
		switch(rec_state) {
			case 0: // Wait for a button press.
				if(button_is_pressed()) {
					rec_state = 1;
				}
				break;

			case 1: // Wait for the button to be released.
				if(!button_is_pressed()) {
					rec_state = 2;
				}
				break;

			case 2: // Create a file on the micro sd to save the microphones raw data.
				set_led(LED1, 1);
				err = f_open(&file, "mic.dat", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
				if (err != FR_OK) {
					rec_state = 0;
				}
				set_led(LED1, 0);

				num_write = 0;
				rec_state = 3;
				break;

			case 3: // Record the microphones to the micro sd.
				set_body_led(1);
				chThdSleepMilliseconds(100);
				break;

			case 4: // Extract the mic0 from the raw data and save it to "mic.wav". Then apply a low pass filter (moving average) and save it to "mic_filt.wav".
					// Finally apply a time scaling effect on the filtered data and save it to "mic_sola.wav".
				set_body_led(0);
				f_close(&file); // The file "mic.dat" was opened on state 2.

				set_led(LED5, 1);

				err = f_open(&file, "mic.dat", FA_READ);
				if (err != FR_OK) {
					rec_state = 0;
				}
				err = f_open(&file_w, "mic.wav", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
				if (err != FR_OK) {
					rec_state = 0;
				}

				// Write the wav header for the mic0.
				struct wav_header *temp_struct = &my_wav;
				err = f_write(&file_w, temp_struct, sizeof(*temp_struct), &bytesWrite); // The header is 44 bytes.
				if (err != FR_OK) {
					f_close(&file_w);
					rec_state = 0;
				}

				index = 0;
				while(index < 231680) { // 231680 = 1.81 seconds * 128000 (16 KHz * 2 bytes per sample * 4 microphones).
					// Read one sample for each microphone.
					err = f_read(&file, temp, 8, &bytesRead);
					if (err != FR_OK) {
						f_close(&file);
						rec_state = 0;
					}
					// Write only the sample of mic0.
					err = f_write(&file_w, &temp[0], 2, &bytesWrite);
					if (err != FR_OK) {
						f_close(&file_w);
						rec_state = 0;
					}
					// Go to next sample.
					index += 8;
				}

				f_close(&file);
				f_close(&file_w);


				err = f_open(&file, "mic.wav", FA_READ);
				if (err != FR_OK) {
					rec_state = 0;
				}

				err = f_open(&file_w, "mic_filt.wav", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
				if (err != FR_OK) {
					rec_state = 0;
				}
				// Write the wav header for filtered mic0.
				err = f_write(&file_w, temp_struct, sizeof(*temp_struct), &bytesWrite); // The header is 44 bytes.
				if (err != FR_OK) {
					f_close(&file_w);
					rec_state = 0;
				}

				// Apply a low pass filter by using a moving average. This is to clean a bit the recorded audio:
				// filt_sample[i] = (sample[i-filter_order]+..+sample[i+filter_order])/(filter_order*2+1)
				index = 0;
				index_out = 0;
				while(index_out < 28960) { // 28960 = 1.81 * 16000 samples.
					if(index_out < filter_order) { // The first "filter_order" samples are simply not filtered.
						f_lseek(&file, 44+index_out*2); // 44 bytes is the wav header + each sample is 2 bytes.
						err = f_read(&file, temp_filt, 2, &bytesRead);
						if (err != FR_OK) {
							f_close(&file);
							rec_state = 0;
						}
						err = f_write(&file_w, &temp_filt[0], 2, &bytesWrite);
						if (err != FR_OK) {
							f_close(&file_w);
							rec_state = 0;
						}
					} else if (index_out >= (uint16_t)(16000-filter_order)) { // The last "filter_order" samples are simply not filtered.
						f_lseek(&file, 44+index_out*2);
						err = f_read(&file, temp_filt, 2, &bytesRead);
						if (err != FR_OK) {
							f_close(&file);
							rec_state = 0;
						}
						err = f_write(&file_w, &temp_filt[0], 2, &bytesWrite);
						if (err != FR_OK) {
							f_close(&file_w);
							rec_state = 0;
						}
					} else {
						f_lseek(&file, 44+index_out*2-filter_order*2);
						err = f_read(&file, temp_filt, (filter_order*2+1)*2, &bytesRead);
						if (err != FR_OK) {
							f_close(&file);
							rec_state = 0;
						}

						samples_sum = 0;
						for(int j=0; j<filter_order*2+1; j++) {
							samples_sum += temp_filt[j];
						}
						samples_sum /= (filter_order*2+1);

						err = f_write(&file_w, &samples_sum, 2, &bytesWrite);
						if (err != FR_OK) {
							f_close(&file_w);
							rec_state = 0;
						}
					}

					index_out++;
				}

				f_close(&file);
				f_close(&file_w);

				err = f_open(&file, "mic_filt.wav", FA_READ);
				if (err != FR_OK) {
					rec_state = 0;
				}
				err = f_open(&file_w, "mic_sola.wav", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
				if (err != FR_OK) {
					rec_state = 0;
				}
				f_lseek(&file, 44); // Skip the header.
				err = f_read(&file, sola_input, 28960*2, &bytesRead);
				if (err != FR_OK) {
					f_close(&file);
					rec_state = 0;
				}

				// Apply a time scaling algorithm to the data.
				num_out_samples = sola(sola_output, sola_input, 28960);

				// Fill the wav header with actual data.
				wav_sola.wav_size = 36 + num_out_samples*2;
				wav_sola.data_bytes = num_out_samples*2;
				temp_struct = &wav_sola;

				// Write the wav header for the output file.
				err = f_write(&file_w, temp_struct, sizeof(*temp_struct), &bytesWrite); // The header is 44 bytes.
				if (err != FR_OK) {
					f_close(&file_w);
					rec_state = 0;
				}

				// Copy from the CCM to the main RAM in order to write to the micro sd.
				memcpy(sola_input, sola_output, num_out_samples*2);
				err = f_write(&file_w, sola_input, num_out_samples*2, &bytesWrite);
				if (err != FR_OK) {
					f_close(&file_w);
					rec_state = 0;
				}

				f_close(&file);
				f_close(&file_w);

				set_led(LED5, 0);

				rec_state = 5;
				break;

			case 5: // Play the recorded data (pitch scaled).
				chThdSleepMilliseconds(1000);
				// Resample based on the output of the sola algorithm:
				// - if the data is changed to a slower tempo, then play faster to get the same length but with higher pitch
				// - if the data is changed to a faster tempo, then play slower to get the same length but with lower pitch.
				playSoundFile("mic_sola.wav", SF_FORCE_CHANGE, 16000*num_out_samples/28960);
				waitSoundFileHasFinished();
				rec_state = 0;
				break;
		}

    }

}

int main(void) {

    halInit();
    chSysInit();
    mpu_init();

	clear_leds();
	set_body_led(0);
	set_front_led(0);
    mic_start(mic_callback);
    dac_start();
    sdio_start();
    playSoundFileStart();
    spi_comm_start();

    chThdCreateStatic(processing_thd_wa, sizeof(processing_thd_wa), NORMALPRIO, processing_thd, NULL);
    chThdCreateStatic(record_thd_wa, sizeof(record_thd_wa), NORMALPRIO, record_thd, NULL);

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
