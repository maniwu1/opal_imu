#include "apdm.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/**
 * This program is used to configure all attached access points and motion monitors on a host into wireless streaming mode.
 *
 * It can accept a single, optional command line parameter for wireless channel. Default is 80.
 *
 * After autoconfiguring a system, you can removed the motion monitors from the docks/cable, wait 15 seconds for them to synchronize with access point(s), at which
 * the LED on the AP will start blinking.
 */

int main(int argc, char **argv)
{
	bool rapid_streaming_mode = false;

	int wireless_channel = 80;
	uint32_t output_data_rate = 128;

	for(int i = 1; i < argc; i++ ) {
		//printf("Argument %d is '%s'\n", i, argv[i]);
		if( strcmp("--rapid", argv[i]) == 0 ) {
			rapid_streaming_mode = true;
		} else if( strcmp("--odr", argv[i]) == 0 ) {
		  i++;
		  if( i < argc ) {
		    output_data_rate = atoi(argv[i]);
		  }
		} else {
			wireless_channel = atoi(argv[i]);
		}
	}

	printf("Autoconfiguring access point and devices on channel %d with output data rate %u...\r\n", wireless_channel, output_data_rate);

	//Allocate memory for the context
	apdm_ctx_t apdm_context = apdm_ctx_allocate_new_context();

	//Open all the accesspoints connected to the host
	int r = apdm_ctx_open_all_access_points(apdm_context);
	if( r != APDM_OK ) {
		printf("Unable to open access points, error code %d, '%s'\r\n", r, apdm_strerror(r));
		return(1);
	}
	apdm_streaming_config_t streaming_config;
	apdm_init_streaming_config(&streaming_config);
	streaming_config.enable_accel = true;
	streaming_config.enable_gyro = true;
	streaming_config.enable_mag = true;
	streaming_config.enable_pressure = true;
	streaming_config.wireless_channel_number = wireless_channel;
	streaming_config.enable_sd_card = true;
	streaming_config.decimation_rate = APDM_DECIMATE_1x1;
	//streaming_config.output_select_rate = APDM_OUTPUT_SELECT_RATE_128;
	//streaming_config.output_rate_hz = 128;
	streaming_config.output_rate_hz = output_data_rate;
	streaming_config.wireless_max_latency_ms = 0;
	streaming_config.wireless_rapid_streaming = rapid_streaming_mode;
	r = apdm_ctx_autoconfigure_devices_and_accesspoint_streaming(apdm_context, &streaming_config);

	if( r != APDM_OK ) {
		printf("Autoconfigure failed, r = %d, %s\n", r, apdm_strerror(r));
		return(-1);
	}


	uint32_t device_id_list[APDM_MAX_NUMBER_OF_SENSORS];
	memset(device_id_list, 0, sizeof(device_id_list));
	if( (r = apdm_ctx_get_device_id_list(apdm_context, device_id_list, APDM_MAX_NUMBER_OF_SENSORS)) != APDM_OK ) {
		printf("Error getting device ID list, r = %d, %s\n", r, apdm_strerror(r));
		return(-1);
	}

	printf("Device ID List is:\n");
	for(int i = 0; i < APDM_MAX_NUMBER_OF_SENSORS && device_id_list[i] != 0; i++ ) {
		printf("  %u\n", device_id_list[i]);
	}


	//Display a full system configuration and context dump
	apdm_log_context(apdm_context, APDM_LL_INFO);



	const char contextDumpfile[] = "./apdm_context_dump.dat";
	if( (r = apdm_ctx_persist_context_to_disk(apdm_context, contextDumpfile)) != APDM_OK ) {
		printf("Failed to persist context to disk... %d %s\n", r, apdm_strerror(r));
	} else {
		printf("Successfully dumped context to disk\n");
	}



	//Disconnect from the accesspoints
	apdm_ctx_disconnect(apdm_context);

	//Free memory for the context
	apdm_ctx_free_context(apdm_context);

	return(0);
}

