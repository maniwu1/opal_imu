#include "apdm.h"
#include <stdio.h>
#include <limits.h>
#include <time.h>
#include <inttypes.h> 
#include <stdlib.h>

/**
 * This application is used to stream data off a set of motion monitors and access points that have been configured in
 * wireless streaming mode (usually by a previously run autoconfigure() functionc call).
 *
 * It will create and initialize a context, then enter a loop to retrive and print correlated samples from the various
 * motion monitors in the system.
 */

#define NUM_SECONDS   30
#define NUM_INITIAL_AP_READS_TO_SKIP       1000

int main(int argc, char **argv)
{
	int num_records_to_collect = -1;
	int num_streaming_sessions = 1;

	for(int i = 0; i < argc; i++ ) {
		printf("Argument %d is '%s'\n", i, argv[i]);
	}
	if( argc > 1 ) {
		num_records_to_collect = atoi(argv[1]);
	}
	if( argc > 2 ) {
		num_streaming_sessions = atoi(argv[2]);
	}

	apdm_log("Starting test application...\r\n");
	uint32_t gpio_toggle_output = 0;

	uint64_t sync_box_set_time_epoch_ms = 0;
	uint64_t sync_box_set_time_sync_value = 0;


	for (uint32_t k=1; k <= num_streaming_sessions; k++) {
		// This is the number of reads off of the AP to attempt. There is not necessarily
		// data available with each read, so the total number of samples gathered will
		// be less than or equal to this number.
    const int32_t num_iterations = 100000;
		int r = 0;
	
		apdm_ctx_t apdm_context = ADPM_DEVICE_COMMUNICATIONS_HANDLE_NEW_T_INITIALIZER;
		apdm_log_info("Allocating apdm context\n");
		apdm_context = apdm_ctx_allocate_new_context();

		if( apdm_context == NULL ) {
			printf("ERROR: Failed to allocate communications handle\n");
			return(1);
		}
	
		r = apdm_ctx_open_all_access_points(apdm_context);
		if( r != APDM_OK ) {
			printf("Failed to open access point, exiting...%d, %s\n", r, apdm_strerror(r));
			return(1);
		}

		const uint32_t num_aps_found = apdm_ctx_get_num_access_points_found(apdm_context);

		uint32_t ap_id_with_sync_box = 0;
		for(int i = 0; i < num_aps_found; i++ ) {
			uint32_t apid = 0;
			if( (r = apdm_ctx_get_ap_id_for_ap_index(apdm_context, i, &apid)) != APDM_OK ) {
				printf("ERROR: Unable to get %d AP ID\n", i);
				return(1);
			}

			uint32_t attached = 0;
			if( (r = apdm_ctx_ap_sync_box_attached(apdm_context, apid, &attached)) != APDM_OK ) {
				printf("ERROR: Unable to check if sync box is attached on AP %u\n", apid);
				return(1);
			} else {
			  printf("Sync box attached flag = %u, to AP ID %u\n", attached, apid);
			}

			if( attached ) {
				ap_id_with_sync_box = apid;

				uint32_t fw_ver = 0;
				if( (r = apdm_ctx_ap_sync_box_query(apdm_context, ap_id_with_sync_box, CAN_QUERY_CMD_GET_FIRMWARE_VERSION, &fw_ver)) != APDM_OK ) {
					printf("ERROR: Unable to query sync box firmware version: %d", r);
					return(1);
				}

				printf("Sync Box firmware version is %u\n", fw_ver);

				break;
			}
		}

		printf("AP ID with sync box attached is %u\n", ap_id_with_sync_box);

	
		//Set the maximum delay for data, the maximum time the application is willing to wait for data from a given sensor to come in.
		r = apdm_ctx_set_max_sample_delay_seconds(apdm_context, APDM_DEFAULT_MAX_LATENCY_SECONDS);
		if( r != APDM_OK ) {
			printf("Failed to set max sample delay\n");
			return(1);
		}

		//Configure the error handling mode that the libraries should use.
		r = apdm_ctx_set_error_handling_mode(apdm_context, APDME_EHB_RETURN_PARTIAL_RECORD);
		if( r != APDM_OK ) {
			printf("Failed to set error handling behavior\n");
			return(1);
		}
	
		//Fetch the list of monitor IDs
		uint32_t deviceIdList[APDM_MAX_NUMBER_OF_SENSORS];
		r = apdm_ctx_get_device_id_list(apdm_context, deviceIdList, APDM_MAX_NUMBER_OF_SENSORS);
		if( r != APDM_OK ) {
			printf("ERROR: Unable to get device id list from access point library...\n");
			return(-1);
		}

		//Configure user-specified per-device meta-data.
		for(int i = 0; i < APDM_MAX_NUMBER_OF_SENSORS; i++ ) {
			printf("Device index %u id is %d\n", i, deviceIdList[i]);
			apdm_ctx_set_metadeta_uint32(apdm_context, deviceIdList[i], 100 + deviceIdList[i]);
		}
	
		//Define the record into which sensor data is to be stored.
		apdm_record_t raw_rec;
	
		apdm_usleep(1000000);
	
		uint32_t records_counter = 0;
		uint32_t total_records_counter = 0;
		uint32_t total_omited_records = 0;
		uint32_t total_omited_sets = 0;
		uint32_t total_retrys = 0;
	
		struct timeval start_time;
		struct timeval now_time;
		struct timeval end_time;
		time_t st_time = apdm_get_time_ms_64(&start_time);
	
		int i;
		uint32_t lastSync = 0;
	
		uint32_t sampleRetryCountBySecond[NUM_SECONDS];
		for(i = 0; i < NUM_SECONDS; i++ ) {
			sampleRetryCountBySecond[i] = 0;
		}
	
		uint64_t min_sync_value = 0;
		min_sync_value = (apdm_ctx_estimate_now_sync_value(apdm_context));
		min_sync_value += (rand() % 50000);//Add some randomization to the cycle

		if( (r = apdm_ctx_set_minimum_sync_value(apdm_context, min_sync_value)) != APDM_OK ) {
			printf("Unable to set minimum sync value BEFORE setting up streaming, r=%d %s\n", r, apdm_strerror(r));
			return(-1);
		} else {
			printf("Successfully set minimum sync value to %"PRIu64"\n", min_sync_value);
		}

#if 0
		apdm_usleep(1500000);
		if( (r = apdm_ctx_set_max_sample_delay_seconds(apdm_context, 5)) != APDM_OK ) {
			printf("Unable to set max latency, r=%d %s\n", r, apdm_strerror(r));
			return(-1);
		}
#endif

		apdm_usleep(1500000);


		printf("Calling apdm_sync_record_list_head...\n");
		const uint64_t ms_start = apdm_get_time_ms_64(NULL);
		r = apdm_ctx_sync_record_list_head(apdm_context);
		if( r != APDM_OK ) {
			apdm_ctx_disconnect(apdm_context);
			printf("Error syncing record head list, r=%d %s\n", r, apdm_strerror(r));
			return(-1);
		}
		const uint64_t ms_stop = apdm_get_time_ms_64(NULL);
		printf("Done syncing record head list... It took %llu ms\n", (ms_stop - ms_start));



		apdm_usleep(1500000);
		if( (r = apdm_ctx_set_max_sample_delay_seconds(apdm_context, APDM_DEFAULT_MAX_LATENCY_SECONDS)) != APDM_OK ) {
			printf("Unable to set max latency, r=%d %s\n", r, apdm_strerror(r));
			return(-1);
		}
		apdm_usleep(1500000);


#if 0
		//min_sync_value = apdm_ctx_estimate_now_sync_value(apdm_context);
		min_sync_value = (apdm_ctx_estimate_now_sync_value(apdm_context) - (5 * 2650));
		if( (r = apdm_ctx_set_minimum_sync_value(apdm_context, min_sync_value)) != APDM_OK ) {
			printf("Unable to set minimum sync value while setting up streaming, r=%d %s\n", r, apdm_strerror(r));
			return(-1);
		} else {
			printf("Successfully set minimum sync value...\n");
		}
#endif


		r = 0;
		int j = 0;

		int num_sensors = 0;
		time_t lastTime = apdm_get_time_ms_64(NULL);
		for(int j = 0; j < APDM_MAX_NUMBER_OF_SENSORS; j++ ) {
			if( deviceIdList[j] == 0 ) {
				continue;
			}
			num_sensors++;
		}

		uint16_t expected_sync_delta = 0;
		if( (r = apdm_ctx_get_expected_sync_delta(apdm_context, &expected_sync_delta)) != APDM_OK ) {
			//FIXME handle this
		}
        
		const bool write_hdf = false;
		// Save a file with a different name for each iteration
		char fileName[60] = "";
		sprintf(fileName, "./stream_data_out_%u.h5", k);
		hid_t file_handle = apdm_ctx_create_file_hdf(fileName, apdm_context);
		if( file_handle < 0 ) {
			printf("ERROR: Unable to open file %s\n", fileName);
			return(1);
		}
		
		int sampleNumber = 0;

		uint64_t last_v2_sync_value = 0;
		uint64_t v2_sync_delta = 0;

		int64_t last_v1_sync_value = 0;
		int64_t v1_sync_delta = 0;

		int64_t skip_read_counter = 0;        


		for (i = 0; i < num_iterations; i++) {
			//Log the context details from time-to-time, helpful for debugging
			if( i % 5000 == 0 ) {
				apdm_log_context(apdm_context, APDM_LL_ERROR);
			}

			if( i == (num_iterations / 2) ) {
				min_sync_value = (apdm_ctx_estimate_now_sync_value(apdm_context));
				min_sync_value += (rand() % 50000);//Add some randomization to the cycle

				if( (r = apdm_ctx_set_minimum_sync_value(apdm_context, min_sync_value)) != APDM_OK ) {
					printf("Unable to set minimum sync value BEFORE setting up streaming, r=%d %s\n", r, apdm_strerror(r));
					return(-1);
				} else {
					printf("Successfully set minimum sync value to %"PRIu64"\n", min_sync_value);
				}

			}

			if( skip_read_counter <= NUM_INITIAL_AP_READS_TO_SKIP) {
				//ignore the first 1000 reads off of the AP for the purpose of transfer rate calculation due to buffering
				skip_read_counter++;
				st_time = apdm_get_time_ms_64(&start_time);
				records_counter = 0;
			} 
		
			if( i % 500 == 0 ) {
				//Dump the AP debug information to the library log, helpful for debugging
				apdm_ctx_get_all_ap_debug_info(apdm_context);
			}

#if 1
			if( ap_id_with_sync_box != 0 && i > 255 && (i  % 32) == 0 ) {
			  sync_box_set_time_epoch_ms = apdm_get_time_ms_64(NULL);
			  sync_box_set_time_sync_value = apdm_ctx_estimate_now_sync_value(apdm_context);

				if( (r = apdm_ctx_ap_set_io_value(apdm_context, ap_id_with_sync_box, APDM_AP_ANALOG_OUT_0, gpio_toggle_output)) != APDM_OK ) {
					printf("ERROR: unable to set AP io value...\r\n");
					//exit(-1);
				} else {
					printf("Successfully set AP io value to %u...\r\n", gpio_toggle_output);
				}
				gpio_toggle_output = (gpio_toggle_output ? 0 : 1);
			}
#endif

			//Retrieve any ranging data from the context
			for(int j = 0; j < 10; j++ ) {
				apdm_ranging_sample_t d;
				r = apdm_ctx_get_next_ranging_record(apdm_context, &d);
				if( r == APDM_OK ) {
					printf(	"Got Ranging sample: sync_time=%"PRIu64", source ID: %u, remote ID: %u, anchor timestamp: %" PRIu64 "\n",
							d.wireless_sync_time_us, d.source_device_id,
							d.remote_device_id, d.anchor_timestamp);

				} else if( r == APDM_NO_MORE_DATA ) {
					break;
				} else {
					printf("Got error while retrieveing ranging record, r=%d %s\n", r, apdm_strerror(r));
				}
			}



			//Retrieve any external synchronization events that the AP has received.
			apdm_external_sync_data_t sync_data;
			r = apdm_ctx_get_next_synchronization_event(apdm_context, &sync_data);
			if( r == APDM_OK ) {
			  uint64_t now_sync_box_set_time_epoch_ms = apdm_get_time_ms_64(NULL);

			  uint64_t sync_time_delta = sync_data.sync_value - sync_box_set_time_sync_value;
			  uint64_t sync_value_delta_ms = apdm_epoch_access_point_to_epoch_millisecond(sync_time_delta);

				printf("Got a external synchronization event: data = 0x%X, data_type = 0x%X, sync_value = %" PRId64 ", ap_id=%u, v2_pin=%u, reflection_time=%"PRIu64"ms, sync_value_delta_ms=%"PRIu64"\n",
						sync_data.data,
						sync_data.data_type,
						sync_data.sync_value,
						sync_data.ap_id,
						sync_data.v2_pin,
						(now_sync_box_set_time_epoch_ms - sync_box_set_time_epoch_ms),
						sync_value_delta_ms);

			} else if( r == APDM_NO_MORE_DATA ) {
				//ignore
			} else {
				//an error occured
			}

			apdm_button_data_t bed;
			r = apdm_ctx_get_next_button_event(apdm_context, &bed);
			if( r == APDM_OK ) {
			  printf("Streaming got button event: device_id=%u, button_event=%u (%s), event_sync_time=%"PRIu64", stm32_time_us=%"PRIu64", unix_epoch_second=%"PRIu64", sync_val64=%"PRIu64"\n",
			                bed.device_button_data.device_id,
			                bed.device_button_data.button_event,
			                button_event_t_to_str(bed.device_button_data.button_event),
			                bed.device_button_data.event_sync_time,
			                bed.device_button_data.stm32_time_us,
			                bed.unix_epoch_second,
			                bed.sync_val64
			                );

      } else if( r == APDM_NO_MORE_DATA ) {
        //ignore
      } else {
        //an error occured
      }


			/*
			//Toggle the sync output line on the access point once per second
			const int access_point_id = ####;
			const int gpio_output_value = (apdm_get_time_ms_64(NULL) / 1000) % 2;
			apdm_ctx_ap_set_gpio_value(apdm_context, access_point_id, APDM_AP_GPIO_0, gpio_output_value);
			*/

			//Request the next full set of samples from the AP. All samples returned will be
			//from the same point in time for all sensors configured in the system.
			r = apdm_ctx_get_next_access_point_record_list(apdm_context);


			if( r == APDM_NO_MORE_DATA ) {
				//The host libraries have not received a full set of data, wait a while for more data to stream in from the monitors.
				printf("INFO: Waiting in main() as there is no more data in the fifos...\n");
				apdm_usleep(4000);//Note: this is a sensitive number while in rapid streaming mode
				continue;
			} else if( r != APDM_OK ) {
				printf("ERROR: apdm_get_next_access_point_record_list return an unexpected error %d, '%s'\n", r, apdm_strerror(r));
				apdm_ctx_get_all_ap_debug_info(apdm_context);
				break;
			} else {
				//Successfully got a set of monitor samples.
				//printf("Got more data, r = %d...\n", r);
			}
		
			printf("=========================================================================================\n");
			printf("r = %d, sampleNumber = %d\n", r, sampleNumber);


			// Write the retrieved record (one synchronized data point per device) to the open HDF file.
			bool store_raw = false;
			bool store_si = true;
			bool store_filtered = false;
			bool compress = false;

			if( write_hdf ) {
        if( (apdm_ctx_write_record_hdf(file_handle, apdm_context, sampleNumber, store_raw, store_si, store_filtered, compress)) != APDM_OK ) {
          printf("ERROR: Bad return value when writing record, %d %s.\n", r, apdm_strerror(r));
          break;
        }
			}
			sampleNumber++;
		
			const uint64_t nowSyncValue = apdm_ctx_estimate_now_sync_value(apdm_context);
			const uint64_t nowSyncValueV2 = apdm_ctx_estimate_now_sync_value_v2(apdm_context);
		
			total_omited_records += apdm_ctx_get_num_omitted_samples(apdm_context);
			total_omited_sets += apdm_ctx_get_num_omitted_sample_sets(apdm_context);

			int32_t wireless_reliability[APDM_MAX_NUMBER_OF_SENSORS];
			uint32_t rssi_values[APDM_MAX_NUMBER_OF_SENSORS];
			int32_t avg_retrys[APDM_MAX_NUMBER_OF_SENSORS];

			printf("Estimated Now Sync Value is: %"PRIu64"\r\n", nowSyncValue);
			printf("Estimated Now Sync V2 Value is: %"PRIu64"\r\n", nowSyncValueV2);

			for(int j = 0; j < APDM_MAX_NUMBER_OF_SENSORS; j++ ) {
				if( deviceIdList[j] == 0 ) {
					continue;
				}

        if( false ) {
          apdm_streaming_status_t ss;
          if( apdm_ctx_get_streaming_status(apdm_context, deviceIdList[j], &ss) == APDM_OK ) {
            printf("Streaming Status: device_id=%u, battery_millivolts=%u, wireless_percent=%u, battery_percent=%u\n",
                ss.ap_sensor_status_data.device_id,
                ss.ap_sensor_status_data.battery_millivolts,
                ss.ap_sensor_status_data.wireless_percent,
                ss.ap_sensor_status_data.battery_percent);
          }
        }

		
				//Retrieve wireless reliability statistics on a per-device basis from the library
				wireless_reliability[j] = apdm_ctx_get_wireless_reliability_value(apdm_context, deviceIdList[j]);
				apdm_ctx_get_rssi_value(apdm_context, deviceIdList[j], &rssi_values[j]);
				avg_retrys[j] = apdm_ctx_avg_retry_count_for_device(apdm_context, deviceIdList[j]);
			
			
				//Get the sensor data for the given device ID
				int ret = apdm_ctx_extract_data_by_device_id(apdm_context, deviceIdList[j], &raw_rec);

				if( j == 0 ) {
					v2_sync_delta = raw_rec.v2_sync_val64_us - last_v2_sync_value;
					last_v2_sync_value = raw_rec.v2_sync_val64_us;

					v1_sync_delta = raw_rec.sync_val64 - last_v1_sync_value;
					last_v1_sync_value = raw_rec.sync_val64;
				}
			
				if( ret != APDM_OK ) {
					if( ret == APDM_NO_MORE_DATA ) {
						//Depending on the error handling mode, this monitor may or may not have data for it.
						printf("No More data for device id %d...\n", deviceIdList[j]);
					}
					continue;
				}
			
				records_counter++;	
				total_records_counter++;
                
			
				if( raw_rec.opt_select == MONITOR_OPT_SELECT_MODULE_ID ) {
					printf("Got opt-select module id as %u\n", raw_rec.optional_data);
				}
				if( raw_rec.accl_isPopulated ) {
					if( raw_rec.sync_val32_low >= lastSync && (raw_rec.sync_val32_low - lastSync) > expected_sync_delta ) {
						printf("1-ERROR: ");
					} else if( (raw_rec.sync_val32_low < lastSync) && ((UINT32_MAX - lastSync) + raw_rec.sync_val32_low) > expected_sync_delta ) {
						printf("2-ERROR: ");
					}
				
					lastSync = raw_rec.sync_val32_low;
					//const uint64_t full_sync_value = raw_rec.sync_val64;


					//Calcualte the age in milliseconds of the sample we just retrieved
					uint32_t sample_age_ms = apdm_calculate_sync_value_age(nowSyncValue, raw_rec.sync_val64);

					if( apdm_ctx_get_num_omitted_samples(apdm_context) > 0 ) {
						printf("ERROR: ");
					}
				
				
					sampleRetryCountBySecond[0] += raw_rec.num_retrys;
					time_t nowTime = apdm_get_time_ms_64(NULL);
					if( lastTime != nowTime ) {
						lastTime = nowTime;
						for(int q = NUM_SECONDS-1; q > 0 ; q-- ) {
							sampleRetryCountBySecond[q] = sampleRetryCountBySecond[q - 1]; 
						}
						sampleRetryCountBySecond[0] = 0;
					}
				
					total_retrys += raw_rec.num_retrys;

					const int64_t time_delta = ((nowTime - st_time)/1000);
					double retrys_per_minute = 0;
					if( time_delta > 0 ) {
						retrys_per_minute = 60.0 * ((double) total_retrys) / ((double) time_delta);
					}
					if( num_sensors <= 12 ) {//Printing is CPU intensive and can cause the age to rise
						printf("raw, ");
						printf("ap#=%d, ", raw_rec.source_ap_index);
						printf("nOmitedR=%u, ", apdm_ctx_get_num_omitted_samples(apdm_context));
						printf("tOmitedR=%u, ", total_omited_records);
						printf("tOmitedL=%u, ", total_omited_sets);
						printf("tOmitedS=%u, ", apdm_ctx_get_total_omitted_samples(apdm_context));
						printf("tRetrys=%u, ", total_retrys);
						printf("retrys/Min=%f, ", retrys_per_minute);
						printf("rtry=%u, ", raw_rec.num_retrys);
					}
					printf("age_ms=%u, ", sample_age_ms);
					if( num_sensors <= 12 ) {//Printing is CPU intensive and can cause the age to rise
#ifdef M64
						printf("nowSyncValue=%lu, ", (long unsigned int) nowSyncValue);
						printf("fullSyncVal=%lu, ", (long unsigned int) full_sync_value);
#else
						printf("nowSyncValue=%" PRIu64 ", ", (uint64_t) nowSyncValue);
						printf("sync_val64=%" PRIu64 ", ", (uint64_t) raw_rec.sync_val64);
						printf("nowV2SyncVal=%" PRIu64 ", ", (uint64_t) nowSyncValueV2);
						printf("V2SyncDelta=%" PRIu64 ", ", (uint64_t) v2_sync_delta);
						printf("V1SyncDelta=%" PRId64 ", ", (int64_t) v1_sync_delta);
#endif

						//printf("sync32low=%u, ", raw_rec.sync_val32_low);
						printf("v2_sync=%"PRIu64", ", raw_rec.v2_sync_val64_us);
						printf("v2_mcu_time=%"PRIu64", ", raw_rec.v2_mcu_time_val64_us);
						//printf("pipe=%d, ", raw_rec.nRF_pipe);
						printf("serial=%d, ", raw_rec.device_info_serial_number);
						printf("accl=%u, %u, %u, ", raw_rec.accl_x_axis, raw_rec.accl_y_axis, raw_rec.accl_z_axis);
						printf("gyro=%u, %u, %u, ", raw_rec.gyro_x_axis, raw_rec.gyro_y_axis, raw_rec.gyro_z_axis);
						printf("mag=%u, %u, %u, ", raw_rec.mag_x_axis, raw_rec.mag_y_axis, raw_rec.mag_z_axis);
						printf("flags=0x%X, ", raw_rec.flags);
						printf("optData=%u, ", raw_rec.optional_data);
						printf("button=%u, ", raw_rec.button_status);
						printf("sync_lock=%u, ", ((raw_rec.flags & MONITOR_DATA_FLAG_SYNC_LOCK ? 1 : 0)));
						printf("sync_reset=%u, ", ((raw_rec.flags & MONITOR_DATA_FLAG_SYNC_RESET ? 1 : 0)));


						//The monitor will occasionally send out tag data, for example, if it's external button is pressed.
						if( raw_rec.opt_select == MONITOR_OPT_SELECT_TAG_DATA ) {
							printf("tag=0x%X, ", raw_rec.optional_data);
						}

						if( raw_rec.temperature_isPopulated ) {
							printf("Temp=%f,%f, ", raw_rec.temperature, raw_rec.temperature_si);
						}

						printf("si, ");
						printf("%.3f, %.3f, %.3f, ", raw_rec.accl_x_axis_si, raw_rec.accl_y_axis_si, raw_rec.accl_z_axis_si);
						printf("%.3f, %.3f, %.3f, ", raw_rec.gyro_x_axis_si, raw_rec.gyro_y_axis_si, raw_rec.gyro_z_axis_si);
						printf("%.3f, %.3f, %.3f, ", raw_rec.mag_x_axis_si, raw_rec.mag_y_axis_si, raw_rec.mag_z_axis_si);
						printf("%.3fPa, ", raw_rec.pressure_si);
						printf("%.3fC, ", raw_rec.temperature_si);
					}
					printf("\n");
				}
			}




			if( num_sensors <= 12 ) {//Printing is CPU intensive and can cause the age to rise
				uint32_t streaming_status = 0;
				//Retrive the status of wireless streaming, will be a value from the enum apdm_ap_wireless_streaming_status_t
				if( (r = apdm_ctx_get_wireless_streaming_status(apdm_context, &streaming_status)) != APDM_OK ) {
					return(r);
				}

				printf("Current wireless streaming status is %u %s\n", streaming_status, apdm_ap_wireless_streaming_status_t_str(streaming_status));


				printf("Wireless: ");
				for(int j = 0; j < APDM_MAX_NUMBER_OF_SENSORS; j++ ) {
					if( deviceIdList[j] == 0 ) {
						continue;
					}
					printf("%u: %d, %d, %u   ", deviceIdList[j], wireless_reliability[j], avg_retrys[j], rssi_values[j]);
				}
				printf("\n");

				double retryTotalBySecond = 0;
				for(int q = 1; q < NUM_SECONDS; q++ ) {
					retryTotalBySecond += sampleRetryCountBySecond[q];
				}
				const double retrysPerSecond = retryTotalBySecond / ((double) (NUM_SECONDS - 1));

				apdm_get_time_ms_64(&end_time);
				int64_t diffUsecs = (end_time.tv_sec - start_time.tv_sec - 1) * 1000000;
				diffUsecs += end_time.tv_usec;
				diffUsecs += 1000000 - start_time.tv_usec;
				double samplesPerSec = (((double) records_counter) / ((double) diffUsecs/1000000.0));
				printf("It took %d seconds to read %d samples, %f / sec, retrys/sec = %f\n", ((uint32_t) diffUsecs/1000000), records_counter, samplesPerSec, retrysPerSecond);
				printf("totalOmitedRecords = %d\n", total_omited_records);
			}
		
			// If the command line option was set to only collect a certain number of records
			// and this number of records has been collected, terminate.	
			if (num_records_to_collect > 0 && total_records_counter >= num_records_to_collect) {
				break;
			}
		}

		apdm_get_time_ms_64(&end_time);
		int64_t diffUsecs = (end_time.tv_sec - start_time.tv_sec - 1) * 1000000;
		diffUsecs += end_time.tv_usec;
		diffUsecs += 1000000 - start_time.tv_usec;
		double samplesPerSec = (((double) records_counter) / ((double) diffUsecs/1000000.0));

		printf("It took %d seconds to read %d samples, %f / sec\n", ((uint32_t) diffUsecs/1000000), i, samplesPerSec);
		printf("A total of %d samples were collected\n", total_records_counter);

		apdm_ctx_disconnect(apdm_context);
		apdm_ctx_free_context(apdm_context);
   	apdm_close_file_hdf(file_handle);
	}
		
	apdm_exit();
	return(0);
}

