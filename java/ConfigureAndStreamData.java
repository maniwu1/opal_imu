import java.io.File;
import java.util.List;
import com.apdm.APDMException;
import com.apdm.APDMNoMoreDataException;
import com.apdm.Context;
import com.apdm.Device;
import com.apdm.DockingStation;
import com.apdm.RecordRaw;
import com.apdm.swig.SWIGTYPE_p_hid_t;
import com.apdm.swig.apdm;
import com.apdm.swig.apdm_logging_level_t;
import com.apdm.swig.apdm_streaming_config_t;
public class ConfigureAndStreamData {
    public static void main(String args[]) throws Exception {
        Context.setLoggingLevel(apdm_logging_level_t.APDM_LL_ERROR);
        String logFile = "./log.txt";
        apdm.apdm_set_log_file(logFile);
        setLabels();
        apAutoConfig();
        System.out.println("Please remove monitors from their docks and press enter...");
        System.in.read();
        streamAndRecordData();
    }

    public static void setLabels() throws APDMException {
        Context context = null;
        try {
            context = Context.getInstance();
            context.open();
            int nDocks = DockingStation.getNumAttached();
            for (int iDock=0; iDock<nDocks; iDock++) {
                DockingStation dock = null;
                Device sensor = null;
                try {
                    dock = DockingStation.openByIndex(iDock);
                    if (!dock.isMonitorPresent()) {
                        return;
                    }
                    sensor = dock.attachedDevice;
                    String caseId = sensor.cmd_get_device_case_id();
                    // For this example, just set the label to the case ID. This can be any string <= 15 charac
                    // More typically, the user will know which label they which to apply to which sensor based
                    // unique case ID on the back of the sensor, so a mapping may be used here to determine the
                    sensor.cmd_set_device_label(caseId);
                    sensor.cmd_config_commit();
                    System.out.println("Set label on sensor in dock " + (iDock+1) + " to " + caseId);
                } finally {
                    if (dock != null) {
                        dock.close();
                    }
                }
            }
        } catch (Exception ex) {
            System.out.println(ex.getMessage());
            System.out.println("Could not set labels on docked sensors");
        } finally {
            if (context != null) {
                context.close();
            }
        }
    }
    public static void apAutoConfig() throws Exception {
        Context context = null;
        try {
            context = Context.getInstance();
            context.open();
            apdm_streaming_config_t streamingConfig = new apdm_streaming_config_t();
            apdm.apdm_init_streaming_config(streamingConfig);
            context.autoConfigureDevicesAndAccessPointStreaming(streamingConfig);
            System.out.println("Done configuring system for wireless streaming");
        } finally {
            if (context != null) {
                context.close();
            }
        }
    }
    public static void streamAndRecordData() throws Exception {
        Context context = Context.getInstance();
        SWIGTYPE_p_hid_t hdfFileHandle = new SWIGTYPE_p_hid_t(0,false);
        try {
            context.open();
            // Create the HDF file to write to
            String filePath = "./recording.h5";
            hdfFileHandle = context.createHDFFile("recording.h5");
            File file = new File(filePath);

            if (SWIGTYPE_p_hid_t.getCPtr(hdfFileHandle) == 0 || !file.exists()) {
                throw new Exception("Could not create file at: " + filePath);
            }
            System.out.println("File created at " + file.getAbsolutePath());
            int min_latency_seconds = 0;
            int max_latency_seconds = 0xffff;
            // Set the max latency to something small to flush
            // any existing data buffered on the monitors
            context.setMaxLatency(min_latency_seconds);
            // Wait a little bit to give the monitors time to receive
            // the command and process it.
            Thread.sleep(3000);
            // Set the max latency time back to something big.
            // This will force the AP to process old packets if the sensors
            // go out of range of the AP
            context.setMaxLatency(max_latency_seconds);
            // Wait a little bit for the max latency command to take effect
            Thread.sleep(3000);
            // Sync the record head list. This is a method in the host
            // libraries which waits to correlate data from all streaming devices
            // before emitting correlated sets of data.
            context.syncRecordHeadList();
            // Call this many times to stream data
            int sampleNum = 0;
            for (int i = 0; i < 1000; i++) {
                List<RecordRaw> records = null;
                try {
                    records = context.getNextRecordList();
                } catch (APDMNoMoreDataException ex) {
                    // No data found, so wait just a bit for data to become available.
                    Thread.sleep(100);
                }
                // The list of records will be empty if no data was found.
                if (!records.isEmpty()) {
                    boolean storeRaw = false;
                    boolean storeSI = true;
                    boolean storeFiltered = false;
                    boolean compress = true;
                    context.writeRecordToHDF(hdfFileHandle, sampleNum, storeRaw, storeSI, storeFiltered, compress);
                    for (RecordRaw rec : records) {
                        System.out.println(rec.toString());
                    }
                    sampleNum++;
                }
            }
        } finally {
            if (SWIGTYPE_p_hid_t.getCPtr(hdfFileHandle) != 0) {
                context.closeHDFFile(hdfFileHandle);
            }
            if (context != null) {
                context.close();
            }
        }
    }
}


