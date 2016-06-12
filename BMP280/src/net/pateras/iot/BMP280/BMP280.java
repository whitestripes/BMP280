package net.pateras.iot.BMP280;
import com.pi4j.io.i2c.I2CBus;
import com.pi4j.io.i2c.I2CDevice;
import com.pi4j.io.i2c.I2CFactory;
import com.pi4j.io.spi.SpiChannel;
import com.pi4j.io.spi.SpiDevice;
import com.pi4j.io.spi.SpiFactory;

/* This class provides an abstraction to easily interface the Raspberry Pi with the 
 * BMP280 Temperature and Pressure sensor by BOSCH via I2C and SPI protocols.
 * 
 * Copyright 2016 - By James Pateras
 * jamespateras@gmail.com
 */
public class BMP280
{
	/* The 7-bit device address is 111011x. The 6 MSB bits are fixed. The last bit is changeable by
	 * SDO value and can be changed during operation. Connecting SDO to GND results in slave address
	 * 1110110 (0x76); connection it to VDDIO results in slave address 1110111 (0x77) */
	public static final int ADDR_SDO_2_GND = 0x76;
	public static final int ADDR_SDO_2_VDDIO = 0x77;
	
	/* Protocol offerings */
	public enum Protocol {I2C, SPI}
	
	/* Compensation (trim) value positions in compensation array */
	public static final int DIG_T1 = 0;
	public static final int DIG_T2 = 1;
	public static final int DIG_T3 = 2;
	public static final int DIG_P1 = 3;
	public static final int DIG_P2 = 4;
	public static final int DIG_P3 = 5;	
	public static final int DIG_P4 = 6;	
	public static final int DIG_P5 = 7;	
	public static final int DIG_P6 = 8;	
	public static final int DIG_P7 = 9;	
	public static final int DIG_P8 = 10;	
	public static final int DIG_P9 = 11;
	
	/* Memory address which represents the start of trim values on device */
	public static final int TRIM_MEM_START_ADDR = 0x88;
	
	/* Memory address which represents the start of sensor values on device */
	public static final int SENSOR_START_ADDR = 0xF7;	
	
	/* Memory address which represents the start of sensor values on device */
	public static final int CONFIG_START_ADDR = 0xF4;
	public static final int CONFIG_OVERSAMPLE_ADDR = 0xF4;
	public static final int CONFIG_WAIT_AND_FILTER_ADDR = 0xF5;
	
	/* Array position values in result array to get temperature and pressure */
	public static final int TEMP_VAL_C = 0;
	public static final int TEMP_VAL_F = 1;
	public static final int PRES_VAL = 2;
	
	/* The value the device will return to code if reading was skipped */
	public static final int SKIP_VAL = 0x80000;
	
	/* The value this library will return when a reding is skipped */
	public static final int DEFAULT_SKIP_VAL_TEMP = 0;
	public static final int DEFAULT_SKIP_VAL_PRES = 0;
	
	/* ----------- Sensor Operation Mode -----------
	 * SLEEP	- Sleep mode is set by default after power on reset. In sleep mode, no 
	 * 	          measurements are performed and power consumption is at a minimum. All 
	 *            registers are accessible; Chip-ID and compensation coefficients can be read. 
	 *            
	 * NORMAL	- Normal mode continuously cycles between an (active) measurement period and an
	 *            (inactive) standby period, whose time is configurable. After setting the 
	 *            mode, measurement and filter options, the last measurement results can be 
	 *            obtained from the data registers without the need of further write accesses. 
	 *            Normal mode is recommended when using the IIR filter, and useful for applications 
	 *            in which short-term disturbances (e.g. blowing into the sensor) should be filtered.
	 * 
	 * FORCE	- In forced mode, a single measurement is performed according to selected 
	 * 			  measurement and filter options. When the measurement is finished, the sensor 
	 *            returns to sleep mode and the measurement results can be obtained from the 
	 *            data registers. For a next measurement, forced
	 */	
	public enum Mode {
		SLEEP((byte)0x00), NORMAL((byte)0x03), FORCE((byte)0x01);
		
		private byte value;
 
		private Mode(byte value) {
			this.value = value;
		}
	}	

	/* ----------- Standby Time (for NORMAL mode) -----------	
	 * Time in milliseconds to wait between samples in normal mode
	 * Valid options are 0.5ms, 62.5ms, 125ms, 250ms, 500ms, 1000ms, 2000ms, 4000ms
	 */
	public enum Standby_Time {
		MS_POINT_5((byte)0x00), MS_62_POINT_5((byte)0x01), MS_125((byte)0x02), 
		MS_250((byte)0x03), MS_500((byte)0x04), MS_1000((byte)0x05), 
		MS_2000((byte)0x06), MS_4000((byte)0x07);
		
		private byte value;
 
		private Standby_Time(byte value) {
			this.value = value;
		}
	}	
	
	/* ----------- Pressure Settings -----------
	 * Pressure measurement can be enabled or skipped. Skipping the measurement could be useful
	 * if BMP280 is used as a temperature sensor only. When enabled, several oversampling options exist.
	 * Each oversampling step reduces noise and increases the output resolution by one bit
	 * Each sample consumes at most 2.3ms... ie x16 sampling will consume 36.8ms

		|-----------------------------------------------------------------------|
		|  Oversampling  |  Pressure      |  Typical pressure  |  Recommended   | 
		|  setting       |  oversampling  |  resolution        |  temperature   |
		|                |                |                    |  oversampling  | 
		|----------------+----------------+--------------------+----------------|
		|  Pressure      |  Skipped       |                    |                |
		|  measurement   |  (output set   |         NA         |  As needed     |
		|  skipped       |  to 0x80000)   |                    |                |
		|----------------+----------------+--------------------+----------------| 
		|  Ultra low     |      x1        |  16 bit / 2.62 Pa  |      x1        | 
		|  power         |                |                    |                |
		|----------------+----------------+--------------------+----------------|
		|  Low Power     |      x2        |  17 bit / 1.31 Pa  |      x1        |
		|----------------+----------------+--------------------+----------------|
		|  Standard      |      x4        |  18 bit / 0.66 Pa  |      x1        |
		|  resolution    |                |                    |                |
		|----------------+----------------+--------------------+----------------|
		|  High          |      x8        |  19 bit / 0.33 Pa  |      x1        |
		|  Resolution    |                |                    |                |
		|----------------+----------------+--------------------+----------------|
		|  Ultra high    |      x16       |  20 bit / 0.16     |      x2        |
		|  resolution    |                |                    |                |
		|-----------------------------------------------------------------------|
	 */
	public enum Pressure_Sample_Resolution {
		SKIP((byte)0x00), ONE((byte)0x01), TWO((byte)0x02), FOUR((byte)0x03), EIGHT((byte)0x04), SIXTEEN((byte)0x05);
		
		private byte value;
 
		private Pressure_Sample_Resolution(byte value) {
			this.value = value;
		}
	}	
	
	/* ----------- Temperature Settings -----------
	 * 	Temperature measurement can be enabled or skipped. Skipping the measurement could be
	 * useful to measure pressure extremely rapidly. When enabled, several oversampling options
	 * exist. Each oversampling step reduces noise and increases the output resolution by one bit, 	
	 * Each sample consumes at most 2.3ms... ie x16 sampling will consume 36.8ms
	 
		|-------------------------------------|
		|  Temperature   |  Typical           |
		|  oversampling  |  temperature       |
		|                |  resolution        |
		|----------------|--------------------|
		|  Skipped       |                    |
		|  (output set   |         NA         |
		|  to 0x80000    |                    |
		|----------------|--------------------|
		|       x1       | 16 bit / 0.0050 °C |
		|----------------|--------------------|
		|       x2       | 17 bit / 0.0025 °C |
		|----------------|--------------------|
		|       x4       | 18 bit / 0.0012 °C |
		|----------------|--------------------|
		|       x8       | 19 bit / 0.0006 °C |
		|----------------|--------------------|
		|       x16      | 20 bit / 0.0003 °C |
		|-------------------------------------|
	 */
	public enum Temperature_Sample_Resolution {
		SKIP((byte)0x00), ONE((byte)0x01), TWO((byte)0x02), FOUR((byte)0x03), EIGHT((byte)0x04), SIXTEEN((byte)0x05);
		
		private byte value;
 
		private Temperature_Sample_Resolution(byte value) {
			this.value = value;
		}
	}	
	
	/* ----------- IIR Filter -----------
	 * The environmental pressure is subject to many short-term changes, caused e.g. by slamming of
	 * a door or window, or wind blowing into the sensor. To suppress these disturbances in the output
	 * data without causing additional interface traffic and processor work load, the BMP280 features
	 * an internal IIR filter

		|-------------------------------------------------|
		|             RMS Noise in preassure [Pa]         |
		|-------------------------------------------------|
		|  Pressure      |      IIR filter coefficient    |
		|  oversampling  |--------------------------------|
		|                |  Off  |  2  |  4  |  8  |  16  |
		|----------------+-------+-----+-----+-----+------|
		|  Skipped       |       |     |     |     |      |
		|  (output set   |  3.3  | 1.9 | 1.2 | 0.9 | 0.4  |
		|  to 0x80000)   |       |     |     |     |      |
		|----------------+-------+-----+-----+-----+------|
		|      x2        |  2.6  | 1.5 | 1.0 | 0.6 | 0.4  |
		|----------------+-------+-----+-----+-----+------|
		|      x4        |  2.1  | 1.2 | 0.8 | 0.5 | 0.3  |
		|----------------+-------+-----+-----+-----+------|
		|      x8        |  1.6  | 1.0 | 0.6 | 0.4 | 0.2  |
		|----------------+-------+-----+-----+-----+------|
		|      x16       |  1.3  | 0.8 | 0.5 | 0.4 | 0.2  |
		|-------------------------------------------------|
	 */
	public enum IIRFilter {
		OFF((byte)0x00), TWO((byte)0x01), FOUR((byte)0x02), EIGHT((byte)0x03), SIXTEEN((byte)0x04);
		
		private byte value;
 
		private IIRFilter(byte value) {
			this.value = value;
		}
	}	
	
	/* ----------- Start of local variables ----------- */
	
	/* The communication protocol being used */
	private Protocol protocol;
	
	/* I2C specific variables */
	private I2CBus I2Cbus;
	private I2CDevice I2Cdevice;
	
	/* SPI specific variables */
	private SpiDevice SPIdevice;
	
	/* Register 0xF4 controls the sampling rate (resolution) for temperature and pressure
	 * as well as the mode the device runs in
	 * 
		|-------------------------------|
		|    Bit Map (0xF4 Register)    |
		|-------------------------------|
		| 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 |
		|   Temp    |  Pressure | Mode  |
		|-------------------------------|
	 
	 */
	private byte config_register_0xF4 = 0x00;
	
	/* Register 0xF5 controls the standby time (normal mode) and IIR filter
	 * 
		|-------------------------------|
		|    Bit Map (0xF5 Register)    |
		|-------------------------------|
		| 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 |
		|   Standby |    IIR    |       |
		|-------------------------------|
	 
	 */
	private byte config_register_0xF5 = 0x00;
	
	
	/* The device has trimming parameters programmed into non-volatile memory (NVM) during
	 * production because each sensor is slightly different. This array stores the 
	 * compensation values required to calculate temperature and pressure
	 */
	private int[] compensationVals = new int[12];
	
	/* ----------- End of local variables ----------- */	
	
	/* ----------- Start of Initialisation Methods ----------- */
	
	/* Method to load compesation values from device memory to correctly calculate
	 * temperature and pressure values. The values read are specific to the device
	 * and do not change.
	 */
	private void loadCompensationValues() throws Exception {
		byte[] b = readBytesFromDevice(TRIM_MEM_START_ADDR, 24);
		parseCompensationValues(b);
	}
	
	/* Method to format the compensation values into usable values */
	private void parseCompensationValues(byte[] b) {
		compensationVals[DIG_T1] = parse16BitUnsignedValue(b,0);
		compensationVals[DIG_T2] = parse16BitSignedValue(b,2);
		compensationVals[DIG_T3] = parse16BitSignedValue(b,4);
		compensationVals[DIG_P1] = parse16BitUnsignedValue(b,6);
		compensationVals[DIG_P2] = parse16BitSignedValue(b,8);
		compensationVals[DIG_P3] = parse16BitSignedValue(b,10);
		compensationVals[DIG_P4] = parse16BitSignedValue(b,12);
		compensationVals[DIG_P5] = parse16BitSignedValue(b,14);
		compensationVals[DIG_P6] = parse16BitSignedValue(b,16);
		compensationVals[DIG_P7] = parse16BitSignedValue(b,18);
		compensationVals[DIG_P8] = parse16BitSignedValue(b,20);
		compensationVals[DIG_P9] = parse16BitSignedValue(b,22);
	}
	
	private int parse16BitUnsignedValue(byte[] b, int startPos) {
		return (b[startPos] & 0xFF) + ((b[startPos+1] & 0xFF) << 8);
	}
	
	private int parse16BitSignedValue(byte[] b, int startPos) {
		int val = parse16BitUnsignedValue(b, startPos);
		if(val > 32767)
			val -= 65536;
		
		return val;
	}
	
	/* Probes the device's memory to obtain the current configuration it is running
	 * and updates the 0xF4 and 0xF5 register variables to reflect the values */
	private void loadCurrentConfigSettingsFromDevice() throws Exception {
		byte[] b = readBytesFromDevice(CONFIG_START_ADDR, 2);
		
		config_register_0xF4 = (byte)(b[0] & 0xFF);
		config_register_0xF5 = (byte)(b[1] & 0xFF);
	}
	
	/* ----------- End of Initialisation Methods ----------- */
	
	/* ----------- Start Of Configuration Methods ----------- */
	
	public byte getConfig_register_0xF4() {
		return config_register_0xF4;
	}

	public byte getConfig_register_0xF5() {
		return config_register_0xF5;
	}	
	
	public void setMode(Mode mode, boolean flush) throws Exception {
		config_register_0xF4 = (byte)(((config_register_0xF4 >> 2) << 2) + mode.value);
		
		if(flush)
			writeConfiguration();
	}

	public void setPressureSampleRate(Pressure_Sample_Resolution p, boolean flush) throws Exception {
		byte other_vals;
		byte mask = (byte)0b11100011;
		
		other_vals = (byte)(config_register_0xF4 & mask);
		config_register_0xF4 =  (byte) (other_vals | (p.value << 2));
		
		if(flush)
			writeConfiguration();
	}

	public void setTemperatureSampleRate(Temperature_Sample_Resolution t, boolean flush) throws Exception {
		byte other_vals;
		byte mask = (byte)0b00011111;
		
		other_vals = (byte)(config_register_0xF4 & mask);
		config_register_0xF4 =  (byte) (other_vals | (t.value << 5));

		if(flush)
			writeConfiguration();
	}
	
	public void setStandbyTime(Standby_Time t, boolean flush) throws Exception {
		byte other_vals;
		byte mask = (byte)0b00011111;
		
		other_vals = (byte)(config_register_0xF5 & mask);
		config_register_0xF5 =  (byte) (other_vals | (t.value << 5));	
		
		if(flush)
			writeConfiguration();
	}
	
	public void setIIRFilter(IIRFilter f, boolean flush) throws Exception {
		byte other_vals;
		byte mask = (byte)0b11100011;
		
		other_vals = (byte)(config_register_0xF5 & mask);
		config_register_0xF5 =  (byte) (other_vals | (f.value << 2));
		
		if(flush)
			writeConfiguration();
	}
	
	private boolean isDeviceSleeping() throws Exception {
		loadCurrentConfigSettingsFromDevice();
		
		if((config_register_0xF4 & 0b11) == 0)
			return true;
		
		return false;
	}
	
	/* ----------- End Of Configuration Methods ----------- */
	
	/* ----------- Start of Presets within BOSCH Documentation ----------- */
	
	/* 10Hz possible sample rate - Ultra High Resolution*/ 
	public void setHandheldDeviceLowPowerMode() throws Exception {
		setMode(Mode.NORMAL, false);
		setPressureSampleRate(Pressure_Sample_Resolution.SIXTEEN, false);
		setTemperatureSampleRate(Temperature_Sample_Resolution.TWO, false);
		setIIRFilter(IIRFilter.FOUR, false);
		setStandbyTime(Standby_Time.MS_62_POINT_5, true);
	}
	
	/* ~80Hz possible sample rate - Standard Resolution*/ 
	public void setHandheldDeviceDynamicMode() throws Exception {
		setMode(Mode.NORMAL, false);
		setPressureSampleRate(Pressure_Sample_Resolution.FOUR, false);
		setTemperatureSampleRate(Temperature_Sample_Resolution.ONE, false);
		setIIRFilter(IIRFilter.SIXTEEN, false);
		setStandbyTime(Standby_Time.MS_POINT_5, true);
	}	
	
	/* Ultra Low Power */
	public void setWeatherMonitoringMode() throws Exception {
		setMode(Mode.FORCE, false);
		setPressureSampleRate(Pressure_Sample_Resolution.ONE, false);
		setTemperatureSampleRate(Temperature_Sample_Resolution.ONE, false);
		setIIRFilter(IIRFilter.OFF, false);
		setStandbyTime(Standby_Time.MS_4000, true);
	}
	
	/* ~7Hz possible sample rate - Standard Resolution */
	public void setFloorChangeDetectionMode() throws Exception {
		setMode(Mode.NORMAL, false);
		setPressureSampleRate(Pressure_Sample_Resolution.FOUR, false);
		setTemperatureSampleRate(Temperature_Sample_Resolution.ONE, false);
		setIIRFilter(IIRFilter.FOUR, false);
		setStandbyTime(Standby_Time.MS_125, true);
	}
	
	/* 125Hz possible sample rate - Low Power */
	public void setDropDetectionMode() throws Exception {
		setMode(Mode.NORMAL, false);
		setPressureSampleRate(Pressure_Sample_Resolution.TWO, false);
		setTemperatureSampleRate(Temperature_Sample_Resolution.ONE, false);
		setIIRFilter(IIRFilter.OFF, false);
		setStandbyTime(Standby_Time.MS_POINT_5, true);
	}

	/* ~25Hz possible sample rate - Ultra High Resolution */
	public void setIndoorNavigationMode() throws Exception {
		setMode(Mode.NORMAL, false);
		setPressureSampleRate(Pressure_Sample_Resolution.SIXTEEN, false);
		setTemperatureSampleRate(Temperature_Sample_Resolution.TWO, false);
		setIIRFilter(IIRFilter.SIXTEEN, false);
		setStandbyTime(Standby_Time.MS_POINT_5, true);
	}
	
	/* ----------- End of Presets within BOSCH Documentation ----------- */

	/* ----------- Start of Runtime Methods ----------- */

	public BMP280(Protocol protocol, int deviceID, int i2cBusID) throws Exception {
		this.protocol = protocol;
		
		if(protocol == Protocol.I2C) {
			I2Cbus = I2CFactory.getInstance(i2cBusID);
			I2Cdevice = I2Cbus.getDevice(deviceID);	
		}
		else if(protocol == Protocol.SPI) {
			/* Set SPI to run default speed (1Mhz) in default mode (Mode 0) */
			SPIdevice = SpiFactory.getInstance(SpiChannel.CS0, SpiDevice.DEFAULT_SPI_SPEED, SpiDevice.DEFAULT_SPI_MODE);
		}
		else {
			throw new Exception("Invalid protocol set: " + protocol);
		}
		
		loadCurrentConfigSettingsFromDevice();
		loadCompensationValues();		
	}
	
	private byte[] readBytesFromDevice(int address, int length) throws Exception {
		byte[] data = new byte[length];
		if(protocol == Protocol.I2C) {
			I2Cdevice.read(address, data, 0, length);
		}
		else if(protocol == Protocol.SPI) {
			byte currentAddr = (byte)(address & 0xFF);
			for(int i = 0; i < length; i++) {
				byte packet[] = new byte[2];
			    packet[0] = (byte)(currentAddr | 0x80) ;
			    packet[1] = 0x00;
			    byte[] result = SPIdevice.write(packet);
			    data[i] = result[1];
			    currentAddr += 1;
			}
		}
		else {
			throw new Exception("Invalid protocol set: " + protocol);	
		}
		
		return data;
	}
	
	private void writeConfiguration() throws Exception {
	
		if(protocol == Protocol.I2C) {
			I2Cdevice.write(CONFIG_OVERSAMPLE_ADDR, config_register_0xF4);
			I2Cdevice.write(CONFIG_WAIT_AND_FILTER_ADDR, config_register_0xF5);
		}
		else if(protocol == Protocol.SPI) {
			byte packet[] = new byte[2];
			packet[0] = (byte)(CONFIG_OVERSAMPLE_ADDR & 0x7F) ;
			packet[1] = config_register_0xF4;
			SPIdevice.write(packet);
			packet[0] = (byte)(CONFIG_WAIT_AND_FILTER_ADDR & 0x7F) ;
			packet[1] = config_register_0xF5;
			SPIdevice.write(packet);
		}
		else {
			throw new Exception("Protocol not implemented yet");
		}

		/* Small sleep time to let everything settle (DO NOT REMOVE!) */
		Thread.sleep(200);	
	}

	public double[] sampleDeviceReads() throws Exception {
		double[] result = new double[3];
		
		if(isDeviceSleeping()) {
			System.out.println("Device was sleeping. Forcing a read");
			setMode(Mode.FORCE, true);
		}
		
		byte[] data = readBytesFromDevice(SENSOR_START_ADDR, 8);

		// Create 20 bit values of raw values 
		long adc_p = (long)((data[0] & 0xFF) << 12) + (long)((data[1] & 0xFF) << 4) + (long)(data[2] & 0xFF);
		long adc_t = (long)((data[3] & 0xFF) << 12) + (long)((data[4] & 0xFF) << 4) + (long)(data[5] & 0xFF);		

		// Temperature calculations
		double var1 = 
					(
						(
							((double)adc_t) / 16384.0
						) 
						- ((double)compensationVals[DIG_T1]) / 1024.0
					)
					* ((double)compensationVals[DIG_T2]);
		
		double var2 = 
					(
						(
							(
								((double)adc_t) / 131072.0
							) 
							- ((double)compensationVals[DIG_T1]) / 8192.0
						) 
						* 
						(
							(
								((double)adc_t) / 131072.0
							) 
							- ((double)compensationVals[DIG_T1]) / 8192.0
						)
					) 
					*
					((double)compensationVals[DIG_T3]);
		
		double t_fine = (long)(var1 + var2);
		double cTemp = (var1 + var2) / 5120.0;
		double fTemp = cTemp * 1.8 + 32;

		// Pressure calculations
		var1 = ((double)t_fine / 2.0) - 64000.0;
		var2 = var1 * var1 * ((double)compensationVals[DIG_P6]) / 32768.0;
		var2 = var2 + var1 * ((double)compensationVals[DIG_P5]) * 2.0;
		var2 = (var2 / 4.0) + (((double)compensationVals[DIG_P4]) * 65536.0);
		var1 = (((double)compensationVals[DIG_P3]) * var1 * var1 / 524288.0 + ((double)compensationVals[DIG_P2]) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0) * ((double)compensationVals[DIG_P1]);
		double p = 1048576.0 - (double)adc_p;
		p = (p - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)compensationVals[DIG_P9]) * p * p / 2147483648.0;
		var2 = p * ((double)compensationVals[DIG_P8]) / 32768.0;
		double pressure = (p + (var1 + var2 + ((double)compensationVals[DIG_P7])) / 16.0) / 100;		
		
		if(adc_t != SKIP_VAL) { 
			result[TEMP_VAL_C] = cTemp;
			result[TEMP_VAL_F] = fTemp;
		}
		else {
			result[TEMP_VAL_C] = DEFAULT_SKIP_VAL_TEMP;
			result[TEMP_VAL_F] = DEFAULT_SKIP_VAL_TEMP;			
		}
		
		if(adc_p != SKIP_VAL) {
			result[PRES_VAL] = pressure;
		}
		else {
			result[PRES_VAL] = DEFAULT_SKIP_VAL_PRES;
		}
		
		return result;		
	}	
}