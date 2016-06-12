package net.pateras.iot.BMP280Example;

import net.pateras.iot.BMP280.BMP280;
import net.pateras.iot.BMP280.BMP280.IIRFilter;
import net.pateras.iot.BMP280.BMP280.Mode;
import net.pateras.iot.BMP280.BMP280.Pressure_Sample_Resolution;
import net.pateras.iot.BMP280.BMP280.Standby_Time;
import net.pateras.iot.BMP280.BMP280.Temperature_Sample_Resolution;

/* This class demonstrates how the device settings can be set and readings obtained */
public class BMP280Example {

	public static void main(String args[]) throws Exception {
		
		BMP280 bmp280;
		
		/* If using I2C protocol, uncomment this line */
		//bmp280 = new BMP280(BMP280.Protocol.I2C, BMP280.ADDR_SDO_2_VDDIO, I2CBus.BUS_1);
		
		/* If using SPI protocol, uncomment this like */
		bmp280 = new BMP280(BMP280.Protocol.SPI, 0, 0);
		
		
		/* Set the device to run in desired mode
		 * All pre-defined modes within the BOSCH product documentation have been created
		 * or the user can specific each control separately. Read the BMP280 library source
		 * code which contains comments on what all the available options are, or refer to 
		 * the BOSCH product documentation 
		 * 
		 * NOTE: the config settings control the resolution and how frequently the device
		 * updates its memory registers with the current temperature and pressure. 
		 * It is up to you to call the sampleDeviceReads() method as frequently as you wish
		 * to obtain the current register values. eg. If the device has been configured to 
		 * to take a sample once ever 4 seconds. there is no value in calling sampleDeviceReads()
		 * more frequently than every 4 seconds, otherwise you will simply get the same (old)
		 * values as the last time you called the method, 
		 */
		
		/* Example of using a pre-defined setup (refer to source code comments which explains each
		 * pre-defined setup */
		bmp280.setIndoorNavigationMode();
		
		/* Example of setting individual settings 
		 * Settings which can be set are
		 * - Operation mode (sleep, force, normal)
		 * - Temperate oversample (skip, x1, x2, x4, x8, x16)
		 * - Pressure oversample (skip, x1, x2, x4, x8, x16)
		 * - IIRFilter value (off, x2, x4, x8, x16)
		 * - Standby time between device updating temp/pres registers (for normal mode) */
		bmp280.setMode(Mode.NORMAL, true);
		bmp280.setTemperatureSampleRate(Temperature_Sample_Resolution.TWO, true);
		bmp280.setPressureSampleRate(Pressure_Sample_Resolution.SIXTEEN, true);
		bmp280.setIIRFilter(IIRFilter.SIXTEEN, true);
		bmp280.setStandbyTime(Standby_Time.MS_POINT_5, true);
		
		/* Poll the device once per second and display the reading */
		while(true) {
			double[] results = bmp280.sampleDeviceReads();
			
			// Output data to screen
			System.out.printf("Pressure : %.2f hPa %n", results[BMP280.PRES_VAL]);
			System.out.printf("Temperature in Celsius : %.2f C %n", results[BMP280.TEMP_VAL_C]);
			
			Thread.sleep(1000);
		}
	}

}
