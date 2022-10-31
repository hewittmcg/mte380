Taking a single measurement of TOF sensor Front Left:

		VL53L0X_PerformSingleRangingMeasurement(DevI2C1, &TOF_FL.RangingData);
		data => TOF_BR.RangingData.RangeMilliMeter;