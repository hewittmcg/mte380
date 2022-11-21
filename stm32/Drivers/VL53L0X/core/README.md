Calibrating Offset: 

	static int32_t offset = 0;
	static uint32_t dist = 100;
	dist = (dist*(1<<16));
	static VL53L0X_Error err;
	err = VL53L0X_PerformOffsetCalibration(&TOF_I2C[sensor], (FixPoint1616_t)dist, &offset);

Testing ToFs:

	uint16_t range_f, range_sf, range_sr;
	VL53L0X_Error err1, err2, err3;
	while(1) {
		err1 = get_tof_rangedata_cts(FORWARD_TOF, &range_f);
		err2 = get_tof_rangedata_cts(FRONT_SIDE_TOF, &range_sf);
		err3 = get_tof_rangedata_cts(REAR_SIDE_TOF, &range_sr);
	}

Taking a single measurement of TOF sensor Front Left:

	VL53L0X_PerformSingleRangingMeasurement(DevI2C1, &TOF_FL.RangingData);
	data => TOF_BR.RangingData.RangeMilliMeter;
		
For taking continous ranging:
	Set device to continuous operation mode
	set interrupt gpio mode:
    		/* Set interrupt config to new sample ready */
    		if (Status == VL53L0X_ERROR_NONE) {
        	Status = VL53L0X_SetGpioConfig(Dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
        	VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
        	VL53L0X_INTERRUPTPOLARITY_LOW);
	start measurement

	receive interrupt:
		Enter ISR
			Set Data readyflag for whichever TOF
		Exit ISR
  		In Main:
		Read Data (FROM TOF)
  		Clear interrupt mask (For TOF)
		Clear Data readyflag
