package bno055

// register of the sensor BNO055 (see 4.2)
const (
	RegMagRadiusMSB  byte = 0x6A // Magnetometer Radius
	RegMagRadiusLSB  byte = 0x69 // Magnetometer Radius
	RegAccRadiusMSB  byte = 0x68 // Accelerometer Radius
	RegAccRadiusLSB  byte = 0x67 // Accelerometer Radius
	RegGyrOffsetZMSB byte = 0x66 // Gyroscope Offset Z <15:8>
	RegGyrOffsetZLSB byte = 0x65 // Gyroscope Offset Z <7:0>
	RegGyrOffsetYMSB byte = 0x64 // Gyroscope Offset Y <15:8>
	RegGyrOffsetYLSB byte = 0x63 // Gyroscope Offset Y <7:0>
	RegGyrOffsetXMSB byte = 0x62 // Gyroscope Offset X <15:8>
	RegGyrOffsetXLSB byte = 0x61 // Gyroscope Offset X <7:0>
	RegMagOffsetZMSB byte = 0x60 // Magnetometer Offset Z <15:8>
	RegMagOffsetZLSB byte = 0x5F // Magnetometer Offset Z <7:0>
	RegMagOffsetYMSB byte = 0x5E // Magnetometer Offset Y <15:8>
	RegMagOffsetYLSB byte = 0x5D // Magnetometer Offset Y <7:0>
	RegMagOffsetXMSB byte = 0x5C // Magnetometer Offset X <15:8>
	RegMagOffsetXLSB byte = 0x5B // Magnetometer Offset X <7:0>
	RegAccOffsetZMSB byte = 0x5A // Accelerometer Offset Z <15:8>
	RegAccOffsetZLSB byte = 0x59 // Accelerometer Offset Z <7:0>
	RegAccOffsetYMSB byte = 0x58 // Accelerometer Offset Y <15:8>
	RegAccOffsetYLSB byte = 0x57 // Accelerometer Offset Y <7:0>
	RegAccOffsetXMSB byte = 0x56 // Accelerometer Offset X <15:8>
	RegAccOffsetXLSB byte = 0x55 // Accelerometer Offset X <7:0>
	RegAxisMapSign   byte = 0x42 // Remapped axis signs
	RegAxisMapConfig byte = 0x41 // Remapped axis value
	RegTempSource    byte = 0x40 // TEMPSource <1:0>
	RegSysTrigger    byte = 0x3F // CLK_SEL, RST_INT, RST_SYS, Self_Test
	RegPwrMode       byte = 0x3E // Power Mode <1:0>
	RegOprMode       byte = 0x3D // Operation Mode <3:0>
	RegUnitSel       byte = 0x3B // ORI_Android_Windows, TEMP_Unit, EUL_Unit, GYR_Unit, ACC_Unit
	RegSysErr        byte = 0x3A // System Error Code
	RegSysStatus     byte = 0x39 // System Status Code
	RegSysClkStatus  byte = 0x38 // ST_MAIN_CLK
	RegIntSta        byte = 0x37 // ACC_NM, ACC_AM, ACC_HIGH_G, GYR_HIGH_RATE, GYRO_AM
	RegStResult      byte = 0x36 // ST_MCU, ST_GYR, ST_MAG, ST_ACC
	RegCalibStat     byte = 0x35 // SYS, GYR, ACC, MAG Calib Status
	RegTemp          byte = 0x34 // Temperature
	RegGrvDataZMSB   byte = 0x33 // Gravity Vector Data Z <15:8>
	RegGrvDataZLSB   byte = 0x32 // Gravity Vector Data Z <7:0>
	RegGrvDataYMSB   byte = 0x31 // Gravity Vector Data Y <15:8>
	RegGrvDataYLSB   byte = 0x30 // Gravity Vector Data Y <7:0>
	RegGrvDataXMSB   byte = 0x2F // Gravity Vector Data X <15:8>
	RegGrvDataXLSB   byte = 0x2E // Gravity Vector Data X <7:0>
	RegLiaDataZMSB   byte = 0x2D // Linear Acceleration Data Z <15:8>
	RegLiaDataZLSB   byte = 0x2C // Linear Acceleration Data Z <7:0>
	RegLiaDataYMSB   byte = 0x2B // Linear Acceleration Data Y <15:8>
	RegLiaDataYLSB   byte = 0x2A // Linear Acceleration Data Y <7:0>
	RegLiaDataXMSB   byte = 0x29 // Linear Acceleration Data X <15:8>
	RegLiaDataXLSB   byte = 0x28 // Linear Acceleration Data X <7:0>
	RegQuaDataZMSB   byte = 0x27 // Quaternion Data Z <15:8>
	RegQuaDataZLSB   byte = 0x26 // Quaternion Data Z <7:0>
	RegQuaDataYMSB   byte = 0x25 // Quaternion Data Y <15:8>
	RegQuaDataYLSB   byte = 0x24 // Quaternion Data Y <7:0>
	RegQuaDataXMSB   byte = 0x23 // Quaternion Data X <15:8>
	RegQuaDataXLSB   byte = 0x22 // Quaternion Data X <7:0>
	RegQuaDataWMSB   byte = 0x21 // Quaternion Data W <15:8>
	RegQuaDataWLSB   byte = 0x20 // Quaternion Data W <7:0>
	RegEulPitchMSB   byte = 0x1F // Pitch Data <15:8>
	RegEulPitchLSB   byte = 0x1E // Pitch Data <7:0>
	RegEulRollMSB    byte = 0x1D // Roll Data <15:8>
	RegEulRollLSB    byte = 0x1C // Roll Data <7:0>
	RegEulHeadingMSB byte = 0x1B // Heading Data <15:8>
	RegEulHeadingLSB byte = 0x1A // Heading Data <7:0>
	RegGyrDataZMSB   byte = 0x19 // Gyroscope Data Z <15:8>
	RegGyrDataZLSB   byte = 0x18 // Gyroscope Data Z <7:0>
	RegGyrDataYMSB   byte = 0x17 // Gyroscope Data Y <15:8>
	RegGyrDataYLSB   byte = 0x16 // Gyroscope Data Y <7:0>
	RegGyrDataXMSB   byte = 0x15 // Gyroscope Data X <15:8>
	RegGyrDataXLSB   byte = 0x14 // Gyroscope Data X <7:0>
	RegMagDataZMSB   byte = 0x13 // Magnetometer Data Z <15:8>
	RegMagDataZLSB   byte = 0x12 // Magnetometer Data Z <7:0>
	RegMagDataYMSB   byte = 0x11 // Magnetometer Data Y <15:8>
	RegMagDataYLSB   byte = 0x10 // Magnetometer Data Y <7:0>
	RegMagDataXMSB   byte = 0x0F // Magnetometer Data X <15:8>
	RegMagDataXLSB   byte = 0x0E // Magnetometer Data X <7:0>
	RegAccDataZMSB   byte = 0x0D // Acceleration Data Z <15:8>
	RegAccDataZLSB   byte = 0x0C // Acceleration Data Z <7:0>
	RegAccDataYMSB   byte = 0x0B // Acceleration Data Y <15:8>
	RegAccDataYLSB   byte = 0x0A // Acceleration Data Y <7:0>
	RegAccDataXMSB   byte = 0x09 // Acceleration Data X <15:8>
	RegAccDataXLSB   byte = 0x08 // Acceleration Data X <7:0>
	RegPageID        byte = 0x07 // Page ID
	RegBLRevID       byte = 0x06 // Bootloader Version
	RegSWRevIDMSB    byte = 0x05 // SW Revision ID <15:8>
	RegSWRevIDLSB    byte = 0x04 // SW Revision ID <7:0>
	RegGyrID         byte = 0x03 // GYRO chip ID
	RegMagID         byte = 0x02 // MAG chip ID
	RegAccID         byte = 0x01 // ACC chip ID
	RegChipID        byte = 0x00 // BNO055 CHIP ID

)

// operation mode of the sensor BNO055 (see 3.3)
const (
	ModeConfigMode byte = 0x00
	ModeAccOnly    byte = 0x01
	ModeMagOnly    byte = 0x02
	ModeGyrOnly    byte = 0x03
	ModeAccMag     byte = 0x04
	ModeAccGyro    byte = 0x05
	ModeMagGyro    byte = 0x06
	ModeAMG        byte = 0x07
	ModeIMU        byte = 0x08
	ModeCompass    byte = 0x09
	ModeM4G        byte = 0x0A
	ModeNdofFMCOff byte = 0x0B
	ModeNdof       byte = 0x0C
)

// power mode of the sensor BNO055 (see 3.2)
const (
	PwrmodeNormal  byte = 0x00
	PwrmodeLow     byte = 0x01
	PwrmodeSuspend byte = 0x02
)

// vector type of the sensor BNO055 (see 3.6.5)
const (
	VectorMagnetometer  byte = RegMagDataXLSB
	VectorGyroscope     byte = RegGyrDataXLSB
	VectorEuler         byte = RegEulHeadingLSB
	VectorAccelerometer byte = RegAccDataXLSB
	VectorLinearAccel   byte = RegLiaDataXLSB
	VectorGravity       byte = RegGrvDataXLSB
)

// remap setting of the sensor BNO055 (see 3.4)
const (
	RemapconfigP0 byte = 0x21
	RemapconfigP1 byte = 0x24
	RemapconfigP2 byte = 0x24 // default
	RemapconfigP3 byte = 0x21
	RemapconfigP4 byte = 0x24
	RemapconfigP5 byte = 0x21
	RemapconfigP6 byte = 0x21
	RemapconfigP7 byte = 0x24
)

// remap sign of the sensor BNO055 (see 3.4)
const (
	RemapsignP0 byte = 0x04
	RemapsignP1 byte = 0x00 // default
	RemapsignP2 byte = 0x06
	RemapsignP3 byte = 0x02
	RemapsignP4 byte = 0x03
	RemapsignP5 byte = 0x01
	RemapsignP6 byte = 0x07
	RemapsignP7 byte = 0x05
)
