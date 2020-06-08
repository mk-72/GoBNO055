package bno055

import (
	"time"

	"github.com/d2r2/go-i2c"
)

/*Sensor is a struct that represents the sensor.
 *
 * i2cbus 			*I2C struct 	 	Points to the i2c bus struct.
 * operationmode 	byte 				Contains the current oprmode of the sensor.
 * SensorRevision 	RevInfo struct 		Contains information about sensor component revisions.
 * SensorCalib 		Calibration struct 	Contains last detected calibration status.
 * SystemStatus   	uint8				Contains last detected system stauts (section 4.3.58).
 * SelfTestResult 	uint8				Contains last detected self test result (section 4.3.55).
 * SystemError    	uint8				Contains last detected system error (section 4.3.59).
 */
type Sensor struct {
	i2cbus         *i2c.I2C
	operationmode  byte
	SensorRevision RevInfo
	SensorCalib    Calibration
	SystemStatus   uint8
	SelfTestResult uint8
	SystemError    uint8
}

/*NewSensor creates new sensor struct and connects it via i2c bus.
 *
 * param	addr	uint8	The i2c device address.
 * param	bus		int		The i2c bus number.
 * return	sens	*Sensor	The sensor struct representing the sensor.
 * return	err		error
 */
func NewSensor(addr uint8, bus int) (sens *Sensor, err error) {
	newi2cbus, err := i2c.NewI2C(addr, bus)
	if err != nil {
		return nil, err
	}
	sensor := Sensor{i2cbus: newi2cbus, operationmode: ModeConfigMode}
	return &sensor, err
}

/*Close closes i2c connection of the sensor.
 *
 * return	err	error
 */
func (sens *Sensor) Close() (err error) {
	err = sens.i2cbus.Close()
	sens.operationmode = ModeConfigMode
	return err
}

/*Begin sets up the sensor correctly with the desired operationmode.
 *
 * param	oprmode	byte	The desired operationmode of the sensor.
 *			oprmode values:
 *					ModeConfigMode
 *					ModeAccOnly
 *					ModeMagOnly
 *					ModeGyrOnly
 *					ModeAccMag
 *					ModeAccGyro
 *					ModeMagGyro
 *					ModeAMG
 *					ModeIMU
 *					ModeCompass
 *					ModeM4G
 *					ModeNdofFMCOff
 *					ModeNdof
 * return	success	bool	True if set up successful, else false.
 * return	err		error
 */
func (sens *Sensor) Begin(oprmode byte) (success bool, err error) {
	// check if connected device is the right device (bno055 chip id is 0xa0)
	id, err := sens.i2cbus.ReadRegU8(RegChipID)
	if err != nil {
		return false, err
	}
	if id != 0xa0 {
		// wait for boot and check again
		time.Sleep(time.Millisecond * 1000)
		id, err = sens.i2cbus.ReadRegU8(RegChipID)
		if err != nil {
			return false, err
		}
		if id != 0xa0 {
			return false, err // still not okay
		}
	}

	// switch to config mode, should be default when starting up the sensor
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return false, err
	}

	// reset
	err = sens.i2cbus.WriteRegU8(RegSysTrigger, 0x20)
	if err != nil {
		return false, err
	}
	time.Sleep(time.Millisecond * 30)
	disconnected := true
	for disconnected {
		time.Sleep(time.Millisecond * 10)
		id, _ := sens.i2cbus.ReadRegU8(RegChipID)
		if id == 0xa0 {
			disconnected = false
		}
	}
	time.Sleep(time.Millisecond * 50)

	// set power to normal mode
	sens.i2cbus.WriteRegU8(RegPwrMode, PwrmodeNormal)
	if err != nil {
		return false, err
	}
	time.Sleep(time.Millisecond * 10)

	sens.i2cbus.WriteRegU8(RegPageID, 0)
	if err != nil {
		return false, err
	}

	sens.i2cbus.WriteRegU8(RegSysTrigger, 0x0)
	if err != nil {
		return false, err
	}
	time.Sleep(time.Millisecond * 10)

	// set requestet operation mode
	err = sens.SetMode(oprmode)
	if err != nil {
		return false, err
	}
	time.Sleep(time.Millisecond * 20)

	return true, err
}

/*SetMode sets the operation mode of the sensor.
 *
 * param	mode	byte	The desired operationmode of the sensor.
 *			mode	values:
 *					ModeConfigMode
 *					ModeAccOnly
 *					ModeMagOnly
 *					ModeGyrOnly
 *					ModeAccMag
 *					ModeAccGyro
 *					ModeMagGyro
 *					ModeAMG
 *					ModeIMU
 *					ModeCompass
 *					ModeM4G
 *					ModeNdofFMCOff
 *					ModeNdof
 * return	err		error
 */
func (sens *Sensor) SetMode(mode byte) (err error) {
	sens.operationmode = mode
	err = sens.i2cbus.WriteRegU8(RegOprMode, mode)
	return err
}

/*SetAxisRemap changes the chip's axis remap.
 *
 * param	remapcode	byte	The remapcode.
 *			remapcode	values:
 *						RemapconfigP0
 *						RemapconfigP1 (default)
 *						RemapconfigP2
 *						RemapconfigP3
 *						RemapconfigP4
 *						RemapconfigP5
 *						RemapconfigP6
 *						RemapconfigP7
 * return	err			error
 */
func (sens *Sensor) SetAxisRemap(remapcode byte) (err error) {
	modeback := sens.operationmode
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 25)
	err = sens.i2cbus.WriteRegU8(RegAxisMapConfig, remapcode)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 10)
	err = sens.SetMode(modeback)
	time.Sleep(time.Millisecond * 20)
	return err
}

/*SetAxisSign changes the chip's axis signs.
 *
 * param	remapsign	byte	The remapsign.
 *			remapsign	values:
 *						RemapsignP0
 *						RemapsignP1 (default)
 *						RemapsignP2
 *						RemapsignP3
 *						RemapsignP4
 *						RemapsignP5
 *						RemapsignP6
 *						RemapsignP7
 * return	err			error
 */
func (sens *Sensor) SetAxisSign(remapsign byte) (err error) {
	modeback := sens.operationmode
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 25)
	err = sens.i2cbus.WriteRegU8(RegAxisMapSign, remapsign)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 10)
	err = sens.SetMode(modeback)
	time.Sleep(time.Millisecond * 20)
	return err
}

/*SetExtCrystalUse use the external 32.768kHz crystal.
 *
 * param	usextal	bool	True if external crystall should be used, else false.
 * return	err		error
 */
func (sens *Sensor) SetExtCrystalUse(usextal bool) (err error) {
	modeback := sens.operationmode
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 25)
	err = sens.i2cbus.WriteRegU8(RegPageID, 0)
	if err != nil {
		return err
	}
	if usextal {
		err = sens.i2cbus.WriteRegU8(RegSysTrigger, 0x80)
	} else {
		err = sens.i2cbus.WriteRegU8(RegSysTrigger, 0x0)
	}
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 10)
	err = sens.SetMode(modeback)
	time.Sleep(time.Second) // seems to be necessary, without this the sensor only returns 0 for one second
	return err
}

/*GetSystemStatus gets the latest system status info.
 *
 * return	err	error
 */
func (sens *Sensor) GetSystemStatus() (err error) {
	err = sens.i2cbus.WriteRegU8(RegPageID, 0)
	if err != nil {
		return err
	}

	sens.SystemStatus, err = sens.i2cbus.ReadRegU8(RegSysStatus)
	if err != nil {
		return err
	}

	sens.SelfTestResult, err = sens.i2cbus.ReadRegU8(RegStResult)
	if err != nil {
		return err
	}

	sens.SystemError, err = sens.i2cbus.ReadRegU8(RegSysErr)

	time.Sleep(time.Millisecond * 200)
	return err
}

/*GetRevInfo gets the chip's revision numbers.
 *
 * return	err	error
 */
func (sens *Sensor) GetRevInfo() (err error) {
	sens.SensorRevision.AccRev, err = sens.i2cbus.ReadRegU8(RegAccID)
	if err != nil {
		return err
	}

	sens.SensorRevision.MagRev, err = sens.i2cbus.ReadRegU8(RegMagID)
	if err != nil {
		return err
	}

	sens.SensorRevision.GyrRev, err = sens.i2cbus.ReadRegU8(RegGyrID)
	if err != nil {
		return err
	}

	sens.SensorRevision.BlRev, err = sens.i2cbus.ReadRegU8(RegBLRevID)
	if err != nil {
		return err
	}

	a, err := sens.i2cbus.ReadRegU8(RegSWRevIDLSB)
	if err != nil {
		return err
	}
	b, err := sens.i2cbus.ReadRegU8(RegSWRevIDMSB)
	sens.SensorRevision.SwRev = (uint16(b) << 8) | uint16(a)

	return err
}

/*GetCalibration gets current calibration state.
 *
 * return	err	error
 */
func (sens *Sensor) GetCalibration() (err error) {
	calibStat, err := sens.i2cbus.ReadRegU8(RegCalibStat)

	sens.SensorCalib.SysCalib = (calibStat >> 6) & 0x03
	sens.SensorCalib.GyrCalib = (calibStat >> 4) & 0x03
	sens.SensorCalib.AccCalib = (calibStat >> 2) & 0x03
	sens.SensorCalib.MagCalib = calibStat & 0x03

	return err
}

/*IsFullyCalibrated checks if all necessary sensors are calibrated.
 *
 * return	isCalibrated	bool	True if all necessary calib values are 3, else false.
 * return	err				error
 */
func (sens *Sensor) IsFullyCalibrated() (isCalibrated bool, err error) {
	err = sens.GetCalibration()

	switch sens.operationmode {
	case ModeAccOnly:
		return (sens.SensorCalib.AccCalib == 3), err
	case ModeMagOnly:
		return (sens.SensorCalib.MagCalib == 3), err
	case ModeGyrOnly:
	case ModeM4G:
		return (sens.SensorCalib.GyrCalib == 3), err
	case ModeAccMag:
	case ModeCompass:
		return (sens.SensorCalib.AccCalib == 3 && sens.SensorCalib.MagCalib == 3), err
	case ModeAccGyro:
	case ModeIMU:
		return (sens.SensorCalib.AccCalib == 3 && sens.SensorCalib.GyrCalib == 3), err
	case ModeMagGyro:
		return (sens.SensorCalib.MagCalib == 3 && sens.SensorCalib.GyrCalib == 3), err
	default:
	}
	return (sens.SensorCalib.SysCalib == 3 && sens.SensorCalib.AccCalib == 3 && sens.SensorCalib.MagCalib == 3 && sens.SensorCalib.GyrCalib == 3), err
}

/*SetOffsets writes calibration values to the offset registers.
 *
 * param	offsets	Offsets struct	The structure containing calibration values.
 * return	err		error
 */
func (sens *Sensor) SetOffsets(offsets Offsets) (err error) {
	modeback := sens.operationmode
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 25)

	sens.i2cbus.WriteRegU8(RegAccOffsetXLSB, (byte((offsets.AccOffsetX) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegAccOffsetXMSB, (byte((offsets.AccOffsetX >> 8) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegAccOffsetYLSB, (byte((offsets.AccOffsetY) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegAccOffsetYMSB, (byte((offsets.AccOffsetY >> 8) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegAccOffsetZLSB, (byte((offsets.AccOffsetZ) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegAccOffsetZMSB, (byte((offsets.AccOffsetZ >> 8) & 0x0FF)))

	sens.i2cbus.WriteRegU8(RegMagOffsetXLSB, (byte((offsets.MagOffsetX) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegMagOffsetXMSB, (byte((offsets.MagOffsetX >> 8) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegMagOffsetYLSB, (byte((offsets.MagOffsetY) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegMagOffsetYMSB, (byte((offsets.MagOffsetY >> 8) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegMagOffsetZLSB, (byte((offsets.MagOffsetZ) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegMagOffsetZMSB, (byte((offsets.MagOffsetZ >> 8) & 0x0FF)))

	sens.i2cbus.WriteRegU8(RegGyrOffsetXLSB, (byte((offsets.GyrOffsetX) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegGyrOffsetXMSB, (byte((offsets.GyrOffsetX >> 8) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegGyrOffsetYLSB, (byte((offsets.GyrOffsetY) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegGyrOffsetYMSB, (byte((offsets.GyrOffsetY >> 8) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegGyrOffsetZLSB, (byte((offsets.GyrOffsetZ) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegGyrOffsetZMSB, (byte((offsets.GyrOffsetZ >> 8) & 0x0FF)))

	sens.i2cbus.WriteRegU8(RegAccRadiusLSB, (byte((offsets.AccRadius) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegAccRadiusMSB, (byte((offsets.AccRadius >> 8) & 0x0FF)))

	sens.i2cbus.WriteRegU8(RegMagRadiusLSB, (byte((offsets.MagRadius) & 0x0FF)))
	sens.i2cbus.WriteRegU8(RegMagRadiusMSB, (byte((offsets.MagRadius >> 8) & 0x0FF)))

	err = sens.SetMode(modeback)
	return err
}

/*GetOffsets reads calibration values from the offset registers.
 *
 * return	offsets	Offsets struct	The structure containing calibration values.
 * return	success	bool			True if sensor is fully calibrated and offsets are correct, else false.
 * return	err		error
 */
func (sens *Sensor) GetOffsets() (offsets Offsets, success bool, err error) {
	fullyCalibrated, err := sens.IsFullyCalibrated()
	if !fullyCalibrated {
		return offsets, false, err
	}

	modeback := sens.operationmode
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return offsets, false, err
	}
	time.Sleep(time.Millisecond * 25)

	// Accelerometer offsets
	accOffsetXMSB, _ := sens.i2cbus.ReadRegU8(RegAccOffsetXMSB)
	accOffsetXLSB, _ := sens.i2cbus.ReadRegU8(RegAccOffsetXLSB)
	offsets.AccOffsetX = (int(accOffsetXMSB) << 8) | int(accOffsetXLSB)

	accOffsetYMSB, _ := sens.i2cbus.ReadRegU8(RegAccOffsetYMSB)
	accOffsetYLSB, _ := sens.i2cbus.ReadRegU8(RegAccOffsetYLSB)
	offsets.AccOffsetY = (int(accOffsetYMSB) << 8) | int(accOffsetYLSB)

	accOffsetZMSB, _ := sens.i2cbus.ReadRegU8(RegAccOffsetZMSB)
	accOffsetZLSB, _ := sens.i2cbus.ReadRegU8(RegAccOffsetZLSB)
	offsets.AccOffsetZ = (int(accOffsetZMSB) << 8) | int(accOffsetZLSB)

	// Magnetometer offsets
	magOffsetXMSB, _ := sens.i2cbus.ReadRegU8(RegMagOffsetXMSB)
	magOffsetXLSB, _ := sens.i2cbus.ReadRegU8(RegMagOffsetXLSB)
	offsets.AccOffsetX = (int(magOffsetXMSB) << 8) | int(magOffsetXLSB)

	magOffsetYMSB, _ := sens.i2cbus.ReadRegU8(RegMagOffsetYMSB)
	magOffsetYLSB, _ := sens.i2cbus.ReadRegU8(RegMagOffsetYLSB)
	offsets.AccOffsetY = (int(magOffsetYMSB) << 8) | int(magOffsetYLSB)

	magOffsetZMSB, _ := sens.i2cbus.ReadRegU8(RegMagOffsetZMSB)
	magOffsetZLSB, _ := sens.i2cbus.ReadRegU8(RegMagOffsetZLSB)
	offsets.AccOffsetZ = (int(magOffsetZMSB) << 8) | int(magOffsetZLSB)

	// Gyroscope offsets
	gyrOffsetXMSB, _ := sens.i2cbus.ReadRegU8(RegGyrOffsetXMSB)
	gyrOffsetXLSB, _ := sens.i2cbus.ReadRegU8(RegGyrOffsetXLSB)
	offsets.AccOffsetX = (int(gyrOffsetXMSB) << 8) | int(gyrOffsetXLSB)

	gyrOffsetYMSB, _ := sens.i2cbus.ReadRegU8(RegGyrOffsetYMSB)
	gyrOffsetYLSB, _ := sens.i2cbus.ReadRegU8(RegGyrOffsetYLSB)
	offsets.AccOffsetY = (int(gyrOffsetYMSB) << 8) | int(gyrOffsetYLSB)

	gyrOffsetZMSB, _ := sens.i2cbus.ReadRegU8(RegGyrOffsetZMSB)
	gyrOffsetZLSB, _ := sens.i2cbus.ReadRegU8(RegGyrOffsetZLSB)
	offsets.AccOffsetZ = (int(gyrOffsetZMSB) << 8) | int(gyrOffsetZLSB)

	// Accelerometer radius
	accRadiusMSB, _ := sens.i2cbus.ReadRegU8(RegAccRadiusMSB)
	accRadiusLSB, _ := sens.i2cbus.ReadRegU8(RegAccRadiusLSB)
	offsets.AccOffsetZ = (int(accRadiusMSB) << 8) | int(accRadiusLSB)

	// Magnetometer radius
	magRadiusMSB, _ := sens.i2cbus.ReadRegU8(RegMagRadiusMSB)
	magRadiusLSB, _ := sens.i2cbus.ReadRegU8(RegMagRadiusLSB)
	offsets.AccOffsetZ = (int(magRadiusMSB) << 8) | int(magRadiusLSB)

	err = sens.SetMode(modeback)
	return offsets, true, err
}

/*GetTemp gets temperature in degrees celsius.
 *
 * return	temp	int		The temperature in degrees celsius.
 * return	err		error
 */
func (sens *Sensor) GetTemp() (temp int, err error) {
	tempByte, err := sens.i2cbus.ReadRegU8(RegTemp)
	if err != nil {
		return 0, err
	}
	return int(tempByte), err
}

/*GetVector reads a vector from the specified source.
 *
 * param	vectortype	byte			The source of the vector.
 *			vectortype	values:
 *						VectorMagnetometer
 *						VectorGyroscope
 *						VectorEuler
 *						VectorAccelerometer
 *						VectorLinearAccel
 *						VectorGravity
 * return	xyz			Vector struct	The vector from the specified source.
 * return	err			error
 */
func (sens *Sensor) GetVector(vectortype byte) (xyz Vector, err error) {
	var x, y, z int16 = 0, 0, 0

	buffer, _, err := sens.i2cbus.ReadRegBytes(vectortype, 6)

	x = (int16(buffer[0]) | (int16(buffer[1]) << 8))
	y = (int16(buffer[2]) | (int16(buffer[3]) << 8))
	z = (int16(buffer[4]) | (int16(buffer[5]) << 8))

	switch vectortype {
	case VectorMagnetometer:
		// 1 uT = 16 LSB
		xyz.X = (float64(x)) / 16.0
		xyz.Y = (float64(y)) / 16.0
		xyz.Z = (float64(z)) / 16.0
		break
	case VectorGyroscope:
		// 1 dps = 16 LSB
		xyz.X = (float64(x)) / 16.0
		xyz.Y = (float64(y)) / 16.0
		xyz.Z = (float64(z)) / 16.0
		break
	case VectorEuler:
		// 1 degree = 16 LSB
		xyz.X = (float64(x)) / 16.0
		xyz.Y = (float64(y)) / 16.0
		xyz.Z = (float64(z)) / 16.0
		break
	case VectorAccelerometer:
		// 1 m/s^2 = 100 LSB
		xyz.X = (float64(x)) / 100
		xyz.Y = (float64(y)) / 100
		xyz.Z = (float64(z)) / 100
		break
	case VectorLinearAccel:
		// 1 m/s^2 = 100 LSB
		xyz.X = (float64(x)) / 100
		xyz.Y = (float64(y)) / 100
		xyz.Z = (float64(z)) / 100
		break
	case VectorGravity:
		// 1 m/s^2 = 100 LSB
		xyz.X = (float64(x)) / 100
		xyz.Y = (float64(y)) / 100
		xyz.Z = (float64(z)) / 100
		break
	default:
		xyz.X = (float64(x))
		xyz.Y = (float64(y))
		xyz.Z = (float64(z))
		break
	}

	return xyz, err
}

/*GetMagnetometer reads a vector from the magnetometer.
 *
 * return	xyz	Vector struct	The vector from the magnetometer.
 * return	err	error
 */
func (sens *Sensor) GetMagnetometer() (xyz Vector, err error) {
	xyz, err = sens.GetVector(VectorMagnetometer)
	return xyz, err
}

/*GetGyroscope reads a vector from the gyroscope.
 *
 * return	xyz	Vector struct	The vector from the gyroscope.
 * return	err	error
 */
func (sens *Sensor) GetGyroscope() (xyz Vector, err error) {
	xyz, err = sens.GetVector(VectorGyroscope)
	return xyz, err
}

/*GetEuler reads a vector from the euler rotation sensor.
 *
 * return	xyz	Vector struct	The vector from the euler rotation sensor.
 * return	err	error
 */
func (sens *Sensor) GetEuler() (xyz Vector, err error) {
	xyz, err = sens.GetVector(VectorEuler)
	return xyz, err
}

/*GetAccelerometer reads a vector from the accelerometer.
 *
 * return	xyz	Vector struct	The vector from the accelerometer.
 * return	err	error
 */
func (sens *Sensor) GetAccelerometer() (xyz Vector, err error) {
	xyz, err = sens.GetVector(VectorAccelerometer)
	return xyz, err
}

/*GetLinearAccel reads a vector from the linear accelerometer.
 *
 * return	xyz	Vector struct	The vector from the linear accelerometer.
 * return	err	error
 */
func (sens *Sensor) GetLinearAccel() (xyz Vector, err error) {
	xyz, err = sens.GetVector(VectorLinearAccel)
	return xyz, err
}

/*GetGravity reads a vector from the gravity sensor.
 *
 * return	xyz	Vector struct	The vector from the gravity sensor.
 * return	err	error
 */
func (sens *Sensor) GetGravity() (xyz Vector, err error) {
	xyz, err = sens.GetVector(VectorGravity)
	return xyz, err
}

/*GetQuat reads a quaternion.
 *
 * return	quat	Quaternion struct	The quaternion.
 * return	err		error
 */
func (sens *Sensor) GetQuat() (quat Quaternion, err error) {
	var x, y, z, w int16 = 0, 0, 0, 0

	buffer, _, err := sens.i2cbus.ReadRegBytes(RegQuaDataWLSB, 8)

	w = (int16(buffer[1]) << 8) | int16(buffer[0])
	x = (int16(buffer[3]) << 8) | int16(buffer[2])
	y = (int16(buffer[5]) << 8) | int16(buffer[4])
	z = (int16(buffer[7]) << 8) | int16(buffer[6])

	var scale float64 = 1.0 / (1 << 14)

	quat.W = scale * float64(w)
	quat.X = scale * float64(x)
	quat.Y = scale * float64(y)
	quat.Z = scale * float64(z)

	return quat, err
}

/*EnterSuspendMode enter suspend mode (i.e., sleep).
 *
 * return	err	error
 */
func (sens *Sensor) EnterSuspendMode() (err error) {
	modeback := sens.operationmode
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 25)
	err = sens.i2cbus.WriteRegU8(RegPwrMode, PwrmodeSuspend)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 10)
	err = sens.SetMode(modeback)
	time.Sleep(time.Millisecond * 20)
	return err
}

/*EnterNormalMode enter normal mode (i.e., wake).
 *
 * return	err	error
 */
func (sens *Sensor) EnterNormalMode() (err error) {
	modeback := sens.operationmode
	err = sens.SetMode(ModeConfigMode)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 25)
	err = sens.i2cbus.WriteRegU8(RegPwrMode, PwrmodeNormal)
	if err != nil {
		return err
	}
	time.Sleep(time.Millisecond * 10)
	err = sens.SetMode(modeback)
	time.Sleep(time.Millisecond * 20)
	return err
}

/*ReadRegister reads desired register from the sensor
 *
 * param	reg		byte	The register address.
 * return	value	byte	The value in the desired register.
 * return	err		error
 */
func (sens *Sensor) ReadRegister(reg byte) (value byte, err error) {
	return sens.i2cbus.ReadRegU8(reg)
}
