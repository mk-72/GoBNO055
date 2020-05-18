package bno055

/*Vector structure for returning sensor data
 *
 */
type Vector struct {
	X float64
	Y float64
	Z float64
}

/*Quaternion structure for returning sensor data
 *
 */
type Quaternion struct {
	X float64
	Y float64
	Z float64
	W float64
}

/*Calibration structure to represent calibration status
 *
 */
type Calibration struct {
	SysCalib byte
	GyrCalib byte
	AccCalib byte
	MagCalib byte
}

/*Offsets structure to represent offsets
 *
 */
type Offsets struct {
	AccOffsetX int
	AccOffsetY int
	AccOffsetZ int

	MagOffsetX int
	MagOffsetY int
	MagOffsetZ int

	GyrOffsetX int
	GyrOffsetY int
	GyrOffsetZ int

	AccRadius int

	MagRadius int
}

/*RevInfo structure to represent revisions
 *
 */
type RevInfo struct {
	AccRev uint8
	MagRev uint8
	GyrRev uint8
	SwRev  uint16
	BlRev  uint8
}
