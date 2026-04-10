

/*08 Apr 2026
 * [ARQ.c]
 * Reworked BLE Set settings
 * - Added BLE_Valid_Value(value) Guard clause
 * - Implemented breaks on successful setting of values
 * - Added Setting of arQ Serial number
 * - case 'M': Added code for extracting Sensor Data
 *
 * Renamed a function to Examine_BLE_String
 * - removed parameters and used globar buffer instead
 *
 * Reworked Extract_Value_From_BLE_DebugMessage
 * - returns pointer to abuffer instead of a destination parameter
 * - removed destination parameter
 *
 * Reworked BLE_Valid_Value
 * - Added Serial number data validation
 *
 *
 * */
