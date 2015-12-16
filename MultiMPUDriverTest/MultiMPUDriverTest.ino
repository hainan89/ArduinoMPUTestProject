#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <MPUDriver.h>

unsigned char HW_addr;

MPUDriver mpu;
//uint8_t myNSC = 10;

#define DEFAULT_MPU_HZ 100
int mpu_init_state = 1;
int mpu_dmp_state = 1;
int dmp_load_state = 1;

unsigned long sensor_timestamp;
short gyro[3], accel[3], sensors;
long quat[4];
//float gyro_f[3], accel_f[3], quat_f[4];
unsigned char more;
uint8_t read_r;
#define MPUNUM 4
uint8_t ncs_l[MPUNUM] = {10, 9, 8, 7};
uint8_t current_mpu = 0;

// HC05 RXD <-> 11  // 5
// HC05 TXD <-> 10  // 4
#define BT_SERIAL_TX 4
/* to communicate with the Bluetooth module's RXD pin */
#define BT_SERIAL_RX 5
/* Initialise the software serial port */
SoftwareSerial BluetoothSerial(BT_SERIAL_TX, BT_SERIAL_RX);

void setup() {

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  

  //Serial.begin(115200);
  Serial.begin(57600); // 为了配合Bluetooth传输，降低Baud
  while (!Serial);

  BluetoothSerial.begin(57600);
  BluetoothSerial.println("Blt Started.");
  
  for (uint8_t i = 0 ; i < MPUNUM; i++)
  {
    pinMode(ncs_l[i], OUTPUT);
    digitalWrite(ncs_l[i], HIGH);
  }
  
  for (uint8_t i = 0 ; i < MPUNUM; i++)
  {
    HW_addr = ncs_l[i];
        
    mpu_init_state = mpu.mpu_init(0); // without interrupt

    if (mpu_init_state)
    {
      Serial.print(mpu_init_state);
      Serial.println("\t");
      Serial.println("MPU Init failed!");
      return;
    }
    
    mpu.mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu.mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu.mpu_set_sample_rate(DEFAULT_MPU_HZ);

    signed char gyro_orientation[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};

    dmp_load_state = mpu.dmp_load_motion_driver_firmware();
    if (dmp_load_state)
    {
      Serial.print(dmp_load_state);
      Serial.println("\t");
      Serial.println("DMP load failed!");
      return;
    }

    mpu.dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));

    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_TAP | DMP_FEATURE_GYRO_CAL;
    mpu.dmp_enable_feature(dmp_features);
    mpu.dmp_set_fifo_rate(DEFAULT_MPU_HZ);

    mpu_dmp_state = mpu.mpu_set_dmp_state(1);
    if (mpu_dmp_state)
    {
      Serial.print(mpu_dmp_state);
      Serial.println("\t");
      Serial.println("DMP enable failed");
      return;
    }

    long gyr_bias[3] = { -72, 12, 12};
    mpu.mpu_set_gyro_bias_reg(gyr_bias);

    //long acc_bias[3] = {-1200, -100, -250};
    //mpu01.mpu_set_accel_bias_6050_reg(acc_bias);

    delay(100);
  }
}

void loop() {
  
  current_mpu = current_mpu % 4;
  //Serial.println(current_mpu);
  HW_addr = ncs_l[current_mpu];
  
  if (mpu_init_state || mpu_dmp_state || dmp_load_state)
  {
    Serial.println("Init Failed!");
    return;
  }
  memset(gyro, 0, 6);
  memset(accel, 0, 6);
  memset(quat, 0, 16);
  memset(&sensors, 0, 2);
  memset(&read_r, 0, 1);
  
  read_r = mpu.dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
  if (!read_r && sensors)
  {
    if ((sensors & INV_XYZ_GYRO) && (sensors & INV_XYZ_ACCEL) && (sensors & INV_WXYZ_QUAT))
    {
      //          gyro_f[0] = (float)gyro[0]  / 32768 * 200;
      //          gyro_f[1] = (float)gyro[1]  / 32768 * 200;
      //          gyro_f[2] = (float)gyro[2]  / 32768 * 200;
      Serial.print(current_mpu);
      Serial.print("\t");
      Serial.print(gyro[0]);
      Serial.print("\t");
      Serial.print(gyro[1]);
      Serial.print("\t");
      Serial.print(gyro[2]);

      BluetoothSerial.print(current_mpu);
      BluetoothSerial.print("\t");
      BluetoothSerial.print(gyro[0]);
      BluetoothSerial.print("\t");
      BluetoothSerial.print(gyro[1]);
      BluetoothSerial.print("\t");
      BluetoothSerial.print(gyro[2]);

      //          accel_f[0] = (float)accel[0] / 32768 * 2;
      //          accel_f[1] = (float)accel[1] / 32768 * 2;
      //          accel_f[2] = (float)accel[2] / 32768 * 2;
      Serial.print("\t");
      //Serial.print("# ");
      Serial.print(accel[0]);
      Serial.print("\t");
      Serial.print(accel[1]);
      Serial.print("\t");
      Serial.print(accel[2]);

      BluetoothSerial.print("\t");
      //Serial.print("# ");
      BluetoothSerial.print(accel[0]);
      BluetoothSerial.print("\t");
      BluetoothSerial.print(accel[1]);
      BluetoothSerial.print("\t");
      BluetoothSerial.print(accel[2]);

      //2,147,483,648
      //          quat_f[0] = (float)quat[0] / 2147483647;
      //          quat_f[1] = (float)quat[1] / 2147483647;
      //          quat_f[2] = (float)quat[2] / 2147483647;
      //          quat_f[3] = (float)quat[3] / 2147483647;
      Serial.print("\t");
      Serial.print(quat[0]);
      Serial.print("\t");
      Serial.print(quat[1]);
      Serial.print("\t");
      Serial.print(quat[2]);
      Serial.print("\t");
      Serial.println(quat[3]);

      BluetoothSerial.print("\t");
      BluetoothSerial.print(quat[0]);
      BluetoothSerial.print("\t");
      BluetoothSerial.print(quat[1]);
      BluetoothSerial.print("\t");
      BluetoothSerial.print(quat[2]);
      BluetoothSerial.print("\t");
      BluetoothSerial.println(quat[3]);
    }
    current_mpu ++;
  }

  
}

unsigned short inv_orientation_matrix_to_scalar(signed char *mtx)
{
  unsigned short scalar;
  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;
  return scalar;
}

unsigned short inv_row_2_scale(signed char *row)
{
  unsigned short b;
  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;      // error
  return b;
}

//void gyro_data_ready_cb()
//{
//  Serial.println("Gyro data ready!");
//}
