#include <FlexCAN_T4.h> 
const int MOTOR_NUM = 1;
int USB_UART_SPEED = 1000000;
float joint_pos_desired[1]= {0.0};   // desired joint(motor) position [rad]
float motor_pos[MOTOR_NUM]; // rotor position [rad]
float motor_pos_prev[MOTOR_NUM]; // former timestamp rotor position [rad]
int   r_num[MOTOR_NUM]; // number of revolution of rotor
float time_now; // timestamp of the present loop [s]
float time_former; // timestamp of former loop [s]
float delta_t; // loop time difference

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> myCan;

// Set CAN bus message structure as CAN2.0
CAN_message_t msg_recv; // For receiving data on CAN bus
CAN_message_t msg_send; // For sending data on CAN bus
uint16_t joint_can_addr [MOTOR_NUM]={0x141};


void Motor_Init() {
  // Motor position initial CAN bus command
  // All motors rotate to position 0 rad
  msg_send.buf[0] = 0xA3; //CAN bus position command ID
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = 0x00;
  msg_send.buf[5] = 0x00;
  msg_send.buf[6] = 0x00;
  msg_send.buf[7] = 0x00;

  // Motors' IDs range from 0x141 to 0x146.
  // CAN_F is connected to upper body motors(id: 0x141 to 0x146)
  // CAN_B is connected to lower body motors(id: 0x141 to 0x146)
  for (int i = 0; i < MOTOR_NUM; ++i) {
      msg_send.id = joint_can_addr[i];
      myCan.write(msg_send);
  }
  delay(400);
}

void processMotorData(int id) {
  // Receiving motor angle
  // Transfer hex number to rad
  int motor_pos_raw = 0; // Rotor position before devided by gear reduction
  *(uint8_t *)(&motor_pos_raw) = msg_recv.buf[6];
  *((uint8_t *)(&motor_pos_raw)+1) = msg_recv.buf[7];

  motor_pos[id] = (float)motor_pos_raw / 65535.0 * 2 * PI; // 65535(0xFFFF) refers to 2PI
  if (motor_pos[id] - motor_pos_prev[id] < -PI)
    r_num[id] += 1;
  else if (motor_pos[id] - motor_pos_prev[id] > PI)
    r_num[id] -= 1;

  // Calculate shaft angular position [rad]
  motor_pos_prev[id] = motor_pos[id];
  //joint_pos[id] = (motor_pos [id]+ r_num[id] * 2 * PI) ;

  // Calculate shaft velocity [rad/s]
  int motor_vel_raw = 0;
  *(uint8_t *)(&motor_vel_raw) = msg_recv.buf[4];
  *((uint8_t *)(&motor_vel_raw)+1) = msg_recv.buf[5];
  //joint_vel[id] = (float)motor_vel_raw * PI / (180.0 * REDUCTION_RATIO);

  // Calculate motor's current [A], -33A ~ 33A
  int cur_raw = 0;
  *(uint8_t *)(&cur_raw) = msg_recv.buf[2];
  *((uint8_t *)(&cur_raw)+1) = msg_recv.buf[3];
  //joint_cur[id] = (float)cur_raw * 33.0 / 2048.0; // 2048 refers to 33A
}

void Angle_Control_Loop(int motor_id, float pos_command) {
  // Convert motor shaft angle command [rad] to rotor angle command [degree]
  //pos_command = pos_command * 180.0 * REDUCTION_RATIO / PI;
  // see motor manual p12 (0xA3)
  int32_t pos = (int32_t)round(pos_command / 0.01);
  
  // Motor position command is clockwise
  // 0x00000001 - 0x80000000 counter_clockwise
  // 0x80000001 - 0xffffffff clockwise
  // 0x00000000 position 0
  // if (pos < 0)
  //   pos = 0x100000000 + pos;

  // unsigned int pos_1 = pos & 0xff;
  // unsigned int pos_2 = (pos >> 8) & 0xff;
  // unsigned int pos_3 = (pos >> 16) & 0xff;
  // unsigned int pos_4 = (pos >> 24) & 0xff;

  // Set the CAN message ID as 0x200
  msg_send.id = joint_can_addr[motor_id];
  msg_send.buf[0] = 0xA3;
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = *(uint8_t*)(&pos);
  msg_send.buf[5] = *((uint8_t*)(&pos)+1);
  msg_send.buf[6] = *((uint8_t*)(&pos)+2);;
  msg_send.buf[7] = *((uint8_t*)(&pos)+3);;


  myCan.write(msg_send); // Write the message on CAN bus
    while (true) {
      if (myCan.read(msg_recv)) {
        processMotorData(motor_id);
        break;
      }
    }
}


void setup() {
  Serial.begin(USB_UART_SPEED);
  // Turn on the LED on Teensy to indicate IMU is initializing
  pinMode(13, OUTPUT);  

  for (int i = 0; i < 10; ++i) {
      digitalWrite(13, HIGH);
      delay(500);
      digitalWrite(13, LOW);
      delay(500);
  }



  //pinMode(0,INPUT);
  //pinMode(1,OUTPUT);
  pinMode(22,OUTPUT);
  pinMode(23,INPUT);

  myCan.begin();
  myCan.setBaudRate(1000000);
  myCan.setClock(CLK_60MHz);


  Motor_Init();

  time_former = (float)micros();
}



void loop() {
  // put your main code here, to run repeatedly:
  time_now = (float)micros();
  delta_t = (time_now - time_former) / 1000000.0;
  time_former = time_now;


  //Angle_Control_Loop(0, 60);
  int32_t pos = (int32_t)round(60 / 0.01);
  msg_send.id = 0x141;
  msg_send.buf[0] = 0xA3;
  msg_send.buf[1] = 0x00;
  msg_send.buf[2] = 0x00;
  msg_send.buf[3] = 0x00;
  msg_send.buf[4] = *(uint8_t*)(&pos);
  msg_send.buf[5] = *((uint8_t*)(&pos)+1);
  msg_send.buf[6] = *((uint8_t*)(&pos)+2);;
  msg_send.buf[7] = *((uint8_t*)(&pos)+3);;
  while (true) {
      if (myCan.read(msg_recv)) {
        processMotorData(0);
        break;
      }
    }
  

}
