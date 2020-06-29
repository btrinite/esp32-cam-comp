
#include "HardwareSerial.h"
#include "esp_camera.h"
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

#include "SBUS.h"
#include <Adafruit_NeoPixel.h>

#define ROSSERIAL_BAUD 1500000
#define ROSSERIAL_OVERRIDE_SERIAL_CLASS &Serial
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_brain_state.h>

//LED
#define BUILT_IN_LED               33   // built in ESP32 Led
#define GPIO_OUTPUT_LED            2
#define LED_COUNT 6
Adafruit_NeoPixel statusLed (LED_COUNT, GPIO_OUTPUT_LED, NEO_GRB + NEO_KHZ800);

void led_gpio_initialize (void) {
  pinMode(BUILT_IN_LED, OUTPUT);
  pinMode(GPIO_OUTPUT_LED, OUTPUT);
}

//PWM

#define PWM_RC_THROTTLE_OUTUT_PIN 13   //Set GPIO 13 as PWM0A
#define PWM_RC_STEERING_OUTUT_PIN 15   //Set GPIO 15 as PWM1A

#define PWM_FREQ 50
const int throttleChannel = 14;
const int steeringChannel = 15;

// Throttle and steering output order
int cmd_throttle = 1500;
int cmd_steering = 1500;

static void pwm_gpio_initialize()
{
    ledcAttachPin(PWM_RC_THROTTLE_OUTUT_PIN, throttleChannel);
    ledcAttachPin(PWM_RC_STEERING_OUTUT_PIN, steeringChannel);

    ledcSetup(throttleChannel, PWM_FREQ, 8);
    ledcSetup(steeringChannel, PWM_FREQ, 8);
}


static void mcpwm_set_throttle_pwm(int pwm_width_in_us)
{
  const int dutyCycle = (255*pwm_width_in_us)/(1000000/PWM_FREQ);
  ledcWrite(throttleChannel, dutyCycle);
}

static void mcpwm_set_steering_pwm(int pwm_width_in_us)
{
  const int dutyCycle = (255*pwm_width_in_us)/(1000000/PWM_FREQ);
  ledcWrite(steeringChannel, dutyCycle);
}

void steering_cb( const robocars_msgs::robocars_actuator_output& steering_msg){
  mcpwm_set_steering_pwm (steering_msg.pwm);
}

void throttling_cb( const robocars_msgs::robocars_actuator_output& throttling_msg){
  mcpwm_set_throttle_pwm (throttling_msg.pwm);
}

// State
void processStatusFromHost (int status) {

  switch(status) {
    case robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE:
    break;
    case robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING:
    break;
    case robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING:
    break;
  }  
}

void state_msg_cb( const robocars_msgs::robocars_brain_state& state_msg){
  processStatusFromHost(state_msg.state);
}


// ROS
ros::NodeHandle  nh;
sensor_msgs::Image image_msg;  
ros::Publisher pub_image( "/esp32cam/raw_image_jpeg", &image_msg);
ros::Subscriber<robocars_msgs::robocars_actuator_output> steering_sub("steering_ctrl/output", steering_cb);
ros::Subscriber<robocars_msgs::robocars_actuator_output> throttling_sub("throttling_ctrl/output", throttling_cb);
ros::Subscriber<robocars_msgs::robocars_brain_state> state_sub("robocars_brain_state", &state_msg_cb );

//camera

camera_config_t camera_config = {
  .pin_pwdn  = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 10000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_XGA,//QQVGA-QXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = 12, //0-63 lower number means higher quality
  .fb_count = 2 //if more than one, i2s runs in continuous mode. Use only with JPEG
};

const char * const pixFormat2Encoding[] = {"2BPP/RGB565", "2BPP/YUV422", "1BPP/GRAYSCALE", "PEG/COMPRESSED", "rgb8", "RAW", "3BP2P/RGB444", "3BP2P/RGB555"  };

esp_err_t camera_init() {

  //power up the camera if PWDN pin is defined
  if (PWDN_GPIO_NUM != -1) {
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, LOW);
  }

  if(psramFound()){
    camera_config.frame_size = FRAMESIZE_UXGA;
    camera_config.jpeg_quality = 10;
    camera_config.fb_count = 2;
  } else {
    camera_config.frame_size = FRAMESIZE_SVGA;
    camera_config.jpeg_quality = 12;
    camera_config.fb_count = 1;
  }
  
  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera Init Failed\n");
    return err;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QQVGA);

  return ESP_OK;
}


void ROS_LOG (const char * msg) {
  nh.loginfo(msg);
}
 

esp_err_t camera_capture() {
  static uint seq=0;
  
  //acquire a frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    ROS_LOG ("Camera Capture Failed");
    return ESP_FAIL;
  }
  image_msg.header.seq=seq++;
  image_msg.height = fb->height;
  image_msg.width = fb->width;
  image_msg.step = fb->len;
  image_msg.encoding = "jpeg";
  image_msg.data=fb->buf;
  image_msg.data_length=fb->len;

  pub_image.publish(&image_msg);

  //return the frame buffer back to the driver for reuse
  esp_camera_fb_return(fb);
  return ESP_OK;
}

// Radio part
SBUS x8r(Serial2, 12); //mapped to GPIO12

uint16_t channels[16];
bool failSafe;
bool lostFrame;

robocars_msgs::robocars_radio_channels channels_msg;  
ros::Publisher pub_channels( "radio_channels", &channels_msg);

void switchLight() {
  statusLed.setPixelColor(0, 128, 128, 128);
  statusLed.setPixelColor(5, 128, 128, 128);
  statusLed.setPixelColor(1, 128, 0, 0);
  statusLed.setPixelColor(2, 128, 0, 0);
  statusLed.setPixelColor(3, 128, 0, 0);
  statusLed.setPixelColor(4, 128, 0, 0);
  statusLed.show();
}
void setup() {

  // Init PWM output logic and set to default 1500 us (idle)
  pwm_gpio_initialize();
  led_gpio_initialize();

  digitalWrite(BUILT_IN_LED, LOW);

  // Init Camera
  camera_init();

  mcpwm_set_throttle_pwm (1500);
  mcpwm_set_steering_pwm (1500);

  //Init SBUS
  x8r.begin();

  // LED init
  statusLed.begin();
  statusLed.show();
  
  // Init ROS
  nh.initNode();
  nh.advertise(pub_image);
  nh.advertise(pub_channels);

  switchLight();
  
  digitalWrite(BUILT_IN_LED, HIGH);
  ROS_LOG("Starting ESP32-CAM");  

}


void loop() {

//  while (!nh.connected()) {
//    nh.spinOnce();
//  }
  
  // look for a good SBUS packet from the receiver
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    // write the SBUS packet to an SBUS compatible servo
    if (failSafe == 1) {
      channels_msg.ch1=0;
      channels_msg.ch2=0;
      channels_msg.ch3=0;
      channels_msg.ch4=0;
      channels_msg.ch5=0;
      channels_msg.ch6=0;
    } else {
      channels_msg.ch1=channels[0];
      channels_msg.ch2=channels[1];
      channels_msg.ch3=channels[2];
      channels_msg.ch4=channels[3];
      channels_msg.ch5=channels[4];
      channels_msg.ch6=channels[5];
    }
    pub_channels.publish(&channels_msg);
  } else {
  }
  camera_capture();
  nh.spinOnce(); 
}
