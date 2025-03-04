#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <MCP3428.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include "Display.hpp"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// MCP3428 ADC & Display
MCP3428 MCP(0x68); 
static LGFX lcd;   

float val;

// ROS Configuration
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// micro-ROS Agent Connection States
enum AgentState {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

AgentState agent_state = WAITING_AGENT;

// Function to map float values
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to Draw Bold Text
void drawBoldText(const char *text, int x, int y, uint8_t size, uint16_t color) {
    lcd.setTextSize(size);
    lcd.setTextColor(color, TFT_BLACK);

    // Draw the text multiple times to make it bolder
    lcd.drawString(text, x, y);
    lcd.drawString(text, x + 1, y); // Offset slightly
    lcd.drawString(text, x, y + 1);
    lcd.drawString(text, x - 1, y);
    lcd.drawString(text, x, y - 1);
}

// Error handling loop
void error_loop() {
  while (1) {
    lcd.fillScreen(TFT_BLACK);
    drawBoldText("ROS DISCONNECTED!", lcd.width() / 2, lcd.height() / 2, 2, TFT_RED);
    delay(100);
  }
}

// Timer callback for publishing force value
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg.data = val;  // Publish force value
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

// Create ROS entities
bool create_ros_entities()
{
  const char * node_name = "force_sensor_node";
  const char * ns = "";

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, node_name, ns, &support));

  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "force_sensor_data"));

  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
  return true;
}

// Destroy ROS entities when agent disconnects
void destroy_ros_entities()
{
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  rcl_publisher_fini(&publisher, &node);
}

void setup()
{
    // Initialize Serial & micro-ROS Transport
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    delay(2000);

    Serial.println("MCP3428 Analog to Digital Converter");

    // Initialize Display
    Serial.println("Initializing display...");
    lcd.init();
    lcd.setRotation(1);
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.setFont(&fonts::Font4);  // Use larger font
    drawBoldText("Waiting for Agent...", lcd.width() / 2, lcd.height() / 2, 2, TFT_WHITE);

    Serial.println("Display initialized.");
}

void loop()
{
    // Handle micro-ROS Agent Connection
    switch (agent_state) {
        case WAITING_AGENT:
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                agent_state = AGENT_AVAILABLE;
            }
            break;

        case AGENT_AVAILABLE:
            if (create_ros_entities()) {
                agent_state = AGENT_CONNECTED;
                lcd.fillScreen(TFT_BLACK);
                drawBoldText("ROS Connected!", lcd.width() / 2, lcd.height() / 2, 2, TFT_GREEN);
            } else {
                agent_state = WAITING_AGENT;
            }
            break;

        case AGENT_CONNECTED:
            if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                agent_state = AGENT_DISCONNECTED;
            } else {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;

        case AGENT_DISCONNECTED:
            destroy_ros_entities();
            lcd.fillScreen(TFT_BLACK);
            drawBoldText("ROS Disconnected!", lcd.width() / 2, lcd.height() / 2, 2, TFT_RED);
            agent_state = WAITING_AGENT;
            break;
    }

    // Sensor Reading & Display
    byte error;
    int8_t address = MCP.devAddr;

    // Check if MCP3428 is detected
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
        long Raw_ADC;
        MCP.SetConfiguration(1, 16, 0, 1);
        Raw_ADC = MCP.readADC() * 0.0625; //Convert to mV

        val = constrain(Raw_ADC, 185, 950);
        val = mapfloat(val, 185, 950, 0, 20);

        // Update LCD Display
        lcd.fillScreen(TFT_BLACK);
        lcd.setTextSize(2);

        // Display Force Value in Bold Yellow
        drawBoldText((String(val) + " N").c_str(), lcd.width() / 2, lcd.height() / 3, 2, TFT_YELLOW);

        // Display ADC Value in Bold Green
        drawBoldText(("ADC: " + String(Raw_ADC)).c_str(), lcd.width() / 2, lcd.height() * 2 / 3, 2, TFT_GREEN);
    }
    else
    {
        lcd.fillScreen(TFT_BLACK);
        drawBoldText("MCP3428 Disconnected!", lcd.width() / 2, lcd.height() / 2, 2, TFT_WHITE);
    }

    delay(500);
}
