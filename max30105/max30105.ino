#include <SPI.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <U8g2lib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "image.h"

// OLED配置
#define OLED_SCL_PIN 22
#define OLED_SDA_PIN 21
#define OLED_ADDRESS 0x3C  // OLED地址

// MAX30105配置
#define MAX30105_SDA_PIN 15
#define MAX30105_SCL_PIN 2

// 初始化OLED对象
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
    U8G2_R0,           // 显示方向
    OLED_SCL_PIN,     // SCL引脚
    OLED_SDA_PIN,     // SDA引脚
    U8X8_PIN_NONE     // 复位引脚（不使用）
);

// 创建 MAX30105 传感器对象
MAX30105 particleSensor;

// SpO2测量相关变量
uint32_t irBuffer[200];  // 红外光数据缓冲区
uint32_t redBuffer[200];  // 红光和红外光数据缓冲区
int32_t bufferLength = 100;  // 数据缓冲区长度
int32_t spo2;  // SpO2值
int8_t validSPO2;  // SpO2有效性标志
int32_t heartRate;  // 心率值
int8_t validHeartRate;  // 心率有效性标志
float temperature;  // 温度

// 中值滤波函数，返回中值
int32_t medianFilter(int32_t data[], int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (data[i] > data[j]) {
                int32_t temp = data[i];
                data[i] = data[j];
                data[j] = temp;
            }
        }
    }
    return data[size / 2];
}

// 读取传感器数据到缓冲区
void readSensorData(uint32_t red[], uint32_t ir[], int length) {
    for (byte i = 0; i < length; i++) {
        int timeout = 0;
        while (particleSensor.available() == false) {
            particleSensor.check();
            timeout++;
            if (timeout > 1000) {
                Serial.println("Sensor read timeout!");
                return;
            }
        }
        red[i] = particleSensor.getRed();
        ir[i] = particleSensor.getIR();
        particleSensor.nextSample();
    }
}

void processSensorData() {
    temperature = particleSensor.readTemperature();
    bufferLength = 100;
    Serial.println("Loading data...");

    readSensorData(redBuffer, irBuffer, bufferLength);

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    for (byte i = 25; i < 100; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
    }

    readSensorData(&redBuffer[75], &irBuffer[75], 25);

    if (irBuffer[99] < 50000) {
        Serial.println("No finger detected.");
        return;
    }

    // 中值滤波处理血氧饱和度数据
    int32_t spo2Buffer[5];
    for (int i = 0; i < 5; i++) {
        maxim_heart_rate_and_oxygen_saturation(
            irBuffer, bufferLength, redBuffer, &spo2Buffer[i], &validSPO2, &heartRate, &validHeartRate
        );
    }
    spo2 = medianFilter(spo2Buffer, 5);

    maxim_heart_rate_and_oxygen_saturation(
        irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate
    );
}


void OledDisplay() {  // 更新OLED显示
    u8g2.firstPage();  // 清空显示缓冲区
    do {
        // 心率显示
        u8g2.setCursor(0, 15);
        u8g2.print("心率:");
        u8g2.setCursor(40, 15);
        u8g2.print(validHeartRate ? heartRate : 0);

        // SpO2显示
        u8g2.setCursor(0, 30);
        u8g2.print("血氧:");
        u8g2.setCursor(40, 30);
        u8g2.print(validSPO2 ? spo2 : 0);  // 显示0或无效标志
        u8g2.print("%");

        // 温度显示
        u8g2.setCursor(0, 45);
        u8g2.print("温度:");
        u8g2.setCursor(40, 45);
        u8g2.print(temperature, 2);
        u8g2.print("°C");

    } while (u8g2.nextPage());  // 显示下一页
}

void setup() {
    Serial.begin(115200);  // 初始化串口通信
    // 显示图片
    FirstShowImage();
    // 初始化OLED
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_wqy12_t_gb2312);  // 设置字体
    u8g2.clearBuffer();  // 清空显示缓冲区
    u8g2.setCursor(0, 15);
    u8g2.print("初始化中...");
    u8g2.sendBuffer();  // 显示初始化信息
    delay(2000);
    // 初始化MAX30105
    Wire.begin(MAX30105_SDA_PIN, MAX30105_SCL_PIN);  // 初始化I2C
    if (!particleSensor.begin()) {
        Serial.println("MAX30105 not found!");
        for (int i = 10; i > 0; i--) {
            u8g2.clearBuffer();
            u8g2.setCursor(0, 15);
            u8g2.print("未检测到传感器！");
            u8g2.setCursor(0, 30);
            u8g2.print("肿么肥事(>_<)``");
            u8g2.setCursor(0, 45);
            u8g2.print("还有" + String(i) + "秒钟爆炸");
            u8g2.setCursor(0, 60);
            u8g2.print("靠近一点哦~");
            u8g2.sendBuffer();
            delay(1000);
        }
        unsigned long previousMillis = 0;
        unsigned long showStartMillis = 0;
        const long interval = 5000;  // 5秒的间隔
        const long showDuration = 3000;  // 3秒的显示时长
        bool isShowing = false;
        while (1) {
            unsigned long currentMillis = millis();
            if (!isShowing && currentMillis - previousMillis >= interval) {
                previousMillis = currentMillis;
                showStartMillis = currentMillis;
                isShowing = true;
                u8g2.clearBuffer();
                u8g2.setCursor(0, 15);
                u8g2.print("传感器未连接！");
                u8g2.setCursor(0, 30);
                u8g2.print("检查插线！");
                u8g2.setCursor(0, 45);
                u8g2.print("SDA:15; SCL:2");
                u8g2.sendBuffer();
            }
            if (isShowing && currentMillis - showStartMillis >= showDuration) {
                isShowing = false;
                u8g2.clearBuffer();
                u8g2.sendBuffer();
            }
        }
    } else { 
        u8g2.clearBuffer();
        u8g2.setCursor(0, 15);
        u8g2.print("请把手放按压在传感器");
        u8g2.setCursor(0, 30);
        u8g2.print("上并等待大约10秒...");
        u8g2.setCursor(0, 45);
        u8g2.print("耐心等待喵(=^-ω-^=)");
        u8g2.sendBuffer();
    }

    // 配置MAX30105
    particleSensor.setup();  // 默认配置
    particleSensor.enableDIETEMPRDY();  // 启用温度传感器

}

void loop() {
    processSensorData();  // 处理传感器数据
    OledDisplay();  // 更新OLED显示
    delay(1000);
}