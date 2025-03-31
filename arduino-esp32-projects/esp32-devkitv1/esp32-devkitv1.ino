#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <U8g2lib.h>

// OLED配置
#define OLED_SCL_PIN 18
#define OLED_SDA_PIN 19
#define OLED_ADDRESS 0x3C  // OLED地址

// MAX30105配置
#define MAX30105_SDA_PIN 21
#define MAX30105_SCL_PIN 22
//#define MAX30105_ADDRESS 0x57  // MAX30105默认地址

// 初始化OLED对象（软件I2C"SW",硬件I2C“HW”）
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(
    U8G2_R0,           // 显示方向
    OLED_SCL_PIN,     // SCL引脚
    OLED_SDA_PIN,     // SDA引脚
    U8X8_PIN_NONE     // 复位引脚（不使用）
);

// 创建 MAX30105 传感器对象
MAX30105 particleSensor;

// 心率测量相关变量
const byte RATE_SIZE = 4;  //数组大小，用于存储多个心率值以进行平均计算
byte rates[RATE_SIZE] = {0};  //存储心率值的数组
byte rateSpot = 0;  //当前存储心率值的数组索引
long lastBeat = 0;  //上一次检测到心跳的时间
float beatsPerMinute = 0;  //即时心率值
int beatAvg = 0;  //平均心率值

// SpO2测量相关变量
#define MAX_BRIGHTNESS 255  // 最大亮度值
uint32_t irBuffer[100];  // 红外光数据缓冲区
uint32_t redBuffer[100];  // 红光和红外光数据缓冲区
int32_t bufferLength = 100;  // 数据缓冲区长度
int32_t spo2 = 0;  // SpO2值
int8_t validSPO2 = 0;  // SpO2有效性标志
int32_t heartRate = 0;  // 心率值
int8_t validHeartRate = 0;  // 心率有效性标志

void setup() {
    Serial.begin(115200);  // 初始化串口通信
    Serial.println("Medical Monitor Initializing...");

    // 初始化OLED
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);  // 设置字体
    u8g2.clearBuffer();  // 清空显示缓冲区
    u8g2.drawStr(0, 20, "Initializing...");
    u8g2.sendBuffer();  // 显示初始化信息

    // 初始化MAX30105
    Wire.begin(MAX30105_SDA_PIN, MAX30105_SCL_PIN);  // 初始化I2C
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 not found. Check wiring!");
        u8g2.clearBuffer();
        u8g2.drawStr(0, 20, "SENSOR ERROR");
        u8g2.sendBuffer();
        while(1);
    }

    // 配置MAX30105
    particleSensor.setup();  // 默认配置
    particleSensor.setPulseAmplitudeRed(0x0A);  // 红光亮度
    particleSensor.setPulseAmplitudeGreen(0);  //  绿色光亮度（关闭）

    // 传感器参数配置
    particleSensor.setup(
        60,          // LED亮度
        4,           // 采样平均
        2,           // 模式（红光+红外）
        100,         // 采样率
        411,         // 脉冲宽度
        4096         // ADC范围
    );

    particleSensor.enableDIETEMPRDY();  // 启用温度传感器
}

void loop() {
    processSensorData();  // 处理传感器数据
    updateOLEDDisplay();  // 更新OLED显示
}

void processSensorData() {
    // 读取原始数据
    long irValue = particleSensor.getIR();

    // 检测心跳
    if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;  // 计算心跳间隔
        lastBeat = millis();  // 更新上次心跳时间
        beatsPerMinute = 60.0 / (delta / 1000.0);  // 计算心率

        if (beatsPerMinute > 20 && beatsPerMinute < 255) {
            rates[rateSpot++] = beatsPerMinute;  // 存储心率值
            rateSpot %= RATE_SIZE;  // 循环存储
            beatAvg = calculateAverage(rates, RATE_SIZE);  // 计算平均心率
        }
    }

    // 计算SpO2
    if (particleSensor.available()) {
        static int bufferIndex = 0;  // 数据缓冲区索引
        redBuffer[bufferIndex] = particleSensor.getRed();  // 红光数据
        irBuffer[bufferIndex] = particleSensor.getIR();  // 红外光数据
        bufferIndex = (bufferIndex + 1) % bufferLength;  // 循环索引

        if (bufferIndex == 0) {  // 数据缓冲区满
            maxim_heart_rate_and_oxygen_saturation(
                irBuffer, bufferLength, redBuffer,  // 红光数据
                &spo2, &validSPO2, &heartRate, &validHeartRate  // 心率数据
            );
        }
    }
}

void updateOLEDDisplay() {  // 更新OLED显示
    u8g2.firstPage();  // 清空显示缓冲区
    do {
        // 心率显示
        u8g2.drawStr(0, 20, "BPM:");
        u8g2.setCursor(40, 20);
        //u8g2.print(beatAvg);
        u8g2.print(validHeartRate ? beatAvg : 0);  // 显示0或无效标志

        // SpO2显示
        u8g2.drawStr(0, 40, "SpO2:");
        u8g2.setCursor(40, 40);
        //u8g2.print(spo2);
        u8g2.print(validSPO2 ? spo2 : 0);  // 显示0或无效标志
        u8g2.print("%");

        // 温度显示
        float temp = particleSensor.readTemperature();  // 读取温度
        u8g2.drawStr(0, 60, "Temp:");
        u8g2.setCursor(40, 60);
        u8g2.print(temp, 1);
        u8g2.print("°C");

    } while (u8g2.nextPage());  // 显示下一页
}

int calculateAverage(byte arr[], int size) {  // 计算平均值
    int sum = 0;
    for (int i = 0; i < size; i++) {  // 遍历数组
        sum += arr[i];  // 累加心率值
    }
    return sum / size;  // 返回平均值
}

