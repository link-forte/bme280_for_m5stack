#ifndef _BME280_H_
#define _BME280_H_

#include <M5Stack.h>
#define BME280_SLAVE_ADDRESS (0x77)

class BME280 {
    public:
        BME280();
        ~BME280();
        bool begin(CommUtil *_i2c);
        bool init();
        void offsetSenser();
        bool offsetTemp();
        bool offsetPres();
        bool offsetHum();
        void updateSenser();
        void calcTemp();
        void calcHum();
        void calcPres();
        long int getTemp(); // -40〜85 'C
        long int getHum(); // 0〜100％RH
        unsigned long int getPres(); // 30,000 Pa〜110,000 Pa
        bool isConnected();
    private:
        CommUtil *i2c;
        uint16_t dig_T1;
        uint16_t dig_P1;
        uint16_t dig_H1;
        uint16_t dig_H3;
        uint16_t dig_H6;

        int16_t dig_T2;
        int16_t dig_T3;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
        int16_t dig_H2;
        int16_t dig_H4;
        int16_t dig_H5;

        unsigned long int temp_raw;
        unsigned long int pres_raw;
        unsigned long int hum_raw;
        long int t_fine;

        long int temp;
        long int hum;
        unsigned long int pres;
};

#endif