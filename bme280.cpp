#include "bme280.h"

BME280::BME280(){
    i2c = nullptr;;
    dig_T1 = 0;
    dig_P1 = 0;
    dig_H1 = 0;
    dig_H3 = 0;
    dig_H6 = 0;
    dig_T2 = 0;
    dig_T3 = 0;
    dig_P2 = 0;
    dig_P3 = 0;
    dig_P4 = 0;
    dig_P5 = 0;
    dig_P6 = 0;
    dig_P7 = 0;
    dig_P8 = 0;
    dig_P9 = 0;
    dig_H2 = 0;
    dig_H4 = 0;
    dig_H5 = 0;
    temp_raw = 0;
    pres_raw = 0;
    hum_raw = 0;
    t_fine = 0;
    temp = 0;
    hum = 0;
    pres = 0;
}

BME280::~BME280(){
}

bool BME280::begin(CommUtil *_i2c){
    i2c = _i2c;

    if(isConnected()){
        init();
        delay(10);
        offsetSenser();
    }else{

    return false;
    }

return true;
}

bool BME280::init(){
    if(!i2c->writeByte(BME280_SLAVE_ADDRESS, 0xF5, 0xC0))
        return false;
    delay(10);

    if(!i2c->writeByte(BME280_SLAVE_ADDRESS, 0xF4, 0x27))
        return false;
    delay(10);

    if(!i2c->writeByte(BME280_SLAVE_ADDRESS, 0xF2, 0x1))
        return false;
    delay(10);

return true;
}

void BME280::offsetSenser(){
    offsetTemp();
    offsetPres();
    offsetHum();
}

bool BME280::offsetTemp(){
    uint8_t res[6] = {0};
    if(i2c->readBytes(BME280_SLAVE_ADDRESS, 0x88, 6, res)){
        dig_T1 = (res[1] << 8) | res[0];
        dig_T2 = (res[3] << 8) | res[2];
        dig_T3 = (res[5] << 8) | res[4];

    return true;
    }

return false;
}

bool BME280::offsetPres(){
    uint8_t res[18] = {0};
    if(i2c->readBytes(BME280_SLAVE_ADDRESS, 0x8E, 18, res)){
        dig_P1 = (res[1] << 8) | res[0];
        dig_P2 = (res[3] << 8) | res[2];
        dig_P3 = (res[5] << 8) | res[4];
        dig_P4 = (res[7] << 8) | res[6];
        dig_P5 = (res[9] << 8) | res[8];
        dig_P6 = (res[11] << 8) | res[10];
        dig_P7 = (res[13] << 8) | res[12];
        dig_P8 = (res[15] << 8) | res[14];
        dig_P9 = (res[17] << 8) | res[16];

    return true;
    }

return false;
}

bool BME280::offsetHum(){
    uint8_t res[7] = {0};
    if(i2c->readBytes(BME280_SLAVE_ADDRESS, 0xA1, 1, res)){
        dig_H1 = res[0];

        if(i2c->readBytes(BME280_SLAVE_ADDRESS, 0xE1, 7, res)){
            dig_H2 = (res[1] << 8) | res[0];
            dig_H3 = res[2];
            dig_H4 = (res[3] << 4) | (0x0F & res[4]);
            dig_H5 = (res[5] << 4) | ((res[4] >> 4) & 0x0F);
            dig_H6 = res[6];
    
        return true;
        }
    }

return false;
}

void BME280::updateSenser(){
    uint8_t res[8] = {0};
    if(i2c->readBytes(BME280_SLAVE_ADDRESS, 0xF7, 8, res)){
        pres_raw = (res[0] << 12) | (res[1] << 4) | (res[2] >> 4);
        temp_raw = (res[3] << 12) | (res[4] << 4) | (res[5] >> 4);
        hum_raw  = (res[6] << 8)  | res[7];
    
        calcTemp();
        calcHum();
        calcPres();
    }
}

void BME280::calcTemp(){
    long int var1=0, var2=0;

    var1 = ((((temp_raw >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((temp_raw >> 4) - ((signed long int)dig_T1)) * ((temp_raw>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    temp = (t_fine * 5 + 128) >> 8;
}

void BME280::calcHum(){
    hum = (t_fine - ((signed long int)76800));
    hum = (((((hum_raw << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * hum)) +
          ((signed long int)16384)) >> 15) * (((((((hum * ((signed long int)dig_H6)) >> 10) *
          (((hum * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
          ((signed long int) dig_H2) + 8192) >> 14));
    hum = (hum - (((((hum >> 15) * (hum >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
    hum = (hum < 0 ? 0 : hum);
    hum = (hum > 419430400 ? 419430400 : hum);
    hum = (unsigned long int)(hum >> 12);
}

void BME280::calcPres(){
    long int var1=0, var2=0;

    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);

    if(var1 == 0){
        pres = 0;
    }else{
        pres = (((unsigned long int)(((signed long int)1048576)-pres_raw)-(var2>>12)))*3125;
        if(pres < 0x80000000){
            pres = (pres << 1) / ((unsigned long int) var1);
        }else{
            pres = (pres / (unsigned long int)var1) * 2;
        }
    }

    var1 = (((signed long int)dig_P9) * ((signed long int)(((pres>>3) * (pres>>3))>>13)))>>12;
    var2 = (((signed long int)(pres>>2)) * ((signed long int)dig_P8))>>13;
    pres = (unsigned long int)((signed long int)pres + ((var1 + var2 + dig_P7) >> 4));
}

long int BME280::getTemp(){

return temp / 100;
}

long int BME280::getHum(){

return hum / 1024;
}

unsigned long int BME280::getPres(){

return pres / 100;
}

bool BME280::isConnected() {
    uint8_t res;
    if(i2c->readByte(BME280_SLAVE_ADDRESS, 0xD0, &res))
        if(res == 0x60)
            return true;

return false;
}
