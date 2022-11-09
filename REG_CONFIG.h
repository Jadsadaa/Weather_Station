#define ID_SEMI_1000       1
#define ID_SoilSensor        2
#define ID_CO2_Sensor        3

///////////////////////////////////////////
// Register Power Meter

#define _Wind_Speed  500
#define _Wind_force  501
#define _Wind_Direct_F  502
#define _Wind_Direct_D  503
#define _Hum_Air  504
#define _Temp_Air  505
#define _Noise_SEMI_1000  506
#define _Pm25  507
#define _Pm10  508
#define _atm  509
#define _Lux1  510
#define _Lux2  511
#define _Lux3  512
#define _rainfall  513

#define _Moisture 0
#define _Temp_Soil  1
#define _EC  2
#define _Ph  3
#define _Ni  4
#define _Pho  5
#define _Pot  6


#define _Noise_CO2 502
#define _CO2  503

uint16_t const Address[14] = {
  _Wind_Speed,
  _Wind_force,
  _Wind_Direct_F,
  _Wind_Direct_D,
  _Hum_Air,
  _Temp_Air,
  _Noise_SEMI_1000,
  _Pm25,
  _Pm10,
  _atm,
  _Lux1,
  _Lux2,
  _Lux3,
  _rainfall
};

uint16_t const Address2[7] = {
  _Moisture,
  _Temp_Soil,
  _EC,
  _Ph,
  _Ni,
  _Pho,
  _Pot
};

uint16_t const Address3[2] = {
  _Noise_CO2,
  _CO2
};
