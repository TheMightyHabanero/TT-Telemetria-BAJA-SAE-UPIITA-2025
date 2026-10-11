
bool aux_inicializacion_interfaz = false;
bool aux_run_stop_actualizado = false;

//float pitch = atan2(ax1, sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI;
/////////////  ESPNOW  ////////////
#include <WiFi.h>
#include <esp_now.h>

#define BUFFER_SIZE 512
char incomingString[BUFFER_SIZE];

// Variables para almacenar los datos recibidos
// Variables para almacenar los nuevos datos
float ax1, ay1, az1, vel;
int rpm;
float lat, lon, alt;
long dia, hora_espnow, min_espnow, seg_espnow;
//long lat, lon, alt, dia, hora_espnow, min_espnow, seg_espnow;

// Versión compatible con ESP-IDF v5+ (ESP32 core 3.x.x+)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len >= BUFFER_SIZE) {
    return;
  }

  memcpy(incomingString, data, len);
  incomingString[len] = '\0';  // Terminador de cadena
  
  // Variables temporales para datos GPS crudos
  long lat_raw, lon_raw, alt_raw;

  // Extraer todos los datos del paquete
  sscanf(incomingString, "%f,%f,%f,%f,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
         &ax1, &ay1, &az1, &vel, &rpm,
         &lat_raw, &lon_raw, &alt_raw,
         &dia, &hora_espnow, &min_espnow, &seg_espnow);

  /*// Extraer los datos del nuevo formato
  sscanf(incomingString, "%f,%f,%f,%f,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
         &ax1, &ay1, &az1, &vel, &rpm, &lat, &lon, &alt, &dia, &hora_espnow, &min_espnow, &seg_espnow);*/

  // Convertir inmediatamente a formato real
  lat = lat_raw / 10000000.0f;
  lon = lon_raw / 10000000.0f;
  alt = alt_raw / 1000.0f;

  /*Serial.println("Datos parseados:");
  Serial.print("lat_raw: "); Serial.println(lat_raw);
  Serial.print("lon_raw: "); Serial.println(lon_raw);
  Serial.print("alt_raw: "); Serial.println(alt_raw);*/

  /*
  Serial.print("ax1: "); Serial.println(ax1);
  Serial.print("ay1: "); Serial.println(ay1);
  Serial.print("az1: "); Serial.println(az1);
  Serial.print("vel: "); Serial.println(vel);
  Serial.print("rpm: "); Serial.println(rpm);
  Serial.print("lat: "); Serial.println(lat);
  Serial.print("lon: "); Serial.println(lon);
  Serial.print("alt: "); Serial.println(alt);
  Serial.print("dia: "); Serial.println(dia);
  Serial.print("hora: "); Serial.println(hora);
  Serial.print("min: "); Serial.println(min);
  Serial.print("seg: "); Serial.println(seg);
  */
}
/////////////  ESPNOW  ////////////

/////////////  NEOPIXEL  ////////////
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel tira = Adafruit_NeoPixel(8, 5, NEO_GRB + NEO_KHZ800);
int R = 0, G = 0, B = 0;
int rpm_NEO = 1;
long RPM_R = 0;
/////////////  NEOPIXEL  ////////////


//////////////////PANTALLA
// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

//Librerías necesarias para el display Eink
#include <GxEPD2_BW.h>  //Pantalla Blanco y Negro
//#include <GxEPD2_3C.h>  //Pantalla Colores



//Cargamos las fuentes para usarlas
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include <Fonts/FreeSerifItalic18pt7b.h>
#include <Fonts/FreeSerifItalic24pt7b.h>

//PINOUT DE LA PANTALLA

// ESP32 CS(SS)=7, SCL(SCK/CLK)=4, SDA(MOSI/DIN)=6, BUSY=1,RES(RST)=2,DC=3

//CREAMOS UN OBJETO "TIPO" DISPLAY
// 2.9'' EPD Module
GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ 7, /*DC=*/ 3, /*RES=*/ 2, /*BUSY=*/ 1)); // DEPG0290BS 128x296, SSD1680
//////////////////PANTALLA


///////// BOTONES
// Definir pines de botones
#define BTN_1  0  // Botón para avanzar pantalla
#define BTN_2  10  // Botón para retroceder pantalla

#define BTN_3  20  // Botón para avanzar pantalla
#define BTN_4  21  // Botón para retroceder pantalla


// Variables para almacenar el estado anterior de los botones
bool last_BTN_1_State = HIGH;
bool last_BTN_2_State = HIGH;

bool last_BTN_3_State = HIGH;
bool last_BTN_4_State = HIGH;

////////////BOTONEs


//////////////// FUNCIONES PANTALLA //////////////
float checkpoint_lat = 0.0f;
float checkpoint_lon = 0.0f;

bool checkpoint_pasado = false;

unsigned long tiempo_inicio_vuelta = 0;
unsigned long tiempo_total = 0;
unsigned long ultima_vuelta = 0;
unsigned long mejor_tiempo = 0;

int vuelta_crono_min = 0;
int vuelta_crono_seg = 0;

int vuelta_actual = 0;
////////////Distancia
float CalculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float RADIO_TIERRA = 6371000.0f;  // Radio de la Tierra en metros

  // Convertir grados a radianes
  float radLat1 = lat1 * M_PI / 180.0f;
  float radLon1 = lon1 * M_PI / 180.0f;
  float radLat2 = lat2 * M_PI / 180.0f;
  float radLon2 = lon2 * M_PI / 180.0f;

  float dLat = radLat2 - radLat1;
  float dLon = radLon2 - radLon1;

  float a = sinf(dLat / 2) * sinf(dLat / 2) +
            cosf(radLat1) * cosf(radLat2) *
            sinf(dLon / 2) * sinf(dLon / 2);

  float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
  float distancia = RADIO_TIERRA * c;

  return distancia;  // En metros
  //float distancia = CalculateDistance(checkpoint_lat, checkpoint_lon, lat, lon);
}

void SetCheckPoint () {
  checkpoint_lat = lat;
  checkpoint_lon = lon;
  //Reset a todo
  vuelta_actual = 0;
  tiempo_total = 0;
  mejor_tiempo = 0;
  ultima_vuelta = 0;
  checkpoint_pasado = false;
}

void Manejo_vueltas (float lat_actual, float lon_actual) {
  float distancia = CalculateDistance(checkpoint_lat, checkpoint_lon, lat_actual, lon_actual);
  bool cerca_del_checkpoint = (distancia <= 10.0f); //10.0f exterior & 0.2 interior

  if (cerca_del_checkpoint && !checkpoint_pasado) {
    unsigned long ahora = millis();

    if (vuelta_actual == 0) {
      // Primera pasada por el checkpoint: solo inicia cronómetro
      tiempo_inicio_vuelta = ahora;
      vuelta_actual = 1;
      checkpoint_pasado = true;
      // Preparado para inicar primera vuelta.
      return;
    }

    // Segunda vuelta en adelante
    unsigned long duracion_vuelta = ahora - tiempo_inicio_vuelta;
    //vuelta_crono_min = duracion_vuelta / 1000 / 60;
    //vuelta_crono_seg = (duracion_vuelta / 1000) % 60;
    ultima_vuelta = duracion_vuelta;
    //tiempo_total += duracion_vuelta;
    tiempo_inicio_vuelta = ahora;
    vuelta_actual++;

    if (mejor_tiempo == 0 || duracion_vuelta < mejor_tiempo) {
      mejor_tiempo = duracion_vuelta;
    }

    checkpoint_pasado = true;

    //Vuelta Completada
  }

  else if (!cerca_del_checkpoint) {
    checkpoint_pasado = false;
  }
}


////////////Distancia

//////////// TIRA LED //////////
void NEO_PXL(int RPM_R) {
  //RPM_R = random(1800,3600);
  //int aux = map(RPM_R, 1800, 3600, 1, 8);
  int aux = RPM_R;
  tira.clear();
  for (int i = 0; i<aux; i++){
    // put your main code here, to run repeatedly:
    if (i<2){
      R=255;
      G=255;
      B=0;
    } else if(i<4){
      R=0;
      G=255;
      B=0;
    } else if(i<6){
      R=255;
      G=0;
      B=0;
    } else {
      R=0;
      G=0;
      B=255;
    }
    //tira.setBrightness(5); //Brillo global para toda la tira (0-255)
    tira.setPixelColor(i,R, G, B); //Pixel, R, G, B
    //tira.show();
    //delay(500);
  }
  tira.setBrightness(5);
  tira.show();
  //tira.clear(); // Limpiar leds
  //delay(500);
}
//////////// TIRA LED //////////

////// SELECCION DE MODO
void select_mode_text(int select_mode) {
  display.setRotation(1);
  display.setFont(&FreeMonoBold12pt7b);
  //display.setFont(&FreeMonoBold24pt7b);
  display.setTextColor(GxEPD_BLACK);

  //uint16_t speed_w = display.width();
  uint16_t start_w = 150;
  uint16_t start_h = 40; // Altura ajustada para el encabezado

  //uint16_t speed_x = 148;
  uint16_t start_x = 100;
  uint16_t start_y = 70;
  
  uint16_t aux_y = start_y + start_h;

  // Región de actualización parcial para el encabezado
  display.setPartialWindow(start_x, start_y, start_w, start_h);

  display.firstPage();
  do {
    display.fillRect(start_x, start_y, start_w, start_h, GxEPD_WHITE); // Limpiar la región del encabezado
    //display.setCursor(120, 50);
    display.setCursor(start_x, aux_y);
    //Imprimir Leyenda del modo
    if (select_mode==1){
      display.print("Aceleracion");
    }else if (select_mode==2){
      display.print("Endurance");
    } else if (select_mode==3){
      display.print("Maniobrabilidad");
    } else if (select_mode==4){
      display.print("Suspension");
    } else if (select_mode==5){
      display.print("Traccion");
    } else {
      display.print("Error");
    }

  } while (display.nextPage());

}
////// SELECCION DE MODO

////// Aceleracion
void Aceleracion_inicializacion () {
  display.setRotation(1);
  display.setFont(&FreeMonoBold18pt7b);
  display.setTextColor(GxEPD_BLACK);

  display.setFullWindow();  //Asegura que toda la pantalla este disponible para escribir

  //int km_x = 200; // Ajusta según el ancho de la pantalla
  int km_x = 175;
  int km_y = 100;  // Ajusta para alinearlo con la velocidad

  display.firstPage();
  do {
    display.setCursor(km_x, km_y);
    display.print("Km/h");
  } while (display.nextPage());
}

void Aceleracion_Velocidad () {
  display.setRotation(1);
  //display.setFont(&FreeMonoBold12pt7b);
  display.setFont(&FreeMonoBold24pt7b);
  display.setTextColor(GxEPD_BLACK);

  //uint16_t speed_w = display.width();
  uint16_t speed_w = 55;
  uint16_t speed_h = 40; // Altura ajustada para el encabezado

  //uint16_t speed_x = 148;
  uint16_t speed_x = 120;
  uint16_t speed_y = 60;

  // Región de actualización parcial para el encabezado
  display.setPartialWindow(speed_x, speed_y, speed_w, speed_h);

  display.firstPage();
  do {
    display.fillRect(speed_x, speed_y, speed_w, speed_h, GxEPD_WHITE); // Limpiar la región del encabezado
    //display.setCursor(148, 38);
    display.setCursor(120, 100);

    // Mostrar el valor de vel con 1 decimal
    char temp_str[10];
    dtostrf(vel, 4, 1, temp_str);  // ancho mínimo 4, 1 decimal
    display.print(temp_str);
    
    /*// Mostrar el valor en la caja
      char r_str[10];
      int r_aux = myData.b;
      sprintf(r_str, "%d", r_aux);
    display.print(r_str);*/

  } while (display.nextPage());
}

void Aceleracion_run_stop_text (bool start_run) {
  display.setRotation(1);
  display.setFont(&FreeSerifItalic24pt7b);
  display.setTextColor(GxEPD_BLACK);

  uint16_t start_w = 120;
  uint16_t start_h = 40;

  uint16_t start_x = 120;
  uint16_t start_y = 10;

  // Región de actualización parcial para el encabezado
  display.setPartialWindow(start_x, start_y, start_w, start_h);

  display.firstPage();
  do {
    display.fillRect(start_x, start_y, start_w, start_h, GxEPD_WHITE);
    //display.setCursor(start_x, aux_y);
    display.setCursor(120, 50);
    display.print(start_run ? "RUN!" : "STOP");
  } while (display.nextPage());

}

void Aceleracion_nivel(float pitch) {
  display.setPartialWindow(0, 0, 100, 100);  // Área de visualización de la burbuja
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.drawCircle(50, 50, 40, GxEPD_BLACK);  // Círculo centrado

    // Limita el pitch entre -50° y +50° para que no se salga del rango visual
    pitch = constrain(pitch, -50, 50);

    float angle = pitch * PI / 180.0;  // Convierte a radianes

    // Calcula extremos de la línea según el ángulo
    int x1 = 50 - 40 * cos(angle);
    int y1 = 50 + 40 * sin(angle);
    int x2 = 50 + 40 * cos(angle);
    int y2 = 50 - 40 * sin(angle);
    
    display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);

    // Opcional: muestra valor numérico del pitch
    /*display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(5, 95);
    display.print("Pitch:");
    display.setCursor(60, 95);
    display.print((int)pitch);
    display.print((char)0xB0);  // Símbolo de grados*/
  } while (display.nextPage());
}
/*void Aceleracion_nivel (int value) {
  display.setPartialWindow(0, 0, 100, 100);  // Ajusta si es necesario
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.drawCircle(50, 50, 40, GxEPD_BLACK);  // Nuevo centro (50, 50)
    float angle = map(value, 0, 100, -50, 50) * PI / 180.0;

    int x1 = 50 - 40 * cos(angle);  // Calculado desde el nuevo centro
    int y1 = 50 + 40 * sin(angle);
    int x2 = 50 + 40 * cos(angle);
    int y2 = 50 - 40 * sin(angle);
    
    display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);
  } while (display.nextPage());
}*/
////// Aceleracion

////// Endurance
void Endurance_Inicializacion_1 () {
  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);

  display.setFullWindow();  // Asegura que toda la pantalla esté disponible para escribir

  // Coordenadas aproximadas para cada texto fijo
  int lap_x   = 114;  // Centrado arriba
  int lap_y   = 25;

  int best_x  = 22;   // A la izquierda
  int best_y  = 86;

  int last_x  = 220;  // A la derecha
  int last_y  = 86;

  int kmh_x   = 140;  // Centrado vertical
  int kmh_y   = 63;

  display.firstPage();
  do {
    display.setCursor(lap_x, lap_y);
    display.print("LAP");

    display.setCursor(best_x, best_y);
    display.print("BEST");

    display.setCursor(last_x, last_y);
    display.print("LAST");

    display.setFont(&FreeSerifItalic18pt7b);
    display.setCursor(kmh_x, kmh_y);
    display.print("km/h");

  } while (display.nextPage());
}
void Endurance_Num_Vuelta (int lap) {
  display.setRotation(1);
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

  //uint16_t speed_w = display.width();
  uint16_t lap_w = 40;
  uint16_t lap_h = 20; // Altura ajustada para el encabezado

  //uint16_t speed_x = 148;
  uint16_t lap_x = 156;
  uint16_t lap_y = 5;

  uint16_t aux_y = lap_y + lap_h;

  // Región de actualización parcial para el encabezado
  display.setPartialWindow(lap_x, lap_y, lap_w, lap_h);

  display.firstPage();
  do {
    display.fillRect(lap_x, lap_y, lap_w, lap_h, GxEPD_WHITE); // Limpiar la región del encabezado
    display.setCursor(lap_x, aux_y);
    display.print(lap);
  } while (display.nextPage());
}

void Endurance_Cronometro (int minutos, int segundos) {
  display.setRotation(1);
  display.setFont(&FreeMonoBold18pt7b);
  display.setTextColor(GxEPD_BLACK);

  uint16_t crono_w = 120;
  uint16_t crono_h = 35;
  uint16_t crono_x = 93;
  uint16_t crono_y = 73;
  uint16_t aux_y = crono_y + crono_h;

  display.setPartialWindow(crono_x, crono_y, crono_w, crono_h);

  char buffer[6];
  sprintf(buffer, "%02d:%02d", minutos, segundos);

  display.firstPage();
  do {
    display.fillRect(crono_x, crono_y, crono_w, crono_h, GxEPD_WHITE);
    display.setCursor(crono_x, aux_y);
    display.print(buffer);
  } while (display.nextPage());
}

void Endurance_Velocidad () {
  display.setRotation(1);
  display.setFont(&FreeSerifItalic18pt7b);
  display.setTextColor(GxEPD_BLACK);

  //uint16_t speed_w = display.width();
  uint16_t speed_w = 60;
  uint16_t speed_h = 30; // Altura ajustada para el encabezado

  //uint16_t speed_x = 148;
  uint16_t speed_x = 75;
  uint16_t speed_y = 36;

  uint16_t aux_y = speed_y + speed_h;

  // Región de actualización parcial para el encabezado
  display.setPartialWindow(speed_x, speed_y, speed_w, speed_h);

  display.firstPage();
  do {
    display.fillRect(speed_x, speed_y, speed_w, speed_h, GxEPD_WHITE); // Limpiar la región del encabezado
    //display.setCursor(148, 38);
    display.setCursor(speed_x, aux_y);

    // Mostrar el valor de vel con 1 decimal
    char temp_str[10];
    dtostrf(vel, 4, 0, temp_str);  // ancho mínimo 4, 1 decimal
    display.print(temp_str);

  } while (display.nextPage());
}

void Endurance_Best (int minutos, int segundos) {

  /*unsigned long segundos_totales = mejor_tiempo / 1000;
  minutos = segundos_totales / 60;
  segundos = segundos_totales % 60;*/

  display.setRotation(1);
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

  uint16_t crono_w = 70;
  uint16_t crono_h = 20;
  uint16_t crono_x = 10;
  uint16_t crono_y = 94;
  uint16_t aux_y = crono_y + crono_h;

  display.setPartialWindow(crono_x, crono_y, crono_w, crono_h);

  char buffer[6];
  sprintf(buffer, "%02d:%02d", minutos, segundos);

  display.firstPage();
  do {
    display.fillRect(crono_x, crono_y, crono_w, crono_h, GxEPD_WHITE);
    display.setCursor(crono_x, aux_y);
    display.print(buffer);
  } while (display.nextPage());
}

void Endurance_Last (int minutos, int segundos) {
  display.setRotation(1);
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

  uint16_t crono_w = 70;
  uint16_t crono_h = 20;
  uint16_t crono_x = 210;
  uint16_t crono_y = 94;
  uint16_t aux_y = crono_y + crono_h;

  display.setPartialWindow(crono_x, crono_y, crono_w, crono_h);

  char buffer[6];
  sprintf(buffer, "%02d:%02d", minutos, segundos);

  display.firstPage();
  do {
    display.fillRect(crono_x, crono_y, crono_w, crono_h, GxEPD_WHITE);
    display.setCursor(crono_x, aux_y);
    display.print(buffer);
  } while (display.nextPage());
}
////// Endurance

////// Maniobrabilidad
void Maniobrabilidad_run_stop(bool start_run) {
  display.setRotation(1);
  display.setFont(&FreeSerifItalic24pt7b);
  display.setTextColor(GxEPD_BLACK);

  uint16_t start_w = 150;
  uint16_t start_h = 40;
  uint16_t start_x = 100;
  uint16_t start_y = 70;
  uint16_t aux_y = start_y + start_h;

  display.setPartialWindow(start_x, start_y, start_w, start_h);

  display.firstPage();
  do {
    display.fillRect(start_x, start_y, start_w, start_h, GxEPD_WHITE);
    display.setCursor(start_x, aux_y);
    display.print(start_run ? "RUN!" : "STOP");
  } while (display.nextPage());
}

void Maniobrabilidad_cronometro(int minutos, int segundos) {
  display.setRotation(1);
  display.setFont(&FreeMonoBold24pt7b);
  display.setTextColor(GxEPD_BLACK);

  uint16_t crono_w = 200;
  uint16_t crono_h = 40;
  uint16_t crono_x = 100;
  uint16_t crono_y = 20;
  uint16_t aux_y = crono_y + crono_h;

  display.setPartialWindow(crono_x, crono_y, crono_w, crono_h);

  char buffer[6];
  sprintf(buffer, "%02d:%02d", minutos, segundos);

  display.firstPage();
  do {
    display.fillRect(crono_x, crono_y, crono_w, crono_h, GxEPD_WHITE);
    display.setCursor(crono_x, aux_y);
    display.print(buffer);
  } while (display.nextPage());
}
////// Maniobrabilidad

////// Suspension

void Suspension_pitch(float pitch) {
  display.setPartialWindow(0, 0, 100, 100);  // Área de visualización de la burbuja
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.drawCircle(50, 50, 40, GxEPD_BLACK);  // Círculo centrado

    // Limita el pitch entre -50° y +50° para que no se salga del rango visual
    pitch = constrain(pitch, -50, 50);

    float angle = pitch * PI / 180.0;  // Convierte a radianes

    // Calcula extremos de la línea según el ángulo
    int x1 = 50 - 40 * cos(angle);
    int y1 = 50 + 40 * sin(angle);
    int x2 = 50 + 40 * cos(angle);
    int y2 = 50 - 40 * sin(angle);
    
    display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);

    // Opcional: muestra valor numérico del pitch
    /*display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(5, 95);
    display.print("Pitch:");
    display.setCursor(60, 95);
    display.print((int)pitch);
    display.print((char)0xB0);  // Símbolo de grados*/
  } while (display.nextPage());
}

void Suspension_roll(float roll) {
  display.setPartialWindow(196, 0, 100, 100);  // Área de visualización al extremo derecho
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.drawCircle(246, 50, 40, GxEPD_BLACK);  // Círculo centrado al extremo derecho

    // Limita el roll entre -50° y +50° para que no se salga del rango visual
    roll = constrain(roll, -50, 50);

    float angle = roll * PI / 180.0;  // Convierte a radianes

    // Calcula extremos de la línea según el ángulo (centrado en 246,50)
    int x1 = 246 - 40 * cos(angle);
    int y1 = 50 + 40 * sin(angle);
    int x2 = 246 + 40 * cos(angle);
    int y2 = 50 - 40 * sin(angle);
    
    display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);

    // Opcional: muestra valor numérico del roll
    /*display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(200, 95);
    display.print("Roll:");
    display.setCursor(255, 95);
    display.print((int)roll);
    display.print((char)0xB0);  // Símbolo de grados*/
  } while (display.nextPage());
}

void Suspension_cronometro(int minutos, int segundos) {
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  // Usamos fuente grande para números
  display.setFont(&FreeMonoBold24pt7b);

  // Medidas centradas entre burbujas
  uint16_t crono_x = 100;   // zona central
  uint16_t crono_w = 100;
  uint16_t crono_h = 40;

  // Primer bloque: minutos arriba
  uint16_t min_y = 10;
  uint16_t aux_min_y = min_y + crono_h;

  // Segundo bloque: segundos debajo
  uint16_t seg_y = 70;
  uint16_t aux_seg_y = seg_y + crono_h;

  // ------- Mostrar minutos -------
  display.setPartialWindow(crono_x, min_y, crono_w, crono_h);
  char min_buffer[3];
  sprintf(min_buffer, "%02d", minutos);

  display.firstPage();
  do {
    display.fillRect(crono_x, min_y, crono_w, crono_h, GxEPD_WHITE);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(min_buffer, 0, aux_min_y, &x1, &y1, &w, &h);
    display.setCursor(crono_x + (crono_w - w) / 2, aux_min_y);
    display.print(min_buffer);
  } while (display.nextPage());

  // ------- Mostrar segundos -------
  display.setPartialWindow(crono_x, seg_y, crono_w, crono_h);
  char seg_buffer[3];
  sprintf(seg_buffer, "%02d", segundos);

  display.firstPage();
  do {
    display.fillRect(crono_x, seg_y, crono_w, crono_h, GxEPD_WHITE);
    int16_t x2, y2;
    uint16_t w2, h2;
    display.getTextBounds(seg_buffer, 0, aux_seg_y, &x2, &y2, &w2, &h2);
    display.setCursor(crono_x + (crono_w - w2) / 2, aux_seg_y);
    display.print(seg_buffer);
  } while (display.nextPage());
}
////// Suspension

////// LIMPIEZA PROFUNDA
void deepCleanDisplay() {
  display.setFullWindow();
  for (int i = 0; i < 2; i++) { // alterna blanco y negro
    display.firstPage();
    do {
      display.fillScreen(i % 2 == 0 ? GxEPD_WHITE : GxEPD_BLACK);
    } while (display.nextPage());
    delay(500);
  }
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
  } while (display.nextPage());
}
////// LIMPIEZA PROFUNDA

//////////////// FUNCIONES PANTALLA //////////////


void setup() {

  ///////// ESPNOW ///////////
  //Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  ///////// ESPNOW ///////////

  // Inicializar Serial Monitor
  //Serial.begin(115200);

  /////////////////// DISPLAY ///////////////
  // Inicializar el display
  display.init(115200, true, 50, false);

  // Limpieza profunda al inicio
  deepCleanDisplay();

  // Hibernar el display
  display.hibernate();
  /////////////////// DISPLAY ///////////////

  ////// NEOPIXEL ///////
  tira.begin();
  tira.show();
  ////// NEOPIXEL ///////

  /////// BOTONES ///////
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);

  pinMode(BTN_3, INPUT_PULLUP);
  pinMode(BTN_4, INPUT_PULLUP);
  /////// BOTONES ///////
  

}

int select_mode = 1;
bool watch_mode_selected=0;

bool start_run = 0;
//bool last_start_run = 0;
int timer_seconds = 0;
int timer_minutes = 0;
long rpms = 1500;

bool in_Aceleracion_mode = false;
bool in_Endurance_mode = false;
bool in_Maniobrabilidad_mode = false;
bool in_Suspension_mode = false;
bool in_Traccion_mode = false;


unsigned long timer_inicio = 0;
unsigned long timer_actual = 0;


const unsigned long debounceDelay = 50;  // 50 ms de rebote

unsigned long lastDebounceTime_BTN_1 = 0;
unsigned long lastDebounceTime_BTN_2 = 0;
unsigned long lastDebounceTime_BTN_3 = 0;
unsigned long lastDebounceTime_BTN_4 = 0;

int ultima_vuelta_mostrada = 0;


void loop() {
  // put your main code here, to run repeatedly:
  ////Variables
  //Serial.print("RPM recibidas: ");
  //Serial.println(rpm);
  rpm_NEO = map(rpm, 1800, 3600, 1, 8);
  rpm_NEO = constrain(rpm_NEO, 1, 8);  // Asegura que no se salga de rango
  
  ////Variables
  
  ////// Leer Botones
  // Leer estados actuales
  bool current_BTN_1_State = digitalRead(BTN_1);
  bool current_BTN_2_State = digitalRead(BTN_2);
  bool current_BTN_3_State = digitalRead(BTN_3);
  bool current_BTN_4_State = digitalRead(BTN_4);

  unsigned long currentTime = millis();

  // --- BOTÓN 1 ---
  if (last_BTN_1_State == HIGH && current_BTN_1_State == LOW && (currentTime - lastDebounceTime_BTN_1) > debounceDelay) {
    lastDebounceTime_BTN_1 = currentTime;
    if (!watch_mode_selected) {
      select_mode++;
      if (select_mode > 5) select_mode = 1;
      select_mode_text(select_mode);
    }
  }

  // --- BOTÓN 2 ---
  if (last_BTN_2_State == HIGH && current_BTN_2_State == LOW && (currentTime - lastDebounceTime_BTN_2) > debounceDelay) {
    lastDebounceTime_BTN_2 = currentTime;
    if (!watch_mode_selected) {
      select_mode--;
      if (select_mode < 1) select_mode = 5;
      select_mode_text(select_mode);
    }
  }

  // --- BOTÓN 3 ---
  if (last_BTN_3_State == HIGH && current_BTN_3_State == LOW && (currentTime - lastDebounceTime_BTN_3) > debounceDelay) {
    lastDebounceTime_BTN_3 = currentTime;
    watch_mode_selected = true;
    if (select_mode == 1) in_Aceleracion_mode = true;
    else if (select_mode == 2) in_Endurance_mode = true;
    else if (select_mode == 3) in_Maniobrabilidad_mode = true;
    else if (select_mode == 4) in_Suspension_mode = true;
    else if (select_mode == 5) in_Traccion_mode = true;
  }

  // --- BOTÓN 4 ---
  if (last_BTN_4_State == HIGH && current_BTN_4_State == LOW && (currentTime - lastDebounceTime_BTN_4) > debounceDelay) {
    lastDebounceTime_BTN_4 = currentTime;
    deepCleanDisplay();
    watch_mode_selected = false;
    in_Aceleracion_mode = false;
    in_Endurance_mode = false;
    in_Maniobrabilidad_mode = false;
    in_Suspension_mode = false;
    in_Traccion_mode = false;
    aux_inicializacion_interfaz = false;
    aux_run_stop_actualizado = false;
    start_run = false;
    timer_seconds = 0;
    timer_minutes = 0;
    timer_inicio = 0;
    timer_actual = 0;
    select_mode_text(select_mode);
    NEO_PXL(1);
  }

  if (in_Aceleracion_mode == true) {
    if (!aux_inicializacion_interfaz) {
      Aceleracion_inicializacion();
      Aceleracion_run_stop_text(false);
      Aceleracion_nivel(0);
      aux_inicializacion_interfaz = true;
    }
    if (rpm > 2000 && !aux_run_stop_actualizado){
      start_run = true;
      Aceleracion_run_stop_text(true);
      aux_run_stop_actualizado = true;
    } else {
      start_run = false;
    }
    Aceleracion_Velocidad();
    float pitch = atan2(ax1, sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI;
    Aceleracion_nivel(pitch);
    NEO_PXL(rpm_NEO);
  }

  if (in_Endurance_mode == true) {
    if (!aux_inicializacion_interfaz) {
      Endurance_Inicializacion_1();
      Endurance_Cronometro(0,0);
      Endurance_Best(25,57);
      Endurance_Last(25,57);
      Endurance_Num_Vuelta(22);
      aux_inicializacion_interfaz = true;
    }
    if (last_BTN_1_State == HIGH && current_BTN_1_State == LOW) {
      SetCheckPoint();
      Endurance_Best(1,2);
      Endurance_Last(3,4);
      //timer_inicio = millis();
    }
    if (timer_inicio > 0){
      timer_actual = millis() - timer_inicio;
      timer_seconds = (timer_actual / 1000) % 60;
      timer_minutes = (timer_actual / 1000) / 60;
      Endurance_Cronometro(timer_minutes,timer_seconds);
    }
    unsigned long tiempo_actual_vuelta = millis() - tiempo_inicio_vuelta;
    vuelta_crono_min = tiempo_actual_vuelta / 1000 / 60;
    vuelta_crono_seg = (tiempo_actual_vuelta / 1000) % 60;
    Endurance_Cronometro(vuelta_crono_min, vuelta_crono_seg);
    Endurance_Velocidad();
    NEO_PXL(rpm_NEO);
    // Lógica de gestión de vueltas (ejecuta siempre)
    Manejo_vueltas(lat, lon);

    // Solo si se detecta un nuevo número de vuelta
    if (vuelta_actual != ultima_vuelta_mostrada && vuelta_actual > 1) {
      ultima_vuelta_mostrada = vuelta_actual;

      Endurance_Num_Vuelta(vuelta_actual);

      int minutos_best = mejor_tiempo / 1000 / 60;
      int segundos_best = (mejor_tiempo / 1000) % 60;
      int minutos_last = ultima_vuelta / 1000 / 60;
      int segundos_last = (ultima_vuelta / 1000) % 60;

      Endurance_Best(minutos_best, segundos_best);
      Endurance_Last(minutos_last, segundos_last);
    }
  }

  if (in_Maniobrabilidad_mode == true) {
    if (!aux_inicializacion_interfaz) {
      Maniobrabilidad_run_stop(false);
      aux_inicializacion_interfaz = true;
    }
    if (rpm > 2000 && !aux_run_stop_actualizado){
      start_run = true;
      Maniobrabilidad_run_stop(true);
      aux_run_stop_actualizado = true;
    } else {
      start_run = false;
    }
    if (last_BTN_1_State == HIGH && current_BTN_1_State == LOW) {
      timer_inicio = millis(); 
    }
    if (timer_inicio > 0){
      timer_actual = millis() - timer_inicio;
      timer_seconds = (timer_actual / 1000) % 60;
      timer_minutes = (timer_actual / 1000) / 60;
    }
    //Maniobrabilidad(start_run,timer_minutes,timer_seconds);
    //Maniobrabilidad_cronometro(timer_minutes,timer_seconds);
    Maniobrabilidad_cronometro(252,23);
    
    NEO_PXL(rpm_NEO);
  }

  if (in_Suspension_mode == true) {
    if (!aux_inicializacion_interfaz) {
      //limpiar Region de TExto
      display.setPartialWindow(100, 70, 150, 40);  // Área donde aparece el texto "Suspension"
      display.firstPage();
      do {
        display.fillRect(100, 70, 150, 40, GxEPD_WHITE);
      } while (display.nextPage());
      Suspension_cronometro(0,0);
      Suspension_pitch(0);
      Suspension_roll(0);
      aux_inicializacion_interfaz = true;
    }
    if (last_BTN_1_State == HIGH && current_BTN_1_State == LOW) {
      timer_inicio = millis(); 
    }
    if (timer_inicio > 0){
      timer_actual = millis() - timer_inicio;
      timer_seconds = (timer_actual / 1000) % 60;
      timer_minutes = (timer_actual / 1000) / 60;
    }
    float pitch = atan2(ax1, sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI;
    float roll = atan2(ay1, sqrt(ax1 * ax1 + az1 * az1)) * 180.0 / PI;
    Suspension_pitch(pitch);
    Suspension_roll(roll);
    Suspension_cronometro(timer_minutes,timer_seconds);
    NEO_PXL(rpm_NEO);
  }

  if (in_Traccion_mode == true) {
    if (!aux_inicializacion_interfaz) {
      Aceleracion_inicializacion();
      Aceleracion_run_stop_text(false);
      Aceleracion_nivel(0);
      aux_inicializacion_interfaz = true;
    }
    if (rpm > 2000 && !aux_run_stop_actualizado){
      start_run = true;
      Aceleracion_run_stop_text(true);
      aux_run_stop_actualizado = true;
    } else {
      start_run = false;
    }
    Aceleracion_Velocidad();
    float pitch = atan2(ax1, sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI;
    Aceleracion_nivel(pitch);
    NEO_PXL(rpm_NEO);
  }

  /*Serial.print("Lat actual: "); Serial.println(lat, 7);
  Serial.print("Lon actual: "); Serial.println(lon, 7);
  Serial.print("Lat checkpoint: "); Serial.println(checkpoint_lat, 7);
  Serial.print("Lon checkpoint: "); Serial.println(checkpoint_lon, 7);
  Serial.print("Distancia: "); Serial.println(CalculateDistance(lat, lon, checkpoint_lat, checkpoint_lon), 2);
  Serial.print("Checkpoint pasado: "); Serial.println(checkpoint_pasado);*/

  // Guardar estado actual para la siguiente iteración
  last_BTN_1_State = current_BTN_1_State;
  last_BTN_2_State = current_BTN_2_State;
  last_BTN_3_State = current_BTN_3_State;
  last_BTN_4_State = current_BTN_4_State;

  
}
