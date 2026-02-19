/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "EG3"
#define IO_KEY "aio_sZpK5935BKjmw90cONIS26131k52"

#define WIFI_SSID "VISIT-D15"
#define WIFI_PASS "vivelaexperiencia"

// comment out the following lines if you are using fona or ethernet
#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);