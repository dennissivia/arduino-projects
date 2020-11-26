/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "dennis_sivia"
#define IO_KEY "568ea858e8654252a14cef906f3de593"
#define WIFI_SSID "WILHELM.TEL-9QKQ228R6E"
#define WIFI_PASS "48142664097321249186"
#include "AdafruitIO_WiFi.h"
#define DEBUG 1

AdafruitIO_WiFi io1(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
