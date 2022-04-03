//==========================================================================
//
// Petoi Bittle CAM
// A simple web interface based on ESP32CAM module for controlling the Petoi Bittle robot
// Please find more details at Hackster.io:
// https://www.hackster.io/idreams/getting-started-with-petoi-bittle-7dc898
//
//==========================================================================

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems
#include "esp_http_server.h"

//Grove Port >> GND/5V/G13/G4
#define GROVE_TX 13
#define GROVE_RX 4

int servoPos = 55;

const char* ssid = "Bittle-CAM";
const char* password = "987654320";

#define PART_BOUNDARY "123456789000000000000987654321"
#define URI_STATIC_JPEG "/image.jpg"


//#define CAMERA_MODEL_AI_THINKER //
//#define CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM //
//#define CAMERA_MODEL_M5STACK_PSRAM_B
//#define CAMERA_MODEL_WROVER_KIT

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       32
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       17
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM_B)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     22
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       32
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

#else
#error "Camera model not selected"
#endif


static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <link rel="shortcut icon" href="data:image/x-icon;base64,AAABAAMAEBAAAAEAIABoBAAANgAAACAgAAABACAAKBEAAJ4EAAAwMAAAAQAgAGgmAADGFQAAKAAAABAAAAAgAAAAAQAgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////+zs7P+YmJj/W1tb/z4+Pv8+Pj7/XV1d/5ubm//v7+//////////////////////////////////+fn5/39/f/8fHx//dHR0/7a2tv9ZWVn/AElf/wCLs/8AVm//DRoe/4aGhv/7+/v/////////////////8fHx/z4+Pv9paWn/6Ojo//v7+///////ioqK/wBxkv8Axf//AMH6/wCx5f8ASmD/RkZG//T09P///////////15eXv9paWn/enp6/x0UCf9AMR7/2tHF/2FhYf8AUWn/AI22/w0aHv8XGxz/AGWD/wBHXf9oaGj///////////89PT3/sLCw/yocCv95UR3/dk8c/5Z2Tf+IiIj/AG+Q/wAiLP+MjIz/fHx8/yEjI/8Aha3/R0dH////////////PT09/6+vr/8lGQv/hlkg/1U5FP88PDz/iIiI/wBvkP8AJzP/i4uL/4eHh/8bHyD/AJO+/0dHR////////////z09Pf9ZWVn/kJCQ/xoYFP8qKij/zMzM/4iIiP8Ab5D/AJnG/wMfKP8IFxz/AHaZ/wCh0f9HZm////////////89PT3/HR0d/1JSUv/l5eX///////////+IiIj/AG+Q/wDF//8Axf//AMX//wDF//8AxP7/R9P9////////////PT09/4mJif9hYWH/GBgY/1xcXP+hoaH/YmJi/wBQaP8Am8n/AL/4/wDF//8Axf//HbX//5Ss/v///////////z09Pf+Kior/oKCg/0VFRf9LS0v/c3Nz/1ZWVv9TU1P/PDw8/wQSF/8AcpT/BcL//1+R/v+epv7///////////89PT3/Wlpa/yoqKv9UVFT/9vb2///////////////////////i4uL/SkpK/wA2Rv8AibL/R8Ln////////////bm5u/19fX/+4uLj///////////////////////////////////////7+/v+0tLT/XV1d/3V1df////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAKAAAACAAAABAAAAAAQAgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////6+vr/zc3N/6CgoP+Dg4P/dHR0/3V1df+EhIT/o6Oj/9HR0f/8/Pz///////////////////////////////////////////////////////////////////////////////////////////////////////39/f+3t7f/WFhY/w8PD/8AAAD/AAAA/wAAAP8AAAD/AAAA/wAAAP8AAAD/AAAA/xISEv9eXl7/v7+///7+/v////////////////////////////////////////////////////////////////////////////////+wsLD/Kysr/wAAAP8AAAD/FxcX/1lZWf+Dg4P/Pz8//wAAAP8AAAD/ADVF/wBifv8AP1L/AA0R/wAAAP8AAAD/NDQ0/7y8vP/////////////////////////////////////////////////////////////////p6en/Tk5O/wAAAP8BAQH/UFBQ/7y8vP/8/Pz////////////7+/v/LCws/wAtOv8AxP3/AMX//wDF//8Awfr/AImx/wA0Q/8AAAD/AAAA/11dXf/x8fH/////////////////////////////////////////////////z8/P/xwcHP8AAAD/NDQ0/8vLy/////////////////////////////////9SUlL/AE1j/wDF//8Axf//AMX//wDF//8Axf//AMX//wCSvf8AHyj/AAAA/ycnJ//a2tr//////////////////////////////////////8jIyP8PDw//AAAA/3Z2dv/7+/v/+vr6/93d3f/v7+//////////////////zc3N/woKCv8ADRH/AKbW/wDF//8Axf//AMX//wC37P8ArN7/AML7/wC+9v8AS2L/AAAA/xcXF//U1NT////////////////////////////h4eH/ExMT/wAAAP9/f3///Pz8/4CAgP8PDw//AAAA/wICAv9NTU3/4+Pj//////92dnb/AAAA/wABAf8AZoT/AMX//wCp2v8AM0P/AAEB/wAAAP8ADxP/AG2N/wDE/v8AT2f/AAAA/xwcHP/s7Oz//////////////////////4aGhv8AAAD/KSkp//v7+/9ubm7/AAAA/xMMBP9TNxP/WDsU/1g7FP+JaDz///r1//////8RERH/ABkh/wDF//8As+j/ABMZ/wAAAP8zMzP/UFBQ/w4ODv8AAAD/AGWC/wC99P8AEhj/AAAA/5qamv//////////////////////enp6/wAAAP+Hh4f/39/f/wMDA/8hFgj/3ZQ1/5poJP/VjjL//ak8//2pPP/+2Kb//////xISEv8AGiL/AMX//wBbdf8AAAD/eHh4/8rKyv+bm5v/2NjY/xYWFv8ABgj/ALju/wBWb/8AAAD/jo6O//////////////////////96enr/AAAA/7Kysv+np6f/AAAA/4VZH/9uSRr/AAAA/wEBAP8EAwH/BAMB/1pWUf//////EhIS/wAaIv8Axf//AC07/wEBAf/j4+P/CgoK/wAAAP9/f3//cHBw/wAAAP8Akbv/AHeb/wAAAP+Ojo7//////////////////////3p6ev8AAAD/ra2t/62trf8AAAD/elEd/4NYH/8AAAD/Fg4F/xsSBv8AAAD/VVVV//////8SEhL/ABoi/wDF//8AMkH/AQEB/97e3v8bGxv/AAAA/5SUlP9lZWX/AAAA/wCVwf8AfKH/AAAA/46Ojv//////////////////////enp6/wAAAP95eXj/6+vr/woKCv8RCwT/yYYw/8uIMP/hlzX/QywQ/wAAAP+dnZ3//////xISEv8AGiL/AMX//wBqif8AAAD/U1NT/+Dg4P/Ly8v/vb29/wkJCf8ADxP/AL72/wB8of8AAAD/jo6O//////////////////////96enr/AAAA/xkZGf/z8/P/lJSU/wAAAP8BAQD/IhcI/woHAv8AAAD/QUFB//v7+///////EhIS/wAaIv8Axf//AL31/wAnMv8AAAD/DQ0N/yAgIP8AAAD/AAIC/wCBp/8Axf//AHyh/wAAAP+Pk5T//////////////////////3p6ev8AAAD/AAAA/1hYWP/8/Pz/sLCw/zk5Of8ODg7/IyMj/319ff/29vb///////////8SEhL/ABoi/wDF//8Axf//ALrx/wBYc/8AGB//AAwP/wAxQP8AkLv/AMX//wDF//8AgKX/ACk1/4/d9P//////////////////////enp6/wAAAP8AAAD/AAAA/0lJSf/q6ur//////////////////////////////////////xISEv8AGiL/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDC+/8Av/f/j+X///////////////////////96enr/AAAA/0tLS/8oKCj/AAAA/xUVFf+cnJz/+/v7////////////////////////////EhIS/wAaIv8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//+P5f///////////////////////3p6ev8AAAD/c3Nz/56env9TU1P/BQUF/wAAAP8jIyP/jIyM/+Pj4/////////////////8SEhL/ABoi/wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Dw///Tpv+/7zN/v//////////////////////enp6/wAAAP9zc3P/oaGh/6Ghof+Li4v/Ozs7/wMDA/8AAAD/AQEB/ykpKf9cXFz/dHR0/wMDA/8ABwn/AFx4/wBkgv8AfqT/AK/i/wDF//8Axf//AMX//wDF//8Axf//ELz//2OP/v+AgP7/x8f+//////////////////////96enr/AAAA/3Nzc/+hoaH/oaGh/6Ghof+hoaH/LS0t/wAAAP8YGBj/BgYG/wAAAP8AAAD/AAAA/wAAAP8AAAD/AAAA/wAAAP8AAQL/ADdH/wCdy/8Axf//AMX//xS6//95g/7/gID+/4CA/v/Hx/7//////////////////////3p6ev8AAAD/c3Nz/6Ghof+hoaH/n5+f/0dHR/8AAAD/JiYm//Dw8P/39/f/0dHR/7Ozs/+lpaX/paWl/6ioqP+SkpH/Xl5e/xISEv8AAAD/AAID/wBmhP8AxP7/AcT//yqu/v9Zlf7/c4f+/7/L/v//////////////////////enp6/wAAAP9zc3P/oaGh/4KCgv8mJib/AAAA/yAgIP/d3d3/////////////////////////////////////////////////9/f3/5KSkv8QEBD/AAAA/wA1Rf8ApNT/AMX//wDF//8Axf//j+X///////////////////////96enr/AAAA/y4uLv8mJib/AAAA/wAAAP9HR0f/6Ojo/////////////////////////////////////////////////////////////////9/f3/87Ozv/AAAA/wABAf8AMkH/AGqJ/wCJsv+P1+z//////////////////////3p6ev8AAAD/AAAA/wAAAP83Nzf/ra2t//7+/v////////////////////////////////////////////////////////////////////////////39/f+ioqL/Ly8v/wAAAP8AAAD/AAAA/46Pj///////////////////////wcHB/3x8fP+goKD/3d3d////////////////////////////////////////////////////////////////////////////////////////////////////////////2NjY/52dnf96enr/zMzM//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////8AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAKAAAADAAAABgAAAAAQAgAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAP///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Pz8//Hx8f/m5ub/3d3d/9XV1f/IyMj/wMDA/8DAwP/Jycn/1dXV/97e3v/n5+f/8/Pz//39/f///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////+Xl5f+vr6//f39//1dXV/81NTX/Gxsb/wgICP8CAgL/AAAA/wAAAP8CAgL/CgoK/x0dHf84ODj/Wlpa/4ODg/+0tLT/6+vr//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7+/v/W1tb/ampq/yMjI/8QEBD/AgIC/wAAAP8AAAD/AAAA/wAAAP8AAAD/AAAA/wAAAP8AAAD/AAAA/wAAAP8AAAD/AAAA/wMDA/8SEhL/Jycn/3d3d//f39/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////4+Pj/2NjY/2lpaP8MDAz/AAAA/wAAAP8BAQH/DAwM/xsbG/8qKir/NDQ0/x0dHf8EBAT/AAAA/wAAAP8ABAb/ABgf/wAmMf8AHSb/ABIY/wAHCf8AAAD/AAAA/wAAAP8SEhL/eXl5/9/f3//6+vr//////////////////////////////////////////////////////////////////////////////////////////////////////+Pj4/98fHz/HBwc/wAAAP8AAAD/AQEB/y0tLf9ra2v/oaGh/8/Pz//y8vL//f39/9XV1f9RUVH/AwMD/wAEBf8ASF7/AKrc/wDD/P8Atuz/AJnH/wB1mP8ASmH/ABoh/wAAAP8AAAD/AAAA/yUlJf+JiYn/7Ozs////////////////////////////////////////////////////////////////////////////////////////////w8PD/y0tLf8GBgb/AAAA/wEBAf8sLCz/paWl/+Pj4//39/f///////////////////////////+8vLz/FRUV/wAUGv8Ans3/AMX//wDF//8Axf//AMX//wDF/v8AvfT/AK3f/wBwkf8AGB//AAAA/wAAAP8ICAj/Ojo6/9jY2P////////////////////////////////////////////////////////////////////////////j4+P+enp7/EBAQ/wAAAP8CAgL/Ghoa/4ODg//w8PD////////////////////////////////////////////Y2Nj/Hh4e/wAbI/8AtOn/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Asub/AFJr/wAQFf8AAQH/AAAA/xkZGf+xsbH/+vr6////////////////////////////////////////////////////////////+fn5/4WFhf8TExP/AAAA/wcHB/9XV1f/zMzM//////////////////////////////////////////////////7+/v+srKz/EBAQ/wAQFf8Akr3/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMT+/wCQu/8ANkb/AAME/wAAAP8bGxv/mJiY//39/f/////////////////////////////////////////////////6+vr/dHR0/wgICP8AAAD/AQEB/4ODg//v7+////////z8/P/c3Nz/w8PD/9nZ2f/7+/v//////////////////////+Dg4P83Nzf/AQEB/wABAv8ANET/ALPn/wDF//8Axf//AMX//wDF//8AwPj/AKTV/wCXxP8AruH/AMP9/wDE/v8As+j/AE5l/wAAAP8AAAD/DQ0N/4yMjP/+/v7///////////////////////////////////////v7+/+Tk5P/CgoK/wAAAP8JCQn/jo6O///////v7+//np6e/ygoKP8BAQH/AAAA/wEBAf8qKir/oaGh//Dw8P///////////4aGhv8JCQn/AAAA/wAAAP8ACAr/AHqe/wDF//8Axf//ALbr/wBvkf8AGSD/AAEB/wAAAP8AAgL/ACcz/wCGrv8AvPP/AMT+/wBSa/8ABAX/AAAA/xAQEP+np6f//f39/////////////////////////////////+Li4v8rKyv/AAAA/wICAv9cXFz/8fHx/+zs7P9iYmL/BwcH/wAAAP8AAAD/AAAA/wAAAP8AAAD/AwMD/19eXv/l5OP//////+fn5/9iYmL/AwMD/wAEBv8AVG7/ALbs/wDF//8Aptf/ADtN/wAAAf8AAAD/AAAA/wAAAP8AAAD/AAAA/wAICv8AWnX/ALrx/wC06f8ANUX/AAAA/wAAAP8/Pz//6enp/////////////////////////////////8/Pz/8EBAT/AAAA/xQUFP+6urr//////3x8fP8GBgb/AAAA/wsIAv9fPxb/kGAi/5NiIv+TYiL/k2Ii/51rKP/nwpH///z5//////+RkZH/BgYG/wAJC/8AfKH/AMX//wC78v8ATGL/AAID/wAAAP8YGBj/bW1t/42Njf9UVFT/CAgI/wAAAP8ABwn/AHaZ/wDE/v8AfaL/AAoN/wAAAP8UFBT/29vb/////////////////////////////////8zMzP8AAAD/AAAA/zExMf/19fX/4eHh/wgICP8AAAD/FQ4F/8GBLv/snjj/25M0/+mcOP/9qjz//qo9//6qPf/+tFP///Pj//////+RkZH/BgYG/wAJC/8AfKH/AMX//wCdzP8ACg3/AAAA/ycnJ//Jycn/5eXl/9vb2//t7e3/qamp/wwMDP8AAAD/ABIX/wC88/8Ar+L/ABsj/wAAAP8PDw//2dnZ/////////////////////////////////8zMzP8AAAD/AAAA/2VlZf//////j4+P/wAAAP8BAQD/mGYk/+GXNv9LMhH/FQ4F/0AqD/9+VB3/glce/4JXHv+EWB//08Sw//////+RkZH/BgYG/wAJC/8AfKH/AMX//wBmhP8AAAD/AAAA/6Wlpf/AwMD/Nzc3/xMTE/9WVlb/6enp/3l5ef8AAAD/AAAA/wCJsf8Aw/3/ADJB/wAAAP8PDw//2dnZ/////////////////////////////////8zMzP8AAAD/AAAA/4SEhP//////ZWVl/wAAAP8PCgP/15Az/51pJf8CAQD/AAAA/wAAAP8BAQD/AgEA/wIBAP8CAQD/gH99//////+RkZH/BgYG/wAJC/8AfKH/AMX//wBGW/8AAAD/BAQE/+vr6/9wcHD/AQEB/wAAAP8EBAT/uLi4/8jIyP8AAAD/AAAA/wBoh/8Axf//AEhd/wAAAP8PDw//2dnZ/////////////////////////////////8zMzP8AAAD/AAAA/39/f///////a2tr/wAAAP8KBwL/0Ysy/610Kf8GBAL/AAAA/wMCAf8QCgT/BAMB/wAAAP8AAAD/g4OD//////+RkZH/BgYG/wAJC/8AfKH/AMX//wBMYv8AAAD/AgIC/+Pj4/99fX3/AwMD/wAAAP8KCgr/xcXF/7y8vP8AAAD/AAAA/wBujv8Axf//AExj/wAAAP8PDw//2dnZ/////////////////////////////////8zMzP8AAAD/AAAA/1dXV//+/v7/pKSk/wAAAP8AAAD/c00b/+2fOP98Ux3/RS4Q/3BLGv+mbyf/JBgI/wAAAP8EBAT/tra2//////+RkZH/BgYG/wAJC/8AfKH/AMX//wB2mP8AAAD/AAAA/4GBgf/V1dX/Z2dn/0RERP+Ghob/7Ozs/1ZWVv8AAAD/AAAB/wCYxf8Axf//AExj/wAAAP8PDw//2dnZ/////////////////////////////////8zMzP8AAAD/AAAA/ycnJ//p6ej/8vLy/xsbG/8AAAD/CgYC/35UHf/rnTj/6544/+6gOf+FWR//DQgD/wAAAP8xMTH/4+Pj//////+RkZH/BgYG/wAJC/8AfKH/AMX//wCo2f8AGB//AAAA/xQUFP+YmJj/7u7u/+vr6//m5ub/ampq/wUFBf8AAAD/ACg1/wDB+v8Axf//AExj/wAAAP8PDw//2dnZ/////////////////////////////////8zMzP8AAAD/AAAA/w0NDf+jo6P//v7+/6qqqv8ODg7/AAAA/wEAAP8aEQb/PykO/xwTBv8AAAD/AAAA/w0NDf+Xl5f//f39//////+RkZH/BgYG/wAJC/8AfKH/AMX//wDB+v8AZoT/AAYI/wAAAP8BAQH/IyMj/z09Pf8VFRX/AAAA/wAAAP8AERb/AJXC/wDF//8Axf//AExj/wAAAP8PERH/2dra/////////////////////////////////8zMzP8AAAD/AAAA/wAAAP8/Pz//5OTk//b29v+SkpL/Ghoa/wAAAP8AAAD/AAAA/wAAAP8AAAD/GRkZ/4yMjP/z8/P///////////+RkZH/BgYG/wAJC/8AfKH/AMX//wDF//8Atuz/AF97/wANEf8AAAD/AAAA/wAAAP8AAAD/AAAA/wAbI/8AfaL/AMD5/wDF//8Axf//AExj/wABAf8QOkf/2efr/////////////////////////////////8zMzP8AAAD/AAAA/wAAAP8EBAT/UVFR//f39//5+fn/1dXV/3V1df8uLi7/FRUV/ywsLP91dXX/0tLS//r6+v////////////////+RkZH/BgYG/wAJC/8AfKH/AMX//wDF//8Axf//AL/3/wCcyv8AUWr/AB4n/wARFf8AKDT/AGOB/wCq3P8Awvv/AMX//wDF//8Axf//AFVv/wAZIP8Qocz/2vX+/////////////////////////////////8zMzP8AAAD/AAAA/wAAAP8AAAD/AAAA/0NDQ//W1tb//f39//////////////////////////////////////////////////////+RkZH/BgYG/wAJC/8AfKH/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AL31/wC06f8QyP//2vb//////////////////////////////////8zMzP8AAAD/AAAA/wkJCf8RERH/AQEB/wICAv8vLy//nJyc//T09P////////////////////////////////////////////////+RkZH/BgYG/wAJC/8AfKH/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8QyP//2vb//////////////////////////////////8zMzP8AAAD/AAAA/0hISP9/f3//IiIi/wAAAP8AAAD/DAwM/0JCQv/ExMT//f39//////////////////////////////////////+RkZH/BgYG/wAJC/8AfKH/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8QyP//2vb//////////////////////////////////8zMzP8AAAD/AAAA/1VVVf+hoaH/mpqa/0NDQ/8LCwv/AQEB/wAAAP8ICAj/WFhY/8PDw//o6Oj/+vr6//////////////////////+RkZH/BgYG/wAJC/8AfKH/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AsT//yCz//9hnP7/5vD//////////////////////////////////8zMzP8AAAD/AAAA/1VVVf+hoaH/oaGh/52dnf9wcHD/MTEx/wQEBP8AAAD/AAAA/wgICP87Ozv/cXFx/5+fn//ExMT/4uLi//T09P+FhYX/BAQE/wAHCf8AcpT/AL/3/wDC+/8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wDF//8Hwf//Nqf//3SG/v+Hh/7/7Oz//////////////////////////////////8zMzP8AAAD/AAAA/1VVVf+hoaH/oaGh/6Ghof+goKD/lpaW/3R0dP8mJib/AgIC/wAAAP8AAAD/AQEB/wsLC/8XFxf/ISEh/ykpKf8VFRX/AAAA/wABAf8AEhj/ACEr/wAkL/8ANUX/AFp1/wCRu/8Avvf/AMX//wDF//8Axf//AMX//wDF//8Axf//AMX//wrA//9Qmf7/e4P+/4CA/v+Hh/7/7Oz//////////////////////////////////8zMzP8AAAD/AAAA/1VVVf+hoaH/oaGh/6Ghof+hoaH/oaGh/6Ghof+Hh4f/EhIS/wAAAP8CAgL/CwsL/wMDA/8AAAD/AAAA/wAAAP8AAAD/AAAA/wAAAP8AAAD/AAAA/wAAAP8AAAD/AAAA/wABAf8AHSb/AHOV/wCx5f8Aw/z/AMX//wDF//8Axf//Fbn//26K/v+AgP7/gID+/4CA/v+Hh/7/7Oz//////////////////////////////////8zMzP8AAAD/AAAA/1VVVf+hoaH/oaGh/6Ghof+hoaH/oaGh/5mZmf9JSUn/BQUF/wAAAP83Nzf/nJyc/4eHh/9mZmb/S0tL/zc3N/8rKyr/JCQk/yUlJf8pKSn/Jycn/xgYGP8FBQX/AAAA/wAAAP8AAAD/AAAA/wApNf8AfqP/AL/3/wDF//8Axf//G7b//2GQ/v98gv7/gID+/4CA/v+Hh/7/7Oz//////////////////////////////////8zMzP8AAAD/AAAA/1VVVf+hoaH/oaGh/6Ghof+hoaH/nJyc/z4+Pv8GBgb/AAAA/xUVFf+4uLj//f39//7+/v/29vb/7e3t/+fn5//j4+L/4ODg/+Dg4P/i4uL/4eHh/9zc3P/Hx8f/iYmJ/zExMf8BAQH/AAAA/wAAAP8ACw//AFt2/wDC/P8Axf//AMX//wu///8is/7/SZ3+/2aO/v9yk/7/6O///////////////////////////////////8zMzP8AAAD/AAAA/1VVVf+hoaH/oaGh/5ubm/98fHz/IyMj/wAAAP8AAAD/EhIS/6Ojo/////////////////////////////////////////////////////////////////////////////r6+v+zs7P/ODg4/wkJCf8AAAD/AAAA/wA1Rf8Ans3/AL/3/wDF//8Axf//AMX//wDF//8QyP//2vb//////////////////////////////////8zMzP8AAAD/AAAA/0hISP+Ojo7/bW1t/z8/P/8PDw//AAAA/wEBAf8sLCz/ra2t////////////////////////////////////////////////////////////////////////////////////////////7+/v/46Ojv8iIiL/AAAA/wAAAf8AFh3/AFJr/wCJsf8Asuf/AMT+/wDF//8QyP//2vb//////////////////////////////////8zMzP8AAAD/AAAA/woKCv8VFRX/CgoK/wAAAP8AAAD/AAAA/0ZGRv/S0tL//Pz8//////////////////////////////////////////////////////////////////////////////////////////////////v7+//Gxsb/NjY2/wAAAP8AAAD/AAEB/wANEf8AHSX/ADpM/wBceP8Qg6b/2u71/////////////////////////////////8zMzP8AAAD/AAAA/wAAAP8AAAD/AAAA/w0NDf8vLy//nJyc//39/f//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////+fn5/4qKiv8oKCj/CgoK/wAAAP8AAAD/AAAA/wAAAP8PEBD/2dna/////////////////////////////////9DQ0P8GBgb/BQUF/xkZGf88PDz/ampq/6ampv/m5ub//v7+//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7+/v/f39//nZ2d/2RkZP84ODj/FRUV/wMDA/8WFhb/3Nzc//////////////////////////////////b29v/Jycn/z8/P/93d3f/o6Oj/+Pj4//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////b29v/n5+f/29vb/83Nzf/Nzc3/+Pj4/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////wAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA==" />
    <title>Petoi Bittle CAM</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 30px;}
      table { margin-left: auto; margin-right: auto; }
      td { padding: 8 px; }
      .button {
        background-color: #FFA400;
        border: none;
        color: white;
        padding: 10px 20px;
        text-align: center;
        text-decoration: none;
        display: inline-block;
        font-size: 18px;
        margin: 6px 3px;
        cursor: pointer;
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
        -webkit-tap-highlight-color: rgba(0,0,0,0);
      }
      img {  width: auto ;
        max-width: 100% ;
        height: auto ; 
      }
    </style>
  </head>
  <body>
    <h1>Petoi Bittle CAM</h1><br>
    <img src="" id="photo" width="240" height="320"><br><br><br>
    <table>
      <tr><td align="center"><button class="button" onmousedown="toggleCheckbox('headleft');" ontouchstart="toggleCheckbox('headleft');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');"><<</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('forward');" ontouchstart="toggleCheckbox('forward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Forward</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('headright');" ontouchstart="toggleCheckbox('headright');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">>></button></td></tr>
      <tr><td align="center"><button class="button" onmousedown="toggleCheckbox('left');" ontouchstart="toggleCheckbox('left');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Left</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('stop');" ontouchstart="toggleCheckbox('stop');">Stop</button></td><td align="center"><button class="button" onmousedown="toggleCheckbox('right');" ontouchstart="toggleCheckbox('right');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Right</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('backward');" ontouchstart="toggleCheckbox('backward');" onmouseup="toggleCheckbox('stop');" ontouchend="toggleCheckbox('stop');">Backward</button></td></tr>                   
    </table>
   <script>
   function toggleCheckbox(x) {
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=" + x, true);
     xhr.send();
   }
   window.onload = document.getElementById("photo").src = window.location.href.slice(0, -1) + ":81/stream";
   document.getElementById("photo").style.transform = 'rotate(' + 90 + 'deg)';
  </script>
  </body>
</html>
)rawliteral";
//------------------------------------------------------
static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}
//------------------------------------------------------
/*
   This method only stream one JPEG image
*/
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t fb_len = 0;
  int64_t fr_start = esp_timer_get_time();

  res = httpd_resp_set_type(req, "image/jpeg");
  if (res == ESP_OK)
  {
    res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=image.jpg");  //capture
  }
  if (res == ESP_OK) {
    ESP_LOGI(TAG, "Take a picture");
    //while(1){
    fr_start = esp_timer_get_time();
    fb = esp_camera_fb_get();
    if (!fb)
    {
      ESP_LOGE(TAG, "Camera capture failed");
      httpd_resp_send_500(req);
      return ESP_FAIL;
    } else {
      fb_len = fb->len;
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
      esp_camera_fb_return(fb);
      // Uncomment if you want to know the bit rate
      //int64_t fr_end = esp_timer_get_time();
      //ESP_LOGD(TAG, "JPG: %uKB %ums", (uint32_t)(fb_len / 1024), (uint32_t)((fr_end - fr_start) / 1000));
      return res;
    }
    //}
  }
}
//------------------------------------------------------
static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      //if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      //}
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}
//------------------------------------------------------
static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  String strServo;
  
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  sensor_t * s = esp_camera_sensor_get();
  int res = 0;
  
  if(!strcmp(variable, "forward")) {
    Serial2.print("kwkF");
    Serial.println("Forward");
  }
  else if(!strcmp(variable, "left")) {
    Serial2.print("kwkL");
    Serial.println("Left");
  }
  else if(!strcmp(variable, "right")) {
    Serial2.print("kwkR");
    Serial.println("Right");
  }
  else if(!strcmp(variable, "backward")) {
    Serial2.print("kbk");
    Serial.println("Backward");
  }
  else if(!strcmp(variable, "headleft")) {
		strServo = "c0 " + String(servoPos);
		Serial2.print(strServo);
    Serial.println(strServo);
  }
    else if(!strcmp(variable, "headright")) {
    strServo = "c0 -" + String(servoPos);
    Serial2.print(strServo);
    Serial.println(strServo);
  }
  else if(!strcmp(variable, "stop")) {
    Serial2.print("kbalance");
    Serial.println("Stop");
  }
  else {
    res = -1;
  }

  if(res){
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}
//------------------------------------------------------
void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t cmd_uri = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  httpd_uri_t static_image = {
    .uri       = URI_STATIC_JPEG,
    .method    = HTTP_GET,
    .handler   = capture_handler,
    .user_ctx  = NULL
  };
  
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &static_image);
  }
  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
//------------------------------------------------------
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  
  Serial.begin(9600);

  Serial2.begin(115200, SERIAL_8N1, GROVE_RX, GROVE_TX);
  Serial.setDebugOutput(false);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
   
  if(psramFound()){
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;//
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);


  Serial.print("Petoi-CAM Ready! Use 'http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("' to connect");
  
  // Start streaming web server
  startCameraServer();
}
//------------------------------------------------------
void loop() {
  
}
