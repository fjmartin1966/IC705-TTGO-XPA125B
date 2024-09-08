// Updates by KK7PZE in z_userprog:
//  Set RF Power in IC-705 based on band to prevent overdriving the XPA125B
//  Change power output display on TTGO to percentage with P= to denote RF Power
//  Change the RX / TX display to remove graphics and replace with RX/TX text
//  Change frequency display to show a second period to match readios display
//  Set RF Gain in IC-705 based on band to cut down noise floor on S-meter.
//  Set Pre-Amp in IC-705 based on band to cut down noise floor on S-meter.
//  TTGO Button Use:
//    For Home use the top button is default
//    For Away use the bottom button is default
//    Top button will follow the tables for RF Gain and Pre-Amp settings per band
//    Bottom button will set the RF Gain to 100% and the Pre-Amp to the highest setting if in Home use mode
//    Bottom button will set the RF Gain to 100% but leave the Pre-Amp settings alone if in Away mode
//    You can press the buttons during operation to change the mode in which they function
//  If using a Hotspot you can specify the frequencies and RF Power the radio will use
//  When setting the TTGO For Home use by setting Home = 1 in CIV_template.ino the configuration will try to achieve the highest amp output possible without overdriving the amps input or output interfaces
//  When setting the TTGO for Away use by setting Home = 0 in CIV_template.ino the configuration will try to achieve an 80 watt output of the amp to lower battery consumption and heat
//  NOTE: Make sure the setting for Home is correctly set in CIV_template.ino
// 7/25/2024 - V1.40

/*
CIV_template - z_userprog - adapted for the requirements of Glenn, VK3PE by DK8RW, May 16, 22
    vk3pe is using a "TTGO" ESP32 module.
    Band select BCD outputs set to be active Hi.NOTE: a BCD to Decimal chip will be used also
     to provide 10 band outputs.
    PTT output is active LOW

This is the part, where the user can put his own procedures in

The calls to this user programs shall be inserted wherever it suits - search for //!//
in all files

*/

#include <TFT_eSPI.h>       //using this LIB now.  https://github.com/Bodmer/TFT_eSPI    
// IMPORTANT!  
//      In the "User_Setup_Select.h" file, disable "// #include <User_Setup.h>           // Default setup is root library folder"
//      In the "User_Setup_Select.h" file, enable "#include <User_Setups/Setup25_TTGO_T_Display.h>"

// #define WIFI
#ifdef WIFI
// activate Wifi
45464748495051525354555657585960616263646566
#include <CIVclient.h>
#include <CIVcmds.h>
#include <CIVcmdsRotor.h>
#include <CIVmaster.h>
#include <ICradio.h>

#include <CIVclient.h>
#include <CIVcmds.h>
#include <CIVcmdsRotor.h>
#include <CIVmaster.h>


#include <WiFi.h>
#include <PubSubClient.h>

// Replace the next variables with your SSID/Password combination
// WE WILL NOT BE USING WIFI FOR THIS APPLICAITON SO LEAVE THESE BLANK
const char* ssid = "";
const char* password = "";
#define CONNECTION_TIMEOUT 10
#endif

//Added by KK7PZE for advanced math and string functions
#include <cmath>
#include <string>

//=========================================================================================
// user part of the defines

// if defined, the bit pattern of the output pins is inverted in order to compensate
// the effect of inverting HW drivers (active, i.e.uncommented by default)
#define invDriver         //if active, inverts band BCD out
//#define Inv_PTT           //if active, PTT out is Low going.




#define VERSION_USER "usrprg VK3PE V0_3 May 31st, 2022 with mods by DL1BZ, 2023 and mods by KK7PZE, 2024"

#define NUM_BANDS 13   /* Number of Bands (depending on the radio) */

//-----------------------------------------------------------------------------------------
//for TFT
TFT_eSPI tft = TFT_eSPI();

#define screen_width  240       //placement of text etc must fit withing these boundaries.
#define screen_heigth 135

// all my known colors for ST7789 TFT (but not all used in program)
#define B_DD6USB 0x0004    //   0,   0,   4  my preferred background color !!!   now vk3pe ?
#define BLACK 0x0000       //   0,   0,   0
#define NAVY 0x000F        //   0,   0, 123
#define DARKGREEN 0x03E0   //   0, 125,   0
#define DARKCYAN 0x03EF    //   0, 125, 123
#define MAROON 0x7800      // 123,   0,   0
#define PURPLE 0x780F      // 123,   0, 123
#define OLIVE 0x7BE0       // 123, 125,   0
#define LIGHTGREY 0xC618   // 198, 195, 198
#define DARKGREY 0x7BEF    // 123, 125, 123
#define BLUE 0x001F        //   0,   0, 255
#define GREEN 0x07E0       //   0, 255,   0
#define CYAN 0x07FF        //   0, 255, 255
#define RED 0xF800         // 255,   0,   0
#define MAGENTA 0xF81F     // 255,   0, 255
#define YELLOW 0xFFE0      // 255, 255,   0
#define WHITE 0xFFFF       // 255, 255, 255
#define ORANGE 0xFD20      // 255, 165,   0
#define GREENYELLOW 0xAFE5 // 173, 255,  41
#define PINK 0xFC18        // 255, 130, 198
//*************************************************************

//=================================================
// Mapping of port-pins to functions on ESP32 TTGO
//=================================================

// the Pins for SPI
#define TFT_CS    5
#define TFT_DC   16
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_RST  23
#define TFT_BL    4

#define PTTpin    17      //PTT out pin
#define PTTpinHF  26      //PTT out HF
#define PTTpinVHF 33      //PTT out VHF
#define PTTpinUHF 32      //PTT out UHF

boolean HF_ptt_Enable;
boolean VHF_ptt_Enable;
boolean UHF_ptt_Enable;

int bandvoltage;
#define LED 27       //Band voltage
#define C_RELAIS  25 //Coax relais HF / VHF-UHF

// For analogue PWM output
int brightness = 0;
const int freq = 5000;     //PWM Freq
const int ledChannel = 0;  //
const int resolution = 10; // possible resolution 8, 10, 12, 15 bit

//=========================================================================================
// user part of the database
// e.g. :
uint8_t         G_currentBand = NUM_BANDS;  // Band in use (default: not defined)

//=====================================================
// this is called, when the RX/TX state changes ...
//=====================================================
void  userPTT(uint8_t newState) {

#ifdef debug
    Serial.println(newState);                     //prints '1' for Tx, '0' for Rx
#endif
    tft.setFreeFont(&FreeSansBold9pt7b);        //previous setup text was smaller.
    //tft.setTextColor(WHITE) ;


    if (newState) {                                    // '1' = Tx mode
       Draw_TX();
    }   //Tx mode
    else {
       Draw_RX();
    } //Rx mode

#ifdef Inv_PTT 
    digitalWrite(PTTpin, !newState);    //--inverted-- output version:  Clr =Tx, Hi =Rx  
    if (HF_ptt_Enable) {
        digitalWrite(PTTpinHF, !newState);
    }
    if (VHF_ptt_Enable) {
        digitalWrite(PTTpinVHF, !newState);
    }
    if (UHF_ptt_Enable) {
        digitalWrite(PTTpinUHF, !newState);
    }
#else
    digitalWrite(PTTpin, newState);    // Clr =Rx, Hi =Tx 
    if (HF_ptt_Enable) {
        digitalWrite(PTTpinHF, newState);
    }
    if (VHF_ptt_Enable) {
        digitalWrite(PTTpinVHF, newState);
    }
    if (UHF_ptt_Enable) {
        digitalWrite(PTTpinUHF, newState);
    }
#endif 
}

//=========================================================================================
// creating bandinfo based on the frequency info

//-----------------------------------------------------------------------------------------
// tables for band selection and bittpattern calculation

//for IC-705 which has no 60M band:
//---------------------------------
// !!! pls adapt "NUM_BANDS" if changing the number of entries in the tables below !!!

//Added by KK7PZE - changed lower and upper limits of the bands by 100Hz to limit the swapping of relays in the PA
// lower limits[kHz] of the bands: NOTE, these limits may not accord with band  edges in your country.
constexpr unsigned long lowlimits[NUM_BANDS] = {
  1700, 3400, 5230.5,  6900,  10000, 13900, 17968, 20900, 24790, 27900, 49900, 143900 , 429900
};
// upper limits[kHz] of the bands:  //see NOTE above.
constexpr unsigned long uplimits[NUM_BANDS] = {
  2100, 4100, 5645,  7400, 10250, 14450, 18268, 21550, 25090, 29800, 54100 ,148100 , 450100
};


// "xxM" display for the TFT display. ie show what band the unit is current on in "meters"
const String (band2string[NUM_BANDS + 1]) = {
    // 160     80      60       40     30      20     17      15     12     10      6       NDEF
      "160m"," 80m", " 60m",  " 40m"," 30m", " 20m"," 17m", " 15m"," 12m"," 10m","  6m", "  2m","70cm" ," Out"

};

//Added by KK7PZE - RF Power Per Band Byte 1 when Home = 1
const unsigned int (BandPower1[NUM_BANDS + 1]) = {
//160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out // Use Table to set values.  Table is in file RF Power Settings Table.txt
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    2,      2,     0
};

//Added by KK7PZE - RF Power Per Band Byte 2 when Home = 1
const unsigned int (BandPower2[NUM_BANDS + 1]) = {
//160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out // Use Table to set values.  Table is in file RF Power Settings Table.txt
     0,   19,    3,   35,   33,   69,   38,   57,   69,  133,   40,   85,     85,     0
};

// The result of BandPower1 and BandPower2, Home = 1, as a percentage in my configuration is:
// 160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out
//   0%    5%    1%    9%    8%   17%   10%   15%   17%   33%   50%  100%    100%     0%

//Added by KK7PZE - RF Power Per Band Byte 1 when Home = 0
const unsigned int (BandPower3[NUM_BANDS + 1]) = {
//160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out // Use Table to set values.  Table is in file RF Power Settings Table.txt
     0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    1,    2,      2,     0
};

//Added by KK7PZE - RF Power Per Band Byte 2 when Home = 0
const unsigned int (BandPower4[NUM_BANDS + 1]) = {
//160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out // Use Table to set values.  Table is in file RF Power Settings Table.txt
     0,   17,    3,   19,   33,   22,   24,   24,   38,   49,   40,   85,     85,     0
};
// The result of BandPower3 and BandPower4, Home = 0, as a percentage in my configuration is:
// 160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out
//   0%    4%    1%    5%    8%    6%    7%    7%   10%   12%   50%  100%    100%     0%

//Added by KK7PZE - RF Gain Per Band
const unsigned int (RfGain1[NUM_BANDS + 1]) = {
//160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out // Use Table to set values.  Table is in file RF Power / RF Gain Settings Table.txt
  // 2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,      2,     2 // TEST CONFIGURATION
     2,    1,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,      2,     2 // REGULAR CONFIGURATION
};
const unsigned int (RfGain2[NUM_BANDS + 1]) = {
//160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out // Use Table to set values.  Table is in file RF Power / RF Gain Settings Table.txt
 // 85,   85,   85,    85,  85,   85,   85,   85,   85,   85,   85,   85,     85,    85 // TEST CONFIGURATION
    64,  132,   16,    1,   48,   85,   85,   85,   85,   85,   85,   85,     85,    85 // REGULAR CONFIGURATION
};
//Adjusting RF Gain so on a quiet frequency 3kHz wide the S-Meter reads 0.
// The result of RFgain as a percentage in my configuration is:
//  94%   72%  100%   78%  90%  100%  100%  100%  100%  100%  100%  100%    100%   100%
//Added by KK7PZE - Pre-Amp Settings per Band
const unsigned int (PreAmp[NUM_BANDS + 1]) = {
//160m,  80m,  60m,  40m,  30m,  20m,  17m,  15m,  12m,  10m,   6m,   2m,   70cm,   Out
  // 2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    1,      1,     0 // TEST CONFIGURATION
     0,    0,    0,    0,    0,    0,    0,    0,    1,    2,    2,    0,      0,     0 // REGULAR CONFIGURATION
};
// off   off   off   off   off   off   off   off   off   PA1   PA1   off     off    N/A

/*
Table to build RF Power and RF Gain per band
BandPower1,3/RfGain1  BandPower2,4/RfGain2          % of RF Power or RF Gain the radio outputs
0                     0                             0
0                     3                             1
0                     7                             2
0                     9                             3
0                     17                            4
0                     19                            5
0                     22                            6
0                     24                            7
0                     33                            8
0                     35                            9
0                     38                            10
0                     48                            11
0                     49                            12
0                     52                            13
0                     54                            14
0                     57                            15
0                     65                            16
0                     69                            17
0                     70                            18
0                     73                            19
0                     81                            20
0                     84                            21
0                     87                            22
0                     89                            23
0                     98                            24
0                     100                           25
0                     103                           26
0                     104                           27
0                     114                           28
0                     115                           29
0                     119                           30
0                     128                           31
0                     131                           32
0                     133                           33
0                     135                           34
0                     144                           35
0                     146                           36
0                     149                           37
0                     151                           38
1                     1                             39
1                     2                             40
1                     5                             41
1                     8                             42
1                     16                            43
1                     19                            44
1                     21                            45
1                     24                            46
1                     32                            47
1                     35                            48
1                     39                            49
1                     40                            50
1                     49                            51
1                     51                            52
1                     54                            53
1                     56                            54
1                     65                            55
1                     67                            56
1                     70                            57
1                     72                            58
1                     81                            59
1                     84                            60
1                     86                            61
1                     89                            62
1                     97                            63
1                     100                           64
1                     102                           65
1                     105                           66
1                     114                           67
1                     116                           68
1                     118                           69
1                     121                           70
1                     130                           71
1                     132                           72
1                     135                           73
1                     137                           74
1                     147                           75
1                     148                           76
1                     151                           77
2                     1                             78
2                     2                             79
2                     4                             80
2                     7                             81
2                     16                            82
2                     18                            83
2                     21                            84
2                     24                            85
2                     32                            86
2                     34                            87
2                     37                            88
2                     39                            89
2                     48                            90
2                     51                            91
2                     54                            92
2                     56                            93
2                     64                            94
2                     67                            95
2                     69                            96
2                     72                            97
2                     80                            98
2                     84                            99
2                     85                            100
*/

//Added by KK7PZE to define the buttons on the TTGO unit
int Button35; //Define top button
int Button0; //Define bottom button

//Added by KK7PZE to define the state of button0 (Bottom Button)
int BottomButtonStateRfGain; //Define
int BottomButtonStatePreAmp; //Define

//Added by KK7PZE to define RF Power to be used when connected to a hotspot
const unsigned int HotSpotRxFrequency = 144570000; //144.570.000
const unsigned int HotSpotTxFrequency = 144582500; //144.582.500
const unsigned int HotSpotRfPower1 = 1;  //Use the above chart to determine values to match your desired RF Power percentage
const unsigned int HotSpotRfPower2 = 40;  //Use the above chart to determine values to match your desired RF Power percentage
     // 50% RF Power
//------------------------------------------------------------
// set the bitpattern in the HW
void set_HW(uint8_t BCDsetting) {

   

#ifdef debug
    // Test output to control the proper functioning:
    Serial.print(" Pins ");
    #endif

}

//-----------------------------------------------------------------------------------------
// get the bandnumber matching to the frequency (in kHz)

byte get_Band(unsigned long frq) {
    byte i;
    for (i = 0; i < NUM_BANDS; i++) {
        //for (i=1; i<NUM_BANDS; i++) {   
        if ((frq >= lowlimits[i]) && (frq <= uplimits[i])) {
            return i;
        }
    }
    return NUM_BANDS; // no valid band found -> return not defined
}

//------------------------------------------------------------------
//    Show frequency in 'kHz' and band in 'Meters' text on TFT vk3pe
//------------------------------------------------------------------
void show_Meters(void)
{

    // Show Freq[KHz]
    tft.setCursor(5, 120);                //- 
    tft.fillRect(0, 80, 105, 55, BLACK);   //-erase   x,y,width, height 
    tft.drawRoundRect(0, 80, 105, 55, 5, WHITE);
    tft.setTextColor(YELLOW);               //-
    tft.setFreeFont(&FreeSansBold9pt7b);
    tft.setTextSize(2);
    tft.print(band2string[G_currentBand]); //-

}

void show_Mode(uint8_t newModMode, uint8_t newRXfilter)
{
       tft.setFreeFont(NULL);         // Set font to GLCD
       // tft.setFreeFont(&FreeSans9pt7b);
       // tft.setFreeFont(&Tiny3x3a2pt7b);
       tft.fillRect(105, 80, 85, 55, BLACK);   //erase previous freq   vk3pe x,y,width,height,colour 10,40,137,40
       tft.drawRoundRect(105, 80, 85, 55, 5, WHITE);
       tft.setTextSize(2);
       tft.setCursor(115, 90);
       tft.setTextColor(YELLOW);
       tft.print(modModeStr[newModMode]);
       tft.setCursor(115, 115);
       tft.print(FilStr[newRXfilter]);

}

void user_TXPWR(unsigned short getTXPWR) {
       unsigned long TXPWR_P;
       TXPWR_P=((getTXPWR*100)/255); // calculate in percent like IC705
       #ifdef debug
        Serial.print("getTXPWR: ");     Serial.println(getTXPWR);
        Serial.print("TXPWR_P: ");     Serial.println(TXPWR_P);
      #endif
       tft.setFreeFont(NULL);         // Set default font
       tft.setTextSize(2);
       tft.fillRect(190, 1, 45, 28, MAROON); // productive setting background
       // tft.fillRect(170, 1, 65, 28, BLACK); //debug setting background
       tft.setTextColor(WHITE);  // print TXPWR white if output power < 40% 
       if (getTXPWR > 101) {
         tft.setTextColor(YELLOW); // print TXPWR yellow if out > 40% for attention of PA max. input power
       }
       if (getTXPWR > 128) {
         tft.setTextColor(ORANGE); // print TXPWR orange if out > 50% for attention of PA max. input power
       }
       tft.setCursor(190,7);
       if (getTXPWR == 0) {
         tft.setTextColor(RED);
         tft.print("0");
       }
       else {
         tft.print(1*TXPWR_P, 1); //output TXPWR as Percent with one decimal digit
       }

       tft.println("%"); // add % for display TXPWR value
}

#ifdef WIFI
void setup_wifi() {
     delay(1000);

    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");
    int timeout_counter = 0;

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(200);
        timeout_counter++;
        if(timeout_counter >= CONNECTION_TIMEOUT*5){
        // ESP.restart();
        Serial.print("no WLAN connect...exit WiFi");
        return;
        }
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}
#endif

void init_DAC() {
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(LED, ledChannel);
}

void Draw_TX() {
        tft.fillRoundRect(190, 80, 50, 55, 10, RED);
        tft.setCursor(192,117);
        tft.setFreeFont(&FreeSansBold9pt7b);
        tft.setTextColor(WHITE);
        tft.setTextSize(2);
        tft.print("Tx");
}

void Draw_RX() {
        tft.fillRoundRect(190, 80, 50, 55, 10, GREEN);
        tft.setCursor(192,117);
        tft.setFreeFont(&FreeSansBold9pt7b);
        tft.setTextColor(MAROON);
        tft.setTextSize(2);
        tft.print("Rx");
}

void Clear_Scr() {
        tft.fillRect(0, 31, 240, 104, BLACK);
}

void BT_Conn_Status(const char* read_Conn_Status) {
    const char* Conn_Yes = strstr(read_Conn_Status, "R_ON");

    tft.setFreeFont(NULL);         // Set default font
    tft.setTextSize(2);
   
    if (read_Conn_Status == Conn_Yes) {
      tft.fillRect(80, 1, 155, 28, MAROON); // productive setting background
      tft.setTextColor(GREEN);
      tft.setCursor(82,7);
      tft.print("RF Power=");
      Clear_Scr();
      Draw_RX();
      //Added by KK7PZE to show 0% if RF Power is at 0 at startup
      tft.setCursor(190,7);
      tft.setTextColor(RED);
      tft.setFreeFont(NULL);         // Set default font
      tft.setTextSize(2);
      tft.print("0");
      tft.println("%");
      set_Buttons(); //Added by KK7PZE to set the initial button states on the TTGO device
    }
    else {
      tft.fillRect(80, 1, 155, 28, MAROON); // productive setting background
      tft.setTextColor(WHITE);
      tft.setCursor(153,7);
      tft.print("OFFLINE");
      Clear_Scr(); // clear screen
      tft.setCursor(0,40);
      tft.print(" Please Power ON or\n     Pair your:\n\n    ICOM IC-705\n\n  via Bluetooth!!!");

    }
}

//Added by KK7PZE - Set initial button values and use of device location
void set_Buttons(){
    Serial.print("Home: "); Serial.println(Home);
    if (Home == 1){
      // For Home use:
        Button35 = 1;  //Top Button
        Button0 = 0;  //Bottom Button
        Serial.println("Buttons set for home use");
    }
    else {
      // For Away use:
        Button35 = 0;  //Top Button
        Button0 = 1;  //Bottom Button
        Serial.println("Buttons set for away use");
    }
}

//Added by KK7PZE - Function to control RF Gain based on TTGO Buttons
//  Button description is based on orientation.  With the TTGO unit facing you and the USB-C plug on the right
//  there are 3 buttons on the TTGO.  The button facing up is the reset button.  Only use it to reboot the unit.
//  The button just above the USB plug is button 35.  This button enables the automatic RF Gain control.  This is enabled by default.
//  The button just below the USB plug is button 0.  This button disables the automatic RF Gain control and sets the RF gain to 100%.
//  If you press button 0 you can now manually adjust the RF Gain and it will hold your settings as you change frequency and bands.
//  If you then press button 35 the RF gain will automatically change back to the array selected value.  If you change the RF Gain
//  setting manually it will stay at that setting until you either change frequency or band and then it will return to the array
//  selected value for the band.
void userRfGain(unsigned long ButtonZero, unsigned long ButtonThirtyFive) {
    if(ButtonThirtyFive == 1){
    int RfGainLevel1 = RfGain1[G_currentBand];
    int RfGainLevel2 = RfGain2[G_currentBand];
    uint8_t SetRfGainData[3];
    SetRfGainData[0] = 2; // Length of data
    SetRfGainData[1] = RfGainLevel1; // Field 1 of data
    SetRfGainData[2] = RfGainLevel2; // Field 2 of data
    civ.writeMsg (civAddr,CIV_C_RF_GAIN,SetRfGainData,CIV_wFast); //CIV_wChk or CIV_wFast
    CIVwaitForAnswer  = true;
    int PreAmpLevel = PreAmp[G_currentBand];
    uint8_t SetPreAmpData[2];
    SetPreAmpData[0] = 1; //Length of data
    SetPreAmpData[1] = PreAmpLevel; // Field 1 of data
     civ.writeMsg (civAddr,CIV_C_PRE_AMP,SetPreAmpData,CIV_wFast); //CIV_wChk or CIV_wFast
    CIVwaitForAnswer  = true;
    BottomButtonStateRfGain = 0;
  }else if(ButtonThirtyFive == 0){
    if (Home == 1){
      if(BottomButtonStateRfGain == 0){
        int RfGainLevel1 = 2;
        int RfGainLevel2 = 85;
        uint8_t SetRfGainData[3];
        SetRfGainData[0] = 2; // Length of data
        SetRfGainData[1] = RfGainLevel1; // Field 1 of data
        SetRfGainData[2] = RfGainLevel2; // Field 2 of data
        civ.writeMsg (civAddr,CIV_C_RF_GAIN,SetRfGainData,CIV_wFast); //CIV_wChk or CIV_wFast
        CIVwaitForAnswer  = true;
        BottomButtonStateRfGain = 1;
      }
    }else if (Home == 0){
      //Don't Do Anything
    }
  }
}

//Added by KK7PZE - Control of Pre-Amp settings per band
//  To help with adjusting the noise floor beyond RF Gain, I also adjust the Pre-Amp settings per band.
void userPreAmp(unsigned long ButtonZero, unsigned long ButtonThirtyFive) {
    Serial.println("Doing userPreAmp");
    if(ButtonThirtyFive == 1){
      Serial.println("Doing userPreAmp with Button 35 set to 1");
      int PreAmpLevel = PreAmp[G_currentBand];
      uint8_t SetPreAmpData[2];
      SetPreAmpData[0] = 1; //Length of data
      SetPreAmpData[1] = PreAmpLevel; // Field 1 of data
      civ.writeMsg (civAddr,CIV_C_PRE_AMP,SetPreAmpData,CIV_wFast); //CIV_wChk or CIV_wFast
      CIVwaitForAnswer  = true;
      BottomButtonStatePreAmp = 0;
    }else if(ButtonThirtyFive == 0){
      Serial.println("Doing userPreAmp with Button 35 set to 0");
      int PreAmpLevel = 0;
      Serial.print("G_currentBand: "); Serial.println(G_currentBand);
      if (Home == 1){
        if (BottomButtonStatePreAmp == 0){
          if (G_currentBand < 11){
            PreAmpLevel = 0; //This is a fixed option for when you press the bottom button on HF bands.  Do you want no pre-amp=0, level 1=1, level 2=2
            Serial.println("PreAmpLevel 2");
          }else {
            PreAmpLevel = 0; //This is a fixed option for when you press the bottom button on VHF or UHF.  Do you want no pre-amp=0, pre-amp=1
            Serial.println("PreAmpLevel 1");
          }
          uint8_t SetPreAmpData[2];
          SetPreAmpData[0] = 1; //Length of data
          SetPreAmpData[1] = PreAmpLevel; // Field 1 of data
          civ.writeMsg (civAddr,CIV_C_PRE_AMP,SetPreAmpData,CIV_wFast); //CIV_wChk or CIV_wFast
          CIVwaitForAnswer  = true;
          BottomButtonStatePreAmp = 1;
        }
      }else if (Home == 0){
        //Don't Do Anything
      }
    }
}
//------------------------------------------------------------
// process the frequency received from the radio
//------------------------------------------------------------

void set_PAbands(unsigned long frequency) {
    unsigned long freq_kHz;
    
    freq_kHz = G_frequency / 1000;            // frequency is now in kHz
    G_currentBand = get_Band(freq_kHz);     // get band according the current frequency

    // tft.setFreeFont(&FreeSansBold9pt7b);   //bigger numbers etc from now on. <<<<<<<<-------------------
    // tft.setFreeFont(&FreeMonoBold18pt7b);
    tft.setFreeFont(&Orbitron_Light_32);
    tft.setTextSize(1);
    tft.setCursor(5, 67);
    
    if (freq_kHz < 100000) {
      tft.setCursor(25, 67);                 // for bigger print size
    } 
    if (freq_kHz < 10000) {
       tft.setCursor(40, 67);
    }
    
    //already use white from previous :-
    tft.fillRoundRect(0, 35, tft.width(), 40, 5, BLUE);   //erase previous freq   vk3pe x,y,width,height,colour 10,40,137,40
    // tft.drawRoundRect(0, 35, tft.width(), 40, 5, BLACK);    //with white border.
    tft.setTextColor(WHITE);               // at power up not set!
    // mod by DL1BZ and KK7PZE
    int FreqKHz = frequency/1000;
    int FreqMHz = int(FreqKHz)*1000;
    int FreqHz = frequency - FreqMHz;
    tft.print(FreqKHz*.001, 3);
    tft.print(".");
    if (FreqHz < 100) {
      tft.print("0");
      tft.print(FreqHz/10,1);
    }
    else {
      tft.print(FreqHz/10,1);
    }
    // tft.println("M"); // add % for display TXPWR value

#ifdef debug
    // Test-output to serial monitor:
    Serial.print("Frequency: ");  Serial.println(freq_kHz);
    Serial.print("Band: ");     Serial.println(G_currentBand);
    Serial.println(band2string[G_currentBand]);
 #endif

    // MR Hier stellen we het voltage in
        int bandcode;
    bandcode = G_currentBand;
        int sendDAC;  // calculated value we will send to DAC at the end
        int corrFact; // because if a load exist like PA we will have a lower, non-linear voltage output as calculated
        corrFact = 0; // add a little bit more mV, here with R=470Ohm and C=22uF at PIN 27

    switch (bandcode) {
    case 0:  // 160M
        // Manual XPA125B 230mV
        bandvoltage=230; // in mV
        // 3
        corrFact = 6;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 1:  // 80M
        // Manual XPA125B 460mV
        bandvoltage = 460;
        // 4
        corrFact = 14;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 2:  // 60M
        // Manual XPA125B 690mV
        bandvoltage = 690;
        // 5
        corrFact = 19;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 3:  // 40M
        // Manual XPA125B 920mV
        bandvoltage = 920;
        // 8
        corrFact = 26;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 4:  // 30M
        // Manual XPA125B 1150mV
        bandvoltage = 1150;
        // 8
        corrFact = 30;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 5:  // 20M
        // Manual XPA125B 1380mV
        bandvoltage = 1380;
        // 9
        corrFact = 35;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 6:  // 17M
        // Manual XPA125B 1610mV
        bandvoltage = 1610;
        // 9
        corrFact = 44;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 7:  // 15M
        // Manual XPA125B 1840mV
        bandvoltage = 1840;
        // 10
        corrFact = 58;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 8:  // 12M
        // Manual XPA125B 2070mV
        bandvoltage = 2070;
        // 12
        corrFact = 55;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 9:  // 10M
        // Manual XPA125B 2300mV
        bandvoltage = 2300;
        // 15
        corrFact = 63;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 10:  // 6M
        // Manual XPA125B 2530mV
        bandvoltage = 2530;
        // 15
        corrFact = 71;
        bandvoltage = bandvoltage + corrFact; // add corrFact for new bandvoltage for calculate
        break;
    case 11:  // 2M
        bandvoltage = 0;
        break;
    case 12:  // 70CM
        bandvoltage = 0;
        break;
    case 13:  // NDEF
        bandvoltage = 0;
        break;

    }

    
    if (freq_kHz > 0 && freq_kHz < 60000) {
        digitalWrite(C_RELAIS, LOW);
        HF_ptt_Enable = 1;  //Default for the HF/6M
        VHF_ptt_Enable = 0;
        UHF_ptt_Enable = 0;

        
    }

    else if
        (freq_kHz > 144000 && freq_kHz < 148000) {
        digitalWrite(C_RELAIS, HIGH);
        HF_ptt_Enable = 0;
        VHF_ptt_Enable = 1; //Default for VHF
        UHF_ptt_Enable = 0;

    }
    else if
        (freq_kHz > 430000 && freq_kHz < 470000) {
        digitalWrite(C_RELAIS, HIGH);
        HF_ptt_Enable = 0;
        VHF_ptt_Enable = 0;
        UHF_ptt_Enable = 1; //Default for UHF

    }

    #ifdef debug
      Serial.print("Bandvoltage in mV for XIEGU-PA with LOAD: "); Serial.println(bandvoltage);
      Serial.print("Correct factor in mV                    : "); Serial.println(corrFact);
    #endif
    sendDAC = bandvoltage * 1024 / 3300; // a value at 1024 is 3V3 output without load
    
    // check if the value not greater as 1024 because we use a resolution of 10bit = 2^10 = 1024 = 3V3
    if (sendDAC > 1024) {
      sendDAC = 1024;
    }
    
    #ifdef debug
      Serial.print("Send value to DAC: ");     Serial.println(sendDAC);
    #endif

    // set analog voltage for bandswitching on PA for XIEGU protocol, e.g. XPA125B or Micro PA50 at PIN 27
    ledcWrite(ledChannel, sendDAC); // send to ESP32-DAC

    show_Meters();            //Show frequency in kHz and band in Meters (80m etc) on TFT
}

//=========================================================================================
// this is called, whenever there is new frequency information ...
// this is available in the global variable "G_frequency" ...
void userFrequency(unsigned long newFrequency) {

    set_PAbands(G_frequency);

  //Added by KK7PZE to set the radio RF Power output based on the band selected to not overdrive the Xiegu XPA125B amplifier
  //  In the RfBandPower1 and RfBandPower2 arrays you can configure per band what the RF Power output percentage will be when Home = 1, ~100W.
  //  In the RfBandPower3 and RfBandPower4 arrays you can configure per band what the RF Power output percentage will be when Home = 0, ~ 80W.
  //  This is extremely important to keep from burning out components on your XPA125B amplifier.
  //  NEVER use a value above 50%.  50% is 5 watts and that is the maximum input allowed by the XPA125B.
  //  The maximum output of the XPA125B is 100W.
  //  Different bands have different efficiencies in the XPA125B.  As such setting 50% RF Power output is fine for 10M as working
  //  any mode (SSB, RTTY, etc) does not drive the XPA125B above 100W.  But a band like 20M can drive the output of the XPA125B
  //  100W with the IC-705 RF Power output set above 12%.
  //  Thus the array sets the RF Power output of the IC-705 per band.  You can manually adjust this power but it will revert to
  //  the array based value if you change frequencies or change bands to protect the XPA125B from damage.
  //  If you operate in split mode the RF Output power will always change to the array based value as that is seen as a change in
  //  frequency when you key up the mic.
  //  If using a hotspot the frequencies and RF Power are set to reduce the transmission outside of the hotspot area
  if (G_frequency == HotSpotRxFrequency || G_frequency == HotSpotTxFrequency){ //Hotspot RX TX frequencies
    int RfBandPower1 = HotSpotRfPower1;
    int RfBandPower2 = HotSpotRfPower2;
    uint8_t SetPowerData[3];
    SetPowerData[0] = 2; // Length of data
    SetPowerData[1] = RfBandPower1; // Field 1 of data
    SetPowerData[2] = RfBandPower2; // Field 2 of data
    civ.writeMsg (civAddr,CIV_C_RF_POW,SetPowerData,CIV_wFast); //CIV_wChk or CIV_wFast
    CIVwaitForAnswer  = true;
    #ifdef debug
    Serial.println("HotSpot in use");
    Serial.print("Set value RFPower1: ");     Serial.println(RfBandPower1);
    Serial.print("Set value RFPower2: ");     Serial.println(RfBandPower2);
    Serial.print("Power Data0:");     Serial.println(SetPowerData[0]);
    Serial.print("Power Data1:");     Serial.println(SetPowerData[1]);
    Serial.print("Power Data2:");     Serial.println(SetPowerData[2]);
    Serial.print("Band Number:");     Serial.println(G_currentBand);
    #endif
  }
  else if (Home == 1) {
    int RfBandPower1 = BandPower1[G_currentBand];
    int RfBandPower2 = BandPower2[G_currentBand];
    uint8_t SetPowerData[3];
    SetPowerData[0] = 2; // Length of data
    SetPowerData[1] = RfBandPower1; // Field 1 of data
    SetPowerData[2] = RfBandPower2; // Field 2 of data
    civ.writeMsg (civAddr,CIV_C_RF_POW,SetPowerData,CIV_wFast); //CIV_wChk or CIV_wFast
    CIVwaitForAnswer  = true;
    #ifdef debug
    Serial.print("Set value RFPower1: ");     Serial.println(RfBandPower1);
    Serial.print("Set value RFPower2: ");     Serial.println(RfBandPower2);
    Serial.print("Power Data0:");     Serial.println(SetPowerData[0]);
    Serial.print("Power Data1:");     Serial.println(SetPowerData[1]);
    Serial.print("Power Data2:");     Serial.println(SetPowerData[2]);
    Serial.print("Band Number:");     Serial.println(G_currentBand);
    #endif
  }
else if (Home == 0) {
    int RfBandPower1 = BandPower3[G_currentBand];
    int RfBandPower2 = BandPower4[G_currentBand];
    uint8_t SetPowerData[3];
    SetPowerData[0] = 2; // Length of data
    SetPowerData[1] = RfBandPower1; // Field 1 of data
    SetPowerData[2] = RfBandPower2; // Field 2 of data
    civ.writeMsg (civAddr,CIV_C_RF_POW,SetPowerData,CIV_wFast); //CIV_wChk or CIV_wFast
    CIVwaitForAnswer  = true;
    #ifdef debug
    Serial.print("Set value RFPower1: ");     Serial.println(RfBandPower1);
    Serial.print("Set value RFPower2: ");     Serial.println(RfBandPower2);
    Serial.print("Power Data0:");     Serial.println(SetPowerData[0]);
    Serial.print("Power Data1:");     Serial.println(SetPowerData[1]);
    Serial.print("Power Data2:");     Serial.println(SetPowerData[2]);
    Serial.print("Band Number:");     Serial.println(G_currentBand);
    #endif
  }

  //Added by KK7PZE to set the radio RF Gain based on the band selected to reduce the S-Level to close to 0 on an idle frequency
  //Added by KK7PZE to set the radio Pre-Amp based on the band selected to reduce the S-Level to close to 0 on an idle frequency
  if ((Button0 == 0) || (Button0 == 1)){
    #ifdef debug
      Serial.print("Button0:");     Serial.println(Button0);
      Serial.print("Button35:");     Serial.println(Button35);
    #endif
    userRfGain(Button0, Button35);
    userPreAmp(Button0, Button35);
  }
}

//-----------------------------------------------------------------------------------------
// initialise the TFT display
//-----------------------------------------------------------------------------------------

void init_TFT(void)
{
    //tft.init(screen_heigth, screen_width) ;  //not used

    tft.init();
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);              // switch backlight on

    tft.fillScreen(BLACK);
    tft.setRotation(1);
    tft.fillRoundRect(0, 0, tft.width(), 30, 5, MAROON);   // background for screen title
    tft.drawRoundRect(0, 0, tft.width(), 30, 5, WHITE);    //with white border.

    tft.setTextSize(2);                  //for default Font only.Font is later changed.
    tft.setTextColor(YELLOW);
    tft.setCursor(2, 7);                //top line
    tft.print("IC-705");

    tft.setTextColor(WHITE);            //white from now on

    tft.setCursor(185, 45);               //
    // tft.setTextSize(3);
    // tft.print("MHz");
    tft.setTextSize(1);
    // tft.setCursor(150, 95) ; 
    // tft.setCursor(135, 107);             //
    // tft.print("band");                   //"160m" etc   or Out if invalid Freq. for Ham bands.
}

//=========================================================================================
// this will be called in the setup after startup
void  userSetup() {

    Serial.println(VERSION_USER);

    // set the used HW pins (see defines.h!) as output and set it to 0V (at the Input of the PA!!) initially

    pinMode(PTTpin, OUTPUT);     //PTT out pin
    pinMode(PTTpinHF, OUTPUT);   //PTTHF out pin
    pinMode(PTTpinVHF, OUTPUT);  //PTTVHF out pin
    pinMode(PTTpinUHF, OUTPUT);  //PTTUHF out pin

    pinMode(C_RELAIS, OUTPUT);   // Coax Relais HF / VHF-UHF
    digitalWrite(PTTpin, LOW);       //set 'Rx mode' > high
    digitalWrite(PTTpinHF, LOW);
    digitalWrite(PTTpinVHF, LOW);
    digitalWrite(PTTpinUHF, LOW);
 


    init_TFT();  // initialize T-DISPLAY LILYGO TTGO v1.1
    
    init_DAC(); // initialize analog output

    userPTT(0);  // initialize the "RX" symbol in the screen

    #ifdef WIFI
    setup_wifi();
    #endif

    //Added by KK7PZE - Initialize Buttons on TTGO
    pinMode(35, INPUT); // Top Button
    pinMode(0, INPUT);  // Bottom Button

    //Added by KK7PZE to monitor the state of the bottom button and frequency/band changes
    BottomButtonStateRfGain = 0; //Initialize to 0
    BottomButtonStatePreAmp = 0; //Initialize to 0
}
//-------------------------------------------------------------------------------------
// this will be called in the baseloop every BASELOOP_TICK[ms]
void  userBaseLoop() {
  //Added by KK7PZE to set the radio RF Gain and Pre-Amp per band based on S-level noise at the QTH
  if(digitalRead(0) == 0){
    Button0 = 1;
    Button35 = 0;
    #ifdef debug
      Serial.println("Bottom Button Pressed");
      Serial.print("Button0:");     Serial.println(Button0);
      Serial.print("Button35:");     Serial.println(Button35);
    #endif
    BottomButtonStateRfGain = 0;
    BottomButtonStatePreAmp = 0;
    userRfGain(Button0, Button35);
    userPreAmp(Button0, Button35);
  }else if(digitalRead(35) == 0){
    Button0 = 0;
    Button35 = 1;
    #ifdef debug
      Serial.println("Top Button Pressed");
      Serial.print("Button0:");     Serial.println(Button0);
      Serial.print("Button35:");     Serial.println(Button35);
    #endif
    BottomButtonStateRfGain = 0;
    BottomButtonStatePreAmp = 0;
    userRfGain(Button0, Button35);
    userPreAmp(Button0, Button35);
   }
}
