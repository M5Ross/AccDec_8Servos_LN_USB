// **************************************************************** //
//                           CONFIGRATION                           //
// **************************************************************** //

// by Mattia Rossetti 3/2/2019

// ******** UNLESS YOU WANT ALL CV'S RESET UPON EVERY POWER UP
// ******** AFTER THE INITIAL DECODER LOAD REMOVE THE "//" IN THE FOOLOWING LINE!!
#define DECODER_LOADED

#define Accessory_Address    1           // THIS ADDRESS IS THE START OF THE SWITCHES RANGE
                                         // EXAMPLE: 1 => 1,2,3,4,5,6,7,8
                                         //          5 => 5,6,7,8,9,10,11,12

// Uncomment if you wont to set turnaut to the last known position in CV
#define SET_LAST_SAVED_POSITION

// Uncomment if you wont to set specific address for each output
//#define USE_MULTIPLE_ADDRESS

// Uncomment to invert direction of all outputs by default
//#define INVERT_DIRECTION

// Uncomment if you wont to enable by default Loconet-USB gateway
//#define LOCONET_USB_COM

// ******** REMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR
//#define DEBUG

// ******** REMOVE THE "//" IN THE FOOLOWING LINE TO SEND / RECEIVE
// ******** CONFIGURATION TO THE SERIAL MONITOR
#define SERIALCOM


// ******** INCLUDE CUSTOM CV CONFIGURATION FILE IF REQUIRED
#include <confCV.h>
#include "CVlist.h"
