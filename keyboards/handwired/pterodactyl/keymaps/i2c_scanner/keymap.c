#include QMK_KEYBOARD_H

#include "i2c_master.h"
#include "debug.h"
#include "version.h"
#include "bno055_support.h"

#define TIMEOUT 50

enum
{
    _BL = 0,
    _SYMB,
    _MDIA,
};

// TODO: remove patch
#ifdef PROTOCOL_CHIBIOS
#    pragma message("ChibiOS is currently 'best effort' and might not report accurate results")

i2c_status_t i2c_start_bodge(uint8_t address, uint16_t timeout) {
    i2c_start(address);

    // except on ChibiOS where the only way is do do "something"
    uint8_t data = 0;
    return i2c_readReg(address, 0, &data, sizeof(data), TIMEOUT);
}

#    define i2c_start i2c_start_bodge
#endif



void do_scan(void) {
    uint8_t nDevices = 0;

    dprintf("Scanning...\n");

    for (uint8_t address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // i2c_start to see if a device did acknowledge to the address.
        i2c_status_t error = i2c_start(address << 1, TIMEOUT);
        if (error == I2C_STATUS_SUCCESS) {
            i2c_stop();
            dprintf("  I2C device found at address 0x%02X\n", address);
            nDevices++;
        } else {
            // dprintf("  Unknown error (%u) at address 0x%02X\n", error, address);
        }
    }

    if (nDevices == 0)
        dprintf("No I2C devices found\n");
    else
        dprintf("done\n");
}

uint16_t scan_timer = 0;

void matrix_scan_user(void) {
    if (timer_elapsed(scan_timer) > 5000) {
        do_scan();
        scan_timer = timer_read();
    }
}

void keyboard_post_init_user(void) {
    debug_enable = true;
    debug_matrix = true;

    i2c_init();
    scan_timer = timer_read();
}

// #define BASE 0 // default layer
// #define TIMEOUT 200

// static struct bno055_t myBNO;
// static struct bno055_euler myEulerData;
// uint16_t scan_timer = 0;
// // unsigned char bno055_i2c_address = BNO055_I2C_ADDR << 1;

// void read_bno055(void){

//     i2c_status_t error = i2c_start(BNO055_I2C_ADDR, I2C_TIMEOUT);
//     if (error == I2C_STATUS_SUCCESS) {
//         BNO_Init(&myBNO);
//         bno055_set_operation_mode(OPERATION_MODE_CONFIG);

//         // use low power mode where accelerometer is always on
//         // unsigned char mode = 1;
//         // int status = BNO055_I2C_bus_write(BNO055_I2C_ADDR, BNO055_PWR_MODE_ADDR, &mode, I2C_TIMEOUT);
//         // dprintf("%d ", status);
//         // dprintf("%d ", BNO055_I2C_ADDR);

//         dprintf("%d ", myBNO.dev_addr);
//         dprintf("%d ", myBNO.sw_revision_id);

//         unsigned char accelCalibStatus = 0;   //Variable to hold the calibration status of the Accelerometer
//         unsigned char magCalibStatus = 0;   //Variable to hold the calibration status of the Magnetometer
//         unsigned char gyroCalibStatus = 0;    //Variable to hold the calibration status of the Gyroscope
//         unsigned char sysCalibStatus = 0;   //Variable to hold the calibration status of the System (BNO055's MCU)
//         bno055_get_accelcalib_status(&accelCalibStatus);  //To read out the Acceleration Calibration Status (0-3)
//         bno055_get_magcalib_status(&magCalibStatus);      //To read out the Magnetometer Calibration Status (0-3)
//         bno055_get_gyrocalib_status(&gyroCalibStatus);     //To read out the Gyroscope Calibration Status (0-3)
//         bno055_get_syscalib_status(&sysCalibStatus);      //To read out the System Calibration Status (0-3)
//         dprintf("%d ", accelCalibStatus);
//         dprintf("%d ", magCalibStatus);
//         dprintf("%d ", gyroCalibStatus);
//         dprintf("%d ", sysCalibStatus);
//         dprintf(" calibration\n");

//         int status = bno055_set_operation_mode(OPERATION_MODE_NDOF);
//         dprintf("%d ", status);
//         status = bno055_read_euler_hrp(&myEulerData);
//         dprintf("%d ", status);

//         // int x = int(float(myEulerData.r) / 16.0) * 100;     // roll in degrees
//         // int y = int(float(myEulerData.p) / 16.0) * 100;     // pitch in degrees
//         // int z = int(float(myEulerData.h) / 16.0) * 100;     // heading in degrees
//         dprintf("%d ",myEulerData.r);
//         dprintf("%d ",myEulerData.p);
//         dprintf("%d ",myEulerData.h);
//         dprintf(" euler\n");

//         i2c_stop();
//     } else {
//         // dprintf("  Unknown error (%u) at address %d\n", error, BNO055_I2C_ADDR);
//     }
// }


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
/* Keymap 0: Basic layer
 *
 * ,-----------------------------------------.                    ,-----------------------------------------.
 * |   =  |   1  |   2  |   3  |   4  |   5  |                    |   6  |   7  |   8  |   9  |   0  |  -   |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * | Del  |   Q  |   W  |   E  |   R  |   T  |                    |   Y  |   U  |   I  |   O  |   P  |  \   |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * | BkSp |   A  |   S  |   D  |   F  |   G  |                    |   H  |   J  |   K  |   L  |; / L2|'/Cmd |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |LShift|Z/Ctrl|   X  |   C  |   V  |   B  |                    |   N  |   M  |   ,  |   .  |//Ctrl|RShift|
 * |------+------+------+------+------+------'                    `------+------+------+------+------+------|
 * |Grv/L1|  '"  |AltShf| Left | Right|                                  |  Up  | Down |   [  |   ]  | ~L1  |
 * `----------------------------------'                                  `----------------------------------'
 *                                      ,-------------.  ,-------------.
 *                                      | App  | LGui |  | Alt  | ^/Esc|
 *                               ,------|------|------|  |------+------+------.
 *                               |      |      | Home |  | PgUp |      |      |
 *                               | Space|Backsp|------|  |------|  Tab |Enter |
 *                               |      |ace   | End  |  | PgDn |      |      |
 *                               `--------------------'  `--------------------'
 */
[_BL] = LAYOUT_pterodactyl(  // layer 0 : default
        // left hand
          KC_EQL,          KC_1,           KC_2,     KC_3,     KC_4,  KC_5,                             KC_6,   KC_7,     KC_8,     KC_9,               KC_0,         KC_MINS,
          KC_DEL,          KC_Q,           KC_W,     KC_E,     KC_R,  KC_T,                             KC_Y,   KC_U,     KC_I,     KC_O,               KC_P,         KC_BSLS,
          KC_BSPC,         KC_A,           KC_S,     KC_D,     KC_F,  KC_G,                             KC_H,   KC_J,     KC_K,     KC_L,  LT(_MDIA, KC_SCLN),  GUI_T(KC_QUOT),
          KC_LSFT,  CTL_T(KC_Z),           KC_X,     KC_C,     KC_V,  KC_B,                             KC_N,   KC_M,  KC_COMM,   KC_DOT,     CTL_T(KC_SLSH),         KC_RSFT,
  LT(_SYMB,KC_GRV),      KC_QUOT,  LALT(KC_LSFT),  KC_LEFT,  KC_RGHT,                                           KC_UP, KC_DOWN,  KC_LBRC,            KC_RBRC,       TT(_SYMB),

                                                                 ALT_T(KC_APP),  KC_LGUI,      KC_RALT,  CTL_T(KC_ESC),
                                                                                 KC_HOME,      KC_PGUP,
                                                          KC_SPC,      KC_BSPC,   KC_END,      KC_PGDN, KC_TAB, KC_ENT
),
/* Keymap 1: Symbol Layer
 *
 * ,-----------------------------------------.                    ,-----------------------------------------.
 * |Versn |  F1  |  F2  |  F3  |  F4  |  F5  |                    |  F6  |  F7  |  F8  |  F9  |  F10 |  F11 |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |   !  |   @  |   {  |   }  |   |  |                    |  Up  |   7  |   8  |   9  |   *  |  F12 |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |   #  |   $  |   (  |   )  |   `  |                    | Down |   4  |   5  |   6  |   +  |      |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |   %  |   ^  |   [  |   ]  |   ~  |                    |   &  |   1  |   2  |   3  |   \  |      |
 * |------+------+------+------+------+------'                    `------+------+------+------+------+------|
 * |RESET |      |      |      |      |                                  |      |   .  |   0  |   =  |      |
 * `----------------------------------'                                  `----------------------------------'
 *                                      ,-------------.  ,-------------.
 *                                      |      |      |  |      |      |
 *                               ,------|------|------|  |------+------+------.
 *                               |      |      |      |  |      |      |      |
 *                               |      |      |------|  |------|      |      |
 *                               |      |      |      |  |      |      |      |
 *                               `--------------------'  `--------------------'
 */
// SYMBOLS
[_SYMB] = LAYOUT_pterodactyl(
       // left hand
       KC_TRNS,    KC_F1,    KC_F2,    KC_F3,    KC_F4,    KC_F5,                KC_F6,      KC_F7,   KC_F8, KC_F9,  KC_F10,   KC_F11,
       KC_TRNS,  KC_EXLM,    KC_AT,  KC_LCBR,  KC_RCBR,  KC_PIPE,                KC_UP,       KC_7,    KC_8,  KC_9, KC_ASTR,   KC_F12,
       KC_TRNS,  KC_HASH,   KC_DLR,  KC_LPRN,  KC_RPRN,   KC_GRV,                KC_DOWN,     KC_4,    KC_5,  KC_6, KC_PLUS,  KC_TRNS,
       KC_TRNS,  KC_PERC,  KC_CIRC,  KC_LBRC,  KC_RBRC,  KC_TILD,                KC_AMPR,     KC_1,    KC_2,  KC_3, KC_BSLS,  KC_TRNS,
         RESET,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,                                    KC_TRNS,  KC_DOT,  KC_0,  KC_EQL,  KC_TRNS,

                                                     KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,
                                                               KC_TRNS,      KC_TRNS,
                                           KC_TRNS,  KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,  KC_TRNS
),
/* Keymap 2: Media and mouse keys
 * ,-----------------------------------------.                    ,-----------------------------------------.
 * |      |      |      |      |      |      |                    |      |      |      |      |      |      |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |      |      | MsUp |      |      |                    |      |      |      |      |      |      |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |      |MsLeft|MsDown|MsRght|      |                    |      |      |      |      |      | Play |
 * |------+------+------+------+------+------|                    |------+------+------+------+------+------|
 * |      |      |      |      |      |      |                    |      |      | Prev | Next |      |      |
 * |------+------+------+------+------+------'                    `------+------+------+------+------+------|
 * |      |      |      | Lclk | Rclk |                                  | VolUp| VolDn| Mute |      |      |
 * `----------------------------------'                                  `----------------------------------'
 *                                      ,-------------.  ,-------------.
 *                                      |      |      |  |      |      |
 *                               ,------|------|------|  |------+------+------.
 *                               |      |      |      |  |      |      |Brwser|
 *                               |      |      |------|  |------|      |Back  |
 *                               |      |      |      |  |      |      |      |
 *                               `--------------------'  `--------------------'
 *
 */
// MEDIA AND MOUSE
[_MDIA] = LAYOUT_pterodactyl(
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_MS_U,  KC_TRNS,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,
       KC_TRNS,  KC_TRNS,  KC_MS_L,  KC_MS_D,  KC_MS_R,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_MPLY,
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_TRNS,                KC_TRNS,  KC_TRNS,  KC_MPRV,  KC_MNXT,  KC_TRNS,  KC_TRNS,
       KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_BTN1,  KC_BTN2,                                    KC_VOLU,  KC_VOLD,  KC_MUTE,  KC_TRNS,  KC_TRNS,
                                                    KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,
                                                              KC_TRNS,      KC_TRNS,
                                          KC_TRNS,  KC_TRNS,  KC_TRNS,      KC_TRNS,  KC_TRNS,  KC_WBAK
),
};


// // TODO: remove patch
// #ifdef PROTOCOL_CHIBIOS
// #    pragma message("ChibiOS is currently 'best effort' and might not report accurate results")

// i2c_status_t i2c_start_bodge(uint8_t address, uint16_t timeout) {
//     i2c_start(address);

//     // except on ChibiOS where the only way is do do "something"
//     uint8_t data = 0;
//     return i2c_readReg(address, 0, &data, sizeof(data), timeout);
// }

// #    define i2c_start i2c_start_bodge
// #endif

// // const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
// //     LAYOUT_ortho_1x1(KC_A)
// // };

// void do_scan(void) {
//     uint8_t nDevices = 0;

//     dprintf("Scanning...\n");

//     for (uint8_t address = 1; address < 127; address++) {
//         // The i2c_scanner uses the return value of
//         // i2c_start to see if a device did acknowledge to the address.
//         i2c_status_t error = i2c_start(address << 1, I2C_TIMEOUT);
//         if (error == I2C_STATUS_SUCCESS) {
//             i2c_stop();
//             dprintf("  I2C device found at address %d\n", address);
//             nDevices++;
//         } else {
//             // dprintf("  Unknown error (%u) at address %d\n", error, address);
//         }
//     }

//     if (nDevices == 0)
//         dprintf("No I2C devices found\n");
//     else
//         dprintf("done\n");
// }


// void keyboard_post_init_user(void) {
//     debug_enable = true;
//     debug_matrix = true;

//     i2c_init(); // BNO055_I2C_ADDR

//     scan_timer = timer_read();

// //     BNO_Init(&myBNO); //Assigning the structure to hold information about the device
// //     bno055_set_operation_mode(OPERATION_MODE_NDOF);
// // dprintf("i2c_init %d %d",myBNO.chip_id,myBNO.sw_revision_id);
// }



// void matrix_scan_user(void) {
//         // read_bno055();

//     if (timer_elapsed(scan_timer) > 5000) {
//         do_scan();
//         // read_bnos055();

//         scan_timer = timer_read();
//     }
// }
