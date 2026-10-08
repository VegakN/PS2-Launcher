#ifndef _DS34COMMON_H_
#define _DS34COMMON_H_
#include <types.h>

#define USB_CLASS_WIRELESS_CONTROLLER 0xE0
#define USB_SUBCLASS_RF_CONTROLLER    0x01
#define USB_PROTOCOL_BLUETOOTH_PROG   0x01

#define SONY_VID            0x12ba // Sony
#define DS34_VID            0x054C // Sony Corporation
#define DS3_PID             0x0268 // PS3 Controller
#define DS4_PID             0x05C4 // PS4 Controller
#define DS4_PID_SLIM        0x09CC // PS4 Slim Controller
#define DS5_PID             0x0CE6 // DualSense
#define DS5_EDGE_PID        0x0DF2 // DualSense Edge
#define GUITAR_HERO_PS3_PID 0x0100 // PS3 Guitar Hero Guitar
#define ROCK_BAND_PS3_PID   0x0200 // PS3 Rock Band Guitar

// NOTE: struct member prefixed with "n" means it's active-low (i.e. value of 0 indicates button is pressed, value 1 is released)
enum DS2ButtonBitNumber {
    DS2BtnBit_Select = 0,
    DS2BtnBit_L3 = 1,
    DS2BtnBit_R3 = 2,
    DS2BtnBit_Start = 3,
    DS2BtnBit_Up = 4,
    DS2BtnBit_Right = 5,
    DS2BtnBit_Down = 6,
    DS2BtnBit_Left = 7,
    DS2BtnBit_L2 = 8,
    DS2BtnBit_R2 = 9,
    DS2BtnBit_L1 = 10,
    DS2BtnBit_R1 = 11,
    DS2BtnBit_Triangle = 12,
    DS2BtnBit_Circle = 13,
    DS2BtnBit_Cross = 14,
    DS2BtnBit_Square = 15,
};


enum DS2Buttons {
    DS2ButtonSelect = (1 << 0),
    DS2ButtonL3 = (1 << 1),
    DS2ButtonR3 = (1 << 2),
    DS2ButtonStart = (1 << 3),
    DS2ButtonUp = (1 << 4),
    DS2ButtonRight = (1 << 5),
    DS2ButtonDown = (1 << 6),
    DS2ButtonLeft = (1 << 7),
    DS2ButtonL2 = (1 << 8),
    DS2ButtonR2 = (1 << 9),
    DS2ButtonL1 = (1 << 10),
    DS2ButtonR1 = (1 << 11),
    DS2ButtonTriangle = (1 << 12),
    DS2ButtonCircle = (1 << 13),
    DS2ButtonCross = (1 << 14),
    DS2ButtonSquare = (1 << 15),
};

struct ds2report
{
    union
    {
        u16 nButtonState;
        struct
        {
            u8 nButtonStateL; // Main buttons low byte (active-low)
            u8 nButtonStateH; // Main buttons high byte (active-low)
        };
        struct
        {
            u16 nSelect   : 1;
            u16 nL3       : 1;
            u16 nR3       : 1;
            u16 nStart    : 1;
            u16 nUp       : 1;
            u16 nRight    : 1;
            u16 nDown     : 1;
            u16 nLeft     : 1;
            u16 nL2       : 1;
            u16 nR2       : 1;
            u16 nL1       : 1;
            u16 nR1       : 1;
            u16 nTriangle : 1;
            u16 nCircle   : 1;
            u16 nCross    : 1;
            u16 nSquare   : 1;
        };
    };
    u8 RightStickX;
    u8 RightStickY;
    u8 LeftStickX;
    u8 LeftStickY;

    u8 PressureRight;
    u8 PressureLeft;
    u8 PressureUp;
    u8 PressureDown;

    u8 PressureTriangle;
    u8 PressureCircle;
    u8 PressureCross;
    u8 PressureSquare;

    u8 PressureL1;
    u8 PressureR1;
    u8 PressureL2;
    u8 PressureR2;

} __attribute__((packed));

/* DualSense USB input report 0x01, translated to the native PS2 pad layout. */
static inline void translate_pad_ds5_usb(const u8 *in, struct ds2report *out)
{
    u16 buttons = 0;
    u8 hat = in[8] & 0x0F;

    if (hat == 0 || hat == 1 || hat == 7) buttons |= DS2ButtonUp;
    if (hat == 1 || hat == 2 || hat == 3) buttons |= DS2ButtonRight;
    if (hat == 3 || hat == 4 || hat == 5) buttons |= DS2ButtonDown;
    if (hat == 5 || hat == 6 || hat == 7) buttons |= DS2ButtonLeft;
    if (in[8] & 0x10) buttons |= DS2ButtonSquare;
    if (in[8] & 0x20) buttons |= DS2ButtonCross;
    if (in[8] & 0x40) buttons |= DS2ButtonCircle;
    if (in[8] & 0x80) buttons |= DS2ButtonTriangle;
    if (in[9] & 0x01) buttons |= DS2ButtonL1;
    if (in[9] & 0x02) buttons |= DS2ButtonR1;
    if (in[9] & 0x04) buttons |= DS2ButtonL2;
    if (in[9] & 0x08) buttons |= DS2ButtonR2;
    if ((in[9] & 0x10) || (in[10] & 0x02)) buttons |= DS2ButtonSelect;
    if (in[9] & 0x20) buttons |= DS2ButtonStart;
    if (in[9] & 0x40) buttons |= DS2ButtonL3;
    if (in[9] & 0x80) buttons |= DS2ButtonR3;

    out->nButtonState = (u16)~buttons;
    out->RightStickX = in[3];
    out->RightStickY = in[4];
    out->LeftStickX = in[1];
    out->LeftStickY = in[2];
    out->PressureRight = (buttons & DS2ButtonRight) ? 0xFF : 0;
    out->PressureLeft = (buttons & DS2ButtonLeft) ? 0xFF : 0;
    out->PressureUp = (buttons & DS2ButtonUp) ? 0xFF : 0;
    out->PressureDown = (buttons & DS2ButtonDown) ? 0xFF : 0;
    out->PressureTriangle = (buttons & DS2ButtonTriangle) ? 0xFF : 0;
    out->PressureCircle = (buttons & DS2ButtonCircle) ? 0xFF : 0;
    out->PressureCross = (buttons & DS2ButtonCross) ? 0xFF : 0;
    out->PressureSquare = (buttons & DS2ButtonSquare) ? 0xFF : 0;
    out->PressureL1 = (buttons & DS2ButtonL1) ? 0xFF : 0;
    out->PressureR1 = (buttons & DS2ButtonR1) ? 0xFF : 0;
    out->PressureL2 = in[5];
    out->PressureR2 = in[6];
}

struct ds3report
{
    union
    {
        u16 ButtonState;
        struct
        {
            u8 ButtonStateL; // Main buttons low byte
            u8 ButtonStateH; // Main buttons high byte
        };
        struct
        {
            u16 Select   : 1;
            u16 L3       : 1;
            u16 R3       : 1;
            u16 Start    : 1;
            u16 Up       : 1;
            u16 Right    : 1;
            u16 Down     : 1;
            u16 Left     : 1;
            u16 L2       : 1;
            u16 R2       : 1;
            u16 L1       : 1;
            u16 R1       : 1;
            u16 Triangle : 1;
            u16 Circle   : 1;
            u16 Cross    : 1;
            u16 Square   : 1;
        };
    };
    u8 PSButton;         // PS button
    u8 Reserved1;        // Unknown
    u8 LeftStickX;       // left Joystick X axis 0 - 255, 128 is mid
    u8 LeftStickY;       // left Joystick Y axis 0 - 255, 128 is mid
    u8 RightStickX;      // right Joystick X axis 0 - 255, 128 is mid
    u8 RightStickY;      // right Joystick Y axis 0 - 255, 128 is mid
    u8 Reserved2[4];     // Unknown
    u8 PressureUp;       // digital Pad Up button Pressure 0 - 255
    u8 PressureRight;    // digital Pad Right button Pressure 0 - 255
    u8 PressureDown;     // digital Pad Down button Pressure 0 - 255
    u8 PressureLeft;     // digital Pad Left button Pressure 0 - 255
    u8 PressureL2;       // digital Pad L2 button Pressure 0 - 255
    u8 PressureR2;       // digital Pad R2 button Pressure 0 - 255
    u8 PressureL1;       // digital Pad L1 button Pressure 0 - 255
    u8 PressureR1;       // digital Pad R1 button Pressure 0 - 255
    u8 PressureTriangle; // digital Pad Triangle button Pressure 0 - 255
    u8 PressureCircle;   // digital Pad Circle button Pressure 0 - 255
    u8 PressureCross;    // digital Pad Cross button Pressure 0 - 255
    u8 PressureSquare;   // digital Pad Square button Pressure 0 - 255
    u8 Reserved3[3];     // Unknown
    u8 Charge;           // charging status ? 02 = charge, 03 = normal
    u8 Power;            // Battery status ? 05=full - 02=dying, 01=just before shutdown, EE=charging
    u8 Connection;       // Connection Type ? 14 when operating by bluetooth, 10 when operating by bluetooth with cable plugged in, 16 when bluetooh and rumble
    u8 Reserved4[9];     // Unknown
    s16 AccelX;
    s16 AccelY;
    s16 AccelZ;
    s16 GyroZ;

} __attribute__((packed));

struct ds3guitarreport
{
    union
    {
        u16 ButtonState;
        struct
        {
            u8 ButtonStateL; // Main buttons low byte
            u8 ButtonStateH; // Main buttons high byte
        };
        struct
        {
            u16 Blue      : 1;
            u16 Green     : 1;
            u16 Red       : 1;
            u16 Yellow    : 1;
            u16 Orange    : 1;
            u16 StarPower : 1;
            u16           : 1;
            u16           : 1;
            u16 Select    : 1;
            u16 Start     : 1;
            u16           : 1;
            u16           : 1;
            u16 PSButton  : 1;
            u16           : 1;
            u16           : 1;
            u16           : 1;
        };
    };
    u8 Dpad; // hat format, 0x08 is released, 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
    u8 : 8;
    u8 : 8;
    u8 Whammy; // Whammy axis 0 - 255, 128 is mid
    u8 : 8;
    u8 PressureRightYellow; // digital Pad Right + Yellow button Pressure 0 - 255 (if both are pressed, then they cancel eachother out)
    u8 PressureLeft;        // digital Pad Left button Pressure 0 - 255
    u8 PressureUpGreen;     // digital Pad Up + Green button Pressure 0 - 255 (if both are pressed, then they cancel eachother out)
    u8 PressureDownOrange;  // digital Pad Down + Orange button Pressure 0 - 255 (if both are pressed, then they cancel eachother out)
    u8 PressureBlue;        // digital Pad Blue button Pressure 0 - 255
    u8 PressureRed;         // digital Pad Red button Pressure 0 - 255
    u8 Reserved3[6];        // Unknown
    s16 AccelX;
    s16 AccelZ;
    s16 AccelY;
    s16 GyroZ;

} __attribute__((packed));

enum DS4DpadDirections {
    DS4DpadDirectionN = 0,
    DS4DpadDirectionNE,
    DS4DpadDirectionE,
    DS4DpadDirectionSE,
    DS4DpadDirectionS,
    DS4DpadDirectionSW,
    DS4DpadDirectionW,
    DS4DpadDirectionNW,
    DS4DpadDirectionReleased,
};

struct ds4report
{
    u8 ReportID;
    u8 LeftStickX;   // left Joystick X axis 0 - 255, 128 is mid
    u8 LeftStickY;   // left Joystick Y axis 0 - 255, 128 is mid
    u8 RightStickX;  // right Joystick X axis 0 - 255, 128 is mid
    u8 RightStickY;  // right Joystick Y axis 0 - 255, 128 is mid
    u8 Dpad     : 4; // hat format, 0x08 is released, 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
    u8 Square   : 1;
    u8 Cross    : 1;
    u8 Circle   : 1;
    u8 Triangle : 1;
    u8 L1       : 1;
    u8 R1       : 1;
    u8 L2       : 1;
    u8 R2       : 1;
    u8 Share    : 1;
    u8 Option   : 1;
    u8 L3       : 1;
    u8 R3       : 1;
    u8 PSButton : 1;
    u8 TPad     : 1;
    u8 Counter1 : 6; // counts up by 1 per report
    u8 PressureL2;   // digital Pad L2 button Pressure 0 - 255
    u8 PressureR2;   // digital Pad R2 button Pressure 0 - 255
    u8 Counter2;
    u8 Counter3;
    u8 Battery; // battery level from 0x00 to 0xff
    s16 AccelX;
    s16 AccelY;
    s16 AccelZ;
    s16 GyroZ;
    s16 GyroY;
    s16 GyroX;
    u8 Reserved1[5];    // Unknown
    u8 Power       : 4; // from 0x0 to 0xA - charging, 0xB - charged
    u8 Usb_plugged : 1;
    u8 Headphones  : 1;
    u8 Microphone  : 1;
    u8 Padding     : 1;
    u8 Reserved2[2];        // Unknown
    u8 TPpack;              // number of trackpad packets (0x00 to 0x04)
    u8 PackCounter;         // packet counter
    u8 Finger1ID      : 7;  // counter
    u8 nFinger1Active : 1;  // 0 - active, 1 - unactive
    u16 Finger1X      : 12; // finger 1 coordinates resolution 1920x943
    u16 Finger1Y      : 12;
    u8 Finger2ID      : 7;
    u8 nFinger2Active : 1;
    u16 Finger2X      : 12; // finger 2 coordinates resolution 1920x943
    u16 Finger2Y      : 12;

} __attribute__((packed));

/**
 * Translate DS3 pad data into DS2 pad data.
 * @param in DS3 report
 * @param out DS2 report
 * @param pressure_emu set to 1 to extrapolate digital buttons into button pressure
 * NOTE: if set to 0, ds3report must be large enough for that data to be read!
 */
void translate_pad_ds3(const struct ds3report *in, struct ds2report *out, u8 pressure_emu);

/**
 * Translate PS3 Guitar pad data into DS2 Guitar pad data.
 * @param in PS3 Guitar report
 * @param out PS2 Guitar report
 * @param guitar_hero_format set to 1 if this is a guitar hero guitar, set to 0 if this is a rock band guitar
 */
void translate_pad_guitar(const struct ds3guitarreport *in, struct ds2report *out, u8 guitar_hero_format);

/**
 * Translate DS3 pad data into DS2 pad data.
 * @param in DS4 report
 * @param out DS2 report
 * @param have_touchpad set to 1 if input report has touchpad data
 * NOTE: if set to 1, ds4report must be large enough for that data to be read!
 */
void translate_pad_ds4(const struct ds4report *in, struct ds2report *out, u8 have_touchpad);

#endif
