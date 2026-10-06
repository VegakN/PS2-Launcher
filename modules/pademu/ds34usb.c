#include "types.h"
#include "loadcore.h"
#include "stdio.h"
#include "sifrpc.h"
#include "sysclib.h"
#include "usbd.h"
#include "usbd_macro.h"
#include "thbase.h"
#include "thsemap.h"
#include "ds34usb.h"
#include "sys_utils.h"
#include "padmacro.h"

// #define DPRINTF(x...) printf(x)
#define DPRINTF(x...)

#define REQ_USB_OUT (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE)
#define REQ_USB_IN  (USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE)

#define MAX_PADS 4
#define XBOX_VENDOR_MICROSOFT 0x045E
#define XBOXUSB_INPUT_PACKET  0x20

static u8 output_01_report[] =
    {
        0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x02,
        0xff, 0x27, 0x10, 0x00, 0x32,
        0xff, 0x27, 0x10, 0x00, 0x32,
        0xff, 0x27, 0x10, 0x00, 0x32,
        0xff, 0x27, 0x10, 0x00, 0x32,
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00};

static u8 led_patterns[][2] =
    {
        {0x1C, 0x02},
        {0x1A, 0x04},
        {0x16, 0x08},
        {0x0E, 0x10},
};

static u8 power_level[] =
    {
        0x00, 0x00, 0x02, 0x06, 0x0E, 0x1E};

static const u8 xboxone_power_on[] = {0x05, 0x20, 0x00, 0x01, 0x00};
static const u8 xboxone_s_init[] = {0x05, 0x20, 0x00, 0x0F, 0x06};
static const u8 xboxone_led_on[] = {0x0A, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14};
static const u8 xboxone_auth_done[] = {0x06, 0x20, 0x00, 0x02, 0x01, 0x00};
static const u8 xbox360_led_p1[] = {0x01, 0x03, 0x02};
static const u8 xbox360_init_cmd[] = {0x01, 0x03, 0x10};
static u8 usb_ctrl_buf[MAX_PADS][32] __attribute((aligned(4))) = {{0}};

static u8 rgbled_patterns[][2][3] =
    {
        {{0x00, 0x00, 0x10}, {0x00, 0x00, 0x7F}}, // light blue/blue
        {{0x00, 0x10, 0x00}, {0x00, 0x7F, 0x00}}, // light green/green
        {{0x10, 0x10, 0x00}, {0x7F, 0x7F, 0x00}}, // light yellow/yellow
        {{0x00, 0x10, 0x10}, {0x00, 0x7F, 0x7F}}, // light cyan/cyan
};

static u8 usb_in_buf[MAX_PADS][MAX_BUFFER_SIZE + 32] __attribute((aligned(4))) = {{0}};
static u8 usb_out_buf[MAX_PADS][MAX_BUFFER_SIZE + 32] __attribute((aligned(4))) = {{0}};
static u8 xbox_poll_buf[MAX_PADS][MAX_BUFFER_SIZE + 32] __attribute((aligned(4))) = {{0}};
static int usb_resulCode[MAX_PADS] = {1, 1, 1, 1};
static int xbox_poll_sema = -1;
static int xbox_poll_result = 1;
static int xbox_poll_tid = -1;
static int xbox_poll_pending = 0;
static short xbox_axis_center[MAX_PADS][4] = {{0}};
static u8 xbox_axis_center_valid[MAX_PADS] = {0};

int usb_probe(int devId);
int usb_connect(int devId);
int usb_disconnect(int devId);

static void usb_release(int pad);
static void usb_config_set(int result, int count, void *arg);

UsbDriver usb_driver = {NULL, NULL, "ds34usb", usb_probe, usb_connect, usb_disconnect};

static void DS3USB_init(int pad);
static void readReport(u8 *data, int pad);
static int LEDRumble(u8 *led, u8 lrum, u8 rrum, int pad);
static void TransferWait(int sema);
static void TransferWaitTimeout(int sema, u32 timeout_lo);
static int xboxusb_is_input_packet(const u8 *data);
static void xboxusb_poll_thread(void *arg);
static void xboxusb_poll_cb(int resultCode, int bytes, void *arg);
static int xboxusb_axis_to_ds2_centered(int pad, int axis, u8 low, u8 high, int invert);
static void xboxusb_update_axis_center(int pad, const u8 *data);
static void xboxusb_apply_buttons(const u8 *in, struct ds2report *out);
static void xboxusb_translate_input(int pad, const u8 *in, struct ds2report *out);
static int xboxusb_send_packet(int pad, const u8 *data, int len);
static int xboxusb_send_packet_raw(int pad, const u8 *data, int len);
static int xboxusb_send_init(int pad);

ds34usb_device ds34pad[MAX_PADS];

static int is_xbox_vendor(u16 vid, u16 pid)
{
    if (vid == XBOX_VENDOR_MICROSOFT) return 1;
    if (vid == 0x0E6F || vid == 0x0738 || vid == 0x1532 || vid == 0x24C6 || vid == 0x11C0 || vid == 0x256F || vid == 0x146B || vid == 0x0F0D || vid == 0x20D6 || vid == 0x057E) return 1;
    if (pid == 0x028E || pid == 0x028F || pid == 0x02A1 || pid == 0x0719 || pid == 0x0291 || pid == 0x02EA || pid == 0x0B12 || pid == 0x0B13) return 1;
    return 0;
}

static int is_gamepad_device(int devId, UsbDeviceDescriptor *device)
{
    UsbConfigDescriptor *config;

    if (device == NULL)
        return 0;

    if (device->idVendor == SONY_VID && (device->idProduct == GUITAR_HERO_PS3_PID || device->idProduct == ROCK_BAND_PS3_PID))
        return 1;

    if (device->idVendor == DS34_VID && (device->idProduct == DS3_PID || device->idProduct == DS4_PID || device->idProduct == DS4_PID_SLIM))
        return 1;

    if (is_xbox_vendor(device->idVendor, device->idProduct))
        return 1;

    config = (UsbConfigDescriptor *)UsbGetDeviceStaticDescriptor(devId, device, USB_DT_CONFIG);
    if (config == NULL)
        return 0;

    u8 *desc = (u8 *)config + config->bLength;
    int len = config->wTotalLength - config->bLength;
    int has_gamepad_intf = 0;

    while (len > 0) {
        u8 bLength = desc[0];
        u8 bDescriptorType = desc[1];

        if (bLength == 0) break;

        if (bDescriptorType == USB_DT_INTERFACE) {
            UsbInterfaceDescriptor *intf = (UsbInterfaceDescriptor *)desc;
            if (intf->bInterfaceClass == 0x03 || intf->bInterfaceClass == 0xFF)
                has_gamepad_intf = 1;
        }

        desc += bLength;
        len -= bLength;
    }

    return has_gamepad_intf;
}

int usb_probe(int devId)
{
    UsbDeviceDescriptor *device = NULL;

    DPRINTF("DS34USB: probe: devId=%i\n", devId);

    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    return is_gamepad_device(devId, device);
}

int usb_connect(int devId)
{
    int pad;
    UsbDeviceDescriptor *device;
    UsbConfigDescriptor *config;

    DPRINTF("DS34USB: connect: devId=%i\n", devId);

    for (pad = 0; pad < MAX_PADS; pad++) {
        if (ds34pad[pad].devId == -1 && ds34pad[pad].enabled)
            break;
    }

    if (pad >= MAX_PADS) {
        DPRINTF("DS34USB: Error - only %d device allowed !\n", MAX_PADS);
        return 1;
    }

    PollSema(ds34pad[pad].sema);

    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    config = (UsbConfigDescriptor *)UsbGetDeviceStaticDescriptor(devId, device, USB_DT_CONFIG);
    if (device == NULL || config == NULL) {
        SignalSema(ds34pad[pad].sema);
        return 1;
    }

    ds34pad[pad].devId = devId;
    ds34pad[pad].vid = device->idVendor;
    ds34pad[pad].pid = device->idProduct;

    ds34pad[pad].status = DS34USB_STATE_AUTHORIZED;
    ds34pad[pad].controlEndp = UsbOpenEndpoint(devId, NULL);
    ds34pad[pad].interruptEndp = -1;
    ds34pad[pad].outEndp = -1;

    if (device->idProduct == DS3_PID) {
        ds34pad[pad].type = DS3;
    } else if (device->idProduct == GUITAR_HERO_PS3_PID) {
        ds34pad[pad].type = GUITAR_GH;
    } else if (device->idProduct == ROCK_BAND_PS3_PID) {
        ds34pad[pad].type = GUITAR_RB;
    } else if (device->idVendor == DS34_VID && (device->idProduct == DS4_PID || device->idProduct == DS4_PID_SLIM)) {
        ds34pad[pad].type = DS4;
    } else {
        ds34pad[pad].type = XBOX_USB;
        ds34pad[pad].analog_btn = 1;
        xbox_axis_center_valid[pad] = 0;
    }

    u8 *desc = (u8 *)config + config->bLength;
    int len = config->wTotalLength - config->bLength;

    while (len > 0) {
        u8 bLength = desc[0];
        u8 bDescriptorType = desc[1];

        if (bLength == 0) break;

        if (bDescriptorType == USB_DT_ENDPOINT) {
            UsbEndpointDescriptor *ep = (UsbEndpointDescriptor *)desc;

            if (ep->bmAttributes == USB_ENDPOINT_XFER_INT) {
                if ((ep->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN && ds34pad[pad].interruptEndp < 0) {
                    ds34pad[pad].interruptEndp = UsbOpenEndpointAligned(devId, ep);
                    DPRINTF("DS34USB: register Event endpoint id =%i addr=%02X packetSize=%i\n", ds34pad[pad].interruptEndp, ep->bEndpointAddress, (unsigned short int)ep->wMaxPacketSizeHB << 8 | ep->wMaxPacketSizeLB);
                }
                if ((ep->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT && ds34pad[pad].outEndp < 0) {
                    ds34pad[pad].outEndp = UsbOpenEndpointAligned(devId, ep);
                    DPRINTF("DS34USB: register Output endpoint id =%i addr=%02X packetSize=%i\n", ds34pad[pad].outEndp, ep->bEndpointAddress, (unsigned short int)ep->wMaxPacketSizeHB << 8 | ep->wMaxPacketSizeLB);
                }
            }
        }

        desc += bLength;
        len -= bLength;
    }

    if (ds34pad[pad].interruptEndp < 0) {
        usb_release(pad);
        return 1;
    }

    ds34pad[pad].status |= DS34USB_STATE_CONNECTED;

    UsbSetDeviceConfiguration(ds34pad[pad].controlEndp, config->bConfigurationValue, usb_config_set, (void *)pad);
    SignalSema(ds34pad[pad].sema);

    return 0;
}

int usb_disconnect(int devId)
{
    u8 pad;

    DPRINTF("DS34USB: disconnect: devId=%i\n", devId);

    for (pad = 0; pad < MAX_PADS; pad++) {
        if (ds34pad[pad].devId == devId)
            break;
    }

    if (pad < MAX_PADS)
        usb_release(pad);

    return 0;
}

static void usb_release(int pad)
{
    PollSema(ds34pad[pad].sema);

    if (ds34pad[pad].interruptEndp >= 0)
        UsbCloseEndpoint(ds34pad[pad].interruptEndp);

    if (ds34pad[pad].outEndp >= 0)
        UsbCloseEndpoint(ds34pad[pad].outEndp);

    ds34pad[pad].controlEndp = -1;
    ds34pad[pad].interruptEndp = -1;
    ds34pad[pad].outEndp = -1;
    ds34pad[pad].devId = -1;
    ds34pad[pad].vid = 0;
    ds34pad[pad].pid = 0;
    ds34pad[pad].xbox_seq = 0;
    ds34pad[pad].status = DS34USB_STATE_DISCONNECTED;
    xbox_axis_center_valid[pad] = 0;

    SignalSema(ds34pad[pad].sema);
}

static void usb_data_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;

    // DPRINTF("DS34USB: usb_data_cb: res %d, bytes %d, arg %p \n", resultCode, bytes, arg);

    usb_resulCode[pad] = resultCode;

    SignalSema(ds34pad[pad].sema);
}

static void usb_cmd_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;

    // DPRINTF("DS34USB: usb_cmd_cb: res %d, bytes %d, arg %p \n", resultCode, bytes, arg);

    SignalSema(ds34pad[pad].cmd_sema);
}

static void usb_config_set(int result, int count, void *arg)
{
    int pad = (int)arg;
    u8 led[4];

    PollSema(ds34pad[pad].sema);

    if (result != USB_RC_OK) {
        SignalSema(ds34pad[pad].sema);
        return;
    }

    ds34pad[pad].status |= DS34USB_STATE_CONFIGURED;

    if (ds34pad[pad].type == DS3) {
        DS3USB_init(pad);
        DelayThread(10000);
        led[0] = led_patterns[pad][1];
        led[3] = 0;
    } else if (ds34pad[pad].type == DS4) {
        led[0] = rgbled_patterns[pad][1][0];
        led[1] = rgbled_patterns[pad][1][1];
        led[2] = rgbled_patterns[pad][1][2];
        led[3] = 0;
    } else if (ds34pad[pad].type == XBOX_USB)
        xboxusb_send_init(pad);

    if (ds34pad[pad].type != XBOX_USB)
        LEDRumble(led, 0, 0, pad);

    ds34pad[pad].status |= DS34USB_STATE_RUNNING;

    SignalSema(ds34pad[pad].sema);
}

static void DS3USB_init(int pad)
{
    usb_out_buf[pad][0] = 0x42;
    usb_out_buf[pad][1] = 0x0c;
    usb_out_buf[pad][2] = 0x00;
    usb_out_buf[pad][3] = 0x00;

    UsbControlTransfer(ds34pad[pad].controlEndp, REQ_USB_OUT, USB_REQ_SET_REPORT, (HID_USB_GET_REPORT_FEATURE << 8) | 0xF4, 0, 4, usb_out_buf[pad], NULL, NULL);
}

#define MAX_DELAY 10

static void readReport(u8 *data, int pad_idx)
{
    ds34usb_device *pad = &ds34pad[pad_idx];
    if (pad->type == GUITAR_GH || pad->type == GUITAR_RB) {
        struct ds3guitarreport *report;

        report = (struct ds3guitarreport *)data;

        translate_pad_guitar(report, &pad->ds2, pad->type == GUITAR_GH);
        padMacroPerform(&pad->ds2, report->PSButton);
    }
    if (pad->type == XBOX_USB) {
        if (!xboxusb_is_input_packet(data))
            return;

        if (!xbox_axis_center_valid[pad_idx])
            xboxusb_update_axis_center(pad_idx, data);
        xboxusb_translate_input(pad_idx, data, &pad->ds2);
        padMacroPerform(&pad->ds2, 0);
        return;
    }

    if (data[0]) {

        if (pad->type == DS3) {
            struct ds3report *report;

            report = (struct ds3report *)&data[2];

            if (report->RightStickX == 0 && report->RightStickY == 0) // ledrumble cmd causes null report sometime
                return;

            pad->data[0] = ~report->ButtonStateL;
            pad->data[1] = ~report->ButtonStateH;

            translate_pad_ds3(report, &pad->ds2, 0);
            padMacroPerform(&pad->ds2, report->PSButton);
            if (report->PSButton) {                                    // display battery level
                if (report->Select && (pad->btn_delay == MAX_DELAY)) { // PS + SELECT
                    if (pad->analog_btn < 2)                           // unlocked mode
                        pad->analog_btn = !pad->analog_btn;

                    pad->oldled[0] = led_patterns[pad_idx][(pad->analog_btn & 1)];
                    pad->btn_delay = 1;
                } else {
                    if (report->Power <= 0x05)
                        pad->oldled[0] = power_level[report->Power];

                    if (pad->btn_delay < MAX_DELAY)
                        pad->btn_delay++;
                }
            } else {
                pad->oldled[0] = led_patterns[pad_idx][(pad->analog_btn & 1)];

                if (pad->btn_delay > 0)
                    pad->btn_delay--;
            }

            if (report->Power == 0xEE) // charging
                pad->oldled[3] = 1;
            else
                pad->oldled[3] = 0;

        } else if (pad->type == DS4) {
            struct ds4report *report;
            report = (struct ds4report *)data;
            translate_pad_ds4(report, &pad->ds2, 1);
            padMacroPerform(&pad->ds2, report->PSButton);

            if (report->PSButton) {                                   // display battery level
                if (report->Share && (pad->btn_delay == MAX_DELAY)) { // PS + Share
                    if (pad->analog_btn < 2)                          // unlocked mode
                        pad->analog_btn = !pad->analog_btn;

                    pad->oldled[0] = rgbled_patterns[pad_idx][(pad->analog_btn & 1)][0];
                    pad->oldled[1] = rgbled_patterns[pad_idx][(pad->analog_btn & 1)][1];
                    pad->oldled[2] = rgbled_patterns[pad_idx][(pad->analog_btn & 1)][2];
                    pad->btn_delay = 1;
                } else {
                    pad->oldled[0] = report->Battery;
                    pad->oldled[1] = 0;
                    pad->oldled[2] = 0;

                    if (pad->btn_delay < MAX_DELAY)
                        pad->btn_delay++;
                }
            } else {
                pad->oldled[0] = rgbled_patterns[pad_idx][(pad->analog_btn & 1)][0];
                pad->oldled[1] = rgbled_patterns[pad_idx][(pad->analog_btn & 1)][1];
                pad->oldled[2] = rgbled_patterns[pad_idx][(pad->analog_btn & 1)][2];

                if (pad->btn_delay > 0)
                    pad->btn_delay--;
            }

            if (report->Power != 0xB && report->Usb_plugged) // charging
                pad->oldled[3] = 1;
            else
                pad->oldled[3] = 0;
        }
        if (pad->btn_delay > 0) {
            pad->update_rum = 1;
        }
    }
}

static int xboxusb_is_input_packet(const u8 *data)
{
    return 1;
}

static void xboxusb_poll_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;

    xbox_poll_result = resultCode;
    WaitSema(ds34pad[pad].sema);
    if (xbox_poll_result == USB_RC_OK && xboxusb_is_input_packet(xbox_poll_buf[pad])) {
        if (!xbox_axis_center_valid[pad])
            xboxusb_update_axis_center(pad, xbox_poll_buf[pad]);
        xboxusb_translate_input(pad, xbox_poll_buf[pad], &ds34pad[pad].ds2);
        padMacroPerform(&ds34pad[pad].ds2, 0);
    }
    SignalSema(ds34pad[pad].sema);

    xbox_poll_result = 1;
    xbox_poll_pending = 0;
}

static int xboxusb_axis_to_ds2(u8 low, u8 high)
{
    int value = (short)((high << 8) | low);

    if (value > -12000 && value < 12000)
        value = 0;

    value = (value >> 8) + 128;
    if (value < 0)
        value = 0;
    if (value > 255)
        value = 255;

    return value;
}

static void xboxusb_poll_thread(void *arg)
{
    int pad, ret;

    while (1) {
        if (!xbox_poll_pending) {
            for (pad = 0; pad < MAX_PADS; pad++) {
                if (ds34pad[pad].type != XBOX_USB || !(ds34pad[pad].status & DS34USB_STATE_CONFIGURED) || ds34pad[pad].interruptEndp < 0)
                    continue;

                xbox_poll_pending = 1;
                ret = UsbInterruptTransfer(ds34pad[pad].interruptEndp, xbox_poll_buf[pad], MAX_BUFFER_SIZE, xboxusb_poll_cb, (void *)pad);
                if (ret != USB_RC_OK)
                    xbox_poll_pending = 0;
                break;
            }
        }

        DelayThread(16000);
    }
}

static void xboxusb_update_axis_center(int pad, const u8 *data)
{
    const u8 *in = data;
    if (in[0] != XBOXUSB_INPUT_PACKET) return;

    xbox_axis_center[pad][0] = (short)((in[11] << 8) | in[10]);
    xbox_axis_center[pad][1] = (short)((in[13] << 8) | in[12]);
    xbox_axis_center[pad][2] = (short)((in[15] << 8) | in[14]);
    xbox_axis_center[pad][3] = (short)((in[17] << 8) | in[16]);
    xbox_axis_center_valid[pad] = 1;
}

static int xboxusb_axis_to_ds2_centered(int pad, int axis, u8 low, u8 high, int invert)
{
    int value = (short)((high << 8) | low);
    int center = xbox_axis_center_valid[pad] ? xbox_axis_center[pad][axis] : 0;
    int delta = value - center;

    if (delta > -12000 && delta < 12000)
        delta = 0;

    if (invert)
        delta = -delta;

    value = (delta >> 8) + 128;
    if (value < 0)
        value = 0;
    if (value > 255)
        value = 255;

    return value;
}

static void xboxusb_translate_input(int pad, const u8 *in, struct ds2report *out)
{
    u16 buttons = 0;
    u16 lt = 0, rt = 0;
    u8 lx = 128, ly = 128, rx = 128, ry = 128;

    if (in[0] == 0x00) {
        int offset = (in[1] == 0x01 && in[3] == 0xF0) ? 2 : 0;
        u8 b2 = in[2 + offset];
        u8 b3 = in[3 + offset];

        lt = in[4 + offset];
        rt = in[5 + offset];

        if (b2 & 0x01) buttons |= DS2ButtonUp;
        if (b2 & 0x02) buttons |= DS2ButtonDown;
        if (b2 & 0x04) buttons |= DS2ButtonLeft;
        if (b2 & 0x08) buttons |= DS2ButtonRight;
        if (b2 & 0x10) buttons |= DS2ButtonStart;
        if (b2 & 0x20) buttons |= DS2ButtonSelect;
        if (b2 & 0x40) buttons |= DS2ButtonL3;
        if (b2 & 0x80) buttons |= DS2ButtonR3;

        if (b3 & 0x01) buttons |= DS2ButtonL1;
        if (b3 & 0x02) buttons |= DS2ButtonR1;
        if (b3 & 0x10) buttons |= DS2ButtonCross;
        if (b3 & 0x20) buttons |= DS2ButtonCircle;
        if (b3 & 0x40) buttons |= DS2ButtonSquare;
        if (b3 & 0x80) buttons |= DS2ButtonTriangle;

        lx = xboxusb_axis_to_ds2(in[6 + offset], in[7 + offset]);
        ly = 255 - xboxusb_axis_to_ds2(in[8 + offset], in[9 + offset]);
        rx = xboxusb_axis_to_ds2(in[10 + offset], in[11 + offset]);
        ry = 255 - xboxusb_axis_to_ds2(in[12 + offset], in[13 + offset]);

        if (lt > 0) buttons |= DS2ButtonL2;
        if (rt > 0) buttons |= DS2ButtonR2;

        out->nButtonState = ~buttons;
        out->LeftStickX = lx;
        out->LeftStickY = ly;
        out->RightStickX = rx;
        out->RightStickY = ry;

        out->PressureL2 = lt;
        out->PressureR2 = rt;
    } else if (in[0] == 0x20) {
        lt = in[6] | (in[7] << 8);
        rt = in[8] | (in[9] << 8);

        if (in[5] & 0x01) buttons |= DS2ButtonUp;
        if (in[5] & 0x02) buttons |= DS2ButtonDown;
        if (in[5] & 0x04) buttons |= DS2ButtonLeft;
        if (in[5] & 0x08) buttons |= DS2ButtonRight;
        if (in[5] & 0x10) buttons |= DS2ButtonL1;
        if (in[5] & 0x20) buttons |= DS2ButtonR1;
        if (in[5] & 0x40) buttons |= DS2ButtonL3;
        if (in[5] & 0x80) buttons |= DS2ButtonR3;

        if (in[4] & 0x04) buttons |= DS2ButtonStart;
        if (in[4] & 0x08) buttons |= DS2ButtonSelect;
        if (in[4] & 0x10) buttons |= DS2ButtonCross;
        if (in[4] & 0x20) buttons |= DS2ButtonCircle;
        if (in[4] & 0x40) buttons |= DS2ButtonSquare;
        if (in[4] & 0x80) buttons |= DS2ButtonTriangle;

        if (lt > 0) buttons |= DS2ButtonL2;
        if (rt > 0) buttons |= DS2ButtonR2;

        out->nButtonState = ~buttons;
        out->LeftStickX = xboxusb_axis_to_ds2_centered(pad, 0, in[10], in[11], 0);
        out->LeftStickY = xboxusb_axis_to_ds2_centered(pad, 1, in[12], in[13], 1);
        out->RightStickX = xboxusb_axis_to_ds2_centered(pad, 2, in[14], in[15], 0);
        out->RightStickY = xboxusb_axis_to_ds2_centered(pad, 3, in[16], in[17], 1);

        out->PressureL2 = lt > 0x3FF ? 0xFF : (lt >> 2);
        out->PressureR2 = rt > 0x3FF ? 0xFF : (rt >> 2);
    } else if (in[0] == 0x01) {
        if (in[1] == 0x03) return;

        lx = in[1];
        ly = in[2];
        rx = in[3];
        ry = in[4];

        u8 hat = in[5] & 0x0F;
        u8 b1_high = in[5] >> 4;
        u8 b1 = in[6];
        u8 b2 = in[7];
        u8 b3 = in[8];

        if (hat <= 7) {
            switch (hat) {
                case 0: buttons |= DS2ButtonUp; break;
                case 1: buttons |= DS2ButtonUp | DS2ButtonRight; break;
                case 2: buttons |= DS2ButtonRight; break;
                case 3: buttons |= DS2ButtonDown | DS2ButtonRight; break;
                case 4: buttons |= DS2ButtonDown; break;
                case 5: buttons |= DS2ButtonDown | DS2ButtonLeft; break;
                case 6: buttons |= DS2ButtonLeft; break;
                case 7: buttons |= DS2ButtonUp | DS2ButtonLeft; break;
            }
        }

        if ((b1 & 0x01) || (b1_high & 0x01)) buttons |= DS2ButtonCross;
        if ((b1 & 0x02) || (b1_high & 0x02)) buttons |= DS2ButtonCircle;
        if ((b1 & 0x04) || (b1_high & 0x04)) buttons |= DS2ButtonSquare;
        if ((b1 & 0x08) || (b1_high & 0x08)) buttons |= DS2ButtonTriangle;

        if ((b1 & 0x10) || (b2 & 0x01)) buttons |= DS2ButtonL1;
        if ((b1 & 0x20) || (b2 & 0x02)) buttons |= DS2ButtonR1;
        if ((b1 & 0x40) || (b2 & 0x04)) { buttons |= DS2ButtonL2; lt = 255; }
        if ((b1 & 0x80) || (b2 & 0x08)) { buttons |= DS2ButtonR2; rt = 255; }

        if ((b2 & 0x10) || (b2 & 0x01) || (b3 & 0x01)) buttons |= DS2ButtonSelect;
        if ((b2 & 0x20) || (b2 & 0x02) || (b3 & 0x02)) buttons |= DS2ButtonStart;
        if ((b2 & 0x40) || (b2 & 0x04)) buttons |= DS2ButtonL3;
        if ((b2 & 0x80) || (b2 & 0x08)) buttons |= DS2ButtonR3;

        out->nButtonState = ~buttons;
        out->LeftStickX = lx;
        out->LeftStickY = ly;
        out->RightStickX = rx;
        out->RightStickY = ry;
        out->PressureL2 = lt;
        out->PressureR2 = rt;
    } else { // Raw DInput HID without Report ID
        lx = in[0];
        ly = in[1];
        rx = in[2];
        ry = in[3];

        u8 hat = in[4] & 0x0F;
        if (in[4] == 0x00 && (in[6] & 0x0F) <= 15) {
            hat = in[6] & 0x0F;
        }

        u8 b1 = in[5];
        u8 b2 = in[6];
        u8 b3 = in[7];

        if (hat <= 7) {
            switch (hat) {
                case 0: buttons |= DS2ButtonUp; break;
                case 1: buttons |= DS2ButtonUp | DS2ButtonRight; break;
                case 2: buttons |= DS2ButtonRight; break;
                case 3: buttons |= DS2ButtonDown | DS2ButtonRight; break;
                case 4: buttons |= DS2ButtonDown; break;
                case 5: buttons |= DS2ButtonDown | DS2ButtonLeft; break;
                case 6: buttons |= DS2ButtonLeft; break;
                case 7: buttons |= DS2ButtonUp | DS2ButtonLeft; break;
            }
        }

        if (b1 & 0x01) buttons |= DS2ButtonCross;
        if (b1 & 0x02) buttons |= DS2ButtonCircle;
        if (b1 & 0x04) buttons |= DS2ButtonSquare;
        if (b1 & 0x08) buttons |= DS2ButtonTriangle;
        if (b1 & 0x10) buttons |= DS2ButtonL1;
        if (b1 & 0x20) buttons |= DS2ButtonR1;
        if (b1 & 0x40) { buttons |= DS2ButtonL2; lt = 255; }
        if (b1 & 0x80) { buttons |= DS2ButtonR2; rt = 255; }

        if (b2 & 0x01 || b3 & 0x01) buttons |= DS2ButtonSelect;
        if (b2 & 0x02 || b3 & 0x02) buttons |= DS2ButtonStart;
        if (b2 & 0x04 || b3 & 0x04) buttons |= DS2ButtonL3;
        if (b2 & 0x08 || b3 & 0x08) buttons |= DS2ButtonR3;

        out->nButtonState = ~buttons;
        out->LeftStickX = lx;
        out->LeftStickY = ly;
        out->RightStickX = rx;
        out->RightStickY = ry;
        out->PressureL2 = lt;
        out->PressureR2 = rt;
    }

    out->PressureUp = (buttons & DS2ButtonUp) ? 0xFF : 0x00;
    out->PressureDown = (buttons & DS2ButtonDown) ? 0xFF : 0x00;
    out->PressureLeft = (buttons & DS2ButtonLeft) ? 0xFF : 0x00;
    out->PressureRight = (buttons & DS2ButtonRight) ? 0xFF : 0x00;
    out->PressureTriangle = (buttons & DS2ButtonTriangle) ? 0xFF : 0x00;
    out->PressureCircle = (buttons & DS2ButtonCircle) ? 0xFF : 0x00;
    out->PressureCross = (buttons & DS2ButtonCross) ? 0xFF : 0x00;
    out->PressureSquare = (buttons & DS2ButtonSquare) ? 0xFF : 0x00;
    out->PressureL1 = (buttons & DS2ButtonL1) ? 0xFF : 0x00;
    out->PressureR1 = (buttons & DS2ButtonR1) ? 0xFF : 0x00;
}

static int xboxusb_send_packet(int pad, const u8 *data, int len)
{
    int ret;

    if (ds34pad[pad].outEndp < 0 || len <= 0 || len > MAX_BUFFER_SIZE)
        return 0;

    PollSema(ds34pad[pad].cmd_sema);
    mips_memset(usb_out_buf[pad], 0, sizeof(usb_out_buf[pad]));
    mips_memcpy(usb_out_buf[pad], data, len);
    usb_out_buf[pad][2] = ds34pad[pad].xbox_seq++;

    ret = UsbInterruptTransfer(ds34pad[pad].outEndp, usb_out_buf[pad], len, usb_cmd_cb, (void *)pad);
    if (ret == USB_RC_OK)
        TransferWait(ds34pad[pad].cmd_sema);

    return ret == USB_RC_OK;
}

static int xboxusb_send_packet_raw(int pad, const u8 *data, int len)
{
    int ret;

    if (ds34pad[pad].outEndp < 0 || len <= 0 || len > MAX_BUFFER_SIZE)
        return 0;

    PollSema(ds34pad[pad].cmd_sema);
    mips_memset(usb_out_buf[pad], 0, sizeof(usb_out_buf[pad]));
    mips_memcpy(usb_out_buf[pad], data, len);

    ret = UsbInterruptTransfer(ds34pad[pad].outEndp, usb_out_buf[pad], len, usb_cmd_cb, (void *)pad);
    if (ret == USB_RC_OK)
        TransferWait(ds34pad[pad].cmd_sema);

    return ret == USB_RC_OK;
}

static int xboxusb_send_init(int pad)
{
    if (ds34pad[pad].status & DS34USB_STATE_INIT_SENT)
        return 1;

    if (is_xbox_vendor(ds34pad[pad].vid, ds34pad[pad].pid)) {
        mips_memset(usb_ctrl_buf[pad], 0, sizeof(usb_ctrl_buf[pad]));
        UsbControlTransfer(ds34pad[pad].controlEndp, 0xC1, 0x01, 0x0100, 0x00, 20, usb_ctrl_buf[pad], NULL, NULL);

        xboxusb_send_packet_raw(pad, xbox360_led_p1, sizeof(xbox360_led_p1));
        xboxusb_send_packet_raw(pad, xbox360_init_cmd, sizeof(xbox360_init_cmd));
    }

    ds34pad[pad].status |= DS34USB_STATE_INIT_SENT;
    return 1;
}

static int LEDRumble(u8 *led, u8 lrum, u8 rrum, int pad)
{
    int ret = 0;

    PollSema(ds34pad[pad].cmd_sema);

    mips_memset(usb_out_buf[pad], 0, sizeof(usb_out_buf[pad]));

    if (ds34pad[pad].type == DS3) {
        mips_memcpy(usb_out_buf[pad], output_01_report, sizeof(output_01_report));

        usb_out_buf[pad][1] = 0xFE; // rt
        usb_out_buf[pad][2] = rrum; // rp
        usb_out_buf[pad][3] = 0xFE; // lt
        usb_out_buf[pad][4] = lrum; // lp

        usb_out_buf[pad][9] = led[0] & 0x7F; // LED Conf

        if (led[3]) // means charging, so blink
        {
            usb_out_buf[pad][13] = 0x32;
            usb_out_buf[pad][18] = 0x32;
            usb_out_buf[pad][23] = 0x32;
            usb_out_buf[pad][28] = 0x32;
        }

        ret = UsbControlTransfer(ds34pad[pad].controlEndp, REQ_USB_OUT, USB_REQ_SET_REPORT, (HID_USB_SET_REPORT_OUTPUT << 8) | 0x01, 0, sizeof(output_01_report), usb_out_buf[pad], usb_cmd_cb, (void *)pad);
    } else if (ds34pad[pad].type == DS4) {
        usb_out_buf[pad][0] = 0x05;
        usb_out_buf[pad][1] = 0xFF;

        usb_out_buf[pad][4] = rrum * 255; // ds4 has full control
        usb_out_buf[pad][5] = lrum;

        usb_out_buf[pad][6] = led[0]; // r
        usb_out_buf[pad][7] = led[1]; // g
        usb_out_buf[pad][8] = led[2]; // b

        if (led[3]) // means charging, so blink
        {
            usb_out_buf[pad][9] = 0x80;  // Time to flash bright (255 = 2.5 seconds)
            usb_out_buf[pad][10] = 0x80; // Time to flash dark (255 = 2.5 seconds)
        }

        ret = UsbInterruptTransfer(ds34pad[pad].outEndp, usb_out_buf[pad], 32, usb_cmd_cb, (void *)pad);
    }

    ds34pad[pad].oldled[0] = led[0];
    ds34pad[pad].oldled[1] = led[1];
    ds34pad[pad].oldled[2] = led[2];
    ds34pad[pad].oldled[3] = led[3];

    return ret;
}

static unsigned int timeout(void *arg)
{
    int sema = (int)arg;
    iSignalSema(sema);
    return 0;
}

static void TransferWait(int sema)
{
    TransferWaitTimeout(sema, 200000);
}

static void TransferWaitTimeout(int sema, u32 timeout_lo)
{
    iop_sys_clock_t cmd_timeout;

    cmd_timeout.lo = timeout_lo;
    cmd_timeout.hi = 0;

    if (SetAlarm(&cmd_timeout, timeout, (void *)sema) == 0) {
        WaitSema(sema);
        CancelAlarm(timeout, NULL);
    }
}

void ds34usb_set_rumble(u8 lrum, u8 rrum, int port)
{
    WaitSema(ds34pad[port].sema);

    ds34pad[port].update_rum = 1;
    ds34pad[port].lrum = lrum;
    ds34pad[port].rrum = rrum;

    SignalSema(ds34pad[port].sema);
}

int ds34usb_get_data(u8 *dst, int size, int port)
{
    int ret = 0;

    WaitSema(ds34pad[port].sema);

    PollSema(ds34pad[port].sema);

    // If a transfer already completed in background, process it immediately
    if (!usb_resulCode[port]) {
        readReport(usb_in_buf[port], port);
        usb_resulCode[port] = 1;
    }

    ret = UsbInterruptTransfer(ds34pad[port].interruptEndp, usb_in_buf[port], MAX_BUFFER_SIZE, usb_data_cb, (void *)port);

    if (ret == USB_RC_OK) {
        if (ds34pad[port].type == XBOX_USB)
            TransferWaitTimeout(ds34pad[port].sema, 8000);
        else
            TransferWait(ds34pad[port].sema);

        if (!usb_resulCode[port])
            readReport(usb_in_buf[port], port);

        usb_resulCode[port] = 1;
    } else {
        DPRINTF("DS34USB: ds34usb_get_data usb transfer error %d\n", ret);
    }

    mips_memcpy(dst, ds34pad[port].data, size);
    ret = ds34pad[port].analog_btn & 1;

    if (ds34pad[port].update_rum) {
        if (ds34pad[port].type != XBOX_USB) {
            ret = LEDRumble(ds34pad[port].oldled, ds34pad[port].lrum, ds34pad[port].rrum, port);
            if (ret == USB_RC_OK)
                TransferWait(ds34pad[port].cmd_sema);
            else
                DPRINTF("DS34USB: LEDRumble usb transfer error %d\n", ret);
        }
        ds34pad[port].update_rum = 0;
    }

    SignalSema(ds34pad[port].sema);

    return ret;
}

void ds34usb_set_mode(int mode, int lock, int port)
{
    if (lock == 3)
        ds34pad[port].analog_btn = 3;
    else
        ds34pad[port].analog_btn = mode;
}

void ds34usb_reset()
{
    int pad;

    for (pad = 0; pad < MAX_PADS; pad++)
        usb_release(pad);
}

int ds34usb_get_status(int port)
{
    int ret;

    WaitSema(ds34pad[port].sema);
    if (ds34pad[port].type == XBOX_USB && ds34pad[port].devId >= 0)
        ret = DS34USB_STATE_RUNNING | DS34USB_STATE_CONNECTED | DS34USB_STATE_CONFIGURED | DS34USB_STATE_AUTHORIZED;
    else
        ret = ds34pad[port].status;
    SignalSema(ds34pad[port].sema);

    return ret;
}

int ds34usb_get_model(int port)
{
    int ret;

    WaitSema(ds34pad[port].sema);
    if (ds34pad[port].type == GUITAR_GH || ds34pad[port].type == GUITAR_RB) {
        ret = MODEL_GUITAR;
    } else {
        ret = MODEL_PS2;
    }
    SignalSema(ds34pad[port].sema);

    return ret;
}

int ds34usb_init(u8 pads, u8 options)
{
    int pad;

    for (pad = 0; pad < MAX_PADS; pad++) {
        ds34pad[pad].status = 0;
        ds34pad[pad].devId = -1;
        ds34pad[pad].oldled[0] = 0;
        ds34pad[pad].oldled[1] = 0;
        ds34pad[pad].oldled[2] = 0;
        ds34pad[pad].oldled[3] = 0;
        ds34pad[pad].lrum = 0;
        ds34pad[pad].rrum = 0;
        ds34pad[pad].update_rum = 1;
        ds34pad[pad].sema = -1;
        ds34pad[pad].cmd_sema = -1;
        ds34pad[pad].controlEndp = -1;
        ds34pad[pad].interruptEndp = -1;
        ds34pad[pad].outEndp = -1;
        ds34pad[pad].vid = 0;
        ds34pad[pad].pid = 0;
        ds34pad[pad].xbox_seq = 0;
        ds34pad[pad].enabled = (pads >> pad) & 1;
        ds34pad[pad].type = 0;

        ds34pad[pad].data[0] = 0xFF;
        ds34pad[pad].data[1] = 0xFF;
        ds34pad[pad].analog_btn = 0;

        mips_memset(&ds34pad[pad].data[2], 0x7F, 4);
        mips_memset(&ds34pad[pad].data[6], 0x00, 12);

        ds34pad[pad].sema = CreateMutex(IOP_MUTEX_UNLOCKED);
        ds34pad[pad].cmd_sema = CreateMutex(IOP_MUTEX_UNLOCKED);

        if (ds34pad[pad].sema < 0 || ds34pad[pad].cmd_sema < 0) {
            DPRINTF("DS34USB: Failed to allocate I/O semaphore.\n");
            return 0;
        }
    }

    if (UsbRegisterDriver(&usb_driver) != USB_RC_OK) {
        DPRINTF("DS34USB: Error registering USB devices\n");
        return 0;
    }

    return 1;
}
