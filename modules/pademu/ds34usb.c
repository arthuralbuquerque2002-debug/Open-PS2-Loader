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

#define XINPUT_VID 0x045E
#define XINPUT_PID 0x028E
#define XINPUT_TYPE 99

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

static u8 rgbled_patterns[][2][3] =
    {
        {{0x00, 0x00, 0x10}, {0x00, 0x00, 0x7F}}, // light blue/blue
        {{0x00, 0x10, 0x00}, {0x00, 0x7F, 0x00}}, // light green/green
        {{0x10, 0x10, 0x00}, {0x7F, 0x7F, 0x00}}, // light yellow/yellow
        {{0x00, 0x10, 0x10}, {0x00, 0x7F, 0x7F}}, // light cyan/cyan
};

static u8 usb_buf[MAX_BUFFER_SIZE + 32] __attribute((aligned(4))) = {0};

int usb_probe(int devId);
int usb_connect(int devId);
int usb_disconnect(int devId);

static void usb_release(int pad);
static void usb_config_set(int result, int count, void *arg);

UsbDriver usb_driver = {NULL, NULL, "ds34usb", usb_probe, usb_connect, usb_disconnect};

static void DS3USB_init(int pad);
static void readReport(u8 *data, int pad);
static int LEDRumble(u8 *led, u8 lrum, u8 rrum, int pad);

ds34usb_device ds34pad[MAX_PADS];

int usb_probe(int devId)
{
    UsbDeviceDescriptor *device = NULL;

    DPRINTF("DS34USB: probe: devId=%i\n", devId);

    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    if (device == NULL) {
        DPRINTF("DS34USB: Error - Couldn't get device descriptor\n");
        return 0;
    }

    if (device->idVendor == SONY_VID && (device->idProduct == GUITAR_HERO_PS3_PID || device->idProduct == ROCK_BAND_PS3_PID)) {
        return 1;
    }

    if (device->idVendor == DS34_VID && (device->idProduct == DS3_PID || device->idProduct == DS4_PID || device->idProduct == DS4_PID_SLIM))
        return 1;

    if (device->idVendor == XINPUT_VID && device->idProduct == XINPUT_PID)
        return 1;

    return 0;
}

int usb_connect(int devId)
{
    int pad, epCount;
    UsbDeviceDescriptor *device;
    UsbConfigDescriptor *config;
    UsbInterfaceDescriptor *interface;
    UsbEndpointDescriptor *endpoint;

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

    ds34pad[pad].devId = devId;

    ds34pad[pad].status = DS34USB_STATE_AUTHORIZED;

    ds34pad[pad].controlEndp = UsbOpenEndpoint(devId, NULL);

    device = (UsbDeviceDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_DEVICE);
    config = (UsbConfigDescriptor *)UsbGetDeviceStaticDescriptor(devId, device, USB_DT_CONFIG);
    interface = (UsbInterfaceDescriptor *)((char *)config + config->bLength);

    if (device->idVendor == XINPUT_VID && device->idProduct == XINPUT_PID) {
        ds34pad[pad].type = XINPUT_TYPE;
        epCount = 12;
    } else if (device->idProduct == DS3_PID) {
        ds34pad[pad].type = DS3;
        epCount = interface->bNumEndpoints - 1;
    } else if (device->idProduct == GUITAR_HERO_PS3_PID) {
        ds34pad[pad].type = GUITAR_GH;
        epCount = interface->bNumEndpoints - 1;
    } else if (device->idProduct == ROCK_BAND_PS3_PID) {
        ds34pad[pad].type = GUITAR_RB;
        epCount = interface->bNumEndpoints - 1;
    } else {
        ds34pad[pad].type = DS4;
        epCount = 20; // ds4 v2 returns interface->bNumEndpoints as 0
    }

    endpoint = (UsbEndpointDescriptor *)UsbGetDeviceStaticDescriptor(devId, NULL, USB_DT_ENDPOINT);

    while (endpoint != NULL && epCount-- >= 0) {
        if (endpoint->bmAttributes == USB_ENDPOINT_XFER_INT) {
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN && ds34pad[pad].interruptEndp < 0) {
                ds34pad[pad].interruptEndp = UsbOpenEndpointAligned(devId, endpoint);
                DPRINTF("DS34USB: register Event endpoint id =%i addr=%02X\n", ds34pad[pad].interruptEndp, endpoint->bEndpointAddress);
            }
            if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT && ds34pad[pad].outEndp < 0) {
                ds34pad[pad].outEndp = UsbOpenEndpointAligned(devId, endpoint);
                DPRINTF("DS34USB: register Output endpoint id =%i addr=%02X\n", ds34pad[pad].outEndp, endpoint->bEndpointAddress);
            }
        }

        if (ds34pad[pad].interruptEndp >= 0 && ds34pad[pad].outEndp >= 0)
            break;

        endpoint = (UsbEndpointDescriptor *)UsbGetDeviceStaticDescriptor(devId, endpoint, USB_DT_ENDPOINT);
    }

    if (ds34pad[pad].interruptEndp < 0 || ds34pad[pad].outEndp < 0) {
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
    ds34pad[pad].status = DS34USB_STATE_DISCONNECTED;

    SignalSema(ds34pad[pad].sema);
}

static int usb_resulCode;

static void usb_data_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;
    usb_resulCode = resultCode;
    SignalSema(ds34pad[pad].sema);
}

static void usb_cmd_cb(int resultCode, int bytes, void *arg)
{
    int pad = (int)arg;
    SignalSema(ds34pad[pad].cmd_sema);
}

static void usb_config_set(int result, int count, void *arg)
{
    int pad = (int)arg;
    u8 led[4];

    PollSema(ds34pad[pad].sema);

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
    } else if (ds34pad[pad].type == XINPUT_TYPE) {
        led[0] = (u8)(pad + 0x06); // LED player 1..4 on Xbox 360 ring
        led[1] = 0;
        led[2] = 0;
        led[3] = 0;
    }

    LEDRumble(led, 0, 0, pad);

    ds34pad[pad].status |= DS34USB_STATE_RUNNING;

    SignalSema(ds34pad[pad].sema);
}

static void DS3USB_init(int pad)
{
    usb_buf[0] = 0x42;
    usb_buf[1] = 0x0c;
    usb_buf[2] = 0x00;
    usb_buf[3] = 0x00;

    UsbControlTransfer(ds34pad[pad].controlEndp, REQ_USB_OUT, USB_REQ_SET_REPORT, (HID_USB_GET_REPORT_FEATURE << 8) | 0xF4, 0, 4, usb_buf, NULL, NULL);
}

static void translate_pad_xinput(const u8 *data, struct ds2report *ds2)
{
    u8 b0 = data[2]; // D-Pad, Start, Back, L3, R3
    u8 b1 = data[3]; // LB, RB, Guide, A, B, X, Y
    u8 lt = data[4];
    u8 rt = data[5];

    // Mapeamento PS2 (0 = pressionado, 1 = solto)
    u8 ps2_l = 0xFF;
    u8 ps2_h = 0xFF;

    if (b0 & 0x20) ps2_l &= ~(1 << 0); // Select (Back)
    if (b0 & 0x40) ps2_l &= ~(1 << 1); // L3
    if (b0 & 0x80) ps2_l &= ~(1 << 2); // R3
    if (b0 & 0x10) ps2_l &= ~(1 << 3); // Start
    if (b0 & 0x01) ps2_l &= ~(1 << 4); // Up
    if (b0 & 0x08) ps2_l &= ~(1 << 5); // Right
    if (b0 & 0x02) ps2_l &= ~(1 << 6); // Down
    if (b0 & 0x04) ps2_l &= ~(1 << 7); // Left

    if (lt > 0x40) ps2_h &= ~(1 << 0); // L2
    if (rt > 0x40) ps2_h &= ~(1 << 1); // R2
    if (b1 & 0x01) ps2_h &= ~(1 << 2); // L1 (LB)
    if (b1 & 0x02) ps2_h &= ~(1 << 3); // R1 (RB)
    if (b1 & 0x80) ps2_h &= ~(1 << 4); // Triangle (Y)
    if (b1 & 0x20) ps2_h &= ~(1 << 5); // Circle (B)
    if (b1 & 0x10) ps2_h &= ~(1 << 6); // Cross (A)
    if (b1 & 0x40) ps2_h &= ~(1 << 7); // Square (X)

    ds2->nButtonStateL = ps2_l;
    ds2->nButtonStateH = ps2_h;

    // Eixos XInput (signed 16-bit LE) -> PS2 (0..255, centro 128, Y invertido)
    s16 lx = (s16)(data[6] | (data[7] << 8));
    s16 ly = (s16)(data[8] | (data[9] << 8));
    s16 rx = (s16)(data[10] | (data[11] << 8));
    s16 ry = (s16)(data[12] | (data[13] << 8));

    ds2->LeftStickX  = (u8)((lx >> 8) + 128);
    ds2->LeftStickY  = (u8)(255 - ((ly >> 8) + 128));
    ds2->RightStickX = (u8)((rx >> 8) + 128);
    ds2->RightStickY = (u8)(255 - ((ry >> 8) + 128));

    // Pressão analógica dos botões
    ds2->PressureRight    = (b0 & 0x08) ? 0xFF : 0x00;
    ds2->PressureLeft     = (b0 & 0x04) ? 0xFF : 0x00;
    ds2->PressureUp       = (b0 & 0x01) ? 0xFF : 0x00;
    ds2->PressureDown     = (b0 & 0x02) ? 0xFF : 0x00;
    ds2->PressureTriangle = (b1 & 0x80) ? 0xFF : 0x00;
    ds2->PressureCircle   = (b1 & 0x20) ? 0xFF : 0x00;
    ds2->PressureCross    = (b1 & 0x10) ? 0xFF : 0x00;
    ds2->PressureSquare   = (b1 & 0x40) ? 0xFF : 0x00;
    ds2->PressureL1       = (b1 & 0x01) ? 0xFF : 0x00;
    ds2->PressureR1       = (b1 & 0x02) ? 0xFF : 0x00;
    ds2->PressureL2       = lt;
    ds2->PressureR2       = rt;
}

#define MAX_DELAY 10

static void readReport(u8 *data, int pad_idx)
{
    ds34usb_device *pad = &ds34pad[pad_idx];

    if (pad->type == XINPUT_TYPE) {
        if (data[0] == 0x00 && data[1] == 0x14) {
            u8 guide = (data[3] & 0x04) ? 1 : 0;
            translate_pad_xinput(data, &pad->ds2);
            padMacroPerform(&pad->ds2, guide);

            if (guide && (data[2] & 0x20) && (pad->btn_delay == MAX_DELAY)) { // Guide + Back alterna analógico
                if (pad->analog_btn < 2)
                    pad->analog_btn = !pad->analog_btn;
                pad->btn_delay = 1;
            } else if (guide) {
                if (pad->btn_delay < MAX_DELAY)
                    pad->btn_delay++;
            } else if (pad->btn_delay > 0) {
                pad->btn_delay--;
            }
        }
        return;
    }

    if (pad->type == GUITAR_GH || pad->type == GUITAR_RB) {
        struct ds3guitarreport *report;

        report = (struct ds3guitarreport *)data;

        translate_pad_guitar(report, &pad->ds2, pad->type == GUITAR_GH);
        padMacroPerform(&pad->ds2, report->PSButton);
    }
    if (data[0]) {

        if (pad->type == DS3) {
            struct ds3report *report;

            report = (struct ds3report *)&data[2];

            if (report->RightStickX == 0 && report->RightStickY == 0)
                return;

            pad->data[0] = ~report->ButtonStateL;
            pad->data[1] = ~report->ButtonStateH;

            translate_pad_ds3(report, &pad->ds2, 0);
            padMacroPerform(&pad->ds2, report->PSButton);
            if (report->PSButton) {
                if (report->Select && (pad->btn_delay == MAX_DELAY)) {
                    if (pad->analog_btn < 2)
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

            if (report->Power == 0xEE)
                pad->oldled[3] = 1;
            else
                pad->oldled[3] = 0;

        } else if (pad->type == DS4) {
            struct ds4report *report;
            report = (struct ds4report *)data;
            translate_pad_ds4(report, &pad->ds2, 1);
            padMacroPerform(&pad->ds2, report->PSButton);

            if (report->PSButton) {
                if (report->Share && (pad->btn_delay == MAX_DELAY)) {
                    if (pad->analog_btn < 2)
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

            if (report->Power != 0xB && report->Usb_plugged)
                pad->oldled[3] = 1;
            else
                pad->oldled[3] = 0;
        }
        if (pad->btn_delay > 0) {
            pad->update_rum = 1;
        }
    }
}

static int LEDRumble(u8 *led, u8 lrum, u8 rrum, int pad)
{
    int ret = 0;

    PollSema(ds34pad[pad].cmd_sema);

    mips_memset(usb_buf, 0, sizeof(usb_buf));

    if (ds34pad[pad].type == XINPUT_TYPE) {
        usb_buf[0] = 0x00;
        usb_buf[1] = 0x08;
        usb_buf[2] = 0x00;
        usb_buf[3] = lrum;
        usb_buf[4] = rrum;
        ret = UsbInterruptTransfer(ds34pad[pad].outEndp, usb_buf, 8, usb_cmd_cb, (void *)pad);
    } else if (ds34pad[pad].type == DS3) {
        mips_memcpy(usb_buf, output_01_report, sizeof(output_01_report));

        usb_buf[1] = 0xFE; // rt
        usb_buf[2] = rrum; // rp
        usb_buf[3] = 0xFE; // lt
        usb_buf[4] = lrum; // lp

        usb_buf[9] = led[0] & 0x7F; // LED Conf

        if (led[3]) {
            usb_buf[13] = 0x32;
            usb_buf[18] = 0x32;
            usb_buf[23] = 0x32;
            usb_buf[28] = 0x32;
        }

        ret = UsbControlTransfer(ds34pad[pad].controlEndp, REQ_USB_OUT, USB_REQ_SET_REPORT, (HID_USB_SET_REPORT_OUTPUT << 8) | 0x01, 0, sizeof(output_01_report), usb_buf, usb_cmd_cb, (void *)pad);
    } else if (ds34pad[pad].type == DS4) {
        usb_buf[0] = 0x05;
        usb_buf[1] = 0xFF;

        usb_buf[4] = rrum * 255;
        usb_buf[5] = lrum;

        usb_buf[6] = led[0];
        usb_buf[7] = led[1];
        usb_buf[8] = led[2];

        if (led[3]) {
            usb_buf[9] = 0x80;
            usb_buf[10] = 0x80;
        }

        ret = UsbInterruptTransfer(ds34pad[pad].outEndp, usb_buf, 32, usb_cmd_cb, (void *)pad);
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
    iop_sys_clock_t cmd_timeout;

    cmd_timeout.lo = 200000;
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

    ret = UsbInterruptTransfer(ds34pad[port].interruptEndp, usb_buf, MAX_BUFFER_SIZE, usb_data_cb, (void *)port);

    if (ret == USB_RC_OK) {
        TransferWait(ds34pad[port].sema);
        if (!usb_resulCode)
            readReport(usb_buf, port);

        usb_resulCode = 1;
    } else {
        DPRINTF("DS34USB: ds34usb_get_data usb transfer error %d\n", ret);
    }

    mips_memcpy(dst, ds34pad[port].data, size);
    ret = ds34pad[port].analog_btn & 1;

    if (ds34pad[port].update_rum) {
        ret = LEDRumble(ds34pad[port].oldled, ds34pad[port].lrum, ds34pad[port].rrum, port);
        if (ret == USB_RC_OK)
            TransferWait(ds34pad[port].cmd_sema);
        else
            DPRINTF("DS34USB: LEDRumble usb transfer error %d\n", ret);

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
