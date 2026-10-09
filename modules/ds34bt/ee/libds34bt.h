
enum eDS34BTStatus {
    DS34BT_STATE_USB_DISCONNECTED = 0x00,
    DS34BT_STATE_USB_AUTHORIZED = 0x01,
    DS34BT_STATE_USB_CONFIGURED = 0x02,
    DS34BT_STATE_CONNECTED = 0x04,
    DS34BT_STATE_RUNNING = 0x08,
};

typedef struct
{
    u8 hci_ver;     // Version of the Current HCI in the BR/EDR Controller
    u16 hci_rev;    // Revision of the Current HCI in the BR/EDR Controller
    u8 lmp_ver;     // Version of the Current LMP or PAL in the Controller
    u16 mf_name;    // Manufacturer Name of the BR/EDR Controller
    u16 lmp_subver; // Subversion of the Current LMP or PAL in the Controller
    u16 vid;
    u16 pid;
    u16 rev;
} __attribute__((packed)) hci_information_t;

#define DS34BT_SCAN_MAX 8
#define DS34BT_NAME_MAX 32

enum eDS34BTScanState {
    DS34BT_SCAN_IDLE = 0,
    DS34BT_SCAN_RUNNING = 1,
    DS34BT_SCAN_COMPLETE = 2,
};

typedef struct
{
    u8 bdaddr[6];
    u8 class_of_device[3];
    u8 status;
    char name[DS34BT_NAME_MAX];
} ds34bt_scan_result_t;

int ds34bt_init();
int ds34bt_deinit();
int ds34bt_reinit_ports(u8 ports);
int ds34bt_init_charging();
int ds34bt_get_status(int port);
int ds34bt_get_bdaddr(u8 *bdaddr);
int ds34bt_set_rumble(int port, u8 lrum, u8 rrum);
int ds34bt_set_led(int port, u8 *led); // rgb for ds4
int ds34bt_get_data(int port, u8 *data);
int ds34bt_reset();
int ds34bt_get_version(hci_information_t *info);
int ds34bt_get_features(u8 *info);
int ds34bt_scan_start();
int ds34bt_scan_get_status(int *count);
int ds34bt_scan_get(int index, ds34bt_scan_result_t *result);
int ds34bt_pair(const u8 *bdaddr);
