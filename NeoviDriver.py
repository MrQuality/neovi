"""
Driver for NeoVI and alike devices.

this device is intended to be used by any testing framework.
The implementation of the event-loop should be handled by the calling module.

this driver expose only one object which is used to interface all internal implementation ( NeoviDriver )

"""
import sys
import os
import datetime
import time

import base36
from ctypes import *

try:
    neovi_dll = WinDLL('icsneo40.dll')
except TypeError as te:
    raise ImportError("unable to find the called dll") from te

# handler to the current used deviceHandler
m_hObject = c_int(0)

#  This is an array of device types to look for. Specifies the types of neoVI devices to find.
#  Each element in the array need to have a value for the device type to look for
device_types = {
    "NEODEVICE_UNKNOWN": 0x00000000,
    "NEODEVICE_BLUE": 0x00000001,
    "NEODEVICE_ECU_AVB": 0x00000002,
    "NEODEVICE_RADSUPERMOON": 0x00000003,
    "NEODEVICE_DW_VCAN": 0x00000004,
    "NEODEVICE_RADMOON2": 0x00000005,
    "NEODEVICE_RADGIGALOG": 0x00000006,
    "NEODEVICE_VCAN41": 0x00000007,
    "NEODEVICE_FIRE": 0x00000008,
    "NEODEVICE_RADPLUTO": 0x00000009,
    "NEODEVICE_VCAN42_EL": 0x0000000a,
    "NEODEVICE_RADIO_CANHUB": 0x0000000b,
    "NEODEVICE_NEOECU12": 0x0000000c,
    "NEODEVICE_OBD2_LCBADGE": 0x0000000d,
    "NEODEVICE_RAD_MOON_DUO": 0x0000000e,
    "NEODEVICE_VCAN3": 0x00000010,
    "NEODEVICE_RADJUPITER": 0x00000011,
    "NEODEVICE_VCAN4_IND": 0x00000012,
    "NEODEVICE_GIGASTAR": 0x00000013,
    "NEODEVICE_ECU22": 0x00000015,
    "NEODEVICE_RED": 0x00000040,
    "NEODEVICE_ECU": 0x00000080,
    "NEODEVICE_IEVB": 0x00000100,
    "NEODEVICE_PENDANT": 0x00000200,
    "NEODEVICE_OBD2_PRO": 0x00000400,
    "NEODEVICE_PLASMA": 0x00001000,
    "NEODEVICE_NEOANALOG": 0x00004000,
    "NEODEVICE_CT_OBD": 0x00008000,
    "NEODEVICE_ION": 0x00040000,
    "NEODEVICE_RADSTAR": 0x00080000,
    "NEODEVICE_VCAN44": 0x00200000,
    "NEODEVICE_VCAN42": 0x00400000,
    "NEODEVICE_CMPROBE": 0x00800000,
    "NEODEVICE_EEVB": 0x01000000,
    "NEODEVICE_VCANRF": 0x02000000,
    "NEODEVICE_FIRE2": 0x04000000,
    "NEODEVICE_FLEX": 0x08000000,
    "NEODEVICE_RADGALAXY": 0x10000000,
    "NEODEVICE_RADSTAR2": 0x20000000,
    "NEODEVICE_VIVIDCAN": 0x40000000,
    "NEODEVICE_OBD2_SIM": 0x80000000,
    "NEODEVICE_ALL": 0xFFFFBFFF}

# There are two status bitfields in the message structures that define specific attributes of the message.
# The two status bitfields are named StatusBitfield, StatusBitfield2 and StatusBitfield3.
StatusBitfield = {
    "SPY_STATUS_GLOBAL_ERR": 0x01,
    "SPY_STATUS_TX_MSG": 0x02,
    "SPY_STATUS_XTD_FRAME": 0x04,
    "SPY_STATUS_REMOTE_FRAME": 0x08,
    "SPY_STATUS_CRC_ERROR": 0x10,
    "SPY_STATUS_CAN_ERROR_PASSIVE": 0x20,
    "SPY_STATUS_INCOMPLETE_FRAME": 0x40,
    "SPY_STATUS_LOST_ARBITRATION": 0x80,
    "SPY_STATUS_UNDEFINED_ERROR": 0x100,
    "SPY_STATUS_CAN_BUS_OFF": 0x200,
    "SPY_STATUS_CAN_ERROR_WARNING": 0x400,
    "SPY_STATUS_BUS_SHORTED_PLUS": 0x800,
    "SPY_STATUS_BUS_SHORTED_GND": 0x1000,
    "SPY_STATUS_CHECKSUM_ERROR": 0x2000,
    "SPY_STATUS_BAD_MESSAGE_BIT_TIME_ERROR": 0x4000,
    "SPY_STATUS_IFR_DATA": 0x8000,
    "SPY_STATUS_HARDWARE_COMM_ERROR": 0x10000,
    "SPY_STATUS_EXPECTED_LEN_ERROR": 0x20000,
    "SPY_STATUS_INCOMING_NO_MATCH": 0x40000,
    "SPY_STATUS_BREAK": 0x80000,
    "SPY_STATUS_AVSI_REC_OVERFLOW": 0x100000,
    "SPY_STATUS_TEST_TRIGGER": 0x200000,
    "SPY_STATUS_AUDIO_COMMENT": 0x400000,
    "SPY_STATUS_GPS_DATA": 0x800000,
    "SPY_STATUS_ANALOG_DIGITAL_INPUT": 0x1000000,
    "SPY_STATUS_TEXT_COMMENT": 0x2000000,
    "SPY_STATUS_NETWORK_MESSAGE_TYPE": 0x4000000,
    "SPY_STATUS_VSI_TX_UNDERRUN": 0x8000000,
    "SPY_STATUS_VSI_IFR_CRC_Bit": 0x10000000,
    "SPY_STATUS_INIT_MESSAGE": 0x20000000,
    "SPY_STATUS_HIGH_SPEED_MESSAGE": 0x40000000,
    "SPY_STATUS_FLEXRAY_SECOND_STARTUP_FRAME": 0x40000000,
    "SPY_STATUS_EXTENDED": 0x80000000
}

StatusBitfield2 = {
    "SPY_STATUS2_HAS_VALUE": 1,
    "SPY_STATUS2_VALUE_IS_BOOLEAN": 2,
    "SPY_STATUS2_HIGH_VOLTAGE": 4,
    "SPY_STATUS2_LONG_MESSAGE": 8,
    "SPY_STATUS2_GLOBAL_CHANGE": 0x10000,
    "SPY_STATUS2_ERROR_FRAME": 0x20000,
    "SPY_STATUS2_END_OF_LONG_MESSAGE": 0x100000,
    "SPY_STATUS2_LIN_ERR_RX_BREAK_NOT_0": 0x200000,
    "SPY_STATUS2_LIN_ERR_RX_BREAK_TOO_SHORT": 0x400000,
    "SPY_STATUS2_LIN_ERR_RX_SYNC_NOT_55": 0x800000,
    "SPY_STATUS2_LIN_ERR_RX_DATA_GREATER_8": 0x1000000,
    "SPY_STATUS2_LIN_ERR_TX_RX_MISMATCH": 0x2000000,
    "SPY_STATUS2_LIN_ERR_MSG_ID_PARITY": 0x4000000,
    "SPY_STATUS2_ISO_FRAME_ERROR": 0x8000000,
    "SPY_STATUS2_LIN_SYNC_FRAME_ERROR": 0x8000000,
    "SPY_STATUS2_ISO_OVERFLOW_ERROR": 0x10000000,
    "SPY_STATUS2_LIN_ID_FRAME_ERROR": 0x10000000,
    "SPY_STATUS2_ISO_PARITY_ERROR": 0x20000000,
    "SPY_STATUS2_LIN_SLAVE_BYTE_ERROR": 0x20000000,
    "SPY_STATUS2_RX_TIMEOUT_ERROR": 0x40000000,
    "SPY_STATUS2_LIN_NO_SLAVE_DATA": 0x80000000,
    "SPY_STATUS2_MOST_PACKET_DATA": 0x200000,
    "SPY_STATUS2_MOST_STATUS": 0x400000,
    "PY_STATUS2_MOST_LOW_LEVEL": 0x800000,
    "SPY_STATUS2_MOST_CONTROL_DATA": 0x1000000,
    "SPY_STATUS2_MOST_MHP_USER_DATA": 0x2000000,
    "SPY_STATUS2_MOST_MHP_CONTROL_DATA": 0x4000000,
    "SPY_STATUS2_MOST_I2S_DUMP": 0x8000000,
    "SPY_STATUS2_MOST_TOO_SHORT": 0x10000000,
    "SPY_STATUS2_MOST_MOST50": 0x20000000,
    "SPY_STATUS2_MOST_MOST150": 0x40000000,
    "SPY_STATUS2_MOST_CHANGED_PAR": 0x80000000,
    "SPY_STATUS2_ETHERNET_CRC_ERROR": 0x200000,
    "SPY_STATUS2_ETHERNET_FRAME_TOO_SHORT": 0x400000,
    "SPY_STATUS2_ETHERNET_FCS_AVAILABLE": 0x800000
}

StatusBitfield3 = {
    "SPY_STATUS3_LIN_JUST_BREAK_SYNC": 1,
    "SPY_STATUS3_LIN_SLAVE_DATA_TOO_SHORT": 2,
    "SPY_STATUS3_LIN_ONLY_UPDATE_SLAVE_TABLE_ONCE": 4
}

#  Network ID List for the neoVI API
network_id = {
    "NETID_DEVICE": 0,
    "NETID_HSCAN": 1,
    "NETID_MSCAN": 2,
    "NETID_SWCAN": 3,
    "NETID_LSFTCAN": 4,
    "NETID_ISO": 9,
    "NETID_ISO2": 14,
    "NETID_ISO14230": 15,
    "NETID_LIN": 16,
    "NETID_OP_ETHERNET1": 17,
    "NETID_OP_ETHERNET2": 18,
    "NETID_OP_ETHERNET3": 19,
    "NETID_ISO3": 41,
    "NETID_HSCAN2": 42,
    "NETID_HSCAN3": 44,
    "NETID_OP_ETHERNET4": 45,
    "NETID_OP_ETHERNET5": 46,
    "NETID_ISO4": 47,
    "NETID_LIN2": 48,
    "NETID_LIN3": 49,
    "NETID_LIN4": 50,
    "NETID_MOST": 51,
    "NETID_CGI": 53,
    "NETID_HSCAN4": 61,
    "NETID_HSCAN5": 62,
    "NETID_UART": 64,
    "NETID_UART2": 65,
    "NETID_UART3": 66,
    "NETID_UART4": 67,
    "NETID_SWCAN2": 68,
    "NETID_ETHERNET_DAQ": 69,
    "NETID_OP_ETHERNET6": 73,
    "NETID_OP_ETHERNET7": 75,
    "NETID_OP_ETHERNET8": 76,
    "NETID_OP_ETHERNET9": 77,
    "NETID_OP_ETHERNET10": 78,
    "NETID_OP_ETHERNET11": 79,
    "NETID_FLEXRAY1A": 80,
    "NETID_FLEXRAY1B": 81,
    "NETID_FLEXRAY2A": 82,
    "NETID_FLEXRAY2B": 83,
    "NETID_LIN5": 84,
    "NETID_FLEXRAY": 85,
    "NETID_FLEXRAY2": 86,
    "NETID_OP_ETHERNET12": 87,
    "NETID_MOST25": 90,
    "NETID_MOST50": 91,
    "NETID_MOST150": 92,
    "NETID_ETHERNET": 93,
    "NETID_HSCAN6": 96,
    "NETID_HSCAN7": 97,
    "NETID_LIN6": 98,
    "NETID_LSFTCAN2": 99
}

# idication what protocol is used.
protocol_val = {
    "SPY_PROTOCOL_CAN": 1,
    "SPY_PROTOCOL_J1850VPW": 3,
    "SPY_PROTOCOL_J1850PWM": 4,
    "SPY_PROTOCOL_ISO9141": 5,
    "SPY_PROTOCOL_Keyword2000": 6,
    "SPY_PROTOCOL_LIN": 12,
    "SPY_PROTOCOL_FLEXRAY": 16,
    "SPY_PROTOCOL_ETHERNET": 29,
    "SPY_PROTOCOL_CANFD": 30
}

# the physical location of the coreMini script to be executed on the neoVI device
script_location = {
    "SCRIPT_LOCATION_FLASH_MEM": 0,
    "SCRIPT_LOCATION_SDCARD": 1,
    "SCRIPT_LOCATION_VCAN3_MEM": 2
}

icscm_int16 = c_short
icscm_uint16 = c_ushort
icscm_uint32 = c_uint32
icscm_int32 = c_int32
icscm_uint8 = c_ubyte
icscm_int64 = c_int64


# Special Exceptions
class CanbusGetMessageError(Exception):
    def __init__(self, *args):
        if args:
            self.error = args[0]
            self.message_buffer = args[1]
        else:
            self.error = None
            self.message_buffer = []

    def __str__(self):
        if self.error:
            return 'CanbusGetMessageError, {0} '.format(self.error)
        else:
            return 'CanbusGetMessageError has been raised'


class icsSpyMessage(Structure):
    """
    This structure is used to represent messages both received and transmitted by the neoVI device.
    """
    _pack_ = 8
    _fields_ = [
        ("StatusBitField", c_uint32),
        ("StatusBitField2", c_uint32),
        ("TimeHardware", c_uint32),
        ("TimeHardware2", c_uint32),
        ("TimeSystem", c_uint32),
        ("TimeSystem2", c_uint32),
        ("TimeStampHardwareID", c_ubyte),
        ("TimeStampSystemID", c_ubyte),
        ("NetworkID", c_ubyte),
        ("NodeID", c_ubyte),
        ("Protocol", c_ubyte),
        ("MessagePieceID", c_ubyte),
        ("ExtraDataPtrEnabled", c_ubyte),
        ("NumberBytesHeader", c_ubyte),
        ("NumberBytesData", c_ubyte),
        ("DescriptionID", c_short),
        ("ArbIDOrHeader", c_int32),
        ("Data", c_ubyte * 8),  # Msg Data
        ("StatusBitField3", c_uint32),
        ("StatusBitField4", c_uint32),
        ("iExtraDataPtr", c_void_p),
        ("MiscData", c_ubyte)]


class icsSpyMessageRX(Structure):
    """
    This structure is used to represent messages received by the neoVI device.
    """
    _pack_ = 8
    _fields_ = [
        ("StatusBitField", c_uint32),
        ("StatusBitField2", c_uint32),
        ("TimeHardware", c_uint32),
        ("TimeHardware2", c_uint32),
        ("TimeSystem", c_uint32),
        ("TimeSystem2", c_uint32),
        ("TimeStampHardwareID", c_ubyte),
        ("TimeStampSystemID", c_ubyte),
        ("NetworkID", c_ubyte),
        ("NodeID", c_ubyte),
        ("Protocol", c_ubyte),
        ("MessagePieceID", c_ubyte),
        ("ExtraDataPtrEnabled", c_ubyte),
        ("NumberBytesHeader", c_ubyte),
        ("NumberBytesData", c_ubyte),
        ("DescriptionID", c_short),
        ("ArbIDOrHeader", c_int),
        ("Data", c_ubyte * 8),  # Msg Data
        ("StatusBitField3", c_uint32),
        ("StatusBitField4", c_uint32),
        ("iExtraDataPtr", POINTER(c_uint8)),
        ("MiscData", c_ubyte)]


class icsSpyMessageJ1850(Structure):
    """
        This structure is used to represent J1850 messages both received and transmitted by the neoVI device.
    """
    _pack_ = 8
    _fields_ = [("StatusBitField", c_ulong),
                ("StatusBitField2", c_ulong),
                ("TimeHardware", c_ulong),
                ("TimeHardware2", c_ulong),
                ("TimeSystem", c_ulong),
                ("TimeSystem2", c_ulong),
                ("TimeStampHardwareID", c_byte),
                ("TimeStampSystemID", c_byte),
                ("NetworkID", c_byte),
                ("NodeID", c_byte),
                ("Protocol", c_byte),
                ("MessagePieceID", c_byte),
                ("ColorID", c_byte),
                ("NumberBytesHeader", c_byte),
                ("NumberBytesData", c_ubyte),  # DLC
                ("DescriptionID", c_short),
                ("Header", c_ubyte * 4),  # Msg Header
                ("Data", c_ubyte * 8),  # Msg Data
                ("AckBytes", c_byte * 8),
                ("Value", c_int32),
                ("MiscData", c_char)]


class _stCM_ISO157652_TxMessage(Structure):
    """
    This structure is used by icsneoISO15765_TransmitMessage
    """
    _pack_ = 2
    _fields_ = [
        ('vs_netid', c_uint16),
        ('padding', c_uint8),
        ('reserved2', c_uint8),
        ('id', c_uint32),
        ('fc_id', c_uint32),
        ('fc_id_mask', c_uint32),
        ('stMin', c_uint8),
        ('blockSize', c_uint8),
        ('flowControlExtendedAddress', c_uint8),
        ('extendedAddress', c_uint8),
        ('fs_timeout', c_uint16),
        ('fs_wait', c_uint16),
        ('data', c_uint8 * 4096),
        ('num_bytes', c_uint32),
        # ('tx_dl', c_uint8),
        ('id_29_bit_enable', c_uint16, 1),  # [Bitfield]
        ('fc_id_29_bit_enable', c_uint16, 1),  # [Bitfield]
        ('ext_address_enable', c_uint16, 1),  # [Bitfield]
        ('fc_ext_address_enable', c_uint16, 1),  # [Bitfield]
        ('overrideSTmin', c_uint16, 1),  # [Bitfield]
        ('overrideBlockSize', c_uint16, 1),  # [Bitfield]
        ('paddingEnable', c_uint16, 1),  # [Bitfield]
        ('iscanFD', c_uint16, 1),  # [Bitfield]
        ('isBRSEnabled', c_uint16, 1),  # [Bitfield]
        ('flags', c_uint16),
    ]


class NeoDevice(Structure):
    """
    A structure used by FindNeoDevicesEx  and OpenNeoDeviceEx  to locate and open neoVI devices.
    """
    _pack_ = 8
    _fields_ = [("DeviceType", c_int),
                ("Handle", c_int),
                ("NumberOfClients", c_int),
                ("SerialNumber", c_int),
                ("MaxAllowedClients", c_int)]


class neoDeviceOption(Structure):
    _pack_ = 8
    _fields_ = [("CANOptions", c_int),
                ("Reserved00", c_int),
                ("Reserved01", c_int),
                ("Reserved02", c_int),
                ("Reserved03", c_int),
                ("Reserved04", c_int),
                ("Reserved05", c_int),
                ("Reserved06", c_int),
                ("Reserved07", c_int),
                ("Reserved08", c_int),
                ("Reserved09", c_int),
                ("Reserved10", c_int),
                ("Reserved11", c_int),
                ("Reserved12", c_int),
                ("Reserved13", c_int),
                ("Reserved14", c_int),
                ("Reserved15", c_int)]


class NeoDeviceEx(Structure):
    _pack_ = 8
    _fields_ = [("ndneoDevice", NeoDevice),
                ("FirmwareMajor", c_uint),
                ("FirmwareMinor", c_uint),
                ("Status", c_uint),
                ("Options", c_uint),
                ("pAvailWIFINetwork", c_longlong),
                ("pWIFIInterfaceInfo", c_longlong),
                ("isEthernetDevice", c_int),
                ("MACAddress0", c_ubyte),
                ("MACAddress1", c_ubyte),
                ("MACAddress2", c_ubyte),
                ("MACAddress3", c_ubyte),
                ("MACAddress4", c_ubyte),
                ("MACAddress5", c_ubyte),
                ("hardwareRev", c_ushort),
                ("revReserved", c_ushort),
                ("Reserved0", c_uint),
                ("Reserved1", c_uint),
                ("Reserved2", c_uint),
                ("Reserved3", c_uint),
                ("Reserved4", c_uint),
                ("Reserved5", c_uint)]


class CAN_SETTINGS(Structure):
    _pack_ = 2
    _fields_ = [("Mode", c_ubyte),
                ("SetBaudrate", c_ubyte),
                ("Baudrate", c_ubyte),
                ("Transceiver_Mode", c_ubyte),
                ("TqSeg1", c_ubyte),
                ("TqSeg2", c_ubyte),
                ("TqProp", c_ubyte),
                ("TqSync", c_ubyte),
                ("BRP", c_short),
                ("auto_baud", c_ubyte),
                ("innerFrameDelay25us", c_ubyte)]


class SWCAN_SETTINGS(Structure):
    _pack_ = 2
    _fields_ = [("Mode", c_uint8),
                ("SetBaudrate", c_uint8),
                ("Baudrate", c_uint8),
                ("NetworkType", c_uint8),
                ("TqSeg1", c_uint8),
                ("TqSeg2", c_uint8),
                ("TqProp", c_uint8),
                ("TqSync", c_uint8),
                ("BRP", c_uint16),
                ("high_speed_auto_switch", c_uint16),
                ("auto_baud", c_uint16)]


class LIN_SETTINGS(Structure):
    _pack_ = 2
    _fields_ = [("Baudrate", c_uint32),
                ("spbrg", c_uint16),
                ("brgh", c_byte),
                ("NumBitsDelay", c_byte),
                ("MasterResistor", c_byte),
                ("Mode", c_byte)]


class CANFD_SETTINGS(Structure):
    _pack_ = 2
    _fields_ = [("FDMode", c_ubyte),
                ("FDBaudrate", c_ubyte),
                ("FDTqSeg1", c_ubyte),
                ("FDTqSeg2", c_ubyte),
                ("FDTqProp", c_ubyte),
                ("FDTqSync", c_ubyte),
                ("FDBRP", c_ushort),
                ("FDTDC", c_ubyte),
                ("reserved", c_ubyte)]


class ISO9141_KEYWORD2000__INIT_STEP(Structure):
    _fields_ = [
        ("time_500us", icscm_uint16),
        ("k", icscm_uint16),
        ("l", icscm_uint16)
    ]


class ISO9141_KEYWORD2000_SETTINGS(Structure):
    _pack_ = 2
    _fields_ = [("Baudrate", icscm_uint32),
                ("spbrg", icscm_uint16),
                ("brgh", icscm_uint16),
                ("init_steps", ISO9141_KEYWORD2000__INIT_STEP * 16),
                ("init_step_count", icscm_uint8),
                ("p2_500us", icscm_uint16),
                ("p3_500us", icscm_uint16),
                ("p4_500us", icscm_uint16),
                ("chksum_enabled", icscm_uint16)]


class UART_SETTINGS(Structure):
    _fields_ = [
        ("Baudrate", c_ushort),
        ("spbrg", c_ushort),
        ("brgh", c_ushort),
        ("parity", c_ushort),
        ("stop_bits", c_ushort),
        ("flow_control", c_ubyte),
        ("reserved_1", c_ubyte),
        ("bOptions", c_uint)
    ]


class STextAPISettings(Structure):
    _pack_ = 2
    _fields_ = [("can1_tx_id", c_uint),
                ("can1_rx_id", c_uint),
                ("can1_options", c_uint),
                ("can2_tx_id", c_uint),
                ("can2_rx_id", c_uint),
                ("can2_options", c_uint),
                ("network_enables", c_uint),
                ("can3_tx_id3", c_uint),
                ("can3_rx_id3", c_uint),
                ("can3_options", c_uint),
                ("can4_tx_id4", c_uint),
                ("can4_rx_id4", c_uint),
                ("can4_options", c_uint),
                ("Reserved0", c_int),
                ("Reserved1", c_int),
                ("Reserved2", c_int),
                ("Reserved3", c_int),
                ("Reserved4", c_int)]


class SVCAN412Settings(Structure):
    _pack_ = 2
    _fields_ = [("perf_en", c_ushort),
                ("can1", CAN_SETTINGS),
                ("canfd1", CANFD_SETTINGS),
                ("can2", CAN_SETTINGS),
                ("canfd2", CANFD_SETTINGS),
                ("network_enables", c_ulonglong),
                ("termination_enables", c_ulonglong),
                ("pwr_man_timeout", c_uint),
                ("pwr_man_enable", c_uint),
                ("network_enabled_on_boot", c_ushort),
                ("iso15765_separation_time_offset", c_short),
                ("text_api", STextAPISettings),
                ("reserved", c_uint)]


class SFireSettings(Structure):
    _pack_ = 2
    _fields_ = [
        ("can1", CAN_SETTINGS),
        ("can2", CAN_SETTINGS),
        ("can3", CAN_SETTINGS),
        ("can4", CAN_SETTINGS),
        ("swcan", SWCAN_SETTINGS),
        ("lsftcan", CAN_SETTINGS),
        ("lin1", LIN_SETTINGS),
        ("lin2", LIN_SETTINGS),
        ("lin3", LIN_SETTINGS),
        ("lin4", LIN_SETTINGS),
        ("cgi_enable_reserved", icscm_uint16),
        ("cgi_baud", icscm_uint16),
        ("cgi_tx_ifs_bit_times", icscm_uint16),
        ("cgi_rx_ifs_bit_times", icscm_uint16),
        ("cgi_chksum_enable", icscm_uint16),
        ("network_enables", icscm_uint16),
        ("network_enabled_on_boot", icscm_uint16),
        ("pwm_man_timeout", icscm_uint32),
        ("pwr_man_enable", icscm_uint16),
        ("misc_io_initial_ddr", icscm_uint16),
        ("misc_io_initial_latch", icscm_uint16),
        ("misc_io_analog_enable", icscm_uint16),
        ("misc_io_report_period", icscm_uint16),
        ("misc_io_on_report_events", icscm_uint16),
        ("ain_sample_period", icscm_uint16),
        ("ain_threshold", icscm_uint16),
        ("iso15765_separation_time_offset", icscm_uint16),
        ("iso9141_kwp_enable_reserved", icscm_uint16),
        ("iso9141_kwp_settings", ISO9141_KEYWORD2000_SETTINGS),
        ("perf_en", icscm_uint16),
        ("iso_parity", icscm_uint16),
        ("iso_msg_termination", icscm_uint16),
        ("iso_tester_pullup_enable", icscm_uint16),
        ("network_enables_2", icscm_uint16),
        ("iso9141_kwp_settings2", ISO9141_KEYWORD2000_SETTINGS),
        ("iso_parity_2", icscm_uint16),
        ("iso_msg_termination_2", icscm_uint16),
        ("iso9141_kwp_settings_3", ISO9141_KEYWORD2000_SETTINGS),
        ("iso_parity_3", icscm_uint16),
        ("iso_msg_termination_3", icscm_uint16),
        ("iso9141_kwp_settings_4", ISO9141_KEYWORD2000_SETTINGS),
        ("iso_parity_4", icscm_uint16),
        ("iso_msg_termination_4", icscm_uint16),
        ("fast_init_network_enables_1", icscm_uint16),
        ("fast_init_network_enables_2", icscm_uint16),
        ("uart", UART_SETTINGS),
        ("uart2", UART_SETTINGS),
        ("text_api", STextAPISettings)]


class stAPIFirmwareInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ("iType", c_int32),
        ("iMainFirmDateDay", c_int32),
        ("iMainFirmDateMonth", c_int32),
        ("iMainFirmDateYear", c_int32),
        ("iMainFirmDateHour", c_int32),
        ("iMainFirmDateMin", c_int32),
        ("iMainFirmDateSecond", c_int32),
        ("iMainFirmChkSum", c_int32),
        ("iAppMajor", c_ubyte),
        ("iAppMinor", c_ubyte),
        ("iManufactureDay", c_ubyte),
        ("iManufactureMonth", c_ubyte),
        ("iManufactureYear", c_ushort),
        ("iBootLoaderVersionMajor", c_ubyte),
        ("iBootLoaderVersionMinor", c_ubyte),
        ("iMainVnetHWrevMajor", c_ubyte),
        ("iMainVnetHWrevMinor", c_ubyte),
        ("iMainVnetSRAMSize", c_ubyte)
    ]


# Structure for GetRTC and SetRTC
class icsSpyTime(Structure):
    _pack_ = 8
    _fields_ = [
        ("sec", c_byte),
        ("min", c_byte),
        ("hour", c_byte),
        ("day", c_byte),
        ("month", c_byte),
        ("year", c_byte),
    ]


class NeoviDeviceFoundStructure:
    """
    this class is used to hold all the devices found on the station.
    """

    def __init__(self, serial_number: int, neo_device_artifact: NeoDevice):
        self._settings: SFireSettings = SFireSettings()
        self._SerialNumber: int = serial_number
        self._NeoDevice: NeoDevice = neo_device_artifact
        self._isOpen: bool = False
        self.message_error_counter = 0  # this parameter is used to count the number of errors occured when reading from
        # a device.
        self.iso15765_message_buffer = []  # holds all incoming messages while iso15765 mode is active
        self.iso15765_mutex = False  # this flag indicate if the ISO15765 and thus get_message
        # won't work in normal mode.
        self._max_allowed_threshold = 5

        # _hObject parameter needs to be 32 bit in a 32 bit program and 64 bit in a 64 bit program.
        if sys.maxsize > 2 ** 32:
            tObject = c_uint64(0)
        else:
            tObject = c_uint32(0)
        self._hObject: c_void_p = cast(byref(tObject), c_void_p)

        self._isISO15765Enabled: bool = False

    @property
    def Settings(self) -> SFireSettings:
        return self._settings

    @Settings.setter
    def Settings(self, value: SFireSettings):
        if isinstance(value, SFireSettings):
            self._settings = value

    @property
    def SerialNumber(self) -> int:
        return self._SerialNumber

    @SerialNumber.setter
    def SerialNumber(self, value: int):
        self._SerialNumber = value

    @property
    def NeoviDevice(self) -> NeoDevice:
        return self._NeoDevice

    @NeoviDevice.setter
    def NeoviDevice(self, value: NeoDevice):
        if isinstance(value, NeoDevice):
            self._NeoDevice = value

    @property
    def isISO15765Enabled(self) -> bool:
        return self._isISO15765Enabled

    @isISO15765Enabled.setter
    def isISO15765Enabled(self, value: bool):
        if isinstance(value, bool):
            self._isISO15765Enabled = value

    @property
    def isOpen(self) -> bool:
        return self._isOpen

    @isOpen.setter
    def isOpen(self, value: bool):
        if isinstance(value, bool):
            self._isOpen = value

    @property
    def hObject(self) -> c_void_p:
        return self._hObject

    @hObject.setter
    def hObject(self, value: c_void_p):
        self._hObject = value


# holds all found devices structure is {SerialNumber, deviceHandler}
neovi_device_found = []


def _get_error_messages(error_number: str):
    """
    Returns the possible error messages returned in the GetErrorMessages API call.

    :param error_number: a string represent the error number occurred.
    :returns: error description
    :raises KeyError: if error is unknown
    """
    try:
        return {
            "0": "NEOVI_ERROR_DLL_TX_BUFFER_OVERFLOW",
            "1": "NEOVI_ERROR_DLL_ERROR_BUFFER_OVERFLOW",
            "2": "NEOVI_ERROR_DLL_USB_SEND_DATA_ERROR",
            "3": "NEOVI_ERROR_DLL_ISO_DATA_BUFFER_ALLOC",
            "4": "NEOVI_ERROR_DLL_ISO_DATA_READ_BUFFER",
            "5": "NEOVI_ERROR_DLL_ISO_DATA_ZERO_PACKETS",
            "6": "NEOVI_ERROR_DLL_RX_MSG_BUFFER_OVERFLOW",
            "7": "NEOVI_ERROR_DLL_STOP_ISO_STREAM",
            "8": "NEOVI_ERROR_DLL_INVALID_NETID",
            "9": "NEOVI_ERROR_DLL_PROBLEM_STOPPING_RX_THREAD",
            "10": "NEOVI_ERROR_DLL_PROBLEM_STOPPING_TX_THREAD",
            "11": "NEOVI_ERROR_DLL_MAIN_PIC_BUFFER_OVERFLOW",
            "12": "NEOVI_ERROR_DLL_INVALID_DEVICE_RESPONSE",
            "13": "NEOVI_ERROR_DLL_ISOTX_DATA_BUFFER_ALLOC",
            "14": "NEOVI_ERROR_DLL_RX_CMD_BUFFER_OVERFLOW",
            "15": "NEOVI_ERROR_DLL_RS232_RX_BUFFER_OVERFLOW",
            "16": "NEOVI_ERROR_DLL_RS232_ERR_READCOMERR",
            "17": "NEOVI_ERROR_DLL_RS232_ERR_READ",
            "18": "NEOVI_ERROR_DLL_RS232_BUFFER_ALLOC",
            "19": "NEOVI_ERROR_DLL_RS232_TX_BUFFER_OVERFLOW",
            "20": "NEOVI_ERROR_DLL_RS232_MISC_ERROR",
            "21": "NEOVI_ERROR_DLL_RS232_FIND_WRITE",
            "22": "NEOVI_ERROR_DLL_RS232_FIND_BUFFER_ALLOC",
            "23": "NEOVI_ERROR_DLL_RS232_FIND_CLEARCOMM",
            "24": "NEOVI_ERROR_DLL_RS232_FIND_READCOMM",
            "25": "NEOVI_ERROR_DLL_RS232_FIND_TIMEOUT",
            "26": "NEOVI_ERROR_DLL_RS232_ERR_BREAK",
            "27": "NEOVI_ERROR_DLL_RS232_ERR_FRAME",
            "28": "NEOVI_ERROR_DLL_RS232_ERR_IOE",
            "29": "NEOVI_ERROR_DLL_RS232_ERR_OVERRUN",
            "30": "NEOVI_ERROR_DLL_RS232_ERR_PARITY",
            "31": "NEOVI_ERROR_DLL_RS232_TXBUFFER_ALLOC",
            "32": "NEOVI_ERROR_DLL_USB_TX_RS232_ERROR",
            "33": "NEOVI_ERROR_DLL_RS232_CREATE_FILE",
            "34": "NEOVI_ERROR_DLL_RS232_GET_COMM_STATE",
            "35": "NEOVI_ERROR_DLL_RS232_SET_COMM_STATE",
            "36": "NEOVI_ERROR_DLL_RS232_START_COMM_RX_THREAD",
            "37": "NEOVI_ERROR_DLL_RS232_START_COMM_TX_THREAD",
            "38": "NEOVI_ERROR_DLL_SYNC_COUNT_ERR",
            "39": "NEOVI_ERROR_DLL_RX_MSG_FRAME_ERR",
            "40": "NEOVI_ERROR_DLL_RX_MSG_FIFO_OVER",
            "41": "NEOVI_ERROR_DLL_RX_MSG_CHK_SUM_ERR",
            "42": "NEOVI_ERROR_DLL_PROBLEM_STOPPING_BULKIN_THREAD",
            "43": "NEOVI_ERROR_DLL_BULKIN_ERR_READ",
            "44": "NEOVI_ERROR_DLL_MAIN51_RX_FIFO_OVERFLOW",
            "45": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW",
            "46": "NEOVI_ERROR_DLL_MAIN51_DEV_FIFO_OVERFLOW",
            "47": "NEOVI_ERROR_DLL_RESET_STATUS_CHANGED",
            "48": "NEOVI_ERROR_DLL_ISO_LONG_CACHE_OVERFLOW",
            "49": "NEOVI_ERROR_DLL_ISORX_LONG_BUFFER_ALLOC",
            "50": "NEOVI_ERROR_DLL_J1708_LONG_CACHE_OVERFLOW",
            "51": "NEOVI_ERROR_DLL_J1708_LONG_BUFFER_ALLOC",
            "52": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_DEVICE",
            "53": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_HSCAN",
            "54": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_MSCAN",
            "55": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_SWCAN",
            "56": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_LSFTCAN",
            "57": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_FORDSCP",
            "58": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_J1708",
            "59": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_AUX",
            "60": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_JVPW",
            "61": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_ISO",
            "62": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_ISOPIC",
            "63": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_MAIN51",
            "64": "NEOVI_ERROR_DLL_MAIN51_TX_FIFO_OVERFLOW_HOST",
            "65": "NEOVI_ERROR_DLL_READ_ENTIRE_DEEPROM_ERROR",
            "66": "NEOVI_ERROR_DLL_WRITE_ENTIRE_DEEPROM_ERROR",
            "67": "NEOVI_ERROR_DLL_USB_PORT_ALREADY_OPEN",
            "68": "NEOVI_ERROR_DLL_JVPW_TX_REPORT_FIFO_ERR_IN",
            "69": "NEOVI_ERROR_DLL_ISOJ_TX_REPORT_FIFO_ERR_IN",
            "70": "NEOVI_ERROR_DLL_JVPW_TX_REPORT_FIFO_ERR_OUT",
            "71": "NEOVI_ERROR_DLL_ISOJ_TX_REPORT_FIFO_ERR_OUT",
            "72": "NEOVI_ERROR_DLL_MAIN51_TX_IN_FROM_HOST_FIFO",
            "73": "NEOVI_ERROR_DLL_MAIN51_TX_HOST_CHKSUM",
            "74": "NEOVI_ERROR_DLL_ISOJ_TX_HOST_MISSED_BYTE",
            "75": "NEOVI_ERROR_DLL_NEOVI_NO_RESPONSE",
            "76": "NEOVI_ERROR_DLL_RX_SOCKET_FIFO_OVER",
            "77": "NEOVI_ERROR_DLL_PROBLEM_STOPPING_TXSOCKET_THREAD",
            "78": "NEOVI_ERROR_DLL_PROBLEM_STOPPING_RXSOCKET_THREAD",
            "79": "NEOVI_ERROR_DLL_PROBLEM_STOPPING_RXSOCKET_CLIENT_THREAD",
            "80": "NEOVI_ERROR_DLL_TCP_CLIENT_TX",
            "81": "NEOVI_ERROR_DLL_TCP_CLIENT_RX",
            "82": "NEOVI_ERROR_DLL_TCP_CLIENT_RX_SOCK",
            "83": "NEOVI_ERROR_DLL_UNABLE_CONNECT_TO_SRVR",
            "84": "NEOVI_ERROR_DLL_UNABLE_CREATE_CLIENT_SOCK",
            "85": "NEOVI_ERROR_DLL_UNABLE_WSASTARTUP",
            "86": "NEOVI_ERROR_DLL_SOCK_CL_RD_BUFFER_ALLOC",
            "87": "NEOVI_ERROR_DLL_SOCK_CL_TX_BUFFER_ALLOC",
            "88": "NEOVI_ERROR_DLL_SOCK_SRVR_RX_BUFFER_ALLOC",
            "89": "NEOVI_ERROR_DLL_SOCK_SRVR_TX_BUFFER_ALLOC",
            "90": "NEOVI_ERROR_DLL_ILLEGAL_TX_NETWORK",
            "91": "NEOVI_ERROR_DLL_MAIN51_TX_HOST_OVERRUN",
            "92": "NEOVI_ERROR_DLL_OPEN_GET_COMM_TIMEOUT",
            "93": "NEOVI_ERROR_DLL_OPEN_SET_COMM_TIMEOUT",
            "94": "NEOVI_ERROR_DLL_OPEN_READ_DEVICE_TYPE",
            "95": "NEOVI_ERROR_DLL_OPEN_READ_DEVICE_TYPE_TOUT",
            "96": "NEOVI_ERROR_DLL_CLOSE_PURGE_COMM",
            "97": "NEOVI_ERROR_DLL_TX_COM_FIFO_OVERFLOW",
            "98": "NEOVI_ERROR_DLL_GET_USBSERIAL_DEVICES",
            "99": "NEOVI_ERROR_DLL_USB_TX_RS232_BCOUNT",
            "100": "NEOVI_ERROR_DLL_OPEN_INBOOTLOADER",
            "101": "NEOVI_ERROR_DLL_TOO_MANY_PERIODICS",
            "102": "NEOVI_ERROR_DLL_PROBLEM_FIRMWARE_INFO",
            "103": "NEOVI_ERROR_DLL_NRED_ODDNUMBYTES",
            "104": "NEOVI_ERROR_DLL_NRED_UNKNOWN_RED_NETID",
            "105": "NEOVI_ERROR_DLL_RED_NOT_SUPPORTED",
            "106": "NEOVI_ERROR_DLL_RED_BL_START_INDEX",
            "107": "NEOVI_ERROR_DLL_3G_BL_FAILURE",
            "116": "NEOVI_ERROR_DLL_RED_BL_END_INDEX",
            "117": "NEOVI_ERROR_DLL_RED_FAILED_TO_ENTER_BL",
            "118": "NEOVI_ERROR_DLL_RED_REQ_SERIAL_NUMBER",
            "119": "NEOVI_ERROR_DLL_RED_AUTHENTICATE",
            "120": "NEOVI_ERROR_DLL_RED_APP_VERSION",
            "121": "NEOVI_ERROR_DLL_RED_SET_BAUD_COMM_FAILURE",
            "122": "NEOVI_ERROR_DLL_RED_INVALID_BAUD_SPECIFIED",
            "123": "NEOVI_ERROR_DLL_RED_READ_BAUD_COMM_FAILURE",
            "124": "NEOVI_ERROR_DLL_RED_FAILED_TO_SAVE_EEPROM",
            "125": "NEOVI_ERROR_DLL_RED_FAILED_TO_UPDATE_WAVEFORM_CHANNEL",
            "126": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_UNKNOWN_NETWORK",
            "127": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_UNKNOWN_NETWORK",
            "128": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_UNKNOWN_NETWORK",
            "129": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_HSCAN1",
            "130": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_HSCAN1",
            "131": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_HSCAN1",
            "132": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_HSCAN1",
            "133": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_HSCAN2",
            "134": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_HSCAN2",
            "135": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_HSCAN2",
            "136": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_HSCAN2",
            "137": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_MSCAN1",
            "138": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_MSCAN1",
            "139": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_MSCAN1",
            "140": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_MSCAN1",
            "141": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_HSCAN3",
            "142": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_HSCAN3",
            "143": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_HSCAN3",
            "144": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_HSCAN3",
            "145": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_SWCAN",
            "146": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_SWCAN",
            "147": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_SWCAN",
            "148": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_SWCAN",
            "149": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_LSFTCAN",
            "150": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_LSFTCAN",
            "151": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_LSFTCAN",
            "152": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_LSFTCAN",
            "153": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_LIN1",
            "154": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_LIN1",
            "155": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_LIN1",
            "156": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_LIN1",
            "157": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_LIN2",
            "158": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_LIN2",
            "159": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_LIN2",
            "160": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_LIN2",
            "161": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_LIN3",
            "162": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_LIN3",
            "163": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_LIN3",
            "164": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_LIN3",
            "165": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_LIN4",
            "166": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_LIN4",
            "167": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_LIN4",
            "168": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_LIN4",
            "169": "NEOVI_ERROR_DLL_USB_PURGE_FAILED",
            "170": "NEOVI_ERROR_FIRE_COMM_BAD_PACKET",
            "171": "NEOVI_ERROR_FIRE_CGI_COMM_BAD_PACKET",
            "172": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_HSCAN1",
            "173": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_HSCAN2",
            "174": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_HSCAN3",
            "175": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_MSCAN",
            "176": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_SWCAN",
            "177": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_LSFTCAN",
            "178": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_LIN1",
            "179": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_LIN2",
            "180": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_LIN3",
            "181": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_LIN4",
            "182": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_UNKNOWN_NETWORK",
            "183": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_JVPW",
            "184": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_JVPW",
            "185": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_JVPW",
            "186": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_JVPW",
            "187": "NEOVI_ERROR_DLL_INTERNAL_SERIAL_NO_DOES_NOT_MATCH_REGISTRY_SERIAL_NO",
            "188": "NEOVI_ERROR_DLL_JVPW_LONG_CACHE_OVERFLOW",
            "189": "NEOVI_ERROR_DLL_FAILED_TO_SET_LICENSE",
            "190": "NEOVI_ERROR_DLL_3G_DEVICE_LICENSE_NEEDS_TO_BE_UPGRADED",
            "191": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_HSCAN",
            "192": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_MSCAN",
            "193": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_HSCAN2",
            "194": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_HSCAN3",
            "195": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_LSFT",
            "196": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_SW",
            "197": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_LIN1",
            "198": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_LIN2",
            "199": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_LIN3",
            "200": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_LIN4",
            "201": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_JVPW",
            "202": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_KYW",
            "203": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_J1708",
            "204": "NEOVI_ERROR_DLL_MAIN51_RTC_INVALID",
            "205": "NEOVI_ERROR_DLL_MAIN51_LOADED_DEFAULT_SETTINGS",
            "206": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_CGI",
            "207": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_CGI",
            "208": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_CGI",
            "209": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_CGI",
            "210": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_CGI",
            "211": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_CGI",
            "212": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_JVPW",
            "213": "NEOVI_ERROR_DLL_INVALID_SCRIPT_LOCATION",
            "214": "NEOVI_ERROR_DLL_SDCARD_NOT_INSERTED",
            "215": "NEOVI_ERROR_DLL_SDCARD_NOT_FORMATTED",
            "216": "NEOVI_ERROR_DLL_SDCARD_WRITE_ERROR",
            "217": "NEOVI_ERROR_DLL_SDCARD_READ_ERROR",
            "218": "NEOVI_ERROR_DLL_SCRIPT_START_ERROR",
            "219": "NEOVI_ERROR_DLL_SCRIPT_INVALID_FUNCBLOCK_INDEX",
            "220": "NEOVI_ERROR_DLL_SCRIPT_ERROR_DOWNLOADING_SCRIPT",
            "221": "NEOVI_ERROR_DLL_SCRIPT_ERROR_CLEARING_SCRIPT",
            "222": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_ISO",
            "223": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_ISO",
            "224": "NEOVI_ERROR_DLL_SCRIPT_INVALID_MSG_INDEX",
            "225": "NEOVI_ERROR_DLL_SCRIPT_INVALID_APPSIG_INDEX",
            "226": "NEOVI_ERROR_DLL_SCRIPT_NO_SCRIPT_RUNNING",
            "227": "NEOVI_ERROR_DLL_COULD_NOT_SET_SETTINGS_FIRMWARE_MISMATCH",
            "228": "NEOVI_ERROR_DLL_FIRE_CGI_TX_NOT_ENABLED",
            "229": "NEOVI_ERROR_DLL_SEND_DEVICE_CONFIG_ERROR",
            "230": "NEOVI_ERROR_DLL_GET_DEVICE_CONFIG_ERROR",
            "231": "NEOVI_ERROR_DLL_UNKNOWN_NEOVI_TYPE",
            "232": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_ISO2",
            "233": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_ISO3",
            "234": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_ISO4",
            "235": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_ISO2",
            "236": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_ISO2",
            "237": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_ISO2",
            "238": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_ISO2",
            "239": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_ISO2",
            "240": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_ISO3",
            "241": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_ISO3",
            "242": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_ISO3",
            "243": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_ISO3",
            "244": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_ISO3",
            "245": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_ISO4",
            "246": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_ISO4",
            "247": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_ISO4",
            "248": "NEOVI_ERROR_DLL_RED_DRIVER_OVERFLOW_ISO4",
            "249": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_ISO4",
            "250": "NEOVI_ERROR_DLL_RED_FAILED_TO_CLEAR_LIN_SLAVE_DATA",
            "251": "NEOVI_ERROR_DLL_NETWORK_NOT_ENABLED_ISO1",
            "252": "NEOVI_ERROR_DLL_RED_RX_MSG_FULL_ISO1",
            "253": "NEOVI_ERROR_DLL_RED_TX_MSG_FULL_ISO1",
            "254": "NEOVI_ERROR_DLL_RED_TX_REPORT_MSG_FULL_ISO1",
            "255": "NEOVI_ERROR_DLL_RED_SETTINGS_NOT_SET_ISO1",
            "256": "NEOVI_ERROR_DLL_DROPPED_RTC_CMD",
            "257": "NEOVI_ERROR_DLL_J1850_TX_REQUESTS_FLUSHED",
            "258": "NEOVI_ERROR_J1708_COMM_BAD_PACKET",
            "259": "NEOVI_ERROR_DLL_NETWORK_NOT_SUPPORTED_BY_HARDWARE",
            "260": "NEOVI_ERROR_DLL_FEATURE_NOT_UNLOCKED",
            "261": "NEOVI_ERROR_DLL_DEVICE_NOT_POWERED",
            "262": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_OK",
            "263": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORBADINIT",
            "264": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORNOCANPIPE",
            "265": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORISONEG7FRESPONSE",
            "266": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORTOOLNOTSELECTED",
            "267": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORINVALIDDEVICESELECTED",
            "268": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORCOULDNOTOPENTOOL",
            "269": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORNOFLOWCONTROL",
            "270": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORUNSPECIFIC",
            "271": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORCOREMININULLPTR",
            "272": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORCOREMINIZEROLEN",
            "273": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORRESERVED3",
            "274": "NEOVI_ERROR_DLL_3GCANDOWNLOADER_ERRORRESERVED4",
            "275": "NEOVI_ERROR_DLL_YELLOW_DEPRECATED",
            "276": "NEOVI_ERROR_DLL_HARDWARE_FAILURE_SRAM",
            "277": "NEOVI_ERROR_ACTIVE_CONNECTION_ALREADY_EXISTS",
            "278": "NEOVI_ERROR_DLL_MAIN51_RTC_FAILED_READ",
            "279": "NEOVI_ERROR_DLL_MAIN51_RTC_AUTO_RTC",
            "287": "NEOVI_ERROR_DLL_SEND_DEVICE_CONFIG_NOTPOSSIBLE",
            "288": "NEOVI_ERROR_CHANNEL_LOCKED_BY_OTHER_CLIENT",
            "289": "NEOVI_ERROR_NEOVISERVER_GENERAL_ERROR",
            "290": "NEOVI_ERROR_CHANNEL_LOCKING_NOT_SUPPORTED_FOR_DEVICE"
        }[str(error_number)]
    except KeyError:
        raise KeyError("Unknown(" + str(error_number) + ")")


def _read_message_status_bits(message: icsSpyMessage) -> dict:
    """
    Internal usage only: this function convert icsSpyMessage datatype to dict

    :param message: icsSpyMessage datatype
    :return: detailed message information as dict.
    """
    return {
        "SPY_STATUS_GLOBAL_ERR": message.StatusBitField & StatusBitfield['SPY_STATUS_GLOBAL_ERR'],
        "SPY_STATUS_TX_MSG": message.StatusBitField & StatusBitfield['SPY_STATUS_TX_MSG'],
        "SPY_STATUS_XTD_FRAME": message.StatusBitField & StatusBitfield["SPY_STATUS_XTD_FRAME"],
        "SPY_STATUS_REMOTE_FRAME": message.StatusBitField & StatusBitfield["SPY_STATUS_REMOTE_FRAME"],
        "SPY_STATUS_CRC_ERROR": message.StatusBitField & StatusBitfield["SPY_STATUS_CRC_ERROR"],
        "SPY_STATUS_CAN_ERROR_PASSIVE": message.StatusBitField & StatusBitfield["SPY_STATUS_CAN_ERROR_PASSIVE"],
        "SPY_STATUS_INCOMPLETE_FRAME": message.StatusBitField & StatusBitfield["SPY_STATUS_INCOMPLETE_FRAME"],
        "SPY_STATUS_LOST_ARBITRATION": message.StatusBitField & StatusBitfield["SPY_STATUS_LOST_ARBITRATION"],
        "SPY_STATUS_UNDEFINED_ERROR": message.StatusBitField & StatusBitfield["SPY_STATUS_UNDEFINED_ERROR"],
        "SPY_STATUS_CAN_BUS_OFF": message.StatusBitField & StatusBitfield["SPY_STATUS_CAN_BUS_OFF"],
        "SPY_STATUS_CAN_ERROR_WARNING": message.StatusBitField & StatusBitfield["SPY_STATUS_CAN_ERROR_WARNING"],
        "SPY_STATUS_BUS_SHORTED_PLUS": message.StatusBitField & StatusBitfield["SPY_STATUS_BUS_SHORTED_PLUS"],
        "SPY_STATUS_BUS_SHORTED_GND": message.StatusBitField & StatusBitfield["SPY_STATUS_BUS_SHORTED_GND"],
        "SPY_STATUS_CHECKSUM_ERROR": message.StatusBitField & StatusBitfield["SPY_STATUS_CHECKSUM_ERROR"],
        "SPY_STATUS_BAD_MESSAGE_BIT_TIME_ERROR": message.StatusBitField & StatusBitfield[
            "SPY_STATUS_BAD_MESSAGE_BIT_TIME_ERROR"],
        "SPY_STATUS_IFR_DATA": message.StatusBitField & StatusBitfield["SPY_STATUS_IFR_DATA"],
        "SPY_STATUS_HARDWARE_COMM_ERROR": message.StatusBitField & StatusBitfield["SPY_STATUS_HARDWARE_COMM_ERROR"],
        "SPY_STATUS_EXPECTED_LEN_ERROR": message.StatusBitField & StatusBitfield["SPY_STATUS_EXPECTED_LEN_ERROR"],
        "SPY_STATUS_INCOMING_NO_MATCH": message.StatusBitField & StatusBitfield["SPY_STATUS_INCOMING_NO_MATCH"],
        "SPY_STATUS_BREAK": message.StatusBitField & StatusBitfield["SPY_STATUS_BREAK"],
        "SPY_STATUS_AVSI_REC_OVERFLOW": message.StatusBitField & StatusBitfield["SPY_STATUS_AVSI_REC_OVERFLOW"],
        "SPY_STATUS_TEST_TRIGGER": message.StatusBitField & StatusBitfield["SPY_STATUS_TEST_TRIGGER"],
        "SPY_STATUS_AUDIO_COMMENT": message.StatusBitField & StatusBitfield["SPY_STATUS_AUDIO_COMMENT"],
        "SPY_STATUS_GPS_DATA": message.StatusBitField & StatusBitfield["SPY_STATUS_GPS_DATA"],
        "SPY_STATUS_ANALOG_DIGITAL_INPUT": message.StatusBitField & StatusBitfield["SPY_STATUS_ANALOG_DIGITAL_INPUT"],
        "SPY_STATUS_TEXT_COMMENT": message.StatusBitField & StatusBitfield["SPY_STATUS_TEXT_COMMENT"],
        "SPY_STATUS_NETWORK_MESSAGE_TYPE": message.StatusBitField & StatusBitfield["SPY_STATUS_NETWORK_MESSAGE_TYPE"],
        "SPY_STATUS_VSI_TX_UNDERRUN": message.StatusBitField & StatusBitfield["SPY_STATUS_VSI_TX_UNDERRUN"],
        "SPY_STATUS_VSI_IFR_CRC_Bit": message.StatusBitField & StatusBitfield["SPY_STATUS_VSI_IFR_CRC_Bit"],
        "SPY_STATUS_INIT_MESSAGE": message.StatusBitField & StatusBitfield["SPY_STATUS_INIT_MESSAGE"],
        "SPY_STATUS_HIGH_SPEED_MESSAGE": message.StatusBitField & StatusBitfield["SPY_STATUS_HIGH_SPEED_MESSAGE"],
        "SPY_STATUS_FLEXRAY_SECOND_STARTUP_FRAME": message.StatusBitField & StatusBitfield[
            "SPY_STATUS_FLEXRAY_SECOND_STARTUP_FRAME"],
        "SPY_STATUS_EXTENDED": message.StatusBitField & StatusBitfield["SPY_STATUS_EXTENDED"],
        "SPY_STATUS2_HAS_VALUE": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_HAS_VALUE"],
        "SPY_STATUS2_VALUE_IS_BOOLEAN": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_VALUE_IS_BOOLEAN"],
        "SPY_STATUS2_HIGH_VOLTAGE": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_HIGH_VOLTAGE"],
        "SPY_STATUS2_LONG_MESSAGE": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_LONG_MESSAGE"],
        "SPY_STATUS2_GLOBAL_CHANGE": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_GLOBAL_CHANGE"],
        "SPY_STATUS2_ERROR_FRAME": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_ERROR_FRAME"],
        "SPY_STATUS2_END_OF_LONG_MESSAGE": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_END_OF_LONG_MESSAGE"],
        "SPY_STATUS2_LIN_ERR_RX_BREAK_NOT_0": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_ERR_RX_BREAK_NOT_0"],
        "SPY_STATUS2_LIN_ERR_RX_BREAK_TOO_SHORT": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_ERR_RX_BREAK_TOO_SHORT"],
        "SPY_STATUS2_LIN_ERR_RX_SYNC_NOT_55": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_ERR_RX_SYNC_NOT_55"],
        "SPY_STATUS2_LIN_ERR_RX_DATA_GREATER_8": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_ERR_RX_DATA_GREATER_8"],
        "SPY_STATUS2_LIN_ERR_TX_RX_MISMATCH": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_ERR_TX_RX_MISMATCH"],
        "SPY_STATUS2_LIN_ERR_MSG_ID_PARITY": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_ERR_MSG_ID_PARITY"],
        "SPY_STATUS2_ISO_FRAME_ERROR": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_ISO_FRAME_ERROR"],
        "SPY_STATUS2_LIN_SYNC_FRAME_ERROR": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_SYNC_FRAME_ERROR"],
        "SPY_STATUS2_ISO_OVERFLOW_ERROR": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_ISO_OVERFLOW_ERROR"],
        "SPY_STATUS2_LIN_ID_FRAME_ERROR": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_LIN_ID_FRAME_ERROR"],
        "SPY_STATUS2_ISO_PARITY_ERROR": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_ISO_PARITY_ERROR"],
        "SPY_STATUS2_LIN_SLAVE_BYTE_ERROR": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_LIN_SLAVE_BYTE_ERROR"],
        "SPY_STATUS2_RX_TIMEOUT_ERROR": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_RX_TIMEOUT_ERROR"],
        "SPY_STATUS2_LIN_NO_SLAVE_DATA": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_LIN_NO_SLAVE_DATA"],
        "SPY_STATUS2_MOST_PACKET_DATA": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_PACKET_DATA"],
        "SPY_STATUS2_MOST_STATUS": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_STATUS"],
        "PY_STATUS2_MOST_LOW_LEVEL": message.StatusBitField2 & StatusBitfield2["PY_STATUS2_MOST_LOW_LEVEL"],
        "SPY_STATUS2_MOST_CONTROL_DATA": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_CONTROL_DATA"],
        "SPY_STATUS2_MOST_MHP_USER_DATA": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_MHP_USER_DATA"],
        "SPY_STATUS2_MOST_MHP_CONTROL_DATA": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_MOST_MHP_CONTROL_DATA"],
        "SPY_STATUS2_MOST_I2S_DUMP": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_I2S_DUMP"],
        "SPY_STATUS2_MOST_TOO_SHORT": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_TOO_SHORT"],
        "SPY_STATUS2_MOST_MOST50": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_MOST50"],
        "SPY_STATUS2_MOST_MOST150": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_MOST150"],
        "SPY_STATUS2_MOST_CHANGED_PAR": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_MOST_CHANGED_PAR"],
        "SPY_STATUS2_ETHERNET_CRC_ERROR": message.StatusBitField2 & StatusBitfield2["SPY_STATUS2_ETHERNET_CRC_ERROR"],
        "SPY_STATUS2_ETHERNET_FRAME_TOO_SHORT": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_ETHERNET_FRAME_TOO_SHORT"],
        "SPY_STATUS2_ETHERNET_FCS_AVAILABLE": message.StatusBitField2 & StatusBitfield2[
            "SPY_STATUS2_ETHERNET_FCS_AVAILABLE"],
        "SPY_STATUS3_LIN_JUST_BREAK_SYNC": message.StatusBitField3 & StatusBitfield3["SPY_STATUS3_LIN_JUST_BREAK_SYNC"],
        "SPY_STATUS3_LIN_SLAVE_DATA_TOO_SHORT": message.StatusBitField3 & StatusBitfield3[
            "SPY_STATUS3_LIN_SLAVE_DATA_TOO_SHORT"],
        "SPY_STATUS3_LIN_ONLY_UPDATE_SLAVE_TABLE_ONCE": message.StatusBitField3 & StatusBitfield3[
            "SPY_STATUS3_LIN_ONLY_UPDATE_SLAVE_TABLE_ONCE"]
    }


def _handle_error(Device: neovi_device_found):
    """
    Internal usage only: This method returns the error generated by the last API call.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: string represent the execution status with the device serial number
    :raises ConnectionError: connectivity problem with the connected device.
    """
    error = c_int(0)
    if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(error)):
        szErrorDescriptionShort = c_char_p()
        szErrorDescriptionLong = c_char_p()
        lMaxLengthShort = c_int(0)
        lMaxLengthLong = c_int(0)
        lErrorSeverity = c_int(0)
        lRestartNeeded = c_int(0)
        neovi_dll.icsneoGetErrorInfo(error,
                                     byref(szErrorDescriptionShort),
                                     byref(szErrorDescriptionLong),
                                     byref(lMaxLengthShort),
                                     byref(lMaxLengthLong),
                                     byref(lErrorSeverity),
                                     byref(lRestartNeeded),
                                     )
        raise ValueError("".join([
            "error code: ", str(error),
            "Short Description: ", [szErrorDescriptionShort[item] for item in range(lMaxLengthShort.value)]
        ]))


def _get_error_messages_description(lErrorNumber: c_int) -> dict:
    """
    Internal usage only: this function convert error code returned by the device to detailed information

    :param lErrorNumber: error code returned by the device
    :return: detailed error information as dict
    """
    ErrorDescriptionShort = c_char * 256
    sErrorDescriptionShort = ErrorDescriptionShort()
    ErrorDescriptionLong = c_char * 256
    sErrorDescriptionLong = ErrorDescriptionLong()
    lMaxLengthShort = c_int32(255)
    lMaxLengthLong = c_int32(255)
    lErrorSeverity = c_int32(0)
    lRestartNeeded = c_int32(0)

    # This method returns a text description of an intrepidcs API error number.
    neovi_dll.icsneoGetErrorInfo(
        lErrorNumber,
        byref(sErrorDescriptionShort),
        byref(sErrorDescriptionLong),
        byref(lMaxLengthShort),
        byref(lMaxLengthLong),
        byref(lErrorSeverity),
        byref(lRestartNeeded)
    )

    return {"errorNumber": lErrorNumber,
            "errorDescriptionShort": "".join([sErrorDescriptionShort[index].decode() for index in
                                              range(lMaxLengthShort.value - 1)]),
            "errorDescriptionLong": "".join([sErrorDescriptionLong[index].decode() for index in
                                             range(lMaxLengthLong.value - 1)]),
            "errorSeverity": lErrorSeverity.value,
            "restartNeeded": lRestartNeeded.value}


def _neovi_get_error_information(Device: NeoviDeviceFoundStructure) -> list:
    """
    This method returns a text description of an intrepidcs API error number.

    :param Device: the device datastructure (neovi_device_found)
    :return: list of error description
    :raise ConnectionError: if the device is not connected
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (4): the requested device is closed, can't work with it.")

    ErrorMsgsArray = c_int32 * 600
    pErrorMsgs = ErrorMsgsArray()
    pNumberOfErrors = c_int32(0)

    # This method reads the neoVI DLL error message queue.
    neovi_dll.icsneoGetErrorMessages(Device.hObject,
                                     byref(pErrorMsgs),
                                     byref(pNumberOfErrors))

    return [str(_get_error_messages_description(pErrorMsgs[index]))
            for index in range(pNumberOfErrors.value)]


def neovi_get_serial_number(Device: NeoDeviceEx) -> int:
    """
    return the serial number by device type
    Mainly used by neovi_list_can_devices function.

    :param Device: the device datastructure (NeoDeviceEx)
    :return: Serial number
    """

    if Device.ndneoDevice.DeviceType in (device_types['NEODEVICE_VCAN42'],
                                         device_types['NEODEVICE_VCAN41'],
                                         device_types['NEODEVICE_VCAN44'],
                                         device_types['NEODEVICE_VCAN42_EL']):
        try:
            return int(base36.dumps(Device.ndneoDevice.SerialNumber).replace('v', ''))
        except ValueError:
            return 0
    else:
        return Device.ndneoDevice.SerialNumber


def neovi_list_can_devices(number_of_devices_to_search: int = 15) -> [NeoviDeviceFoundStructure]:
    """
    function to return a list of serial devices found connected the locally.
    also, this function fills the device_found dict, for future use.

    :param number_of_devices_to_search: the number of devices to search.
    :return: list of Devices (NeoviDeviceFoundStructure) connected.
    :raise ConnectionError: no device were found
    """
    NeoDevice_Array = NeoDeviceEx * number_of_devices_to_search
    Devices = NeoDevice_Array()
    iNumberOfDevices = c_int(number_of_devices_to_search)

    # CAN netowork ID for connecting to devices over CAN.  Set to null for USB or Ethernet connections.
    neoDeviceOptionEx = neoDeviceOption()
    neoDeviceOptionEx.CANOptions = c_int(0)

    # This is an array of device types to look for. Specifies the types of neoVI devices to find.
    # Each element in the array need to have a value for the device type to look for.
    pDeviceTypes = c_uint(device_types['NEODEVICE_ALL'])
    # Specifies the size of the DeviceTypes array. Must be in the range 0 to 255.
    numDeviceTypes = c_uint(0)
    try:
        neovi_dll.icsneoFindDevices(byref(Devices), byref(iNumberOfDevices), byref(pDeviceTypes),
                                    numDeviceTypes, byref(neoDeviceOptionEx), 0)
    except Exception:
        raise ConnectionError("Problem (3): Unable to find devices on this platform")

    # we save the information of the found devices for future management.
    return [NeoviDeviceFoundStructure(serial_number=neovi_get_serial_number(Devices[index]),
                                      neo_device_artifact=Devices[index].ndneoDevice)  # TODO: I'm copy the object
            for index in range(iNumberOfDevices.value)]


def neovi_get_dll_version() -> str:
    """
    This method returns the software version of the DLL.

    :returns: string represent the DLL version.
    """
    return str(neovi_dll.icsneoGetDLLVersion())


def neovi_open_can_device(Device: NeoviDeviceFoundStructure) -> str:
    """
    function to open a device that was previously found from the list.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: string represent the execution status with the device serial number
    :raises ConnectionError: connectivity problem with the connected device.
    """
    if Device.isOpen:
        raise ConnectionError("Problem (0):The selected device is already open, can't re-open it again.")

    # Specifies whether the DLL should read the neoVI's device configuration before enabling the device.
    # It is recommended that this value be set to 1.
    bConfigRead = c_int(1)

    # this task might take few seconds
    Res = neovi_dll.icsneoOpenNeoDevice(Device.NeoviDevice, byref(Device.hObject),
                                        0, bConfigRead, 0)
    if Res == 0:
        Device.isOpen = False
        raise ConnectionError("Problem (1):Unable to connect to device")
    # used to determine if a driver object is valid.
    if neovi_dll.icsneoValidateHObject(Device.hObject) == 0:
        raise ConnectionError("Problem (2):connection to the device is not valid")
    Device.isOpen = True
    return "".join(["Neo Device #", str(Device.SerialNumber), " - Port Opened OK!"])


def neovi_close_can_device(Device: NeoviDeviceFoundStructure) -> list:
    """
    Function to close open device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: list of errors.
    """
    result = []

    pNumberOfErrors = c_int(0)
    res = neovi_dll.icsneoClosePort(Device.hObject, byref(pNumberOfErrors))

    if res == 1 or pNumberOfErrors.value > 0:
        return _neovi_get_error_information(Device)

    Device.hObject = c_int(0)
    Device.isOpen = False

    # This method releases system resources used by the neoVI device.
    neovi_dll.icsneoFreeObject(Device.hObject)
    return result


def _get_fire_device_configuration(Device: NeoviDeviceFoundStructure) -> None:
    """
    Internal: ConnectionError This method reads the configuration settings from a neoVI Fire device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: None
    :raises ConnectionError: connectivity problem with the connected device.
    :raises TypeError: device selected is not NeoFire.
    :raises CanbusGetMessageError: if operation failed
    """

    # check we're dealing with NeoFire device type:
    if Device.NeoviDevice.DeviceType != device_types['NEODEVICE_FIRE'] and \
            Device.NeoviDevice.DeviceType != device_types['NEODEVICE_RED']:
        raise TypeError("Problem(7.1) wrong type of Neo device: " + str(Device.NeoviDevice.DeviceType))

    if not Device.isOpen:
        raise ConnectionError("Problem (6): the requested device is closed, can't work with it.")

    Settings = SFireSettings()
    iNumBytes = c_int(sizeof(Settings))
    res = neovi_dll.icsneoGetFireSettings(Device.hObject,
                                          byref(Settings),
                                          iNumBytes)
    if res == 0:
        error_list = _neovi_get_error_information(Device=Device)
        raise CanbusGetMessageError("".join(error_list))

    Device.Settings = Settings


def neovi_get_fire_device_configuration(Device: NeoviDeviceFoundStructure) -> dict:
    """
    This method reads the configuration settings from a neoVI Fire device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: dict contains the configuration of the device
    :raises ConnectionError: connectivity problem with the connected device.
    :raises TypeError: device selected is not NeoFire.
    """

    if not Device.isOpen:
        raise ConnectionError("Problem (6): the requested device is closed, can't work with it.")

    _get_fire_device_configuration(Device=Device)

    return {
        "can1": {
            "Mode": str(Device.Settings.can1.Mode),
            "SetBaudrate": str(Device.Settings.can1.SetBaudrate),
            "Baudrate": str(Device.Settings.can1.Baudrate),
            "Transceiver_Mode": str(Device.Settings.can1.Transceiver_Mode),
            "TqSeg1": str(Device.Settings.can1.TqSeg1),
            "TqSeg2": str(Device.Settings.can1.TqSeg2),
            "TqProp": str(Device.Settings.can1.TqProp),
            "TqSync": str(Device.Settings.can1.TqSync),
            "BRP": str(Device.Settings.can1.BRP),
            "auto_baud": str(Device.Settings.can1.auto_baud),
            "innerFrameDelay25us": str(Device.Settings.can1.innerFrameDelay25us)
        },
        "can2": {
            "Mode": str(Device.Settings.can2.Mode),
            "SetBaudrate": str(Device.Settings.can2.SetBaudrate),
            "Baudrate": str(Device.Settings.can2.Baudrate),
            "Transceiver_Mode": str(Device.Settings.can2.Transceiver_Mode),
            "TqSeg1": str(Device.Settings.can2.TqSeg1),
            "TqSeg2": str(Device.Settings.can2.TqSeg2),
            "TqProp": str(Device.Settings.can2.TqProp),
            "TqSync": str(Device.Settings.can2.TqSync),
            "BRP": str(Device.Settings.can2.BRP),
            "auto_baud": str(Device.Settings.can2.auto_baud),
            "innerFrameDelay25us": str(Device.Settings.can2.innerFrameDelay25us)
        },
        "can3": {
            "Mode": str(Device.Settings.can3.Mode),
            "SetBaudrate": str(Device.Settings.can3.SetBaudrate),
            "Baudrate": str(Device.Settings.can3.Baudrate),
            "Transceiver_Mode": str(Device.Settings.can3.Transceiver_Mode),
            "TqSeg1": str(Device.Settings.can3.TqSeg1),
            "TqSeg2": str(Device.Settings.can3.TqSeg2),
            "TqProp": str(Device.Settings.can3.TqProp),
            "TqSync": str(Device.Settings.can3.TqSync),
            "BRP": str(Device.Settings.can3.BRP),
            "auto_baud": str(Device.Settings.can3.auto_baud),
            "innerFrameDelay25us": str(Device.Settings.can3.innerFrameDelay25us)
        },
        "can4": {
            "Mode": str(Device.Settings.can4.Mode),
            "SetBaudrate": str(Device.Settings.can4.SetBaudrate),
            "Baudrate": str(Device.Settings.can4.Baudrate),
            "Transceiver_Mode": str(Device.Settings.can4.Transceiver_Mode),
            "TqSeg1": str(Device.Settings.can4.TqSeg1),
            "TqSeg2": str(Device.Settings.can4.TqSeg2),
            "TqProp": str(Device.Settings.can4.TqProp),
            "TqSync": str(Device.Settings.can4.TqSync),
            "BRP": str(Device.Settings.can4.BRP),
            "auto_baud": str(Device.Settings.can4.auto_baud),
            "innerFrameDelay25us": str(Device.Settings.can4.innerFrameDelay25us)
        },
        "swcan": {
            "Mode": str(Device.Settings.swcan.Mode),
            "SetBaudrate": str(Device.Settings.swcan.SetBaudrate),
            "Baudrate": str(Device.Settings.swcan.Baudrate),
            "NetworkType": str(Device.Settings.swcan.NetworkType),
            "TqSeg1": str(Device.Settings.swcan.TqSeg1),
            "TqSeg2": str(Device.Settings.swcan.TqSeg2),
            "TqProp": str(Device.Settings.swcan.TqProp),
            "TqSync": str(Device.Settings.swcan.TqSync),
            "BRP": str(Device.Settings.swcan.BRP),
            "high_speed_auto_switch": str(Device.Settings.swcan.high_speed_auto_switch),
            "auto_baud": str(Device.Settings.swcan.auto_baud)
        },
        "lsftcan": {
            "Mode": str(Device.Settings.lsftcan.Mode),
            "SetBaudrate": str(Device.Settings.lsftcan.SetBaudrate),
            "Baudrate": str(Device.Settings.lsftcan.Baudrate),
            "Transceiver_Mode": str(Device.Settings.lsftcan.Transceiver_Mode),
            "TqSeg1": str(Device.Settings.lsftcan.TqSeg1),
            "TqSeg2": str(Device.Settings.lsftcan.TqSeg2),
            "TqProp": str(Device.Settings.lsftcan.TqProp),
            "TqSync": str(Device.Settings.lsftcan.TqSync),
            "BRP": str(Device.Settings.lsftcan.BRP),
            "auto_baud": str(Device.Settings.lsftcan.auto_baud),
            "innerFrameDelay25us": str(Device.Settings.lsftcan.innerFrameDelay25us)
        },
        "lin1": {
            "Baudrate": str(Device.Settings.lin1.Baudrate),
            "spbrg": str(Device.Settings.lin1.spbrg),
            "brgh": str(Device.Settings.lin1.brgh),
            "NumBitsDelay": str(Device.Settings.lin1.NumBitsDelay),
            "MasterResistor": str(Device.Settings.lin1.MasterResistor),
            "Mode": str(Device.Settings.lin1.Mode)
        },
        "lin2": {
            "Baudrate": str(Device.Settings.lin2.Baudrate),
            "spbrg": str(Device.Settings.lin2.spbrg),
            "brgh": str(Device.Settings.lin2.brgh),
            "NumBitsDelay": str(Device.Settings.lin2.NumBitsDelay),
            "MasterResistor": str(Device.Settings.lin2.MasterResistor),
            "Mode": str(Device.Settings.lin2.Mode)
        },
        "lin3": {
            "Baudrate": str(Device.Settings.lin3.Baudrate),
            "spbrg": str(Device.Settings.lin3.spbrg),
            "brgh": str(Device.Settings.lin3.brgh),
            "NumBitsDelay": str(Device.Settings.lin3.NumBitsDelay),
            "MasterResistor": str(Device.Settings.lin3.MasterResistor),
            "Mode": str(Device.Settings.lin3.Mode)
        },
        "lin4": {
            "Baudrate": str(Device.Settings.lin4.Baudrate),
            "spbrg": str(Device.Settings.lin4.spbrg),
            "brgh": str(Device.Settings.lin4.brgh),
            "NumBitsDelay": str(Device.Settings.lin4.NumBitsDelay),
            "MasterResistor": str(Device.Settings.lin4.MasterResistor),
            "Mode": str(Device.Settings.lin4.Mode)
        },
        "cgi_enable_reserved": str(Device.Settings.cgi_enable_reserved),
        "cgi_baud": str(Device.Settings.cgi_baud),
        "cgi_tx_ifs_bit_times": str(Device.Settings.cgi_tx_ifs_bit_times),
        "cgi_rx_ifs_bit_times": str(Device.Settings.cgi_rx_ifs_bit_times),
        "cgi_chksum_enable": str(Device.Settings.cgi_chksum_enable),
        "network_enables": str(Device.Settings.network_enables),
        "network_enabled_on_boot": str(Device.Settings.network_enabled_on_boot),
        "pwm_man_timeout": str(Device.Settings.pwm_man_timeout),
        "pwr_man_enable": str(Device.Settings.pwr_man_enable),
        "misc_io_initial_ddr": str(Device.Settings.misc_io_initial_ddr),
        "misc_io_initial_latch": str(Device.Settings.misc_io_initial_latch),
        "misc_io_analog_enable": str(Device.Settings.misc_io_analog_enable),
        "misc_io_report_period": str(Device.Settings.misc_io_report_period),
        "misc_io_on_report_events": str(Device.Settings.misc_io_on_report_events),
        "ain_sample_period": str(Device.Settings.ain_sample_period),
        "ain_threshold": str(Device.Settings.ain_threshold),
        "iso15765_separation_time_offset": str(Device.Settings.iso15765_separation_time_offset),
        "iso9141_kwp_enable_reserved": str(Device.Settings.iso9141_kwp_enable_reserved),
        "iso9141_kwp_settings": {
            "Baudrate": str(Device.Settings.iso9141_kwp_settings.Baudrate),
            "spbrg": str(Device.Settings.iso9141_kwp_settings.spbrg),
            "brgh": str(Device.Settings.iso9141_kwp_settings.brgh),
            "init_steps": [{
                "time_500us": Device.Settings.iso9141_kwp_settings.init_steps[index].time_500us,
                "k": Device.Settings.iso9141_kwp_settings.init_steps[index].k,
                "l": Device.Settings.iso9141_kwp_settings.init_steps[index].l
            } for index in range(16)],
            "init_step_count": str(Device.Settings.iso9141_kwp_settings.init_step_count),
            "p2_500us": str(Device.Settings.iso9141_kwp_settings.p2_500us),
            "p3_500us": str(Device.Settings.iso9141_kwp_settings.p3_500us),
            "p4_500us": str(Device.Settings.iso9141_kwp_settings.p4_500us),
            "chksum_enabled": str(Device.Settings.iso9141_kwp_settings.chksum_enabled)
        },
        "perf_en": str(Device.Settings.perf_en),
        "iso_parity": str(Device.Settings.iso_parity),
        "iso_msg_termination": str(Device.Settings.iso_msg_termination),
        "iso_tester_pullup_enable": str(Device.Settings.iso_tester_pullup_enable),
        "network_enables_2": str(Device.Settings.network_enables_2),
        "iso9141_kwp_settings2": {
            "Baudrate": str(Device.Settings.iso9141_kwp_settings2.Baudrate),
            "spbrg": str(Device.Settings.iso9141_kwp_settings2.spbrg),
            "brgh": str(Device.Settings.iso9141_kwp_settings2.brgh),
            "init_steps": [{
                "time_500us": Device.Settings.iso9141_kwp_settings2.init_steps[index].time_500us,
                "k": Device.Settings.iso9141_kwp_settings2.init_steps[index].k,
                "l": Device.Settings.iso9141_kwp_settings2.init_steps[index].l
            } for index in range(16)],
            "init_step_count": str(Device.Settings.iso9141_kwp_settings2.init_step_count),
            "p2_500us": str(Device.Settings.iso9141_kwp_settings2.p2_500us),
            "p3_500us": str(Device.Settings.iso9141_kwp_settings2.p3_500us),
            "p4_500us": str(Device.Settings.iso9141_kwp_settings2.p4_500us),
            "chksum_enabled": str(Device.Settings.iso9141_kwp_settings2.chksum_enabled)
        },
        "iso_parity_2": str(Device.Settings.iso_parity_2),
        "iso_msg_termination_2": str(Device.Settings.iso_msg_termination_2),
        "iso9141_kwp_settings_3": {
            "Baudrate": str(Device.Settings.iso9141_kwp_settings_3.Baudrate),
            "spbrg": str(Device.Settings.iso9141_kwp_settings_3.spbrg),
            "brgh": str(Device.Settings.iso9141_kwp_settings_3.brgh),
            "init_steps": [{
                "time_500us": Device.Settings.iso9141_kwp_settings_3.init_steps[index].time_500us,
                "k": Device.Settings.iso9141_kwp_settings_3.init_steps[index].k,
                "l": Device.Settings.iso9141_kwp_settings_3.init_steps[index].l
            } for index in range(16)],
            "init_step_count": str(Device.Settings.iso9141_kwp_settings_3.init_step_count),
            "p2_500us": str(Device.Settings.iso9141_kwp_settings_3.p2_500us),
            "p3_500us": str(Device.Settings.iso9141_kwp_settings_3.p3_500us),
            "p4_500us": str(Device.Settings.iso9141_kwp_settings_3.p4_500us),
            "chksum_enabled": str(Device.Settings.iso9141_kwp_settings_3.chksum_enabled)
        },
        "iso_parity_3": str(Device.Settings.iso_parity_3),
        "iso_msg_termination_3": str(Device.Settings.iso_msg_termination_3),
        "iso9141_kwp_settings_4": {
            "Baudrate": str(Device.Settings.iso9141_kwp_settings_4.Baudrate),
            "spbrg": str(Device.Settings.iso9141_kwp_settings_4.spbrg),
            "brgh": str(Device.Settings.iso9141_kwp_settings_4.brgh),
            "init_steps": [{
                "time_500us": Device.Settings.iso9141_kwp_settings_4.init_steps[index].time_500us,
                "k": Device.Settings.iso9141_kwp_settings_4.init_steps[index].k,
                "l": Device.Settings.iso9141_kwp_settings_4.init_steps[index].l
            } for index in range(16)],
            "init_step_count": str(Device.Settings.iso9141_kwp_settings_4.init_step_count),
            "p2_500us": str(Device.Settings.iso9141_kwp_settings_4.p2_500us),
            "p3_500us": str(Device.Settings.iso9141_kwp_settings_4.p3_500us),
            "p4_500us": str(Device.Settings.iso9141_kwp_settings_4.p4_500us),
            "chksum_enabled": str(Device.Settings.iso9141_kwp_settings_4.chksum_enabled),
        },
        "iso_parity_4": str(Device.Settings.iso_parity_4),
        "iso_msg_termination_4": str(Device.Settings.iso_msg_termination_4),
        "fast_init_network_enables_1": str(Device.Settings.fast_init_network_enables_1),
        "fast_init_network_enables_2": str(Device.Settings.fast_init_network_enables_2),
        "uart": {
            "Baudrate": str(Device.Settings.uart.Baudrate),
            "spbrg": str(Device.Settings.uart.spbrg),
            "brgh": str(Device.Settings.uart.brgh),
            "parity": str(Device.Settings.uart.parity),
            "stop_bits": str(Device.Settings.uart.stop_bits),
            "flow_control": str(Device.Settings.uart.flow_control),
            "reserved_1": str(Device.Settings.uart.reserved_1),
            "bOptions": str(Device.Settings.uart.bOptions)
        },
        "uart2": {
            "Baudrate": str(Device.Settings.uart2.Baudrate),
            "spbrg": str(Device.Settings.uart2.spbrg),
            "brgh": str(Device.Settings.uart2.brgh),
            "parity": str(Device.Settings.uart2.parity),
            "stop_bits": str(Device.Settings.uart2.stop_bits),
            "flow_control": str(Device.Settings.uart2.flow_control),
            "reserved_1": str(Device.Settings.uart2.reserved_1),
            "bOptions": str(Device.Settings.uart2.bOptions)
        },
        "text_api": {
            "can1_tx_id": str(Device.Settings.text_api.can1_tx_id),
            "can1_rx_id": str(Device.Settings.text_api.can1_rx_id),
            "can1_options": str(Device.Settings.text_api.can1_options),
            "can2_tx_id": str(Device.Settings.text_api.can2_tx_id),
            "can2_rx_id": str(Device.Settings.text_api.can2_rx_id),
            "can2_options": str(Device.Settings.text_api.can2_options),
            "network_enables": str(Device.Settings.text_api.network_enables),
            "can3_tx_id3": str(Device.Settings.text_api.can3_tx_id3),
            "can3_rx_id3": str(Device.Settings.text_api.can3_rx_id3),
            "can3_options": str(Device.Settings.text_api.can3_options),
            "can4_tx_id4": str(Device.Settings.text_api.can4_tx_id4),
            "can4_rx_id4": str(Device.Settings.text_api.can4_rx_id4),
            "can4_options": str(Device.Settings.text_api.can4_options),
            "Reserved0": str(Device.Settings.text_api.Reserved0),
            "Reserved1": str(Device.Settings.text_api.Reserved1),
            "Reserved2": str(Device.Settings.text_api.Reserved2),
            "Reserved3": str(Device.Settings.text_api.Reserved3),
            "Reserved4": str(Device.Settings.text_api.Reserved4)
        }
    }


def _set_fire_device_configuration(Device: NeoviDeviceFoundStructure) -> str:
    """
    (internal) This method writes configuration settings to a neoVI Fire device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: string represent the execution status.
    :raises ConnectionError: connectivity problem with the connected device.
    :raises TypeError: device selected is not NeoFire.
    """
    # check we're dealing with NeoFire device type:
    if Device.NeoviDevice.DeviceType != device_types['NEODEVICE_FIRE'] and \
            Device.NeoviDevice.DeviceType != device_types['NEODEVICE_RED']:
        raise TypeError("Problem(7.1) wrong type of Neo device: " + str(Device.NeoviDevice.DeviceType))

    if not Device.isOpen:
        raise ConnectionError("Problem (7): the requested device is closed, can't work with it.")

    iNumBytes = c_int(sizeof(Device.Settings))  # [in] This value is always the size, in bytes, of the SFireSettings
    # structure.
    disable_network = 0
    enable_network = 1

    # This function disable network traffic for all client applications connected to the neoVI.
    neovi_dll.icsneoEnableNetworkCom(Device.hObject, disable_network)

    bSaveToEEPROM = c_int(0)  # [in] If set to 0, the settings changes will revert to the values stored in
    # EEPROM when the neoVI is power-cycled. If set to 1, the values will overwrite
    # the EEPROM settings and become persistent across power-cycles of the neoVI.
    res = neovi_dll.icsneoSetFireSettings(Device.hObject,
                                          byref(Device.Settings),
                                          iNumBytes, bSaveToEEPROM)

    time.sleep(0.01)  # 10 milliseconds delay
    # This function enable network traffic for all client applications connected to the neoVI.
    neovi_dll.icsneoEnableNetworkCom(Device.hObject, enable_network)

    if res:
        return "Configuration done successfully"
    else:
        error_reason = _neovi_get_error_information(Device=Device)
        return "problem: " + "".join(error_reason)


def neovi_set_fire_can1_settings_SetBaudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
    This method writes SetBaudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param configuration_value: value to set.
    :returns: string represent the execution status.
    :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.SetBaudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_Baudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
    This method writes Baudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param configuration_value: value to set.
    :returns: string represent the execution status.
    :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.Baudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_Transceiver_Mode(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
    This method writes Transceiver_Mode field settings to a neoVI Fire device with CAN_SETTINGS structure

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param configuration_value: value to set.
    :returns: string represent the execution status.
    :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.Transceiver_Mode = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_TqSeg1(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
    This method writes TqSeg1 field settings to a neoVI Fire device with CAN_SETTINGS structure

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param configuration_value: value to set.
    :returns: string represent the execution status.
    :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.TqSeg1 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_TqSeg2(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSeg2 field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.TqSeg2 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_TqProp(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqProp field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.TqProp = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_TqSync(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSync field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.TqSync = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_BRP(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes BRP field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.BRP = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_auto_baud(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes auto_baud field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.auto_baud = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can1_settings_innerFrameDelay25us(Device: NeoviDeviceFoundStructure,
                                                     configuration_value: int) -> str:
    """
        This method writes innerFrameDelay25us field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can1.innerFrameDelay25us = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_SetBaudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes SetBaudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.SetBaudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_Baudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes Baudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.Baudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_Transceiver_Mode(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes Transceiver_Mode field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.Transceiver_Mode = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_TqSeg1(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSeg1 field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.TqSeg1 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_TqSeg2(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSeg2 field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.TqSeg2 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_TqProp(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqProp field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.TqProp = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_TqSync(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSync field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.TqSync = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_BRP(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes BRP field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.BRP = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_auto_baud(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes auto_baud field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.auto_baud = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can2_settings_innerFrameDelay25us(Device: NeoviDeviceFoundStructure,
                                                     configuration_value: int) -> str:
    """
        This method writes innerFrameDelay25us field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can2.innerFrameDelay25us = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_SetBaudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes SetBaudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.SetBaudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_Baudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes Baudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.Baudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_Transceiver_Mode(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes Transceiver_Mode field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.Transceiver_Mode = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_TqSeg1(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSeg1 field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.TqSeg1 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_TqSeg2(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSeg2 field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.TqSeg2 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_TqProp(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqProp field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.TqProp = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_TqSync(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSync field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.TqSync = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_BRP(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes BRP field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.BRP = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_auto_baud(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes auto_baud field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.auto_baud = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can3_settings_innerFrameDelay25us(Device: NeoviDeviceFoundStructure,
                                                     configuration_value: int) -> str:
    """
        This method writes innerFrameDelay25us field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can3.innerFrameDelay25us = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_SetBaudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes SetBaudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.SetBaudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_Baudrate(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes Baudrate field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.Baudrate = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_Transceiver_Mode(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes Transceiver_Mode field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.Transceiver_Mode = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_TqSeg1(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSeg1 field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.TqSeg1 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_TqSeg2(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSeg2 field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.TqSeg2 = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_TqProp(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqProp field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.TqProp = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_TqSync(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes TqSync field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.TqSync = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_BRP(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes BRP field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.BRP = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_auto_baud(Device: NeoviDeviceFoundStructure, configuration_value: int) -> str:
    """
        This method writes auto_baud field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.auto_baud = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_set_fire_can4_settings_innerFrameDelay25us(Device: NeoviDeviceFoundStructure,
                                                     configuration_value: int) -> str:
    """
        This method writes innerFrameDelay25us field settings to a neoVI Fire device with CAN_SETTINGS structure

        :param Device: Specifies the driver object created by OpenNeoDevice.
        :param configuration_value: value to set.
        :returns: string represent the execution status.
        :raises TypeError: configuration string not supported.
    """

    # get current configuration from device before changing configuration
    _get_fire_device_configuration(Device=Device)

    try:
        Device.Settings.can4.innerFrameDelay25us = c_ubyte(configuration_value)
    except IndexError:
        raise IndexError("field selected or value not supported")

    return _set_fire_device_configuration(Device=Device)


def neovi_get_rtc(Device: NeoviDeviceFoundStructure) -> dict:
    """
    This method returns the value of the real-time clock on a connected neoVI device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: dict contains the rtc data or empty dict.
    :raises ConnectionError: connectivity problem with the connected device.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (8): the requested device is closed, can't work with it.")

    pTime = icsSpyTime()
    if neovi_dll.icsneoGetRTC(Device.hObject, byref(pTime)):
        return {
            "sec": pTime.sec,
            "min": pTime.min,
            "hour": pTime.hour,
            "day": pTime.day,
            "month": pTime.month,
            "year": pTime.year
        }
    else:
        return {}


def neovi_set_rtc(Device: NeoviDeviceFoundStructure, sec: int = 0, min: int = 0, hour=0, day=0, month=0,
                  year=0):
    """
    This method sets the value of the real-time clock on a connected neoVI device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param sec: seconds
    :param min: minute
    :param hour: hour (24h clock)
    :param day: day of the month.
    :param month: month of the year
    :param year: year 4 digits.
    :returns: string represent the execution status.
    :raises ConnectionError: connectivity problem with the connected device.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (9): the requested device is closed, can't work with it.")

    pTime = icsSpyTime()
    pTime.sec = c_byte(sec)
    pTime.min = c_byte(min)
    pTime.hour = c_byte(hour)
    pTime.day = c_byte(day)
    pTime.month = c_byte(month)
    pTime.year = c_byte(year)
    if neovi_dll.icsneoSetRTC(Device.hObject, byref(pTime)):
        return "".join(["Successfully set RTC to: ",
                        str({"sec": pTime.sec,
                             "min": pTime.min,
                             "hour": pTime.hour,
                             "day": pTime.day,
                             "month": pTime.month,
                             "year": pTime.year}
                            )
                        ])
    else:
        return ""


def neovi_get_hw_firmware_info(Device: NeoviDeviceFoundStructure) -> dict:
    """
    This method returns the firmware version and information of the open neoVI device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: dict contains the firmware data.
    :raises ConnectionError: connectivity problem with the connected device.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (10): the requested device is closed, can't work with it.")

    FirmwareInfo = stAPIFirmwareInfo * 1
    info = FirmwareInfo()
    res = neovi_dll.icsneoGetHWFirmwareInfo(Device.hObject, byref(info))
    if res == 0:
        _handle_error(Device)

    return {
        "iType": info[0].iType,
        "iMainFirmDateDay": info[0].iMainFirmDateDay,
        "iMainFirmDateMonth": info[0].iMainFirmDateMonth,
        "iMainFirmDateYear": info[0].iMainFirmDateYear,
        "iMainFirmDateHour": info[0].iMainFirmDateHour,
        "iMainFirmDateMin": info[0].iMainFirmDateMin,
        "iMainFirmDateSecond": info[0].iMainFirmDateSecond,
        "iMainFirmChkSum": info[0].iMainFirmChkSum,
        "iAppMajor": info[0].iAppMajor,
        "iAppMinor": info[0].iAppMinor,
        "iManufactureDay": info[0].iManufactureDay,
        "iManufactureMonth": info[0].iManufactureMonth,
        "iManufactureYear": info[0].iManufactureYear,
        "iBootLoaderVersionMajor": info[0].iBootLoaderVersionMajor,
        "iBootLoaderVersionMinor": info[0].iBootLoaderVersionMinor,
        "iMainVnetHWrevMajor": info[0].iMainVnetHWrevMajor,
        "iMainVnetHWrevMinor": info[0].iMainVnetHWrevMinor,
        "iMainVnetSRAMSize": info[0].iMainVnetSRAMSize
    }


def neovi_get_dll_firmware_info(Device: NeoviDeviceFoundStructure) -> dict:
    """
    This method returns the firmware version stored within the DLL API.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: dict contains the firmware data.
    """

    FirmwareInfo = stAPIFirmwareInfo * 1
    info = FirmwareInfo()
    res = neovi_dll.icsneoGetDLLFirmwareInfo(Device.hObject, byref(info))
    if res == 0:
        _handle_error(Device)

    return {
        "iType": info[0].iType,
        "iMainFirmDateDay": info[0].iMainFirmDateDay,
        "iMainFirmDateMonth": info[0].iMainFirmDateMonth,
        "iMainFirmDateYear": info[0].iMainFirmDateYear,
        "iMainFirmDateHour": info[0].iMainFirmDateHour,
        "iMainFirmDateMin": info[0].iMainFirmDateMin,
        "iMainFirmDateSecond": info[0].iMainFirmDateSecond,
        "iMainFirmChkSum": info[0].iMainFirmChkSum,
        "iAppMajor": info[0].iAppMajor,
        "iAppMinor": info[0].iAppMinor,
        "iManufactureDay": info[0].iManufactureDay,
        "iManufactureMonth": info[0].iManufactureMonth,
        "iManufactureYear": info[0].iManufactureYear,
        "iBootLoaderVersionMajor": info[0].iBootLoaderVersionMajor,
        "iBootLoaderVersionMinor": info[0].iBootLoaderVersionMinor,
        "iMainVnetHWrevMajor": info[0].iMainVnetHWrevMajor,
        "iMainVnetHWrevMinor": info[0].iMainVnetHWrevMinor,
        "iMainVnetSRAMSize": info[0].iMainVnetSRAMSize
    }


def neovi_force_firmware_update(Device: NeoviDeviceFoundStructure) -> str:
    """
    Forces the firmware to updated on a neoVI device

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :returns: string of the execution status.
    :raises ConnectionError: connectivity problem with the connected device.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (11): the requested device is closed, can't work with it.")

    neovi_dll.icsneoForceFirmwareUpdate(Device.hObject)
    return "Finished Firmware Update"


def neovi_set_bit_rate(Device: NeoviDeviceFoundStructure, BitRate: int, NetworkID: int) -> str:
    """
    This method sets bit rates for networks on neoVI devices

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param BitRate: Specifies bit rate setting (depends of device type)
    :param NetworkID: Specifies the network.
    :returns: string of the execution status.
    :raises ConnectionError: connectivity problem with the connected device.
    """
    iBitRate = c_int32(BitRate)
    iNetworkID = c_int32(NetworkID)
    piErrorNumber = c_int32(0)

    if not Device.isOpen:
        raise ConnectionError("Problem (12): the requested device is closed, can't work with it.")

    res = neovi_dll.icsneoSetBitRate(Device.hObject, iBitRate, iNetworkID)
    if res:
        return "Bit rate " + str(BitRate) + " at network " + str(NetworkID) + \
               " (" + str(list(filter(lambda x: x[1] == NetworkID, network_id.items()))[0][0]) + ")" \
               + " was set successfully"
    else:
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            return "Problem (" + str(piErrorNumber) + ") while trying to set bit rate " + str(
                BitRate) + " and network ID: " + str(NetworkID) + \
                   " (" + str(list(filter(lambda x: x[1] == NetworkID, network_id.items()))[0][0]) + ")"
        else:
            raise ConnectionError("Problem unable to fetch error number: " + str(piErrorNumber))


def _MessageProcessing_GetTimeStamp(hObject: NeoviDeviceFoundStructure.hObject,
                                    message: icsSpyMessageRX) -> str:
    """
    (internal) This method calculates the timestamp for a message, based on the connected hardware type,
    and converts it to a usable variable.

    :param hObject: pointer to the device handler
    :param message: message received on the bus.
    :return: string of timestamp.
    """
    pTimeStamp = c_longdouble(0)
    if neovi_dll.icsneoGetTimeStampForMsg(hObject, byref(message), byref(pTimeStamp)):
        time_t = datetime.timedelta(seconds=pTimeStamp.value)
        date2007 = datetime.datetime(2007, 1, 1)
        date1970 = datetime.datetime(1970, 1, 1)
        timestamp = (date2007 + time_t) - date1970
        return str(int(timestamp.total_seconds() * 1000))
    #  TODO: handle error using GetLastAPIError


def _MessageProcessing_GetMsgDirection(message: icsSpyMessageRX) -> str:
    """
    (internal) function to pull the message direction (Tx/Rx).

    :param message: message returned from the device.
    :return: string of the direction (Tx/Rx)
    """
    if message.StatusBitField & StatusBitfield['SPY_STATUS_TX_MSG']:
        return "Tx"
    else:
        return "Rx"


def _MessageProcessing_GetNetwork(message: icsSpyMessageRX) -> str:
    """
    (internal) function to pull the message network.

    :param message: message returned from the device.
    :return: string of the network.
    """
    Switcher = {
        0: "DEVICE",
        1: "HSCAN",
        2: "MSCAN",
        3: "SWCAN",
        4: "LSFTCAN",
        9: "ISO",
        14: "ISO2",
        15: "ISO14230",
        16: "LIN",
        17: "OP_ETHERNET1",
        18: "OP_ETHERNET2",
        19: "OP_ETHERNET3",
        41: "ISO3",
        42: "HSCAN2",
        44: "HSCAN3",
        45: "OP_ETHERNET4",
        46: "OP_ETHERNET5",
        47: "ISO4",
        48: "LIN2",
        49: "LIN3",
        50: "LIN4",
        51: "MOST",
        53: "CGI",
        61: "HSCAN4",
        62: "HSCAN5",
        64: "UART",
        65: "UART2",
        66: "UART3",
        67: "UART4",
        68: "SWCAN2",
        69: "ETHERNET_DAQ",
        73: "OP_ETHERNET6",
        75: "OP_ETHERNET7",
        76: "OP_ETHERNET8",
        77: "OP_ETHERNET9",
        78: "OP_ETHERNET10",
        79: "OP_ETHERNET11",
        80: "FLEXRAY1A",
        81: "FLEXRAY1B",
        82: "FLEXRAY2A",
        83: "FLEXRAY2B",
        84: "LIN5",
        85: "FLEXRAY",
        86: "FLEXRAY2",
        87: "OP_ETHERNET12",
        90: "MOST25",
        91: "MOST50",
        92: "MOST150",
        93: "ETHERNET",
        96: "HSCAN6",
        97: "HSCAN7",
        98: "LIN6",
        99: "LSFTCAN2"
    }
    return Switcher.get(message.NetworkID, "Unknown")


def _MessageProcessing_GetExtendedId(message: icsSpyMessageRX) -> bool:
    """
    (internal) function to pull the if the arbitration header is 29 bits or 11 bits.

    :param message: message returned from the device.
    :return: True means 19 bits.
    """
    if message.StatusBitField & StatusBitfield['SPY_STATUS_XTD_FRAME']:
        return True
    else:
        return False


def _MessageProcessing_ArbitrationId(message: icsSpyMessageRX) -> str:
    """
    (internal) function to pull the arbitration id.

    :param message: message returned from the device.
    :return: string of the arbitration ID.
    :raise KeyError: if the protocol in the message is not supported.
    """
    try:
        if message.Protocol == protocol_val["SPY_PROTOCOL_CAN"]:  # CAN 2.0B
            #  SPY_STATUS_XTD_FRAME indicate if the ArbID is 11 bits or 29 bits.
            return str(hex(message.ArbIDOrHeader)).replace("0x", "")
        elif message.Protocol in (protocol_val["SPY_PROTOCOL_Keyword2000"], protocol_val["SPY_PROTOCOL_ISO9141"]):
            # reformat the struct according to K-line struct.
            header_val = {
                '1': f'{str(hex(message.ArbIDOrHeader & 0xFF)).replace("0x", ""):0>2}',
                '2': " ".join([f'{str(hex(message.ArbIDOrHeader & 0xFF)).replace("0x", ""):0>2}',
                               f'{str(hex((message.ArbIDOrHeader & 0xFF00) >> 8)).replace("0x", ""):0>2}']),
                '3': " ".join([f'{str(hex(message.ArbIDOrHeader & 0xFF)).replace("0x", ""):0>2}',
                               f'{str(hex((message.ArbIDOrHeader & 0xFF00) >> 8)).replace("0x", ""):0>2}',
                               f'{str(hex((message.ArbIDOrHeader & 0xFF0000) >> 16)).replace("0x", ""):0>2}'])
            }.get(str(message.NumberBytesHeader), "")

            return header_val
        elif message.Protocol == protocol_val["SPY_PROTOCOL_LIN"]:  # LIN
            return str(message.ArbIDOrHeader & 0xFF).replace("0x", "")
        elif message.Protocol == protocol_val["SPY_PROTOCOL_CANFD"]:  # CAN FD
            return str(hex(message.ArbIDOrHeader)).replace("0x", "")
        elif message.Protocol == protocol_val["SPY_PROTOCOL_ETHERNET"]:
            return "0"
        else:
            return "0"
    except KeyError:
        return "0"


def _MessageProcessing_Data(message: icsSpyMessageRX) -> str:
    """
        (internal) function to pull the message data.

        :param message: message returned from the device.
        :return: string of the message data.
        :raise KeyError: if the protocol in the message is not supported.
        """
    try:

        if message.Protocol in (protocol_val["SPY_PROTOCOL_CAN"],
                                protocol_val["SPY_PROTOCOL_Keyword2000"],
                                protocol_val["SPY_PROTOCOL_ISO9141"]):  # CAN 2.0B

            return " & ".join([str(hex(message.StatusBitField)).replace('0x', ''),
                               str(hex(message.StatusBitField2)).replace('0x', ''),
                               str(hex(message.StatusBitField3)).replace('0x', ''),
                               str(hex(message.StatusBitField4)).replace('0x', ''),
                               ])
        elif message.Protocol == protocol_val["SPY_PROTOCOL_LIN"]:  # LIN
            tReturn = ""
            if message.NumberBytesHeader > 1:
                tReturn = tReturn + str((message.ArbIDOrHeader & 0xFF00) / 0x100)
            if message.NumberBytesHeader > 2:
                tReturn = tReturn + str((message.ArbIDOrHeader & 0xFF0000) / 0x10000)
            return tReturn + "".join(message.Data[0: message.NumberBytesData])
        elif message.Protocol == protocol_val["SPY_PROTOCOL_CANFD"]:  # CAN FD
            if message.ExtraDataPtrEnabled == 0:  # 8 Bytes
                return str(message.Data[0: message.NumberBytesData])
            else:  # >8 Bytes
                return "".join([str(hex(message.iExtraDataPtr[index])) for
                                index in range(message.NumberBytesData)])
        elif message.Protocol == protocol_val["SPY_PROTOCOL_ETHERNET"]:
            eth_total_number_bytes = message.NumberBytesData + (message.NumberBytesHeader * 0x100)
            return "".join([str(hex(message.iExtraDataPtr[index])) for index in range(eth_total_number_bytes)])
        else:
            return "0"
    except KeyError:
        return "0"


def _MessageProcessing_DataBytes(message: icsSpyMessageRX) -> list:
    """
    (internal) function to pull a list of data from the message.

    :param message: message returned from the device.
    :return: list of data bytes.
    """
    return [str(hex(message.Data[index])).replace("0x", "").zfill(2) for index in range(message.NumberBytesData)]


def _MessageProcessing_NumberDataBytes(message: icsSpyMessageRX) -> str:
    """
    (internal) function to measure the number of bytes in the payload (data).

    :param message: message returned from the device.
    :return: string of the number of bytes.
    """
    return str(message.NumberBytesData)


def neovi_change_max_allowed_errors(Device: NeoviDeviceFoundStructure, value: int) -> str:
    """
    Update the MaxAllowedErrors threshold, this threshold is evaluated for every message reading operation, if the
    number of error exceed this value a problem will be generated.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param value: the value to update the threshold.
    :returns: string of the execution status.
    """

    if isinstance(value, int):
        Device._max_allowed_threshold = value
        return "OK!"
    else:
        return "Failure!"


def neovi_get_messages(Device: NeoviDeviceFoundStructure, iso15765=False) -> list:
    """"
    Reads messages from the neoVI device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param iso15765: (optional) flag to bypass ISO15765 mode.
    :returns: list of json formatted messages.
    :raises ConnectionError: connectivity problem with the connected device.
    :raises CanbusGetMessageError: the number of consecutive errors messages exceeded the threshold.
    """
    # check if the device is open before dealing with the request.
    if not Device.isOpen:
        raise ConnectionError("Problem (13): problem can't get messages, make sure device is open")

    # if the ISO15765 mode is active do not perform reading.
    if Device.iso15765_mutex and iso15765 is False:
        return []

    message_buffer = list()

    Msg = icsSpyMessageRX * 20_000
    pMsg = Msg()
    pNumberOfMessages = c_int(0)
    pNumberOfErrors = c_int(0)

    if neovi_dll.icsneoGetMessages(Device.hObject, byref(pMsg),
                                   byref(pNumberOfMessages), byref(pNumberOfErrors)):

        for index in range(pNumberOfMessages.value):
            error_info = ""
            # check for errors
            if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_GLOBAL_ERR']:
                Device.message_error_counter += 1

                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_CRC_ERROR']:
                    error_info += "error with J1850 VPW message which do not have a proper CRC byte; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_INCOMPLETE_FRAME']:
                    error_info += "error with a J1850 VPW message which is received that ended on a non-byte boundary; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_UNDEFINED_ERROR']:
                    error_info += "undefined error in the neoVI hardware; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_CAN_BUS_OFF']:
                    error_info += "there is a change in error status of a MCP2510 CAN controller; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_BAD_MESSAGE_BIT_TIME_ERROR']:
                    error_info += "J1850 VPW messages which do not meet the specified bit times for the SOF or bit " \
                                  "signals; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_CAN_ERROR_WARNING']:
                    error_info += "SPY_STATUS_CAN_ERROR_WARNING; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_CHECKSUM_ERROR']:
                    error_info += "SPY_STATUS_CHECKSUM_ERROR; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_HARDWARE_COMM_ERROR']:
                    error_info += "SPY_STATUS_HARDWARE_COMM_ERROR; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_EXPECTED_LEN_ERROR']:
                    error_info += "SPY_STATUS_EXPECTED_LEN_ERROR; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_INCOMING_NO_MATCH']:
                    error_info += "SPY_STATUS_INCOMING_NO_MATCH; "
                if pMsg[index].StatusBitField & StatusBitfield['SPY_STATUS_AVSI_REC_OVERFLOW']:
                    error_info += "SPY_STATUS_AVSI_REC_OVERFLOW; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_ERROR_FRAME']:
                    error_info += "SPY_STATUS2_ERROR_FRAME; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_ERR_RX_BREAK_NOT_0']:
                    error_info += "SPY_STATUS2_LIN_ERR_RX_BREAK_NOT_0; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_ERR_RX_BREAK_TOO_SHORT']:
                    error_info += "SPY_STATUS2_LIN_ERR_RX_BREAK_TOO_SHORT; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_ERR_RX_SYNC_NOT_55']:
                    error_info += "SPY_STATUS2_LIN_ERR_RX_SYNC_NOT_55; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_ERR_RX_DATA_GREATER_8']:
                    error_info += "SPY_STATUS2_LIN_ERR_RX_DATA_GREATER_8; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_ERR_TX_RX_MISMATCH']:
                    error_info += "SPY_STATUS2_LIN_ERR_TX_RX_MISMATCH; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_ERR_MSG_ID_PARITY']:
                    error_info += "SPY_STATUS2_LIN_ERR_MSG_ID_PARITY; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_ISO_FRAME_ERROR']:
                    error_info += "SPY_STATUS2_ISO_FRAME_ERROR; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_SYNC_FRAME_ERROR']:
                    error_info += "SPY_STATUS2_LIN_SYNC_FRAME_ERROR; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_ISO_OVERFLOW_ERROR']:
                    error_info += "SPY_STATUS2_ISO_OVERFLOW_ERROR; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_ID_FRAME_ERROR']:
                    error_info += "SPY_STATUS2_LIN_ID_FRAME_ERROR; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_ISO_PARITY_ERROR']:
                    error_info += "SPY_STATUS2_ISO_PARITY_ERROR; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_SLAVE_BYTE_ERROR']:
                    error_info += "SPY_STATUS2_LIN_SLAVE_BYTE_ERROR; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_RX_TIMEOUT_ERROR']:
                    error_info += "SPY_STATUS2_RX_TIMEOUT_ERROR; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_LIN_NO_SLAVE_DATA']:
                    error_info += "SPY_STATUS2_LIN_NO_SLAVE_DATA; "
                if pMsg[index].StatusBitField2 & StatusBitfield2['SPY_STATUS2_ETHERNET_CRC_ERROR']:
                    error_info += "SPY_STATUS2_ETHERNET_CRC_ERROR; "

                if len(error_info) == 0:
                    error_info = "general error"
            else:
                Device.message_error_counter = 0
                error_info = "No Error"

            message_buffer.append({
                "unixTime": _MessageProcessing_GetTimeStamp(Device.hObject, pMsg[index]),
                "msgType": _MessageProcessing_GetMsgDirection(pMsg[index]),
                "netType": _MessageProcessing_GetNetwork(pMsg[index]),
                "extId": _MessageProcessing_GetExtendedId(pMsg[index]),
                "arbId": _MessageProcessing_ArbitrationId(pMsg[index]),
                "numDataBytes": _MessageProcessing_NumberDataBytes(pMsg[index]),
                "dataBytes": _MessageProcessing_DataBytes(pMsg[index]),
                "statusMsgHex": _MessageProcessing_Data(pMsg[index]),
                "errInfo": error_info
            })
        if Device.message_error_counter >= Device._max_allowed_threshold:
            raise CanbusGetMessageError("Problem (13.6): " + str(Device.message_error_counter) +
                                        " or more consecutive messages with error in bitfields", message_buffer)

    return message_buffer


def neovi_tx_message_can(Device: NeoviDeviceFoundStructure, network_name: str, extended_arbId: bool,
                         arb_id: str,
                         databytes: list, ) -> str:
    """
    Send message using the device

    :param Device: the device datastructure
    :param network_name: the name of the network to send the message in.
    :param extended_arbId: is the arbitration header is 29 bits or 11 bits.
    :param arb_id: the arbitration id as string.
    :param databytes: list of data bytes.
    :return: execution status as string
    :raise ConnectionError: if the device is closed.
    """
    # check if the device is open before dealing with the request.
    if not Device.isOpen:
        raise ConnectionError("Problem (14): the requested device is closed, can't work with it.")

    msg_buffer = icsSpyMessage * 1
    msg_buffer_array = msg_buffer()

    if extended_arbId:
        msg_buffer_array[0].ExtraDataPtrEnabled = c_ubyte(1)
        msg_buffer_array[0].StatusBitField = StatusBitfield['SPY_STATUS_XTD_FRAME']
    else:
        msg_buffer_array[0].ExtraDataPtrEnabled = c_ubyte(0)

    # Mark this message as Tx
    msg_buffer_array[0].StatusBitField |= StatusBitfield['SPY_STATUS_TX_MSG']

    if network_name == 'ISO':
        msg_buffer_array[0].StatusBitField |= StatusBitfield['SPY_STATUS_INIT_MESSAGE']

    # Holds up to 3 byte 1850 header (bytes 1 through 3) or a 29 bit CAN header.
    msg_buffer_array[0].ArbIDOrHeader = c_int32(int(arb_id, 16))

    for element in range(len(databytes)):
        msg_buffer_array[0].Data[element] = c_ubyte(int(databytes[element], 16))

    # Used for J1850/ISO messages. It indicates how many bytes are stored in the Header(1 to 4) array.
    msg_buffer_array[0].NumberBytesData = 0

    # Holds the number of bytes in the Data(1 to 8) array or the number of bytes in a CAN remote frame (The DLC).
    msg_buffer_array[0].NumberBytesData = len(databytes)

    # Set the Network
    try:
        NetworkID = c_ubyte(network_id["NETID_" + network_name])
    except IndexError:
        NetworkID = c_ubyte(network_id['NETID_HSCAN'])

    Num_of_Msg = c_int(1)

    msg_buffer_array[0].NetworkID = NetworkID
    if neovi_dll.icsneoTxMessages(Device.hObject, byref(msg_buffer_array), NetworkID, Num_of_Msg):
        return "Message transmitted successfully"
    else:
        return "Problem Transmitting Message"


def neovi_iso15765_create_first_message(payload: list, paddingEnabled: bool, paddingByte: str,
                                        numOfBytes: int) -> list:
    """
    Return the first message to send as multi-frame

    :param payload: The payload to use in the message.
    :param paddingEnabled: flag to mark if there is a need to pad the message or not.
    :param paddingByte: the byte to add as padding to the message.
    :param numOfBytes: the total number of the payload bytes for the entire multiframe message.
    :returns: the message to send.
    """
    # creating the first packet
    first_packet_header = hex(numOfBytes & 0x0FFF | (0x1 << 12)).replace("0x", "")
    first_packet = [first_packet_header[0:2], first_packet_header[2:], *payload]
    if paddingEnabled:
        first_packet += [paddingByte] * (8 - len(first_packet))
    return first_packet


def neovi_iso15765_create_trailing_messages(payload: list, paddingEnabled: bool, paddingByte: str) -> list:
    """
    Return a list of all trailing messages to send as multi-frame

    :param payload: The payload to use in the message.
    :param paddingEnabled: flag to mark if there is a need to pad the message or not.
    :param paddingByte: the byte to add as padding to the message.
    :returns: the message to send.
    """
    payload_size = len(payload)

    # create a list of 7 bytes (because we need to add 1 byte header) out of the payload.
    messages_payload = [payload[index: index + 7] for index in range(0, payload_size, 7)]

    if paddingEnabled:
        payload_list = [[hex(((message_index + 1) & 0xF) | 0x20).replace("0x", ""),
                         *messages_payload[message_index]] +
                        [paddingByte] * (8 - (len(messages_payload[message_index]) + 1))
                        for message_index in range(len(messages_payload))]
    else:
        payload_list = [[hex(((message_index + 1) & 0xF) | 0x20).replace("0x", ""), *messages_payload[message_index]]
                        for message_index in range(len(messages_payload))]
    return payload_list


def neovi_iso15765_release_buffer(Device: NeoviDeviceFoundStructure) -> list:
    """
    release all the messages received during iso15765 mode.

    :returns: list of all messages received while iso1765 mode as active.
    """

    message_buffer = Device.iso15765_message_buffer.copy()
    Device.iso15765_message_buffer.clear()
    return message_buffer


def neovi_tx_message_iso15765(Device: NeoviDeviceFoundStructure,
                              netType: str,
                              extId: bool,
                              arbId: str,
                              fcExtendedId: bool,
                              fcArbId: str,
                              fcArbIdMask: str,
                              numOfBytes: int,
                              paddingEnabled: bool,
                              paddingByte: str,
                              databytes: list,
                              fcTimeout: str = '100',
                              fcWait: str = '3000',
                              useCanFd: bool = False
                              ) -> str:
    """
    This method sends ISO15765-2 multi-frame message.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param netType: the network type to use.
    :param extId: if this message is 11 bits or 29 bits message header.
    :param arbId: Arbitration ID for CAN messages (HEX)
    :param fcExtendedId:
    :param fcArbId: ArbID of the flow control frame to look for (HEX)
    :param fcArbIdMask: The flow control arb filter mask (response id from receiver) Bitwise mask for the flow
                        control arbitration ID.  (HEX).
    :param fcTimeout: Flow Control Timeout in ms, (default=100).
    :param fcWait: Flow Control FS=Wait Timeout ms, (default=3000) - not active.
    :param useCanFd: reserved.
    :param numOfBytes: the number of bytes in the message payload.
    :param paddingEnabled: enable flag for the padding functionality (for compatibility always true)
    :param paddingByte: HEX value used for padding to fill the rest of the last data frame for no padding leave it empty
    :param databytes: Data bytes of the message (HEX)
    :returns: bool represent the execution status, True for success.
    :raises ConnectionError: connectivity problem with the connected device.
    :raises TimeoutError: if the return ack message wait time passed after fcWait value.
    """
    # for backward compatibility we'll leave this flag as always true. since calling side assume it's always true.
    paddingEnabled = True

    if not Device.isOpen:
        raise ConnectionError("Problem (16): the requested device is closed, can't work with it.")
    try:
        ack_timeout = datetime.timedelta(microseconds=int(fcTimeout) * 1000)
    except ValueError as ve:
        # raise ValueError("Problem when converting fcTimeout to stopwatch") from ve
        raise ValueError("Problem when converting fcTimeout to stopwatch: " + str(ve))
    ack_message_databytes = ["30", "00", "00", "00", "00", "00", "00", "00"]

    # enable ISO1765 mode
    Device.iso15765_mutex = True

    # building the first packet
    first_packet = neovi_iso15765_create_first_message(payload=databytes[0:6], numOfBytes=numOfBytes,
                                                       paddingEnabled=paddingEnabled, paddingByte=paddingByte)
    # building rest of packets
    messages_payload_list = neovi_iso15765_create_trailing_messages(payload=databytes[6:],
                                                                    paddingEnabled=paddingEnabled,
                                                                    paddingByte=paddingByte)
    # start measuring time.
    latest_time = time_start = datetime.datetime.now()

    neovi_tx_message_can(Device=Device, network_name=netType, extended_arbId=extId, arb_id=arbId,
                         databytes=first_packet)

    while (latest_time - time_start) < ack_timeout:
        incoming_messages_list = neovi_get_messages(Device=Device, iso15765=True)

        for incoming_message in incoming_messages_list:

            # save the messages for future reporting by neovi_iso15765_release_buffer calls.
            Device.iso15765_message_buffer.append(incoming_message)

            try:
                # search for the ISO15765 ack message
                if incoming_message['msgType'] == 'Rx' and incoming_message['arbId'].lower() == fcArbId.lower() \
                        and incoming_message['dataBytes'] == ack_message_databytes:
                    for message in messages_payload_list:
                        neovi_tx_message_can(Device=Device, network_name=netType,
                                             extended_arbId=extId, arb_id=arbId,
                                             databytes=message)
                    Device.iso15765_mutex = False
                    return "ISO15765 Message transmitted successfully"
            except IndexError:
                Device.iso15765_mutex = False

        latest_time = datetime.datetime.now()

    # timeout occur
    Device.iso15765_mutex = False  # cancel iso15765 mode.
    return "Warning: ISO15765 Message timeout"


def neovi_script_start_function_block(Device: NeoviDeviceFoundStructure, iFunctionBlockIndex: int) -> str:
    """
    Starts the specified function block within a script on a neoVI device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param iFunctionBlockIndex: The index value of the function block.
    :returns: string represent the execution status.
    :raises ConnectionError: connectivity problem with the connected device.
    :raises ValueError: if the iFunctionBlockIndex is invalid.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (18): the requested device is closed, can't work with it.")

    if iFunctionBlockIndex < 0:
        raise ValueError("Problem (19): function block index is not valid")

    if neovi_dll.icsneoScriptStartFBlock(Device.hObject, iFunctionBlockIndex):
        return "Function block: " + str(iFunctionBlockIndex) + " activated"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script function block start failure: " + error_reason
            except KeyError as kr:
                return "Problem: script function block start failure: " + str(kr)
        return "Problem: script function block start failure"


def neovi_script_stop_function_block(Device: NeoviDeviceFoundStructure, iFunctionBlockIndex: int) -> str:
    """
    Stops the execution of a specified function block within a script on a neoVI device.

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param iFunctionBlockIndex: The index value of the function block.
    :returns: string represent the execution status.
    :raises ConnectionError: connectivity problem with the connected device.
    :raises ValueError: if the iFunctionBlockIndex is invalid.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (21): the requested device is closed, can't work with it.")

    if iFunctionBlockIndex < 0:
        raise ValueError("Problem (22): function block index is not valid")

    if neovi_dll.icsneoScriptStopFBlock(Device.hObject, iFunctionBlockIndex):
        return "Function block: " + str(iFunctionBlockIndex) + " deactivated"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script function block stop failure: " + error_reason
            except KeyError as kr:
                return "Problem: script function block stop failure: " + str(kr)
        return "Problem: script function block stop failure"


def neovi_script_get_function_block_status(Device: NeoviDeviceFoundStructure, iFunctionBlockIndex: int) -> str:
    """
    Status of a function block within a script on a neoVI device

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param iFunctionBlockIndex: The index value of the function block.
    :returns: string represent the execution status.
    :raises ConnectionError: connectivity problem with the connected device.
    :raises ValueError: if the iFunctionBlockIndex is invalid.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (24): the requested device is closed, can't work with it.")

    if iFunctionBlockIndex < 0:
        raise ValueError("Problem (25): function block index is not valid")

    piStatus = c_int(0)

    if neovi_dll.icsneoScriptGetFBlockStatus(Device.hObject, iFunctionBlockIndex, byref(piStatus)):
        if piStatus:
            return "Function block status: running"
        else:
            return "Function block status: stopped"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script function block status failure: " + error_reason
            except KeyError as kr:
                return "Problem: script function block status failure: " + str(kr)
        return "Problem: script function block status failure"


def neovi_script_load(Device: NeoviDeviceFoundStructure, filepath: str) -> str:
    """
    Downloads a script to a connected neoVI device into a specified location

    :param Device: Specifies the driver object created by OpenNeoDevice.
    :param filepath: full path of the file to use.
    :returns: confirmation of execution string
    :raises ConnectionError: connectivity problem with the connected device.
    :raises IOError: file location or existence problem.
    :raises KeyError: Unknown problem detected in script start.
    :raises TypeError: Function does not supported by this device.
    """
    if not Device.isOpen:
        raise ConnectionError("Problem (27): the requested device is closed, can't work with it.")

    if not os.path.isfile(filepath):
        raise IOError("Problem (28): unable to find the file at location: " + filepath)

    try:
        file_size = os.path.getsize(filepath)

        s_block = c_ubyte * file_size
        p_block = s_block()

        with open(filepath, 'rb') as f:
            for index in range(file_size):
                p_block[index] = int.from_bytes(f.read(1), 'big')

    except IOError:
        raise IOError("Problem (29): unable to find the file at location: " + filepath)

    if Device.NeoviDevice.DeviceType == device_types['NEODEVICE_FIRE'] or \
            Device.NeoviDevice.DeviceType == device_types['NEODEVICE_RED']:
        iLocation = script_location['SCRIPT_LOCATION_FLASH_MEM']
    elif Device.NeoviDevice.DeviceType == device_types['NEODEVICE_VCAN3']:
        iLocation = script_location['SCRIPT_LOCATION_VCAN3_MEM']
    else:
        raise TypeError("Problem: Function is not supported by this device type: " + str(Device.NeoviDevice.DeviceType))

    len_bytes = c_ulong(file_size)

    if neovi_dll.icsneoScriptLoad(Device.hObject, byref(p_block), len_bytes, iLocation):
        return "Script load successfully"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script load failure: " + error_reason
            except KeyError as kr:
                return "Problem: script load failure: " + str(kr)
        return "Problem: script load failure"


def neovi_script_start(Device: NeoviDeviceFoundStructure) -> str:
    """
    Starts the execution of a script that has been downloaded to a neoVI device.

    :pram Device: Specifies the driver object created by OpenNeoDevice.
    :returns: confirmation of execution string
    :raises KeyError: Unknown problem detected in script start.
    :raises TypeError: Function does not supportted by this device.
    :raises ConnectionError: connectivity to device failure.
    """
    # check if device is open
    if not Device.isOpen:
        raise ConnectionError("Problem (30): the requested device is closed, can't work with it.")

    # check the type of device and use the relevant memory location to use the script.
    if Device.NeoviDevice.DeviceType == device_types['NEODEVICE_FIRE'] or \
            Device.NeoviDevice.DeviceType == device_types['NEODEVICE_RED']:
        res = neovi_dll.icsneoScriptStart(Device.hObject, script_location['SCRIPT_LOCATION_FLASH_MEM'])
    elif Device.NeoviDevice.DeviceType == device_types['NEODEVICE_VCAN3']:
        res = neovi_dll.icsneoScriptStart(Device.hObject, script_location['SCRIPT_LOCATION_VCAN3_MEM'])
    else:
        raise TypeError("Problem (31): Function is not supported by this device type: "
                        + str(Device.NeoviDevice.DeviceType))

    if res:
        return "Script started successfully"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script load failure: " + error_reason
            except KeyError as kr:
                return "Problem: script load failure: " + str(kr)


def neovi_script_stop(Device: NeoviDeviceFoundStructure) -> str:
    """
    Stops the execution of a script that is running on a neoVI device

    :pram Device: Specifies the driver object created by OpenNeoDevice.
    :returns: confirmation of execution string
    :raises KeyError: Unknown problem detected in script start.
    :raises ConnectionError: connectivity to device failure.
    """
    # check if device is open
    if not Device.isOpen:
        raise ConnectionError("Problem (32): the requested device is closed, can't work with it.")

    if neovi_dll.icsneoScriptStop(Device.hObject):
        return "Script stopped successfully"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script load failure: " + error_reason
            except KeyError as kr:
                return "Problem: script load failure: " + str(kr)


def neovi_script_clear(Device: NeoviDeviceFoundStructure) -> str:
    """
    Clears a script from a specific location on a neoVI device.

    :pram Device: Specifies the driver object created by OpenNeoDevice.
    :returns: confirmation of execution string
    :raises KeyError: Unknown problem detected in script start.
    :raises ConnectionError: connectivity to device failure.
    """
    # check if device is open
    if not Device.isOpen:
        raise ConnectionError("Problem (35): the requested device is closed, can't work with it.")

    # check the type of device and use the relevant memory location to use the script.
    if Device.NeoviDevice.DeviceType == device_types['NEODEVICE_FIRE'] or \
            Device.NeoviDevice.DeviceType == device_types['NEODEVICE_RED']:
        res = neovi_dll.icsneoScriptClear(Device.hObject, script_location['SCRIPT_LOCATION_FLASH_MEM'])
    elif Device.NeoviDevice.DeviceType == device_types['NEODEVICE_VCAN3']:
        res = neovi_dll.icsneoScriptClear(Device.hObject, script_location['SCRIPT_LOCATION_VCAN3_MEM'])
    else:
        raise TypeError("Problem (36): Function is not supported by this device type: "
                        + str(Device.NeoviDevice.DeviceType))

    if res:
        return "Script cleared successfully"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script clear failure: " + error_reason
            except KeyError as kr:
                return "Problem: script clear failure: " + str(kr)


def neovi_script_status(Device: NeoviDeviceFoundStructure) -> str:
    """
    Returns the status of the script on a neoVI device.

    :pram Device: Specifies the driver object created by OpenNeoDevice.
    :returns: confirmation of execution string
    :raises KeyError: Unknown problem detected in script start.
    :raises ConnectionError: connectivity to device failure.
    """
    # check if device is open
    if not Device.isOpen:
        raise ConnectionError("Problem (37): the requested device is closed, can't work with it.")

    status = c_int(0)

    if neovi_dll.icsneoScriptGetScriptStatus(Device.hObject, byref(status)):
        if status.value:
            return "Script status: running"
        else:
            return "Script status: stopped"
    else:
        piErrorNumber = c_int32(0)
        if neovi_dll.icsneoGetLastAPIError(Device.hObject, byref(piErrorNumber)):
            try:
                error_reason = _get_error_messages(str(piErrorNumber.value))
                return "Problem: script status failure: " + error_reason
            except KeyError as kr:
                return "Problem: script status failure: " + str(kr)


class NeoViDriver:
    """
    This class act as interface to the neovi drivers internal implementation.
    """
    __instance = None

    @staticmethod
    def getInstance():
        """ Static access method. """
        if NeoViDriver.__instance is None:
            NeoViDriver()
        return NeoViDriver.__instance

    def __init__(self):

        """ Virtually private constructor. """
        if NeoViDriver.__instance is not None:
            raise Exception("Problem (17): This class is a singleton!")
        else:
            NeoViDriver.__instance = self

        self.neovi_device_found = neovi_device_found
        self.neovi_device_found_structure = NeoviDeviceFoundStructure
        self.neovi_get_error_messages = _get_error_messages_description
        self.neovi_get_error_information = _neovi_get_error_information
        self.neovi_list_can_devices = neovi_list_can_devices
        self.neovi_get_dll_version = neovi_get_dll_version
        self.neovi_open_can_device = neovi_open_can_device
        self.neovi_close_can_device = neovi_close_can_device
        self.neovi_get_fire_device_configuration = neovi_get_fire_device_configuration
        self.neovi_set_fire_can1_settings_SetBaudrate = neovi_set_fire_can1_settings_SetBaudrate
        self.neovi_set_fire_can1_settings_Baudrate = neovi_set_fire_can1_settings_Baudrate
        self.neovi_set_fire_can1_settings_Transceiver_Mode = neovi_set_fire_can1_settings_Transceiver_Mode
        self.neovi_set_fire_can1_settings_TqSeg1 = neovi_set_fire_can1_settings_TqSeg1
        self.neovi_set_fire_can1_settings_TqSeg2 = neovi_set_fire_can1_settings_TqSeg2
        self.neovi_set_fire_can1_settings_TqProp = neovi_set_fire_can1_settings_TqProp
        self.neovi_set_fire_can1_settings_TqSync = neovi_set_fire_can1_settings_TqSync
        self.neovi_set_fire_can1_settings_BRP = neovi_set_fire_can1_settings_BRP
        self.neovi_set_fire_can1_settings_auto_baud = neovi_set_fire_can1_settings_auto_baud
        self.neovi_set_fire_can1_settings_innerFrameDelay25us = neovi_set_fire_can1_settings_innerFrameDelay25us
        self.neovi_set_fire_can2_settings_SetBaudrate = neovi_set_fire_can2_settings_SetBaudrate
        self.neovi_set_fire_can2_settings_Baudrate = neovi_set_fire_can2_settings_Baudrate
        self.neovi_set_fire_can2_settings_Transceiver_Mode = neovi_set_fire_can2_settings_Transceiver_Mode
        self.neovi_set_fire_can2_settings_TqSeg1 = neovi_set_fire_can2_settings_TqSeg1
        self.neovi_set_fire_can2_settings_TqSeg2 = neovi_set_fire_can2_settings_TqSeg2
        self.neovi_set_fire_can2_settings_TqProp = neovi_set_fire_can2_settings_TqProp
        self.neovi_set_fire_can2_settings_TqSync = neovi_set_fire_can2_settings_TqSync
        self.neovi_set_fire_can2_settings_BRP = neovi_set_fire_can2_settings_BRP
        self.neovi_set_fire_can2_settings_auto_baud = neovi_set_fire_can2_settings_auto_baud
        self.neovi_set_fire_can2_settings_innerFrameDelay25us = neovi_set_fire_can2_settings_innerFrameDelay25us
        self.neovi_set_fire_can3_settings_SetBaudrate = neovi_set_fire_can3_settings_SetBaudrate
        self.neovi_set_fire_can3_settings_Baudrate = neovi_set_fire_can3_settings_Baudrate
        self.neovi_set_fire_can3_settings_Transceiver_Mode = neovi_set_fire_can3_settings_Transceiver_Mode
        self.neovi_set_fire_can3_settings_TqSeg1 = neovi_set_fire_can3_settings_TqSeg1
        self.neovi_set_fire_can3_settings_TqSeg2 = neovi_set_fire_can3_settings_TqSeg2
        self.neovi_set_fire_can3_settings_TqProp = neovi_set_fire_can3_settings_TqProp
        self.neovi_set_fire_can3_settings_TqSync = neovi_set_fire_can3_settings_TqSync
        self.neovi_set_fire_can3_settings_BRP = neovi_set_fire_can3_settings_BRP
        self.neovi_set_fire_can3_settings_auto_baud = neovi_set_fire_can3_settings_auto_baud
        self.neovi_set_fire_can3_settings_innerFrameDelay25us = neovi_set_fire_can3_settings_innerFrameDelay25us
        self.neovi_set_fire_can4_settings_SetBaudrate = neovi_set_fire_can4_settings_SetBaudrate
        self.neovi_set_fire_can4_settings_Baudrate = neovi_set_fire_can4_settings_Baudrate
        self.neovi_set_fire_can4_settings_Transceiver_Mode = neovi_set_fire_can4_settings_Transceiver_Mode
        self.neovi_set_fire_can4_settings_TqSeg1 = neovi_set_fire_can4_settings_TqSeg1
        self.neovi_set_fire_can4_settings_TqSeg2 = neovi_set_fire_can4_settings_TqSeg2
        self.neovi_set_fire_can4_settings_TqProp = neovi_set_fire_can4_settings_TqProp
        self.neovi_set_fire_can4_settings_TqSync = neovi_set_fire_can4_settings_TqSync
        self.neovi_set_fire_can4_settings_BRP = neovi_set_fire_can4_settings_BRP
        self.neovi_set_fire_can4_settings_auto_baud = neovi_set_fire_can4_settings_auto_baud
        self.neovi_set_fire_can4_settings_innerFrameDelay25us = neovi_set_fire_can4_settings_innerFrameDelay25us
        self.neovi_get_rtc = neovi_get_rtc
        self.neovi_set_rtc = neovi_set_rtc
        self.neovi_get_hw_firmware_info = neovi_get_hw_firmware_info
        self.neovi_get_dll_firmware_info = neovi_get_dll_firmware_info
        self.neovi_force_firmware_update = neovi_force_firmware_update
        self.neovi_set_bit_rate = neovi_set_bit_rate
        self.neovi_change_max_allowed_errors = neovi_change_max_allowed_errors
        self.neovi_get_messages = neovi_get_messages
        self.neovi_tx_message_can = neovi_tx_message_can
        self.neovi_tx_message_iso15765 = neovi_tx_message_iso15765
        self.neovi_script_start_function_block = neovi_script_start_function_block
        self.neovi_script_stop_function_block = neovi_script_stop_function_block
        self.neovi_script_get_function_block_status = neovi_script_get_function_block_status
        self.neovi_script_load = neovi_script_load
        self.neovi_script_start = neovi_script_start
        self.neovi_script_stop = neovi_script_stop
        self.neovi_script_clear = neovi_script_clear
        self.neovi_script_status = neovi_script_status
        self.neovi_iso15765_release_buffer = neovi_iso15765_release_buffer


# to allow exposure of the object outside of the package
NeoviDriver = NeoViDriver()
