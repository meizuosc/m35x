#ifndef __LINUX_ES305B_PARAM_H_
#define __LINUX_ES305B_PARAM_H_

static const u8 incall_ct_buf[] =
{
// !Preset id: 0 ; Handson_NB (CT)
	0x80, 0x31, 0x00, 0x00, // ; !Preset id: 0 ; Handson_NB (CT)
	0x80, 0x17, 0x00, 0x0D, 0x80, 0x18, 0xFF, 0xFA, // output limiter from -3db to -6dB
	0x80, 0x1B, 0x01, 0x04,
};

static const u8 incall_dv_buf[] =
{
// !Preset id: 3 ; Handsfree_NB (DV)
	0x80, 0x31, 0x00, 0x03, // !Preset id: 3 ; Handsfree_NB (DV)
};

static const u8 incall_whs_buf[] =
{
// !Preset id: 2 ; Wired Headset_NB (1MD)
	0x80, 0x31, 0x00, 0x02, // ; !Preset id: 2 ; Wired Headset_NB (1MD)
};

static const u8 incall_bt_buf[] =
{
// !Preset id: 4 ; BT
	0x80, 0x31, 0x00, 0x04, // ; !Preset id: 2 ; Wired Headset_NB (1MD)
	0x80, 0x17, 0x00, 0x03, 0x80, 0x18, 0x00, 0x01,

};

static const u8 incall_bt_vpoff_buf[] =
{
// !Preset id: 4 ; BT
	0x80, 0x31, 0x00, 0x04, // ; !Preset id: 2 ; Wired Headset_NB (1MD)
	0x80, 0x1C, 0x00, 0x00, // Disable Voice Processing
};

static const u8 voip_ct_buf[] =
{
};

static const u8 voip_dv_buf[] =
{
};

static const u8 voip_whs_buf[] =
{
};

static const u8 voip_bt_buf[] =
{
};


static const u8 voip_bt_vpoff_buf[] =
{
};

static const u8 bt_ring_buf[] = {
// !Preset id: 5 ; codec to BT
	0x80, 0x31, 0x00, 0x05, // preset 5 for bt ringtone
};

static const u8 bypass_a2c[] = {
	0x80, 0x52, 0x00, 0x00, // disable digital pass through
	0x80, 0x52, 0x00, 0xC8,
};

static const u8 bypass_c2a[] = {
	0x80, 0x52, 0x00, 0x00, // disable digital pass through
	0x80, 0x52, 0x00, 0xE2,
};

#endif /* __LINUX_ES305B_PARAM_H_ */
