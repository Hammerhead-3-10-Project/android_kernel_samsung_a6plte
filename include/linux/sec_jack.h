/* Samsung 3.5mm headset jack: types reported on /sys/class/switch/h2w */
#ifndef __LINUX_SEC_JACK_H
#define __LINUX_SEC_JACK_H

enum sec_jack_type {
	SEC_JACK_NO_DEVICE	= 0x0,
	SEC_HEADSET_4POLE	= 0x01 << 0,
	SEC_HEADSET_3POLE	= 0x01 << 1,
	SEC_EXTERNAL_ANTENNA	= 0x01 << 2,
};

/* implemented by the PMIC analog codec: switch the headset microphone bias */
int msm_anlg_cdc_sec_micbias(bool on);

#endif
