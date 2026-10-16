/*
 * Samsung S6E3FA7 AMS604NL01 AMOLED brightness control (Galaxy A6+).
 *
 * The panel ignores the generic DCS 0x51 brightness command: Samsung drives it
 * by rewriting AID (B1h), ACL (B4h/55h), ELVSS (B5h), VINT (F4h), IRC (B8h) and
 * the gamma curve (CAh) for every luminance step. The gamma is computed from the
 * calibration data (MTP, C8h) that the panel stores, with the smart dimming code
 * taken from the Samsung 3.18 kernel. Tables come from the stock device tree.
 *
 * If the panel cannot be read, nothing is sent and the caller falls back to 0x51.
 */
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/init.h>

#include "mdss_dsi.h"
#include "samsung_bl/ss_dsi_smart_dimming_common.h"
#include "samsung_bl/ss_bl_tables_S6E3FA7.h"

struct smartdim_conf *smart_get_conf_S6E3FA7_AMS604NL01(void);

#define SS_BL_NORMAL_MAX	255	/* AID/IRC tables have 256 entries */
#define SS_BL_MTP_LEN		0x20
#define SS_BL_READ_CHUNK	10
#define SS_BL_TEMPERATURE	20

/* lcd_id=0x811143 is passed by the bootloader: DA/DB/DC manufacture id */
static unsigned int ss_lcd_id = 0x811143;

static int __init ss_bl_lcd_id_setup(char *s)
{
	unsigned int v;

	if (!kstrtouint(s, 0, &v))
		ss_lcd_id = v;
	return 1;
}
__setup("lcd_id=", ss_bl_lcd_id_setup);

static struct {
	struct mutex lock;
	bool tried;
	bool ready;
	struct smartdim_conf *sd;
	u8 elvss_otp;		/* B5h 23rd parameter */
	u8 irc_otp[2];		/* B8h 1st, 2nd parameter */
	int lux[SS_BL_NLUX];
} ss = { .lock = __MUTEX_INITIALIZER(ss.lock) };

static char key_on[]  = { 0xf0, 0x5a, 0x5a };
static char key_off[] = { 0xf0, 0xa5, 0xa5 };
static char gamma_update[] = { 0xf7, 0x03 };	/* latch AID/ELVSS/gamma, as stock gamma_tx_cmds */

static void ss_fill(struct dsi_cmd_desc *d, char dtype, char *payload, int len)
{
	d->dchdr.dtype = dtype;
	d->dchdr.last = 1;
	d->dchdr.vc = 0;
	d->dchdr.ack = 0;
	d->dchdr.wait = 0;
	d->dchdr.dlen = len;
	d->payload = payload;
}

static int ss_tx(struct mdss_dsi_ctrl_pdata *ctrl, struct dsi_cmd_desc *cmds, int cnt)
{
	struct dcs_cmd_req req;

	memset(&req, 0, sizeof(req));
	req.cmds = cmds;
	req.cmds_cnt = cnt;
	req.flags = CMD_REQ_COMMIT | CMD_REQ_HS_MODE;
	return mdss_dsi_cmdlist_put(ctrl, &req);
}

/* Read @len bytes of register @reg starting at parameter @off. */
static int ss_read(struct mdss_dsi_ctrl_pdata *ctrl, char reg, int off, int len, u8 *out)
{
	struct dsi_cmd_desc d;
	char pos[2] = { 0xb0, 0 };
	char rx[SS_BL_READ_CHUNK + 4];
	int done = 0, n, ret;

	while (done < len) {
		n = min(len - done, SS_BL_READ_CHUNK);
		pos[1] = off + done;
		ss_fill(&d, 0x15, pos, 2);
		ret = ss_tx(ctrl, &d, 1);
		if (ret < 0)
			return ret;
		memset(rx, 0, sizeof(rx));
		ret = mdss_dsi_panel_cmd_read(ctrl, reg, 0, NULL, rx, n);
		if (ret < 0)
			return ret;
		pr_info("ss_bl: read %02x off=%d n=%d ret=%d:%*phC\n", reg, off + done, n, ret, n, rx);
		memcpy(out + done, rx, n);
		done += n;
	}
	return 0;
}

static bool ss_mtp_valid(const u8 *b, int len)
{
	int i;
	bool all0 = true, allff = true;

	for (i = 0; i < len; i++) {
		if (b[i] != 0x00)
			all0 = false;
		if (b[i] != 0xff)
			allff = false;
	}
	return !all0 && !allff;
}

static int ss_init(struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct dsi_cmd_desc d;
	u8 b5[2], b8[2];
	int i, ret;

	ss.sd = smart_get_conf_S6E3FA7_AMS604NL01();
	if (!ss.sd) {
		pr_err("ss_bl: no smart dimming configuration\n");
		return -ENOMEM;
	}

	ss_fill(&d, 0x39, key_on, sizeof(key_on));
	ret = ss_tx(ctrl, &d, 1);
	if (ret < 0)
		return ret;

	ret = ss_read(ctrl, 0xc8, 0, SS_BL_MTP_LEN, (u8 *)ss.sd->mtp_buffer);
	if (!ret)
		ret = ss_read(ctrl, 0xb5, 0x16, 2, b5);
	if (!ret)
		ret = ss_read(ctrl, 0xb8, 0, 2, b8);

	ss_fill(&d, 0x39, key_off, sizeof(key_off));
	ss_tx(ctrl, &d, 1);

	if (ret < 0) {
		pr_err("ss_bl: panel read failed (%d)\n", ret);
		return ret;
	}
	if (!ss_mtp_valid((u8 *)ss.sd->mtp_buffer, SS_BL_MTP_LEN)) {
		pr_err("ss_bl: MTP data looks invalid, keeping 0x51 brightness\n");
		return -EINVAL;
	}

	ss.elvss_otp = b5[0];
	ss.irc_otp[0] = b8[0];
	ss.irc_otp[1] = b8[1];

	for (i = 0; i < SS_BL_NLUX; i++)
		ss.lux[i] = ss_bl_cd[i];
	ss.sd->lux_tab = ss.lux;
	ss.sd->lux_tabsize = SS_BL_NLUX;
	ss.sd->man_id = ss_lcd_id;
	ss.sd->panel_revision = 'A';
	ss.sd->hbm_payload = (char *)&ss_bl_hbm_gamma[1];
	ss.sd->init(ss.sd);

	pr_info("ss_bl: ready (lcd_id=0x%x mtp %02x %02x %02x %02x elvss=0x%02x irc=%02x %02x)\n",
		ss_lcd_id, (u8)ss.sd->mtp_buffer[0], (u8)ss.sd->mtp_buffer[1],
		(u8)ss.sd->mtp_buffer[2], (u8)ss.sd->mtp_buffer[3],
		ss.elvss_otp, ss.irc_otp[0], ss.irc_otp[1]);
	return 0;
}

/* Binary search of the luminance step for a backlight level */
static int ss_step(int level)
{
	int l = 0, r = SS_BL_NLUX - 1, p;

	while (l <= r) {
		p = (l + r) / 2;
		if (level >= ss_bl_from[p] && level <= ss_bl_end[p])
			return p;
		if (level < ss_bl_from[p])
			r = p - 1;
		else
			l = p + 1;
	}
	return SS_BL_NLUX - 1;
}

/*
 * Returns 0 when the Samsung sequence was sent, a negative value when the
 * caller must use the generic DCS brightness instead.
 */
int mdss_dsi_samsung_bl_set(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dsi_cmd_desc d[10];
	char aid[3], acl_b4[sizeof(ss_bl_acl_off_b4)], acl_55[sizeof(ss_bl_acl_off_55)];
	char elvss[24], vint[3], irc[34], gamma[36];
	int step, idx, n = 0, ret;

	mutex_lock(&ss.lock);
	if (!ss.tried) {
		ss.tried = true;
		ss.ready = !ss_init(ctrl);
	}
	if (!ss.ready) {
		mutex_unlock(&ss.lock);
		return -ENODEV;
	}

	if (level < 0)
		level = 0;
	if (level > SS_BL_MAX_LEVEL)
		level = SS_BL_MAX_LEVEL;
	step = ss_step(level);
	idx = min(level, SS_BL_NORMAL_MAX);

	memcpy(aid, ss_bl_aid[idx], sizeof(aid));
	memcpy(acl_b4, ss_bl_acl_off_b4, sizeof(acl_b4));
	memcpy(acl_55, ss_bl_acl_off_55, sizeof(acl_55));
	memcpy(elvss, ss_bl_elvss[ss_bl_elvss_map[step]], sizeof(elvss));
	elvss[1] = SS_BL_TEMPERATURE;
	elvss[23] = ss.elvss_otp;
	memcpy(vint, ss_bl_vint[ss_bl_vint_map[step]], sizeof(vint));
	memcpy(irc, ss_bl_irc[idx], sizeof(irc));
	irc[1] = ss.irc_otp[0];
	irc[2] = ss.irc_otp[1];
	memset(gamma, 0, sizeof(gamma));
	gamma[0] = 0xca;
	ss.sd->generate_gamma(ss.sd, ss_bl_cd[step], &gamma[1]);

	ss_fill(&d[n++], 0x39, key_on, sizeof(key_on));
	ss_fill(&d[n++], 0x39, aid, sizeof(aid));
	ss_fill(&d[n++], 0x39, acl_b4, sizeof(acl_b4));
	ss_fill(&d[n++], 0x39, acl_55, sizeof(acl_55));
	ss_fill(&d[n++], 0x39, elvss, sizeof(elvss));
	ss_fill(&d[n++], 0x39, vint, sizeof(vint));
	ss_fill(&d[n++], 0x39, irc, sizeof(irc));
	ss_fill(&d[n++], 0x39, gamma, 34);
	ss_fill(&d[n++], 0x39, gamma_update, sizeof(gamma_update));
	ss_fill(&d[n++], 0x39, key_off, sizeof(key_off));

	ret = ss_tx(ctrl, d, n);
	pr_info("ss_bl: level=%d step=%d cd=%d ret=%d\n", level, step, ss_bl_cd[step], ret);
	mutex_unlock(&ss.lock);
	return ret < 0 ? ret : 0;
}
