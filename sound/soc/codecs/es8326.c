// SPDX-License-Identifier: GPL-2.0-only
//
// es8326.c -- es8326 ALSA SoC audio driver
// Copyright Everest Semiconductor Co., Ltd
//
// Authors: David Yang <yangxiaohua@everest-semi.com>
//

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/delay.h>
#include "es8326.h"

struct es8326_priv {
	struct clk *mclk;
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct snd_soc_component *component;
	struct delayed_work jack_detect_work;
	struct delayed_work button_press_work;
	struct snd_soc_jack *jack;
	int irq;
	int hp;
	/* The lock protects the situation that an irq is generated
	 * while enabling or disabling or during an irq.
	 */
	struct mutex lock;
	u8 mic1_src;
	u8 mic2_src;
	u8 jack_pol;
	u8 interrupt_src;
	u8 interrupt_clk;
	bool jd_inverted;
	unsigned int sysclk;

	int hp_detect_gpio;
	int calidone;	
};

static int es8326_dacstream_event(struct snd_soc_dapm_widget *w,
		  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	unsigned int regv = 0;
	printk("enter into %s,\n", __func__);
	regmap_read(es8326->regmap, ES8326_HP_CAL, &regv);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		printk("enter into %s, ---> SND_SOC_DAPM_PRE_PMU\n", __func__);
		break;
	case SND_SOC_DAPM_POST_PMU:
		regmap_write(es8326->regmap, ES8326_HP_DRIVER, 0xA0);
		regmap_write(es8326->regmap, ES8326_ANA_PDN, 0x00);
		regmap_write(es8326->regmap, ES8326_ANA_LP, 0xf0);
		regmap_write(es8326->regmap, ES8326_ANA_LP, 0xe8);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		regmap_write(es8326->regmap, ES8326_HP_CAL, 0x00);
		regmap_write(es8326->regmap, ES8326_HP_DRIVER, 0x8b);
		break;
	}

	return 0;
}

static int headphone_enable_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	unsigned int regv = 0;

	regmap_read(es8326->regmap, ES8326_HP_CAL, &regv);
	printk("enter into %s, regv = %2x\n", __func__, regv);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		printk("enter into es8326_headphone ---> SND_SOC_DAPM_PRE_PMU\n");

		break;
	case SND_SOC_DAPM_POST_PMU:
		printk("enter into es8326_headphone ---> SND_SOC_DAPM_POST_PMU\n");
		regmap_write(es8326->regmap, ES8326_RESET, 0x80);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		printk("enter into es8326_headphone ---> SND_SOC_DAPM_PRE_PMD\n");
		regmap_write(es8326->regmap, ES8326_HP_CAL, 0x00);
		break;
	default:
		dev_dbg(component->dev,
			"Unhandled dapm widget event %d from %s\n",
			event, w->name);
	}
	return 0;
}

static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(dac_vol_tlv, -9550, 50, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(adc_vol_tlv, -9550, 50, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(adc_analog_pga_tlv, 0, 300, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(adc_pga_tlv, 0, 600, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(softramp_rate, 0, 100, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(drc_target_tlv, -3200, 200, 0);
static const SNDRV_CTL_TLVD_DECLARE_DB_SCALE(drc_recovery_tlv, -125, 250, 0);

static const char *const winsize[] = {
	"0.25db/2  LRCK",
	"0.25db/4  LRCK",
	"0.25db/8  LRCK",
	"0.25db/16  LRCK",
	"0.25db/32  LRCK",
	"0.25db/64  LRCK",
	"0.25db/128  LRCK",
	"0.25db/256  LRCK",
	"0.25db/512  LRCK",
	"0.25db/1024  LRCK",
	"0.25db/2048  LRCK",
	"0.25db/4096  LRCK",
	"0.25db/8192  LRCK",
	"0.25db/16384  LRCK",
	"0.25db/32768  LRCK",
	"0.25db/65536  LRCK",
};

static const char *const dacpol_txt[] =	{
	"Normal", "R Invert", "L Invert", "L + R Invert" };

static const struct soc_enum dacpol =
	SOC_ENUM_SINGLE(ES8326_DAC_DSM, 4, 4, dacpol_txt);
static const struct soc_enum alc_winsize =
	SOC_ENUM_SINGLE(ES8326_ADC_RAMPRATE, 4, 16, winsize);
static const struct soc_enum drc_winsize =
	SOC_ENUM_SINGLE(ES8326_DRC_WINSIZE, 4, 16, winsize);

static const struct snd_kcontrol_new es8326_snd_controls[] = {
	SOC_SINGLE_TLV("DAC Playback Volume", ES8326_DAC_VOL, 0, 0xbf, 0, dac_vol_tlv),
	SOC_ENUM("Playback Polarity", dacpol),
	SOC_SINGLE_TLV("DAC Ramp Rate", ES8326_DAC_RAMPRATE, 0, 0x0f, 0, softramp_rate),
	SOC_SINGLE_TLV("DRC Recovery Level", ES8326_DRC_RECOVERY, 0, 4, 0, drc_recovery_tlv),
	SOC_ENUM("DRC Winsize", drc_winsize),
	SOC_SINGLE_TLV("DRC Target Level", ES8326_DRC_WINSIZE, 0, 0x0f, 0, drc_target_tlv),

	SOC_DOUBLE_R_TLV("ADC Capture Volume", ES8326_ADC1_VOL, ES8326_ADC2_VOL, 0, 0xff, 0,
			 adc_vol_tlv),
	SOC_DOUBLE_TLV("ADC PGA Volume", ES8326_ADC_SCALE, 4, 0, 5, 0, adc_pga_tlv),
	SOC_SINGLE_TLV("ADC PGA Gain Volume", ES8326_PGAGAIN, 0, 10, 0, adc_analog_pga_tlv),
	SOC_SINGLE_TLV("ADC Ramp Rate", ES8326_ADC_RAMPRATE, 0, 0x0f, 0, softramp_rate),
	SOC_SINGLE("ALC Capture Switch", ES8326_ALC_RECOVERY, 3, 1, 0),
	SOC_SINGLE_TLV("ALC Capture Recovery Level", ES8326_ALC_LEVEL,
			0, 4, 0, drc_recovery_tlv),
	SOC_ENUM("ALC Capture Winsize", alc_winsize),
	SOC_SINGLE_TLV("ALC Capture Target Level", ES8326_ALC_LEVEL,
			0, 0x0f, 0, drc_target_tlv),

};

static const struct snd_soc_dapm_widget es8326_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("MIC3"),
	SND_SOC_DAPM_INPUT("MIC4"),

	SND_SOC_DAPM_ADC("ADC L", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC R", NULL, SND_SOC_NOPM, 0, 0),

	/* Digital Interface */
	SND_SOC_DAPM_AIF_OUT("I2S OUT", "I2S1 Capture", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN_E("I2S IN", "I2S1 Playback", 0, SND_SOC_NOPM, 0, 0,
			es8326_dacstream_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
	
	/* ADC Digital Mute */
	SND_SOC_DAPM_PGA("ADC L1", ES8326_ADC_MUTE, 0, 1, NULL, 0),
	SND_SOC_DAPM_PGA("ADC R1", ES8326_ADC_MUTE, 1, 1, NULL, 0),
	SND_SOC_DAPM_PGA("ADC L2", ES8326_ADC_MUTE, 2, 1, NULL, 0),
	SND_SOC_DAPM_PGA("ADC R2", ES8326_ADC_MUTE, 3, 1, NULL, 0),

	/* Analog Power Supply*/
	SND_SOC_DAPM_DAC("Right DAC", NULL, ES8326_ANA_PDN, 0, 1),
	SND_SOC_DAPM_DAC("Left DAC", NULL, ES8326_ANA_PDN, 1, 1),
	SND_SOC_DAPM_SUPPLY("MICBIAS1", ES8326_ANA_MICBIAS, 2, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("MICBIAS2", ES8326_ANA_MICBIAS, 3, 0, NULL, 0),	
	
	SND_SOC_DAPM_PGA("LHPMIX", ES8326_DAC2HPMIX, 7, 0, NULL, 0),
	SND_SOC_DAPM_PGA("RHPMIX", ES8326_DAC2HPMIX, 3, 0, NULL, 0),

	/* Headphone Charge Pump and Output */
	SND_SOC_DAPM_OUT_DRV_E("HPOR Enable", ES8326_HP_CAL, 6, 0, NULL, 0,
			headphone_enable_event,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("HPOL Enable", ES8326_HP_CAL, 2, 0, NULL, 0,
			headphone_enable_event, 
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
			SND_SOC_DAPM_PRE_PMD),
	
	SND_SOC_DAPM_OUTPUT("HPOL"),
	SND_SOC_DAPM_OUTPUT("HPOR"),
};

static const struct snd_soc_dapm_route es8326_dapm_routes[] = {
	{"ADC L1", NULL, "MIC1"},
	{"ADC R1", NULL, "MIC2"},
	{"ADC L2", NULL, "MIC3"},
	{"ADC R2", NULL, "MIC4"},

	{"ADC L", NULL, "ADC L1"},
	{"ADC R", NULL, "ADC R1"},
	{"ADC L", NULL, "ADC L2"},
	{"ADC R", NULL, "ADC R2"},

	{"I2S OUT", NULL, "ADC L"},
	{"I2S OUT", NULL, "ADC R"},

	{"Right DAC", NULL, "I2S IN"},
	{"Left DAC", NULL, "I2S IN"},
	{"LHPMIX", NULL, "Left DAC"},
	{"RHPMIX", NULL, "Right DAC"},
	{"HPOL Enable", NULL, "LHPMIX"},
	{"HPOR Enable", NULL, "RHPMIX"},

	{"HPOL", NULL, "HPOL Enable"},
	{"HPOR", NULL, "HPOR Enable"},
};

static const struct regmap_range es8326_volatile_ranges[] = {
	regmap_reg_range(ES8326_HP_DETECT, ES8326_HP_DETECT),
};

static const struct regmap_access_table es8326_volatile_table = {
	.yes_ranges = es8326_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(es8326_volatile_ranges),
};

static const struct regmap_config es8326_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
	.volatile_table = &es8326_volatile_table,
	.cache_type = REGCACHE_RBTREE,
};

struct _coeff_div {
	u16 fs;
	u32 rate;
	u32 mclk;
	u8 reg4;
	u8 reg5;
	u8 reg6;
	u8 reg7;
	u8 reg8;
	u8 reg9;
	u8 rega;
	u8 regb;
};

/* codec hifi mclk clock divider coefficients */
/* {ratio, LRCK, MCLK, REG04, REG05, REG06, REG07, REG08, REG09, REG10, REG11} */
static const struct _coeff_div coeff_div[] = {
	{32, 8000, 256000, 0x60, 0x00, 0x0F, 0x75, 0x0A, 0x1B, 0x1F, 0x7F},
	{32, 16000, 512000, 0x20, 0x00, 0x0D, 0x75, 0x0A, 0x1B, 0x1F, 0x3F},
	{32, 44100, 1411200, 0x00, 0x00, 0x13, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{32, 48000, 1536000, 0x00, 0x00, 0x13, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{36, 8000, 288000, 0x20, 0x00, 0x0D, 0x75, 0x0A, 0x1B, 0x23, 0x47},
	{36, 16000, 576000, 0x20, 0x00, 0x0D, 0x75, 0x0A, 0x1B, 0x23, 0x47},
	{48, 8000, 384000, 0x60, 0x02, 0x1F, 0x75, 0x0A, 0x1B, 0x1F, 0x7F},
	{48, 16000, 768000, 0x20, 0x02, 0x0F, 0x75, 0x0A, 0x1B, 0x1F, 0x3F},
	{48, 48000, 2304000, 0x00, 0x02, 0x0D, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{64, 8000, 512000, 0x60, 0x00, 0x0D, 0x75, 0x0A, 0x1B, 0x1F, 0x7F},
	{64, 16000, 1024000, 0x20, 0x00, 0x05, 0x75, 0x0A, 0x1B, 0x1F, 0x3F},

	{64, 44100, 2822400, 0x00, 0x00, 0x11, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{64, 48000, 3072000, 0x00, 0x00, 0x11, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{72, 8000, 576000, 0x20, 0x00, 0x13, 0x35, 0x0A, 0x1B, 0x23, 0x47},
	{72, 16000, 1152000, 0x20, 0x00, 0x05, 0x75, 0x0A, 0x1B, 0x23, 0x47},
	{96, 8000, 768000, 0x60, 0x02, 0x1D, 0x75, 0x0A, 0x1B, 0x1F, 0x7F},
	{96, 16000, 1536000, 0x20, 0x02, 0x0D, 0x75, 0x0A, 0x1B, 0x1F, 0x3F},
	{100, 48000, 4800000, 0x04, 0x04, 0x3F, 0x6D, 0x38, 0x08, 0x4f, 0x1f},
	{125, 48000, 6000000, 0x04, 0x04, 0x1F, 0x2D, 0x0A, 0x0A, 0x27, 0x27},
	{128, 8000, 1024000, 0x60, 0x00, 0x13, 0x35, 0x0A, 0x1B, 0x1F, 0x7F},
	{128, 16000, 2048000, 0x20, 0x00, 0x11, 0x35, 0x0A, 0x1B, 0x1F, 0x3F},

	{128, 44100, 5644800, 0x00, 0x00, 0x01, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{128, 48000, 6144000, 0x00, 0x00, 0x01, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{144, 8000, 1152000, 0x20, 0x00, 0x03, 0x35, 0x0A, 0x1B, 0x23, 0x47},
	{144, 16000, 2304000, 0x20, 0x00, 0x11, 0x35, 0x0A, 0x1B, 0x23, 0x47},
	{192, 8000, 1536000, 0x60, 0x02, 0x0D, 0x75, 0x0A, 0x1B, 0x1F, 0x7F},
	{192, 16000, 3072000, 0x20, 0x02, 0x05, 0x75, 0x0A, 0x1B, 0x1F, 0x3F},
	{200, 48000, 9600000, 0x04, 0x04, 0x0F, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{250, 48000, 12000000, 0x04, 0x04, 0x0F, 0x2D, 0x0A, 0x0A, 0x27, 0x27},

	{256, 8000, 2048000, 0x01, 0x00, 0x30, 0x2D, 0x26, 0x26, 0x1F, 0x1F},
	{256, 16000, 4096000, 0x01, 0x00, 0x30, 0x2D, 0x26, 0x26, 0x1F, 0x1F},

	{256, 44100, 11289600, 0x01, 0x00, 0x30, 0x2D, 0x26, 0x26, 0x4F, 0x1F},
	{256, 48000, 12288000, 0x01, 0x00, 0x30, 0x2D, 0x26, 0x26, 0x4F, 0x1F},
	{288, 8000, 2304000, 0x20, 0x00, 0x01, 0x35, 0x0A, 0x1B, 0x23, 0x47},
	{384, 8000, 3072000, 0x60, 0x02, 0x05, 0x75, 0x0A, 0x1B, 0x1F, 0x7F},
	{384, 16000, 6144000, 0x20, 0x02, 0x03, 0x35, 0x0A, 0x1B, 0x1F, 0x3F},
	{384, 48000, 18432000, 0x00, 0x02, 0x01, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{400, 48000, 19200000, 0x09, 0x04, 0x0f, 0x6d, 0x3a, 0x0A, 0x4F, 0x1F},
	{500, 48000, 24000000, 0x18, 0x04, 0x1F, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{512, 8000, 4096000, 0x60, 0x00, 0x01, 0x35, 0x0A, 0x1B, 0x1F, 0x7F},
	{512, 16000, 8192000, 0x20, 0x00, 0x10, 0x35, 0x0A, 0x1B, 0x1F, 0x3F},

	{512, 44100, 22579200, 0x00, 0x00, 0x00, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{512, 48000, 24576000, 0x00, 0x00, 0x00, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{768, 8000, 6144000, 0x60, 0x02, 0x11, 0x35, 0x0A, 0x1B, 0x1F, 0x7F},
	{768, 16000, 12288000, 0x20, 0x02, 0x01, 0x35, 0x0A, 0x1B, 0x1F, 0x3F},
	{800, 48000, 38400000, 0x00, 0x18, 0x13, 0x2D, 0x0A, 0x0A, 0x1F, 0x1F},
	{1024, 8000, 8192000, 0x60, 0x00, 0x10, 0x35, 0x0A, 0x1B, 0x1F, 0x7F},
	{1024, 16000, 16384000, 0x20, 0x00, 0x00, 0x35, 0x0A, 0x1B, 0x1F, 0x3F},
	{1152, 16000, 18432000, 0x20, 0x08, 0x11, 0x35, 0x0A, 0x1B, 0x1F, 0x3F},
	{1536, 8000, 12288000, 0x60, 0x02, 0x01, 0x35, 0x0A, 0x1B, 0x1F, 0x7F},

	{1536, 16000, 24576000, 0x20, 0x02, 0x10, 0x35, 0x0A, 0x1B, 0x1F, 0x3F},
	{1625, 8000, 13000000, 0x0C, 0x18, 0x1F, 0x2D, 0x0A, 0x0A, 0x27, 0x27},
	{1625, 16000, 26000000, 0x0C, 0x18, 0x1F, 0x2D, 0x0A, 0x0A, 0x27, 0x27},
	{2048, 8000, 16384000, 0x60, 0x00, 0x00, 0x35, 0x0A, 0x1B, 0x1F, 0x7F},
	{2304, 8000, 18432000, 0x40, 0x02, 0x10, 0x35, 0x0A, 0x1B, 0x1F, 0x5F},
	{3072, 8000, 24576000, 0x60, 0x02, 0x10, 0x35, 0x0A, 0x1B, 0x1F, 0x7F},
	{3250, 8000, 26000000, 0x0C, 0x18, 0x0F, 0x2D, 0x0A, 0x0A, 0x27, 0x27},

};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

static int es8326_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_component *codec = codec_dai->component;
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(codec);

	es8326->sysclk = freq;

	return 0;
}

static int es8326_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_component *component = codec_dai->component;
	u8 iface = 0;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFP:
		snd_soc_component_update_bits(component, ES8326_RESET,
				ES8326_MASTER_MODE_EN, ES8326_MASTER_MODE_EN);
		break;
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		dev_err(component->dev, "Codec driver does not support right justified\n");
		return -EINVAL;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= ES8326_DAIFMT_LEFT_J;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= ES8326_DAIFMT_DSP_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= ES8326_DAIFMT_DSP_B;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_component_update_bits(component, ES8326_FMT, ES8326_DAIFMT_MASK, iface);

	return 0;
}

static int es8326_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	u8 srate = 0;
	int coeff;

	printk("Enter into %s\n", __func__);
	
	coeff = get_coeff(es8326->sysclk, params_rate(params));
	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		srate |= ES8326_S16_LE;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		srate |= ES8326_S20_3_LE;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		srate |= ES8326_S18_LE;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		srate |= ES8326_S24_LE;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		srate |= ES8326_S32_LE;
		break;
	default:
		return -EINVAL;
	}

	/* set iface & srate */
	snd_soc_component_update_bits(component, ES8326_FMT, ES8326_DATA_LEN_MASK, srate);

	if (coeff >= 0) {
		regmap_write(es8326->regmap,  ES8326_CLK_DIV1,
			     coeff_div[coeff].reg4);
		regmap_write(es8326->regmap,  ES8326_CLK_DIV2,
			     coeff_div[coeff].reg5);
		regmap_write(es8326->regmap,  ES8326_CLK_DLL,
			     coeff_div[coeff].reg6);
		regmap_write(es8326->regmap,  ES8326_CLK_MUX,
			     coeff_div[coeff].reg7);
		regmap_write(es8326->regmap,  ES8326_CLK_ADC_SEL,
			     coeff_div[coeff].reg8);
		regmap_write(es8326->regmap,  ES8326_CLK_DAC_SEL,
			     coeff_div[coeff].reg9);
		regmap_write(es8326->regmap,  ES8326_CLK_ADC_OSR,
			     coeff_div[coeff].rega);
		regmap_write(es8326->regmap,  ES8326_CLK_DAC_OSR,
			     coeff_div[coeff].regb);
	} else {
		dev_warn(component->dev, "Clock coefficients do not match");
	}

	return 0;
}

static int es8326_stream_prepare(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	unsigned int regv = 0;

	printk("Enter into %s\n", __func__);
	if(es8326->calidone == 0) {
		printk("%s, startup headphone calibrate\n", __func__);
		es8326->calidone = 1;
		regmap_read(es8326->regmap, ES8326_HP_CAL, &regv);
		regv |= 0x88;
		regmap_write(es8326->regmap, ES8326_HP_CAL, regv);
		regmap_write(es8326->regmap, ES8326_RESET, 0x80);
		msleep(100);
	}

	return 0;
}

static void es8326_stream_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct snd_soc_component *component = dai->component;
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);

	printk("Enter into %s\n", __func__);
	if(substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
	{
		printk("Enter into %s----->SNDRV_PCM_STREAM_PLAYBACK\n", __func__);
		regmap_write(es8326->regmap, ES8326_HP_CAL, 0x00);
	}

}

static int es8326_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	int ret;

	printk("Enter into %s, level = %d\n", __func__, level);
	switch (level) {
	case SND_SOC_BIAS_ON:
		ret = clk_prepare_enable(es8326->mclk);
		if (ret)
			return ret;
		regmap_write(es8326->regmap, ES8326_RESET, 0x90);
		snd_soc_component_update_bits(component, ES8326_DAC_DSM,
				0x01, 0x00);
		regmap_write(es8326->regmap, ES8326_INTOUT_IO, 0x45);
		regmap_write(es8326->regmap, ES8326_SDINOUT1_IO,
			    (ES8326_IO_DMIC_CLK << ES8326_SDINOUT1_SHIFT));
		regmap_write(es8326->regmap, ES8326_SDINOUT23_IO, ES8326_IO_INPUT);
		regmap_write(es8326->regmap, ES8326_CLK_RESAMPLE, 0x05);
		regmap_write(es8326->regmap, ES8326_VMIDSEL, 0x02);
		regmap_write(es8326->regmap, ES8326_PGA_PDN, 0x40);
		regmap_write(es8326->regmap, ES8326_DAC2HPMIX, 0xAA);
		regmap_write(es8326->regmap, ES8326_RESET, ES8326_CSM_ON);
		regmap_write(es8326->regmap, ES8326_CSM_I2C_STA, 0x80);
		regmap_write(es8326->regmap, ES8326_CSM_I2C_STA, 0x00);
		msleep(10);
		regmap_write(es8326->regmap, ES8326_HP_CAL, 0x55);
		break;
	case SND_SOC_BIAS_PREPARE:
		snd_soc_component_update_bits(component, ES8326_CLK_CTL,
				0x20, 0x20);
		break;
	case SND_SOC_BIAS_STANDBY:
		snd_soc_component_update_bits(component, ES8326_CLK_CTL,
				0x20, 0x00);
		break;
	case SND_SOC_BIAS_OFF:
		clk_disable_unprepare(es8326->mclk);
		regmap_write(es8326->regmap, ES8326_DAC2HPMIX, 0x11);
		regmap_write(es8326->regmap, ES8326_RESET, ES8326_CSM_OFF);
		regmap_write(es8326->regmap, ES8326_PGA_PDN, 0xF8);
		regmap_write(es8326->regmap, ES8326_VMIDSEL, 0x00);
		regmap_write(es8326->regmap, ES8326_INT_SOURCE, 0x0c);
		regmap_write(es8326->regmap, ES8326_SDINOUT1_IO, ES8326_IO_INPUT);
		regmap_write(es8326->regmap, ES8326_SDINOUT23_IO, ES8326_IO_INPUT);
		regmap_write(es8326->regmap, ES8326_RESET,
			     ES8326_CODEC_RESET | ES8326_PWRUP_SEQ_EN);
		break;
	}

	return 0;
}

#define es8326_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_dai_ops es8326_ops = {
	.prepare = es8326_stream_prepare,
	.shutdown = es8326_stream_shutdown,
	.hw_params = es8326_pcm_hw_params,
	.set_fmt = es8326_set_dai_fmt,
	.set_sysclk = es8326_set_dai_sysclk,
};

static struct snd_soc_dai_driver es8326_dai = {
	.name = "ES8326 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = es8326_FORMATS,
		},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = es8326_FORMATS,
		},
	.ops = &es8326_ops,
	.symmetric_rate = 1,
};

static void es8326_enable_micbias(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_mutex_lock(dapm);
	printk("Enter into %s\n", __func__);
	snd_soc_dapm_force_enable_pin_unlocked(dapm, "MICBIAS1");
	snd_soc_dapm_force_enable_pin_unlocked(dapm, "MICBIAS2");
	snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);
}

static void es8326_disable_micbias(struct snd_soc_component *component)
{
	struct snd_soc_dapm_context *dapm = snd_soc_component_get_dapm(component);

	snd_soc_dapm_mutex_lock(dapm);
	printk("Enter into %s\n", __func__);
	snd_soc_dapm_disable_pin_unlocked(dapm, "MICBIAS1");
	snd_soc_dapm_disable_pin_unlocked(dapm, "MICBIAS2");
	snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);
}

/*
 *	For button detection, set the following in soundcard
 *	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
 *	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOLUMEUP);
 *	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEDOWN);
 */
static void es8326_jack_button_handler(struct work_struct *work)
{
	struct es8326_priv *es8326 =
		container_of(work, struct es8326_priv, button_press_work.work);
	unsigned int iface;
	static int button_to_report, press_count;
	static int prev_button, cur_button;

	printk("Enter into %s\n", __func__);
	if (!(es8326->jack->status & SND_JACK_HEADSET)) /* Jack unplugged */
		return;

	mutex_lock(&es8326->lock);
	regmap_read(es8326->regmap, ES8326_HP_DETECT, &iface);
	switch (iface) {
	case 0x93:
		/* pause button detected */
		printk("%s, pause button!\n", __func__);
		cur_button = SND_JACK_BTN_0;
		break;
	case 0x6f:
	case 0x4b:
		/* button volume up */
		printk("%s, vol+ button!\n", __func__);
		cur_button = SND_JACK_BTN_1;
		break;
	case 0x27:
		/* button volume down */
		printk("%s, vol- button!\n", __func__);
		cur_button = SND_JACK_BTN_2;
		break;
	case 0x1e:
	case 0xe2:
		/* button released or not pressed */
		printk("%s, button release!\n", __func__);
		cur_button = 0;
		break;
	default:
		break;
	}

	if ((prev_button == cur_button) && (cur_button != 0)) {
		press_count++;
		if (press_count > 3) {
			/* report a press every 500ms */
			printk("%s, report button press event!\n", __func__);
			snd_soc_jack_report(es8326->jack, cur_button,
					SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2);
			press_count = 0;
		}
		button_to_report = cur_button;
		queue_delayed_work(system_wq, &es8326->button_press_work,
				   msecs_to_jiffies(30));
	} else if (prev_button != cur_button) {
		/* mismatch, detect again */
		prev_button = cur_button;
		queue_delayed_work(system_wq, &es8326->button_press_work,
				   msecs_to_jiffies(30));
	} else {
		/* released or no pressed */
		if (button_to_report != 0) {
			printk("%s, report button release event!\n", __func__);
			snd_soc_jack_report(es8326->jack, button_to_report,
				    SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2);
			snd_soc_jack_report(es8326->jack, 0,
				    SND_JACK_BTN_0 | SND_JACK_BTN_1 | SND_JACK_BTN_2);
			button_to_report = 0;
		}
	}
	mutex_unlock(&es8326->lock);
	printk("Exit %s\n", __func__);
}

static void es8326_jack_detect_handler(struct work_struct *work)
{
	struct es8326_priv *es8326 =
		container_of(work, struct es8326_priv, jack_detect_work.work);
	struct snd_soc_component *comp = es8326->component;
	unsigned int iface;

	mutex_lock(&es8326->lock);
	printk("Enter into %s\n", __func__);
	if(es8326->hp == 0){
		printk("Enter into %s,hp=%d\n", __func__, es8326->hp);
		msleep(110);
	}
	regmap_read(es8326->regmap, ES8326_HP_DETECT, &iface);
	dev_dbg(comp->dev, "gpio flag %#04x", iface);
	if ((iface & ES8326_HPINSERT_FLAG) == 0) {
		/* Jack unplugged or spurious IRQ */
		dev_dbg(comp->dev, "No headset detected");
		if (es8326->jack->status & SND_JACK_HEADPHONE) {
			snd_soc_jack_report(es8326->jack, 0, SND_JACK_HEADSET);
			snd_soc_component_write(comp, ES8326_SDINOUT23_IO, 0x00);
			snd_soc_component_write(comp, ES8326_ADC1_SRC, 0x44);
			snd_soc_component_write(comp, ES8326_ADC2_SRC, 0x66);
			es8326_disable_micbias(comp);
			es8326->hp =0;
		}
	} else if ((iface & ES8326_HPINSERT_FLAG) == ES8326_HPINSERT_FLAG) {
		if (es8326->jack->status & SND_JACK_HEADSET) {
			/* detect button */
			queue_delayed_work(system_wq, &es8326->button_press_work, 10);
		} else {
			if ((iface & ES8326_HPBUTTON_FLAG) == 0x00) {
				dev_dbg(comp->dev, "Headset detected");
				snd_soc_jack_report(es8326->jack,
						    SND_JACK_HEADSET, SND_JACK_HEADSET);
				snd_soc_component_update_bits(comp, ES8326_PGA_PDN,
						0x08, 0x08);
				snd_soc_component_update_bits(comp, ES8326_PGAGAIN,
						0x80, 0x80);
				snd_soc_component_write(comp, ES8326_ADC1_SRC, 0x00);
				snd_soc_component_write(comp, ES8326_ADC2_SRC, 0x00);
				msleep(10);
				snd_soc_component_update_bits(comp, ES8326_PGA_PDN,
						0x08, 0x00);
			} else {
				dev_dbg(comp->dev, "Headphone detected");
				snd_soc_jack_report(es8326->jack,
						    SND_JACK_HEADPHONE, SND_JACK_HEADSET);
			}
			es8326->hp = 1;
		}
	}
	mutex_unlock(&es8326->lock);
}

static irqreturn_t es8326_irq(int irq, void *dev_id)
{
	struct es8326_priv *es8326 = dev_id;
	struct snd_soc_component *comp = es8326->component;

	printk("Enter into %s\n", __func__);
	if (!es8326->jack)
		goto out;

	es8326_enable_micbias(comp);

	if (es8326->jack->status & SND_JACK_HEADSET)
		queue_delayed_work(system_wq, &es8326->jack_detect_work,
				   msecs_to_jiffies(10));
	else
		queue_delayed_work(system_wq, &es8326->jack_detect_work,
				   msecs_to_jiffies(300));

out:
	return IRQ_HANDLED;
}

static int es8326_resume(struct snd_soc_component *component)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	unsigned int reg;

	printk("Enter into %s\n", __func__);
	regcache_cache_only(es8326->regmap, false);
	regcache_sync(es8326->regmap);

	regmap_write(es8326->regmap, ES8326_RESET, 0x17);
	regmap_write(es8326->regmap, ES8326_CLK_CTL, ES8326_CLK_ON);
	regmap_write(es8326->regmap, ES8326_PULLUP_CTL, 0x00);
	/* Two channel ADC */
	regmap_write(es8326->regmap, ES8326_CLK_INV, 0x00);
	regmap_write(es8326->regmap, ES8326_CLK_RESAMPLE, 0x05);
	regmap_write(es8326->regmap, ES8326_CLK_DIV1, 0x01);
	regmap_write(es8326->regmap, ES8326_CLK_DIV2, 0x00);
	regmap_write(es8326->regmap, ES8326_CLK_DLL, 0x30);
	regmap_write(es8326->regmap, ES8326_CLK_MUX, 0x2D);
	regmap_write(es8326->regmap, ES8326_CLK_ADC_SEL, 0x26);
	regmap_write(es8326->regmap, ES8326_CLK_DAC_SEL, 0x26);
	regmap_write(es8326->regmap, ES8326_CLK_ADC_OSR, 0x1F);
	regmap_write(es8326->regmap, ES8326_CLK_DAC_OSR, 0x1F);
	regmap_write(es8326->regmap, ES8326_CLK_DIV_CPC, 0x04);
	regmap_write(es8326->regmap, ES8326_CLK_INV, 0x00);
	regmap_write(es8326->regmap, ES8326_CLK_DIV_CPC, 0x84);
	regmap_write(es8326->regmap, ES8326_CLK_VMIDS1, 0xC4);
	regmap_write(es8326->regmap, ES8326_CLK_VMIDS2, 0x81);
	regmap_write(es8326->regmap, ES8326_CLK_CAL_TIME, 0x04);
	regmap_write(es8326->regmap, ES8326_SYS_BIAS, 0x08);
	regmap_write(es8326->regmap, ES8326_PGAGAIN, 0x10);
	regmap_write(es8326->regmap, ES8326_ANA_VSEL, 0x7F);////
	regmap_write(es8326->regmap, ES8326_HP_DRIVER, 0xAF);//time
	regmap_write(es8326->regmap, ES8326_DAC2HPMIX, 0x22);//SNR
	regmap_write(es8326->regmap, ES8326_HP_DRIVER_REF, 0x83);//THD
	regmap_write(es8326->regmap, ES8326_SYS_BIAS, 0x08);
	regmap_write(es8326->regmap, ES8326_DAC2HPMIX, 0x22);
	regmap_write(es8326->regmap, ES8326_ADC1_SRC, 0x44);
	regmap_write(es8326->regmap, ES8326_ADC2_SRC, 0x66);
	regmap_write(es8326->regmap, ES8326_HPJACK_TIMER, 0x88);
	regmap_write(es8326->regmap, ES8326_HP_DET, 0x81 | ES8326_HP_DET_SRC_PIN9);
	regmap_write(es8326->regmap, ES8326_INT_SOURCE, 0x0c);
	regmap_write(es8326->regmap, ES8326_INTOUT_IO, es8326->interrupt_clk);
	regmap_write(es8326->regmap, ES8326_RESET, ES8326_CSM_ON);
	snd_soc_component_update_bits(component, ES8326_PGAGAIN,
				      ES8326_MIC_SEL_MASK, ES8326_MIC1_SEL);
	msleep(15);
	regmap_write(es8326->regmap, ES8326_DAC2HPMIX, 0xAA);//B00
	regmap_write(es8326->regmap, ES8326_HP_CAL, 0x00);
	msleep(50);
	regmap_write(es8326->regmap, ES8326_ANA_MICBIAS, 0x4C);//
	regmap_write(es8326->regmap, ES8326_HPJACK_TIMER, 0xF8); //F8
	regmap_write(es8326->regmap, ES8326_HP_VOL, 0x00);
	regmap_write(es8326->regmap, ES8326_HP_DRIVER_REF, 0x37);//B7
	regmap_write(es8326->regmap, ES8326_HP_MISC, 0x0c);//0C
	regmap_write(es8326->regmap, ES8326_HP_DET, 0x80 | ES8326_HP_DET_SRC_PIN9);
	regmap_read(es8326->regmap, ES8326_CHIP_VERSION, &reg);
	if ((reg & ES8326_VERSION_B) == 1) {
		regmap_write(es8326->regmap, ES8326_ANA_MICBIAS, 0xEC);
		regmap_write(es8326->regmap, ES8326_ANA_VSEL, 0x7F);
		regmap_write(es8326->regmap, ES8326_VMIDLOW, 0x0F);
		/* enable button detect */
		regmap_write(es8326->regmap, ES8326_HP_DRIVER, 0xA0);
		regmap_write(es8326->regmap, ES8326_HP_VOL, 0x80);
		regmap_write(es8326->regmap, ES8326_HP_DRIVER_REF, 0x97);
	}
	msleep(150);
	regmap_write(es8326->regmap, ES8326_HP_CAL, 0x00);
	es8326_irq(es8326->irq, es8326);
	return 0;
}

static int es8326_suspend(struct snd_soc_component *component)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);

	printk("Enter into %s\n", __func__);
	cancel_delayed_work_sync(&es8326->jack_detect_work);
	es8326_disable_micbias(component);

	regmap_write(es8326->regmap, ES8326_CLK_CTL, ES8326_CLK_OFF);
	regcache_cache_only(es8326->regmap, true);
	regcache_mark_dirty(es8326->regmap);

	return 0;
}

static int es8326_probe(struct snd_soc_component *component)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	int ret;

	printk("Enter into %s\n", __func__);
	es8326->component = component;
	es8326->jd_inverted = device_property_read_bool(component->dev,
							"everest,jack-detect-inverted");
	es8326->calidone = 0;

	ret = device_property_read_u8(component->dev, "everest,mic1-src", &es8326->mic1_src);
	if (ret != 0) {
		dev_dbg(component->dev, "mic1-src return %d", ret);
		es8326->mic1_src = ES8326_ADC_AMIC;
	}
	dev_dbg(component->dev, "mic1-src %x", es8326->mic1_src);

	ret = device_property_read_u8(component->dev, "everest,mic2-src", &es8326->mic2_src);
	if (ret != 0) {
		dev_dbg(component->dev, "mic2-src return %d", ret);
		es8326->mic2_src = ES8326_ADC_DMIC;
	}
	dev_dbg(component->dev, "mic2-src %x", es8326->mic2_src);

	ret = device_property_read_u8(component->dev, "everest,jack-pol", &es8326->jack_pol);
	if (ret != 0) {
		dev_dbg(component->dev, "jack-pol return %d", ret);
		es8326->jack_pol = ES8326_HP_DET_BUTTON_POL | ES8326_HP_TYPE_OMTP;
	}
	dev_dbg(component->dev, "jack-pol %x", es8326->jack_pol);

	ret = device_property_read_u8(component->dev, "everest,interrupt-src", &es8326->jack_pol);
	if (ret != 0) {
		dev_dbg(component->dev, "interrupt-src return %d", ret);
		es8326->interrupt_src = ES8326_HP_DET_SRC_PIN9;
	}
	dev_dbg(component->dev, "interrupt-src %x", es8326->interrupt_src);

	ret = device_property_read_u8(component->dev, "everest,interrupt-clk", &es8326->jack_pol);
	if (ret != 0) {
		dev_dbg(component->dev, "interrupt-clk return %d", ret);
		es8326->interrupt_clk = 0x45;
	}
	dev_dbg(component->dev, "interrupt-clk %x", es8326->interrupt_clk);

	es8326_resume(component);
	return 0;
}

static void es8326_enable_jack_detect(struct snd_soc_component *component,
				struct snd_soc_jack *jack)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);
	printk("Enter into %s\n", __func__);
	mutex_lock(&es8326->lock);
	if (es8326->jd_inverted)
		snd_soc_component_update_bits(component, ES8326_HP_DET,
				ES8326_HP_DET_JACK_POL, ~es8326->jack_pol);
	es8326->jack = jack;

	mutex_unlock(&es8326->lock);
	es8326_irq(es8326->irq, es8326);
}

static void es8326_disable_jack_detect(struct snd_soc_component *component)
{
	struct es8326_priv *es8326 = snd_soc_component_get_drvdata(component);

	printk("Enter into %s\n", __func__);
	if (!es8326->jack)
		return; /* Already disabled (or never enabled) */
	cancel_delayed_work_sync(&es8326->jack_detect_work);

	mutex_lock(&es8326->lock);
	if (es8326->jack->status & SND_JACK_MICROPHONE) {
		es8326_disable_micbias(component);
		snd_soc_jack_report(es8326->jack, 0, SND_JACK_HEADSET);
	}
	es8326->jack = NULL;
	mutex_unlock(&es8326->lock);
}

static int es8326_set_jack(struct snd_soc_component *component,
			struct snd_soc_jack *jack, void *data)
{
	printk("Enter into %s\n", __func__);
	if (jack)
		es8326_enable_jack_detect(component, jack);
	else
		es8326_disable_jack_detect(component);

	return 0;
}

static void es8326_remove(struct snd_soc_component *component)
{
	es8326_disable_jack_detect(component);
	es8326_set_bias_level(component, SND_SOC_BIAS_OFF);
}

static const struct snd_soc_component_driver soc_component_dev_es8326 = {
	.probe		= es8326_probe,
	.remove		= es8326_remove,
	.resume		= es8326_resume,
	.suspend	= es8326_suspend,
	.set_bias_level = es8326_set_bias_level,
	.set_jack	= es8326_set_jack,
	.dapm_widgets	= es8326_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8326_dapm_widgets),
	.dapm_routes		= es8326_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es8326_dapm_routes),
	.controls		= es8326_snd_controls,
	.num_controls		= ARRAY_SIZE(es8326_snd_controls),
	.use_pmdown_time	= 1,
	.endianness		= 1,
};

static int es8326_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8326_priv *es8326;
	int ret;

	es8326 = devm_kzalloc(&i2c->dev, sizeof(struct es8326_priv), GFP_KERNEL);
	if (!es8326)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es8326);
	es8326->i2c = i2c;
	mutex_init(&es8326->lock);
	es8326->regmap = devm_regmap_init_i2c(i2c, &es8326_regmap_config);
	if (IS_ERR(es8326->regmap)) {
		ret = PTR_ERR(es8326->regmap);
		dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}

	es8326->irq = i2c->irq;
	INIT_DELAYED_WORK(&es8326->jack_detect_work,
			  es8326_jack_detect_handler);
	INIT_DELAYED_WORK(&es8326->button_press_work,
			  es8326_jack_button_handler);
	/* ES8316 is level-based while ES8326 is edge-based */
	ret = devm_request_threaded_irq(&i2c->dev, es8326->irq, NULL, es8326_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"es8326", es8326);
	if (ret) {
		dev_warn(&i2c->dev, "Failed to request IRQ: %d: %d\n",
		es8326->irq, ret);
		es8326->irq = -ENXIO;
	}

	es8326->mclk = devm_clk_get_optional(&i2c->dev, "mclk");
	if (IS_ERR(es8326->mclk)) {
		dev_err(&i2c->dev, "unable to get mclk\n");
		return PTR_ERR(es8326->mclk);
	}
	if (!es8326->mclk)
		dev_warn(&i2c->dev, "assuming static mclk\n");

	ret = clk_prepare_enable(es8326->mclk);
	if (ret) {
		dev_err(&i2c->dev, "unable to enable mclk\n");
		return ret;
	}
	return devm_snd_soc_register_component(&i2c->dev,
					&soc_component_dev_es8326,
					&es8326_dai, 1);
}

static const struct i2c_device_id es8326_i2c_id[] = {
	{"es8326", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, es8326_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id es8326_of_match[] = {
	{ .compatible = "everest,es8326", },
	{}
};
MODULE_DEVICE_TABLE(of, es8326_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id es8326_acpi_match[] = {
	{"ESSX8326", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, es8326_acpi_match);
#endif

static struct i2c_driver es8326_i2c_driver = {
	.driver = {
		.name = "es8326",
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(es8326_acpi_match),
#endif
		.of_match_table = of_match_ptr(es8326_of_match),
	},
	.probe = es8326_i2c_probe,
	.id_table = es8326_i2c_id,
};
module_i2c_driver(es8326_i2c_driver);

MODULE_DESCRIPTION("ASoC es8326 driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com>");
MODULE_LICENSE("GPL");
