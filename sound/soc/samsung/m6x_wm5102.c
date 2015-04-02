/*
 *  exynos_wm5102.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <sound/pcm_params.h>
#include <linux/module.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>
#include <plat/clock.h>
#include <plat/s5p-clock.h>
#include <mach/pmu.h>
#include <mach/regs-clock.h>

#include <linux/init.h>
#include <linux/printk.h>
#include <linux/suspend.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/core.h>

#include <linux/mfd/arizona/core.h>
#include <linux/mfd/arizona/registers.h>

#include "../codecs/wm5102.h"
#include "../codecs/arizona.h"
#include "../codecs/wm_adsp.h"
#include "pcm.h"
#include "i2s.h"

#define M6X_WM5102_MCLK1_FREQ 24000000
#define M6X_WM5102_INCALL_SYSCLK_FREQ 24576000

static int incall;
static struct clk *mclk;
static struct clk *xxti_clk;
static struct regulator *apreg;

static void m6x_snd_set_mclk(bool on)
{
    if (on) {
        pr_info("Sound: enabled mclk\n");
        clk_enable(mclk);
        msleep(10);
    } else {
        pr_info("Sound: disabled mclk\n");
        clk_disable(mclk);
    }

    pr_debug("Sound: use_cnt: %d\n", mclk->usage);
}

#if 0
static void m6x_snd_set_apreg(bool on)
{
	if (on) {
		pr_info("Sound: enabled apreg\n");
		regulator_enable(apreg);
	} else {
		pr_info("Sound: disabled apreg\n");
		regulator_disable(apreg);
	}
}
#endif

static bool m6x_snd_get_mclk(void)
{
	return (mclk->usage > 0);
}

static int m6x_aif1_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_out = M6X_WM5102_MCLK1_FREQ;
	int ret;

	printk("++%s\n", __func__);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	switch (params_rate(params)) {
	case 96000:
	case 64000:
	case 48000:
	case 32000:
	case 16000:
	case 8000:
		pll_out = 24576000;
		break;
	case 88200:
	case 44100:
	case 22050:
	case 11025:
		pll_out = 22579200;

		break;

	default:
		dev_err(cpu_dai->dev,  "Unsupported SampleRate, ret = %d\n", -EINVAL);
		return -EINVAL;
	}

	printk("++%s format = %d rate = %d\n", __func__, params_format(params),(params_rate(params)));

	if (pll_out <= 0) {
		dev_err(cpu_dai->dev, "Failed to set AIF1 FLL OUT: %d\n", pll_out);
		return -EINVAL;
	}

	/* set the codec FLL */
	ret = snd_soc_dai_set_pll(codec_dai, WM5102_FLL1, ARIZONA_FLL_SRC_MCLK1, M6X_WM5102_MCLK1_FREQ, pll_out);
	if (ret != 0) {
		dev_err(cpu_dai->dev, "Failed to set AIF1 FLL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(codec_dai->codec, ARIZONA_CLK_ASYNCCLK, ARIZONA_FLL_SRC_FLL1, pll_out, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		pr_err("%s: Failed to switch to FLL2: %d\n", __func__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_ASYNCCLK, pll_out / 2, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(cpu_dai->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	printk("--%s\n", __func__);

	return 0;
}

#undef CONFIG_SND_SAMSUNG_I2S_MASTER

#ifdef CONFIG_SND_SAMSUNG_I2S_MASTER
static int set_epll_rate(unsigned long rate)
{
	struct clk *fout_epll;

	fout_epll = clk_get(NULL, "fout_epll");
	if (IS_ERR(fout_epll)) {
		printk(KERN_ERR "%s: failed to get fout_epll\n", __func__);
		return PTR_ERR(fout_epll);
	}

	if (rate == clk_get_rate(fout_epll))
		goto out;

	clk_set_rate(fout_epll, rate);
out:
	clk_put(fout_epll);

	return 0;
}
#endif /* CONFIG_SND_SAMSUNG_I2S_MASTER */

#ifndef CONFIG_SND_SAMSUNG_I2S_MASTER
static int m6x_aif1_sec_fifo_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_out = M6X_WM5102_MCLK1_FREQ;
	int ret;

	printk("++%s\n", __func__);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	switch (params_rate(params)) {
	case 96000:
	case 64000:
	case 48000:
	case 32000:
	case 16000:
	case 8000:
		pll_out = 24576000;
		break;
	case 88200:
	case 44100:
	case 22050:
	case 11025:
		pll_out = 22579200;

		break;

	default:
		dev_err(cpu_dai->dev,  "Unsupported SampleRate, ret = %d\n", -EINVAL);
		return -EINVAL;
	}

	printk("++%s format = %d rate = %d\n", __func__, params_format(params),(params_rate(params)));

	if (pll_out <= 0) {
		dev_err(cpu_dai->dev, "Failed to set AIF1 FLL OUT: %d\n", pll_out);
		return -EINVAL;
	}

	/* set the codec FLL */
	ret = snd_soc_dai_set_pll(codec_dai, WM5102_FLL1, ARIZONA_FLL_SRC_MCLK1, M6X_WM5102_MCLK1_FREQ, pll_out);
	if (ret != 0) {
		dev_err(cpu_dai->dev, "Failed to set AIF1 FLL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_codec_set_sysclk(codec_dai->codec, ARIZONA_CLK_ASYNCCLK, ARIZONA_FLL_SRC_FLL1, pll_out, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		pr_err("%s: Failed to switch to FLL2: %d\n", __func__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_ASYNCCLK, pll_out / 2, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(cpu_dai->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	printk("--%s\n", __func__);

	return 0;
}
#else
static int m6x_aif1_sec_fifo_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int bfs, psr, rfs, ret;
	unsigned long rclk;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_U24:
	case SNDRV_PCM_FORMAT_S24:
		bfs = 48;
		break;
	case SNDRV_PCM_FORMAT_U16_LE:
	case SNDRV_PCM_FORMAT_S16_LE:
		bfs = 32;
		break;
	default:
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 16000:
	case 22050:
	case 24000:
	case 32000:
	case 44100:
	case 48000:
	case 88200:
	case 96000:
		if (bfs == 48)
			rfs = 384;
		else
			rfs = 256;
		break;
	case 64000:
		rfs = 384;
		break;
	case 8000:
	case 11025:
	case 12000:
		if (bfs == 48)
			rfs = 768;
		else
			rfs = 512;
		break;
	default:
		return -EINVAL;
	}

	rclk = params_rate(params) * rfs;

	switch (rclk) {
	case 4096000:
	case 5644800:
	case 6144000:
	case 8467200:
	case 9216000:
		psr = 8;
		break;
	case 8192000:
	case 11289600:
	case 12288000:
	case 16934400:
	case 18432000:
		psr = 4;
		break;
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		psr = 2;
		break;
	case 67737600:
	case 73728000:
		psr = 1;
		break;
	default:
		printk("Not yet supported!\n");
		return -EINVAL;
	}

	set_epll_rate(rclk * psr);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					| SND_SOC_DAIFMT_NB_NF
					| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_ASYNCCLK,
					rclk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SAMSUNG_I2S_DIV_BCLK, bfs);
	if (ret < 0)
		return ret;

	return 0;
}
#endif

static void sec_fifo_shutdown(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	ret = arizona_set_sysclk(rtd->codec, ARIZONA_CLK_ASYNCCLK, ARIZONA_FLL_SRC_FLL1, 0, 0);
	if (ret < 0) {
		printk("Failed to stop sysclk: %d\n", ret);
		return;
	}

	return;
}


static int m6x_modem_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret = 0;

	printk("++%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	printk("--%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);

	return 0;
}

static int m6x_modem_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;
	incall++;

	printk("++%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);
	ret = arizona_set_fll(&g_wm5102->fll[0], 0, 0, 0);
	if (ret != 0) {
		pr_err("failed to set fll, line %d\n", __LINE__);
		return -EIO;
	}
	ret = arizona_set_fll(&g_wm5102->fll[0], ARIZONA_FLL_SRC_MCLK1, M6X_WM5102_MCLK1_FREQ, M6X_WM5102_INCALL_SYSCLK_FREQ);
	if (ret != 0) {
		pr_err("failed to set fll, line %d\n", __LINE__);
		return -EIO;
	}
	ret = arizona_set_sysclk(codec_dai->codec, ARIZONA_CLK_SYSCLK, ARIZONA_FLL_SRC_FLL1, M6X_WM5102_INCALL_SYSCLK_FREQ, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		pr_err("failed to set fll, line %d\n", __LINE__);
		return -EIO;
	}
	printk("--%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);

	return 0;
}

static void m6x_modem_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	incall--;

	arizona_set_sysclk(codec_dai->codec, ARIZONA_CLK_SYSCLK, 0, 0, 0);
}

static int m6x_pa_speaker_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	return 0;
}

static int invoip;
static int m6x_voip_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	printk("++%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A
						| SND_SOC_DAIFMT_IB_NF
						| SND_SOC_DAIFMT_CONT
						| SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, S3C_PCM_CLKSRC_PCLK, 24576000, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(cpu_dai->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	printk("--%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);

	return 0;
}

static int m6x_voip_startup(struct snd_pcm_substream *substream)
{

	printk("++%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);
	printk("--%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);
	invoip++;

	return 0;
}

static void m6x_voip_shutdown(struct snd_pcm_substream *substream)
{
	invoip--;
	printk("++%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);
	printk("--%s() in %s, Line: %d\n", __func__, __FILE__, __LINE__);
}

/*
 * EXYNOS wm5102 DAI operations.
 */
static struct snd_soc_ops m6x_aif1_hw_ops = {
	.hw_params = m6x_aif1_hw_params,
};
static struct snd_soc_ops m6x_aif1_sec_fifo_hw_ops = {
	.hw_params = m6x_aif1_sec_fifo_hw_params,
	.shutdown = sec_fifo_shutdown,
};
static struct snd_soc_ops m6x_modem_ops = {
	.hw_params = m6x_modem_hw_params,
	.startup = m6x_modem_startup,
	.shutdown = m6x_modem_shutdown,
};
static struct snd_soc_ops m6x_aif3_ops = {
	.hw_params = m6x_pa_speaker_hw_params,
};
static struct snd_soc_ops m6x_voip_ops = {
	.hw_params = m6x_voip_hw_params,
	.startup = m6x_voip_startup,
	.shutdown = m6x_voip_shutdown,
};

static const struct snd_kcontrol_new controls[] = {
	SOC_DAPM_PIN_SWITCH("IN1L"),
	SOC_DAPM_PIN_SWITCH("IN1R"),
	SOC_DAPM_PIN_SWITCH("IN2L"),
	SOC_DAPM_PIN_SWITCH("IN3R"),

	SOC_DAPM_PIN_SWITCH("HPOUT1L"),
	SOC_DAPM_PIN_SWITCH("HPOUT1R"),
	SOC_DAPM_PIN_SWITCH("EPOUT"),
};

#define WM5102_RATES SNDRV_PCM_RATE_8000_192000
#define WM5102_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver m6x_ext_dai[] = {
	{
		.name = "m6x.voice",
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = WM5102_RATES,
			.formats = WM5102_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = WM5102_RATES,
			.formats = WM5102_FORMATS,
		},
	},
	{
		.name = "m6x.pa",
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = WM5102_RATES,
			.formats = WM5102_FORMATS,
		},
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = WM5102_RATES,
			.formats = WM5102_FORMATS,
		},
	},
};

static struct snd_soc_dai_link m6x_dai_wm5102[] = {
	{
		.name = "WM5102 AIF1",
		.stream_name = "AIF1 Stream",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm5102-aif1",
		.platform_name = "samsung-audio",
		.codec_name = "wm5102-codec",
		.ops = &m6x_aif1_hw_ops,
	}, { /* Sec_Fifo DAI i/f */
		.name = "Sec_FIFO TX",
		.stream_name = "Sec_Dai",
		.cpu_dai_name = "samsung-i2s.4",
		.codec_dai_name = "wm5102-aif1",
#if defined (CONFIG_SND_SAMSUNG_LP) || defined(CONFIG_SND_SAMSUNG_ALP)
		.platform_name = "samsung-idma",
#else
		.platform_name = "samsung-audio",
#endif
		.codec_name = "wm5102-codec",
		.ops = &m6x_aif1_sec_fifo_hw_ops,
	},
	{
		.name = "WM5102 Modem",
		.stream_name = "Modem Stream",
		.cpu_dai_name = "m6x.voice",
		.codec_dai_name = "wm5102-aif2",
		.platform_name = "snd-soc-dummy",
		.codec_name = "wm5102-codec",
		.ops = &m6x_modem_ops,
	},
	{
		.name = "WM5102 PA",
		.stream_name = "PA Tx",
		.cpu_dai_name = "m6x.pa",
		.codec_dai_name = "wm5102-aif3",
		.platform_name = "snd-soc-dummy",
		.codec_name = "wm5102-codec",
		.ops = &m6x_aif3_ops,
	},
	{
		.name = "WM5102 voip",
		.stream_name = "voip Stream",
		.cpu_dai_name = "samsung-pcm.0",
		.codec_dai_name = "wm5102-aif2",
		.platform_name = "samsung-audio",
		.codec_name = "wm5102-codec",
		.ops = &m6x_voip_ops,
	},
};

static int m6x_card_suspend_pre(struct snd_soc_card *card)
{
    return 0;
}

static int m6x_card_suspend_post(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec = card->rtd->codec;

	if (!codec->active) {
		m6x_snd_set_mclk(false);
	}

	//exynos_xxti_sys_powerdown((m6x_snd_get_mclk())? 1: 0);
	exynos_xxti_sys_powerdown_enable((m6x_snd_get_mclk())? 1: 0);

	return 0;
}

static int m6x_card_resume_pre(struct snd_soc_card *card)
{
	if (!clk_is_enabled(mclk))
		m6x_snd_set_mclk(true);

	return 0;
}

static int m6x_card_resume_post(struct snd_soc_card *card)
{
	return 0;
}

static int m6x_set_bias_level(struct snd_soc_card *card,
			      struct snd_soc_dapm_context *dapm,
			      enum snd_soc_bias_level level)
{
	int ret = 0;
	struct snd_soc_codec *codec = card->rtd->codec;

	switch (level) {
	/* Fix the issue that power consumption increase after playing music with HEADSET/HEADPHONE */
	case SND_SOC_BIAS_STANDBY:
		if (codec->active)
			break;
	case SND_SOC_BIAS_OFF:
		ret = snd_soc_codec_set_pll(codec, WM5102_FLL1, ARIZONA_CLK_SRC_MCLK1,
						M6X_WM5102_MCLK1_FREQ, 0);
		if (ret < 0) {
			printk("Failed to stop FLL1: %d\n", ret);
			return ret;
		}
		ret = snd_soc_codec_set_pll(codec, WM5102_FLL2, ARIZONA_CLK_SRC_MCLK1,
						M6X_WM5102_MCLK1_FREQ, 0);
		if (ret < 0) {
			printk("Failed to stop FLL2: %d\n", ret);
			return ret;
		}
		break;
	default:
		break;
	}

	card->dapm.bias_level = level;

	return ret;
}

static int m6x_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec = card->rtd[0].codec;

	snd_soc_dapm_disable_pin(&codec->dapm, "IN1L");
	snd_soc_dapm_disable_pin(&codec->dapm, "IN1R");
	snd_soc_dapm_disable_pin(&codec->dapm, "IN2L");
	snd_soc_dapm_disable_pin(&codec->dapm, "IN3R");

	snd_soc_dapm_disable_pin(&codec->dapm, "HPOUT1L");
	snd_soc_dapm_disable_pin(&codec->dapm, "HPOUT1R");
	snd_soc_dapm_disable_pin(&codec->dapm, "EPOUT");

	return snd_soc_dapm_sync(&codec->dapm);
}

static struct snd_soc_card m6x_card = {
	.name = "Exynos WM5102",
	.owner = THIS_MODULE,
	.dai_link = m6x_dai_wm5102,
	.num_links = ARRAY_SIZE(m6x_dai_wm5102),
	.suspend_pre = m6x_card_suspend_pre,
	.suspend_post = m6x_card_suspend_post,
	.resume_pre = m6x_card_resume_pre,
	.resume_post = m6x_card_resume_post,
	.set_bias_level = m6x_set_bias_level,
	.late_probe = m6x_late_probe,

	.controls = controls,
	.num_controls = ARRAY_SIZE(controls),
};

extern int aif1_rx_suspend_resume(struct notifier_block *, unsigned long, void *);
struct platform_device *m6x_snd_device;
static int __devinit m6x_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct m6x_card_data *m6x_card_data;

	printk("%s++\n", __func__);

	mclk = clk_get(NULL, "clkout");
	xxti_clk = clk_get(NULL, "xxti");
	clk_set_parent(mclk, xxti_clk);
	m6x_snd_set_mclk(true);
	apreg = regulator_get(NULL, "ap_32khz");
	if (IS_ERR(apreg)) {
		pr_err("Sound: Getting EN32KHz CP regulator is failed\n");
	}

	m6x_snd_device = platform_device_alloc("soc-audio", 0);
	if (!m6x_snd_device)
		return -ENOMEM;

	ret = snd_soc_register_dais(&m6x_snd_device->dev, m6x_ext_dai,
						ARRAY_SIZE(m6x_ext_dai));
	if (ret != 0)
		pr_err("Failed to register external DAIs: %d\n", ret);

	platform_set_drvdata(m6x_snd_device, &m6x_card);
	ret = platform_device_add(m6x_snd_device);
	if (ret)
		platform_device_put(m6x_snd_device);
	else {
		m6x_card_data = kzalloc(sizeof(struct m6x_card_data), GFP_KERNEL);
		if (m6x_card_data) {
			m6x_card_data->nb.notifier_call = aif1_rx_suspend_resume;
			m6x_card_data->nb.priority = 0;
			m6x_card_data->card = &m6x_card;
			snd_soc_card_set_drvdata(&m6x_card, m6x_card_data);
			register_pm_notifier(&m6x_card_data->nb);
		}
	}
	printk("%s--\n", __func__);

	return ret;
}

static int __devexit m6x_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct m6x_card_data *m6x_card_data;

	card = platform_get_drvdata(m6x_snd_device);
	m6x_card_data = snd_soc_card_get_drvdata(card);
	if (m6x_card_data) {
		unregister_pm_notifier(&m6x_card_data->nb);
		kfree(m6x_card_data);
	}

	platform_device_unregister(m6x_snd_device);
	regulator_put(apreg);
	clk_put(mclk);
	clk_put(xxti_clk);

	return 0;
}

static struct platform_driver m6x_audio_driver = {
	.driver		= {
		.name	= "m6x-audio",
		.owner	= THIS_MODULE,
	},
	.probe		= m6x_audio_probe,
	.remove		= __devexit_p(m6x_audio_remove),
};

static int __init m6x_audio_init(void)
{
	int ret;

	printk("%s++\n", __func__);

	ret = platform_driver_register(&m6x_audio_driver);
	if (ret)
		return -ENOMEM;
	printk("%s--\n", __func__);
	return ret;
}

static void __exit m6x_audio_exit(void)
{
	platform_driver_unregister(&m6x_audio_driver);
}

module_init(m6x_audio_init);
module_exit(m6x_audio_exit);

MODULE_DESCRIPTION("ALSA SoC EXYNOS wm5102");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-audio");
