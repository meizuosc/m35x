/*
 *  tf4_wm1811.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/regulator/machine.h>
#include <linux/delay.h>

#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/jack.h>

#include <mach/regs-clock.h>
#include <mach/pmu.h>
#include <mach/tf4-sound.h>

#include <plat/clock.h>
#include <plat/s5p-clock.h>

#include "i2s.h"
#include "i2s-regs.h"
#include "s3c-i2s-v2.h"
#include "../codecs/wm8994.h"
#include <sound/pcm_params.h>

#include <linux/mfd/wm8994/core.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/mfd/wm8994/pdata.h>

#define TF4_WM1811_MCLK1_FREQ       (24000000)
#define TF4_WM1811_MCLK2_FREQ       (32768)
#define TF4_WM1811_SYNC_CLK_FREQ    (11289600)

#define WM1811_JACKDET_MODE_NONE  0x0000
#define WM1811_JACKDET_MODE_JACK  0x0100
#define WM1811_JACKDET_MODE_MIC   0x0080
#define WM1811_JACKDET_MODE_AUDIO 0x0180

#define WM1811_JACKDET_BTN0 0x04
#define WM1811_JACKDET_BTN1 0x10
#define WM1811_JACKDET_BTN2 0x08

struct wm1811_machine_priv {
	struct snd_soc_jack jack;
	struct snd_soc_codec *codec;
	struct delayed_work mic_work;
	struct wake_lock jackdet_wake_lock;
};

static struct wm8958_micd_rate tf4_det_rates[] = {
    { TF4_WM1811_MCLK2_FREQ,     true,  0,  0 },
    { TF4_WM1811_MCLK2_FREQ,    false,  0,  0 },
    { TF4_WM1811_SYNC_CLK_FREQ,  true,  7,  7 },
    { TF4_WM1811_SYNC_CLK_FREQ, false,  7,  7 },
};

static struct wm8958_micd_rate tf4_jackdet_rates[] = {
    { TF4_WM1811_MCLK2_FREQ,     true,  0,  0 },
    { TF4_WM1811_MCLK2_FREQ,    false,  0,  0 },
    { TF4_WM1811_SYNC_CLK_FREQ,  true, 12, 12 },
    { TF4_WM1811_SYNC_CLK_FREQ, false,  7,  8 },
};

static struct clk *mclk;
static struct clk *xxti_clk;
static struct regulator *cpreg;

static void tf4_snd_set_mclk(bool on)
{
    if (on) {
        pr_info("Sound: enabled mclk\n");
        clk_enable(mclk);
        msleep(10);
    } else {
        pr_info("Sound: disabled mclk\n");
        clk_disable(mclk);
    }

    pr_info("Sound: use_cnt: %d\n", mclk->usage);
}

static void tf4_snd_set_cpreg(bool on)
{
	if (on) {
		pr_info("Sound: enabled cpreg\n");
		regulator_enable(cpreg);
	} else {
		pr_info("Sound: disabled cpreg\n");
		regulator_disable(cpreg);
	}
}

static bool tf4_snd_get_mclk(void)
{
        return (mclk->usage > 0);
}

static void tf4_micd_set_rate(struct snd_soc_codec *codec)
{
    struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(codec);
    int best, i, sysclk, val;
    bool idle;
    const struct wm8958_micd_rate *rates = NULL;
    int num_rates = 0;

    idle = !wm8994->jack_mic;

    sysclk = snd_soc_read(codec, WM8994_CLOCKING_1);
    if (sysclk & WM8994_SYSCLK_SRC)
        sysclk = wm8994->aifclk[1];
    else
        sysclk = wm8994->aifclk[0];

    if (wm8994->jackdet) {
        rates = tf4_jackdet_rates;
        num_rates = ARRAY_SIZE(tf4_jackdet_rates);
        wm8994->pdata->micd_rates = tf4_jackdet_rates;
        wm8994->pdata->num_micd_rates = num_rates;
    } else {
        rates = tf4_det_rates;
        num_rates = ARRAY_SIZE(tf4_det_rates);
        wm8994->pdata->micd_rates = tf4_det_rates;
        wm8994->pdata->num_micd_rates = num_rates;
    }

    best = 0;
    for (i = 0; i < num_rates; i++) {
        if (rates[i].idle != idle)
            continue;
        if (abs(rates[i].sysclk - sysclk) <
            abs(rates[best].sysclk - sysclk))
            best = i;
        else if (rates[best].idle != idle)
            best = i;
    }

    val = (rates[best].start << WM8958_MICD_BIAS_STARTTIME_SHIFT)
        | (rates[best].rate << WM8958_MICD_RATE_SHIFT);

    snd_soc_update_bits(codec, WM8958_MIC_DETECT_1,
                WM8958_MICD_BIAS_STARTTIME_MASK |
                WM8958_MICD_RATE_MASK, val);
}

static void tf4_micdet(u16 status, void *data)
{
    struct wm1811_machine_priv *wm1811 = data;
    struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(wm1811->codec);
    int report;

    dev_dbg(wm1811->codec->dev, "MICDET %x\n", status);
    
    wake_lock_timeout(&wm1811->jackdet_wake_lock, 5 * HZ);

    /* Either nothing present or just starting detection */
    if (!(status & WM8958_MICD_STS)) {
        if (!wm8994->jackdet) {
            /* If nothing present then clear our statuses */
            dev_info(wm1811->codec->dev, "Detected open circuit\n");
            wm8994->jack_mic = false;
            wm8994->mic_detecting = true;

            tf4_micd_set_rate(wm1811->codec);

            snd_soc_jack_report(wm8994->micdet[0].jack, 0,
                        wm8994->btn_mask | SND_JACK_HEADSET);
        }
        return;
    }

    /* If the measurement is showing a high impedence we've got a
     * microphone.
     */
    if (wm8994->mic_detecting && (status & 0x400)) {
        dev_info(wm1811->codec->dev, "Detected microphone\n");

        wm8994->mic_detecting = false;
        wm8994->jack_mic = true;

        tf4_micd_set_rate(wm1811->codec);

        snd_soc_jack_report(wm8994->micdet[0].jack, SND_JACK_HEADSET,
                    SND_JACK_HEADSET);
    }

    if (wm8994->mic_detecting && (status & 0x4)) {
        dev_info(wm1811->codec->dev, "Detected headphone\n");
        wm8994->mic_detecting = false;

        tf4_micd_set_rate(wm1811->codec);

        snd_soc_jack_report(wm8994->micdet[0].jack, SND_JACK_HEADPHONE,
                    SND_JACK_HEADSET);

        /* If we have jackdet that will detect removal */
        if (wm8994->jackdet) {
            mutex_lock(&wm8994->accdet_lock);

            snd_soc_update_bits(wm1811->codec, WM8958_MIC_DETECT_1,
                        WM8958_MICD_ENA, 0);
            if (wm8994->active_refcount) {
                    snd_soc_update_bits(wm1811->codec,
                    WM8994_ANTIPOP_2,
                    WM1811_JACKDET_MODE_MASK,
                    WM1811_JACKDET_MODE_AUDIO);
            }

            mutex_unlock(&wm8994->accdet_lock);

            if (wm8994->pdata->jd_ext_cap) {
                mutex_lock(&wm1811->codec->mutex);
                snd_soc_dapm_disable_pin(&wm1811->codec->dapm,
                            "MICBIAS2");
                snd_soc_dapm_sync(&wm1811->codec->dapm);
                mutex_unlock(&wm1811->codec->mutex);
            }
        }
    }

    /* Report short circuit as a button */
    if (wm8994->jack_mic) {
        report = 0;
        if (status & WM1811_JACKDET_BTN0)
            report |= SND_JACK_BTN_0;

        if (status & WM1811_JACKDET_BTN1)
            report |= SND_JACK_BTN_1;

        if (status & WM1811_JACKDET_BTN2)
            report |= SND_JACK_BTN_2;

        dev_dbg(wm1811->codec->dev, "Detected Button: %08x (%08X)\n",
                        report, status);

        snd_soc_jack_report(wm8994->micdet[0].jack, report,
                    wm8994->btn_mask);
    }
}

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
static int tf4_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_out = 0;
	int ret = 0;

	/* AIF1CLK should be >=3MHz for optimal performance */
	if (params_format(params) == SNDRV_PCM_FORMAT_S24_LE)
		pll_out = params_rate(params) * 384;
	else if (params_rate(params) == 8000 || params_rate(params) == 11025)
		pll_out = params_rate(params) * 512;
	else
		pll_out = params_rate(params) * 256;

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

    /* Switch the FLL */
	ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
					TF4_WM1811_MCLK1_FREQ, pll_out);
	if (ret < 0) {
        printk(KERN_ERR "%s: Unable to start FLL1: %d\n", __func__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1,
					pll_out, SND_SOC_CLOCK_IN);
	if (ret < 0) {
        printk(KERN_ERR "%s: Unable to switch to FLL1: %d\n", __func__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
					0, MOD_OPCLK_PCLK);
	if (ret < 0)
		return ret;

	return 0;
}
#else /* CONFIG_SND_SAMSUNG_I2S_MASTER */
static int tf4_hw_params(struct snd_pcm_substream *substream,
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

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1,
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
#endif /* CONFIG_SND_SAMSUNG_I2S_MASTER */

/*
 * SMDK WM8994 DAI operations.
 */
static struct snd_soc_ops tf4_ops = {
	.hw_params = tf4_hw_params,
};
#ifdef CONFIG_SND_SOC_SAMSUNG_TF4_MODEM
static int tf4_hw_modem_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_out = 0;
	int ret = 0;

	/* AIF2 format is fixed to modem. */
	switch (params_format(params)) {
	default:
		pr_err("AIF2 format should be SNDRV_PCM_FORMAT_S16_LE.\n");
	case SNDRV_PCM_FORMAT_S16_LE:
		pll_out = params_rate(params) * 512;
		break;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A
					 | SND_SOC_DAIFMT_IB_NF
					 | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		return ret;
	}

	/* Switch the FLL */
	ret = snd_soc_dai_set_pll(codec_dai, WM8994_FLL2, WM8994_FLL_SRC_MCLK1,
					TF4_WM1811_MCLK1_FREQ, pll_out);
	if (ret < 0) {
        pr_err("%s: Unable to start FLL1: %d\n", __func__, ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL2,
					pll_out, SND_SOC_CLOCK_IN);
	if (ret < 0) {
        pr_err("%s: Unable to switch to FLL1: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops tf4_modem_ops = {
	.hw_params = tf4_hw_modem_params,
};
#endif

static int tf4_wm1811_init_paiftx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
    struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(codec);
    struct wm8994 *control = codec->control_data;
    struct wm1811_machine_priv *wm1811 = NULL;
    int ret = 0;

    tf4_snd_set_mclk(true);

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_MCLK1,
					TF4_WM1811_MCLK1_FREQ, SND_SOC_CLOCK_IN);
	if (ret < 0) {
        printk(KERN_ERR "%s: Failed to boot clocking: %d\n", __func__, ret);
		return ret;
	}

    wm1811 = kmalloc(sizeof *wm1811, GFP_KERNEL);
    if (!wm1811) {
        dev_err(codec->dev, "Failed to allocate memory!");
        return -ENOMEM;
    }

    wm1811->codec = codec;

	wm1811->jack.status = 0;

	ret = snd_soc_jack_new(codec, "TF4 Jack",
				SND_JACK_HEADSET,
				&wm1811->jack);
	if (ret < 0)
		dev_err(codec->dev, "Failed to create jack: %d\n", ret);

	/* To wakeup for earjack event in suspend mode */
    enable_irq_wake(control->irq);

    wake_lock_init(&wm1811->jackdet_wake_lock,
                WAKE_LOCK_SUSPEND, "tf4_jackdet");

	if (wm8994->revision > 1) {
        ret = wm8958_mic_detect(codec, &wm1811->jack, tf4_micdet, wm1811);
        if (ret < 0)
            dev_err(codec->dev, "Failed start detection: %d\n",	ret);
    } else {
        codec->dapm.idle_bias_off = 0;
	}

    /* Main Speaker */
    snd_soc_dapm_enable_pin(dapm, "SPKOUTLN");
    snd_soc_dapm_enable_pin(dapm, "SPKOUTLP");
    snd_soc_dapm_enable_pin(dapm, "SPKOUTRN");
    snd_soc_dapm_enable_pin(dapm, "SPKOUTRP");

    /* HeadPhone */
    snd_soc_dapm_enable_pin(dapm, "HPOUT1L");
    snd_soc_dapm_enable_pin(dapm, "HPOUT1R");

    /* SMT Mic */
    snd_soc_dapm_enable_pin(dapm, "IN1LN");
    snd_soc_dapm_enable_pin(dapm, "IN1LP");

    /* HEAD Mic */
    snd_soc_dapm_enable_pin(dapm, "IN1RN");
    snd_soc_dapm_enable_pin(dapm, "IN1RP");

    /* Other pins NC */
    snd_soc_dapm_nc_pin(dapm, "HPOUT2P");
    snd_soc_dapm_nc_pin(dapm, "HPOUT2N");
    snd_soc_dapm_nc_pin(dapm, "LINEOUT1N");
    snd_soc_dapm_nc_pin(dapm, "LINEOUT1P");
    snd_soc_dapm_nc_pin(dapm, "LINEOUT2N");
    snd_soc_dapm_nc_pin(dapm, "LINEOUT2P");
    snd_soc_dapm_nc_pin(dapm, "IN2LN");
    snd_soc_dapm_nc_pin(dapm, "IN2LP:VXRN");
    snd_soc_dapm_nc_pin(dapm, "IN2RN");
    snd_soc_dapm_nc_pin(dapm, "IN2RP:VXRP");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link tf4_dai[] = {
	{ /* Primary DAI i/f */
		.name = "WM8994 AIF1",
		.stream_name = "Pri_Dai",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8994-aif1",
		.platform_name = "samsung-audio",
		.codec_name = "wm8994-codec",
		.init = tf4_wm1811_init_paiftx,
		.ops = &tf4_ops,
	}, { /* Sec_Fifo DAI i/f */
		.name = "Sec_FIFO TX",
		.stream_name = "Sec_Dai",
		.cpu_dai_name = "samsung-i2s.4",
		.codec_dai_name = "wm8994-aif1",
#if defined (CONFIG_SND_SAMSUNG_LP) || defined(CONFIG_SND_SAMSUNG_ALP)
		.platform_name = "samsung-idma",
#else
		.platform_name = "samsung-audio",
#endif
		.codec_name = "wm8994-codec",
		.ops = &tf4_ops,
	},
#ifdef CONFIG_SND_SOC_SAMSUNG_TF4_MODEM
	{ // Modem DAI i/f
		.name = "WM8994 AIF2",
		.stream_name = "Modem_Dai",
		.cpu_dai_name = "samsung-pcm.1",
		.codec_dai_name = "wm8994-aif2",
		.platform_name = "samsung-audio",
		.codec_name = "wm8994-codec",
		.ops = &tf4_modem_ops,
	},
#endif
#if 0
	{ // Bt DAI i/f
		.name = "WM8994 AIF3",
		.stream_name = "Bluetooth_Dai",
		.cpu_dai_name = "samsung-i2s.0",
		.codec_dai_name = "wm8994-aif1",
		.platform_name = "samsung-audio",
		.codec_name = "wm8994-codec",
		.ops = &tf4_modem_ops,
	},
#endif
};

static int tf4_card_suspend_pre(struct snd_soc_card *card)
{
    return 0;
}

static int tf4_card_suspend_post(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec = card->rtd->codec;
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	struct snd_soc_dai *aif2_dai = card->rtd[1].codec_dai;
	int ret = 0;

	if (!codec->active) {
		tf4_snd_set_cpreg(true);

		ret = snd_soc_dai_set_sysclk(aif2_dai,
					     WM8994_SYSCLK_MCLK2,
					     TF4_WM1811_MCLK2_FREQ,
					     SND_SOC_CLOCK_IN);

		if (ret < 0)
			dev_err(codec->dev, "Unable to switch to MCLK2: %d\n",
				ret);

		ret = snd_soc_dai_set_pll(aif2_dai, WM8994_FLL2, 0, 0, 0);

		if (ret < 0)
			dev_err(codec->dev, "Unable to stop FLL2\n");

		ret = snd_soc_dai_set_sysclk(aif1_dai,
					     WM8994_SYSCLK_MCLK2,
					     TF4_WM1811_MCLK2_FREQ,
					     SND_SOC_CLOCK_IN);
		if (ret < 0)
			dev_err(codec->dev, "Unable to switch to MCLK2\n");

		ret = snd_soc_dai_set_pll(aif1_dai, WM8994_FLL1, 0, 0, 0);

		if (ret < 0)
			dev_err(codec->dev, "Unable to stop FLL1\n");

		tf4_snd_set_mclk(false);
	}

	exynos_xxti_sys_powerdown((tf4_snd_get_mclk())? 1: 0);

    return 0;
}

static int tf4_card_resume_pre(struct snd_soc_card *card)
{
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	int ret = 0;

	tf4_snd_set_mclk(true);

	/* Switch the FLL */
	ret = snd_soc_dai_set_pll(aif1_dai, WM8994_FLL1,
				  WM8994_FLL_SRC_MCLK1,
				  TF4_WM1811_MCLK1_FREQ,
				  TF4_WM1811_SYNC_CLK_FREQ);

	if (ret < 0)
		dev_err(aif1_dai->dev, "Unable to start FLL1: %d\n", ret);

	/* Then switch AIF1CLK to it */
	ret = snd_soc_dai_set_sysclk(aif1_dai,
				     WM8994_SYSCLK_FLL1,
				     TF4_WM1811_SYNC_CLK_FREQ,
				     SND_SOC_CLOCK_IN);

	if (ret < 0)
		dev_err(aif1_dai->dev, "Unable to switch to FLL1: %d\n", ret);

	tf4_snd_set_cpreg(false);

    return 0;
}

static int tf4_card_resume_post(struct snd_soc_card *card)
{
    return 0;
}

static struct snd_soc_card tf4 = {
	.name = "TF4-I2S",
	.dai_link = tf4_dai,

	/* If you want to use sec_fifo device,
	 * changes the num_link = 2 or ARRAY_SIZE(tf4_dai). */
	.num_links = ARRAY_SIZE(tf4_dai),

    .suspend_pre = tf4_card_suspend_pre,
    .suspend_post = tf4_card_suspend_post,
    .resume_pre = tf4_card_resume_pre,
    .resume_post = tf4_card_resume_post,
};

static struct platform_device *tf4_snd_device;

static int __init tf4_audio_init(void)
{
	int ret = 0;

	mclk = clk_get(NULL, "clkout");
	xxti_clk = clk_get(NULL, "xxti");
	clk_set_parent(mclk, xxti_clk);

	cpreg = regulator_get(NULL, "cp_32khz");
	if (IS_ERR(cpreg)) {
		pr_err("Sound: Getting EN32KHz CP regulator is failed\n");
	}

	tf4_snd_device = platform_device_alloc("soc-audio", 0);
	if (!tf4_snd_device)
		return -ENOMEM;

	platform_set_drvdata(tf4_snd_device, &tf4);

	ret = platform_device_add(tf4_snd_device);
	if (ret)
		platform_device_put(tf4_snd_device);

	return ret;
}
module_init(tf4_audio_init);

static void __exit tf4_audio_exit(void)
{
	platform_device_unregister(tf4_snd_device);

	regulator_put(cpreg);
	clk_put(mclk);
	clk_put(xxti_clk);
}
module_exit(tf4_audio_exit);

MODULE_DESCRIPTION("ALSA SoC TF4 WM1811");
MODULE_LICENSE("GPL");
