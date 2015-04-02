/*
 *  exynos_wm5102.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include "../codecs/wm5102.h"
#include <sound/pcm_params.h>
#include <linux/module.h>


#define  EXYNOS_WM5102_FREQ 24000000

static int exynos_aif1_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_out = EXYNOS_WM5102_FREQ;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFS);
	if (ret < 0)
		return ret;

	pll_out = params_rate(params) * 512;

	if (pll_out <= 0) {
		dev_err(aif1_dai->dev, "Failed to set AIF1 FLL OUT: %d\n", pll_out);
		return -EINVAL;
	}

	/* set the codec FLL */
	ret = snd_soc_dai_set_pll(codec_dai, WM5102_FLL1, ARIZONA_FLL_SRC_MCLK1, EXYNOS_WM5102_FREQ,
				pll_out);
	if (ret != 0) {
		dev_err(aif1_dai->dev, "Failed to set AIF1 FLL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_SYSCLK, pll_out, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(aif1_dai->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	return 0;
}

static int exynos_aif2_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_out = EXYNOS_WM5102_FREQ;
	int ret;

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFS);
	if (ret < 0)
		return ret;

	pll_out = params_rate(params) * 512;

	if (pll_out <= 0) {
		dev_err(aif1_dai->dev, "Failed to set AIF1 FLL OUT: %d\n", pll_out);
		return -EINVAL;
	}

	/* set the codec FLL */
	ret = snd_soc_dai_set_pll(codec_dai, WM5102_FLL2, ARIZONA_FLL_SRC_MCLK2, EXYNOS_WM5102_FREQ,
				pll_out);
	if (ret != 0) {
		dev_err(aif1_dai->dev, "Failed to set AIF1 FLL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, ARIZONA_CLK_SYSCLK, pll_out, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(aif1_dai->dev, "Failed to set AIF1 clock: %d\n", ret);
		return ret;
	}

	return 0;
}

/*
 * EXYNOS wm5102 DAI operations.
 */
static struct snd_soc_ops exynos_aif1_hw_ops = {
	.hw_params = exynos_aif1_hw_params,
};
static struct snd_soc_ops exynos_aif2_hw_ops = {
	.hw_params = exynos_aif2_hw_params,
};
/*
 * Set NC pin
 */
static int exynos_wm5102_init_paiftx(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* HeadPhone */
	snd_soc_dapm_enable_pin(dapm, "HPOUT1R");
	snd_soc_dapm_enable_pin(dapm, "HPOUT1L");

	/* MicIn */
	snd_soc_dapm_enable_pin(dapm, "IN1LN");
	snd_soc_dapm_enable_pin(dapm, "IN1RN");

	/* LineIn */
	snd_soc_dapm_enable_pin(dapm, "IN2LN");
	snd_soc_dapm_enable_pin(dapm, "IN2RN");

	/* Other pins NC */
	snd_soc_dapm_nc_pin(dapm, "HPOUT2P");
	snd_soc_dapm_nc_pin(dapm, "HPOUT2N");


	return 0;
}
static struct snd_soc_dai_link exynos_dai_wm5102[] = {
	{
		.name = "CPU",
		.stream_name = "CPU",
		.cpu_dai_name = "exynos-i2s.0",
		.codec_dai_name = "wm5102-aif1",
		.platform_name = "exynos-audio",
		.codec_name = "wm5102-codec",
		.ops = &exynos_aif1_hw_ops,
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		//		| SND_SOC_DAIFMT_CBM_CFM,
	},
	{
		.name = "Baseband",
		.stream_name = "Baseband",
		.cpu_dai_name = "exynos-i2s.1",
		.codec_dai_name = "wm5102-aif2",
		.codec_name = "wm5102-codec",
		//.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		//		| SND_SOC_DAIFMT_CBM_CFM,
		//.ignore_suspend = 1,
		//.params = &baseband_params,
		.ops = &exynos_aif2_hw_ops,
	},
};

static struct snd_soc_card exynos_cards[] = {
	{
		.name = "Exynos WM5102",
		.owner = THIS_MODULE,
		.dai_link = exynos_dai_wm5102,
		.num_links = ARRAY_SIZE(exynos_dai_wm5102),
		//.codec_conf = exynos_codec_conf,
		//.num_configs = ARRAY_SIZE(exynos_codec_conf),

		//.late_probe = exynos_late_probe,

		//.dapm_routes = exynos_routes,
		//.num_dapm_routes = ARRAY_SIZE(exynos_routes),

		//.set_bias_level = exynos_set_bias_level,
		//.set_bias_level_post = exynos_set_bias_level_post,
	},
};

static int __devinit exynos_audio_probe(struct platform_device *pdev)
{
	int ret;
	struct snd_soc_card *card = &exynos_cards;

	card->dev = &pdev->dev;
	ret = snd_soc_register_card(card);

	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed:%d\n", ret);

	return ret;
}

static int __devexit exynos_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver exynos_audio_driver = {
	.driver		= {
		.name	= "exynos-audio",
		.owner	= THIS_MODULE,
		.pm = &snd_soc_pm_ops,
	},
	.probe		= exynos_audio_probe,
	.remove		= __devexit_p(exynos_audio_remove),
};

module_platform_driver(exynos_audio_driver);

MODULE_DESCRIPTION("ALSA SoC EXYNOS wm5102");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-audio");
