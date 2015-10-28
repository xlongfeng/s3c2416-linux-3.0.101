/*
 * smdk_wm9712.c  --  SoC audio for SMDK
 *
 * Copyright 2015 xlongfeng <xlongfeng@126.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <sound/soc.h>

static struct snd_soc_card smdk;

/*
 Playback (Mono):-
	$ amixer sset 'Mono' unmute
	$ amixer sset 'Phone Mixer PCM' unmute
	$ amixer sset 'PCM' 80%
*/

static struct snd_soc_dai_link smdk_dai = {
	.name = "AC97",
	.stream_name = "AC97 PCM",
	.platform_name = "samsung-audio",
	.cpu_dai_name = "samsung-ac97",
	.codec_dai_name = "wm9712-hifi",
	.codec_name = "wm9712-codec",
};

static struct snd_soc_card smdk = {
	.name = "SMDK WM9712",
	.dai_link = &smdk_dai,
	.num_links = 1,
};

static struct platform_device *smdk_snd_wm9712_device;
static struct platform_device *smdk_snd_ac97_device;

static int __init smdk_init(void)
{
	int ret;

	smdk_snd_wm9712_device = platform_device_alloc("wm9712-codec", -1);
	if (!smdk_snd_wm9712_device)
		return -ENOMEM;

	ret = platform_device_add(smdk_snd_wm9712_device);
	if (ret)
		goto err1;

	smdk_snd_ac97_device = platform_device_alloc("soc-audio", -1);
	if (!smdk_snd_ac97_device) {
		ret = -ENOMEM;
		goto err2;
	}

	platform_set_drvdata(smdk_snd_ac97_device, &smdk);

	ret = platform_device_add(smdk_snd_ac97_device);
	if (ret)
		goto err3;

	return 0;

err3:
	platform_device_put(smdk_snd_ac97_device);
err2:
	platform_device_del(smdk_snd_wm9712_device);
err1:
	platform_device_put(smdk_snd_wm9712_device);
	return ret;
}

static void __exit smdk_exit(void)
{
	platform_device_unregister(smdk_snd_ac97_device);
	platform_device_unregister(smdk_snd_wm9712_device);
}

module_init(smdk_init);
module_exit(smdk_exit);

/* Module information */
MODULE_AUTHOR("xlongfeng <xlongfeng@126.com>");
MODULE_DESCRIPTION("ALSA SoC SMDK+WM9712");
MODULE_LICENSE("GPL");
