/*
 * GPIO vibration motor exposed through the timed_output class, the interface
 * the Samsung vibrator HAL (vendor.samsung.hardware.vibrator@2.0) expects.
 *
 * Based on the Samsung/Linux Foundation msm_vibrator driver (GPL v2); only the
 * "motor-en" GPIO variant used by the Galaxy A6+ is kept.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include "../staging/android/timed_output.h"

#define VIB_DEFAULT_TIMEOUT	10000

struct msm_vib {
	struct hrtimer timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
	struct mutex lock;
	int motor_en;
	int state;
};

static void msm_vib_update(struct work_struct *work)
{
	struct msm_vib *vib = container_of(work, struct msm_vib, work);

	gpio_set_value_cansleep(vib->motor_en, vib->state ? 1 : 0);
}

static void msm_vib_enable(struct timed_output_dev *dev, int value)
{
	struct msm_vib *vib = container_of(dev, struct msm_vib, timed_dev);

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->timer);

	if (value) {
		if (value != 0x7fffffff) {
			if (value > VIB_DEFAULT_TIMEOUT)
				value = VIB_DEFAULT_TIMEOUT;
			hrtimer_start(&vib->timer,
				      ms_to_ktime(value), HRTIMER_MODE_REL);
		}
	}
	vib->state = value;
	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);
}

static int msm_vib_get_time(struct timed_output_dev *dev)
{
	struct msm_vib *vib = container_of(dev, struct msm_vib, timed_dev);

	if (hrtimer_active(&vib->timer))
		return ktime_to_ms(hrtimer_get_remaining(&vib->timer));

	return 0;
}

static enum hrtimer_restart msm_vib_timer(struct hrtimer *timer)
{
	struct msm_vib *vib = container_of(timer, struct msm_vib, timer);

	vib->state = 0;
	schedule_work(&vib->work);

	return HRTIMER_NORESTART;
}

static int msm_vibrator_probe(struct platform_device *pdev)
{
	struct msm_vib *vib;
	int rc;

	vib = devm_kzalloc(&pdev->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->motor_en = of_get_named_gpio(pdev->dev.of_node, "motor-en", 0);
	if (!gpio_is_valid(vib->motor_en)) {
		dev_err(&pdev->dev, "motor-en gpio not specified\n");
		return -EINVAL;
	}

	rc = devm_gpio_request_one(&pdev->dev, vib->motor_en,
				   GPIOF_OUT_INIT_LOW, "motor_en");
	if (rc) {
		dev_err(&pdev->dev, "cannot request gpio %d: %d\n",
			vib->motor_en, rc);
		return rc;
	}

	mutex_init(&vib->lock);
	INIT_WORK(&vib->work, msm_vib_update);
	hrtimer_init(&vib->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->timer.function = msm_vib_timer;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = msm_vib_get_time;
	vib->timed_dev.enable = msm_vib_enable;

	platform_set_drvdata(pdev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0)
		dev_err(&pdev->dev, "timed_output register failed: %d\n", rc);

	return rc;
}

static int msm_vibrator_remove(struct platform_device *pdev)
{
	struct msm_vib *vib = platform_get_drvdata(pdev);

	timed_output_dev_unregister(&vib->timed_dev);
	cancel_work_sync(&vib->work);

	return 0;
}

static int msm_vibrator_suspend(struct device *dev)
{
	struct msm_vib *vib = dev_get_drvdata(dev);

	hrtimer_cancel(&vib->timer);
	cancel_work_sync(&vib->work);
	gpio_set_value_cansleep(vib->motor_en, 0);

	return 0;
}

static SIMPLE_DEV_PM_OPS(msm_vibrator_pm_ops, msm_vibrator_suspend, NULL);

static const struct of_device_id msm_vibrator_match[] = {
	{ .compatible = "vibrator", },
	{}
};

static struct platform_driver msm_vibrator_driver = {
	.driver = {
		.name = "msm_vibrator",
		.of_match_table = msm_vibrator_match,
		.pm = &msm_vibrator_pm_ops,
	},
	.probe = msm_vibrator_probe,
	.remove = msm_vibrator_remove,
};
module_platform_driver(msm_vibrator_driver);

MODULE_DESCRIPTION("timed output vibrator device");
MODULE_LICENSE("GPL v2");
